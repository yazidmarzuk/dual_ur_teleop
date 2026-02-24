#!/usr/bin/env python3
import os
import math
import cv2
import numpy as np
import mediapipe as mp
import mediapipe.python.solutions.drawing_styles as drawing_styles
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D, TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class HandTrackingNode(Node):
    def __init__(self):
        super().__init__("hand_tracking_node")

        self.left_gripper_state = "OPEN"
        self.right_gripper_state = "OPEN"
        self.pinched_threshold = 0.1
        self.closed_pos = 0.0
        self.open_pos = -0.03
        self.move_time = 1.0
        self.alpha = 0.5

        self.publisher_left = self.create_publisher(Pose2D, "/hand_tracking/panda1/pose2d", 10)
        self.publisher_right = self.create_publisher(Pose2D, "/hand_tracking/panda2/pose2d", 10)
        self.left_pose_pub = self.create_publisher(PoseStamped, "/left_teleop_target_pose", 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, "/right_teleop_target_pose", 10)

        self.pub_twist_left = self.create_publisher(
            TwistStamped, "/servo_node_left/delta_twist_cmds", 10
        )
        self.pub_twist_right = self.create_publisher(
            TwistStamped, "/servo_node_right/delta_twist_cmds", 10
        )
        self.left_gripper_pub = self.create_publisher(
            JointTrajectory, "/left_gripper_controller/joint_trajectory", 10
        )
        self.right_gripper_pub = self.create_publisher(
            JointTrajectory, "/right_gripper_controller/joint_trajectory", 10
        )

        use_direct = os.environ.get("USE_DIRECT_CAMERA", "").strip().lower() in ("1", "true", "yes") or os.environ.get("DISPLAY")
        if use_direct:
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cv2.VideoCapture(os.environ.get("VIDEO_SOURCE", "http://192.168.64.1:5000/video_feed"))

        self.mp_hands = mp.solutions.hands
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=2,
            model_complexity=0,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.timer = self.create_timer(1.0 / 60.0, self.process_frame)
        self.prev_left = np.array([0.0, 0.0])
        self.prev_right = np.array([0.0, 0.0])

        self.MAX_LINEAR_VEL = 0.3
        self.MAX_ANGULAR_VEL = 0.15
        self.left_prev_time = None
        self.left_prev_pos = None
        self.left_prev_euler = None
        self.right_prev_time = None
        self.right_prev_pos = None
        self.right_prev_euler = None
        self._twist_logged = {"left": False, "right": False}

        self.twist_frame_left = "left_base_link"
        self.twist_frame_right = "right_base_link"

    def lowpass(self, current: np.ndarray, previous: np.ndarray) -> np.ndarray:
        return self.alpha * current + (1 - self.alpha) * previous

    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return np.array([X, Y, Z])

    def send_gripper_traj(self, publisher, joint_names: list, position: float):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [position] * len(joint_names)
        pt.time_from_start = Duration(sec=int(self.move_time), nanosec=int((self.move_time % 1) * 1e9))
        traj.points = [pt]
        publisher.publish(traj)

    def grip_gesture(self, tip_distance):
        return "PINCH" if tip_distance < self.pinched_threshold else "OPEN"

    def _publish_twist_from_pose_delta(self, pub, frame_id, pos, euler, prev_time, prev_pos, prev_euler):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if prev_time is None:
            return current_time, pos.copy(), euler.copy()

        dt = current_time - prev_time
        if dt <= 0:
            return prev_time, prev_pos, prev_euler

        vx = (pos[0] - prev_pos[0]) / dt
        vy = (pos[1] - prev_pos[1]) / dt
        vz = (pos[2] - prev_pos[2]) / dt
        yaw_rate = (euler[2] - prev_euler[2]) / dt

        vx = max(min(vx, self.MAX_LINEAR_VEL), -self.MAX_LINEAR_VEL)
        vy = max(min(vy, self.MAX_LINEAR_VEL), -self.MAX_LINEAR_VEL)
        yaw_rate = max(min(yaw_rate, self.MAX_ANGULAR_VEL), -self.MAX_ANGULAR_VEL)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = frame_id
        twist.twist.linear.x = float(vx)
        twist.twist.linear.y = float(vy)
        twist.twist.linear.z = float(vz)
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = float(yaw_rate)
        pub.publish(twist)

        side = "left" if "left" in frame_id else "right"
        if not self._twist_logged.get(side, True):
            self._twist_logged[side] = True
            self.get_logger().info(
                f"Twist publishing to Servo (frame_id={frame_id}). Move hand to see motion."
            )

        return current_time, pos.copy(), euler.copy()

    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        frame = cv2.resize(frame, (640, 480))
        frame = cv2.flip(frame, 1)

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(
                results.multi_hand_landmarks, results.multi_handedness
            ):
                label = handedness.classification[0].label
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                index_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

                tip_distance = math.sqrt(
                    (index_finger_tip.x - thumb_tip.x) ** 2 + (index_finger_tip.y - thumb_tip.y) ** 2
                )

                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    drawing_styles.get_default_hand_landmarks_style(),
                    drawing_styles.get_default_hand_connections_style(),
                )

                msg = Pose2D()
                msg.x = float(index_finger_tip.x)
                msg.y = float(index_finger_tip.y)
                raw = np.array([index_finger_tip.x, index_finger_tip.y])

                if label.lower() == "left":
                    self.left_gripper_state = self.grip_gesture(tip_distance)
                    self.publisher_left.publish(msg)

                    filtered = self.lowpass(raw, self.prev_left)
                    self.prev_left = filtered
                    x, y = filtered[0], filtered[1]
                    theta = math.atan2(y, x)

                    pose_l = PoseStamped()
                    pose_l.header.stamp = self.get_clock().now().to_msg()
                    pose_l.header.frame_id = self.twist_frame_left
                    pose_l.pose.position.x = x * 2
                    pose_l.pose.position.y = y * 2
                    pose_l.pose.position.z = 0.4
                    q = self.euler_to_quaternion(theta, 0, 0)
                    pose_l.pose.orientation.x = q[0]
                    pose_l.pose.orientation.y = q[1]
                    pose_l.pose.orientation.z = q[2]
                    pose_l.pose.orientation.w = q[3]
                    self.left_pose_pub.publish(pose_l)

                    pos = np.array([pose_l.pose.position.x, pose_l.pose.position.y, pose_l.pose.position.z])
                    euler = np.array([0.0, 0.0, theta])
                    self.left_prev_time, self.left_prev_pos, self.left_prev_euler = self._publish_twist_from_pose_delta(
                        self.pub_twist_left, self.twist_frame_left,
                        pos, euler,
                        self.left_prev_time, self.left_prev_pos, self.left_prev_euler,
                    )
                    if self.left_gripper_state == "PINCH":
                        self.send_gripper_traj(
                            self.left_gripper_pub,
                            ["left_left_finger_joint", "left_right_finger_joint"],
                            self.closed_pos,
                        )
                    else:
                        self.send_gripper_traj(
                            self.left_gripper_pub,
                            ["left_left_finger_joint", "left_right_finger_joint"],
                            self.open_pos,
                        )

                else:
                    self.right_gripper_state = self.grip_gesture(tip_distance)
                    self.publisher_right.publish(msg)

                    filtered = self.lowpass(raw, self.prev_right)
                    self.prev_right = filtered
                    x, y = filtered[0], filtered[1]
                    theta = math.atan2(y, x)

                    pose_r = PoseStamped()
                    pose_r.header.stamp = self.get_clock().now().to_msg()
                    pose_r.header.frame_id = self.twist_frame_right
                    pose_r.pose.position.x = x * 2
                    pose_r.pose.position.y = y * 2
                    pose_r.pose.position.z = 0.4
                    q = self.euler_to_quaternion(theta, 0, 0)
                    pose_r.pose.orientation.x = q[0]
                    pose_r.pose.orientation.y = q[1]
                    pose_r.pose.orientation.z = q[2]
                    pose_r.pose.orientation.w = q[3]
                    self.right_pose_pub.publish(pose_r)

                    pos = np.array([pose_r.pose.position.x, pose_r.pose.position.y, pose_r.pose.position.z])
                    euler = np.array([0.0, 0.0, theta])
                    self.right_prev_time, self.right_prev_pos, self.right_prev_euler = self._publish_twist_from_pose_delta(
                        self.pub_twist_right, self.twist_frame_right,
                        pos, euler,
                        self.right_prev_time, self.right_prev_pos, self.right_prev_euler,
                    )
                    if self.right_gripper_state == "PINCH":
                        self.send_gripper_traj(
                            self.right_gripper_pub,
                            ["right_left_finger_joint", "right_right_finger_joint"],
                            self.closed_pos,
                        )
                    else:
                        self.send_gripper_traj(
                            self.right_gripper_pub,
                            ["right_left_finger_joint", "right_right_finger_joint"],
                            self.open_pos,
                        )

        if os.environ.get("DISPLAY"):
            cv2.imshow("Hand Tracking", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
