#!/usr/bin/env python3
"""
Subscribes to Servo joint commands and gripper trajectories, merges them,
and publishes /joint_states so the robot moves in RViz when using hand teleop.
Run this with RViz + Servo + hand_tracker_node to see the arms follow your hands.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# Full joint list and home positions (from dual_arm SRDF + gripper open)
ALL_JOINT_NAMES = [
    "left_shoulder_pan_joint",
    "left_shoulder_lift_joint",
    "left_elbow_joint",
    "left_wrist_1_joint",
    "left_wrist_2_joint",
    "left_wrist_3_joint",
    "right_shoulder_pan_joint",
    "right_shoulder_lift_joint",
    "right_elbow_joint",
    "right_wrist_1_joint",
    "right_wrist_2_joint",
    "right_wrist_3_joint",
    "left_left_finger_joint",
    "left_right_finger_joint",
    "right_left_finger_joint",
    "right_right_finger_joint",
]
HOME_POSITIONS = [
    1.5707,
    -1.5707,
    1.5707,
    -1.5707,
    -1.5707,
    0.0,
    -1.5707,
    -1.5707,
    -1.5707,
    -1.5707,
    1.5707,
    0.0,
    -0.03,
    -0.03,
    -0.03,
    -0.03,
]


class ServoToJointStateNode(Node):
    def __init__(self):
        super().__init__("servo_to_joint_state")
        self.positions = list(HOME_POSITIONS)
        self.name_to_index = {n: i for i, n in enumerate(ALL_JOINT_NAMES)}
        self.initialized_from_states = False

        self.sub_left_cmd = self.create_subscription(
            JointTrajectory,
            "/servo_node_left/command",
            self._cb_left_command,
            10,
        )
        self.sub_right_cmd = self.create_subscription(
            JointTrajectory,
            "/servo_node_right/command",
            self._cb_right_command,
            10,
        )
        self.sub_left_gripper = self.create_subscription(
            JointTrajectory,
            "/left_gripper_controller/joint_trajectory",
            self._cb_left_gripper,
            10,
        )
        self.sub_right_gripper = self.create_subscription(
            JointTrajectory,
            "/right_gripper_controller/joint_trajectory",
            self._cb_right_gripper,
            10,
        )
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self._cb_joint_states,
            10,
        )

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.02, self._publish)  # 50 Hz

        self.get_logger().info(
            "Servo->joint_states relay running. Hand teleop will move the robot in RViz."
        )

    def _apply_trajectory(self, msg: JointTrajectory):
        if not msg.points or not msg.joint_names:
            return
        last = msg.points[-1]
        if len(last.positions) != len(msg.joint_names):
            return
        for i, name in enumerate(msg.joint_names):
            idx = self.name_to_index.get(name)
            if idx is not None:
                self.positions[idx] = last.positions[i]

    def _cb_left_command(self, msg: JointTrajectory):
        self._apply_trajectory(msg)

    def _cb_right_command(self, msg: JointTrajectory):
        self._apply_trajectory(msg)

    def _cb_left_gripper(self, msg: JointTrajectory):
        self._apply_trajectory(msg)

    def _cb_right_gripper(self, msg: JointTrajectory):
        self._apply_trajectory(msg)

    def _cb_joint_states(self, msg: JointState):
        if self.initialized_from_states or not msg.name or not msg.position:
            return
        for i, name in enumerate(msg.name):
            idx = self.name_to_index.get(name)
            if idx is not None and i < len(msg.position):
                self.positions[idx] = msg.position[i]
        self.initialized_from_states = True

    def _publish(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = ""
        out.name = ALL_JOINT_NAMES
        out.position = self.positions
        out.velocity = [0.0] * len(ALL_JOINT_NAMES)
        out.effort = []
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ServoToJointStateNode()
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
