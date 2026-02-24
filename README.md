# Dual UR5e Arm Teleoperation with Hand Tracking

Teleoperate two UR5e arms in Cartesian space using a webcam and MediaPipe hand tracking. Each hand controls one arm. Pinching closes the gripper.

---

## Setup

Two UR5e arms placed 0.8 m apart on the X-axis, each with a parallel-jaw gripper. Only X and Y motion is used since depth is not available from a single 2D camera.

---

## Requirements

### ROS 2

ROS 2 Humble with MoveIt 2.

```bash
sudo apt update && sudo apt install \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-ur-description \
  ros-humble-robot-state-publisher
```

### Hand Tracking

```bash
pip install mediapipe opencv-python
```

---

## Building

```bash
git clone <repo-url> catkin_ws
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Camera Setup

### No direct camera access (network stream)

Run a Flask stream server on the machine with the camera:

```bash
python3 src/teleop_hand_tracking/scripts/flask_bringup.py
```

Then set the stream URL before launching:

```bash
export VIDEO_SOURCE=http://<ip>:5000/video_feed
```

### Direct camera access

```bash
export USE_DIRECT_CAMERA=1
```

To preview the feed without a display (e.g. inside Docker), write frames to a file:

```bash
python3 src/teleop_hand_tracking/scripts/camera_bringup.py
```

---

## Running

```bash
ros2 launch teleop_hand_tracking teleop.launch.py
```

Without RViz:

```bash
ros2 launch teleop_hand_tracking teleop.launch.py rviz:=false
```

Wait about 5 seconds for MoveIt Servo to activate, then move your hands in front of the camera.

---

## Nodes

| Node | Description |
|------|-------------|
| `hand_tracker_node` | Reads camera, detects hands via MediaPipe, publishes TwistStamped to each servo node and JointTrajectory to each gripper controller |
| `servo_node_left` / `servo_node_right` | MoveIt Servo — converts Twist commands to JointTrajectory |
| `servo_to_joint_state` | Merges servo and gripper outputs into `/joint_states` |
| `robot_state_publisher` | Computes TF tree from `/joint_states` |
| `rviz2` | Visualisation |

---

## Packages

- `dual_arm_description` — URDF/xacro for dual UR5e with parallel-jaw grippers
- `dual_arm_moveit_config` — SRDF, kinematics, servo configs, launch files
- `teleop_hand_tracking` — Hand tracking and joint state relay nodes

---

## Dependencies

- [MediaPipe](https://mediapipe.dev)
- [OpenCV](https://opencv.org)
- [MoveIt 2](https://moveit.ros.org)
- [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

---

## Limitations

- Z-axis motion is fixed (no depth from 2D camera)
- Singularities can occur depending on workspace coverage
