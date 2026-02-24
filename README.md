# HandServo

Dual UR5e arm teleoperation using hand tracking via MediaPipe and MoveIt Servo.

Each hand controls one arm in Cartesian space. Pinching closes the gripper. Only X and Y motion is used since depth is unavailable from a single 2D camera.

---

## Requirements

ROS 2 Humble with MoveIt 2.

```bash
sudo apt update && sudo apt install \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-ur-description \
  ros-humble-robot-state-publisher
```

```bash
pip install mediapipe opencv-python
```

---

## Build

```bash
git clone <repo-url> catkin_ws
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Use a direct webcam
export USE_DIRECT_CAMERA=1

# Or stream over the network
export VIDEO_SOURCE=http://<ip>:5000/video_feed

ros2 launch teleop_hand_tracking teleop.launch.py
```

Without RViz:

```bash
ros2 launch teleop_hand_tracking teleop.launch.py rviz:=false
```

Wait ~5 seconds for MoveIt Servo to activate, then move your hands in front of the camera.

---

## Nodes

| Node | Description |
|------|-------------|
| `hand_tracker_node` | Detects hands via MediaPipe, publishes velocity commands to each servo node and gripper controller |
| `servo_node_left` / `servo_node_right` | MoveIt Servo — converts Twist commands to joint trajectories |
| `servo_to_joint_state` | Merges servo and gripper outputs into `/joint_states` |
| `robot_state_publisher` | Computes TF tree from `/joint_states` |

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
