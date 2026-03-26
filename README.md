# Dual UR Teleoperation via Hand Tracking

A **ROS2** workspace for teleoperation of **two UR robot arms simultaneously** using real-time hand pose estimation from a camera (MediaPipe-based hand tracking). No glove hardware required — just a webcam. Uses **MoveIt2 Servo** for real-time Cartesian velocity streaming to both arms.

## System Overview

```
Camera (USB/Depth)
      │
      ▼
hand_tracker_node (MediaPipe hand landmarks)
      │  publishes: /left_hand_pose, /right_hand_pose (geometry_msgs/PoseStamped)
      ▼
servo_to_joint_state_node (maps hand pose → joint velocity commands)
      │  uses: MoveIt2 Servo
      │  topics: /servo_left/delta_twist_cmds, /servo_right/delta_twist_cmds
      ▼
joint_state_relay (timestamps + relays /joint_states → /dual_arm/joint_states)
      │
      ▼
dual_arm_moveit_config (SRDF: left_arm + right_arm planning groups)
      │
      ▼
Left UR + Right UR (real hardware or simulation)
```

## Package Structure

```
src/
├── teleop_hand_tracking/
│   ├── teleop_hand_tracking/
│   │   ├── hand_tracker_node.py        # MediaPipe hand detection + pose estimation
│   │   ├── servo_to_joint_state_node.py # Hand pose → MoveIt Servo commands
│   │   └── joint_state_relay_node.py   # /joint_states relay with timestamp fix
│   └── launch/
│       └── teleop.launch.py            # Full system launch
└── dual_arm_moveit_config/
    ├── config/
    │   ├── dual_arm.srdf               # Planning groups: left_arm, right_arm
    │   ├── kinematics.yaml
    │   ├── servo_left.yaml             # Servo config for left arm
    │   └── servo_right.yaml            # Servo config for right arm
    └── launch/
        ├── servo.launch.py
        └── dual_arm_rviz.launch.py
```

## Prerequisites

- ROS2 (Humble recommended)
- MoveIt2 with Servo support
- `mediapipe` Python package
- UR ROS2 driver
- OpenCV

```bash
pip install mediapipe opencv-python
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
# Launch full teleop system
ros2 launch teleop_hand_tracking teleop.launch.py

# Or launch components separately:
ros2 run teleop_hand_tracking hand_tracker_node
ros2 run teleop_hand_tracking servo_to_joint_state_node
ros2 run teleop_hand_tracking joint_state_relay
```

For visualization:
```bash
ros2 launch dual_arm_moveit_config dual_arm_rviz.launch.py
```

## Key Design Notes

**Joint State Relay**: The `joint_state_relay_node` exists to fix a common issue with fake/simulated UR controllers that publish `JointState` messages with zero timestamps. It stamps them with wall clock time before forwarding to `/dual_arm/joint_states`, which MoveIt2 Servo requires for proper operation.

**Dual Servo Configuration**: Two separate MoveIt Servo instances are configured (`servo_left.yaml`, `servo_right.yaml`) with independent planning group IDs so both arms can be commanded simultaneously without planning group conflicts.

## Git History Summary

| Commit | Description |
|---|---|
| `2bc40fa`–`0458e19` | Full dual-arm camera-based hand tracking teleop implementation |