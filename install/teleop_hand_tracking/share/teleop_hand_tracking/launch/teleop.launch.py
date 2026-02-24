"""
Hand-teleop launch for dual UR5e arms.

Starts everything needed to hand-teleoperate the robots in RViz:
  - robot_state_publisher  (TF tree from /joint_states)
  - servo_node_left        (MoveIt Servo for left arm)
  - servo_node_right       (MoveIt Servo for right arm)
  - hand_tracker_node      (MediaPipe hand tracking → twist commands)
  - servo_to_joint_state   (merges servo outputs → /joint_states)
  - rviz2                  (optional, disable with rviz:=false)

Usage:
  ros2 launch teleop_hand_tracking teleop.launch.py
  ros2 launch teleop_hand_tracking teleop.launch.py rviz:=false
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_moveit = get_package_share_directory('dual_arm_moveit_config')
    pkg_desc   = get_package_share_directory('dual_arm_description')

    # ── Robot description (URDF, SRDF, kinematics) ──────────────────────────
    robot_description = {
        'robot_description': Command([
            'xacro ',
            os.path.join(pkg_desc, 'urdf', 'dual_arm.urdf.xacro'),
        ])
    }

    with open(os.path.join(pkg_moveit, 'config', 'dual_arm.srdf'), 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    with open(os.path.join(pkg_moveit, 'config', 'kinematics.yaml'), 'r') as f:
        kinematics = {'robot_description_kinematics': yaml.safe_load(f)}

    # ── Servo parameters ─────────────────────────────────────────────────────
    def load_servo_params(filename):
        with open(os.path.join(pkg_moveit, 'config', filename), 'r') as f:
            raw = yaml.safe_load(f)
        # Strip the /**:/ros__parameters: wrapper so we get a flat dict
        params = raw.get('/**', {}).get('ros__parameters', raw)
        return {'moveit_servo': params}

    left_servo_params  = load_servo_params('servo_left.yaml')
    right_servo_params = load_servo_params('servo_right.yaml')

    # ── RViz config (optional) ───────────────────────────────────────────────
    rviz_config = os.path.join(pkg_moveit, 'config', 'dual_arm.rviz')

    # ── Nodes ────────────────────────────────────────────────────────────────

    # 1. Robot state publisher — computes TF tree from /joint_states
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2. MoveIt Servo — left arm (publishes /servo_node_left/command)
    servo_left_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node_left',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            left_servo_params,
        ],
    )

    # 3. MoveIt Servo — right arm (publishes /servo_node_right/command)
    servo_right_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node_right',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            right_servo_params,
        ],
    )

    # 4. Hand tracker — MediaPipe detects hands, publishes TwistStamped to servo nodes
    hand_tracker_node = Node(
        package='teleop_hand_tracking',
        executable='hand_tracker_node',
        name='hand_tracking_node',
        output='screen',
    )

    # 5. Servo → joint_states aggregator — merges servo outputs + gripper → /joint_states
    servo_to_js_node = Node(
        package='teleop_hand_tracking',
        executable='servo_to_joint_state_node',
        name='servo_to_joint_state',
        output='screen',
    )

    # 6. RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[robot_description, robot_description_semantic],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # MoveIt Servo starts in stopped state — call start_servo for both arms after 5s
    start_servo_left = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call',
                 '/servo_node_left/start_servo',
                 'std_srvs/srv/Trigger', '{}'],
            output='screen',
        )],
    )
    start_servo_right = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call',
                 '/servo_node_right/start_servo',
                 'std_srvs/srv/Trigger', '{}'],
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),
        rsp_node,
        servo_left_node,
        servo_right_node,
        hand_tracker_node,
        servo_to_js_node,
        rviz_node,
        start_servo_left,
        start_servo_right,
    ])
