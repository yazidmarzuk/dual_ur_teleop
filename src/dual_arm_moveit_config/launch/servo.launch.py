from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_moveit_share = get_package_share_directory('dual_arm_moveit_config')
    pkg_desc_share   = get_package_share_directory('dual_arm_description')

    robot_description = {
        'robot_description': Command([
            'xacro ',
            os.path.join(pkg_desc_share, 'urdf', 'dual_arm.urdf.xacro')
        ])
    }

    with open(os.path.join(pkg_moveit_share, 'config', 'dual_arm.srdf'), 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    with open(os.path.join(pkg_moveit_share, 'config', 'kinematics.yaml'), 'r') as f:
        kinematics = {'robot_description_kinematics': yaml.safe_load(f)}

    with open(os.path.join(pkg_moveit_share, 'config', 'servo_left.yaml'), 'r') as f:
        left_raw = yaml.safe_load(f)
    with open(os.path.join(pkg_moveit_share, 'config', 'servo_right.yaml'), 'r') as f:
        right_raw = yaml.safe_load(f)
    left_params = left_raw.get('/**', {}).get('ros__parameters', left_raw)
    right_params = right_raw.get('/**', {}).get('ros__parameters', right_raw)

    servo_left = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node_left',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            {'moveit_servo': left_params},
        ],
        output='screen',
    )

    servo_right = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node_right',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            {'moveit_servo': right_params},
        ],
        output='screen',
    )

    return LaunchDescription([servo_left, servo_right])