from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_teleop_relay = LaunchConfiguration('use_teleop_relay', default='false')

    pkg_desc = FindPackageShare('dual_arm_description')
    pkg_moveit = FindPackageShare('dual_arm_moveit_config')

    urdf_xacro = PathJoinSubstitution([pkg_desc, 'urdf', 'dual_arm.urdf.xacro'])
    srdf_path  = PathJoinSubstitution([pkg_moveit, 'config', 'dual_arm.srdf'])

    robot_description = {'robot_description': Command(['xacro ', urdf_xacro])}

    with open(os.path.join(FindPackageShare('dual_arm_moveit_config').find('dual_arm_moveit_config'), 
                           'config', 'dual_arm.srdf'), 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    kinematics_yaml = PathJoinSubstitution([pkg_moveit, 'config', 'kinematics.yaml'])

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': False},
            {'publish_robot_description_semantic': True},
            {'moveit_controller_manager': 
             'moveit_fake_controller_manager/MoveItFakeControllerManager'},
        ],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_gui': False}],
        condition=UnlessCondition(use_teleop_relay),
    )

    rviz_config = PathJoinSubstitution([pkg_moveit, 'config', 'dual_arm.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[robot_description, robot_description_semantic],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_teleop_relay',
            default_value='false',
            description='Set true when using hand teleop so relay is the only joint_states source (stops flipping).',
        ),
        move_group_node,
        rsp_node,
        jsp_node,
        rviz_node,
    ])
