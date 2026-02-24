from setuptools import setup, find_packages

package_name = 'teleop_hand_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'hand_tracker_node         = teleop_hand_tracking.hand_tracker_node:main',
            'joint_state_relay        = teleop_hand_tracking.joint_state_relay_node:main',
            'servo_to_joint_state_node = teleop_hand_tracking.servo_to_joint_state_node:main',
        ],
    },
)
