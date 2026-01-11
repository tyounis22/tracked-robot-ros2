from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('tracked_robot_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'tracked_robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    motor_node = Node(
        package='tracked_robot_hw',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        motor_node
    ])
