import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('ocslam')
    robot_description_path = os.path.join(package_dir, 'resource', 'ocslam.urdf')

    ocslam_driver = WebotsController(
        robot_name='Robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )


    main = Node(
        package='ocslam',
        executable='main',
        output='screen',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        ocslam_driver, main])