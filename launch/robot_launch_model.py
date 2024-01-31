import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('ocslam')
    robot_description_path = os.path.join(package_dir, 'resource', 'ocslam.urdf')


    default_rviz_config_path = os.path.join(package_dir, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

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

    # yolo_node python file
    yolo_node = Node(
        package='ocslam',
        executable='YOLO.py'
    )

    calibration_node = Node(
        package='ocslam',
        executable='Calibration.py'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=robot_description_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        ocslam_driver, calibration_node,
        yolo_node, main, rviz_node])