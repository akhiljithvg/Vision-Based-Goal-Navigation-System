import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_robot')

    # Base Robot Launch (Gazebo, RSP, Bridge)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'robot.launch.py')]),
    )

    # YOLO Perception Node
    yolo_node = Node(
        package='diff_drive_robot',
        executable='yolo_perception_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Semantic Memory Node
    memory_node = Node(
        package='diff_drive_robot',
        executable='semantic_memory_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Nav2 Bringup (Navigation)
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # SLAM Toolbox (for mapping online)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'slam_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Odom TF Node
    odom_tf_node = Node(
        package='diff_drive_robot',
        executable='odom_tf_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_launch,
        odom_tf_node,
        yolo_node,
        memory_node,
        nav2_launch,
        slam_launch
    ])

