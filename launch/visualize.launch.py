import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='1', description='Launch RViz')

    visualize_map_node = launch_ros.actions.Node(
        package='hba',
        executable='visualize_map',
        name='visualize_map',
        output='screen',
        parameters=[{
            'file_path': '/home/sam/Desktop/kitti07/',#! change this according to the pcd directory
            'downsample_size': 0.1,
            'pcd_name_fill_num': 5,  # set 5 for kitti07 and 0 for park
            'marker_size': 0.5
        }]
    )

    rviz_node = launch_ros.actions.Node(
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '/home/zed/thesis_ws/src/HBA/rviz_cfg/rosbag.rviz']
    )

    return LaunchDescription([
        rviz_arg,
        visualize_map_node,
        rviz_node
    ])
