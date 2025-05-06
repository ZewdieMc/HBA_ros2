import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='0', description='Launch RViz')

    hba_node = launch_ros.actions.Node(
        package='hba',
        executable='hba',
        name='hba',
        output='screen',
        parameters=[{
            'data_path': '/home/zed/Desktop/SC_PGO/data/',
            'total_layer_num': 3,
            'pcd_name_fill_num': 6,
            'thread_num': 16
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
        hba_node,
        rviz_node
    ])
