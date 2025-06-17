from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hba',
            executable='calculate_MME',
            name='calculate_mme',
            output='screen',
            parameters=[
                {'file_path': '/home/zed/Desktop/SC_PGO/data/saxion_fig88/map_saxion01.pcd'},
                {'THR_NUM': 3}
            ]   
        )
    ])
