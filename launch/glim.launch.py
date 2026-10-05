import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('glim_ros'), 'config', 'config_gazebo')

    return LaunchDescription([
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_rosnode',
            output='screen',
            parameters=[
                {'config_path': config_path},
                {"use_sim_time": True}
            ],
        )
    ])