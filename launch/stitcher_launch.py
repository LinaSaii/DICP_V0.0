from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('doppler_icp_stitcher_open3d_pro2')
    param_path = os.path.join(package_dir, 'config', 'param.yaml')

    return LaunchDescription([
        Node(
            package='doppler_icp_stitcher_open3d_pro2',
            executable='stitch_node',
            name='doppler_icp_stitcher',
            output='screen',
            parameters=[param_path]
        )
    ])
