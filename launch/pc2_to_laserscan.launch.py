import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pc2_to_laserscan'),
        'config',
        'params.yaml'
        )
    
    node=Node(
        package="pc2_to_laserscan",
        executable="pc2_to_laserscan_cpp",
        name="pc2_to_laserscan_cpp",
       # output="screen",
        parameters = [config]
    )
    ld.add_action(node)
    return ld