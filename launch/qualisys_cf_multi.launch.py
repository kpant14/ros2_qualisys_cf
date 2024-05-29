import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    qualisys_cf_multi_node = Node(
        package='ros2_qualisys_cf',
        executable='qualisys_cf_multi',
        name='qualisys_cf_multi', 
        output='screen'
    )
   
    return LaunchDescription([
        qualisys_cf_multi_node,        
    ])
