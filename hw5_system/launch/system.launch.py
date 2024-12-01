from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        # Pose Estimator
        Node(
            package='hw5_system',
            executable='pose_estimate_node',
            name='pose_estimate_node',
        ),

        # Mpi twist
        Node(
            package='hw5_system',
            executable='twist_send',
            name='twist_send',
        ),
        
        # Planner
        Node(
            package='hw5_system',
            executable='plan_node',
            name='plan_node',
        ),

        # PID node
        Node(
            package='hw5_system',
            executable='pid_node',
            name='pid_node',
            output='screen'
        ),

    ])