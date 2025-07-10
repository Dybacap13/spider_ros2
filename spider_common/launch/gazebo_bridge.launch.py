from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    
    bridge_params = os.path.join(get_package_share_directory('spider_gazebo'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
 
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Правильный формат: ROS -> Gazebo (JointState -> JointCmd)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.JointCmd',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        joint_state_bridge
    ])