from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

import subprocess
from getpass import getpass

def check_can0_interface():

    # Get network interfaces
    result = subprocess.run(["ifconfig"], capture_output=True, text=True)
    interfaces = result.stdout

    # Check if can0 interface is present
    if "can0" in interfaces:
        print("can0 interface present")
    else:
        raise ValueError('\n can0 interface is absent, please\n1.plug-in CAN-USB adaptor\n2.run\n \n sudo ip link set can0 up type can bitrate 250000 \n \n') 



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get the path to the can_interface_check.py script within the package
    # package_share_dir = os.path.join(os.path.dirname(__file__), '..', 'share', 'agrorob_launch')
    # check_script_path = os.path.join(package_share_dir, 'launch/can_interface_check.py')

    # Call the function to check the can0 interface
    # check_can0_interface_action = ExecuteProcess(
    #     cmd=["python3", check_script_path],
    #     output='screen'
    # )

    return LaunchDescription([
        check_can0_interface(),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='agrorob_driver',
            executable='agrorob_interface',
            name='agrorob_interface'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros2_socketcan'),
                             'launch/socket_can_bridge.launch.xml')
            )
        ),
    ])

