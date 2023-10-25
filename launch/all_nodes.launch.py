from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import subprocess


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
    
    check_can0_interface()

    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')


    return LaunchDescription([

        DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
        TextSubstitution(text=os.path.join( get_package_share_directory('teleop_twist_joy'), 'config', '')),
        joy_config, TextSubstitution(text='.config.yaml')]),
        
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),


        Node(
            package='agrorob_driver',
            executable='agrorob_interface',
            name='agrorob_interface',
            parameters=[{
                'joy_wire': 1,
               
        }]),

        Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
        }]),

        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[config_filepath],
            remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))}
        ),

        ExecuteProcess(
            cmd=["ros2", "param", "set", "/teleop_twist_joy_node", "axis_angular.yaw", "3"],
            output='screen'
        ),

        ## uncomment if wheel turning oposite direction
        # ExecuteProcess(
        #     cmd=["ros2", "param", "set", "/teleop_twist_joy_node", "inverted_reverse", "true"],
        #     output='screen'
        # ),


        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros2_socketcan'),
                             'launch/socket_can_bridge.launch.xml')
            ),
        ),
 
    ])

