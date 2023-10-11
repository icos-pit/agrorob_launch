import launch
import launch_ros.actions
import os

def generate_launch_description():
    # Get the path to the can_interface_check.py script within the package
    package_share_dir = os.path.join(os.path.dirname(__file__), '..', 'share', 'agrorob')
    check_script_path = os.path.join(package_share_dir, 'can_interface_check.py')

    # Call the function to check the can0 interface
    check_can0_interface_action = launch.actions.ExecuteProcess(
        cmd=["python3", check_script_path],
        output='screen'
    )

    return launch.LaunchDescription([
        check_can0_interface_action,
        launch_ros.actions.Node(
            package='ros2_socketcan',
            executable='socket_can_receiver_node_exe',
            name='socket_can_receiver_node'
        ),
        launch_ros.actions.Node(
            package='ros2_socketcan',
            executable='socket_can_sender_node_exe',
            name='socket_can_sender_node'
        ),
        launch_ros.actions.Node(
            package='agrorob_driver',
            executable='agrorob_driver_node',
            name='agrorob_driver_node'
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
