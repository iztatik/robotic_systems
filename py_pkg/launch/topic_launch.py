from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()

    start_publisher = Node(
            package = 'py_pkg',
            namespace = 'nm1',
            executable = 'publisher',
            name = 'pub')
    start_subscriber = Node(
            package = 'py_pkg',
            namespace = 'nm1',
            executable = 'subscriber',
            name = 'sub')

    launch_description.add_action(start_publisher)
    launch_description.add_action(start_subscriber)

    return launch_description

