import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'scara.urdf'
    urdf = os.path.join(
        get_package_share_directory('scara'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
       # Node(
       #     package='joint_state_publisher_gui',
       #     executable='joint_state_publisher_gui',
       #     name='joint_state_publisher_gui',
       #     output='screen'),
        Node(
            package='scara',
            executable='scara_joint_publisher',
            name='scara_joint_publisher',
            output='screen'),
        Node(
            package='scara',                
            executable='virtual_pendant',
            name='virtual_pendant',
            output='screen'),
        Node(
            package='scara',                
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen'),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('scara'), 'rviz_basic_settings.rviz')]
        )
 
    ])
