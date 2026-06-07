import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('industrial_arm_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Paths
    xacro_file = PathJoinSubstitution([pkg_share, 'xacro', 'robot.urdf.xacro'])
    rviz_file = PathJoinSubstitution([pkg_share, 'config', 'display.rviz'])
    
    # Convert Xacro to URDF
    robot_description = Command([
        'xacro ',
        xacro_file,
        ' use_sim_time:=',
        use_sim_time
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_file,
            description='Path to RViz config file'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
