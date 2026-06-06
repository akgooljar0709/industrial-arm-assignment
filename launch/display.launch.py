import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    """
    Launch file to visualize the industrial arm robot in RViz 2.
    
    Usage:
        ros2 launch industrial_arm_description display.launch.py
    """
    
    # Get package directories
    pkg_share = get_package_share_directory('industrial_arm_description')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_file = LaunchConfiguration('rviz_config', 
                                          default=os.path.join(pkg_share, 'config', 'display.rviz'))
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'xacro', 'robot.urdf.xacro')
    
    # Convert Xacro to URDF
    robot_description_content = Command(
        ['xacro ', urdf_file, ' use_sim_time:=', use_sim_time]
    )
    
    robot_description = {
        'robot_description': robot_description_content
    }
    
    # Nodes to launch
    nodes = [
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        
        # Joint State Publisher GUI (allows interactive joint manipulation)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ]
    
    # Launch description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='Path to RViz config file'
        ),
    ])
    
    for node in nodes:
        ld.add_action(node)
    
    return ld
