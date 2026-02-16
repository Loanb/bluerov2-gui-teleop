from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for GUI teleop."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('bluerov2_gui_teleop')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Namespace for ROV topics'
    )
    
    depth_sign_arg = DeclareLaunchArgument(
        'depth_sign',
        default_value='-1',
        description='Sign for depth (1 if positive z is up, -1 if positive z is down)'
    )
    
    gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='True',
        description='Show telemetry GUI window (True/False)'
    )
    
    buoyancy_arg = DeclareLaunchArgument(
        'buoyancy_compensation',
        default_value='-5.5',
        description='Constant force to compensate for positive buoyancy (negative value for downward force)'
    )
    
    # GUI teleop node
    gui_teleop_node = Node(
        package='bluerov2_gui_teleop',
        executable='gui_teleop',
        output='screen',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            {'depth_sign': LaunchConfiguration('depth_sign')},
            {'show_gui': LaunchConfiguration('show_gui')},
            {'buoyancy_compensation': LaunchConfiguration('buoyancy_compensation')},
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        depth_sign_arg,
        gui_arg,
        buoyancy_arg,
        gui_teleop_node,
    ])
