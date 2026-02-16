from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for complete BlueROV2 stack with GUI teleop."""
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Namespace for ROV topics'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo with GUI (true/false)'
    )
    
    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='True',
        description='Show GUI teleop window (True/False)'
    )
    
    # Include world (Gazebo simulation)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('bluerov2_description'),
                'launch',
                'world_launch.py'
            ])
        ),
        launch_arguments=[
            ('gui', LaunchConfiguration('gui')),
            ('spawn', 'true'),
        ]
    )
    
    # Include wrench control (thruster manager + controller)
    wrench_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('bluerov2_control'),
                'launch',
                'wrench_launch.py'
            ])
        ),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', 'false'),  # Disable RViz to avoid crashes
        ]
    )
    
    # Include GUI teleop
    gui_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('bluerov2_gui_teleop'),
                'launch',
                'gui_teleop_launch.py'
            ])
        ),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('show_gui', LaunchConfiguration('show_gui')),
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        gui_arg,
        show_gui_arg,
        world_launch,
        wrench_launch,
        gui_teleop_launch,
    ])
