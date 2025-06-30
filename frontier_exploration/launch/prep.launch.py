import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='True', description='Enable SLAM'
    )
    slam_config = LaunchConfiguration('slam')
    
    # Get existing GAZEBO_MODEL_PATH
    existing_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    turtlebot3_models_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    
    if existing_gazebo_path:
        new_gazebo_path = f"{existing_gazebo_path}:{turtlebot3_models_path}"
    else:
        new_gazebo_path = turtlebot3_models_path

    # Use ONLY the nav2_bringup simulation launch - it includes Gazebo
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                         'launch', 'tb3_simulation_launch.py'])
        ),
        launch_arguments={
            'slam': slam_config,
            'use_sim_time': 'true'
        }.items()
    )
    
    
    # Launch exploration separately (no slam argument needed)
    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('explore_lite'),
                         'launch', 'explore.launch.py'])
        )
    )

    return LaunchDescription([
        slam_arg,
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=new_gazebo_path),
        simulation_launch,  # This includes Gazebo + Nav2 + SLAM
        exploration_launch  # Add exploration on top
    ])
