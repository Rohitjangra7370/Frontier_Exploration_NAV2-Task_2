import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = 'tod_nav2'
    
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    
    world_file_name = 'office_small.world'
    world_path = PathJoinSubstitution([pkg_share, 'gazebo_models_worlds_collection', 'worlds', world_file_name])
    
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    models_path = PathJoinSubstitution([pkg_share,'gazebo_models_worlds_collection','models'])
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to world file to load'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless'
    )
    
    # Set TurtleBot3 model to waffle
    turtlebot3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    
    # Set Gazebo Classic model path (using GAZEBO_MODEL_PATH instead of GZ_SIM_RESOURCE_PATH)
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Use Gazebo Classic launch instead of Gazebo Garden
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'server': 'true',
            'verbose': 'true'
        }.items()
    )
    
    # Spawn TurtleBot3 waffle using gazebo_ros spawn_entity
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.02'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gui_arg,
        turtlebot3_model,
        set_gazebo_model_path,
        robot_state_publisher_cmd,
        gazebo_launch,
        spawn_turtlebot_cmd,
    ])