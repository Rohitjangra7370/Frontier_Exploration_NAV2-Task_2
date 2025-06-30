import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = get_package_share_directory('tod_nav2')
    launch_dir=os.path.join(pkg_name, 'launch')
    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir,'gazebo_launch.py'])
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir,'mapping_launch.py'])
        )
    
    
    ])
