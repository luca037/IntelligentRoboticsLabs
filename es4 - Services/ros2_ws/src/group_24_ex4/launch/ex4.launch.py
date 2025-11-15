from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ir_launch_dir = get_package_share_directory('ir_launch')
    

    external_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ir_launch_dir, 'launch', 'exercise_4.launch.py')
        )
    )

    local_pkg_name = 'group_24_ex4' 
    turtlebot_server_node = Node(
        package=local_pkg_name,
        executable='turtlebot_server',
        name='turtlebot_server',
        output='screen',
        emulate_tty=True
    )

    burrow_client_node = Node(
        package=local_pkg_name,
        executable='burrow_client',
        name='burrow_client',
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        external_launch_file,
        turtlebot_server_node,
        burrow_client_node,
    ])
