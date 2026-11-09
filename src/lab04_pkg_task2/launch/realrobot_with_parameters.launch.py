from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        Node(
        package='lab04_pkg',
        executable='ekfNode',
        name='ekfNode',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lab04_pkg'), 'config', 'params_realrobot.yaml'])
        ],
        ),
        
    ])