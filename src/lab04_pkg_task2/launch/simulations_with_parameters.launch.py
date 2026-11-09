from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'lab04.launch.py'
            ])
        )
    )

    EKF_node = Node(
        package='lab04_pkg',
        executable='ekfNode',
        name='ekfNode',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('lab04_pkg'), 'config', 'params_simulation.yaml'
            ]),
        ],
    )


    delayed_controller = TimerAction(
        period=10.0, 
        actions=[EKF_node]
    )

    return LaunchDescription([
        sim_launch,
        delayed_controller
    ])