import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    params = os.path.join(get_package_share_directory('single_agent_waypoint_following'), 'config', 'simulation.yaml')
    ld = LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='/opt/ros/humble/share/turtlebot3_gazebo/models'),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("turtlebot3_gazebo"), '/launch', '/empty_world.launch.py'])
            )])
    simulator = Node(
        package="single_agent_waypoint_following",
        executable="NEAT_simulator",
        name='NEAT_simulator',
        parameters=[params]
    )
    controller = Node(
        package="single_agent_waypoint_following",
        executable="NEAT_controller",
        name='CNN_controller',
        parameters=[params],
    )

    ld.add_action(simulator)
    ld.add_action(controller)
    return ld
