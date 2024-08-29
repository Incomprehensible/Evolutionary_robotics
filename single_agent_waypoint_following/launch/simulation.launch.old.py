from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='/opt/ros/humble/share/turtlebot3_gazebo/models'),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("turtlebot3_gazebo"), '/launch', '/empty_world.launch.py'])
            )])
    node = Node(
        package="single_agent_waypoint_following",
        executable="NEAT_evolution",
        name='NEAT_evolution',
        # respawn=True,
    )

    ld.add_action(node)
    return ld
