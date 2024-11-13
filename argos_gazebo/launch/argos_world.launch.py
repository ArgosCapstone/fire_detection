# argos_gazebo.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    # Start Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('argos_gazebo'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': '/home/eugene/argos_ros2_ws/src/fire_detection/argos_gazebo/worlds/parking_garage.world'}.items()
    )

    # Start Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('argos_gazebo'), 'launch', 'gzclient.launch.py')
        )
    )

    # Launch the X1 ground robot node
    x1_node = Node(
        package='argos_gazebo',       # Your package name
        executable='x1_node',         # Make sure `x1_node.py` is executable
        name='x1_node',
        output='screen',
        parameters=[{'use_sim_time': True}]  # Use simulation time
    )

    # Launch the quadrotor node
    quadrotor_node = Node(
        package='argos_gazebo',       # Your package name
        executable='quadrotor_node',  # Make sure `quadrotor_node.py` is executable
        name='quadrotor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]  # Use simulation time
    )

    # Launch Gazebo with the parking_garage.world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '/home/eugene/argos_ros2_ws/src/fire_detection/argos_gazebo/worlds/parking_garage.world'],
        output='screen'
    )

    # Add nodes to the launch description
    return LaunchDescription([x1_node, quadrotor_node, gazebo])
