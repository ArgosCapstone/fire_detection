# argos_gazebo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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

    # Add nodes to the launch description
    return LaunchDescription([x1_node, quadrotor_node])
