from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kinematic_model_node = Node(
    
        package='kinematic_model',
        executable='kinematic_model',
        name='kinematic_model_node',
        output='screen'
    
    )

    eight_trajectory_node = Node(
    
        package='eight_trajectory',
        executable='eight_trajectory_node',
        name='eight_trajectory_node',
        output = 'screen'
    
    )

    return LaunchDescription([
    
        kinematic_model_node,
        eight_trajectory_node
    
    ])