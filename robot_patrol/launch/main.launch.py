from launch import LaunchDescription
from launch_ros.actions import Node

# For the `right_index_multiplier` parameter use:
#  - 0.75 for simulation
#  - 0.25 for real robot

# For the `mode` parameter use:
#  - simulation
#  - real

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service_node',
            name='direction_service_node',
            output='screen',
            parameters=[{'right_index_multiplier': 0.75},],                                       
        ),
        Node(
            package='robot_patrol',
            executable='robot_patrol_service',
            name='robot_patrol_service',
            output='screen',
            parameters=[{'mode': 'simulation'},],                                        
        ),
    ])
