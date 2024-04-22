from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      return LaunchDescription([
          Node(
              package='robot_patrol',
              executable='robot_patrol',
              output='screen',
        )
      ])

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():

#     patrol_node = Node(
#         package='robot_patrol',
#         executable='patrol_node',
#         output='screen',
#         remappings=[
#             ('laser_scan', '/laser_scan')
#         ]
#     )

#     return LaunchDescription([
#         patrol_node
#     ])