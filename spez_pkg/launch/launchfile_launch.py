from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='spez_pkg',
        #    executable='camera_pipeline',
        #    name='camera_image'
        #),
        Node(
            package='spez_pkg',
            executable='dummycam',
            name='dummy_camera'
        ),
        Node(
            package='spez_pkg',
            executable='vlfm_pipeline',
            name='vlfm_dummy'
        ),
        Node(
            package='spez_pkg',
            executable='ros_pipeline',
            name='ros_controller'
        )
    ])


#https://www.clearpathrobotics.com/assets/guides/melodic/ros/Launch%20Files.html
#https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

