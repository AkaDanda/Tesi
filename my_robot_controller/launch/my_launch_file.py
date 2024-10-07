import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='mesh_visualizer.py',
            name='mesh_visualizer',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='static_camera_frame.py',
            name='static_camera_frame',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='tf_to_xyz_node.py',
            name='tf_to_xyz_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch.substitutions.LaunchConfiguration('apriltag_ros'), 'v4l2_36h11.launch.yml']
            ),
            launch_arguments={}.items()
        ),
    ])

