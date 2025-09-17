from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


RVIZ_CONFIG = PathJoinSubstitution(
    [get_package_share_directory('nlink_parser'), 'rviz', 'linktrack.rviz']
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='nlink_parser',
            executable='linktrack_rviz_converter',
            name='linktrack_rviz_converter0',
            output='screen',
            parameters=[{
                'map_frame': 'linktrack_map',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base2linktrack',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'linktrack_map'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', RVIZ_CONFIG],
            output='screen',
        ),
    ])
