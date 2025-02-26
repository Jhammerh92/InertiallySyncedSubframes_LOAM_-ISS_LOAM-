from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from math import pi, radians
import os.path


def generate_launch_description():
    ld = LaunchDescription()

    package_name='iss_loam' #<--- CHANGE ME
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')

    lidar_odom_node = Node(
        package="iss_loam",
        name="lidar_odometry",
        executable="lidar_odometry",
        prefix=['stdbuf -o L'],
        parameters=[os.path.join(config_dir, 'lidar_odometry.yaml')],
        output='screen',
        # arguments=['-d', os.path.join(get_package_share_directory('my_learning_package'), 'rviz', 'LO.rviz')]
    )



    ld.add_action(lidar_odom_node)

    return ld