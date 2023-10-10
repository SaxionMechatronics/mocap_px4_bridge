from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Specify the path to the mocap.launch.py file
    mocap_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('mocap_optitrack'), '/launch/mocap.launch.py'])
    )

    # Get the config path
    config = os.path.join(
        get_package_share_directory('mocap_px4_bridge'),
        'config',
        'params.yaml'
        )

    # Define the mocap_px4_bridge node
    mocap_px4_bridge_node = Node(
        package='mocap_px4_bridge',
        executable='mocap_px4_bridge',
        name='mocap_px4_bridge',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        mocap_launch_file,
        mocap_px4_bridge_node,
    ])