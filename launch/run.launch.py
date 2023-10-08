from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Specify the path to the mocap.launch.py file
    mocap_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('mocap_optitrack'), '/launch/mocap.launch.py'])
    )

    # Define the mocap_px4_bridge node
    mocap_px4_bridge_node = Node(
        package='mocap_px4_bridge',
        executable='mocap_px4_bridge',
        name='mocap_px4_bridge',
        output='screen'
    )

    return LaunchDescription([
        mocap_launch_file,
        mocap_px4_bridge_node,
    ])