import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'jenga_blocks'

    # Get the path to the Jenga Base (Xacro) and Jenga Block (URDF)
    base_xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'jenga_base.urdf.xacro')
    block_urdf_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'jenga_block.urdf')

    # Convert Xacro base to URDF
    base_urdf = xacro.process_file(base_xacro_file).toxml()

    # Read Jenga Block URDF (No need to process Xacro)
    with open(block_urdf_file, 'r') as f:
        block_urdf = f.read()

    # Declare the launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'))

    # Robot State Publisher for the Base
    rsp_base = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': base_urdf, 'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(rsp_base)

    # Robot State Publisher for the Jenga Blocks
    rsp_blocks = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': block_urdf, 'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(rsp_blocks)

    # Launch RViz2 with custom config
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'jenga.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    ld.add_action(rviz2)

    return ld
