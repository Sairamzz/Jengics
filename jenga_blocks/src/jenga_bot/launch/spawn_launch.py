import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_name = 'jenga_bot'

    controller_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'wx250s_trajectory_controllers.yaml')

    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')
    ust = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true')

    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp_launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )



    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Start Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='/wx250s',
        parameters=[controller_config],
        output='screen'
    )

    # Load Controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace='/wx250s',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='/wx250s',
        arguments=['arm_controller'],
        output='screen'
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='/wx250s',
        arguments=['gripper_controller'],
        output='screen'
    )

    spawn_base = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                # parameters=[{'robot_description': xacro.process_file(base_file).toxml(), 'use_sim_time': use_sim_time}],
                arguments=[
                    '-topic', '/robot_description',
                    # '-file', base_file,
                    '-entity', 'widowx250s',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.4',
                    '-R', '-3.14',
                    '-P', '3.14',
                    '-Y', '0'
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(gazebo)
    ld.add_action(ust)
    ld.add_action(rsp)
    ld.add_action(controller_manager)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_arm_controller)
    ld.add_action(load_gripper_controller)
    ld.add_action(spawn_base)
    
    # ros2 control load_controller --set-state active joint_effort_controller 

    

    return ld

