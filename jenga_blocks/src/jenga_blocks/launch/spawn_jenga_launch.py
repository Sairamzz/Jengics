import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import random

import xacro

def deploy_jenga_blocks(ld, block_file, use_sim_time, stacked=False):
    # <box size="0.075 0.025 0.015"/>
    num_blocks = 10 # for random blocks : count

    x_position = 0
    y_position = 0
    yaw_rotation = 0
    z_position = 0

    if stacked:
        # spawn blocks in jenga format
        num_layers = 10
        blocks_per_layer = 3
        block_spacing = 0.027
        layer_height = 0.17
        # rotate for jenga
        rotate = True
        
        for i in range(num_layers):
            for j in range(blocks_per_layer):

                if rotate:
                    x_position = (j - 1) * block_spacing + 0.4
                    y_position = 0
                    z_position = layer_height
                    yaw_rotation = 1.57
                else:
                    x_position = 0.4
                    y_position = (j - 1) * block_spacing
                    z_position = layer_height
                    yaw_rotation = 0
                
                layer_height += 0.006
                    
                spawn_entity = TimerAction(
                    period= 2 + (i + j*0.4),
                    actions=[
                        Node(
                            package='gazebo_ros',
                            executable='spawn_entity.py',
                            # parameters=[{'robot_description': block_file.toxml(), 'use_sim_time': use_sim_time}],
                            arguments=[
                                # '-topic', '/robot_description',
                                '-file', block_file,
                                '-entity', f'block_{i}_{j}',
                                '-x', str(x_position),
                                '-y', str(y_position),
                                '-z', str(z_position),
                                '-R', '0',
                                '-P', '0',
                                '-Y', str(yaw_rotation)
                            ],
                            output='screen'
                        )
                    ]
                )
                ld.add_action(spawn_entity)
            rotate = not rotate 
    else:
        spawn_delay = 0.5
        for i in range(num_blocks):
            x_position = random.uniform(-0.1, 0.1)
            y_position = random.uniform(-0.1, 0.1)
            z_position = random.uniform(0.1, 0.5)

            roll = random.uniform(0, 3.14)
            pitch = random.uniform(0, 3.14)
            yaw = random.uniform(0, 6.28)

            spawn_entity = TimerAction(
                period=spawn_delay * i,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        # parameters=[{'robot_description': block_file.toxml(), 'use_sim_time': use_sim_time}],
                        arguments=[
                            # '-topic', '/robot_description',
                            '-file', block_file,
                            '-entity', f'random_block_{i}',
                            '-x', str(x_position),
                            '-y', str(y_position),
                            '-z', str(z_position),
                            '-R', str(roll),
                            '-P', str(pitch),
                            '-Y', str(yaw)
                        ],
                        output='screen'
                    )
                ]
            )

            ld.add_action(spawn_entity)

def deploy_jenga_base(base_file, use_sim_time):
    spawn_base = TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    parameters=[{'robot_description': xacro.process_file(base_file).toxml(), 'use_sim_time': use_sim_time}],
                    arguments=[
                        '-topic', '/robot_description',
                        # '-file', base_file,
                        '-entity', 'base',
                        '-x', '0.4',
                        '-y', '0.0',
                        '-z', '0.03',
                        '-R', '0.0',
                        '-P', '0.0',
                        '-Y', '0.0'
                    ],
                    output='screen'
                )
            ]
        )
    return spawn_base
    
def launch_gazebo():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
    )
    return gazebo
        
 

def generate_launch_description():
    pkg_name = 'jenga_blocks'
    block_file = os.path.join(get_package_share_directory(pkg_name),'description','jenga_block.urdf')
    base_file = os.path.join(get_package_share_directory(pkg_name),'description','jenga_base.urdf.xacro')

    yaml_file_path = os.path.join(get_package_share_directory(pkg_name),'config','base_controllers.yaml')

    

    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')
    ust = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true')
    ld.add_action(ust)



    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp_launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # rsp = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',

    #     # parameters=[{'robot_description': (lambda f: f.read())(open(base_file, 'r')), 'use_sim_time': use_sim_time}]
    #     parameters=[{'robot_description': xacro.process_file(base_file).toxml(), 'use_sim_time': use_sim_time}]
    # )
    ld.add_action(rsp)

    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[yaml_file_path],
        output='screen'
    )
    ld.add_action(controller_manager)

    gazebo = launch_gazebo()
    ld.add_action(gazebo)


    spawn_base = deploy_jenga_base(base_file, use_sim_time)
    ld.add_action(spawn_base)
    

    # deploy jenga blocks either un-stacked (False) or stacked (True)
    deploy_jenga_blocks(ld, block_file, use_sim_time, stacked=True)

    # ros2 control load_controller --set-state active joint_effort_controller 

    
    return ld

