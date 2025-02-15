Requires Nvidia GPU :'(

I couldn't test I get error like

```bash
[gzclient-2] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.```

because there's no Nvidia GPU

Full error: 

rituraj@rituraj-HPENVY:~/Northeastern_University/RSS/jenga_project/project_ws/simulation_config_ws$ ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=wx250s use_rviz:=false headless:=true
[INFO] [launch]: All log files can be found below /home/rituraj/.ros/log/2025-02-13-22-59-05-456945-rituraj-HPENVY-27461
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gzserver-1]: process started with pid [27473]
[INFO] [gzclient-2]: process started with pid [27475]
[INFO] [spawn_entity.py-3]: process started with pid [27477]
[INFO] [robot_state_publisher-4]: process started with pid [27479]
[gzserver-1] [INFO] [1739505547.423893024] [gazebo_ros2_control]: Loading gazebo_ros2_control plugin
[gzserver-1] [INFO] [1739505547.425221795] [wx250s.gazebo_ros2_control]: Starting gazebo_ros2_control plugin in namespace: /wx250s
[gzserver-1] [INFO] [1739505547.425321783] [wx250s.gazebo_ros2_control]: Starting gazebo_ros2_control plugin in ros 2 node: gazebo_ros2_control
[gzserver-1] [INFO] [1739505547.426254618] [wx250s.gazebo_ros2_control]: connected to service!! robot_state_publisher
[gzserver-1] [INFO] [1739505547.428113749] [wx250s.gazebo_ros2_control]: Received urdf from param server, parsing...
[gzserver-1] [INFO] [1739505547.428172524] [wx250s.gazebo_ros2_control]: Loading parameter files /home/rituraj/Northeastern_University/RSS/jenga_project/project_ws/simulation_config_ws/install/interbotix_xsarm_sim/share/interbotix_xsarm_sim/config/trajectory_controllers/wx250s_trajectory_controllers.yaml
[gzserver-1] [INFO] [1739505547.434809456] [wx250s.gazebo_ros2_control]: Loading joint: waist
[gzserver-1] [INFO] [1739505547.434835540] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.434844206] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.434850310] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.434855140] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.434977217] [wx250s.gazebo_ros2_control]: Loading joint: shoulder
[gzserver-1] [INFO] [1739505547.434987448] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.434992385] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.434997359] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435001361] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435053053] [wx250s.gazebo_ros2_control]: Loading joint: elbow
[gzserver-1] [INFO] [1739505547.435059705] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435064245] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435068501] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435072572] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435120878] [wx250s.gazebo_ros2_control]: Loading joint: forearm_roll
[gzserver-1] [INFO] [1739505547.435127405] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435131529] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435137093] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435141368] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435183941] [wx250s.gazebo_ros2_control]: Loading joint: wrist_angle
[gzserver-1] [INFO] [1739505547.435189997] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435194181] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435198345] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435202322] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435241233] [wx250s.gazebo_ros2_control]: Loading joint: wrist_rotate
[gzserver-1] [INFO] [1739505547.435246922] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435251119] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435257013] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435261499] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435301470] [wx250s.gazebo_ros2_control]: Loading joint: gripper
[gzserver-1] [INFO] [1739505547.435306899] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435311179] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435315224] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435319144] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435360654] [wx250s.gazebo_ros2_control]: Loading joint: left_finger
[gzserver-1] [INFO] [1739505547.435366615] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435370807] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435375297] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435380986] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435452604] [wx250s.gazebo_ros2_control]: Loading joint: right_finger
[gzserver-1] [INFO] [1739505547.435460495] [wx250s.gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1739505547.435464842] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435470698] [wx250s.gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1739505547.435475542] [wx250s.gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1739505547.435560640] [resource_manager]: Initialize hardware 'XSHardwareInterface' 
[gzserver-1] [INFO] [1739505547.435643540] [resource_manager]: Successful initialization of hardware 'XSHardwareInterface'
[gzserver-1] [INFO] [1739505547.435697473] [resource_manager]: 'configure' hardware 'XSHardwareInterface' 
[gzserver-1] [INFO] [1739505547.435703445] [resource_manager]: Successful 'configure' of hardware 'XSHardwareInterface'
[gzserver-1] [INFO] [1739505547.435708017] [resource_manager]: 'activate' hardware 'XSHardwareInterface' 
[gzserver-1] [INFO] [1739505547.435711426] [resource_manager]: Successful 'activate' of hardware 'XSHardwareInterface'
[gzserver-1] [INFO] [1739505547.435761968] [wx250s.gazebo_ros2_control]: Loading controller_manager
[gzserver-1] [INFO] [1739505547.443502723] [wx250s.gazebo_ros2_control]: Loaded gazebo_ros2_control.
[INFO] [spawn_entity.py-3]: process has finished cleanly [pid 27477]
[INFO] [spawner-5]: process started with pid [27635]
[gzclient-2] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.
[gzserver-1] [INFO] [1739505547.976178691] [wx250s.controller_manager]: Loading controller 'joint_state_broadcaster'
[spawner-5] [INFO] [1739505547.990314930] [wx250s.joint_state_broadcaster_spawner]: Loaded joint_state_broadcaster
[gzserver-1] [INFO] [1739505547.990827445] [wx250s.controller_manager]: Configuring controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1739505547.991035607] [wx250s.joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[spawner-5] [INFO] [1739505547.999429095] [wx250s.joint_state_broadcaster_spawner]: Configured and activated joint_state_broadcaster
[INFO] [spawner-5]: process has finished cleanly [pid 27635]
[INFO] [spawner-6]: process started with pid [27675]
[INFO] [spawner-7]: process started with pid [27677]
[gzserver-1] [INFO] [1739505548.422295185] [wx250s.controller_manager]: Loading controller 'arm_controller'
[gzserver-1] [WARN] [1739505548.430005826] [wx250s.arm_controller]: [Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true. The default behavior will change to false.
[spawner-7] [INFO] [1739505548.439767103] [wx250s.arm_controller_spawner]: Loaded arm_controller
[gzserver-1] [INFO] [1739505548.440758427] [wx250s.controller_manager]: Configuring controller 'arm_controller'
[gzserver-1] [INFO] [1739505548.440925476] [wx250s.arm_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[gzserver-1] [INFO] [1739505548.440956028] [wx250s.arm_controller]: Command interfaces are [position] and state interfaces are [position].
[gzserver-1] [INFO] [1739505548.440993562] [wx250s.arm_controller]: Using 'splines' interpolation method.
[gzserver-1] [INFO] [1739505548.441540526] [wx250s.arm_controller]: Controller state will be published at 50.00 Hz.
[gzserver-1] [INFO] [1739505548.442650355] [wx250s.arm_controller]: Action status changes will be monitored at 20.00 Hz.
[gzserver-1] [INFO] [1739505548.523650259] [wx250s.controller_manager]: Loading controller 'gripper_controller'
[gzserver-1] [WARN] [1739505548.526036006] [wx250s.gripper_controller]: [Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true. The default behavior will change to false.
[spawner-6] [INFO] [1739505548.537033094] [wx250s.gripper_controller_spawner]: Loaded gripper_controller
[gzserver-1] [INFO] [1739505548.537525584] [wx250s.controller_manager]: Configuring controller 'gripper_controller'
[gzserver-1] [INFO] [1739505548.537600017] [wx250s.gripper_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[gzserver-1] [INFO] [1739505548.537610742] [wx250s.gripper_controller]: Command interfaces are [position] and state interfaces are [position].
[gzserver-1] [INFO] [1739505548.537618710] [wx250s.gripper_controller]: Using 'splines' interpolation method.
[gzserver-1] [INFO] [1739505548.537758015] [wx250s.gripper_controller]: Controller state will be published at 50.00 Hz.
[gzserver-1] [INFO] [1739505548.538599687] [wx250s.gripper_controller]: Action status changes will be monitored at 20.00 Hz.
[spawner-6] [INFO] [1739505548.542862858] [wx250s.gripper_controller_spawner]: Configured and activated gripper_controller
[spawner-7] [INFO] [1739505548.548647555] [wx250s.arm_controller_spawner]: Configured and activated arm_controller
[INFO] [spawner-6]: process has finished cleanly [pid 27675]
[INFO] [spawner-7]: process has finished cleanly [pid 27677]
[ERROR] [gzclient-2]: process has died [pid 27475, exit code -6, cmd 'gzclient'].
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 27479]
[INFO] [gzserver-1]: process has finished cleanly [pid 27473]

