# Jengics
# Hi & Hello

# 2 Packages below
## jenga_blocks
It contains 2 packages 
- jenga_blocks
This command will pile up the jenga stack and the base as well you can open the `spawn_jenga_launch.py` to manually change whether to make blocks spawn randomly or in jenga format
It also launches ros2 controllers through which you can rotate a base by writing your own script
```bash
ros2 launch jenga_blocks spawn_jenga_launch.py
```

- jenga_bot
Well I was trying to run the widow x250s arm using this, it works but the issue is we cant import those controller codes which gazebo needs to acces each joint in arm so no use of this actually
```bash
ros2 launch jenga_bot spawn_launch.py
```

## jenga_project
It contains 2 sub folders
- packages (it has those github packages)

- project_ws
This is where we find sorted folders for each thing to do with the arm
We referred this link here -> https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages.html

The `project_ws` has `dom.txt` which stands for DOCUMENT OBJECT MODEL (idk this name suited more :|)  it has the structure of ros2 packages inside it
`packages.txt` has list of GitHub packages required (at minimum) to run a bot either simulation or real bot

If you'd like to run the bot refer these websites

[Arm Description](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_descriptions.html)
[Arm Control](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_control.html)
[Simulation Configuration](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/simulation_configuration.html)
[Move It Motion Planning](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/moveit_motion_planning_configuration.html)


