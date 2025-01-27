This is a modified version of the original ezrassor_sim_description. This version is designed for ROS2 Humble
The point of ezrassor_sim_description is to simulate a rover connected to the ezrassor_controller application to move the simulation using app control.
In this repo, the simulation is the sidekick rover carrying pavers. This can be modified with different urdf models to simulate different rovers.

# Prerequisites - Make sure to install
- sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
- pip install -U flask-cors (this is used in the controller_server)
- Install Gazebo 

# Simulation information.
These steps start from the very top level of the 2024-RE-RASSOR-CAPABILITES repo!
### Lauch the app
1) First launch the mobile application ezrassor_controller. cd into ezrassor_controller and follow documentation to launch the app.
### Launch the api/controller_server
2) Open a new terminal and 'cd ezrassor_rover'
3) run 'colcon build'
4) Run 'source install/setup.bash'
5) Run 'ros2 launch ezrassor_controller_server controller_server.py' to launch the server_controller. This is sort of the middleman between the mobile app and the simulation. The console should output a 'Running on http://192.168.1.9:5000'. THIS IS THE IP ADDRESS TO PUT ON THE APP WHEN PROMPTED.
### Launch a gazebo world
Do either one of the following to launch gazebo worlds
6) Open a new terminal and 'cd ezrassor_rover'
7) Run source install/setup.bash
8) 'ros2 launch gazebo_ros gazebo.launch.py' to launch an empty gazebo world.
                                      or
6) Seperatly clone the ezrassor_sim_gazebo repo 'https://github.com/FlaSpaceInst/ezrassor_sim_gazebo' In that repo, run 'colcon build', 'source install/setup.bash'
7) Run '. /usr/share/gazebo/setup.sh'
8) Run 'ros2 launch ezrassor_sim_gazebo gazebo_launch.py world:=moon.world'
### Launch the actual rover into the simulation
9) Open a new terminal and 'cd ezrassor_rover'
10) Run 'source install/setup.bash'
11) Run to launch the rover 'ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor'

# Communcation (Rover to Rover) information.
1) On the capability repo there are two branches, 'main' and 'secondroverinsimulation'.
2) Locally clone and run both of these on your computer. Follow the steps in /ezrassor_rover/ezrassor_sim_description/docs/README.md on secondrvoerinsimulation for a slgihtly different process of laucnhing the simulation.
3) By now, you should be able to run two different rovers on on one simulation controlled by two different apps.
4) How does this work? Rover 1 controller server publishes its information on a ROS topic, such as sidekick_rover_info. Rover 2 subscribes to that topic to get the information from the first rover. Then, the second rover controller_server sends this information across HTTP to the frontend, where it is displayed. The same process happens in reverse where Rover 2 sends its information to Rover 1. Trace through the code in the controller_server/source/ezrassor_controller/__main__.py in both branches of the repo to see what is going on. Specifically, you can trace through the sidekick_rover_info topic and see how data is being sent between the two rovers. You should also see publishing and subscribing messages in the terminal when you are running both rovers side by side! This communication can be made better by 1. passing dynmaic data (Such as updating the paver count when the arm rassor takes a paver off of the sidekick rover) and 2. communicating information between more than 2 rovers. In theory, ROS topics such as sidekick_rover_info, can be subscribed to on any rovers controller_server code. Thus, our basic setup of communication paves the way for future groups to bolster this communication technology.
5) Peep the changes from this commit if you want to see how to set up a new rover the communicate with the main branch. https://github.com/FlaSpaceInst/2024-RE-RASSOR-Capabilities/commit/af48504f634f9910199c54dce7c4c5ba4c4a18f5. There are some modifications that must be made in order to run two rovers locally side by side controlled by two different apps. Look through the commits on second_rover_in_simulation branch to see how everything was set up to communicate with main branch.

# Extra notes
This is what a wheel message looks like sent from the frontend on a right button press.
  {'wheel_action': {'linear_x': 0, 'angular_z': '-1.0'}}
This is what that message is changed to in the controller server
  geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-1.0))
This is what the message is changed to in the ezrassor_simulation_rover before it is sent to gazebo
  geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-5.0))

Publishing messages
ros2 topic pub /ezrassor/diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{
  linear: {x: 1.0, y: 0.0, z: 0.0}, 
  angular: {x: 0.0, y: 0.0, z: 0.0}
}"

ros2 topic echo /ezrassor/diff_drive_controller/cmd_vel_unstamped
or
ros2 topic echo /diff_drive_controller/cmd_vel

PREVIOUS REPO DOCUMENTION (OLD 2023 Extension Group)
---------------------------------------------------------------------------------------------------------------
![Build Badge](https://github.com/FlaSpaceInst/ezrassor_sim_description/workflows/Build/badge.svg) 
![Style Badge](https://img.shields.io/badge/Code%20Style-black-000000.svg)

The `ezrassor_sim_description` package contains the xacro description for the ezrassor robot and spawns a new robot into a running Gazebo simulation.

usage
-----
```
command:
  ros2 launch ezrassor_sim_description spawn_ezrassor.py [argument:=value]

optional arguments:
  x             [lowercase] float value representing the spawn value for the robot along the x axis in Gazebo (default 0.0)
  y             [lowercase] float value representing the spawn value for the robot along the y axis in Gazebo (default 0.0)
  z             [lowercase] float value representing the spawn value for the robot along the z axis in Gazebo (default 0.0)
  R             [UPPERCASE] float value representing the spawn roll degree for the robot in Gazebo (default 0.0)
  P             [UPPERCASE] float value representing the spawn pitch degree for the robot in Gazebo (default 0.0)
  Y             [UPPERCASE] float value representing the spawn yaw degree for the robot in Gazebo (default 0.0)
  robot_name    string value for the unique name of the robot, used for topic name and namespace (default ezrassor)
  model         string value for the full path of the robot xacro file (default ezrassor_sim_description/urdf/ezrassor.xacro)
```

example
--------
Launch the Gazebo client with the name `ezrassor_1`:
``` sh
# First launch Gazebo and then spawn the robot with the controller manager
ros2 launch ezrassor_sim_gazebo gazebo_launch.py
or
ros2 launch gazebo_ros gazebo.launch.py
ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor_1
```   

testing
-------
Changes to this package will run through the linters specified in the GitHub Actions workflow which include:
- Black formatting check
- PEP8 compliance check

These tests will run automatically when changes are checked in to the `development` branch.  

Before you check in changes, please test your changes locally:

```sh
python3 -m pip install --upgrade black flake8
python3 -m black --check .
python3 -m flake8 .
```
