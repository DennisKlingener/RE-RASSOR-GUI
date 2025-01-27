## Welcome to the Capabililty team main branch.
## IMPORTANT - information for how to run the simulation and more details about the code can be found in the /ezrassor_rover/ezrassor_sim_description/docs/README.md.
##### Our team worked on the following implementations to bolster the previous 2023_Extension groups work onto a new "Sidekick" rover.
##### This new rover serves the purpose of carrying pavers on its back to the rover with the arm to constantly give refills. 
###### 1. Created a simulation for the EZ-RASSOR rover. Specifically, our new sidekick rover robot that can carry pavers on its back.
###### 2. Added a physical Servo motor to the sidekick rover 
###### 3. Implemented Aruco marker detection for the sidekick rover to detect when it is too close/far away from the Arm rassor to pick up pavers.
###### 4. Implemented communication between two rovers, sending information related to paver count, ip_address, etc between the sidekick and arm rover.

### Ensure that you are running ROS2 with Humble hawksbill, pip, flask, python
#### ROS Installation: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
or
#### clone the repo and navigate to ros_scritps/hawksbill
##### sudo su USERNAME ./hawksbill.sh
##### ensure ROS version = 2, ROS python version = 3, ROS Distro = humble by running the following command: printenv | grep -i ROS
##### install flask: pip install flask
##### install python3: sudo apt install python3 (check version: python3 --version ==should output==> Python 3.10.12)
##### install pip: sudo apt install python3-pip (check version: pip --version ==should output==> pip 22.0.2)
##### install open-cv: pip install opencv-python
##### install appropriate setup tool for python: pip install setuptools==58.2.0
##### install cvbridge: sudo apt-get install ros-${ROS_DISTRO}-cv-bridge

#### Scripts to build and run ROS packages
##### source /opt/ros/humble/setup.bash (you can also add this in bashrc file by opening .bashrc from terminal: nano ~/.bashrc, add the line at the end, ctrl + S to save, ctrl + X to exit)
##### colcon build
##### . install/local_setup.bash
##### ros2 launch ezrassor_controller_server controller_server.py
