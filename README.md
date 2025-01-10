# RE-RASSOR-GUI
A GUI to aid in the installation and use of the capabilities team simulation for the RE-RASSOR.

# Helpful stuff 
[Install Docker on Linux](https://docs.docker.com/engine/install/ubuntu/)

# Notes
- We need Python (maybe) for the GUI.
- we need to make sure that python is correctly installed on the end users machine so that the GUI runs properly.
- The entry point for the GUI should be a bash script that checks to make sure that the users machine has the following:
	- Docker
	- Python
	- Probably more...
- Once the bash script has completed and ensured the dependecies are installed, we can have the bash script start the GUI.
- The GUI pops up with the start simulation button etc. upon starting the simulation the dockerfile is started -> do stuff with the simulation.

# Problems
- Need to figure out how to bundle bash scripts and python stuff etc into one single executable that manages all dependecies with a single click. chatGPT says this is possible...
- Need to set up some kind of access key so that the docker container can clone the repos we need. Might need to do this from the capabilities repo and the ez-rassor-sim repo. 
	- This seems like a bit of a blocker.
 	- We could premake a docker container and push it to dockerhub and then have the gui/script pull it from there.
    	- The problem I ran into is that in order to clone the repo into the docker container we have to provide credentials. The issue is that this would give access to the FSI repos to what ever user that downloads the GUI which I dont think we want.
	- The dockerhub solution could fix this...
 	- This is provided we go with the docker route. 
