# RE-RASSOR-GUI
A GUI to aid in the installation and use of the capabilities team simulation for the RE-RASSOR.


# Notes
- We need python (maybe) for the GUI.
- we need to make sure that python is correctly installed on the end users machine so that the GUI runs properly.
- The entry point for the GUI should be a bash script that checks to make sure that the users machine has the following:
	- Docker
	- Python
	- Probably more...
- Once the bash script has completed and ensured the dependecies are installed, we can have the bash script start the GUI.
- The GUI pops up with the start simulation button etc. upon starting the simulation the dockerfile is started -> do stuff with the simulation.

# Problems
- Need to figure out how to bundle bash scripts and python stuff etc into one single executable that manages all dependecies with a single click. chatGPT says this is possible...
