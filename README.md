# ME 495: Embedded Systems (Fall 2019)
# Final Project - MegaBloks
## Group Members:
- Allie Pinosky
- Luxi Huang
- Marcel Bonnici
- Senthil Palanisamy

## 1. Project Overview: 
For this project, Baxter assembles a MEGA BLOKS pyramid from the blocks provided by the user. The flow is: 
1. Run setup file to get Baxter into position
2. Human places blocks on tray
3. Baxter detects location of a block (with right hand)
4. Goal block is displayed to user on screen
5. Baxter navigates to block, picks it up, and places it in the pyramid location (with left hand)
6. Baxter repeats steps 3 and 4 until the pyramid is complete

### Dependencies: 
- This package uses the rethink_ws "package" provided via .rosinstall file from Matt
- This package also uses a fork of "moveit_robots" 
	- uri: git@github.com:apinosky/moveit_robots.git
	- branch: melodic-devel

### ROS Packages and Libraries:
- Computer Vision:
	- AR Tags
	- OpenCV
	- OpenCV_bridge
- Trajectory Planning: 
	- MoveIt
	- Joint Trajectory Action Server

## 2. Guide:
### A. Dont forget to source: 
`source setup/baxter.bash`
- you can check the connection with `ping 10.42.0.2`
### B. Either run the launch file or steps 1-3: 
`roslaunch blocks setup_blocks.launch`

1. `rosrun blocks setup_blocks_hw`
	- Enables the robot 
	- Moves the arms above the table (`safe_arms` position)
	- Then it moves to the start position
	- Also sets up cameras 
2. `roslaunch blocks load_tower.launch`
	- Loads the block locations into the rosparam list
3. `roslaunch blocks ar_track_baxter_cam.launch`
	- Loads the computer vision node and supporting nodes
### C. Manually enable joint action trajectory server 
`rosrun baxter_interface joint_trajectory_action_server.py` 
### D. Run:
1. `roslaunch baxter_moveit_config baxter_grippers.launch`
	- Starts moveit
2. `rosrun blocks test_find_block` 
	- Moves the robot from detected block locations to hard-coded place positions

(Eventually, will be able to run `roslaunch blocks run_blocks.launch`, but not until we get rid of 'inputs' where you press enter in `test_find_block` debugging)
### E. When you're done, here are the functions to shut down safely:
1. Shut down all nodes, then run: 
2. `rosrun blocks safe_arms`
	- Run this when you're done :)
3. `rosrun baxter_tools enable_robot.py -d`
	- Disable the robot

## 3. List of all nodes and launchfiles and what they do

## 4. System Architecture
1. Setup baxter
	- Enable the robot
	- Move to start position
	- Enable camera
	- Enable computer_vision service
	- Load tower 'place' positions
2. Setup MoveIt (done separately from #1 becasue MoveIt configuration prevents moving to specific joint angles using the limb_interface. also, these steps must be run sequentially)
	1. Run Joint Trajectory Action Server
	2. Enable baxter grippers configuration
3. Run core `find_blocks` function
	- This function can be run many times without running steps 1 and 2
	- It calls the AR service, to them them know when to find the block. This controls the flow of the program 
