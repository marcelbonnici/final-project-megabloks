# ME 495: Embedded Systems (Fall 2019)
# Final Project - MegaBloks
## Group Members:
- Allie Pinosky
- Luxi Huang
- Marcel Bonnici
- Senthil Palanisamy

## Capabilites:
- Locate placed blocks
- Build pyramid with 3 blocks to positions encoded in .yaml file

## Dependencies: 
- This package uses the rethink_ws "package" provided via .rosinstall file from Matt
- This package also uses a fork of "moveit_robots" 
	- uri: git@github.com:apinosky/moveit_robots.git
	- branch: melodic-devel

## General Testing Tools: 
### Camera Tools
- To view a baxter camera while the robot or simulation is running 
	- `rosrun image_view image_view image:=/cameras/left_hand_camera/image` 
	- can also sub in `right_hand_camera` or `head_camera` to view other two cameras
	- simulation lets you view all 3 cameras at once even though you can only actually view two cameras at once on the real hardware
- If cameras are acting up (or if you just turned baxter on), disable all cameras then enable right hand camera: 
	- `rosrun baxter_tools camera_control.py -c right_hand_camera`
	- `rosrun baxter_tools camera_control.py -c left_hand_camera`
	- `rosrun baxter_tools camera_control.py -c head_camera`
	- `rosrun baxter_tools camera_control.py -o right_hand_camera`
### Display Tools
- To display a smiley face:
	- `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/smiley.png`
	- `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/frowny.jpeg`
### Services
- To get a location of a model in gazebo (if running the testing simulation): 
	- `rosservice call /gazebo/get_model_state "model_name: 'block1'"` 
	- can also sub in any other model for `block1`
- To get the location of the of a block through the blocks service:
	- `rosservice call /blocks/next_pickup`
	- Returns the location of a detected block
### Miscellaneous Robot Tools
- To view coordinate transforms in the command line: 
	- `rosrun tf tf_echo /l_gripper /torso`
	- command above is of the form tf_echo <source_frame> <target_frame>
	- According to baxter.srdf.xacro, the `left_arm` group should be from the `base_link = "torso"` to the `tip_link = "left_gripper"`
	- NOTE: This value might not match RVIZ ... should explore more
- To view workspace in RVIZ, check: 
	- MotionPlanning > Planning Request > Show Workspace
- To move the arms to a safe position (away from table), 
	- `rosrun blocks safe_arms`
- To view the current arm positions in joint coordinates:
	- `rostopic echo /robot/joint_states`
	- These can be used to update the positions in `setup_blocks_hw` 

## Blocks Testing Tools:
### A. Dont forget to source: 
`source setup/baxter.bash`
- you can check the connection with `ping 10.42.0.2`
### B. Either run the launch file or steps 1-3: 
`roslaunch blocks setup_blocks.launch`

1. `rosrun blocks setup_blocks_hw`
	- Enables the robot 
	- Moves the arms above the table (`safe_arms` position)
	- Then it moves to the start position
	- Also sets up cameras (TBR, currently commented out)
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
