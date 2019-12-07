Dependencies: 
- This package uses the rethink_ws "package" provided via .rosinstall file from Matt
- This package also uses a fork of "moveit_robots" 
	- uri: git@github.com:apinosky/moveit_robots.git
	- branch: melodic-devel

General Testing Tools: 
- To view a baxter camera while the robot or simulation is running 
	- `rosrun image_view image_view image:=/cameras/left_hand_camera/image` 
	- can also sub in `right_hand_camera` or `head_camera` to view other two cameras
	- simulation lets you view all 3 cameras at once even though you can only actually view two cameras at once on the real hardware
- To get a location of a model in gazebo: 
	- `rosservice call /gazebo/get_model_state "model_name: 'block1'"` 
	- can also sub in any other model for `block1`
- To view coordinate transforms in the command line: 
        - `rosrun tf tf_echo /l_gripper /torso`
        - command above is of the form tf_echo <source_frame> <target_frame>
	- According to baxter.srdf.xacro, the `left_arm` group should be from the `base_link = "torso"` to the `tip_link = "left_gripper"`
	- NOTE: This value might not match RVIZ ... should explore more
- To view workspace in RVIZ, check: 
	- MotionPlanning > Planning Request > Show Workspace
- To move the arms to a safe position (away from table) before shutdown, 
	- `rosrun blocks safe_arms`

Blocks Testing Tools:
- To play with `moveit` for the actual robot:
	1. `source setup/setup/baxter.bash`
		- connect to the robot
		- you can check the connection with `ping 10.42.0.2`
	2. `rosrun baxter_tools enable_robot.py -e`
		- enable the robot
	3. `rosrun blocks safe_arms`
		- this moves the arms above the table before executing the setup file
		- you should also run this before shutting down to make sure the arms don't hit the table when you disable the robot
	4. `rosrun blocks setup_blocks_hw` 
	5. `rosrun baxter_interface joint_trajectory_action_server.py` 
		- Starts joint trajectory server
	6. `roslaunch baxter_moveit_config baxter_grippers.launch`
		- Starts moveit
	7. `roslaunch blocks load_tower.launch`
		- loads the block locations into the rosparam list
	8. `roslaunch blocks ar_track_baxter_cam.launch`
		- loads the computer vision node and supporting nodes
	9. `rosrun blocks find_block_manual` 
		- Moves the robot arm through a random sequence of three trajectories 
		- These can be modified by changing the positions and orientions in the `get_location` sub function
        10. rosrun blocks safe_arms`
		- Run this when you're done :)
        11. `rosrun baxter_tools enable_robot.py -d`
		- disable the robot


- to play with `moveit` in simulation robot
	- replace step 1 with: `roslaunch blocks baxter_gazebo_test.launch`
	- steps 2-4 are the same for testing on the real robot

- to display a smiley face: 
	- `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/smiley.png`
        - `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/frowny.jpeg`

