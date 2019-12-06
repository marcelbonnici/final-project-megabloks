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
- To manually play with `moveit` in simulation
	1. `roslaunch blocks baxter_gazebo_test.launch` 
		- Runs baxter_gazebo, enables robot, and sets up dummy environment in gazebo
	2. `rosrun baxter_interface joint_trajectory_action_server.py` 
		- Starts joint trajectory server
	3. `roslaunch baxter_moveit_config baxter_grippers.launch`
		- Starts moveit
	4. `rosrun blocks find_block_manual` 
		- Moves the robot arm through a random sequence of three trajectories 
		- These can be modified by changing the positions and orientions in the `get_location` sub function

- to play with `moveit` on the actual robot
	- replace step 1 with: `rosrun blocks setup_blocks_hw`
	- steps 2-4 are the same for testing on the real robot

- to display a smiley face: 
	- `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/smiley.png`
        - `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/frowny.jpeg`

