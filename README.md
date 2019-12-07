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
- To display a smiley face:
        - `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/smiley.png`
        - `rosrun baxter_examples xdisplay_image.py --file=`rospack find blocks`/images/frowny.jpeg`

## Blocks Testing Tools:
### To play with `moveit` for the actual robot:
#### A. Dont forget to source: `source setup/setup/baxter.bash`
you can check the connection with `ping 10.42.0.2`
#### B. Either run `roslaunch blocks setup_blocks.launch` or the following:
1. `rosrun blocks setup_blocks_hw`
	- this moves the arms above the table before executing the setup file
	- then it moves to a start position
	- also sets up cameras 
2. `roslaunch blocks load_tower.launch`
	- loads the block locations into the rosparam list
3. `roslaunch blocks ar_track_baxter_cam.launch`
	- loads the computer vision node and supporting nodes
#### C. Manually enable joint action trajectory server `rosrun baxter_interface joint_trajectory_action_server.py` 
#### D. Either run `roslaunch blocks run_blocks.launch` or the following:
1. `roslaunch baxter_moveit_config baxter_grippers.launch`
	- Starts moveit
3. `rosrun blocks test_find_block` 
	- Moves the robot from detected block locations to hard-coded place positions
#### E. When you're done, here are the functions to shut down safely:
1. rosrun blocks safe_arms`
	- Run this when you're done :)
2. `rosrun baxter_tools enable_robot.py -d`
	- disable the robot
