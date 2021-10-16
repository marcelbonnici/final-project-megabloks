# ME 495: Embedded Systems (Fall 2019)
# Final Project - MEGA BLOKS
## Group Members:
- Allie Pinosky
- Luxi Huang
- Marcel Bonnici
- Senthil Palanisamy

### Video Demonstration
[Click Here to Watch](https://youtu.be/mz1FwBR94og)
[![The Baxter Builder](https://public.boxcloud.com/api/2.0/internal_files/574618896892/versions/609277515292/representations/jpg_paged_2048x2048/content/1.jpg?access_token=1!KvffQq8d-e7ZkZfulwTICFsV8Wn7O0wys_lvjzU3X6LNXvrKtogCAKTxwF0rk6RHgmYwADOKKqaPIUCOt23dEAvieo40OOXtfKFfqW8PRUuFuSDNmq7E9p3KwKcP83jywk2U06qwagKn7QOoSzowroYUlmLiTIRjOxhd0gapT4emxeMrDN-hPBsf73LF7ibTh9OrxwDkg0f8xwp_W3a2f91QQugA-NbF6jkys-80ei_WWoABYYfDUvzlDjNXIncG9uT1B8QG0jCYjr8KGLU2MkYowy6sYcGTI0pTu7V296a0o-CfefkBB7as6KPhWWkq2tBZ59BJFxK2fYD_2XrXDPb9MgZwqvZCEbk-wJia7pfV-dtWxEHzEouEw-qWBKOrXY-PQjWIhRIL_gzFqQpDXLk71_GLFx_Cy0A7v0maf6118cPSivVOJSlkZxQ1MbNIM8oln6dl0nduQoP79bvNUO1wZoWBegM1VwckmIeWjTCnmuORpMlELB7gKImdNJNpPVh81FNVVeKP2HMkOSzWtkB1Iui-duNqdN_tsl2k9txmfSUBTXOGo7_4uPMzmg3SAA..&shared_link=https%3A%2F%2Fnorthwestern.app.box.com%2Fs%2Fuiuzp0e9qgijy7erlp6sjc8hpcyl0yf4&box_client_name=box-content-preview&box_client_version=2.80.0)](https://youtu.be/mz1FwBR94og "Baxter")

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
### Nodes
- `computer_vision.py`
	- Uses the robot's right hand to observe the baseplate, spot a red brick and determine its position for pickup
    Detects all AR frames visible in the camera's field of view. Then pre-calibrated intrinsic camera parameters
    are used to do inverse projection to get the 3D location of a point corresponding to a pixel location.
    The 3D point corresponding to the pixel is determined in multiple AR frames and then these points
    are converted to the baxter world frame representation. Then we get muliple points in world frame that
    correspond to the same pixel but obtained using different AR frames as reference. The median of all
    those points is found out and that point is taken to be the true 3D location corresponding to a pixel in
    the image. Reason why we use multiple AR frames when one should be sufficient in theory - Sometimes AR tags
    are affected by lighting and their 3D pose with respect to the camera oscillates. Even worse sometimes, they
    are not detected. Using multiple AR tags this way increases both the roboustness and the precision of the system.
    We were able to locate points within +/- 1 cm accuracy with this process.
- `move_arm_manual`
	- Moves gripper to random manually input positions for testing
- `new_find_block`
	- Determines and executes arm paths for assembling a 3 block pyramid
- `safe_arms`
	- Moves the arms to a hard-coded position away from the build space
- `setup_blocks_hw`
	- Set up configuration for hardware testing and moves arm to 'start' position
- `setup_blocks_world`
	- Spawns dummy configuration for testing and moves arm to 'start' position
- `test_find_block`
	- Robot use pick and place position to assemble the pyramid

### Launchfiles
- `ar_track_baxter_cam.launch`
	- Uses robot with alvar tags, letting it determine the baseplate's location relative to the tag
- `baxter_gazebo_test.launch`
	 - Launches a simulated worled with a building space and Baxter robot
- `run_blocks.launch`
	- Essentially runs the `test_find_block` node.
- `setup_blocks.launch`
	- Uses hard-coded place positions, moves arm to 'start' position, and include the `ar_track_baxter_cam.launch` file
## 4. System Architecture
1. Setup Baxter
	- Enable the robot
	- Move to start position
	- Enable camera
	- Enable computer_vision service
	- Load tower 'place' positions
2. Setup MoveIt (done separately from #1 because MoveIt configuration prevents moving to specific joint angles using the limb_interface. also, these steps must be run sequentially)
	1. Run Joint Trajectory Action Server
	2. Enable Baxter grippers configuration
3. Run core `find_blocks` function
	- This function can be run many times without running steps 1 and 2
	- It calls the AR service, to them them know when to find the block. This controls the flow of the program

## 5. Lessons Learned
The ultimate lesson with "The Baxter Builder" is how ambitious locking toy blocks is for a robot. While more time coding and perfecting the physical building space could improve Baxter's performance, his place position sometimes fluctuated by an entire row of studs/circles with the baseplate static between test runs. To make the program absolutely flawless, both software and hardware modifications would be needed.
