Notes:
- use `rosrun image_view image_view image:=/cameras/left_hand_camera/image` to view the left hand camera in simulation 
	- can also sub in `right_hand_camera` or `head_camera` to view other two cameras
	- simulation lets you view all 3 cameras at once even though you can only actually view two cameras at once on the real hardware
- to test `setup_blocks_world`, you can either run the launch file `baxter_gazebo_test.launch` or you can run the script directly from the terminal using `python setup_blocks_world`
- you can retrieve the location of a model in gazebo using `rosservice call /gazebo/get_model_state "model_name: 'block1'"` 
	- can also sub in any other model for `block`
- to manually play with `moveit`
	- `roslaunch blocks baxter_gazebo_test.launch` to run baxter_gazebo, enable robot, set up dummy environment
	- `rosrun baxter_interface joint_trajectory_action_server.py` to start joint trajectory server
	- `roslaunch baxter_moveit_config baxter_grippers.launch`
	- `python find_block` to to try running moveit from python -- work in progress

