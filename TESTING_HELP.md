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

