# Other things to try if we have time
- In `test_find_block`, update "gripper close" function to check if it successfully found a block: 
```python
    def gripper_close(self):
        self._gripper.close()
	resulting_force = self._gripper.force()
	return resulting_force
```

- In `test_find_block`, display smiley face if it places all the blocks: 
```python
    # imports
    from cv_bridge import CvBridge, CvBridgeError
    import rospkg

    # info for __init__ function
    rospack = rospkg.RosPack()
    rospack.list()
    frowny_face_image = cv2.imread(rospack.get_path('blocks')+'/images/smiley.png')

    self.bridge = CvBridge()
    self.smiley_face = self.bridge.cv2_to_imgmsg(smiley_face_image, encoding="passthrough")
    self.head_display = rospy.Publisher("/robot/xdisplay", Image, queue_size = 1)

    # to publish smiley
    self.head_display.publish(self.smiley_face)
```

- Add table to planning scene to `test_find_block` for collision avoidance:
```python
    # info for __init__ function
    scene = moveit_commander.PlanningSceneInterface()
    scene.addBox=('table', # name
                  0.0762, # size x
                  1.8288, # size y
                  0.0762, # size z
		  0.65, # x
		  -0.9, # y 
                  0, #z
		  wait = True)
```  
