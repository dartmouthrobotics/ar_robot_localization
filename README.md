## BlueROV Visual Pose Nodelet

Currently only working with one AR marker.

## To Do Next
* Extend to multiple AR tags.

## Check Coding Style
``` console
$ catkin_make roslint_bluerov_visual_pose
```

## Run
``` console
$ roslaunch bluerov_visual_pose test.launch
```

Robot pose data will be published to the `/visual_pose/pose` rostopic.
