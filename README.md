## AR Robot Localization Nodelet

Template for retrieving and publishing robot global pose with respect to AR tags in the scene.

## To Do
* Check if it can subscribe to AR tag info
* Check if it can publish (empty) PoseStamped messages
* Publish robot's pose with respect to AR tag info
* Handle condition when AR tag provides no information (loss of sight)
* Handle and improve localization with multiple AR tags

## Setup
* Clone repository and build:
  ```console
  $ git clone https://github.com/dartmouthrobotics/ar_robot_localization.git
  $ catkin_make
  $ source devel/setup.bash
  ```

## Run (Testing)
``` console
$ roslaunch ar_robot_localization test.launch
```

Robot pose data will be published to the `/ar_robot_loc/pose` rostopic.
