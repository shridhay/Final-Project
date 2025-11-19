# Shutter Bringup Package

Set of scripts and configuration files to start the robot.

## Quick Start

To start publishing the robot model and run the ROS Control driver, execute:

```console
$ roslaunch shutter_bringup shutter.launch
```

The launch file will then publish
the robot description (URDF) into the /robot_description parameter in
the [ROS Parameter server](http://wiki.ros.org/Parameter%20Server),
and run two nodes:

- /[robot_state_publisher](http://wiki.ros.org/robot_state_publisher): Publishes the state of the robot to /tf based on its joint angles and its kinematic tree (from the URDF in the parameter server).

- /[shutter_position_interface](https://shutter-ros.readthedocs.io/en/latest/packages/shutter_hardware_interface.html): Shutter hardware interface, which communicates between the robot servos and generic controllers defined in the [ROS Control](http://wiki.ros.org/ros_controllers) package.

To send commands to the servos, use the `/joint_group_controller/command` topic and send the servo angles in radians in a `Float64MultiArray` message.
For example, using the command-line (ROS 2):

```console
$ ros2 topic pub --once /joint_group_controller/command std_msgs/msg/Float64MultiArray "{data: [0.0, 1.54, 1.54, 0.0]}"
```

The joint order is: `[joint_1, joint_2, joint_3, joint_4]` in radians.

To retrieve the status of the servos, check the `/joint_states` topic.

Note that the controller may need to be enabled automatically, as described [Manual Controller Management](#manual-controller-management) subsection.

If you want to run the robot driver, URDF model, and the face all at once, then
launch the `shutter_with_face.launch` file instead:

```console
$ roslaunch shutter_bringup shutter_with_face.launch
```

**NOTE:** For Shutter version 2.x and 3.x, make sure that the robot screen is set to a resolution of 800x480 pixels.
If that is not the case, then follow
[these instructions](https://gitlab.com/interactive-machines/shutter/shutter-ros/-/wikis/screen_issues)
to set the resolution properly.

### Manual Controller Management

By default, passing the `driver:=ros_control` argument to the main `shutter.launch` launch file loads two low-level controllers for the position interface: a JointGroupPositionController and a JointTrajectoryController.
The more advanced packages for motion control (MoveIt, `shutter_servo` and `shutter_teleop`) automatically initialize and start the appropriate low-level controller.
However, using `shutter_bringup` alone will NOT start the controller automatically.

To manually start the JointGroupPositionController, send a service request to the `/controller_manager` node:

```console
$ rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_controller']
 strictness: 1"
ok: True  # expected output
```

Note that you can use tab completion to help construct the service request message.


## Shutter Camera (version 4.x)

Shutter version 4.x has a Logitech C920 webcam instead of a RealSense camera.
To use the webcam, invoke or include the `shutter_webcam.launch` launchfile:

```console
$ roslaunch shutter_bringup shutter_webcam.launch [view_image:=false]
```

The webcam is identified by its [v4l driver](https://www.linuxtv.org/wiki/index.php/V4l-utils).
Detection of the correct webcam ID should occur automatically as part of the package build, assuming that the webcam is connected via USB.
If the webcam changes, or the detected ID is incorrect, the detection can be invoked manually:

```console
$ roscd shutter_bringup/config
$ ./infer_webcam_id.py
```

The detected ID is written as the environment variable `SHUTTER_WEBCAM_ID` to `shutter_bringup/config/env-hooks/webcam_id.bash`.
This environment variable is exported with a [catkin environment hook](http://docs.ros.org/en/noetic/api/catkin/html/dev_guide/generated_cmake_api.html#catkin_add_env_hooks), whenever the workspace `devel/setup.bash` is sourced.

Note that if the detection script fails, the webcam ID environment variable in `shutter_bringup/config/env-hooks/webcam_id.bash` can be edited manually, but may be overwritten by subsequent workspace builds.

### Calibration

Camera intrinsic parameters can be calibrated with the [camera_calibration](https://wiki.ros.org/camera_calibration) package.
In particular, follow the [tutorial for monocular calibration](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

For improved accuracy, it is recommended to calibrate with a ChArUco board, rather than the default chessboard pattern.
ChArUco boards can be generated at this website: [https://calib.io/pages/camera-calibration-pattern-generator](https://calib.io/pages/camera-calibration-pattern-generator).

An adapted command to calibrate with ChArUco:

```console
$ rosrun camera_calibration cameracalibrator.py --pattern charuco --size 9x7 --square 0.028 --charuco_marker_size 0.021 --aruco_dict 4x4_250 image:=/shutter_webcam/image_raw camera:=/shutter_webcam
```


## Simulation

Shutter has a basic simulation built with the Unity game engine.
The simulation can be launched with the `shutter_sim.launch` launchfile:

```console
$ roslaunch shutter_bringup shutter_sim.launch [headless:=false]
```

The `headless` argument specifies whether the simulation should create a display window.
Running headless is useful for scenarios that do not require interaction, such as evaluating a motion control policy or synthesizing joint state data in simulation.

The simulation will be automatically downloaded and installed.
By default, an existing installation will not be  overwritten by a new version of the simulation.
Instead, the simulation manager will throw an error message.
Deleting the existing simulation installation and re-launching should install the new version and resolve the error.

Camera capabilities for Shutter (e.g., the webcam for version 4.x) are currently not supported in simulation.
