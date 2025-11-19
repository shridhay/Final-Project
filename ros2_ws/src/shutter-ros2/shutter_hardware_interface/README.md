# Shutter Hardware Interface Package

This package defines a hardware interface for Shutter's dynamixel servos.
Specifically, it provides a [Position Joint Interface](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/classhardware__interface_1_1PositionJointInterface.html) and a [Velocity Joint Interface](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/classhardware__interface_1_1VelocityJointInterface.html) that can be used by compatible controllers provided by [ROS Control](http://wiki.ros.org/ros_control).


## Dependencies

This hardware interface requires the installation of the following packages through apt:

```
ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-dynamixel-workbench
```

Additionally, the primary use case for shutter_hardware_interface is executing motion planned by MoveIt, which requires additional setup.


## Usage

Using the hardware interface with ROS Control and MoveIt requires three steps, which are enumerated below for the Position Joint Interface implementation:

1. Start the Shutter driver node via `shutter_position_interface.launch`.
This launch file specifies necessary parameters for the USB port and baud rate to communicate with the Dynamixel servos, then starts the Shutter driver node.
2. Load the controller to use with this hardware interface.
This step entails parsing a YAML configuration for the controller (specifying the joint names and optional parameters), then starting a controller manager spawner node to load the controller.
The position joint interface should read from `config/position_controllers.yaml`.
3. Set the MoveIt controller manager to `ros_control`.
This parameter is used by MoveIt's trajectory execution manager to identify the controller(s) to send commands.
Note that this step should already be configured for the launch files located in this repository.

Because both ROS Control and MoveIt have several related components, the entire planning and execution pipeline is most easily validated by adapting an existing launch file, rather than starting individual ROS nodes.
In the `shutter_moveit_config` package, `demo.launch` provides a comprehensive starting point for testing most MoveIt functionality.

Note that when using the controller_manager node `spawner` to load and start a controller, the robot might automatically move to its initial pose, (all joint positions set to 0 radians). To prevent this initial movement, add the flag `--stopped` to the `spawner` invocation.
For example, in `shutter_hardware_interface/launch/shutter_position_interface.launch`:

```xml
<node name="controller_spawner" pkg="controller_manager" type="spawner" args="--stopped follow_trajectory_controller joint_group_controller "/>
```

See [documentation for controller_manager spawner](http://wiki.ros.org/controller_manager?distro=noetic#spawner)


## Velocity Joint Interface

The Velocity Joint Interface is mostly intended for use with the `shutter_teleop` package, and is still experimental.
In particular, there are two outstanding issues to be careful about:
1. Timeout: The generic velocity controller does not include a timeout on the incoming commands. Hence, if the process generating commands crashes (e.g., a segfault), the old command will continue to execute blindly. A `soft_timeout` node tries to detect and repair stale commands, but the node hasn't been extensively tested.
2. PID tuning: The velocity controller is implemented with a PID controller for each joint. The gains currently defined in `shutter_hardware_interface/config/velocity_controllers.yaml` were set naively, and might not have the most performant values.


## ROS Control and Hardware Interfaces
Shutter's hardware interface was motivated by the [ROS Control framework](http://wiki.ros.org/ros_control).
ROS Control comprises several packages that standardize controller use across many different robots.
ROS Control is particularly useful for Shutter in order to use [MoveIt](http://wiki.ros.org/MoveIt), a widely-used motion planning framework.

ROS Control and its generic controllers require a hardware interface that reads and writes commands to the actual robot motors.
MoveIt expects a `FollowJointTrajectory` action service, which is provided in ROS Control by a [joint trajectory controller](http://wiki.ros.org/joint_trajectory_controller), which itself requires a hardware interface of type joint position, joint velocity, or joint effort.

The joint position hardware interface is closest to the provided [dynamixel_workbench_controller](http://wiki.ros.org/dynamixel_workbench_controllers).
The dynamixel workbench controller implementation was adapted to provide a hardware interface compatible with ROS Control.

For details on how these components interact, check out the [MoveIt Concepts overview](https://moveit.ros.org/documentation/concepts/) and the [ROS Control wiki page](http://wiki.ros.org/ros_control)

The following diagram illustrates the various components for motion planning and execution.
Note that the cells correspond to semantic components, and not to ROS nodes or topics.
The hardware interface package is highlighted in orange.

![Shutter MoveIt and ROS Control components](../images/shutter_moveit_ros_control.png){w=700px align=center}
