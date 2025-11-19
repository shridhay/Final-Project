# Shutter Description Package

3D models for visualizing and simulating the Shutter robot.

## Quick Start

Run the following launch file to publish the robot description and joint states, as well as visualize the robot in RVIZ:

```console
$ roslaunch shutter_description shutter_rviz.launch [robot_version:=2.0]
```

By default, the above launch file loads the model version 2.0, as illustrated below:

![Shutter version 2.0](../images/shutter_v0.2_rviz2.png){w=300px align=center}


## Shutter Models

This package contains descriptions for v. 0.1, 0.2, 2.0, and 3.0 of the robot.
The first version of the robot used a ZED camera instead of a RealSense camera, and had no screen face.
A picture of it is shown below:

![Shutter version 0.1](../images/shutter_rviz.png){w=300px align=center}

Versions 2.0 and greater do not use arbotix, but a ROS Control hardware interface that connects through the U2D2 USB communication converter with the Dynamixel servos in the arm.

Version 3.0 of the robot also includes a mesh and simplified collision geometry for the movable cart developed for the Shutter In-the-Wild project.

Version 4.0 of the robot replaces the original screen face assembly and RealSense camera with a 3D printed head enclosure with an embedded Logitech C920 webcam.
A picture of the updated head is shown below:

![Shutter version 4.0](../images/shutter_v4_rviz.png){w=300px align=center}


## Re-using Shutter

The URDF for Shutter can be easily added to multi-robot assemblies.
In particular, the files `shutter.v.1.xacro` and `shutter.v.2.xacro` in `package://shutter_description/urdf` can augment other robots or assemblies by specifying the parent transform.

> WARNING: The versioning for the base Shutter assemblies differs from the user-invoked versions described above!
  The base Shutter assemblies can only be version 1 (original head, equivalent to user-version 2.0) or version 2 (new head, equivalent to user-version 4.0).

The following snippet embeds Shutter (with the new head enclosure) into a larger assembly.
Shutter will be attached to the `tabletop_link` at a translational offset of 2 mm in the z-axis of the parent frame and no rotation.

```xml
<xacro:include filename="$(find shutter_description)/urdf/shutter.v.2.xacro"/>
<xacro:shutter parent="tabletop_link" xyz="0 0 0.002" rpy="0 0 0"/>
```
