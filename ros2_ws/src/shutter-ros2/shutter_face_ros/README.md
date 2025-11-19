# Shutter Face Package

Code to render the robot's face on its screen.

## Requirements

Shutter's face requires the [PySide2](https://pypi.org/project/PySide2/) library.
You can easily install it by running `pip` within the shutter_face package:

```console
$ roscd shutter_face_ros
$ pip3 install -r requirements.txt --user
```

If you don't have pip installed, install it with `sudo apt-get install python3-pip`.


## Quick Start

Run the following launch file to start the face:

```console
$ roslaunch shutter_face_ros simple_face.launch
```

Then, send requests for changing the gaze direction of the robot through the `/gaze/coordinate_points` topic. For example:

```console
$ rostopic pub /gaze/coordinate_points geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'head_link'
point:
  x: 0.0
  y: 0.0
  z: 1.0"
```

### Other facial expressions

Other facial expressions are available and can be accessed by publishing to the topic /gaze/expression_index. Possible expressions to request include:

'neutral' (default), 'angry', 'bored', 'determined', 'happy', 'happy2', 'sad', and 'surprised'

The extra expression of ``blink`` just closes the eyes.

You can change the facial expressions with a controller through the shutter_teleop file:

```console
$ roslaunch shutter_teleop face_controller.launch
```


## Simulation

Shutter's face can be rendered to a separate window, instead of Shutter's face screen:

```console
$ roslaunch shutter_face_ros simple_face.launch move_to_shutter_screen:=false
```

It is also possible to render Shutter's face on the simulated robot in Unity:

```console
$ roslaunch shutter_teleop face_controller.launch simulation:=true
```


### Implementation Details for Face Simulation

Shutter's face is implemented as a [PySide2](https://wiki.qt.io/PySide) application.
When rendered on a physical screen, the application is simply drawn on that screen.
To be rendered on a virtual screen in Unity, the application follows a different pipeline:

1. The PySide2 application is rendered in a virtual X display with [Xvfb](https://www.x.org/releases/X11R7.6/doc/man/man1/Xvfb.1.xhtml).
2. The Xvfb display is converted into a ROS image with [screengrab_ros](https://github.com/yale-img/screen_grab).
3. The ROS image is applied to a Unity object as a texture.

The key parameter binding this pipeline is the `DISPLAY` environment variable.
The PySide2 application must be placed on the display instantiated by Xvfb, and the `screengrab_ros` package must point to the same Xvfb display.

This pipeline is illustrated in the block diagram below:

![Pipeline for Shutter Virtual Face Rendering](../images/shutter_virtual_face_pipeline.png){w=700px align=center}
