System Requirements
-------------------

- [Ubuntu 20.04](http://releases.ubuntu.com/20.04/)
- [ROS Noetic](http://wiki.ros.org/noetic)
- [Python 3.8](https://docs.python.org/release/3.8.0/)

Some of the shutter-ros packages depend on other ROS packages that can
be installed through rosdep or by downloading the git submodules already included
in this repository. Additionally, there are specific Python requirements that must be met.
Follow the installation instructions below to get started.

Expected Utilities
------------------

Install the following utilities for ROS workspaces:

+ vcstool: [installation](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool)

To simulate Shutter with a standalone executable generated with Unity, also install [gdown](https://github.com/wkentaro/gdown#installation) via pip.

Installation
------------

Download the repository, get submodules, install dependencies and build your catkin workspace:

```bash
# update pip and install expected utilities
pip install --upgrade pip
pip install --user gdown
sudo apt install python3-vcstool

# get the code
$ mkdir -pv ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/interactive-machines/shutter/shutter-ros.git

# get submodules
$ cd ~/catkin_ws/src/shutter-ros
$ git submodule update --init

# clone dependencies
$ cd ~/catkin_ws/src
$ mkdir ros-planning
$ vcs import --input shutter-ros/noetic_moveit.repos --recursive ros-planning

# update rosdep
$ cd ~/catkin_ws
$ rosdep update

# install dependencies for Shutter
$ rosdep install -y -r --ignore-src --rosdistro=noetic --from-paths src

# build workspace
$ catkin_make -DCMAKE_BUILD_TYPE=Release

# install shutter_face Python dependencies
$ source ~/catkin_ws/devel/setup.bash
$ roscd shutter_face_ros
$ pip install -r requirements.txt --user
```

Quick Start
-----------

Then, you can start the robot by running the command:

```console
$ roslaunch shutter_bringup shutter_with_face.launch simulation:=true
```

Set the optional argument `simulation:=false` to use the physical robot.
If the driver cannot find or connect to the robot, the launch file will fail.
Check the terminal for any errors.
