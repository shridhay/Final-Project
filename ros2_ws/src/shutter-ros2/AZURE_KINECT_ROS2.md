# Azure Kinect ROS2 Driver Setup and Usage

This guide provides instructions for installing, building, and using the Azure Kinect ROS2 driver with body tracking capabilities.

## Installation

### Step 1: Clone the Official ROS 2 Driver

Navigate to your ROS 2 workspace source directory and clone the official Azure Kinect ROS Driver repository using the `humble` branch:

```bash
cd ~/ros2_ws/src
git clone --branch humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
```

**Note:** The `humble` branch is the official ROS 2 port and builds with `colcon`, unlike the default `melodic` branch which is designed for ROS 1.

### Step 2: Fix Outdated Code

Due to the transition from ROS 1 to ROS 2, some code updates are required. In `Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp`, make the following changes:

1. **Update the cv_bridge include:**
   - Replace `#include <cv_bridge/cv_bridge.h>` with `#include <cv_bridge/cv_bridge.hpp>`

2. **Update the Duration constructor:**
   - Replace `rclcpp::Duration(0.25)` with `rclcpp::Duration::from_seconds(0.25)`

### Step 3: Build the Driver

Build the driver using `colcon`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```

**Note:** You may see stderr messages in the terminal output during the build process. This is normal and can be safely ignored as long as the build completes successfully.

## Running the Driver

Launch the Azure Kinect ROS2 driver with body tracking enabled:

```bash
ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true
```

## Visualization

### Viewing Raw Images in RViz2

With the launch file running, you can visualize the raw RGB images from the Kinect camera:

1. Launch RViz2:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. Add an Image display and subscribe to the topic `/rgb/image_raw` to view the camera feed.

### Inspecting Body Tracking Data

To examine the format of the body tracking data, use the following command:

```bash
ros2 topic echo /body_tracking_data
```

The body tracking data is published as a `MarkerArray` message type. Each marker represents a tracked body joint. An example marker output is shown below:

```yaml
- header:
    stamp:
      sec: 1761163056
      nanosec: 342024396
    frame_id: depth_camera_link
  ns: ''
  id: 619
  type: 2
  action: 0
  pose:
    position:
      x: -0.8145182132720947
      y: 0.7764521837234497
      z: 1.0136748552322388
    orientation:
      x: -0.12694720923900604
      y: 0.5790855884552002
      z: -0.43434086441993713
      w: 0.6781535148620605
  scale:
    x: 0.05
    y: 0.05
    z: 0.05
  color:
    r: 0.0
    g: 0.0
    b: 0.0
    a: 1.0
  lifetime:
    sec: 0
    nanosec: 250000000
  frame_locked: false
  points: []
  colors: []
  texture_resource: ''
  texture:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    format: ''
    data: []
  uv_coordinates: []
  text: ''
  mesh_resource: ''
  mesh_file:
    filename: ''
    data: []
  mesh_use_embedded_materials: false
```

Each marker contains pose information (position and orientation) for a tracked joint, along with visualization properties such as scale, color, and lifetime.

