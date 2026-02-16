# BlueROV2 GUI Teleop

ROS2 package for controlling BlueROV2 with GUI interface, camera feed and telemetry.

## Features

- GUI control interface with clickable buttons
- Live camera feed (640x400)
- Real-time telemetry display
- Optional keyboard shortcuts
- Buoyancy compensation

## Controls

Click the buttons in the GUI to control the ROV:
- **ARM/STOP**: Safety controls
- **Arrow buttons**: Movement (forward/back/left/right)
- **UP/DOWN**: Vertical control
- **Rotate**: Yaw control
- **+/-**: Adjust gain

Keyboard shortcuts (optional):
- **P**: Arm/Disarm, **Space**: Stop
- **ZQSD**: Move, **RF**: Up/Down, **AE**: Rotate

## Install

```bash
pip3 install opencv-python pillow pynput
cd ~/ros2_ws
colcon build --packages-select bluerov2_gui_teleop
source install/setup.bash
```

## Usage

```bash
ros2 launch bluerov2_gui_teleop full_stack_launch.py
```

Or just the teleop node:

```bash
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py
```

## Parameters

- `namespace`: Vehicle namespace (default: bluerov2)
- `depth_sign`: Depth sign convention (default: -1)
- `show_gui`: Show GUI window (default: True)
- `buoyancy_compensation`: Compensation force in N (default: -15.0)

If simulation is already running, launch just the teleop:

```bash
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py
```

### Without GUI Window

To disable the telemetry GUI window:

```bash
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py show_gui:=false
```

### With Custom Namespace

If your ROV uses a different namespace:

```bash
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py namespace:=my_rov
```

### With Custom Depth Sign

If your robot uses positive Z pointing upward instead of downward:

```bash
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py depth_sign:=1
```

### Adjust Buoyancy Compensation

If the ROV drifts upward or downward when no vertical commands are given:

```bash
# Increase downward compensation if ROV floats up strongly
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py buoyancy_compensation:=-30.0

# Decrease if ROV sinks
ros2 launch bluerov2_gui_teleop gui_teleop_launch.py buoyancy_compensation:=-5.0
```

**Note**: The default `-15.0` is calibrated for BlueROV2 in Gazebo.

## Topics

**Published:**
- `/{namespace}/wrench` (geometry_msgs/Wrench) - Control commands

**Subscribed:**
- `/{namespace}/odom` (nav_msgs/Odometry) - Position and velocity feedback
- `/{namespace}/image` (sensor_msgs/Image) - Camera feed
