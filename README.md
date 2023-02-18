# ROS2-Webcam-Image-Processing
UTwente - Advanced Software Development for Robotics - Assigment 1.1

## Building the package
Source your ROS2 SDK, then create a workspace, add this repository to its sources and build the packages.

```
$ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
$ mkdir -p ws/src
$ cd ws/src
$ git clone https://github.com/cybdann/ROS2-Webcam-Image-Processing
$ cd ..
$ colcon build
$ source install/local_setup.sh
```
## Running the executables
Run the each command in a separate terminal window (remember that the ROS2 SDK has to be sourced in every terminal window!)

### Brightnes Level Detector
```
ros2 run cam_capture brightness_level_detector
```

### Light Position Indicator
```
ros2 run cam_capture light_position_indicator
```

