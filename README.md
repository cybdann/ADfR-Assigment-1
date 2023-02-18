# ROS2-Webcam-Image-Processing
UTwente - Advanced Software Development for Robotics - Assigment 1.1

## Description of the package
The package contaiins two excecutables:
1. Brightnes Level Detector - Creates a node that determines the average brightness of the image and, using some threshold,
sends on a new topic if a the light is turned on (it is light) or off (it is dark).
2. Light Position Indicator - Creates a node that, given a camera input, outputs the position of a bright light (if there is any)
in pixel coordinates.

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
Run the each command in a separate terminal window. Remember that the ROS2 SDK has to be sourced in every terminal window!
These packages can be tested with the `image_tools` package provided by ROS.

### Brightnes Level Detector
```
ros2 run cam_capture brightness_level_detector
```

### Light Position Indicator
```
ros2 run cam_capture light_position_indicator
```
