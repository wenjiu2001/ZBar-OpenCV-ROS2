# ZBar-OpenCV-ROS2

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC_BY--NC--SA_4.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Ubuntu:Focal](https://img.shields.io/badge/Ubuntu-Focal-brightgreen)](https://releases.ubuntu.com/focal/)
[![ROS:Foxy](https://img.shields.io/badge/ROS-Foxy-blue)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

"zbar_opencv_ros2" is a ROS2 package designed for the straightforward scanning of barcodes.

## Requirements

- ZBar
   ```
   sudo apt-get install libzbar-dev && pip install zbar
   ```
- UsbCam
   ```
   sudo apt-get install ros-foxy-usb-cam*
   ```

## Install and Build

1. Navigating to the "src" directory within your workspace :
   ```
   cd ~/dev_ws/src
   ```
2. Clone zbar_opencv_ros2 package for github :
   ```
   git clone https://github.com/wenjiu2001/ZBar-OpenCV-ROS2.git zbar_opencv_ros2
   ```
3. Build zbar_opencv_ros2 package :
   ```
   cd ~/dev_ws && colcon build --symlink-install
   ```
4. Package environment setup :
   ```
   source ./install/setup.bash
   ```

## How to Use

Adjustments can be made by modifying the following parameters:

| Parameter name | Data Type | Detail                                                                        |
| -------------- | --------- | ----------------------------------------------------------------------------- |
| image_topic    | string    | Set the names of the image topics for subscription. <br/>default: `image_raw` |
| barcode_topic  | string    | Set the names of the barcode topics for publication. <br/>default: `barcode`  |

- Initiate usbcam and ZBar barcode recognition :
   ```
   ros2 launch zbar_opencv_ros2 zbar_opencv_ros2.launch.py
   ```
   
## References

- zbar_ros (https://wiki.ros.org/zbar_ros)
- usb_cam (https://wiki.ros.org/usb_cam)
