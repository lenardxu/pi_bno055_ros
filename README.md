# pi_bno055_ros
A C Driver program driving bno055 using I2C interface on a Raspberry Pi (4 model B) and a ROS wrapper upon it.

# Introduction
## C Driver Program of BNO055 Using I2C Interface
This project takes the reference to [this original implementation](https://github.com/fm4dd/pi-bno055). My project extends and refactors it for two purposes: 
- (1) build a library for subsequent projects; 
- (2) interface with Adafruit BNO055 for configurations and testings. 

Hereby, sincerely thank [Frank M.](https://github.com/fm4dd) for his sharing in terms of a comprehensive and convenient I2C interface with BNO055.

## ROS Wrapper upon C Driver Program
This ROS package is built upon the CMake project `pi-bno055`, which needs to be first built, installed and packaged. 
Check [pi_bno055](https://github.com/lenardxu/HandTracker/tree/main/utils/pi_bno055) for details. Additionally, please ensure that you place the dependency `pi-bno055` parallel to
`cam_imu_ws` under the same directory as it is, as this ROS package is built upon such relative file locations.

# Installation and Usage
## C Driver Program of BNO055 Using I2C Interface
Please check [the specific readme](./pi_bno055/README.md) in this repo.

## ROS Wrapper upon C Driver Program
Please check [the specific readme](./cam_imu_ws/src/bno055_ros/README.md) in this repo.



