/**
 * @file bno055_utils.h
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Header file bno055_utils. Contains utility variables and functions in configuring BNO055.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#ifndef BNO055_ROS_BNO055_UTILS_H
#define BNO055_ROS_BNO055_UTILS_H

#include "ros/console.h"
extern "C" {
#include "getbno055.h"
}
#include <string>
#include <chrono>
#include <thread>


/* See Section '4.3.60 UNIT_SEL 0x3B' for the values range of unit selection */
// Define unit preferences according to ROS Imu. read/write at UNIT_SEL reg
extern unit_sel_t MY_UNITS;

/* See Section '4.3.54 CALIB_STAT 0x35' for the values range of calib status */
extern int CALIB_REQUIRED_FOR_NDOF_MODE;
extern int CALIB_REQUIRED_FOR_NDOF_MODE_SYS;
extern int CALIB_REQUIRED_FOR_NDOF_MODE_GYRO;
extern int CALIB_REQUIRED_FOR_NDOF_MODE_ACC;
extern int CALIB_REQUIRED_FOR_NDOF_MODE_MAG;
extern int CALIB_REQUIRED_FOR_NDOF_FMC_MODE;
extern int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_SYS;
extern int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_GYRO;
extern int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_ACC;
extern int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_MAG;
extern int CALIB_REQUIRED_FOR_ACCGYRO_MODE;
extern int CALIB_REQUIRED_FOR_ACCGYRO_MODE_SYS;
extern int CALIB_REQUIRED_FOR_ACCGYRO_MODE_GYRO;
extern int CALIB_REQUIRED_FOR_ACCGYRO_MODE_ACC;
extern int CALIB_REQUIRED_FOR_ACCGYRO_MODE_MAG;

/* See Section '3.4 Axis remap' for the possible board placement and corresponding axes remap codes */
// Axis configuration parameters with all pointing to the same z direction; following represents P1 (default), P3, P2, P0 in order
const int AXIS_REMAP_CONFIG_VAL_DEFAULT = 0X24;
const int AXIS_REMAP_CONFIG_VAL_90_CW = 0X21;
const int AXIS_REMAP_CONFIG_VAL_180 = 0X24;
const int AXIS_REMAP_CONFIG_VAL_270_CW = 0X21;

const int AXIS_REMAP_SIGN_VAL_DEFAULT = 0X00;
const int AXIS_REMAP_SIGN_VAL_90_CW = 0X02;
const int AXIS_REMAP_SIGN_VAL_180 = 0X06;
const int AXIS_REMAP_SIGN_VAL_270_CW = 0X04;


/**
  * @brief Check and optionally set current operation mode, unit setting, axis remapping and calibration status for
  *     compatibility with publisher node and functions.
  * @param desired_opr_mode Desired operation mode given by string
  * @param axis_remap_code Axis remap code encoding both axis remap config and sign
  * @return The flag whether it passes through the verification
  */
bool verifyFusionConfig(const std::string& desired_opr_mode, int axis_remap_code);

/**
  * @brief After reset or power on, run this to set mode, unit preferences, and calibration data (including
  *     calibration status plus calibration offsets plus radii) from file, followed by verification. This requires
  *     user to move sensor for calibration.
  * @param desired_opr_mode Desired operation mode given by string
  * @param calib_file Calibration file path
  * @param axis_remap_code Axis remap code encoding both axis remap config and sign
  * @return The flag whether it passes through the verification
  */
bool initializeBNO055(const std::string& desired_opr_mode, const std::string& calib_file, int axis_remap_code);

/**
  * @brief Remap axes according to given code of axis remap axis_map_config.  3.4 - Axis remap
  * @param axis_remap_code Axis remap code encoding both axis remap config and sign
  * @return The flag whether it can set to the new remap axes code
  */
bool setAxisRemap(int axis_remap_code);

/**
  * @brief Get remap axes code encoding remap axes config and sign.
  * @param current_axis_remap_code Current axis remap code encoding both axis remap config and sign
  * @return The flag whether it can get the current remap axes code
  */
bool getAxisMap(int current_axis_remap_code);

/**
  * @brief Move sensor for calibration in the motion according to the uncalibrated sensor
  * @param bno_calib Calibration status holder
  * @param cnt Counter of calibration times
  */
void moveSensorForCalibration(struct bnocal& bno_calib, std::uint8_t& cnt);


#endif //BNO055_ROS_BNO055_UTILS_H
