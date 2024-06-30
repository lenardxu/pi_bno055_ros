/**
 * @file bno55_utils.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file bno55_utils. Contains the functions' implementation as utilities.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#include "bno055_utils.h"


/* Currently the unit selection adopts the ROS specific Imu units for acc and angular rate (m/s^2, rps) and Windows Orientation*/
// Ref. to ROS specific Imu units: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html; 'Orientation' as 'Windows' means turning clockwise increases values
unit_sel_t MY_UNITS = ros_imu_win; // 0b00000010

/* For both NDOF_FMC_OFF and NDOF_FMC_ON opr mode, all the calibration status should be fully calibrated*/
int CALIB_REQUIRED_FOR_NDOF_MODE = 255;  // eg, 60 = 0b00111100 (IMU); 255 = 0b11111111 (all fully calibrated)
int CALIB_REQUIRED_FOR_NDOF_MODE_SYS = 3;
int CALIB_REQUIRED_FOR_NDOF_MODE_GYRO = 3;
int CALIB_REQUIRED_FOR_NDOF_MODE_ACC = 3;
int CALIB_REQUIRED_FOR_NDOF_MODE_MAG = 3;
int CALIB_REQUIRED_FOR_NDOF_FMC_MODE = 255;  // eg, 60 = 0b00111100 (IMU); 255 = 0b11111111 (all fully calibrated)
int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_SYS = 3;
int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_GYRO = 3;
int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_ACC = 3;
int CALIB_REQUIRED_FOR_NDOF_FMC_MODE_MAG = 3;
/* For IMU opr mode, only the full calibration status of gyro and acc are required */
int CALIB_REQUIRED_FOR_IMU_MODE       = 60;  // 60 = 0b00111100 (IMU);
int CALIB_REQUIRED_FOR_IMU_MODE_SYS   = 0;
int CALIB_REQUIRED_FOR_IMU_MODE_GYRO  = 3;
int CALIB_REQUIRED_FOR_IMU_MODE_ACC   = 3;
int CALIB_REQUIRED_FOR_IMU_MODE_MAG   = 0;
/* For AccGyro opr mode, no calibration is required */
int CALIB_REQUIRED_FOR_ACCGYRO_MODE       = 0;  // 00 = 0b00000000 (AccGyro);
int CALIB_REQUIRED_FOR_ACCGYRO_MODE_SYS   = 0;
int CALIB_REQUIRED_FOR_ACCGYRO_MODE_GYRO  = 0;
int CALIB_REQUIRED_FOR_ACCGYRO_MODE_ACC   = 0;
int CALIB_REQUIRED_FOR_ACCGYRO_MODE_MAG   = 0;

/* Flag indicating whether calibration file is loaded or not*/
bool CALIB_FILE_LOADED = false;


bool verifyFusionConfig(const std::string& desired_opr_mode, int axis_remap_code)
{
    int res = -1;  // res = function exit code: 0 = OK, -1 = Error

    // verify if the old operation mode corresponds to the desired one; if not, set to the desired mode
    int opr_mode = get_mode();
    if(opr_mode == -1) {
        if (!CALIB_FILE_LOADED) ROS_WARN("Could not read operation mode. So ENTER initializing BNO055.\n");
        else ROS_ERROR("Could not read operation mode.\n");
        return false;
    }
    opmode_t new_mode;
    if (desired_opr_mode=="config")       new_mode = config;
    else if(desired_opr_mode=="acconly")  new_mode = acconly;
    else if(desired_opr_mode=="magonly")  new_mode = magonly;
    else if(desired_opr_mode=="gyronly")  new_mode = gyronly;
    else if(desired_opr_mode=="accmag")   new_mode = accmag;
    else if(desired_opr_mode=="accgyro")  new_mode = accgyro;
    else if(desired_opr_mode=="maggyro")  new_mode = maggyro;
    else if(desired_opr_mode=="amg")      new_mode = amg;
    else if(desired_opr_mode=="imu")      new_mode = imu;
    else if(desired_opr_mode=="compass")  new_mode = compass;
    else if(desired_opr_mode=="m4g")      new_mode = m4g;
    else if(desired_opr_mode=="ndof")     new_mode = ndof;
    else if(desired_opr_mode=="ndof_fmc") new_mode = ndof_fmc;
    else {
        ROS_ERROR("Invalid operations mode %s.\n", desired_opr_mode.c_str());
        return false;
    }
    if (opr_mode != new_mode) {
        res = set_mode(new_mode);
        if (res != 0) {
            if (!CALIB_FILE_LOADED) ROS_WARN("Could not set sensor to opr mode %s of value [0x%02X]. "
                                             "So ENTER initializing BNO055.\n", desired_opr_mode.c_str(), new_mode);
            else ROS_ERROR("Could not set sensor to opr mode %s of value [0x%02X].\n",
                           desired_opr_mode.c_str(), new_mode);
            return false;
        }
        if (get_mode() == new_mode) {
            ROS_INFO("Successfully set sensor to opr mode %s of value [0x%02X].\n", desired_opr_mode.c_str(), opr_mode);
        }
    } else {
        ROS_INFO("Sensor already in opr mode %s of value [0x%02X].\n", desired_opr_mode.c_str(), opr_mode);
    }

    // verify if the units satisfy the need (Ref.: Section 4.3.60 UNIT_SEL 0x3B); If not, set to the desired one
    int unit_sel = get_unit();
    if(unit_sel == -1) {
        if (!CALIB_FILE_LOADED) ROS_WARN("Could not read unit selection. So ENTER initializing BNO055.\n");
        else ROS_ERROR("Could not read unit selection.\n");
        return false;
    }
    if ( !((unit_sel >> 0) & 0x01) &&   // check acc unit
         ((unit_sel >> 1) & 0x01) &&    // check gyr unit
         !((unit_sel >> 2) & 0x01) &&   // check eul unit
         !((unit_sel >> 4) & 0x01) &&   // check temp unit
         ((unit_sel >> 7) & 0x01) ) {  // check ORI_ANDROID_WINDOWS unit to be ANDROID
        ROS_INFO("Sensor unit of value [0x%02X] already meets our need.\n", unit_sel);
        print_unit(unit_sel);  // desired units for (accel, gyro, euler, temp, orientation) above mentioned are met
    } else {
        res = set_unit(MY_UNITS);
        if (res != 0) {
            if (!CALIB_FILE_LOADED) ROS_WARN("Could not set sensor unit of value [0x%02X]. "
                                             "So ENTER initializing BNO055.\n", MY_UNITS);
            else ROS_ERROR("Could not set sensor unit of value [0x%02X].\n", MY_UNITS);
            return false;
        }
        if (get_unit() == MY_UNITS) {
            ROS_INFO("Successfully set sensor unit of value [0x%02X].\n", MY_UNITS);
            if (!CALIB_FILE_LOADED) {
                ROS_WARN("BUT the newly set sensor unit is not compatible with existing calibration file. So ENTER"
                         "initializing BNO055.");
                return false;
            }
        }
    }

    // verify and/or set axis remap config and sign according to the given axis_remap_code
    if (axis_remap_code == 1) {  // no need to set axis remap as it is already the default : 1 (code value)
        ROS_INFO("The axis remap config and sign are already the default P1.");
    }
    else {
        if (!setAxisRemap(axis_remap_code)) {
            int old_axis_remap_code = -1;
            if (getAxisMap(old_axis_remap_code)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                if(!setAxisRemap(axis_remap_code))
                {
                    if (!CALIB_FILE_LOADED) ROS_WARN("Unable to set axis map config & sign. "
                                                     "So ENTER initializing BNO055.");
                    else ROS_ERROR("Unable to set axis map config & sign!");
                    return false;
                }
            } else {
                if (!CALIB_FILE_LOADED) ROS_WARN("Unable to set or verify axis map config & sign. "
                                                 "So ENTER initializing BNO055.");
                else ROS_ERROR("Unable to set or verify axis map config & sign!");
                return false;
            }
        }
    }

    // verify if the calibration status satisfies the operation (fusion) mode "ndof" or "ndof_fmc"
    struct bnocal bnoc = {0};
    res = get_calstatus(&bnoc);
    if(res != 0) {
        if (!CALIB_FILE_LOADED) ROS_WARN("Could not read calibration state. So ENTER initializing BNO055.\n");
        else ROS_ERROR("Could not read calibration state.\n");
        return false;
    }
    if (!CALIB_FILE_LOADED) {
        if (opr_mode == ndof) {
            if (bnoc.scal_st != CALIB_REQUIRED_FOR_NDOF_MODE_SYS || bnoc.gcal_st != CALIB_REQUIRED_FOR_NDOF_MODE_GYRO ||
                bnoc.acal_st != CALIB_REQUIRED_FOR_NDOF_MODE_ACC || bnoc.mcal_st != CALIB_REQUIRED_FOR_NDOF_MODE_MAG ) {
                ROS_WARN("NDOF_FMC_OFF fusion mode requires calib status at least of %d. "
                         "BUT the current calib status is (%d, %d, %d, %d). "
                         "So ENTER initializing BNO055.\n",
                         CALIB_REQUIRED_FOR_NDOF_MODE, bnoc.scal_st, bnoc.gcal_st, bnoc.acal_st, bnoc.mcal_st);
                return false;
            }
        }
        else if (opr_mode == ndof_fmc) {
            if (bnoc.scal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_SYS || bnoc.gcal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_GYRO ||
                bnoc.acal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_ACC || bnoc.mcal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_MAG ) {
                ROS_WARN("NDOF_FMC_ON fusion mode requires calib status at least of %d. "
                         "BUT the current calib status is (%d, %d, %d, %d). "
                         "So ENTER initializing BNO055.\n",
                         CALIB_REQUIRED_FOR_NDOF_FMC_MODE, bnoc.scal_st, bnoc.gcal_st, bnoc.acal_st, bnoc.mcal_st);
                return false;
            }
        }
        else if (opr_mode == imu) {
            if (bnoc.gcal_st < CALIB_REQUIRED_FOR_IMU_MODE_GYRO || bnoc.acal_st < CALIB_REQUIRED_FOR_IMU_MODE_ACC ) {
                ROS_WARN("IMU mode requires calib status at least of %d. "
                         "BUT the current calib status is (%d, %d, %d, %d). "
                         "So ENTER initializing BNO055.\n",
                         CALIB_REQUIRED_FOR_IMU_MODE, bnoc.scal_st, bnoc.gcal_st, bnoc.acal_st, bnoc.mcal_st);
                return false;
            }
        }
        else if (opr_mode == accgyro) {
            if (bnoc.scal_st != CALIB_REQUIRED_FOR_ACCGYRO_MODE_SYS || bnoc.gcal_st != CALIB_REQUIRED_FOR_ACCGYRO_MODE_GYRO ||
                bnoc.acal_st != CALIB_REQUIRED_FOR_ACCGYRO_MODE_ACC || bnoc.mcal_st != CALIB_REQUIRED_FOR_ACCGYRO_MODE_MAG ) {
                ROS_WARN("ACCGYRO (Non-fusion) mode requires no calibration.");
            }
        }
    }
    else {
        std::uint8_t counter = 0;
        if (opr_mode == ndof) {
            while (bnoc.scal_st != CALIB_REQUIRED_FOR_NDOF_MODE_SYS || bnoc.gcal_st != CALIB_REQUIRED_FOR_NDOF_MODE_GYRO ||
                   bnoc.acal_st != CALIB_REQUIRED_FOR_NDOF_MODE_ACC || bnoc.mcal_st != CALIB_REQUIRED_FOR_NDOF_MODE_MAG ) {
                moveSensorForCalibration(bnoc, counter);
            }
            ROS_INFO("Fully calibration achieved!");
        }
        else if (opr_mode == ndof_fmc) {
            while (bnoc.scal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_SYS || bnoc.gcal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_GYRO ||
                   bnoc.acal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_ACC || bnoc.mcal_st != CALIB_REQUIRED_FOR_NDOF_FMC_MODE_MAG ) {
                moveSensorForCalibration(bnoc, counter);
            }
            ROS_INFO("Fully calibration achieved!");
        }
        else if (opr_mode == imu) {
            while (bnoc.gcal_st < CALIB_REQUIRED_FOR_IMU_MODE_GYRO || bnoc.acal_st < CALIB_REQUIRED_FOR_IMU_MODE_ACC) {
                moveSensorForCalibration(bnoc, counter);
            }
            ROS_INFO("Fully calibration achieved!");
        }
    }

    ROS_INFO("Opr mode, units, axis remapping and calibration OK. IMU ready.");
    return true;
}

void moveSensorForCalibration(struct bnocal& bno_calib, std::uint8_t& cnt)
{
    ROS_INFO("the current calib status is (sys_calib, gyro_calib, accel_calib, mag_calib): (%d, %d, %d, %d)",
             bno_calib.scal_st, bno_calib.gcal_st, bno_calib.acal_st, bno_calib.mcal_st);
    if (cnt == 0) {
        ROS_INFO("Please move sensor for calibration accordingly now...");
        if (bno_calib.acal_st != CALIB_REQUIRED_FOR_NDOF_MODE_ACC) {
            std::this_thread::sleep_for(std::chrono::seconds(25));
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        get_calstatus(&bno_calib);
    }
    else {
        ROS_INFO("Please continue moving sensor for calibration ...");
        if (bno_calib.acal_st != CALIB_REQUIRED_FOR_NDOF_MODE_ACC) {
            std::this_thread::sleep_for(std::chrono::seconds(25));
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        get_calstatus(&bno_calib);
    }
    ++cnt;
}

bool initializeBNO055(const std::string& desired_opr_mode, const std::string& calib_file, int axis_remap_code)
{
    // load calibration data from file
    char file[256] = {0};
    strcpy(file, calib_file.c_str());
    if (load_cal(file)==-1) {
        ROS_ERROR("Unable to load the calib data from file.\n");
        return false;
    }
    CALIB_FILE_LOADED = true;
    ROS_INFO("Writing to calibration registers successful");

    // verify again especially regarding the calibration status just after loading calib data above
    if(!verifyFusionConfig(desired_opr_mode, axis_remap_code))
    {
        ROS_ERROR("\n***************************************************************\n"
                  "*       Unable to verify or set BN0055 Fusion config.         *\n"
                  "*       Verify config before trying to run node               *\n"
                  "***************************************************************");
        return false;
    }

    ROS_INFO("\nInitialization successful\n");
    return true;
}

bool setAxisRemap(int axis_remap_code)
{
    int map_config, map_sign;
    switch(axis_remap_code)
    {
        case 0:
            map_config = AXIS_REMAP_CONFIG_VAL_270_CW;
            map_sign = AXIS_REMAP_SIGN_VAL_270_CW;
            ROS_INFO("P0. Sensor mounted 270 degrees CW from default");
            break;

        case 1:
            map_config = AXIS_REMAP_CONFIG_VAL_DEFAULT;
            map_sign = AXIS_REMAP_SIGN_VAL_DEFAULT;
            ROS_INFO("P1. Default mounting");
            break;

        case 2:
            map_config = AXIS_REMAP_CONFIG_VAL_180;
            map_sign = AXIS_REMAP_SIGN_VAL_180;
            ROS_INFO("P2. Sensor mounted 180 degrees CW from default");
            break;

        case 3:
            map_config = AXIS_REMAP_CONFIG_VAL_90_CW;
            map_sign = AXIS_REMAP_SIGN_VAL_90_CW;
            ROS_INFO("P3. Sensor mounted 90 degrees CW from default");
            break;

        default:
            ROS_ERROR("Invalid axis remap code. Input correct code before using IMU");
            return false;
    }

    if (set_remap('c', map_config) == -1) {
        if (set_remap('c', map_config) == -1)  // do it again
            return false;
    }

    if (set_remap('s', map_sign) == -1) {
        if (set_remap('s', map_sign) == -1)  // do it again
            return false;
    }

    return true;
}

bool getAxisMap(int current_axis_remap_code)
{
    int map_config = get_remap('c');
    if (map_config == -1){map_config = get_remap('c');}
    int map_sign = get_remap('s');
    if (map_sign == -1){map_sign = get_remap('s');}


    if (map_config == AXIS_REMAP_CONFIG_VAL_270_CW && map_sign == AXIS_REMAP_SIGN_VAL_270_CW)
    {
        current_axis_remap_code = 0;
        return true;
    }

    if (map_config == AXIS_REMAP_CONFIG_VAL_DEFAULT && map_sign == AXIS_REMAP_SIGN_VAL_DEFAULT)
    {
        current_axis_remap_code = 1;
        return true;
    }

    if (map_config == AXIS_REMAP_CONFIG_VAL_180 && map_sign == AXIS_REMAP_SIGN_VAL_180)
    {
        current_axis_remap_code = 2;
        return true;
    }

    if (map_config == AXIS_REMAP_CONFIG_VAL_90_CW && map_sign == AXIS_REMAP_SIGN_VAL_90_CW)
    {
        current_axis_remap_code = 3;
        return true;
    }

    ROS_INFO_STREAM("raw bytes out = "<<(int)map_config <<" .. "<< (int)map_sign);
    return false;
}