/**
 * @file bno55_pub_node.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file bno55_pub_node. Contains a class definition for publishing Imu messages wrapping data fed by BNO055.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#include "bno055_pub_node.h"
#include "bno055_utils.h"
#include "ros/package.h"


/* global variable already declared in getbno055.h */
int verbose = 0;  // debug flag, 0 = normal, 1 = debug mode

BNO055Publisher::BNO055Publisher(ros::NodeHandle &nh, bool set_debug, int code_for_axis_remap, bool shut_down) :
quit_node(shut_down), debug_(set_debug), axis_remap_code_(code_for_axis_remap)
{
    // initialization of Vars
    std::string calib_file_path_name;
    const std::string package_path = ros::package::getPath("bno055_ros");

    // start getting ros parameters
    std::vector<std::string> bad_params;
    bad_params.reserve(10);
    nh.getParam("sensor_addr", sensor_addr_str_) ? paramFound("sensor_addr") : bad_params.emplace_back("sensor_addr");
    nh.getParam("i2cbus_sw", i2c_bus_str_) ? paramFound("i2cbus_sw") : bad_params.emplace_back("i2cbus_sw");
    nh.getParam("debug", debug_) ? paramFound("debug") : bad_params.emplace_back("debug");
    nh.getParam("opr_mode", opr_mode_) ? paramFound("opr_mode") : bad_params.emplace_back("opr_mode");
    nh.getParam("calib_file", calib_file_path_name) ? paramFound("calib_file") : bad_params.emplace_back("calib_file");
    nh.getParam("axis_remap_code", axis_remap_code_) ? paramFound("axis_remap_code") : bad_params.emplace_back("axis_remap_code");
    nh.getParam("rate", rate) ? paramFound("rate") : bad_params.emplace_back("rate");
    nh.getParam("imu_tp", imu_tp_) ? paramFound("imu_tp") : bad_params.emplace_back("imu_tp");

    if (!bad_params.empty()) {
        ROS_INFO(" Bad Parameters -> %d", int(bad_params.size()));
        std::string concat_bad_params;
        concat_bad_params.reserve(256);
        for (auto &bad_param : bad_params) concat_bad_params += (" " + bad_param);
        throw std::runtime_error("Couldn't find the parameter(s):" + concat_bad_params);
    }

    // Initialize the imu publisher with queue size equal to frame rate
    imu_publisher_ = nh.advertise<sensor_msgs::Imu>(imu_tp_, 100);

    // open the I2C bus and connect to the sensor i2c address through check
    char i2c_bus[256] = {0};
    char senaddr[256] = {0};
    strcpy(i2c_bus, i2c_bus_str_.c_str());
    strcpy(senaddr, sensor_addr_str_.c_str());
    debug_ ? verbose = 1 : verbose = 0;  // decide on whether to open debugging mode
    get_i2cbus(i2c_bus, senaddr);

    const std::string calib_file_path = package_path + "/config/" + calib_file_path_name;
    if (!verifyFusionConfig(opr_mode_, axis_remap_code_) && !initializeBNO055(opr_mode_, calib_file_path, axis_remap_code_)) {
        ROS_FATAL("\n***************************************************************\n"
                  "* Unable to verify or set BN0055 Fusion config. ABORTING NODE *\n"
                  "***************************************************************");
        quit_node = true;
    }
    ROS_INFO("IMU Config ok. IMU Ready.");

    //set our range message fixed data
    imu_msg_.header.frame_id = "imu_link";

    // set initial values for measurements covariance
    // About the meaning of values of elements, see http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
    for(int i = 0; i<9; i++)
    {
        if(i==0 || i==4 || i==8)
        {
            imu_msg_.linear_acceleration_covariance[i] = .0001;
            imu_msg_.orientation_covariance[i] = .0001;
            imu_msg_.angular_velocity_covariance[i] = .0001;
        }
        else
        {
            imu_msg_.linear_acceleration_covariance[i] = 0;
            imu_msg_.orientation_covariance[i] = 0;
            imu_msg_.angular_velocity_covariance[i] = 0;
        }
    }
}

void BNO055Publisher::update()
{
    imu_sequence_++;
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.header.seq = imu_sequence_;

    if(getImuData2Msg(imu_msg_) == 0)
    {
        imu_msg_.linear_acceleration_covariance[0] = .0001;
        if (opr_mode_ == "ndof" || opr_mode_ == "ndof_fmc") {
            imu_msg_.orientation_covariance[0] = .0001;
        } else if (opr_mode_ == "accgyro") {
            imu_msg_.orientation_covariance[0] = -1;
        }
        imu_msg_.angular_velocity_covariance[0] = .0001;
        imu_publisher_.publish(imu_msg_);
        consecutive_errors_ = 0;
        good_reads_++;
    }
    else
    {
        total_errors_++;
        ROS_INFO_STREAM("Error count = "<<consecutive_errors_++);
        imu_msg_.linear_acceleration_covariance[0] = -1;
        imu_msg_.orientation_covariance[0] = -1;
        imu_msg_.angular_velocity_covariance[0] = -1;
        imu_publisher_.publish(imu_msg_);
    }
    if(good_reads_%1000 == 0)
    {
        ROS_INFO_STREAM("good .. errors ... success % .... "<< good_reads_ <<" ... "<< total_errors_ <<" ... "<<
                        (double)good_reads_ / (total_errors_+good_reads_)*100);
    }
}

int BNO055Publisher::getImuData2Msg(sensor_msgs::Imu &imu)
{
    if (opr_mode_ == "ndof" || opr_mode_ == "ndof_fmc") {
        // declare struct variables for holding the Imu data
        struct bnogyr bnogyr_data = {0.0, 0.0, 0.0};
//        struct bnoeul bnoeul_data = {0.0, 0.0, 0.0};
        struct bnoqua bnoqua_data = {0.0, 0.0, 0.0, 0.0};
        struct bnolin bnolin_data = {0.0, 0.0, 0.0};
        // get Imu data and store them into respective struct variables
        if (get_gyr(&bnogyr_data)==-1 || get_qua(&bnoqua_data)==-1 || get_lin(&bnolin_data)==-1) {  // get_imu(&bnogyr_data, &bnoeul_data, &bnoqua_data, &bnolin_data) == -1
            ROS_WARN("**********ERROR*****RETRYING************");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (get_gyr(&bnogyr_data)==-1 || get_qua(&bnoqua_data)==-1 || get_lin(&bnolin_data)==-1) {  // get_imu(&bnogyr_data, &bnoeul_data, &bnoqua_data, &bnolin_data) == -1
                ROS_ERROR("*********FAILURE THIS ROUND***********");
                return -1;
            }
        } else {
            // TODO Currently the interface implementation adopts the hardcoded unit factor even though I already add the functionality
            //  of acquiring unit factor. Hence, I need then change get_gyr(), get_lin() in the interface implementation using that
            //  functionality. And then add the step of using that functionality in the sensor configuration phase in this code.
            // unit: rad/sec; Given this unit, both get_gyr() & get_imu()[not used] divide the gyroscope buf (in LSB) by 900 (hardcoded).
            imu.angular_velocity.x = bnogyr_data.gdata_x;
            imu.angular_velocity.y = bnogyr_data.gdata_y;
            imu.angular_velocity.z = bnogyr_data.gdata_z;
            //use quaternion instead of euler angle
            imu.orientation.w = bnoqua_data.quater_w;
            imu.orientation.x = bnoqua_data.quater_x;
            imu.orientation.y = bnoqua_data.quater_y;
            imu.orientation.z = bnoqua_data.quater_z;
            //unit: m/s^2
            imu.linear_acceleration.x = bnolin_data.linacc_x;
            imu.linear_acceleration.y = bnolin_data.linacc_y;
            imu.linear_acceleration.z = bnolin_data.linacc_z;
        }
    }
    else if (opr_mode_ == "accgyro") {
        // declare struct variables for holding the accelerometer and gyro data
        struct bnoacc bnoacc_data = {0.0, 0.0, 0.0};
        struct bnogyr bnogyr_data = {0.0, 0.0, 0.0};
        // get accelerometer and gyroscope output under amg mode (non-fusion)
        if (get_acc(&bnoacc_data) == -1 || get_gyr(&bnogyr_data) == -1) {
            ROS_WARN("**********ERROR*****RETRYING************");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (get_acc(&bnoacc_data) == -1 || get_gyr(&bnogyr_data) == -1) {
                ROS_ERROR("*********FAILURE THIS ROUND***********");
                return -1;
            }
        } else {
            // unit: rad/sec;
            imu.angular_velocity.x = bnogyr_data.gdata_x;
            imu.angular_velocity.y = bnogyr_data.gdata_y;
            imu.angular_velocity.z = bnogyr_data.gdata_z;
            //use quaternion instead of euler angle, here all zeros in non-fusion mode.
            imu.orientation.w = 0.;
            imu.orientation.x = 0.;
            imu.orientation.y = 0.;
            imu.orientation.z = 0.;
            //unit: m/s^2
            imu.linear_acceleration.x = bnoacc_data.adata_x;
            imu.linear_acceleration.y = bnoacc_data.adata_y;
            imu.linear_acceleration.y = bnoacc_data.adata_z;
        }
    }

    return 0;
}

void BNO055Publisher::closePort()
{
    close_i2cbus();
}