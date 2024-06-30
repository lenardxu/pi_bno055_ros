/**
 * @file bno055_sub_node.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file bno055_sub_node. Contains a class definition for recording Imu message fed by BNO055
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#include "bno055_sub_node.h"
#include "boost/filesystem.hpp"
#include <ros/package.h>


ImuRecorder::ImuRecorder(ros::NodeHandle& nh)
{
    // assign the configs with ros parameters' values
    if (initCfgs_(nh) == 0)
        ROS_INFO("All ros parameters read successfully!");
    else
        ROS_ERROR("Ros parameters read failed, please refer to details above");

    // Declare the output object in static
    static std::ofstream   imuFilestream;     // Keep imuFilestream's life time for writing till end of node life

    imuFilestream.open(op_imu_, std::ios_base::app);
    auto boundImuCallback = boost::bind(&ImuRecorder::imuCallback, this, _1, boost::ref(imuFilestream));
    imu_sub_              = nh.subscribe<sensor_msgs::Imu>(imu_tp_, 200, boundImuCallback);
}

void ImuRecorder::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_p, std::ofstream& ostrm)
{
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              imu_p->linear_acceleration.x, imu_p->linear_acceleration.y, imu_p->linear_acceleration.z,
              imu_p->angular_velocity.x, imu_p->angular_velocity.y, imu_p->angular_velocity.z,
              imu_p->orientation.x, imu_p->orientation.y, imu_p->orientation.z, imu_p->orientation.w);
    std::vector<uint32_t> stamp_vec;
    stamp_vec.emplace_back(imu_p->header.stamp.sec);
    stamp_vec.emplace_back(imu_p->header.stamp.nsec);
    std::vector<double> imu_vec;
    imu_vec.emplace_back(imu_p->linear_acceleration.x);
    imu_vec.emplace_back(imu_p->linear_acceleration.y);
    imu_vec.emplace_back(imu_p->linear_acceleration.z);
    imu_vec.emplace_back(imu_p->angular_velocity.x);
    imu_vec.emplace_back(imu_p->angular_velocity.y);
    imu_vec.emplace_back(imu_p->angular_velocity.z);
    imu_vec.emplace_back(imu_p->orientation.x);
    imu_vec.emplace_back(imu_p->orientation.y);
    imu_vec.emplace_back(imu_p->orientation.z);
    imu_vec.emplace_back(imu_p->orientation.w);
    auto imu_meas = std::make_pair(stamp_vec, imu_vec);

    if (ostrm)
        ostrm << imu_meas.first[0] << "," << imu_meas.first[1]  << "," << imu_meas.second[0] << "," <<
                imu_meas.second[1] << "," << imu_meas.second[2] << "," << imu_meas.second[3] << "," <<
                imu_meas.second[4] << "," << imu_meas.second[5] << "," << imu_meas.second[6] << "," <<
                imu_meas.second[7] << "," << imu_meas.second[8] << "," << imu_meas.second[9] << "\n";
    else
        ROS_ERROR("Couldn't open the csv file for writing IMU");
}

int ImuRecorder::initCfgs_(ros::NodeHandle& nh)
{
    // get ros parameter for imu topic
    if (nh.getParam("imu_tp", imu_tp_))
        ROS_INFO("Got param for imu topic: %s", imu_tp_.c_str());
    else {
        ROS_ERROR("Failed to get param for imu topic");
        return -1;
    }
    // get ros parameter for frequency of writing frames
    if (nh.getParam("frame_freq", fps))
        ROS_INFO("Got param for image writing freq: %f", fps);
    else {
        ROS_ERROR("Failed to get param for image writing freq");
        return -1;
    }
    // declare and initialize the destination path with the package path given the package's name "bno055_ros"
    boost::filesystem::path tmp_pkg_pth = ros::package::getPath("bno055_ros");
    boost::filesystem::path imu_pth(tmp_pkg_pth);
    std::string imu_rel_part;
    // get ros parameter for imu data file path to be written
    if (nh.getParam("op_imu", imu_rel_part)){
        boost::filesystem::path imu_rel_pth =  imu_rel_part.c_str();
        imu_pth /= imu_rel_pth;
        ROS_INFO("Got param for output file in csv for imu data: %s", boost::filesystem::absolute(imu_pth).c_str());
        op_imu_ = imu_pth.string();
    }
    else {
        ROS_ERROR("Failed to get param for output file in csv for imu data");
        return -1;
    }
    return 0;
}