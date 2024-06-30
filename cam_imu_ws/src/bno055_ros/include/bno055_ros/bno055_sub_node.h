/**
 * @file bno055_sub_node.h
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Header file bno055_sub_node. Contains a class definition for recording messages fed by BNO055.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#ifndef BNO055_ROS_BNO055_SUB_NODE_H
#define BNO055_ROS_BNO055_SUB_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"

#include <string>


/**
 * @brief ROS node class used for recording Imu message fed by BNO055.
 */
class ImuRecorder {
public:
    /**
     * Create a new ImuRecorder object from NodeHandle.
     * @brief Constructor.
     * @param nh The reference to NodeHandle.
     */
    explicit ImuRecorder(ros::NodeHandle& nh);

    /**
     * Destruct a ImgImuRecorder object by default.
     * @brief Destructor.
     */
    ~ImuRecorder()=default;

    /**
     * @brief IMU Subscriber callback.
     * @param msg_p The shared pointer to IMU message.
     * @param ostrm The ofstream object for writing imu stream.
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_p, std::ofstream& ostrm);

    /**
     * Frame rate for recording (/writing).
     */
    double fps = 100;

private:
    /**
     * @brief Initialize all relevant config parameters using ros parameters.
     * @param nh The NodeHandle.
     * @return A flag indicating the success of initializing process or not.
     */
    int initCfgs_(ros::NodeHandle& nh);

    /**
     * ROS subscribers for subscribing Imu.
     */
    ros::Subscriber imu_sub_;

    /**
     * Topic serving for recording Imu.
     */
    std::string     imu_tp_;

    /**
     * Output file paths to store Imu stream..
     */
    std::string     op_imu_;
};


#endif //BNO055_ROS_BNO055_SUB_NODE_H
