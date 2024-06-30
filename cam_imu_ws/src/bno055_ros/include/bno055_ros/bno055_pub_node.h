/**
 * @file bno55_pub_node.h
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Header file bno55_pub_node. Contains a class definition for publishing Imu messages wrapping data fed by BNO055.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#ifndef IMG_SYNC_BNO055_PUB_NODE_H
#define IMG_SYNC_BNO055_PUB_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
extern "C" {
#include "getbno055.h"
}
#include <string>


/**
  * @brief Print the found ros parameter (the returned void for alignment with vector's push_back return).
  * @param param The ros parameter.
  */
inline void paramFound(const std::string& param)
{
    if (verbose) std::cout << param << std::endl;
}


/**
 * @brief ROS node class used for publishing Imu messages wrapping Imu data fed by BNO055.
 */
class BNO055Publisher
{
public:

    /**
     * Create a new BNO055Publisher object from NodeHandle.
     * @brief Constructor which incorporates the verification of configuration and optional reconfiguration of BNO055
     * besides param initialization.
     * @param nh The reference to NodeHandle.
     * @param set_debug The flag indicating whether to enter debugging mode of program (default value: false) to be
     * covered by ros param.
     * @param code_for_axis_remap The int variable indicating the axis remap mode (default value: 1) to be covered
     * by ros param.
     * @param shut_down The flag indicating whether to shut down the node beforehand (default value: false).
     */
    explicit BNO055Publisher(ros::NodeHandle& nh, bool set_debug = false, int code_for_axis_remap = 1, bool shut_down = false);

    /**
     * Destruct a BNO055Publisher object by default.
     * @brief Destructor.
     */
    ~BNO055Publisher()=default;

    /**
     * @brief Acquire fused Imu data via I2C interfacing with BNO055 and then wrap it in Imu message and
     * finally publish it.
     */
    void update();

    /**
     * @brief Acquire fused Imu data via I2C interfacing with BNO055 and then wrap it in Imu message.
     * @param imu The Imu message wrapping the fused Imu data
     */
    int getImuData2Msg(sensor_msgs::Imu &imu);

    /**
     * @brief Acquire fused Imu data via interfacing with BNO055 and then wrap it in Imu message.
     */
    void closePort();

    /**
     * Frame rate for publishing Imu message.
     */
    int rate = 100;

    /**
     * Flag indicating whether to quit the node beforehand.
     */
    bool quit_node;

private:
    /**
     * Variables hold BNO055 address for interface and I2C bus file
     */
    std::string sensor_addr_str_;
    std::string i2c_bus_str_;

    /**
     * Debugging flag of program
     */
    bool debug_;

    /**
     * Mode of sensor operation and axis remapping
     */
    std::string opr_mode_;
    int axis_remap_code_;

    /**
     * Imu message and its publisher and its sequence data member
     */
    sensor_msgs::Imu imu_msg_;
    ros::Publisher imu_publisher_;
    uint32_t imu_sequence_ = 0;

    /**
     * Statistic recordings of acquiring Imu data from BNO055
     */
    int good_reads_ = 0;
    int total_errors_ = 0;
    int consecutive_errors_ = 0;

    /**
     * Topic serving for publishing Imu.
     */
    std::string     imu_tp_;

};

#endif //IMG_SYNC_BNO055_PUB_NODE_H
