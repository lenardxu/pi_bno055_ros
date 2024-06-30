/**
 * @file bno055_sub.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file bno055_sub. Contains the main function of running bno055_sub_node.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#include "bno055_sub_node.h"


/**
 * @brief Main function for running the bno055_sub_node.
 * @param argc Number of arguments provided to the main function.
 * @param argv Char array consisting of arguments provided to the main function.
 * @return A flag indicating the success of program or not.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "bno055_sub_node");
    ros::NodeHandle nh("~");
    ImuRecorder imu_recorder(nh);

    ros::Rate loop_rate(imu_recorder.fps);  // the same rate as publisher
    while (ros::ok())
    {
        // Process messages on global callback queue
        ros::spinOnce();
        ROS_INFO("IMU Subscriber Spinning");
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}

