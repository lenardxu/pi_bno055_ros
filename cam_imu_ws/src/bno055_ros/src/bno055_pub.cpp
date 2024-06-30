/**
 * @file bno55_pub.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file bno55_pub. Contains the main function of running bno55_pub_node.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#include "bno055_pub_node.h"


/**
 * @brief Main function for running the bno55_pub_node.
 * @param argc Number of arguments provided to the main function.
 * @param argv Char array consisting of arguments provided to the main function.
 * @return A flag indicating the success of program or not.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle imu_node_handler("~");
    BNO055Publisher imu(imu_node_handler);

    ros::Rate loop_rate(imu.rate);
    while (ros::ok() && !imu.quit_node)  // also checks whether to quit beforehand
    {
        imu.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    /* closing node */
    imu.closePort();
    return EXIT_SUCCESS;
}