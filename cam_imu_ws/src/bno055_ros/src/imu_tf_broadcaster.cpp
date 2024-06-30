/**
 * @file imu_tf_broadcaster.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Header file imu_tf_broadcaster. Contains a main function for running imu_tf_broadcaster_node.
 * @version 1.0
 * @date 2022-16-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


// declare str vars for frames in tf tree
std::string imu_parent_frame;
std::string imu_child_frame;

/**
 * @brief IMU Subscriber callback which tf broadcasts the Imu data.
 * @param msg_p The shared pointer to IMU message.
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_p);

/**
 * @brief Main function for running the imu_tf_broadcaster_node.
 * @param argc Number of arguments provided to the main function.
 * @param argv Char array consisting of arguments provided to the main function.
 * @return A flag indicating the success of program or not.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_tf_broadcaster_node");
    ros::NodeHandle nh("~");

    // TODO Don't forget to change the topic if needed
    ros::Subscriber sub = nh.subscribe("/sensors/imu", 100, &imuCallback);

    ros::spin();
    return EXIT_SUCCESS;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_p)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    imu_parent_frame = "plane";
    imu_child_frame = "imu_link";

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = imu_parent_frame;
    transformStamped.child_frame_id = imu_child_frame;
    transformStamped.transform.translation.x = 0.;
    transformStamped.transform.translation.y = 0.;
    transformStamped.transform.translation.z = 0.;
    transformStamped.transform.rotation.x = msg_p->orientation.x;
    transformStamped.transform.rotation.y = msg_p->orientation.y;
    transformStamped.transform.rotation.z = msg_p->orientation.z;
    transformStamped.transform.rotation.w = msg_p->orientation.w;

    br.sendTransform(transformStamped);
}