/**
 * @file plot_imu_data_node.h
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Header file plot_imu_data_node. Contains a class definition for plotting Imu data.
 * @version 1.0
 * @date 2022-29-08
 * @copyright Copyright (c) 2021 LICENSE)
 */

#ifndef BNO055_ROS_PLOT_IMU_DATA_NODE_H
#define BNO055_ROS_PLOT_IMU_DATA_NODE_H


#include "ros/console.h"
#include "ros/ros.h"
#include <ros/package.h>

#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <matplot/matplot.h>


const std::string packagePath = ros::package::getPath("bno055_ros");

/**
  * @brief Print the found ros parameter (the returned void for alignment with vector's push_back return).
  * @param param The ros parameter.
  */
inline void paramFound(const std::string& param)
{
    ROS_INFO_STREAM(param << " is found") ;
}

inline void setLoggerDebugLevel()
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

class ImuDataPlotter
{
public:
    enum PlotMode {
        plot_acc,
        plot_gyr,
        plot_eul,
        plot_all
    };

public:
    explicit ImuDataPlotter(ros::NodeHandle& nh, bool save_plot=true, bool debug_mode=false, int plot_mode_val=0);

    [[nodiscard]] PlotMode selectPlotMode() const;  //TODO to learn about this meaning

    /**
     * Load imu data from the given file.
     * @param input_path File name of input Imu data
     * @param plot_mode_sel Mode of plotting content
     */
    void loadImuData(const std::string& input_path, PlotMode plot_mode_sel);

    /**
     * Plot Imu data on demand.
     */
    void plotData(PlotMode plot_mode_sel,
                  const std::string& output_acc_plot_path = packagePath+"/data/plots/acc.jpg",
                  const std::string& output_gyr_plot_path = packagePath+"/data/plots/gyr.jpg",
                  const std::string& output_eul_plot_path = packagePath+"/data/plots/eul.jpg");

    /**
     * @brief Get the debug mode set from corresponding ros parameter server.
     * @return The debug mode
     */
    [[nodiscard]] bool getROSDebugMode() const { return debug_; }

private:
    /**
     * Plot linear accelerations.
     */
    void plotLinAccel_(const std::string& output_acc_plot_path);

    /**
     * Plot angular rates.
     */
    void plotAngRate_(const std::string& output_gyr_plot_path);

    /**
     * Plot (absolute) euler angles.
     */
    void plotEulAngle_(const std::string& output_eul_plot_path);

public:
    /**
     * File stream object that deals with input Imu data.
     */
    std::fstream    file_;

    /**
     * Input relative file path to package root dir for storing Imu messages.
     */
    std::string     ip_imu;

    /**
     * Output relative file path to package root dir for storing Imu messages.
     */
    std::string     op_acc;
    std::string     op_gyr;
    std::string     op_eul;

private:
    /**
     * Whether debugging info is printed out.
     */
    bool debug_;

    /**
     * Whether saving the resulting plot.
     */
    bool save_;

    /**
     * Plot content selection
     */
    int plot_sel_;

    /**
     * Vectors holding the history of time stamps, linear accelerations, angular rates and euler angles.
     */
    std::vector<double> ts_vecp_;
    std::vector<double> acc_x_, acc_y_, acc_z_;
    std::vector<double> gyr_x_, gyr_y_, gyr_z_;
    std::vector<double> eul_x_, eul_y_, eul_z_;  // roll, pitch, yaw

};

#endif //BNO055_ROS_PLOT_IMU_DATA_NODE_H
