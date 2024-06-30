/**
 * @file plot_imu_data.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file plot_imu_data. Contains the main function of running plot_imu_data_node.
 * @version 1.0
 * @date 2022-29-08
 * @copyright Copyright (c) 2021 LICENSE)
 */


#include "plot_imu_data_node.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plot_imu_data_node");
    ros::NodeHandle nh("~");

    /* Instantiate ImuDataPlotter */
    ImuDataPlotter imu_plotter(nh);

    /* Set logger level to debug on demand and initialize the IO paths */
    if(imu_plotter.getROSDebugMode()) {
        setLoggerDebugLevel();
    }
    const std::string input_imu_data_path = packagePath  + "/data/" + imu_plotter.ip_imu;
    const std::string output_acc_plot_path = packagePath + "/data/plots/" + imu_plotter.op_acc;
    const std::string output_gyr_plot_path = packagePath + "/data/plots/" + imu_plotter.op_gyr;
    const std::string output_eul_plot_path = packagePath + "/data/plots/" + imu_plotter.op_eul;

    /* Select mode of plotting content */
    ImuDataPlotter::PlotMode plot_mode = imu_plotter.selectPlotMode();
    ROS_DEBUG_STREAM("Main Func: Plot mode is " << plot_mode);

    /* Load the Imu data given the Input path according to mode of plotting content */
    imu_plotter.loadImuData(input_imu_data_path, plot_mode);

    /* Plot data and/or save according to mode of plotting content */
    imu_plotter.plotData(plot_mode, output_acc_plot_path, output_gyr_plot_path, output_eul_plot_path);

    return EXIT_SUCCESS;
}