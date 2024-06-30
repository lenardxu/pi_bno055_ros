/**
 * @file plot_imu_data_node.cpp
 * @authors Rukang Xu (lenardxu2017@gmail.com)
 * @brief Source file plot_imu_data_node. Contains a class definition for running plot_imu_data_node.
 * @version 1.0
 * @date 2022-29-08
 * @copyright Copyright (c) 2021 LICENSE)
 */


#include "plot_imu_data_node.h"
#include <tf/tf.h>
#include "cmath"


ImuDataPlotter::ImuDataPlotter(ros::NodeHandle &nh, bool save_plot, bool debug_mode, int plot_mode_val)
:debug_(debug_mode), save_(save_plot), plot_sel_(plot_mode_val)
{
    // start getting ros parameters
    std::vector<std::string> bad_params;
    bad_params.reserve(8);
    nh.getParam("debug", debug_) ? paramFound("debug") : bad_params.push_back("debug");
    nh.getParam("ip_imu", ip_imu) ? paramFound("ip_imu") : bad_params.push_back("ip_imu");
    nh.getParam("save", save_) ? paramFound("save") : bad_params.push_back("save");
    nh.getParam("op_acc", op_acc) ? paramFound("op_acc") : bad_params.push_back("op_acc");
    nh.getParam("op_gyr", op_gyr) ? paramFound("op_gyr") : bad_params.push_back("op_gyr");
    nh.getParam("op_eul", op_eul) ? paramFound("op_eul") : bad_params.push_back("op_eul");
    nh.getParam("plot_sel", plot_sel_) ? paramFound("plot_sel") : bad_params.push_back("plot_sel");

    if (!bad_params.empty()) {
        ROS_WARN(" Bad Parameters -> %d", int(bad_params.size()));
        std::string concat_bad_params;
        concat_bad_params.reserve(256);
        for (auto &bad_param : bad_params) concat_bad_params += (" " + bad_param);
        throw std::runtime_error("Couldn't find the parameter(s):" + concat_bad_params);
    }
}

ImuDataPlotter::PlotMode ImuDataPlotter::selectPlotMode() const
{
    PlotMode plot_mode_sel;
    switch(plot_sel_) {
        case 0:
            plot_mode_sel = plot_acc;
            ROS_INFO("Plotting linear acceleration begins...");
            break;
        case 1:
            plot_mode_sel = plot_gyr;
            ROS_INFO("Plotting angular rate begins...");
            break;
        case 2:
            plot_mode_sel = plot_eul;
            ROS_INFO("Plotting euler angle begins...");
            break;
        case 3:
            plot_mode_sel = plot_all;
            ROS_INFO("Plotting linear acceleration & angular rate & euler angle begins...");
            break;
        default:
            ROS_WARN("The input plot selection is wrong. So plot linear acceleration by default.");
            plot_mode_sel = plot_acc;
            ROS_INFO("Plotting linear acceleration begins...");
            break;
    }
    return plot_mode_sel;
}

void ImuDataPlotter::loadImuData(const std::string& input_path, PlotMode plot_mode_sel)
{
    std::string line, word;
    std::vector<uint32_t> ts_1st_row_vec;
    ts_1st_row_vec.reserve(2);
    std::vector<uint32_t> ts_sec_vecp;
    std::vector<uint32_t> ts_nsec_vecp;

    file_.open(input_path, std::ios_base::in);
    if(file_.is_open()) {
        int lines_counter = 0;
        while(std::getline(file_, line)) {
            int words_counter = 0;
            std::stringstream sstr(line);
            std::vector<double> imu_row_vec;
            imu_row_vec.reserve(10);
            while(std::getline(sstr, word, ',')) {
                if (words_counter < 2) {
                    if (lines_counter == 0) {
                        ts_1st_row_vec.emplace_back(static_cast<uint32_t>(std::stoul(word)));
                        if (ts_1st_row_vec.size() == 2)
                            ROS_DEBUG_STREAM("Loop: The first time stamp is " << ts_1st_row_vec[0] << "(sec), " <<
                                                                                 ts_1st_row_vec[1] << "(nsec)");
                    }
                    words_counter == 0 ? ts_sec_vecp.emplace_back(static_cast<uint32_t>(std::stoul(word))) :
                                         ts_nsec_vecp.emplace_back(static_cast<uint32_t>(std::stoul(word)));
                }
                else {
                    imu_row_vec.emplace_back(std::stod(word));
                    if (imu_row_vec.size() == 10) {
                        if (plot_mode_sel == 0 || plot_mode_sel == 3) {
                            acc_x_.emplace_back(imu_row_vec[0]);
                            acc_y_.emplace_back(imu_row_vec[1]);
                            acc_z_.emplace_back(imu_row_vec[2]);
                        }
                        if (plot_mode_sel == 1 || plot_mode_sel == 3) {
                            gyr_x_.emplace_back(imu_row_vec[3]);
                            gyr_y_.emplace_back(imu_row_vec[4]);
                            gyr_z_.emplace_back(imu_row_vec[5]);
                        }
                        if (plot_mode_sel == 2 || plot_mode_sel == 3) {
                            tf::Quaternion q(imu_row_vec[6], imu_row_vec[7], imu_row_vec[8], imu_row_vec[9]);
                            tf::Matrix3x3 m(q);
                            double roll, pitch, yaw;
                            m.getRPY(roll, pitch, yaw);
                            eul_x_.emplace_back(roll/M_PI*180);
                            eul_y_.emplace_back(pitch/M_PI*180);
                            eul_z_.emplace_back(yaw/M_PI*180);
                        }
                    }
                }
                ++words_counter;
            }
            ++lines_counter;
        }
    }
    else ROS_ERROR("Could not open the file of Imu data.\n");
    ROS_DEBUG_STREAM("After Loop: Size of ts_sec_vec is " << ts_sec_vecp.size() << "; " <<
                                 "Size of ts_nsec_vec is " << ts_nsec_vecp.size());
    ROS_DEBUG_STREAM("After Loop: Size of acc_x is " << acc_x_.size() << "; " <<
                                 "Size of acc_y is " << acc_y_.size() << "; " <<
                                 "Size of acc_z is " << acc_z_.size());

    std::transform(ts_sec_vecp.begin(), ts_sec_vecp.end(), ts_nsec_vecp.begin(), std::back_inserter(ts_vecp_),
                   [&](uint32_t sec, uint32_t nsec) { return (double)(sec - ts_1st_row_vec[0]) + ((double)nsec)/1.0e9; });
    if (debug_) {
        ROS_DEBUG_STREAM("After Loop: Size of ts_vec is " << ts_vecp_.size());
        ROS_DEBUG_STREAM("The first ten time stamps of Imu data (100Hz) starting from about 0 sec:");
        std::for_each(ts_vecp_.begin(), ts_vecp_.begin()+10,
                      [](double x){ ROS_DEBUG_STREAM("\t\t" << x); });
        ROS_DEBUG_STREAM("\n");
    }
}

void ImuDataPlotter::plotData(PlotMode plot_mode_sel,
                              const std::string& output_acc_plot_path,
                              const std::string& output_gyr_plot_path,
                              const std::string& output_eul_plot_path)
{
    switch(plot_mode_sel) {
        case 0:
            plotLinAccel_(output_acc_plot_path);
            ROS_INFO("Plotting linear acceleration ends.");
            break;
        case 1:
            plotAngRate_(output_gyr_plot_path);
            ROS_INFO("Plotting angular rate ends.");
            break;
        case 2:
            plotEulAngle_(output_eul_plot_path);
            ROS_INFO("Plotting euler angle ends.");
            break;
        case 3:
            plotLinAccel_(output_acc_plot_path);
            plotAngRate_(output_gyr_plot_path);
            plotEulAngle_(output_eul_plot_path);
            ROS_INFO("Plotting linear acceleration & angular rate & euler angle ends.");
            break;
        default:
            plotLinAccel_(output_acc_plot_path);
            ROS_DEBUG("Plotting linear acceleration ends.");
            break;
    }
}

void ImuDataPlotter::plotLinAccel_(const std::string& output_acc_plot_path)
{
    auto figure = matplot::figure(false);  // open figure in reactive mode
    figure->size(1280, 720);


    matplot::hold(matplot::on);
    matplot::subplot(3, 1, 0);
    matplot::plot(ts_vecp_, acc_x_);
//    matplot::title("Linear Acceleration [X]");
    matplot::ylabel("a_{x}({/:Italic m/s^{2}})");
    matplot::ytickformat("%.2f");

    matplot::subplot(3, 1, 1);
    matplot::plot(ts_vecp_, acc_y_);
//    matplot::title("Linear Acceleration [Y]");
    matplot::ylabel("a_{y}({/:Italic m/s^{2}})");
    matplot::ytickformat("%.2f");

    matplot::subplot(3, 1, 2);
    matplot::plot(ts_vecp_, acc_z_);
//    matplot::title("Linear Acceleration [Z]");
    matplot::xlabel("t({/:Italic s})");
    matplot::ylabel("a_{z}({/:Italic m/s^{2}})");
    matplot::ytickformat("%.2f");

    matplot::sgtitle("Linear Acceleration");
    matplot::hold(matplot::off);


    if(save_) matplot::save(output_acc_plot_path);
    matplot::show();
}

void ImuDataPlotter::plotAngRate_(const std::string& output_gyr_plot_path)
{
    auto figure = matplot::figure(false);  // open figure in reactive mode
    figure->size(1280, 720);


    matplot::hold(matplot::on);
    matplot::subplot(3, 1, 0);
    matplot::plot(ts_vecp_, gyr_x_);
//    matplot::title("Angular Rate [X]");
    matplot::ylabel("ω_{x}({/:Italic rad/s})");
    matplot::ytickformat("%.2f");

    matplot::subplot(3, 1, 1);
    matplot::plot(ts_vecp_, gyr_y_);
//    matplot::title("Angular Rate [Y]");
    matplot::ylabel("ω_{y}({/:Italic rad/s})");
    matplot::ytickformat("%.2f");

    matplot::subplot(3, 1, 2);
    matplot::plot(ts_vecp_, gyr_z_);
//    matplot::title("Angular Rate [Z]");
    matplot::xlabel("t({/:Italic s})");
    matplot::ylabel("ω_{z}({/:Italic rad/s})");
    matplot::ytickformat("%.2f");

    matplot::sgtitle("Angular Rate");
    matplot::hold(matplot::off);


    if(save_) matplot::save(output_gyr_plot_path);
    matplot::show();
}

void ImuDataPlotter::plotEulAngle_(const std::string& output_eul_plot_path)
{
    auto figure = matplot::figure(false);  // open figure in reactive mode
    figure->size(1280, 720);


    matplot::hold(matplot::on);
    matplot::subplot(3, 1, 0);
    matplot::plot(ts_vecp_, eul_x_);
//    matplot::title("Absolute Orientation [X]");
    matplot::ylabel("roll(^{°})");
    matplot::ytickformat("%.2f");

    matplot::subplot(3, 1, 1);
    matplot::plot(ts_vecp_, eul_y_);
//    matplot::title("Absolute Orientation [Y]");
    matplot::ylabel("pitch(^{°})");
    matplot::ytickformat("%.2f");

    matplot::subplot(3, 1, 2);
    matplot::plot(ts_vecp_, eul_z_);
//    matplot::title("Absolute Orientation [Z]");
    matplot::xlabel("t({/:Italic s})");
    matplot::ylabel("yaw(^{°})");
    matplot::ytickformat("%.2f");

    matplot::sgtitle("Absolute Orientation");
    matplot::hold(matplot::off);


    if(save_) matplot::save(output_eul_plot_path);
    matplot::show();
}
