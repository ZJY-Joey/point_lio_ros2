// #ifndef PARAM_H
// #define PARAM_H
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include <string>
#include "preprocess.h"
#include <ivox/ivox3d.h>

// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

extern bool odom_only;
extern std::string odom_header_frame_id;
extern std::string odom_child_frame_id;

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;

extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len;
extern float DET_RANGE;
extern bool imu_en, gravity_align, non_station_start;
extern double imu_time_inte, first_imu_time;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern bool runtime_pos_log, pcd_save_en, path_en, pub_tf, publish_base_outputs;
extern std::string base_output_mode;
extern std::string base_odom_frame_id;
extern std::string base_child_frame_id;
extern std::string base_tf_parent_frame_id;
extern std::string base_tf_child_frame_id;
extern bool scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern double time_lag_imu_to_lidar;
//localization mode parameters
extern bool location_mode;
extern std::string map_path;
extern double initial_z;
extern std::shared_ptr<IVoxType> ivox;  
extern IVoxType::Options ivox_options;

void readParameters(shared_ptr<rclcpp::Node> &nh);
