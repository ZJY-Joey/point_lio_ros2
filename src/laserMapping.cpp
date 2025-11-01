#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "parameters.h"
#include "Estimator.h"
#include "perf.h"
#include "log.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

// for location mode
/// @brief 定位初始化标志位
bool flg_location_inited = false;
bool flg_get_init_guess = false;
/// @brief 初始化姿态
Eigen::Vector3d init_translation = Eigen::Vector3d::Zero();
Eigen::Quaterniond init_rotation = Eigen::Quaterniond::Identity();
/// @brief 初始位置和方向
Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
Eigen::Quaterniond initial_orientation = Eigen::Quaterniond::Identity();
/// @brief 地图点云
PointCloudXYZI::Ptr map_cloud(new PointCloudXYZI());
/// @brief 初始点云发布器
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubInitialCloud;       
mutex init_lock; 
/// @brief tf坐标变换
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
geometry_msgs::msg::TransformStamped tf_world2odom, tf_aft2base;



const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;

int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false, flg_first_scan = true;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

// Time Log Variables
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool lidar_pushed = false, flg_reset = false, flg_exit = false;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
PointCloudXYZI::Ptr init_total_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

MeasureGroup Measures;

sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

auto logger = rclcpp::get_logger("laserMapping");

void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(logger, "catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
                kf_output.x_.gravity(2)); // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I.normalized() * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I.normalized() * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

int points_cache_size = 0;

void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

/// @brief ikdtree的局部窗口更新函数，暂时弃用
// void lasermap_fov_segment() {
//     cub_needrm.shrink_to_fit();

//     V3D pos_LiD;
//     if (use_imu_as_input) {
//         pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
//     } else {
//         pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
//     }
//     if (!Localmap_Initialized) {
//         for (int i = 0; i < 3; i++) {
//             LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
//             LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
//         }
//         Localmap_Initialized = true;
//         return;
//     }
//     float dist_to_map_edge[3][2];
//     bool need_move = false;
//     for (int i = 0; i < 3; i++) {
//         dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
//         dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
//         if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
//             dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
//             need_move = true;
//     }
//     if (!need_move) return;
//     BoxPointType New_LocalMap_Points, tmp_boxpoints;
//     New_LocalMap_Points = LocalMap_Points;
//     float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
//                          double(DET_RANGE * (MOV_THRESHOLD - 1)));
//     for (int i = 0; i < 3; i++) {
//         tmp_boxpoints = LocalMap_Points;
//         if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
//             New_LocalMap_Points.vertex_max[i] -= mov_dist;
//             New_LocalMap_Points.vertex_min[i] -= mov_dist;
//             tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
//             cub_needrm.emplace_back(tmp_boxpoints);
//         } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
//             New_LocalMap_Points.vertex_max[i] += mov_dist;
//             New_LocalMap_Points.vertex_min[i] += mov_dist;
//             tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
//             cub_needrm.emplace_back(tmp_boxpoints);
//         }
//     }
//     LocalMap_Points = New_LocalMap_Points;

//     points_cache_collect();
//     if (cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
// }

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.sec;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    double time_div = get_time_sec(msg->header.stamp);
    p_pre->process(msg, ptr);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar; //get_time_sec(msg->header.stamp);
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    double time_div = get_time_sec(msg->header.stamp);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_div_i = *ptr_div;
                // cout << "ptr div i num:" << ptr_div_i->size() << endl;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar; //get_time_sec(msg->header.stamp);
        }
        // accumulate points for localization
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
    publish_count++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        RCLCPP_ERROR(logger, "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
    if (!imu_en) {
        if (!lidar_buffer.empty()) {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            if (meas.lidar->points.size() < 1) {
                cout << "lose lidar" << std::endl;
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() < 1) {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points) {
            if (pt.curvature > end_time) {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    } else if (!init_map) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;

/// @brief 地图增量更新函数（已弃用）
// void map_incremental() {
//     PointVector PointToAdd;
//     PointVector PointNoNeedDownsample;
//     PointToAdd.reserve(feats_down_size);
//     PointNoNeedDownsample.reserve(feats_down_size);

//     for (int i = 0; i < feats_down_size; i++) {
//         if (!Nearest_Points[i].empty()) {
//             const PointVector &points_near = Nearest_Points[i];
//             bool need_add = true;
//             PointType downsample_result, mid_point;
//             mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
//                           0.5 * filter_size_map_min;
//             mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
//                           0.5 * filter_size_map_min;
//             mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
//                           0.5 * filter_size_map_min;
//             /* If the nearest points is definitely outside the downsample box */
//             if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min ||
//                 fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min ||
//                 fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min) {
//                 PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
//                 continue;
//             }
//             /* Check if there is a point already in the downsample box */
//             float dist = calc_dist<float>(feats_down_world->points[i], mid_point);
//             for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
//                 /* Those points which are outside the downsample box should not be considered. */
//                 if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min &&
//                     fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min &&
//                     fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {
//                     need_add = false;
//                     break;
//                 }
//             }
//             if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
//         } else {
//             // PointToAdd.emplace_back(feats_down_world->points[i]);
//             PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
//         }
//     }
//     int add_point_size = ikdtree.Add_Points(PointToAdd, true);
//     ikdtree.Add_Points(PointNoNeedDownsample, false);
// }

/// @brief  地图增量更新函数
void map_incremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
        const PointVector &points_near = Nearest_Points[i];

        Eigen::Vector3f center =
            ((point_world.getVector3fMap() / filter_size_map_min)
                .array()
                .floor() +
            0.5) *
            filter_size_map_min;
        bool need_add = true;
        for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
            Eigen::Vector3f dis_2_center =
                points_near[readd_i].getVector3fMap() - center;
            if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
            need_add = false;
            break;
            }
        }
        if (need_add) {
            points_to_add.emplace_back(point_world);
        }
        } else {
        points_to_add.emplace_back(point_world);
        }
    }
    ivox->AddPoints(points_to_add);
}

/// @brief 初始姿态回调函数  接收外部重定位点，并在点的0.4米范围，高度0.5米范围内搜索重定位点。
/// \param pose_msg
void initialpose_cbk(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    RCLCPP_INFO(logger, "Receive init pose");
    if (flg_location_inited) {
        return;
    }
    // TODO: safe lock_guard check
    std::lock_guard<std::mutex> guard(init_lock);

    init_translation[0] = pose_msg->pose.pose.position.x;
    init_translation[1] = pose_msg->pose.pose.position.y;

    double sum_z = 0.0, delta_x, delta_y, distance;
    PointType p;
    int count = 0;
    for (std::size_t i = 0; i < map_cloud->points.size(); i++) {
        p = map_cloud->points[i];
        float z_diff = std::fabs(p.z - initial_z);
        if (z_diff > 0.5) {
        continue;
        }

        delta_x = p.x - pose_msg->pose.pose.position.x;
        delta_y = p.y - pose_msg->pose.pose.position.y;
        distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        if (distance < 0.4) {
        sum_z += p.z;
        count++;
        }
    }
    if (count != 0) {
        init_translation[2] = sum_z / count;
    } else {
        init_translation[2] = initial_z;
    }

    init_rotation = Eigen::Quaterniond(
        pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);

    flg_get_init_guess = true;
    RCLCPP_INFO(logger, "Init translation: %f %f %f", init_translation(0),
            init_translation(1), init_translation(2));
    RCLCPP_INFO(logger, "Init quaternion: %f %f %f %f.", init_rotation.w(),
            init_rotation.x(), init_rotation.y(), init_rotation.z());
}

/// @brief 使用NDT+ICP进行初始位姿估计 获得重定位变换矩阵
void initial_pose() {
    Eigen::Affine3d init_guess;
    if (flg_get_init_guess) {
        Eigen::Matrix4d init_guess_matrix = Eigen::Matrix4d::Identity();
        init_guess_matrix.block<3, 3>(0, 0) = init_rotation.toRotationMatrix();
        init_guess_matrix.block<3, 1>(0, 3) = init_translation;
        std::lock_guard<std::mutex> guard(init_lock);
        init_guess.matrix() = init_guess_matrix;
    } else {
        return;
    }

    //计时宏开始
    MFLA_TIMER_BLOCK_START();
    // source 点云降采样
    PointCloudXYZI::Ptr source_cloud_ds(new PointCloudXYZI);
    downSizeFilterSurf.setInputCloud(init_total_world);
    downSizeFilterSurf.filter(*source_cloud_ds);

    // from coarse(ndt) to fine(icp)
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(1e-4);
    ndt.setEuclideanFitnessEpsilon(1e-4);
    ndt.setMaximumIterations(40);
    ndt.setResolution(0.5);
    ndt.setInputSource(source_cloud_ds);
    ndt.setInputTarget(map_cloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(40);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    // icp.setRANSACIterations(0);
    icp.setInputSource(source_cloud_ds);
    icp.setInputTarget(map_cloud);

    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    ndt.align(*unused_result, init_guess.matrix().cast<float>());
    if (ndt.hasConverged()) {
        icp.align(*unused_result, ndt.getFinalTransformation());
    } else {
        RCLCPP_WARN(logger, "NDT has not converged!Please try again!");
        return;
    }
    //计时宏结束
    MFLA_TIMER_BLOCK_END("Initializetion Regisration");
    // 发布初始化点云
    Eigen::Matrix4f cloud_aligned_pose = icp.getFinalTransformation();
    pcl::PointCloud<PointType>::Ptr cloud_aligned(
        new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*init_total_world, *cloud_aligned,
                            cloud_aligned_pose);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_aligned, cloud_msg);
    cloud_msg.header.stamp = get_ros_time(lidar_end_time);
    cloud_msg.header.frame_id = "camera_init";
    pubInitialCloud->publish(cloud_msg);

    double score = icp.getFitnessScore();
    if (icp.hasConverged() == false || score == 0.0 || score > 0.5) {
        RCLCPP_ERROR(logger, "Global Initializing Fail with %f!", score);
        flg_location_inited = false;
        if (flg_get_init_guess) {
        flg_get_init_guess= false;
        }
        return;
    } else {
        init_guess = icp.getFinalTransformation().cast<double>();

        initial_position = init_guess.translation();
        initial_orientation = Eigen::Quaterniond(init_guess.rotation());

        RCLCPP_INFO(logger, "\033[1;35m Initializing Succeed with %f score! \033[0m", score);
        RCLCPP_INFO(logger, "\033[1;35m Initializing Position: %f %f %f,%f %f %f %f.\033[0m",
                initial_position(0), initial_position(1), initial_position(2),
                initial_orientation.w(), initial_orientation.x(),
                initial_orientation.y(), initial_orientation.z());

        flg_location_inited = true;

        // if (!use_imu_as_input) {
        //   Eigen::Matrix<double, 30, 30>
        //       P_init_output;  // = MD(24, 24)::Identity() * 0.01;
        //   reset_cov_output(P_init_output);

        //   state_out = state_output();
        //   state_out.pos = final_position;
        //   state_out.rot = final_rotation.toRotationMatrix();

        //   kf_output.change_P(P_init_output);
        //   kf_output.x_.pos = final_position;
        //   kf_output.x_.rot = final_rotation.toRotationMatrix();
        // } else {
        //   Eigen::Matrix<double, 24, 24> P_init;  // = MD(24, 24)::Identity() *
        //   0.01; reset_cov(P_init);

        //   state_in = state_input();
        //   state_in.pos = final_position;
        //   state_in.rot = final_rotation.toRotationMatrix();

        //   kf_input.change_P(P_init);
        //   kf_input.x_.pos = final_position;
        //   kf_input.x_.rot = final_rotation.toRotationMatrix();
        // }
    }
}

void publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
    
    if (odom_only) {return;}

    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_header_frame_id;
    if (!odom_only) {
        pubLaserCloudFullRes->publish(laserCloudmsg);
    }
}

void publish_init_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
    int size_init_map = init_feats_world->size();

    sensor_msgs::msg::PointCloud2 laserCloudmsg;

    pcl::toROSMsg(*init_feats_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {

    if (odom_only) {return;}

    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            // if (i % 3 == 0)
            // {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // 
            // }
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_header_frame_id;
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body) {

    if (odom_only) {return;}

    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

template<typename T>
void set_posestamp(T &out) {
    if (!use_imu_as_input) {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        out.orientation.x = kf_output.x_.rot.coeffs()[0];
        out.orientation.y = kf_output.x_.rot.coeffs()[1];
        out.orientation.z = kf_output.x_.rot.coeffs()[2];
        out.orientation.w = kf_output.x_.rot.coeffs()[3];
    } else {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        out.orientation.x = kf_input.x_.rot.coeffs()[0];
        out.orientation.y = kf_input.x_.rot.coeffs()[1];
        out.orientation.z = kf_input.x_.rot.coeffs()[2];
        out.orientation.w = kf_input.x_.rot.coeffs()[3];
    }
}

template<typename T>
void set_twist(T &out) {
    if (!use_imu_as_input) {
        out.linear.x = kf_output.x_.vel(0);
        out.linear.y = kf_output.x_.vel(1);
        out.linear.z = kf_output.x_.vel(2);
        out.angular.x = kf_output.x_.omg(0);
        out.angular.y = kf_output.x_.omg(1);
        out.angular.z = kf_output.x_.omg(2);
    } else {
        out.linear.x = kf_input.x_.vel(0);
        out.linear.y = kf_input.x_.vel(1);
        out.linear.z = kf_input.x_.vel(2);
        out.angular.x = imu_last.angular_velocity.x;
        out.angular.y = imu_last.angular_velocity.y;
        out.angular.z = imu_last.angular_velocity.z;
    }
}

/// @brief publish odometry information （已弃用）
// void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
//                       std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_br) {

//     odomAftMapped.header.frame_id = odom_header_frame_id;
//     odomAftMapped.child_frame_id = odom_child_frame_id;

//     if (publish_odometry_without_downsample) {
//         odomAftMapped.header.stamp = get_ros_time(time_current);
//     } else {
//         odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
//     }
//     set_posestamp(odomAftMapped.pose.pose);
//     set_twist(odomAftMapped.twist.twist);

//     if (odom_only){
//         Matrix3d cov = kf_output.get_P().block<3, 3>(0, 0);

//         // Get the position components (first 3x3)
//         for (int i = 0; i < 3; i++) {
//             for (int j = 0; j < 3; j++) {
//                 odomAftMapped.pose.covariance[6 * i + j] = cov(i, j);
//             }
//         }

//         odomAftMapped.pose.covariance[21] = 0.0;    // Covariance for roll
//         odomAftMapped.pose.covariance[28] = 0.0;    // Covariance for pitch
//         odomAftMapped.pose.covariance[35] = 0.05;   // Covariance for yaw

//         odomAftMapped.twist.covariance[0] = 0.1;    // Covariance for linear velocity on x
//         odomAftMapped.twist.covariance[7] = 0.1;    // Covariance for linear velocity on y
//         odomAftMapped.twist.covariance[14] = 0.0;   // Covariance for linear velocity on z
//         odomAftMapped.twist.covariance[21] = 0.0;  // Covariance for angular velocity (roll)
//         odomAftMapped.twist.covariance[28] = 0.0;  // Covariance for angular velocity (pitch)
//         odomAftMapped.twist.covariance[35] = 0.05;  // Covariance for angular velocity (yaw)
//     }

//     pubOdomAftMapped->publish(odomAftMapped);

//     //static tf2_ros::TransformBroadcaster br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
//     geometry_msgs::msg::TransformStamped transform;
//     transform.header.frame_id = odom_header_frame_id;
//     transform.child_frame_id = odom_child_frame_id;

//     transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
//     transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
//     transform.transform.translation.z = odomAftMapped.pose.pose.position.z;

//     transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
//     transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
//     transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
//     transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

//     transform.header.stamp = odomAftMapped.header.stamp;

//     tf_br->sendTransform(transform);
// }

// publish odometry information for b2 configuration
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
                      const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pub_base_odom,
                      const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr &pub_base_pose,
                      const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster) {
    nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";

    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = rclcpp::Time(time_current * 1e9); // Convert seconds to nanoseconds
    } else {
        odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    }
    set_posestamp(odomAftMapped.pose.pose);

    // Add velocity information
    if (!use_imu_as_input) {
        odomAftMapped.twist.twist.linear.x = kf_output.x_.vel(0);
        odomAftMapped.twist.twist.linear.y = kf_output.x_.vel(1);
        odomAftMapped.twist.twist.linear.z = kf_output.x_.vel(2);

        odomAftMapped.twist.twist.angular.x = kf_output.x_.omg(0);
        odomAftMapped.twist.twist.angular.y = kf_output.x_.omg(1);
        odomAftMapped.twist.twist.angular.z = kf_output.x_.omg(2);
    } else {
        odomAftMapped.twist.twist.linear.x = kf_input.x_.vel(0);
        odomAftMapped.twist.twist.linear.y = kf_input.x_.vel(1);
        odomAftMapped.twist.twist.linear.z = kf_input.x_.vel(2);

        Eigen::Vector3d gyro_world =
            Eigen::Quaterniond(kf_input.x_.rot).toRotationMatrix() * input_in.gyro;
        odomAftMapped.twist.twist.angular.x = gyro_world(0);
        odomAftMapped.twist.twist.angular.y = gyro_world(1);
        odomAftMapped.twist.twist.angular.z = gyro_world(2);
    }

    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = odomAftMapped.header.stamp;
    transformStamped.header.frame_id = "camera_init";
    transformStamped.child_frame_id = "aft_mapped";
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation = odomAftMapped.pose.pose.orientation;

    tf_broadcaster->sendTransform(transformStamped);

    // Transform calculations
    tf2::Transform tf_odom2base, tf_world2base;
    tf2::Transform tf_world2odom_tf, tf_aft2base_tf;
    tf2::fromMsg(tf_world2odom.transform, tf_world2odom_tf);
    tf2::fromMsg(tf_aft2base.transform, tf_aft2base_tf);

    tf_odom2base = tf_aft2base_tf * tf2::Transform(tf2::Quaternion(
        odomAftMapped.pose.pose.orientation.x,
        odomAftMapped.pose.pose.orientation.y,
        odomAftMapped.pose.pose.orientation.z,
        odomAftMapped.pose.pose.orientation.w),
        tf2::Vector3(
            odomAftMapped.pose.pose.position.x,
            odomAftMapped.pose.pose.position.y,
            odomAftMapped.pose.pose.position.z));

    tf_world2base = tf_world2odom_tf * tf_odom2base;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = odomAftMapped.header.stamp;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "aliengo";
    odom_msg.pose.pose.position.x = tf_world2base.getOrigin().x();
    odom_msg.pose.pose.position.y = tf_world2base.getOrigin().y();
    odom_msg.pose.pose.position.z = tf_world2base.getOrigin().z();
    geometry_msgs::msg::Quaternion orientation_msg;
    tf2::convert(tf_world2base.getRotation(), orientation_msg);
    odom_msg.pose.pose.orientation = orientation_msg;

    

    tf2::Vector3 twist_world(odomAftMapped.twist.twist.linear.x,
                             odomAftMapped.twist.twist.linear.y,
                             odomAftMapped.twist.twist.linear.z);
    tf2::Matrix3x3 q_world(tf_world2base.getRotation());
    tf2::Vector3 twist_base = q_world.inverse() * twist_world;

    odom_msg.twist.twist.linear.x = twist_base.x();
    odom_msg.twist.twist.linear.y = twist_base.y();
    odom_msg.twist.twist.angular.z = odomAftMapped.twist.twist.angular.z;

    pub_base_odom->publish(odom_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";
    msg_pose.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
    msg_pose.pose.pose.position.x = tf_world2base.getOrigin().x();
    msg_pose.pose.pose.position.y = tf_world2base.getOrigin().y();
    msg_pose.pose.pose.position.z = tf_world2base.getOrigin().z();
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(tf_world2base.getRotation(), quat_msg);
    msg_pose.pose.pose.orientation = quat_msg;

    Eigen::Matrix<double, 6, 6> cov;
    if (use_imu_as_input) {
        cov = kf_input.get_P().topLeftCorner(6, 6);
    } else {
        cov = kf_output.get_P().topLeftCorner(6, 6);
    }
    for (int i = 0; i < 36; ++i) {
        msg_pose.pose.covariance[i] = cov(i / 6, i % 6);
    }

    pub_base_pose->publish(msg_pose);
}


void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath) {

    if (odom_only) {return;}

    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = odom_header_frame_id;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");
    readParameters(nh);

    ivox = std::make_shared<IVoxType>(ivox_options);

    cout << "lidar_type: " << lidar_type << endl;

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = odom_header_frame_id;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;

    /*** initialize variables ***/
    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox_ = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
    } else {
        sub_pcl = nh->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
    }
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_cbk);



    //sub initial pose
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose;
    if (location_mode) {
        sub_init_pose = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, initialpose_cbk);
    }
    

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_body;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_base_odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_base_pose;
    

    if (!odom_only){
        pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_registered", 100000);
        pubLaserCloudFullRes_body = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_registered_body", 100000);
        pubLaserCloudEffect = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_effected", 100000);
        pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/global_map", 100000);  //changed from laser_map 2 global_map
        pubPath = nh->create_publisher<nav_msgs::msg::Path>
                ("/path", 100000);
    }

    // Choose topic name depending on odom_only value
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    if (odom_only){
        pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
                ("/odom_corrected", 100000);
    } else {
        pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
                ("/aft_mapped_to_init", 100000);
        pub_base_odom = nh->create_publisher<nav_msgs::msg::Odometry>("/base_odom", 100);
        pub_base_pose = nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/livox_pose", 100);
    }
    pubInitialCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_initial", 10);

    //auto plane_pub = nh->create_publisher<visualization_msgs::msg::Marker>
    //        ("/planner_normal", 1000);
    //TODO: tf tree should be checked
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);


    // create shared buffer and listener (listener stores a reference to the buffer)
    tf_buffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);
    try {
        RCLCPP_INFO(logger, "Waiting for tf...");
        // use buffer->canTransform / lookupTransform in ROS2
        if (!tf_buffer->canTransform("world", "camera_init", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
            RCLCPP_WARN(logger, "can't transform world -> camera_init yet");
        }
        if (!tf_buffer->canTransform("aft_mapped", "aliengo", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
            RCLCPP_WARN(logger, "can't transform aft_mapped -> aliengo yet");
        }
        tf_world2odom = tf_buffer->lookupTransform("world", "camera_init", rclcpp::Time(0));
        tf_aft2base = tf_buffer->lookupTransform("aft_mapped", "aliengo", rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger, "%s", ex.what());
    }

    ///  读取地图点云并发布
    if (location_mode) {
        RCLCPP_INFO(logger, "Loading global map...");
        pcl::io::loadPCDFile(map_path, *map_cloud);
        RCLCPP_INFO(logger, "Load map cloud with size: %zu.", map_cloud->points.size());

        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(map_cloud);
        sor.setLeafSize(0.1, 0.1, 0.4);
        sor.filter(*map_cloud);

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*map_cloud, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Clock().now();
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudMap->publish(laserCloudmsg);
    }

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    while (rclcpp::ok()) {
        if (flg_exit) break;
        //ros::spinOnce();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(nh);
        executor.spin_some(); // 处理当前可用的回调

        if (sync_packages(Measures)) {
            //location mode to initial pose
            if (location_mode && !flg_location_inited) {
                PointCloudXYZI::Ptr init_world(new PointCloudXYZI());
                init_world->resize(Measures.lidar->points.size());
                for (int i = 0; i < Measures.lidar->points.size(); i++) {
                pointBodyToWorld(&(Measures.lidar->points[i]),
                                &(init_world->points[i]));
                }

                if (init_total_world->points.size() > 10000) {
                initial_pose();
                if (!flg_location_inited) {
                    continue;
                }
                } else {
                *init_total_world += *init_world;
                continue;
                }
            }
            // 重置系统包括定位
            if (flg_reset) {
                RCLCPP_WARN(logger, "reset when rosbag play back");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                if (use_imu_as_input) {
                // state_in = kf_input.get_x();
                state_in = state_input();
                kf_input.change_P(P_init);
                } else {
                // state_out = kf_output.get_x();
                state_out = state_output();
                kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;

                ivox.reset(new IVoxType(ivox_options));
            }

            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                cout << "first lidar time" << first_lidar_time << endl;
            }

            if (flg_reset) {
                RCLCPP_WARN(logger, "reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            double t0, t1, t2, t3, t4, t5, match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, feats_undistort);

            if (feats_undistort->empty() || feats_undistort == nullptr) {
                continue;
            }
            //对齐重力方向
            // if (imu_en) {
            //     if (!p_imu->gravity_align_) {
            //         while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp)) {
            //             imu_last = imu_next;
            //             imu_next = *(imu_deque.front());
            //             imu_deque.pop_front();
            //             // imu_deque.pop();
            //         }
            //         if (non_station_start) {
            //             state_in.gravity << VEC_FROM_ARRAY(gravity_init);
            //             state_out.gravity << VEC_FROM_ARRAY(gravity_init);
            //             state_out.acc << VEC_FROM_ARRAY(gravity_init);
            //             state_out.acc *= -1;
            //         } else {
            //             state_in.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
            //             state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
            //             state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
            //         }
            //         if (gravity_align) {
            //             Eigen::Matrix3d rot_init;
            //             p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
            //             p_imu->Set_init(state_in.gravity, rot_init);
            //             state_in.gravity = state_out.gravity = p_imu->gravity_;
            //             state_in.rot = state_out.rot = rot_init;
            //             state_in.rot.normalize();
            //             state_out.rot.normalize();
            //             state_out.acc = -rot_init.transpose() * state_out.gravity;
            //         }
            //         kf_input.change_x(state_in);
            //         kf_output.change_x(state_out);
            //     }
            // } else {
            //     if (!p_imu->gravity_align_) {
            //         state_in.gravity << VEC_FROM_ARRAY(gravity_init);
            //         state_out.gravity << VEC_FROM_ARRAY(gravity_init);
            //         state_out.acc << VEC_FROM_ARRAY(gravity_init);
            //         state_out.acc *= -1;
            //     }
            // }
            // align IMU and lidar pose 
            /// @brief 使用imu平均加速度/配置重力方向，初始化滤波器 当未初始化成功时，组织后续点云更新 代替上一函数
            if (!p_imu->after_imu_init_) {
                if (!p_imu->imu_need_init_) {
                V3D tmp_gravity;
                if (imu_en) {
                    tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                } else {
                    tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                    p_imu->after_imu_init_ = true;
                }
                // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                M3D rot_init;

                if (!location_mode) {
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                } else {
                    if (flg_location_inited) {
                    rot_init = initial_orientation.toRotationMatrix();
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                    kf_input.x_.pos = initial_position;
                    kf_output.x_.pos = initial_position;
                    } else {
                    AWARN_F("Wating for initial pose!");
                    continue;
                    }
                }
                // kf_input.x_.rot; //.normalize();
                // kf_output.x_.rot; //.normalize();
                kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
                } else {
                continue;
                }
            }

            /*** Segment the map in lidar FOV ***/
            // lasermap_fov_segment();
            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            time_seq = time_compressing<int>(feats_down_body);
            feats_down_size = feats_down_body->points.size();

            /*** initialize the map kdtree ***/
            // if (!init_map) {
            //     if (ikdtree.Root_Node == nullptr) //
            //         // if(feats_down_size > 5)
            //     {
            //         ikdtree.set_downsample_param(filter_size_map_min);
            //     }

            //     feats_down_world->resize(feats_down_size);
            //     for (int i = 0; i < feats_down_size; i++) {
            //         pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            //     }
            //     for (size_t i = 0; i < feats_down_world->size(); i++) {
            //         init_feats_world->points.emplace_back(feats_down_world->points[i]);
            //     }
            //     if (init_feats_world->size() < init_map_size) continue;
            //     ikdtree.Build(init_feats_world->points);
            //     init_map = true;
            //     publish_init_kdtree(pubLaserCloudMap); //(pubLaserCloudFullRes);
            //     continue;
            // }
            /// @brief 初始化体素地图 with ivox
            if (!init_map) {
                if (location_mode) {
                init_feats_world->reserve(map_cloud->size());
                *init_feats_world = *map_cloud;
                } else {
                feats_down_world->resize(feats_undistort->size());
                for (int i = 0; i < feats_undistort->size(); i++) {
                    {
                    pointBodyToWorld(&(feats_undistort->points[i]),
                                    &(feats_down_world->points[i]));
                    }
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                }

                if (init_feats_world->size() < init_map_size) {
                init_map = false;
                } else {
                ivox->AddPoints(init_feats_world->points);
                if (!location_mode) {
                    publish_init_map(pubLaserCloudMap);  //(pubLaserCloudFullRes);
                }

                init_feats_world.reset(new PointCloudXYZI());
                init_map = true;
                }
                continue;
            }
            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);

            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(feats_down_body->points[i].x,
                               feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (extrinsic_est_en) {
                    if (!use_imu_as_input) {
                        point_this = kf_output.x_.offset_R_L_I.normalized() * point_this + kf_output.x_.offset_T_L_I;
                    } else {
                        point_this = kf_input.x_.offset_R_L_I.normalized() * point_this + kf_input.x_.offset_T_L_I;
                    }
                } else {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i] = point_crossmat;
            }

            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** point by point update ****/

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame) {
                        if (imu_en) {
                            while (time_current > get_time_sec(imu_next.header.stamp)) {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                                // imu_deque.pop();
                            }

                            angvel_avr
                                    << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr
                                    << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    /// @brief IMU数据处理及滤波器预测更新 （已弃用）
                    /// 因为第二种代码更加稳健且协方差传播更加正确
                    // if (imu_en && !imu_deque.empty()) {
                    //     bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                    //     while (imu_comes) {
                    //         imu_upda_cov = true;
                    //         angvel_avr
                    //                 << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                    //         acc_avr
                    //                 << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                    //         /*** covariance update ***/
                    //         imu_last = imu_next;
                    //         imu_next = *(imu_deque.front());
                    //         imu_deque.pop_front();
                    //         double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                    //         kf_output.predict(dt, Q_output, input_in, true, false);
                    //         time_predict_last_const = get_time_sec(imu_last.header.stamp); // big problem
                    //         imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                    //         // if (!imu_comes)
                    //         {
                    //             double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;

                    //             if (dt_cov > 0.0) {
                    //                 time_update_last = get_time_sec(imu_last.header.stamp);
                    //                 double propag_imu_start = omp_get_wtime();

                    //                 kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    //                 propag_time += omp_get_wtime() - propag_imu_start;
                    //                 double solve_imu_start = omp_get_wtime();
                    //                 kf_output.update_iterated_dyn_share_IMU();
                    //                 solve_time += omp_get_wtime() - solve_imu_start;
                    //             }
                    //         }
                    //     }
                    // }
                    if (imu_en && !imu_deque.empty()) {
                        bool last_imu = get_time_sec(imu_next.header.stamp) ==
                                        get_time_sec(imu_deque.front()->header.stamp);
                        while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                                !imu_deque.empty()) {
                            if (!last_imu) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            break;
                            } else {
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            }
                        }
                        bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        while (imu_comes) {
                            imu_upda_cov = true;
                            angvel_avr << imu_next.angular_velocity.x,
                                imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr << imu_next.linear_acceleration.x,
                                imu_next.linear_acceleration.y,
                                imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            double dt =
                                get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const =
                                get_time_sec(imu_next.header.stamp);  // big problem

                            {
                            double dt_cov =
                                get_time_sec(imu_next.header.stamp) - time_update_last;

                            if (dt_cov > 0.0) {
                                time_update_last = get_time_sec(imu_next.header.stamp);
                                double propag_imu_start = omp_get_wtime();

                                kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                propag_time += omp_get_wtime() - propag_imu_start;
                                double solve_imu_start = omp_get_wtime();
                                kf_output.update_iterated_dyn_share_IMU();
                                solve_time += omp_get_wtime() - solve_imu_start;
                            }
                            }
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        }
                    }
                    if (flg_reset) {
                    break;
                    }

                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    if (prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (!imu_en && (dt_cov >= imu_time_inte)) // (point_cov_not_prop && imu_prop_cov)
                        {
                            double propag_cov_start = omp_get_wtime();
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            imu_upda_cov = false;
                            time_update_last = time_current;
                            propag_time += omp_get_wtime() - propag_cov_start;
                        }
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample) {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, pub_base_odom, pub_base_pose, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                     << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                     << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                     << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }

                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                    // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                }
            } else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame) {
                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // imu_deque.pop();
                        }
                        imu_prop_cov = true;
                        // imu_upda_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current;
                        // if(prop_at_freq_of_imu)
                        {
                            input_in.gyro << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;

                            input_in.acc << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                            // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                            //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                            //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

                            // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                            //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                            // 0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);

                            // angvel_avr -= state.bias_g;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }

                    while (time_current > get_time_sec(imu_next.header.stamp)) // && !imu_deque.empty())
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro
                                << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc
                                << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                        // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                        //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                        //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

                        // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                        //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                        //             0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        double dt = get_time_sec(imu_last.header.stamp) - t_last;

                        // if(!prop_at_freq_of_imu)
                        // {       
                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = get_time_sec(imu_last.header.stamp); //time_current;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);
                        t_last = get_time_sec(imu_last.header.stamp);
                        imu_prop_cov = true;
                        // imu_upda_cov = true;
                    }

                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();

                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);

                    propag_time += omp_get_wtime() - propag_start;

                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    // if(prop_at_freq_of_imu)
                    // {
                    //     double dt_cov = time_current - time_update_last;
                    //     if ((imu_prop_cov && dt_cov > 0.0) || (dt_cov >= imu_time_inte * 1.2)) 
                    //     {
                    //         double propag_cov_start = omp_get_wtime();
                    //         kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                    //         propag_time += omp_get_wtime() - propag_cov_start;
                    //         time_update_last = time_current;
                    //         imu_prop_cov = false;
                    //     }
                    // }
                    if (publish_odometry_without_downsample) {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, pub_base_odom, pub_base_pose, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                     << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                     << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }
            }

            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample) {
                publish_odometry(pubOdomAftMapped, pub_base_odom, pub_base_pose, tf_broadcaster);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();

            if (!location_mode && feats_down_size > 4) {
                map_incremental();
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_bodyframe_pub_en) publish_frame_body(pubLaserCloudFullRes_body);

            /*** Debug variables Logging ***/
            if (runtime_pos_log) {
                frame_num++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                { aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num; }
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
                       t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                       aver_time_propag);
                if (!publish_odometry_without_downsample) {
                    if (!use_imu_as_input) {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                 << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                 << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                 << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    } else {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                 << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                 << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        rate.sleep();
    }
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
