#pragma once

#include "intrinsics.h"
#include "extrinsics.h"
#include "projector.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Calibrator
{
public:
    Calibrator(const cv::Mat &img, const pcl::PointCloud<pcl::PointXYZI> &pcd, const Intrinsics &intrinsics,
               Extrinsics &extrinsics);
    ~Calibrator() = default;
    void run();

private:
    void update_calibration_adjustment();
    bool hit_keyboard();
    bool update_extrinsics_by_keyboard();
private:
    struct Params
    {
        double change_rate_rpy_ = 0.3;
        double change_rate_xyz_ = 0.06;
        static constexpr double rad2deg_factor = 180.0 * M_PI;
    } p;

    size_t width_;
    size_t height_;
    Extrinsics &extrinsics;
    Projector projector;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> inc_transform_change{12};
    bool show_intensity_color = false;
    bool filter_mode_ = false;
};
