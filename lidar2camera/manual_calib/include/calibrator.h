#pragma once

#include "intrinsics.h"
#include "extrinsics.h"
#include "projector.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pangolin/pangolin.h>

//template <typename T>
//struct SliderWithChange : pangolin::Var<T>
//{
//    SliderWithChange()
//};

class Calibrator
{
public:
    Calibrator(const Intrinsics &intrinsics,
               Extrinsics &extrinsics);
    ~Calibrator() = default;
    void run();

private:
    void update_calibration_adjustment();
    bool hit_keyboard();
    bool update_extrinsics_by_keyboard();
    bool load_img(const std::string& camera_path);
    bool load_pcl(const std::string& lidar_path);
private:
    struct Params
    {
        double change_rate_rpy_ = 0.3;
        double change_rate_xyz_ = 0.06;
        static constexpr double rad2deg_factor = 180.0 * M_PI;
    } p {};

    size_t width_ = 1000;
    size_t height_ = 1000;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> inc_transform_change_{12};
    cv::Mat img_ {};
    pcl::PointCloud<pcl::PointXYZI> cloud_ {};
    pangolin::GlTexture image_texture_ {};
    Projector projector;
};
