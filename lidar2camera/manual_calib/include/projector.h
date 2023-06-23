#pragma once

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "intrinsics.h"
#include "extrinsics.h"

#define OVERLAP_FILTER_WINDOW 4
#define OVERLAP_DEPTH_TH 0.4 // 0.4m

struct Pt
{
    cv::Point point;
    float dist;
    float z;
    float intensity;
};

class Projector
{
public:
    Projector(const cv::Mat &img, const pcl::PointCloud<pcl::PointXYZI> &pcl, const Intrinsics &intrinsics,
              Extrinsics &extrinsics);
    ~Projector() = default;
    cv::Mat project();
    inline void set_point_size(int size) { point_size_ = size; }
    inline void set_intensity_color(bool intensity_show) { intensity_color_ = intensity_show; }
    inline bool get_intensity_color() { return intensity_color_; }
    inline void set_overlap_filter(bool filter_mode) { overlap_filter_ = filter_mode; }
    inline bool get_overlap_filter() { return overlap_filter_; }

private:
    void apply_roi_filter();
    bool load_point_cloud(const pcl::PointCloud<pcl::PointXYZI>& pcl);
    cv::Scalar fake_color(float value);
    cv::Mat project_to_raw_mat(const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& T);

private:
    const float roi_[6] = {-4, 3.5, 5.0, 10.0, -2.1, 3.0};
    int point_size_ = 3;
    bool intensity_color_ = false;
    bool overlap_filter_ = false;
    const cv::Mat& img_;
    const Intrinsics& intrinsics_;
    cv::Mat cloud_;
    Extrinsics& extrinsics_;
    std::vector<float> intensities_;
};
