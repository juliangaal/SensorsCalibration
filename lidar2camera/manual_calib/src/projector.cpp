/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */

#include <algorithm>
#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>

#include <vector>
#include "projector.h"

#define OVERLAP_FILTER_WINDOW 4
#define OVERLAP_DEPTH_TH 0.4 // 0.4m

Projector::Projector(const Intrinsics &intrinsics,
                     Extrinsics &extrinsics)
        : intrinsics_(intrinsics), extrinsics_(extrinsics), intensities_(), img_(), cloud_()
{
}

void Projector::apply_roi_filter()
{
    cv::Mat temp(cv::Size(cloud_.cols, cloud_.rows), CV_32FC1);
    int cnt = 0;
    for (int i = 0; i < cloud_.cols; ++i)
    {
        float x = cloud_.at<float>(0, i);
        float y = cloud_.at<float>(1, i);
        float z = cloud_.at<float>(2, i);
        if (x > roi_[0] && x < roi_[1] && y > roi_[2] && y < roi_[3] && z > roi_[4] &&
            z < roi_[5])
        {
            temp.at<float>(0, cnt) = x;
            temp.at<float>(1, cnt) = y;
            temp.at<float>(2, cnt) = z;
            ++cnt;
        }
    }
    cloud_ = temp.colRange(0, cnt);
}

void Projector::load_point_cloud(const pcl::PointCloud<pcl::PointXYZI> &pcl)
{
    cv::Mat mat(pcl.height, pcl.width, CV_32FC4, const_cast<float*>(reinterpret_cast<const float*>(&pcl.points[0])));

    // Split the cv::Mat into separate channels (x, y, z, intensity)
    cv::Mat channels[4];
    cv::split(mat, channels);

    // Create a new cv::Mat for (x, y, z) channels
    cv::Mat xyzMat;
    cv::merge(channels, 3, xyzMat);

    cloud_ = cv::Mat(cv::Size(pcl.points.size(), 3), CV_32FC1);
    for (size_t i = 0; i < pcl.points.size(); ++i)
    {
        cloud_.at<float>(0, i) = pcl.points[i].x;
        cloud_.at<float>(1, i) = pcl.points[i].y;
        cloud_.at<float>(2, i) = pcl.points[i].z;
        intensities_.push_back(pcl.points[i].intensity);
    }
    // ROIFilter();
}

cv::Scalar Projector::fake_color(float value)
{
    float posSlope = 255 / 60.0;
    float negSlope = -255 / 60.0;
    value *= 255;
    cv::Vec3f color;
    if (value < 60)
    {
        color[0] = 255;
        color[1] = posSlope * value + 0;
        color[2] = 0;
    } else if (value < 120)
    {
        color[0] = negSlope * value + 2 * 255;
        color[1] = 255;
        color[2] = 0;
    } else if (value < 180)
    {
        color[0] = 0;
        color[1] = 255;
        color[2] = posSlope * value - 2 * 255;
    } else if (value < 240)
    {
        color[0] = 0;
        color[1] = negSlope * value + 4 * 255;
        color[2] = 255;
    } else if (value < 300)
    {
        color[0] = posSlope * value - 4 * 255;
        color[1] = 0;
        color[2] = 255;
    } else
    {
        color[0] = 255;
        color[1] = 0;
        color[2] = negSlope * value + 6 * 255;
    }
    return cv::Scalar(color[0], color[1], color[2]);
}

cv::Mat Projector::project_to_raw_mat(const cv::Mat &K, const cv::Mat &D, const cv::Mat &R, const cv::Mat &T)
{
    assert_msg(K.type() == CV_32FC1, "Type must be float");
    assert_msg(D.type() == CV_32FC1, "Type must be float");
    assert_msg(R.type() == CV_32FC1, "Type must be float");
    assert_msg(T.type() == CV_32FC1, "Type must be float");

    cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat mapX, mapY;
    cv::Mat outImg = cv::Mat(img_.size(), CV_32FC3);
    cv::initUndistortRectifyMap(K, D, I, K, img_.size(), CV_32FC1, mapX, mapY);
    cv::remap(img_, outImg, mapX, mapY, cv::INTER_LINEAR);
    cv::Mat dist = cloud_.rowRange(0, 1).mul(cloud_.rowRange(0, 1)) +
                   cloud_.rowRange(1, 2).mul(cloud_.rowRange(1, 2)) +
                   cloud_.rowRange(2, 3).mul(cloud_.rowRange(2, 3));

    cv::Mat projCloud2d = K * (R * cloud_ + repeat(T, 1, cloud_.cols));
    float maxDist = 0;
    float maxIntensity = 0;
    std::vector<Pt> points;
    std::vector<std::vector<int>> filter_pts(img_.rows,
                                             std::vector<int>(img_.cols, -1));

    for (int32_t i = 0; i < projCloud2d.cols; ++i)
    {
        float x = projCloud2d.at<float>(0, i);
        float y = projCloud2d.at<float>(1, i);
        float z = projCloud2d.at<float>(2, i);
        int x2d = cvRound(x / z);
        int y2d = cvRound(y / z);
        float d = sqrt(dist.at<float>(0, i));
        float intensity = intensities_[i];

        if (x2d >= 0 && y2d >= 0 && x2d < img_.cols && y2d < img_.rows && z > 0)
        {
            maxDist = std::max(maxDist, d);
            maxIntensity = std::max(maxIntensity, intensity);
            points.push_back(Pt{cv::Point(x2d, y2d), d, z, intensity});
            // add size
            if (filter_pts[y2d][x2d] != -1)
            {
                int32_t p_idx = filter_pts[y2d][x2d];
                if (z < points[p_idx].z)
                    filter_pts[y2d][x2d] = points.size() - 1;
            } else
                filter_pts[y2d][x2d] = points.size() - 1;
        }
    }
    if (overlap_filter_)
    {
        std::vector<int> filtered_idxes;
        for (int32_t m = 0; m < img_.rows; m++)
        {
            for (int32_t n = 0; n < img_.cols; n++)
            {
                int current_idx = filter_pts[m][n];
                if (current_idx == -1)
                    continue;
                // search window
                bool front = true;
                for (int j = std::max(0, m - OVERLAP_FILTER_WINDOW);
                     j < std::min(img_.rows, m + OVERLAP_FILTER_WINDOW + 1); j++)
                {
                    for (int k = std::max(0, n - OVERLAP_FILTER_WINDOW);
                         k < std::min(img_.cols, n + OVERLAP_FILTER_WINDOW + 1); k++)
                    {
                        if (filter_pts[j][k] == -1)
                            continue;
                        int32_t p_idx = filter_pts[j][k];
                        if (points[current_idx].z - points[p_idx].z > OVERLAP_DEPTH_TH)
                        {
                            front = false;
                            break;
                        }
                    }
                }
                if (front)
                    filtered_idxes.push_back(current_idx);
            }
        }
        for (size_t i = 0; i < filtered_idxes.size(); ++i)
        {
            int pt_idx = filtered_idxes[i];
            cv::Scalar color;
            if (intensity_color_)
            {
                // intensity
                float intensity = points[pt_idx].intensity;
                color = fake_color(intensity / maxIntensity);
            } else
            {
                // distance
                float d = points[pt_idx].dist;
                color = fake_color(d / maxDist);
            }
            circle(outImg, points[pt_idx].point, point_size_, color, -1);
        }
    } else
    {
        sort(points.begin(), points.end(),
             [](const Pt &a, const Pt &b)
             { return a.dist > b.dist; });
        for (size_t i = 0; i < points.size(); ++i)
        {
            cv::Scalar color;
            if (intensity_color_)
            {
                // intensity
                float intensity = points[i].intensity;
                color = fake_color(intensity / maxIntensity);
            } else
            {
                // distance
                float d = points[i].dist;
                color = fake_color(d / maxDist);
            }
            circle(outImg, points[i].point, point_size_, color, -1);
        }
    }
    return outImg;
}

void Projector::load_img(const cv::Mat &mat)
{
    img_ = mat;
}

cv::Mat Projector::project(const cv::Mat &img, const pcl::PointCloud<pcl::PointXYZI> &pcl)
{
    if (img.empty() || pcl.empty())
    {
        return cv::Mat{};
    }

    load_point_cloud(pcl);
    load_img(img);

    const Eigen::Matrix<float, 3, 3> r = extrinsics_.mat.block<3, 3>(0, 0).cast<float>();
    cv::Mat R = cv::Mat(3, 3, CV_32FC1);
    cv::eigen2cv(r, R);

    const Eigen::Matrix<float, 3, 1> t = extrinsics_.mat.block<3, 1>(0, 3).cast<float>();
    cv::Mat T = cv::Mat(3, 1, CV_32FC1);
    cv::eigen2cv(t, T);

    const auto &K = intrinsics_.K;
    const auto &D = intrinsics_.D;

    return project_to_raw_mat(K, D, R, T);
}