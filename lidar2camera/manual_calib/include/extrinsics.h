/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <pangolin/pangolin.h>
#include "extrinsic_param.hpp"

using rpyxyz_tuple_t = std::tuple<double, double, double, double, double, double>;

struct Extrinsics
{
    explicit Extrinsics(const std::string &filename)
    {
        LoadExtrinsic(filename, mat);
        original = mat;
        Eigen::Vector3d ea = mat.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
        std::cout << "to Euler angles:" << std::endl;
        std::cout << ea << "\n\n";
    }

    inline void update_rpy_from_slider(const pangolin::Var<double> &roll, const pangolin::Var<double> &pitch,
                                       const pangolin::Var<double> &yaw)
    {
        Eigen::Matrix4d update = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rpy;
        rpy = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        update.block<3, 3>(0, 0) = rpy;

        auto rot = mat.block<3, 3>(0, 0);
        Eigen::Matrix4d lidar2cv = Eigen::Matrix4d::Identity();
        lidar2cv.block<3, 3>(0, 0) = rot;

        mat = lidar2cv * update;
    }

    inline void update_xyz_from_slider(double x, double y, double z)
    {
        Eigen::Matrix4d update = Eigen::Matrix4d::Identity();
        update.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

        auto rot = mat.block<3, 3>(0, 0);
        Eigen::Matrix4d lidar2cv = Eigen::Matrix4d::Identity();
        lidar2cv.block<3, 3>(0, 0) = rot;

        mat = lidar2cv * update;
    }

    inline void update_incremental(const Eigen::Matrix4d& inc_change)
    {
        mat *= inc_change;
    }

    inline rpyxyz_tuple_t get_rpyxyz()
    {
        auto cv2lidar = mat.inverse();
        auto xyz = cv2lidar.block<3, 1>(0, 3);
        auto rpy = cv2lidar.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
        Eigen::Matrix<double, 6, 1> joined(6);
        joined << rpy, xyz;
        return { joined[0], joined[1], joined[2], joined[3], joined[4], joined[5] };
    }

    inline void save(const fs::path& filename) {
        cv::Mat cv_rot;
        cv::eigen2cv(mat, cv_rot);
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        fs << "lidar2cv4x4" << cv_rot;
        fs.release();
    }

    inline void reset() { mat = original; }

    Eigen::Matrix4d mat {};
    Eigen::Matrix4d original {};
};

