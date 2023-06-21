/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <opencv2/core/eigen.hpp>
#include <filesystem>
#include <stdio.h>
#include <string>

namespace fs = std::filesystem;

struct Intrinsics
{
    Intrinsics(const fs::path &path)
            : K{cv::Mat::eye(3, 3, CV_32FC1)},
              D{cv::Mat::zeros(5, 1, CV_32FC1)}
    {
        load(path);
    }

    ~Intrinsics() = default;

    void load(const fs::path &path)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        assert(fs.isOpened() && "Failed to open calibration file. Check permissions.");

        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        fs.release();
    }

    cv::Mat K;
    cv::Mat D;
};

std::ostream& operator<<(std::ostream& os, const Intrinsics& i)
{
    os << "\nK: \n";
    os << i.K << "\n";
    os << "D: " << i.D << "\n";
    return os;
}