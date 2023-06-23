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
#include <filesystem>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>

#include "assert_msg.h"

namespace fs = std::filesystem;

struct Intrinsics
{
    Intrinsics(const fs::path &path)
            : K{cv::Mat::eye(3, 3, CV_32FC1)},
              D{cv::Mat::zeros(5, 1, CV_32FC1)},
              K_original{},
              D_original{}
    {
        load(path);
        K_original = K;
        D_original = D;
    }

    ~Intrinsics() = default;

    inline void load(const fs::path &path)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        assert_msg(fs.isOpened(), "Failed to open calibration file. Check permissions.");

        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        fs.release();

        // why is explicit conversion necessary when initialized to CV_32FC1
        K.convertTo(K, CV_32FC1);
        D.convertTo(D, CV_32FC1);
    }

    inline void reset() { K = K_original; D = D_original; }

    cv::Mat K;
    cv::Mat D;
    cv::Mat K_original;
    cv::Mat D_original;
};

inline std::ostream& operator<<(std::ostream& os, const Intrinsics& i)
{
    os << "\nK: \n";
    os << i.K << "\n";
    os << "D: " << i.D << "\n";
    return os;
}