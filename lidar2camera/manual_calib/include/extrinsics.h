/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once
#include <Eigen/Core>
#include "extrinsic_param.hpp"

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

    inline void reset() { mat = original; }

    Eigen::Matrix4d mat {};
    Eigen::Matrix4d original {};
};