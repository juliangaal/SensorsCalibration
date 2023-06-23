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
    }

    inline void reset() { mat = original; }

    Eigen::Matrix4d mat {};
    Eigen::Matrix4d original {};
};