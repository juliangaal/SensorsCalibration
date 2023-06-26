/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"
#include "assert_msg.h"
#include "intrinsics.h"
#include "extrinsics.h"
#include "projector.h"

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define APPLY_COLOR_TO_LIDAR_INTENSITY // to set intensity colored or not

constexpr double rad2deg_factor = 180.0 * M_PI;

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
double cali_scale_fxfy_ = 1.005;
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> inc_transform_change(12);

bool keyboard_hit()
{
    termios term;
    tcgetattr(0, &term);
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    tcsetattr(0, TCSANOW, &term);
    return byteswaiting > 0;
}

void UpdateCalibrationAdjustment()
{
    for (int32_t i = 0; i < 12; i++)
    {
        std::vector<int> transform_flag(6, 0);
        transform_flag[i / 2] = (i % 2) ? (-1) : 1;
        Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rot_tmp;
        rot_tmp =
                Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / rad2deg_factor,
                                  Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / rad2deg_factor,
                                  Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / rad2deg_factor,
                                  Eigen::Vector3d::UnitZ());
        tmp.block(0, 0, 3, 3) = rot_tmp;
        tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
        tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
        tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
        inc_transform_change[i] = tmp;
    }
}

//void saveResult(const cv::Mat &calib_img, const int &frame_id)
//{
//    std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
//    std::ofstream fCalib(file_name);
//    if (!fCalib.is_open())
//    {
//        std::cerr << "open file " << file_name << " failed." << std::endl;
//        return;
//    }
//    fCalib << "Extrinsic:" << std::endl;
//    fCalib << "R:\n"
//           << extrinsics.mat(0, 0) << " " << extrinsics.mat(0, 1) << " "
//           << extrinsics.mat(0, 2) << "\n"
//           << extrinsics.mat(1, 0) << " " << extrinsics.mat(1, 1) << " "
//           << extrinsics.mat(1, 2) << "\n"
//           << extrinsics.mat(2, 0) << " " << extrinsics.mat(2, 1) << " "
//           << extrinsics.mat(2, 2) << std::endl;
//    fCalib << "t: " << extrinsics.mat(0, 3) << " "
//           << extrinsics.mat(1, 3) << " " << extrinsics.mat(2, 3)
//           << std::endl;
//    fCalib << "\nIntrinsic:" << std::endl;
//    fCalib << intrinsic_matrix_(0, 0) << " " << intrinsic_matrix_(0, 1) << " "
//           << intrinsic_matrix_(0, 2) << "\n"
//           << intrinsic_matrix_(1, 0) << " " << intrinsic_matrix_(1, 1) << " "
//           << intrinsic_matrix_(1, 2) << "\n"
//           << intrinsic_matrix_(2, 0) << " " << intrinsic_matrix_(2, 1) << " "
//           << intrinsic_matrix_(2, 2) << std::endl;
//
//    fCalib << "************* json format *************" << std::endl;
//    fCalib << "Extrinsic:" << std::endl;
//    fCalib << "[" << extrinsics.mat(0, 0) << "," << extrinsics.mat(0, 1)
//           << "," << extrinsics.mat(0, 2) << "," << extrinsics.mat(0, 3)
//           << "],"
//           << "[" << extrinsics.mat(1, 0) << "," << extrinsics.mat(1, 1)
//           << "," << extrinsics.mat(1, 2) << "," << extrinsics.mat(1, 3)
//           << "],"
//           << "[" << extrinsics.mat(2, 0) << "," << extrinsics.mat(2, 1)
//           << "," << extrinsics.mat(2, 2) << "," << extrinsics.mat(2, 3)
//           << "],"
//           << "[" << extrinsics.mat(3, 0) << "," << extrinsics.mat(3, 1)
//           << "," << extrinsics.mat(3, 2) << "," << extrinsics.mat(3, 3)
//           << "]" << std::endl;
//    fCalib << "\nIntrinsic:" << std::endl;
//    fCalib << "[" << intrinsic_matrix_(0, 0) << "," << intrinsic_matrix_(0, 1)
//           << "," << intrinsic_matrix_(0, 2) << "],"
//           << "[" << intrinsic_matrix_(1, 0) << "," << intrinsic_matrix_(1, 1)
//           << "," << intrinsic_matrix_(1, 2) << "],"
//           << "[" << intrinsic_matrix_(2, 0) << "," << intrinsic_matrix_(2, 1)
//           << "," << intrinsic_matrix_(2, 2) << "]" << std::endl;
//
//    fCalib << "\nDistortion:" << std::endl;
//    fCalib << "[";
//    for (size_t i = 0; i < distortions_.size(); i++)
//    {
//        fCalib << distortions_[i];
//        if (i == distortions_.size() - 1)
//            continue;
//        fCalib << ",";
//    }
//    fCalib << "]";
//    fCalib.close();
//
//    std::string img_name = "calibimg_" + std::to_string(frame_id) + ".jpg";
//    cv::imwrite(img_name, calib_img);
//}

bool ManualCalibration(int key_input)
{
    char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
    bool real_hit = false;
    for (int32_t i = 0; i < 12; i++)
    {
        if (key_input == table[i])
        {
            extrinsics.mat = extrinsics.mat * inc_transform_change[i];
            real_hit = true;
        }
    }
//    // adjust fx, fy
//    if (key_input == 'u')
//    {
//        intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
//        std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
//    }
//    if (key_input == 'j')
//    {
//        intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
//        std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
//    }
//    if (key_input == 'i')
//    {
//        intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
//        std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
//    }
//    if (key_input == 'k')
//    {
//        intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
//        std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
//    }
    return real_hit;
}

void help()
{
    std::cout << "Usage: ./test <image_path> <pcd_path> "
                 "<intrinsic_xml> <extrinsic_json>"
                 "\nexample:\n\t"
                 "./bin/run_lidar2camera data/0.png data/0.pcd "
                 "data/center_camera-intrinsic.json "
                 "data/top_center_lidar-to-center_camera-extrinsic.json"
              << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        help();
        return 0;
    }

    auto camera_path = fs::path(argv[1]);
    auto lidar_path = fs::path(argv[2]);
    auto intrinsic_json = fs::path(argv[3]);
    auto extrinsic_json = fs::path(argv[4]);

    assert_msg(fs::exists(camera_path), "image path doesn't exist");
    assert_msg(fs::exists(lidar_path), "lidar path doesn't exist");
    assert_msg(fs::exists(intrinsic_json), "intrinsics path doesn't exist");
    assert_msg(fs::exists(extrinsic_json), "extrinsics path doesn't exist");

    cv::Mat img = cv::imread(camera_path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZI> pcd = *cloud;
    // load intrinsic

    Intrinsics intrinsics(intrinsic_json);
    std::cout << "* Loaded intrinsics\n";

    Extrinsics extrinsics(extrinsic_json);
    std::cout << "* Loaded extrinsics\n";

    UpdateCalibrationAdjustment();


    Projector projector(<#initializer#>, <#initializer#>, <#initializer#>, <#initializer#>);
    projector.load_point_cloud(pcd);

    // view
    int width = img.cols;
    int height = img.rows;
    std::cout << "width_:" << width << " , height_:" << height << std::endl;
    // const int width_ = 1920, height_ = 1200;
    pangolin::CreateWindowAndBind("lidar2camera player", width, height);

    glEnable(GL_DEPTH_TEST);
    // glDepthMask(GL_TRUE);
    // glDepthFunc(GL_LESS);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

    pangolin::View &project_image =
            pangolin::Display("project")
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
                               -1.0 * (width - 200) / height)
                    .SetLock(pangolin::LockLeft, pangolin::LockTop);

    unsigned char *imageArray = new unsigned char[3 * width * height];
    pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                     GL_UNSIGNED_BYTE);

    // control panel
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                          pangolin::Attach::Pix(150));
    pangolin::Var<bool> displayMode("cp.Intensity Color", false,
                                    true);                            // logscale
    pangolin::Var<bool> filterMode("cp.Overlap Filter", false, true); // logscale
    pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 1);       // logscale
    pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
    pangolin::Var<double> fxfyScale("cp.fxfy scale", 1.005, 1, 1.1);
    pangolin::Var<int> pointSize("cp.point size", 3, 1, 5);

    pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
    pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
    pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
    pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
    pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
    pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
    pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
    pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
    pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
    pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
    pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
    pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

    pangolin::Var<bool> addFx("cp.+ fx", false, false);
    pangolin::Var<bool> minusFx("cp.- fx", false, false);
    pangolin::Var<bool> addFy("cp.+ fy", false, false);
    pangolin::Var<bool> minusFy("cp.- fy", false, false);

    pangolin::Var<bool> resetButton("cp.Reset", false, false);
    pangolin::Var<bool> saveImg("cp.Save Image", false, false);

    std::vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(addXdegree);
    mat_calib_box.push_back(minusXdegree);
    mat_calib_box.push_back(addYdegree);
    mat_calib_box.push_back(minusYdegree);
    mat_calib_box.push_back(addZdegree);
    mat_calib_box.push_back(minusZdegree);
    mat_calib_box.push_back(addXtrans);
    mat_calib_box.push_back(minusXtrans);
    mat_calib_box.push_back(addYtrans);
    mat_calib_box.push_back(minusYtrans);
    mat_calib_box.push_back(addZtrans);
    mat_calib_box.push_back(minusZtrans);

    cv::Mat current_frame = projector.project(<#initializer#>, 0);
    int frame_num = 0;

    std::cout << "\n=>START\n";
    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (displayMode)
        {
            if (display_mode_ == false)
            {
                projector.set_intensity_color(true);
                current_frame = projector.project(<#initializer#>, 0);
                display_mode_ = true;
            }
        } else
        {
            if (display_mode_ == true)
            {
                projector.set_intensity_color(false);
                current_frame = projector.project(<#initializer#>, 0);
                display_mode_ = false;
            }
        }

        if (filterMode)
        {
            if (filter_mode_ == false)
            {
                projector.set_overlap_filter(true);
                current_frame = projector.project(<#initializer#>, 0);
                filter_mode_ = true;
            }
        } else
        {
            if (filter_mode_ == true)
            {
                projector.set_overlap_filter(false);
                current_frame = projector.project(<#initializer#>, 0);
                filter_mode_ = false;
            }
        }

        if (degreeStep.GuiChanged())
        {
            cali_scale_degree_ = degreeStep.Get();
            UpdateCalibrationAdjustment();
            std::cout << "Degree calib scale changed to " << cali_scale_degree_
                      << " degree\n";
        }
        if (tStep.GuiChanged())
        {
            cali_scale_trans_ = tStep.Get() / 100.0;
            UpdateCalibrationAdjustment();
            std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                      << " cm\n";
        }
        if (fxfyScale.GuiChanged())
        {
            cali_scale_fxfy_ = fxfyScale.Get();
            std::cout << "fxfy calib scale changed to " << cali_scale_fxfy_
                      << std::endl;
        }
        if (pointSize.GuiChanged())
        {
            int ptsize = pointSize.Get();
            projector.set_point_size(ptsize);
            current_frame = projector.project(<#initializer#>, 0);
            std::cout << "point size changed to " << ptsize << std::endl;
        }
        for (int i = 0; i < 12; i++)
        {
            if (pangolin::Pushed(mat_calib_box[i]))
            {
                extrinsics.mat = extrinsics.mat * inc_transform_change[i];
                std::cout << "Changed!\n";
                current_frame = projector.project(<#initializer#>, 0);
            }
        }

        if (pangolin::Pushed(addFx))
        {
            intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
            current_frame = projector.project(<#initializer#>, 0);
            std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
        }
        if (pangolin::Pushed(minusFx))
        {
            intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
            current_frame = projector.project(<#initializer#>, 0);
            std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
        }
        if (pangolin::Pushed(addFy))
        {
            intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
            current_frame = projector.project(<#initializer#>, 0);
            std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
        }
        if (pangolin::Pushed(minusFy))
        {
            intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
            current_frame = projector.project(<#initializer#>, 0);
            std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
        }

        if (pangolin::Pushed(resetButton))
        {
            extrinsics.mat = orign_extrinsics.mat;
            intrinsic_matrix_ = orign_intrinsic_matrix_;
            current_frame = projector.project(<#initializer#>, 0);
            std::cout << "Reset!\n";
        }
        if (pangolin::Pushed(saveImg))
        {
            saveResult(current_frame, frame_num);
            std::cout << "\n==>Save Result " << frame_num << std::endl;
            Eigen::Matrix4d transform = extrinsics.mat;
            cout << "Transfromation Matrix:\n" << transform << std::endl;
            frame_num++;
        }

        if (keyboard_hit())
        {
            int c = getchar();
            if (ManualCalibration(c))
            {
                Eigen::Matrix4d transform = extrinsics.mat;
                cout << "\nTransfromation Matrix:\n" << transform << std::endl;
            }
            current_frame = projector.project(<#initializer#>, 0);
        }

        imageArray = current_frame.data;
        imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
        glFinish();
    }

    // delete[] imageArray;

    Eigen::Matrix4d transform = extrinsics.mat;
    cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

    return 0;
}
