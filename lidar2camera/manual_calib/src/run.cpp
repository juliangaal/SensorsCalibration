#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include "assert_msg.h"
#include "calibrator.h"

namespace fs = std::filesystem;

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
    assert_msg(!img.empty(), "Read empty image");

    pcl::PointCloud<pcl::PointXYZI> pcd;
    assert_msg(pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, pcd) != -1, "Couldn't read .pcd file");

    Intrinsics intrinsics(intrinsic_json);
    Extrinsics extrinsics(extrinsic_json);

    Calibrator calib(img, pcd, intrinsics, extrinsics);
    calib.run();
}