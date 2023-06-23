#include "gui.h"

Gui::Gui(cv::Mat& img)
{
    // view
    int width = img.cols;
    int height = img.rows;
    pangolin::CreateWindowAndBind("lidar2camera", width, height);

    glEnable(GL_DEPTH_TEST);
    // glDepthMask(GL_TRUE);
    // glDepthFunc(GL_LESS);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

    pangolin::View &project_image =
            pangolin::Display("project_with")
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
                               -1.0 * (width - 200) / height)
                    .SetLock(pangolin::LockLeft, pangolin::LockTop);

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

    pangolin::Var<bool> plus_roll("cp.+ roll", false, false);
    pangolin::Var<bool> minus_roll("cp.- roll", false, false);
    pangolin::Var<bool> plus_pitch("cp.+ pitch", false, false);
    pangolin::Var<bool> minus_pitch("cp.- pitch", false, false);
    pangolin::Var<bool> plus_yaw("cp.+ yaw", false, false);
    pangolin::Var<bool> minus_yaw("cp.- yaw", false, false);
    pangolin::Var<bool> plus_x("cp.+ x", false, false);
    pangolin::Var<bool> minus_x("cp.- x", false, false);
    pangolin::Var<bool> plus_y("cp.+ y", false, false);
    pangolin::Var<bool> minux__y("cp.- y", false, false);
    pangolin::Var<bool> plus_z("cp.+ z", false, false);
    pangolin::Var<bool> minus_z("cp.- z", false, false);

    pangolin::Var<bool> addFx("cp.+ fx", false, false);
    pangolin::Var<bool> minusFx("cp.- fx", false, false);
    pangolin::Var<bool> addFy("cp.+ fy", false, false);
    pangolin::Var<bool> minusFy("cp.- fy", false, false);

    pangolin::Var<bool> resetButton("cp.Reset", false, false);
    pangolin::Var<bool> saveImg("cp.Save Image", false, false);

    std::vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(plus_roll);
    mat_calib_box.push_back(minus_roll);
    mat_calib_box.push_back(plus_pitch);
    mat_calib_box.push_back(minus_pitch);
    mat_calib_box.push_back(plus_yaw);
    mat_calib_box.push_back(minus_yaw);
    mat_calib_box.push_back(plus_x);
    mat_calib_box.push_back(minus_x);
    mat_calib_box.push_back(plus_y);
    mat_calib_box.push_back(minux__y);
    mat_calib_box.push_back(plus_z);
    mat_calib_box.push_back(minus_z);
}