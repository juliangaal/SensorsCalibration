#include <termios.h>
#include <sys/ioctl.h>
#include <pangolin/pangolin.h>

#include "calibrator.h"

Calibrator::Calibrator(const cv::Mat &img, const pcl::PointCloud<pcl::PointXYZI> &pcd, const Intrinsics &intrinsics,
                       Extrinsics &extrinsics)
        : p(), width_(img.size().width), height_(img.size().height), extrinsics(extrinsics),
          projector(img, pcd, intrinsics, extrinsics)
{
    update_calibration_adjustment();
}

void Calibrator::update_calibration_adjustment()
{
    for (size_t i = 0; i < inc_transform_change.size(); i++)
    {
        std::vector<int> transform_flag(6, 0);
        transform_flag[i / 2] = (i % 2) ? (-1) : 1;
        Eigen::Matrix4d update = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rpy_update;
        rpy_update =
                Eigen::AngleAxisd(transform_flag[0] * p.change_rate_rpy_ / p.rad2deg_factor, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(transform_flag[1] * p.change_rate_rpy_ / p.rad2deg_factor, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(transform_flag[2] * p.change_rate_rpy_ / p.rad2deg_factor, Eigen::Vector3d::UnitZ());
        update.block(0, 0, 3, 3) = rpy_update;
        update(0, 3) = transform_flag[3] * p.change_rate_xyz_;
        update(1, 3) = transform_flag[4] * p.change_rate_xyz_;
        update(2, 3) = transform_flag[5] * p.change_rate_xyz_;
        inc_transform_change[i] = update;
    }
}

void Calibrator::run()
{
    pangolin::CreateWindowAndBind("lidar2camera player", width_, height_);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

    pangolin::View &project_image = pangolin::Display("project")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0, -1.0 * (width_ - 200) / height_)
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(width_, height_, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    // control panel
    pangolin::CreatePanel("ctrl_panel").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0, pangolin::Attach::Pix(150));
    pangolin::Var<bool> show_intensity_color_btn("ctrl_panel.Intensity Color", false, true);
    pangolin::Var<bool> overlap_filter_btn("ctrl_panel.Overlap Filter", false, true);
    pangolin::Var<double> degree_step_slider("ctrl_panel.deg step", 0.3, 0, 1);
    pangolin::Var<double> t_step_slider("ctrl_panel.t step(cm)", 6, 0, 15);
    pangolin::Var<double> fxfyScale("ctrl_panel.fxfy scale", 1.005, 1, 1.1);
    pangolin::Var<int> point_size_slider("ctrl_panel.point size", 3, 1, 5);

    pangolin::Var<bool> plus_roll_btn("ctrl_panel.+ roll", false, false);
    pangolin::Var<bool> minus_roll_btn("ctrl_panel.- roll", false, false);
    pangolin::Var<bool> plus_pitch_btn("ctrl_panel.+ pitch", false, false);
    pangolin::Var<bool> minus_pitch_btn("ctrl_panel.- pitch", false, false);
    pangolin::Var<bool> plus_yaw_btn("ctrl_panel.+ yaw", false, false);
    pangolin::Var<bool> minus_yaw_btn("ctrl_panel.- yaw", false, false);
    pangolin::Var<bool> plus_x_btn("ctrl_panel.+ x", false, false);
    pangolin::Var<bool> minus_x_btn("ctrl_panel.- x", false, false);
    pangolin::Var<bool> plus_y_btn("ctrl_panel.+ y", false, false);
    pangolin::Var<bool> minus_y_btn("ctrl_panel.- y", false, false);
    pangolin::Var<bool> plus_z_btn("ctrl_panel.+ z", false, false);
    pangolin::Var<bool> minus_z_btn("ctrl_panel.- z", false, false);

    pangolin::Var<bool> reset_btn("ctrl_panel.Reset", false, false);
    pangolin::Var<bool> save_img_btn("ctrl_panel.Save Image", false, false);

    pangolin::Var<int> current_image("ctrl_panel.Image", 0);
    pangolin::Var<bool> image_switch_button("ctrl_panel.Switch Image", false, false);

    std::vector<pangolin::Var<bool>> mat_calib_box;
    mat_calib_box.push_back(plus_roll_btn);
    mat_calib_box.push_back(minus_roll_btn);
    mat_calib_box.push_back(plus_pitch_btn);
    mat_calib_box.push_back(minus_pitch_btn);
    mat_calib_box.push_back(plus_yaw_btn);
    mat_calib_box.push_back(minus_yaw_btn);
    mat_calib_box.push_back(plus_x_btn);
    mat_calib_box.push_back(minus_x_btn);
    mat_calib_box.push_back(plus_y_btn);
    mat_calib_box.push_back(minus_y_btn);
    mat_calib_box.push_back(plus_z_btn);
    mat_calib_box.push_back(minus_z_btn);

    cv::Mat current_frame = projector.project();

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (show_intensity_color_btn)
        {
            projector.set_intensity_color(true);
            current_frame = projector.project();
        }
        else
        {
            if (projector.get_intensity_color())
            {
                projector.set_intensity_color(false);
                current_frame = projector.project();
            }
        }

        if (overlap_filter_btn)
        {
            projector.set_overlap_filter(true);
            current_frame = projector.project();
        }
        else
        {
            if (projector.get_overlap_filter())
            {
                projector.set_overlap_filter(false);
                current_frame = projector.project();
            }
        }

        if (degree_step_slider.GuiChanged())
        {
            p.change_rate_rpy_ = degree_step_slider.Get();
            update_calibration_adjustment();
        }

        if (t_step_slider.GuiChanged())
        {
            p.change_rate_xyz_ = t_step_slider.Get() / 100.0;
            update_calibration_adjustment();
        }

        if (point_size_slider.GuiChanged())
        {
            int ptsize = point_size_slider.Get();
            projector.set_point_size(ptsize);
            current_frame = projector.project();
        }

        for (size_t i = 0; i < inc_transform_change.size(); i++)
        {
            if (pangolin::Pushed(mat_calib_box[i]))
            {
                extrinsics.mat = extrinsics.mat * inc_transform_change[i];
                current_frame = projector.project();
            }
        }

        if (pangolin::Pushed(reset_btn))
        {
            extrinsics.reset();
            current_frame = projector.project();
            std::cout << "Reset Parameters\n";
        }

        if (pangolin::Pushed(save_img_btn))
        {
//            saveResult(current_frame, frame_num);
            Eigen::Matrix4d transform = extrinsics.mat;
            std::cout << "Saving:\n" << transform << std::endl;
        }

        if (pangolin::Pushed(image_switch_button))
        {
            current_image = !current_image;
        }

        if (update_extrinsics_by_keyboard())
        {
            current_frame = projector.project();
        }

        imageTexture.Upload(current_frame.data, GL_BGR, GL_UNSIGNED_BYTE);

        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewportFlipY(); // flip because of opencv data layout

        pangolin::FinishFrame();
        glFinish();
    }

}

bool Calibrator::update_extrinsics_by_keyboard()
{
    constexpr static char table[] = {'R', 'r', 'P', 'p', 'Y', 'y', 'X', 'x', 'Y', 'y', 'z', 'Z'};

    if (hit_keyboard())
    {
        int key = getchar();
        if (auto it = std::find(std::begin(table), std::end(table), key); it != std::end(table))
        {
            extrinsics.mat *= inc_transform_change[std::distance(std::begin(table), it)];
            return true;
        }
    }

    return false;
}

bool Calibrator::hit_keyboard()
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
