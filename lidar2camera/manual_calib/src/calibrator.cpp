#include <termios.h>
#include <sys/ioctl.h>
#include <pcl/io/pcd_io.h>
#include "calibrator.h"
#include "portable-file-dialogs/portable-file-dialogs.h"

Calibrator::Calibrator(const Intrinsics &intrinsics, Extrinsics &extrinsics)
        : projector(intrinsics, extrinsics)
{
    update_calibration_adjustment();
}

void Calibrator::update_calibration_adjustment()
{
    for (size_t i = 0; i < inc_transform_change_.size(); i++)
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
        inc_transform_change_[i] = update;
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

    image_texture_ = pangolin::GlTexture(width_, height_, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    // control panel
    pangolin::CreatePanel("ui").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0, pangolin::Attach::Pix(150));
    pangolin::Var<bool> show_intensity_color_btn("ui.Intensity Color", false, true);
    pangolin::Var<bool> overlap_filter_btn("ui.Overlap Filter", false, true);
    pangolin::Var<double> degree_step_slider("ui.deg step", 0.3, 0, 1);
    pangolin::Var<double> t_step_slider("ui.t step(cm)", 6, 0, 15);
    pangolin::Var<double> fxfyScale("ui.fxfy scale", 1.005, 1, 1.1);
    pangolin::Var<int> point_size_slider("ui.point size", 3, 1, 5);

    pangolin::Var<bool> plus_roll_btn("ui.+ roll", false, false);
    pangolin::Var<bool> minus_roll_btn("ui.- roll", false, false);
    pangolin::Var<bool> plus_pitch_btn("ui.+ pitch", false, false);
    pangolin::Var<bool> minus_pitch_btn("ui.- pitch", false, false);
    pangolin::Var<bool> plus_yaw_btn("ui.+ yaw", false, false);
    pangolin::Var<bool> minus_yaw_btn("ui.- yaw", false, false);
    pangolin::Var<bool> plus_x_btn("ui.+ x", false, false);
    pangolin::Var<bool> minus_x_btn("ui.- x", false, false);
    pangolin::Var<bool> plus_y_btn("ui.+ y", false, false);
    pangolin::Var<bool> minus_y_btn("ui.- y", false, false);
    pangolin::Var<bool> plus_z_btn("ui.+ z", false, false);
    pangolin::Var<bool> minus_z_btn("ui.- z", false, false);

    pangolin::Var<bool> reset_parameters_btn("ui.Reset Parameters", false, false);
    pangolin::Var<bool> save_img_btn("ui.Save Image", false, false);

    pangolin::Var<int> current_image("ui.Image", 0);
    pangolin::Var<bool> image_switch_btn("ui.Switch Image", false, false);

    pangolin::Var<bool> image_load_btn("ui.Load Image", false, false);
    pangolin::Var<bool> pcd_load_btn("ui.Load PCD", false, false);

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

    pangolin::Var<double> roll_slider("ui.roll", 0, -M_PI, M_PI);
    pangolin::Var<double> pitch_slider("ui.pitch", 0, -M_PI, M_PI);
    pangolin::Var<double> yaw_slider("ui.yaw", 0, -M_PI, M_PI);

    cv::Mat current_frame {};

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (!img_.empty() && !cloud_.empty())
        {
            if (show_intensity_color_btn)
            {
                projector.set_intensity_color(true);
            }
            else
            {
                if (projector.get_intensity_color())
                {
                    projector.set_intensity_color(false);
                }
            }

            if (overlap_filter_btn)
            {
                projector.set_overlap_filter(true);
            }
            else
            {
                if (projector.get_overlap_filter())
                {
                    projector.set_overlap_filter(false);
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
            }

            update_extrinsics_by_keyboard();

            for (size_t i = 0; i < inc_transform_change_.size(); i++)
            {
                if (pangolin::Pushed(mat_calib_box[i]))
                {
                    projector.get_extrinsics().mat *= inc_transform_change_[i];
                }
            }

            if (pangolin::Pushed(reset_parameters_btn))
            {
                projector.get_extrinsics().reset();
                std::cout << "Reset Parameters\n";
            }

            if (pangolin::Pushed(save_img_btn))
            {
//            saveResult(current_frame, frame_num);
                Eigen::Matrix4d transform = projector.get_extrinsics().mat;
                std::cout << "Saving:\n" << transform << std::endl;
            }

            if (pangolin::Pushed(image_switch_btn))
            {
                current_image = !current_image;
            }

            current_frame = projector.project(img_, cloud_);
        }

        if (pangolin::Pushed(image_load_btn))
        {
            auto selection = pfd::open_file("Select a file", ".", {"Image Files", "*.png *.jpeg *.jpg"}).result();
            if (selection.empty())
            {
                std::cerr << "No image selection!\n";
            }
            else
            {
                const auto &camera_path = selection[0];
                if (!load_img(camera_path))
                {
                    std::cout << "Couldn't load image\n";
                }
            }
        }

        if (pangolin::Pushed(pcd_load_btn))
        {
            auto selection = pfd::open_file("Select a file", ".", {"PCD Files", "*.pcd"}).result();
            if (selection.empty())
            {
                std::cerr << "No pcd selection!\n";
            }
            else
            {
                const auto& lidar_path = selection[0];
                if (!load_pcl(lidar_path))
                {
                    std::cout << "Couldn't load pcl\n";
                }
            }
        }

        image_texture_.Upload(current_frame.data, GL_BGR, GL_UNSIGNED_BYTE);
        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        image_texture_.RenderToViewportFlipY(); // flip because of opencv data layout

        pangolin::FinishFrame();
        glFinish();
    }

}

bool Calibrator::update_extrinsics_by_keyboard()
{
    constexpr static char table[] = {'R', 'r', 'P', 'p', 'Y', 'y', 'X', 'x', 'l', 'L', 'z', 'Z'};

    if (hit_keyboard())
    {
        int key = getchar();
        if (auto it = std::find(std::begin(table), std::end(table), key); it != std::end(table))
        {
            projector.get_extrinsics().mat *= inc_transform_change_[std::distance(std::begin(table), it)];
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

bool Calibrator::load_img(const std::string &camera_path)
{
    img_ = cv::imread(camera_path);
    if (!img_.empty())
    {
        width_ = img_.size().width;
        height_ = img_.size().height;
        pangolin::GetBoundWindow()->Resize(width_, height_);
        image_texture_ = pangolin::GlTexture(width_, height_, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
        return true;
    }

    return false;
}

bool Calibrator::load_pcl(const std::string &lidar_path)
{
    return pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, cloud_) != -1;
}
