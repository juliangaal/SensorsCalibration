#pragma once

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

struct Gui
{
public:
    Gui(cv::Mat &img);

    ~Gui() = default;

    void render();
private:
    cv::Mat& current_frame_;
};