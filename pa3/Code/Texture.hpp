//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0) u = 0;
        if (v < 0) v = 0;
        if (u > 1) u = 1;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilner(float u, float v) {
        if (u < 0) u = 0;
        if (v < 0) v = 0;
        if (u > 1) u = 1;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float u0 = std::max(1.0,floor(u_img-0.5)), u1 = floor(u_img+0.5);
        float v0 = std::max(1.0,floor(v_img-0.5)), v1 = floor(v_img+0.5);
        float s = (u_img-u0)/(u1-u0);
        float t = (v_img-v0)/(v1-v0);


        auto color00 = image_data.at<cv::Vec3b>(v0, u0);
        auto color01 = image_data.at<cv::Vec3b>(v0, u1);
        auto color10 = image_data.at<cv::Vec3b>(v1, u0);
        auto color11 = image_data.at<cv::Vec3b>(v1, u1);
        auto color0 = color00 + s*(color01 - color00);
        auto color1 = color10 + s*(color11 - color10);
        auto color = color0 + t*(color1-color0);
        return Eigen::Vector3f(color[0], color[1], color[2]);

    }

};
#endif //RASTERIZER_TEXTURE_H
