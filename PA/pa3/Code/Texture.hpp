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
        // auto u_img = u * width;
        // auto v_img = (1 - v) * height;
        // auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        // return Eigen::Vector3f(color[0], color[1], color[2]);
        return getColorBilinear(u, v);
    }

private:
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;
        
        // find the four surrounding pixels
        int x0 = std::floor(u_img);
        int y0 = std::floor(v_img);
        int x1 = std::min(x0 + 1, width - 1); 
        int y1 = std::min(y0 + 1, height - 1);

        // Ensure the lower bounds are also valid
        x0 = std::max(0, x0);
        y0 = std::max(0, y0);

        // Compute the fractional part's ratio
        float u_ratio = u_img - x0;
        float v_ratio = v_img - y0;

        // Get the colors of the four corners
        auto c00 = image_data.at<cv::Vec3b>(y0, x0);
        auto c10 = image_data.at<cv::Vec3b>(y0, x1);
        auto c01 = image_data.at<cv::Vec3b>(y1, x0);
        auto c11 = image_data.at<cv::Vec3b>(y1, x1);

        // Perform bilinear interpolation
        // First interpolate in the horizontal direction
        Eigen::Vector3f color00(c00[0], c00[1], c00[2]);
        Eigen::Vector3f color10(c10[0], c10[1], c10[2]);
        Eigen::Vector3f color01(c01[0], c01[1], c01[2]);
        Eigen::Vector3f color11(c11[0], c11[1], c11[2]);

        Eigen::Vector3f c0 = (1 - u_ratio) * color00 + u_ratio * color10;
        Eigen::Vector3f c1 = (1 - u_ratio) * color01 + u_ratio * color11;

        // Then interpolate in the vertical direction
        Eigen::Vector3f result_color = (1 - v_ratio) * c0 + v_ratio * c1;
        return result_color;
    }

};

#endif //RASTERIZER_TEXTURE_H
