#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) 
    {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_points;
    for (size_t i = 0; i < control_points.size() - 1; ++i) 
    {
        cv::Point2f p1 = control_points[i], p2 = control_points[i + 1];
        cv::Point2f new_point = (1 - t) * p1 + t * p2;
        new_points.push_back(new_point);
    }
    return recursive_bezier(new_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    bool antialiasing = true;
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        if (!antialiasing)
            window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        else
        {
            cv::Point2i u00 = {static_cast<int>(point.x), static_cast<int>(point.y)};
            cv::Point2i u01 = {static_cast<int>(point.x), static_cast<int>(point.y + 1)};
            cv::Point2i u10 = {static_cast<int>(point.x + 1), static_cast<int>(point.y)};
            cv::Point2i u11 = {static_cast<int>(point.x + 1), static_cast<int>(point.y + 1)};

            // 计算(x,y)与四个整数点的相对位置
            float dx = point.x - u00.x; // [0,1]范围内
            float dy = point.y - u00.y; // [0,1]范围内

            // 正确计算权重
            float w00 = (1-dx) * (1-dy);
            float w01 = (1-dx) * dy;
            float w10 = dx * (1-dy);
            float w11 = dx * dy;

            // 权重总和必为1，直接应用
            window.at<cv::Vec3b>(u00.y, u00.x)[1] += 255 * w00;
            window.at<cv::Vec3b>(u01.y, u01.x)[1] += 255 * w01;
            window.at<cv::Vec3b>(u10.y, u10.x)[1] += 255 * w10;
            window.at<cv::Vec3b>(u11.y, u11.x)[1] += 255 * w11;

            window.at<cv::Vec3b>(u00.y, u00.x)[1] = std::min(window.at<cv::Vec3b>(u00.y, u00.x)[1], static_cast<unsigned char>(255));
            window.at<cv::Vec3b>(u01.y, u01.x)[1] = std::min(window.at<cv::Vec3b>(u01.y, u01.x)[1], static_cast<unsigned char>(255));
            window.at<cv::Vec3b>(u10.y, u10.x)[1] = std::min(window.at<cv::Vec3b>(u10.y, u10.x)[1], static_cast<unsigned char>(255));
            window.at<cv::Vec3b>(u11.y, u11.x)[1] = std::min(window.at<cv::Vec3b>(u11.y, u11.x)[1], static_cast<unsigned char>(255));
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("../my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
