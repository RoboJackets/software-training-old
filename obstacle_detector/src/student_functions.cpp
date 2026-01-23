#include "student_functions.hpp"

cv::Mat FindColors(const cv::Mat input, const cv::Scalar range_min, const cv::Scalar range_max)
{
    cv::Mat input_hsv;
    cv::cvtColor(input, input_hsv, cv::COLOR_BGR2HSV);

    cv::Mat output(input.size(), CV_8UC1);

    cv::inRange(input_hsv, range_min, range_max, output);

    return output;
}

cv::Mat ReprojectToGroundPlane(const cv::Mat input,
                             const cv::Mat homography,
                             const cv::Size map_size)
{
    cv::Mat output(map_size, CV_8UC1);

    cv::warpPerspective(
        input, output, homography, map_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT,
        cv::Scalar(127));

    return output;
}