#include "student_functions.hpp"

void FindColors(
  const cv::Mat & input, const cv::Scalar & range_min, const cv::Scalar & range_max,
  cv::Mat & output)
{
  cv::Mat input_hsv;
  cv::cvtColor(input, input_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(input_hsv, range_min, range_max, output);
}

void ReprojectToGroundPlane(
  const cv::Mat & input, const cv::Mat & homography,
  const cv::Size & map_size, cv::Mat & output)
{
  cv::warpPerspective(
    input, output, homography, map_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT,
    cv::Scalar(127));
}
