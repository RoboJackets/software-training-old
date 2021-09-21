#ifndef STUDENT_FUNCTIONS_HPP
#define STUDENT_FUNCTIONS_HPP

#include <opencv2/opencv.hpp>

cv::Mat FindColors(const cv::Mat input, const cv::Scalar range_min, const cv::Scalar range_max);

cv::Mat ReprojectToGroundPlane(
  const cv::Mat input, const cv::Mat homography,
  const cv::Size map_size);

#endif
