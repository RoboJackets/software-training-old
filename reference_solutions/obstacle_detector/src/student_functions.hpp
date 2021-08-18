#ifndef STUDENT_FUNCTIONS_HPP
#define STUDENT_FUNCTIONS_HPP

#include <opencv2/opencv.hpp>

void FindColors(
  const cv::Mat & input, const cv::Scalar & range_min, const cv::Scalar & range_max,
  cv::Mat & output);

void ReprojectToGroundPlane(
  const cv::Mat & input, const cv::Mat & homography,
  const cv::Size & map_size, cv::Mat & output);

#endif  // STUDENT_FUNCTIONS_HPP
