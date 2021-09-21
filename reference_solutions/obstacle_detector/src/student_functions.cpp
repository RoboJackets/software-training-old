#include "student_functions.hpp"

cv::Mat FindColors(const cv::Mat input, const cv::Scalar range_min, const cv::Scalar range_max)
{
  cv::Mat input_hsv;
  cv::cvtColor(input, input_hsv, cv::COLOR_BGR2HSV);
  cv::Mat output(input.size(), CV_8UC1);

  for (auto r = 0; r < input_hsv.rows; ++r) {
    for (auto c = 0; c < input_hsv.cols; ++c) {
      const auto input_color = input_hsv.at<cv::Vec3b>(r, c);
      if (input_color[0] >= range_min[0] && input_color[0] <= range_max[0] &&
        input_color[1] >= range_min[1] && input_color[1] <= range_max[1] &&
        input_color[2] >= range_min[2] && input_color[2] <= range_max[2])
      {
        output.at<uint8_t>(r, c) = 255;
      } else {
        output.at<uint8_t>(r, c) = 0;
      }
    }
  }
  return output;

  /*
   * Or, using the library functions
   */
  // cv::Mat input_hsv;
  // cv::cvtColor(input, input_hsv, cv::COLOR_BGR2HSV);
  // cv::Mat output;
  // cv::inRange(input_hsv, range_min, range_max, output);
  // return output;
}

cv::Mat ReprojectToGroundPlane(
  const cv::Mat input, const cv::Mat homography,
  const cv::Size map_size)
{
  cv::Mat output(map_size, CV_8UC1);
  for (auto y = 0; y < output.rows; ++y) {
    for (auto x = 0; x < output.cols; ++x) {
      const cv::Vec3d dest_vec(x, y, 1);
      const cv::Vec3d src_vec = cv::Mat1d(homography.inv() * dest_vec);
      const cv::Point2i dest_point(x, y);
      const cv::Point2i src_point(src_vec[0] / src_vec[2], src_vec[1] / src_vec[2]);
      if (src_point.inside(cv::Rect(cv::Point(), input.size()))) {
        output.at<uint8_t>(dest_point) = input.at<uint8_t>(src_point);
      } else {
        output.at<uint8_t>(dest_point) = 127;
      }
    }
  }
  return output;

  /*
   * Or, using the library functions
   */
  // cv::Mat output;
  // cv::warpPerspective(
  //   input, output, homography, map_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT,
  //   cv::Scalar(127));
  // return output;
}
