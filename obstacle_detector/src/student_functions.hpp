#ifndef STUDENT_FUNCTIONS_HPP  // Check that the unique macro has not already been defined
#define STUDENT_FUNCTIONS_HPP  // Define the unique macro

#include <opencv2/opencv.hpp>

cv::Mat FindColors(const cv::Mat input,
                   const cv::Scalar range_min, 
                   const cv::Scalar range_max);

cv::Mat ReprojectToGroundPlane(const cv::Mat input,
                             const cv::Mat homography,
                             const cv::Size map_size);

#endif  // Close the pre-processor if block