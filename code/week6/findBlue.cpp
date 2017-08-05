#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;


Mat findBlue(const Mat& frameBGR) { 
      //cv::Scalar is a container that is used to hold pixel values
      //the values shown below are tuned by hand on example data that is
      //gathered
      const Scalar blue_low{78, 50, 70};     //these are the lowest HSV values
                                             //that we accept as blue 
      const Scalar blue_high{138, 255, 255}; //these are the highest HSV
                                             //values we accept as blue
      //applying a GaussianBlur makes the image "fuzzier". This smoothes out
      //noisy pixels
      Mat frameBlurred;
      GaussianBlur(frameBGR, frameBlurred, Size{7,7}, 0);
      //We shift the mat which is encodes as an RGB image into an HSV image
      Mat frameHSV;
      cvtColor(frameBlurred, frameHSV, CV_BGR2HSV); 
      //We initialize an empty Mat to store all the pixels that we identify
      //as blue. CV_8U is the image depth: each pixel is only one bit
      Mat output_blue = Mat::zeros(mask.height, mask.width, CV_8U);
      //for every pixel in our frame that we define as blue, write a white
      //pixel to the output_blue Mat 
      inRange(frame_masked, blue_low, blue_high, output_blue);
      //apply an erode operation onto the filtered image. This helps us
      //remove specks of blue that show up due to noise or glare
      erode(output_blue, output_blue, erosion_kernel_blue);
      //the logic to publish the result of the filtering in ROS would go here
    return output_blue;
}

int main() {
    VideoCapture stream1(0);
    while (true) {
        Mat cameraFrame;
        stream1.read(cameraFrame);
        imshow("camera". cameraFrame);
        Mat blue = findBlue(cameraFrame);
        imshow("blue parts", blue);
    }
}
