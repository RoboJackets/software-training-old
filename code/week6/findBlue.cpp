#include <unistd.h>
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

      // kernel that is an ellipse of size 11X11 pixels
      Mat erosion_kernel_blue = getStructuringElement(MORPH_ELLIPSE, Size(11, 11));

      //applying a GaussianBlur makes the image "fuzzier". This smoothes out
      //noisy pixels
      Mat frameBlurred;
      GaussianBlur(frameBGR, frameBlurred, Size{7,7}, 0);

      //We shift the mat which is encodes as an RGB image into an HSV image
      Mat frameHSV;
      cvtColor(frameBlurred, frameHSV, CV_BGR2HSV);

      //We initialize an empty Mat to store all the pixels that we identify
      //as blue. CV_8U is the image depth: each pixel is only one bit
      Mat output_blue = Mat::zeros(200, 640, CV_8U);

      //for every pixel in our frame that we define as blue, write a white
      //pixel to the output_blue Mat
      inRange(frameHSV, blue_low, blue_high, output_blue);

      //apply an erode operation onto the filtered image. This helps us
      //remove specks of blue that show up due to noise or glare
      erode(output_blue, output_blue, erosion_kernel_blue);

      //the logic to publish the result of the filtering in ROS would go here
    return output_blue;
}

int main() {
    // sets up the input stream to the default video device
    // /dev/video0 must exist
    VideoCapture stream1(0);

    while (true) {
        Mat cameraFrame;
        // starts reading in images from camera
        stream1.read(cameraFrame);

        // checks that the image is not empty
        if(!cameraFrame.empty()) {
            // creates a window to display the actual camera image
            imshow("camera", cameraFrame);
            Mat blue = findBlue(cameraFrame);

            // creates a window to display the blue parts of the image
            imshow("blue parts", blue);

            // waits and displays the image for 1 millisecond
            waitKey(1);
        } else {
            /* this only executes when your camera is not working
             * type
             *    ls /dev | grep video0
             * you should see
             *    video0 in printed to the terminal
            */
            std::cout << "No image recieved from camera" << std::endl;
            // waits for the camera to return an image
            while(cameraFrame.empty()) {
                stream1.read(cameraFrame);
            }
        }
    }
}
