#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat findBlue(const cv::Mat& frameBGR) {
      //cv::Scalar is a container that is used to hold pixel values
      //the values shown below are tuned by hand on example data that is
      //gathered
      const cv::Scalar blue_low{78, 50, 70};     //these are the lowest HSV values
                                             //that we accept as blue
      const cv::Scalar blue_high{138, 255, 255}; //these are the highest HSV
                                             //values we accept as blue

      // kernel that is an ellipse of size 11X11 pixels
      cv::Mat erosion_kernel_blue = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));

      //applying a GaussianBlur makes the image "fuzzier". This smoothes out
      //noisy pixels
      cv::Mat frameBlurred;
      cv::GaussianBlur(frameBGR, frameBlurred, cv::Size{7,7}, 0);

      //We shift the mat which is encoded as an RGB image into an HSV image
      cv::Mat frameHSV;
      cv::cvtColor(frameBlurred, frameHSV, CV_BGR2HSV);

      //We initialize an empty Mat to store all the pixels that we identify
      //as blue. CV_8U is the image depth: each pixel is only one byte
      cv::Mat output_blue = cv::Mat::zeros(200, 640, CV_8U);

      //for every pixel in our frame that we define as blue, write a white
      //pixel to the output_blue Mat
      cv::inRange(frameHSV, blue_low, blue_high, output_blue);

      //apply an erode operation onto the filtered image. This helps us
      //remove specks of blue that show up due to noise or glare
      cv::erode(output_blue, output_blue, erosion_kernel_blue);

      return output_blue;
}

int main() {
    // sets up the input stream to the video findBlueDemo.mp4
    // the file must be in the local directory
    cv::VideoCapture video("findBlueDemo.mp4");

    if(!video.isOpened()) {
        std::cerr << "Error opening video stream or file" << std::endl;
        std::cerr << "Verify that findBlueDemo.mp4 is in the local directory" << std::endl;
        return -1;
    }

    // this checks to see if a key has been pressed and then shows the image for
    // 300 milisecond so you can see what is happening
    cv::Mat cameraFrame;
    while (static_cast<char>(cv::waitKey(300)) != 'q') {
        // starts reading in images from camera
        video.read(cameraFrame);

        // checks that the image is not empty
        if(!cameraFrame.empty()) {
            // creates a window to display the actual camera image
            cv::imshow("camera", cameraFrame);
            cv::Mat blue = findBlue(cameraFrame);

            // creates a window to display the blue parts of the image
            cv::imshow("blue parts", blue);
        } else {
            // gets the frame number
            int frameCnt = video.get(CV_CAP_PROP_FRAME_COUNT);
            // gets the next image
            video.read(cameraFrame);
            // if it is the same frame count
            if(frameCnt == video.get(CV_CAP_PROP_FRAME_COUNT)) {
                // resets the video to the first frame
                video.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
            }
        }

    }
    // releases the video capture object
    video.release();
    // stops all windows
    cv::destroyAllWindows();

    return 0;
}
