#include <opencv2/opencv.hpp>
#include <string.h>
#include <stdlib.h>

int main() {
    using namespace std;

    // Mat http://docs.opencv.org/3.3.0/d3/d63/classcv_1_1Mat.html
    // this creates an image header. The array is not allocated yet
    cv::Mat my_image;

    /* This creates a matrix
     * cv::Mat(rows, cols, type, default color)
     * type: CV_[The number of bits per item][Signed or Unsigned][Type Prefix]C[The channel number]
     * this type CV_8UC3
     * 8 is the number of bits per item
     * U means that it is unsigned
     * C3 means that the color channel has three entires (BGR)
    */
    my_image = cv::Mat(2, 2, CV_8UC3, cv::Scalar(0,0,255));

    // a sinlge pixel is made up of 3 8 bit values
    // Blue, Green, Red
    cout << "my_image = \n" << my_image << endl;

    // iterate over n image
    cv::MatIterator_<cv::Vec3b> test;
    for(cv::MatIterator_<cv::Vec3b> it = my_image.begin<cv::Vec3b>(); it != my_image.end<cv::Vec3b>(); it++) {
        cout << "Blue = " << to_string((*it)[0]) << endl;
        cout << "Green = " << to_string((*it)[1]) << endl;
        cout << "Red = " << to_string((*it)[2]) << endl;
    }

    // to access a single pixel use .at
    cv::Vec3b vector = my_image.at<cv::Vec3b>(2, 2);

    // reads in an image
    cv::Mat image = cv::imread("igvc_gazebo.png", cv::IMREAD_COLOR);

    // performs a gaussian blur
    GaussianBlur(image, image, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

    // converts image to grey scale
    cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // applies to laplacian kernel
    cv::Laplacian(image, image, image.depth(), 3, 1, 0, cv::BORDER_DEFAULT);

    // creates a window and shows the image until 0 is pressed
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
    cv::imshow("test", image);
    cv::waitKey(0);
}
