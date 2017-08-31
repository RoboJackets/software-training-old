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

    // iterate over an image
    for(auto it = my_image.begin<cv::Vec3b>(); it != my_image.end<cv::Vec3b>(); it++) {
        cout << "Blue = " << to_string((*it)[0]) << endl;
        cout << "Green = " << to_string((*it)[1]) << endl;
        cout << "Red = " << to_string((*it)[2]) << endl;
    }
    cout << "\n";

    // to access a single pixel use .at
    cv::Vec3b vector = my_image.at<cv::Vec3b>(2, 2);

    // reads in an image
    cv::Mat image = cv::imread("Jaymii_img.png", cv::IMREAD_COLOR);

    // check that the iamge was loaded
    if(!image.empty()) {
        cv::Vec3b image_vector = image.at<cv::Vec3b>(154,160);
        cout << "Blue = " << to_string(image_vector[0]) << endl;
        cout << "Green = " << to_string(image_vector[1]) << endl;
        cout << "Red = " << to_string(image_vector[2]) << endl;

        // converts image to grey scale
        cvtColor(image, image, cv::COLOR_BGR2GRAY);

        // applies to laplacian kernel, a common technique for edge detection
        cv::Laplacian(image, image, image.depth(), 3, 1, 0, cv::BORDER_DEFAULT);

        // creates a window and shows the image until a key is pressed
        cv::namedWindow("Jaymii", cv::WINDOW_AUTOSIZE);
        cv::imshow("Jaymii", image);
        cv::waitKey(0);
    }

    return 0;
}
