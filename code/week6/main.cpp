#include <opencv2/opencv.hpp>
#include <string>

int main() {
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

    // a single pixel is made up of 3 8 bit values
    // Blue, Green, Red
    std::cout << "my_image = \n" << my_image << std::endl;

    // iterate over an image the safe way
    for(auto it = my_image.begin<cv::Vec3b>(); it != my_image.end<cv::Vec3b>(); it++) {
        std::cout << "Blue = " << std::to_string((*it)[0]) << std::endl;
        std::cout << "Green = " << std::to_string((*it)[1]) << std::endl;
        std::cout << "Red = " << std::to_string((*it)[2]) << std::endl;
    }
    std::cout << "\n";

    // faster way to iterate over an image
    int channels = my_image.channels();
    std::cout << "my_image has " << channels << " channels" << std::endl;
    int nRows = my_image.rows;
    std::cout << "my_image has " << nRows << " rows" << std::endl;
    std::cout << "my_image has " << my_image.cols << " cols" << std::endl;
    // here we multiply by the number of channels since we will be dereferencing it with uchar
    // and each channel is a uchar
    int nCols = my_image.cols * channels;

    // if the image is stored continuously then we have only a single row of
    // length nRows * nCols
    if(my_image.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    for(int i = 0; i < nRows; i++) {
        // get pointer to the start of the current row
        uchar* current = my_image.ptr<uchar>(i);
        for(int j = 0; j < nCols; j++) {
            // prints out the value at the nCols in nRows
            std::cout << std::to_string(current[j]);
            // formats the printing of the statement
            if((j + 1) % 3 != 0) {
                std::cout << ",";
            } else {
                std::cout << std::endl;
            }
        }
    }

    // to access a single pixel use .at
    cv::Vec3b vector = my_image.at<cv::Vec3b>(2, 2);

    // reads in an image
    cv::Mat image = cv::imread("Jaymii_img.png", cv::IMREAD_COLOR);

    // check that the image was loaded
    if(!image.empty()) {
        cv::Vec3b image_vector = image.at<cv::Vec3b>(154,160);
        std::cout << "Blue = " << std::to_string(image_vector[0]) << std::endl;
        std::cout << "Green = " << std::to_string(image_vector[1]) << std::endl;
        std::cout << "Red = " << std::to_string(image_vector[2]) << std::endl;

        // converts image to grey scale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        // applies to laplacian kernel, a common technique for edge detection
        cv::Laplacian(image, image, image.depth(), 3, 1, 0, cv::BORDER_DEFAULT);

        // creates a window and shows the image until a key is pressed
        cv::namedWindow("Jaymii", cv::WINDOW_AUTOSIZE);
        cv::imshow("Jaymii", image);
        cv::waitKey(0);
    } else {
        std::cerr << "\nERROR\n" << std::endl;
        std::cerr << "Make sure that Jaymii.img is in the same directory as main when run" << std::endl;
        std::cerr << "\nERROR\n" << std::endl;
    }

    return 0;
}
