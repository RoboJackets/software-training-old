#include <STSL/RJRobot.h>


int tick_time;
float power;
int min_size;
float min_circularity;
int min_value;


// direction: < 0 will turn counter-clockwise, > 0 will turn clockwise. 0 does nothing
void scoot(RJRobot& r, int direction) {
    if (direction == 0) {
        return;
    }
    float sign = (direction < 0) ? -1.0 : 1.0;
    
    r.setDriveMotors(sign * power, sign * power);
    r.wait(std::chrono::duration_cast<std::chrono::microseconds>(operator""ms(tick_time)));
    r.stopMotors();
}

int find_circle_column(const std::vector<std::vector<cv::Point>>& contours) {
    bool found_circle = false;
        
    int most_circular = 0;
    double best_circularity = 0;
    for (int i = 0; i < contours.size(); i++) {
        const auto& contour = contours[i];
        
        double area = cv::contourArea(contour);
        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * M_PI * area / std::pow(perimeter, 2);
        
        if (circularity > min_circularity && circularity > best_circularity && area > min_size) {
            best_circularity = circularity;
            most_circular = i;
            found_circle = true;
        }
    }
    
    if (found_circle) {
        cv::Moments moments = cv::moments(contours[most_circular]);
        double best_center_x = moments.m10 / moments.m00;
        return static_cast<int>(best_center_x);
    } else {
        return -1;
    }
}

int main(int argc, char** argv) {
    const int n = 5;
    if (argc != n + 1) {
        std::cout << "error: needs exactly " << n << " arguments" << std::endl;
        return 1;
    }
    
    tick_time = std::atoi(argv[1]);
    power = std::atof(argv[2]);
    min_size = std::atoi(argv[3]);
    min_circularity = std::atof(argv[4]);
    min_value = std::atoi(argv[5]);
    
    RJRobot robot;
    
    std::cout << "[STATUS] Robot battery level is " << robot.getBatteryVoltage() << "v" << std::endl;
    
    while (true) {
        auto img = robot.getImage();
        
        // blur
        cv::GaussianBlur(img, img, cv::Size(7,7), 3, 3);
        
        // convert to HSV
        cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
        
        // threshold
        cv::inRange(img, cv::Scalar(0,0,0), cv::Scalar(1000, 1000, min_value), img);
        
        // find outlines/contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        
        int circle_column = find_circle_column(contours);
        if (circle_column >= 0) {
            int offset = circle_column - (img.cols / 2);
            std::cout << "offset is " << offset << std::endl;
            if (std::abs(offset) > 40) {
                scoot(robot, offset);
            }
        } else {
            std::cout << "no circles found" << std::endl;
        }
    }
    
    return 0;
}
