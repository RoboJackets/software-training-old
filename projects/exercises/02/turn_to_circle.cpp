#include <STSL/RJRobot.h>


int tick_time;
float power;
int min_size;
float min_circularity;
int max_value;


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

        // find area
        double area = 0;

        // find perimeter
        double perimeter = 0;

        // find circularity (use M_PI for pi)
        double circularity = 0;

        if (circularity > min_circularity && circularity > best_circularity && area > min_size) {
            best_circularity = circularity;
            most_circular = i;
            found_circle = true;
        }
    }

    if (found_circle) {
        // find the moments of the chosen circle and use these to calculate the mean
        double center_x = 0;

        return static_cast<int>(center_x);
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
    max_value = std::atoi(argv[5]);

    RJRobot robot;

    std::cout << "[STATUS] Robot battery level is " << robot.getBatteryVoltage() << "v" << std::endl;

    while (true) {
        cv::Mat img = robot.getImage();

        // blur

        // convert to HSV

        // threshold

        // find outlines/contours
        std::vector<std::vector<cv::Point>> contours;

        int circle_column = find_circle_column(contours);
        if (circle_column >= 0) {
            // turn if necessary
        } else {
            std::cout << "no circles found" << std::endl;
        }

        // cv::imwrite("img.jpg", img);
        // robot.wait(100ms);
    }

    return 0;
}
