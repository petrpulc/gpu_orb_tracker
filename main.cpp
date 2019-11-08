#include <string>
#include <opencv2/opencv.hpp>
#include "settings.h"
#include "Motion.h"

int frame_width = 0;
int frame_number = 0;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <filename> <number of points of interest>" << std::endl;
        return 1;
    }

    // gather filename
    const std::string filename(argv[1]);

    // declare GPU frame storage
    cv::Mat cpu_frame, cpu_gray;

    // declare feature point and description storage
    std::vector<cv::KeyPoint> points;
    cv::Mat cpu_desc;

    // setup ORB detector
    cv::Ptr<cv::ORB> cpu_detector = cv::ORB::create(std::stoi(argv[2]), 2.0f, OCTAVES, 31, 0, 2,
                                                    cv::ORB::HARRIS_SCORE, 31, 50);

    // open file on CPU
    cv::VideoCapture cpu_reader(filename);

    //introduce motion object
    Motion motion;

    // main loop
    for (;;) {
        if (!cpu_reader.read(cpu_frame))
            break;
        if (frame_width == 0) {
            frame_width = cpu_frame.cols;
        }

        //for each frame until end of input:
        // - convert to gray scale
        cv::cvtColor(cpu_frame, cpu_gray, cv::COLOR_BGRA2GRAY);
        // - detect feature points and compute their descriptors
        cpu_detector->detectAndCompute(cpu_gray, cv::noArray(), points, cpu_desc);

        motion.add_frame(points, cpu_desc);
        motion.show(cpu_gray);

        frame_number++;
    }

    return 0;
}
