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
    cv::cuda::GpuMat gpu_frame, gpu_gray, gpu_clahe;

    // declare feature point and description storage
    std::vector<cv::KeyPoint> points;
    cv::cuda::GpuMat gpu_desc;
    cv::Mat cpu_desc;

    // add local contrast enhancer
    cv::Ptr<cv::cuda::CLAHE> clahe = cv::cuda::createCLAHE(CONTRAST);

    // setup GPU-accelerated ORB detector
    cv::Ptr<cv::cuda::ORB> gpu_detector = cv::cuda::ORB::create(std::stoi(argv[2]), 2.0f, OCTAVES, 31, 0, 2,
                                                                cv::cuda::ORB::HARRIS_SCORE, 31, 50, true);

    // open file on GPU
    cv::Ptr<cv::cudacodec::VideoReader> gpu_reader = cv::cudacodec::createVideoReader(filename);

    //introduce motion object
    Motion motion;

    // main loop
    for (;;) {
        if (!gpu_reader->nextFrame(gpu_frame))
            break;
        if (frame_width == 0) {
            frame_width = gpu_frame.cols;
        }

        //for each frame until end of input:
        // - convert to gray scale
        cv::cuda::cvtColor(gpu_frame, gpu_gray, cv::COLOR_BGRA2GRAY);
        // - enhance contrast
        clahe->apply(gpu_gray, gpu_clahe);
        // - detect feature points and compute their descriptors
        gpu_detector->detectAndCompute(gpu_clahe, cv::noArray(), points, gpu_desc);
        // - download descriptors to CPU RAM
        gpu_desc.download(cpu_desc);

        motion.add_frame(points, cpu_desc);

        frame_number++;
    }

    return 0;
}
