#ifndef GPU_ORB_TRACKER_MOTION_H
#define GPU_ORB_TRACKER_MOTION_H

#include "settings.h"
#include "Layer.h"
#include <opencv2/core/mat.hpp>

class Motion {

public:
    // Container for all processing layers
    std::vector<Layer *> layers;

    Motion();

    ~Motion();

    void add_frame(const std::vector<cv::KeyPoint> &points, const cv::Mat &descriptors);

    void show(const cv::Mat &frame);
};

#endif //GPU_ORB_TRACKER_MOTION_H
