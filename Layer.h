#ifndef GPU_ORB_TRACKER_LAYER_H
#define GPU_ORB_TRACKER_LAYER_H

#include <vector>
#include <opencv2/opencv.hpp>

class Layer {
public:
    // Initialize layer from an appropriate slice of incoming points of interest
    virtual void initialize(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc, unsigned int i_start,
                            unsigned int i_end) {};

    // Estimate motion vector for given point
    virtual void estimate(float *in, float *out) {};

    // Pair points with estimation from layer above
    virtual void pair(Layer *layer_above) {};
};

#endif //GPU_ORB_TRACKER_LAYER_H
