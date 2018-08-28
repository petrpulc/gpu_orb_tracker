#ifndef GPU_ORB_TRACKER_FEATURE_H
#define GPU_ORB_TRACKER_FEATURE_H

extern int frame_width;
extern int frame_number;

#include <opencv2/opencv.hpp>
#include <map>
#include "settings.h"

struct Feature {
    // Position
    float p[2]; // Estimated point coordinates
    float d[2]; // Last known point delta
    std::map<int, cv::Point2f> history; // History of point coordinates
    float u; // Point uncertainty relative to frame size

    // ORB descriptor
    cv::Mat desc;

    Feature(float *point, const cv::Mat &description);

    void add_match(const float *point, const cv::Mat &description, const float *upper_estimation);

    void no_match(const float *upper_estimation);
};

#endif //GPU_ORB_TRACKER_FEATURE_H
