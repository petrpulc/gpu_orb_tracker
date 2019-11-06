#include "Feature.h"

Feature::Feature(float *point, const cv::Mat &description) {
    // Relative motion in last frame; zero for new point
    d[0] = 0;
    d[1] = 0;
    // Position of detected point
    p[0] = point[0];
    p[1] = point[1];
    // Add point to history
    history[frame_number] = cv::Point2f(point[0], point[1]);
    // Store feature point descriptor
    desc = description;
    // Assign default uncertainty
    u = UNCERTAINTY;
}

void Feature::add_match(const float *point, const cv::Mat &description, const float *upper_estimation) {
    // Relative motion from last position with respect to layer above
    d[0] = point[0] - p[0] - upper_estimation[0];
    d[1] = point[1] - p[1] - upper_estimation[1];
    // Update point position
    p[0] = point[0];
    p[1] = point[1];
    // Store also in history
    history[frame_number] = cv::Point2f(point[0], point[1]);
    // Update point description
    desc = description;
    // Lower uncertainty
    if (u > MIN_UNCERTAINTY)
        u *= DELTA_UNCERTAINTY_GOOD;
}

void Feature::no_match(const float *upper_estimation) {
    // Update point position from upper estimation and last known motion vector
    p[0] += upper_estimation[0] + d[0];
    p[1] += upper_estimation[1] + d[1];
    // Store in history?
    if (STORE_UNCERTAIN) {
        history[frame_number] = cv::Point2f(p[0], p[1]);
    }
    // Raise uncertainty
    u *= DELTA_UNCERTAINTY_BAD;
}

void Feature::print() {
    std::map<int, cv::Point2f>::iterator frame;
    for (int frame_no = 0; frame_no < frame_count; frame_no++) {
        frame = history.find(frame_no);
        if (frame != history.end()) {
            std::cout << frame->second.x << "," << frame->second.y << ",";
        } else
            std::cout << ",,";
    }
    std::cout << std::endl;
}
