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
    auto previous = history.find(0);
    std::map<int, cv::Point2f>::iterator current;
    for (int frame_no = 0; frame_no < frame_count; frame_no++) {
        current = history.find(frame_no);
        if (current != history.end() && previous != history.end()) {
            float delta_x = current->second.x - previous->second.x;
            float delta_y = current->second.y - previous->second.y;
            float angle_sin = std::sin(std::atan2(delta_y, delta_x));
            float delta_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
            std::cout << previous->second.x << "," << previous->second.y << "," << angle_sin << "," << delta_length
                      << ",";
        } else
            std::cout << ",,,,";
        previous = current;
    }
    std::cout << std::endl;
}
