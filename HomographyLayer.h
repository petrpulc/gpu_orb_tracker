#ifndef GPU_ORB_TRACKER_HOMOGRAPHYLAYER_H
#define GPU_ORB_TRACKER_HOMOGRAPHYLAYER_H

#include <opencv2/core/mat.hpp>
#include "Layer.h"

class HomographyLayer : public Layer {
    // Features in last frame
    std::vector<cv::Point2f> last_frame_points;
    cv::Mat last_frame_desc;

    // Computed homography matrix
    cv::Mat homography;

public:
    void initialize(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc, unsigned int i_start,
                    unsigned int i_end) override;

    void estimate(float *in, float *out) override;

};

#endif //GPU_ORB_TRACKER_HOMOGRAPHYLAYER_H
