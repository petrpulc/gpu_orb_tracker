#include "PointCloud.h"

void PointCloud::update(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc,
                        unsigned i_start, unsigned i_end) {
    // Resize container
    unsigned count = i_end - i_start;
    pts.resize(count);

    // Replace data
    for (unsigned i = 0; i < count; i++) {
        pts[i].p[0] = points[i_start + i].pt.x;
        pts[i].p[1] = points[i_start + i].pt.y;
        pts[i].desc = desc.row(i_start + i);
        pts[i].used = false;
    }
}
