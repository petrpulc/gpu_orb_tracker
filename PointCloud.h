#ifndef GPU_ORB_TRACKER_POINTCLOUD_H
#define GPU_ORB_TRACKER_POINTCLOUD_H

#include <opencv2/opencv.hpp>

struct PointCloud {
    struct Point {
        float p[2];         // position
        cv::Mat desc;       // key point description
        bool used = false;  // whether the point had match (was used)
    };

    // Internal point storage
    std::vector<Point> pts;

    // Set given key points with their descriptors to point cloud
    void update(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc, unsigned i_start, unsigned i_end);

    // Definitions used internally by the NanoFLANN k-D tree
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline float kdtree_distance(const float *p1, const size_t idx_p2, size_t) const {
        float dx = p1[0] - pts[idx_p2].p[0];
        float dy = p1[1] - pts[idx_p2].p[1];
        return sqrtf(dx * dx + dy * dy);
    }

    inline float kdtree_get_pt(const size_t idx, int dim) const {
        return pts[idx].p[dim];
    }

    template<class BBOX>
    inline bool kdtree_get_bbox(BBOX &) const { return false; }
};

#endif //GPU_ORB_TRACKER_POINTCLOUD_H
