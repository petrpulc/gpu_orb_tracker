#ifndef GPU_ORB_TRACKER_VECTORCLOUD_H
#define GPU_ORB_TRACKER_VECTORCLOUD_H

#include <opencv2/core/types.hpp>

struct VectorCloud {
    struct Vector {
        float p[2]; // initial position
        float d[2]; // deltas of the vector

        Vector(const float *point, float dx, float dy) {
            p[0] = point[0];
            p[1] = point[1];
            d[0] = dx;
            d[1] = dy;
        };
    };

    // Internal vector storage
    std::vector<Vector> vectors;

    void clear();

    void add(float *point, float *new_point);

    //Definitions used internally by the NanoFLANN k-D tree
    inline size_t kdtree_get_point_count() const { return vectors.size(); }

    inline float kdtree_distance(const float *p1, const size_t idx_p2, size_t) const {
        float dx = p1[0] - vectors[idx_p2].p[0];
        float dy = p1[1] - vectors[idx_p2].p[1];
        return sqrtf(dx * dx + dy * dy);
    }

    inline float kdtree_get_pt(const size_t idx, int dim) const {
        return vectors[idx].p[dim];
    }

    template<class BBOX>
    inline bool kdtree_get_bbox(BBOX &) const { return false; }
};

#endif //GPU_ORB_TRACKER_VECTORCLOUD_H
