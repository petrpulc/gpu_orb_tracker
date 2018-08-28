#ifndef GPU_ORB_TRACKER_POINTLAYER_H
#define GPU_ORB_TRACKER_POINTLAYER_H

#include "Layer.h"
#include "PointCloud.h"
#include "nanoflann.hpp"
#include "Feature.h"
#include "VectorCloud.h"

// Nanoflann point cloud indexing tree
typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloud>,
        PointCloud, 2> Point2DTree;

// Nanoflann vector cloud indexing tree
typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, VectorCloud>,
        VectorCloud, 2> Vector2DTree;

class PointLayer : public Layer {
    // Point cloud storage
    PointCloud new_frame;
    Point2DTree *new_frame_index;

    // Vector cloud storage
    VectorCloud last_vectors;
    Vector2DTree *last_vectors_index;

public:
    // List of active features
    std::list<Feature> active;

    PointLayer() {
        // Initialization of indexes
        new_frame_index = new Point2DTree(2, new_frame, nanoflann::KDTreeSingleIndexAdaptorParams(5));
        last_vectors_index = new Vector2DTree(2, last_vectors, nanoflann::KDTreeSingleIndexAdaptorParams(2));
    }

    ~PointLayer() {
        delete new_frame_index;
        delete last_vectors_index;
    }

    void initialize(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc, unsigned int i_start,
                    unsigned int i_end) override;

    void estimate(float *in, float *out) override;

    void pair(Layer *layer_above) override;
};

#endif //GPU_ORB_TRACKER_POINTLAYER_H
