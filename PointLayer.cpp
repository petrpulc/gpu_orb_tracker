#include "PointLayer.h"

void PointLayer::initialize(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc, unsigned int i_start,
                            unsigned int i_end) {
    // Update point cloud and build k-NN index
    new_frame.update(points, desc, i_start, i_end);
    new_frame_index->buildIndex();
}

void PointLayer::pair(Layer *layer_above) {
    // Clear vector cloud
    last_vectors.clear();

    // Storage for radius search results
    std::vector<std::pair<size_t, float>> ret_matches;
    // Default NanoFLANN parameters
    nanoflann::SearchParams params;

    // Whether match was found
    bool match_found;

    // Estimation of motion from upper layer and new position
    float upper_estimation[2], new_position[2];

    // For all active features
    auto i = active.begin();
    while (i != active.end()) {
        match_found = false;

        // Erase point if uncertainty too high
        if (i->u > MAX_UNCERTAINTY) {
            active.erase(i++);
            continue;
        }

        // Estimate new position
        layer_above->estimate(i->p, upper_estimation);
        new_position[0] = i->p[0] + i->d[0] + upper_estimation[0];
        new_position[1] = i->p[1] + i->d[1] + upper_estimation[1];

        // Search for nearest neighbours to the estimated position
        new_frame_index->radiusSearch(new_position, i->u * frame_width, ret_matches, params);
        for (auto &proposed:ret_matches) {
            if (new_frame.pts[proposed.first].used) {
                // Point already used :(
                continue;
            }
            if (cv::norm(i->desc, new_frame.pts[proposed.first].desc, cv::NORM_HAMMING) > 64) {
                // Point description not similar enough :(
                continue;
            }

            // Got a match!
            match_found = true;

            // Cannot be used again
            new_frame.pts[proposed.first].used = true;

            // Store vector for estimation in lower levels
            last_vectors.add(i->p, new_frame.pts[proposed.first].p);

            // Finally add a feature match and try another active feature
            i->add_match(new_frame.pts[proposed.first].p, new_frame.pts[proposed.first].desc, upper_estimation);
            break;
        }

        // No match for this feature at all
        if (!match_found) {
            i->no_match(upper_estimation);
        }

        ++i;
    }

    // Add all unused points as new features
    for (auto &new_point: new_frame.pts) {
        if (!new_point.used) {
            active.emplace_back(new_point.p, new_point.desc);
        }
    }

    // Build index used for estimation in lower levels
    last_vectors_index->buildIndex();
}

void PointLayer::estimate(float *in, float *out) {
    // Index of match from last_vectors
    size_t indices[1];

    // Unused value
    float distances[1];

    // Number of results to check if something was found
    size_t result;

    // Search for one nearest neighbour to position we make estimation for
    result = last_vectors_index->knnSearch(in, 1, indices, distances);

    if (result > 0) {
        // Return found motion vector
        out[0] = last_vectors.vectors[*indices].d[0];
        out[1] = last_vectors.vectors[*indices].d[1];
    } else {
        out[0] = 0;
        out[1] = 0;
    }
}

void PointLayer::draw(const cv::Mat &frame) {
    for (auto &i:active) {
        for (auto h = i.history.begin(), h_end = i.history.end();;) {
            auto h_old = h++;
            if (h == h_end) { break; }
            cv::arrowedLine(frame, h_old->second, h->second,
                        i.color, 1, cv::LINE_AA, 0, 0.2);
        }
    }
}