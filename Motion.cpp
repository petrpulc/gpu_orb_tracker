#include "Motion.h"
#include "PointLayer.h"
#include "HomographyLayer.h"

Motion::Motion() {
    // Initialize same number of Point layers as octaves
    for (int i = 0; i < OCTAVES; i++) {
        this->layers.push_back(new PointLayer);
    }
    // Add Homography layer on top
    this->layers.push_back(new HomographyLayer);
}


Motion::~Motion() {
    for (auto &layer: layers) {
        delete layer;
    }
}

void Motion::add_frame(const std::vector<cv::KeyPoint> &points, const cv::Mat &descriptors) {
    unsigned octave_start = 0, current_octave = 0, i;

    // Find split points and store data from each octave in respective layer
    for (i = 0; i < points.size(); i++) {
        if (points[i].octave != current_octave) {
            layers[current_octave]->initialize(points, descriptors, octave_start, i);
            octave_start = i;
            ++current_octave;
        }
    }
    layers[current_octave]->initialize(points, descriptors, octave_start, i);

    // Pass data from last layer to homography layer as well
    layers[current_octave + 1]->initialize(points, descriptors, octave_start, i);

    // Pair new frame in all point layers from top to bottom
    for (i = OCTAVES; i > 0; i--) {
        layers[i - 1]->pair(layers[i]);
    }
}
