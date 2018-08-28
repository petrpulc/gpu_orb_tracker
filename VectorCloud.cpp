#include "VectorCloud.h"

void VectorCloud::clear() {
    vectors.clear();
}

void VectorCloud::add(float *point, float *new_point) {
    vectors.emplace_back(point, new_point[0]-point[0], new_point[1]-point[1]);
}
