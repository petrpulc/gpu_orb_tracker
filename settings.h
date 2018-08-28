#ifndef GPU_ORB_TRACKER_CONSTS_H
#define GPU_ORB_TRACKER_CONSTS_H

#define OCTAVES 3

#define UNCERTAINTY 0.02f //Default uncertainty for new points
#define MIN_UNCERTAINTY 0.02f //Minimal allowed uncertainty
#define MAX_UNCERTAINTY 0.03f //Maximal allowed uncertainty - if bigger, point is considered lost
#define DELTA_UNCERTAINTY_GOOD 0.99f //Change of uncertainty if match found
#define DELTA_UNCERTAINTY_BAD 1.1f //Change of uncertainty if no match found

#endif //GPU_ORB_TRACKER_CONSTS_H
