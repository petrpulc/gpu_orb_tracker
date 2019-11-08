#include <opencv2/opencv.hpp>
#include "HomographyLayer.h"

void HomographyLayer::initialize(const std::vector<cv::KeyPoint> &points, const cv::Mat &desc, unsigned int i_start,
                                 unsigned int i_end) {

    // Not first frame
    if (!last_frame_points.empty()) {
        // Prepare new frame descriptor matrix header
        cv::Mat new_frame_desc = desc.rowRange(i_start, i_end);

        // Match points based on descriptions
        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        matcher.match(new_frame_desc, last_frame_desc, matches, cv::noArray());

        // Filter good matches (threshold of 64) and store corresponding points
        std::vector<cv::Point2f> last_frame_matched, new_frame_matched;
        for (auto &match: matches) {
            if (match.distance > 64) continue;
            last_frame_matched.push_back(last_frame_points[match.trainIdx]);
            new_frame_matched.push_back(points[i_start + match.queryIdx].pt);
        }

        // Find homography, identity if fails
        if (last_frame_matched.empty()) {
            homography = cv::Mat::eye(3, 3, CV_64F);
        } else {
            homography = cv::findHomography(last_frame_matched, new_frame_matched, cv::RANSAC, 0.1);
        }
        if (homography.empty()) {
            homography = cv::Mat::eye(3, 3, CV_64F);
        }
    }

    // Store only KeyPoint.pt
    last_frame_points.clear();
    for (unsigned i = i_start; i < i_end; ++i) {
        last_frame_points.push_back(points[i].pt);
    }

    // Slice section of descriptions
    desc.rowRange(i_start, i_end).copyTo(last_frame_desc);
}

void HomographyLayer::estimate(float *in, float *out) {
    // Return zero vector on invalid homography
    if (homography.empty()) {
        out[0] = 0;
        out[1] = 0;
        return;
    }

    // Auxiliary matrices for multiplication
    cv::Mat point(3, 1, CV_64F), result(3, 1, CV_64F);

    // Fill input matrix
    point.at<double>(0, 0) = in[0];
    point.at<double>(1, 0) = in[1];
    point.at<double>(2, 0) = 1;

    // Compute new position
    result = homography * point;

    // Extract x,y from result matrix
    out[0] = (float) result.at<double>(0, 0) - in[0];
    out[1] = (float) result.at<double>(1, 0) - in[1];
}
