#ifndef _FEATURECAPTURE_HPP_
#define _FEATURECAPTURE_HPP_

#include <vector>

#include <opencv2/core/core.hpp>

struct FeatureCapture
{
    FeatureCapture();
    FeatureCapture(const cv::Mat &frame, const cv::Mat &mask);

    cv::Mat frame;
    cv::Mat mask;
    cv::Scalar mean;

    int poi_x;
    int poi_y;
};

#endif
