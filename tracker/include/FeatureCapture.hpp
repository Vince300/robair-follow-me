#ifndef _FEATURECAPTURE_HPP_
#define _FEATURECAPTURE_HPP_

#include <vector>

#include <opencv2/core/core.hpp>

struct FeatureCapture
{
    FeatureCapture();
    FeatureCapture(const cv::Mat &mat);

    cv::Mat frame;
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keyPoints;
    cv::Scalar color;
};

#endif
