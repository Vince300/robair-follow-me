#include "FeatureCapture.hpp"

FeatureCapture::FeatureCapture()
    : frame(), mask(), mean(0)
{
}

FeatureCapture::FeatureCapture(const cv::Mat &frame, const cv::Mat &mask)
    : frame(frame), mask(mask), mean(0)
{
}
