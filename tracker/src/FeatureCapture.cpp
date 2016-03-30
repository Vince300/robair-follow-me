#include "FeatureCapture.hpp"

FeatureCapture::FeatureCapture()
    : frame(), descriptors(), keyPoints(), color()
{
}

FeatureCapture::FeatureCapture(const cv::Mat &mat)
    : frame(mat), descriptors(), keyPoints(), color()
{
}
