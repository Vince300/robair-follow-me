#include "VideoStream.hpp"
#include "DepthVideoStream.hpp"

DepthVideoStream::DepthVideoStream(const openni::Device &device)
    : VideoStream(device, openni::SENSOR_DEPTH), lpfMax(0.5)
{}

DepthVideoStream::~DepthVideoStream()
{}

cv::Mat DepthVideoStream::captureFrame()
{
    return cv::Mat(rows(), cols(), CV_16UC1, static_cast<openni::DepthPixel*>(const_cast<void*>(readFrameData())));
}

cv::Mat DepthVideoStream::displayFrame(const cv::Mat &source)
{
    // Apply low-pass filter on data
    double min;
    double max;
    cv::minMaxIdx(source, &min, &max);

    cv::Mat adjMap;
    cv::convertScaleAbs(source, adjMap, 255 / lpfMax.nextStep(max));
    return adjMap;
}
