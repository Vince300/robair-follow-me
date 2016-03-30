#include <opencv2/imgproc/imgproc.hpp>

#include "VideoStream.hpp"
#include "ColorVideoStream.hpp"

ColorVideoStream::ColorVideoStream(const openni::Device &device) 
    : VideoStream(device, openni::SENSOR_COLOR)
{
}

ColorVideoStream::~ColorVideoStream()
{
}

cv::Mat ColorVideoStream::captureFrame()
{
    cv::Mat raw(rows(), cols(), CV_8UC3, static_cast<openni::RGB888Pixel*>(const_cast<void*>(readFrameData())));
    cv::Mat bgr(rows(), cols(), CV_8UC3);
    cv::cvtColor(raw, bgr, CV_RGB2BGR);
    return bgr;
}
