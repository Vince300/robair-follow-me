#ifndef _DEPTHVIDEOSTREAM_HPP_
#define _DEPTHVIDEOSTREAM_HPP_

#include "VideoStream.hpp"
#include "LowPassFilter.hpp"

class DepthVideoStream : public VideoStream
{
public:
    DepthVideoStream(const openni::Device &device);

    ~DepthVideoStream() override;

    cv::Mat captureFrame() override;

    cv::Mat displayFrame(const cv::Mat &source) override;

private:
    LowPassFilter<double> lpfMax;
};

#endif
