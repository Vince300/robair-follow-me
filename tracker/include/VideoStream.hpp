#ifndef _VIDEOSTREAM_HPP_
#define _VIDEOSTREAM_HPP_

#include <OpenNI.h>
#include <opencv2/core/core.hpp>

class VideoStream
{
protected:
    VideoStream(const openni::Device &device, openni::SensorType sensorType);

    virtual ~VideoStream();

public:
    int frameRate() const;
    int rows() const;
    int cols() const;

    virtual cv::Mat captureFrame() = 0;
    virtual cv::Mat displayFrame(const cv::Mat &source);

    virtual void VideoStream::printStreamInfo(std::ostream &os);

protected:
    const void *readFrameData();

private:
    const openni::Device &device;
    openni::VideoMode vm;
    openni::VideoStream vs;
    openni::VideoFrameRef frame;
};

#endif
