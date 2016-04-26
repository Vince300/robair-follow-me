#include <iostream>

#include "Utils.hpp"
#include "VideoStream.hpp"

VideoStream::VideoStream(const openni::Device &device, openni::SensorType sensorType)
    : device(device), vs(), frame()
{
    // Create device capture
    NICHECK(vs.create(device, sensorType));

    vs.setMirroringEnabled(false);

    // Get video info
    vm = vs.getVideoMode();
    
    // Start capture
    vs.start();
}

VideoStream::~VideoStream()
{
    vs.stop();
}

int VideoStream::frameRate() const { return vm.getFps(); }
int VideoStream::rows() const { return vm.getResolutionY(); }
int VideoStream::cols() const { return vm.getResolutionX(); }

cv::Mat VideoStream::displayFrame(const cv::Mat &source) { return captureFrame(); }

void VideoStream::printStreamInfo(std::ostream &os)
{
    os << "Video stream(" << rows() << ", " << cols() << ") at " << frameRate() << " FPS" << std::endl;
}

const void *VideoStream::readFrameData()
{
    // Read frame from device
    vs.readFrame(&frame);

    // Return pointer to frame data
    return frame.getData();
}
