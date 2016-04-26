#ifndef _APP_HPP_
#define _APP_HPP_

#define WINDOW_TITLE "Person tracker - FabLab Project"
#include "TargetTracker.hpp"

typedef enum
{
    Video,
    Depth
} WindowDisplayMode;

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <OpenNI.h>

#include "VideoStream.hpp"
#include "MotorDriver.hpp"

class Application
{
public:
    Application();

    ~Application();

    void run();

    VideoStream &cvs();

    // Used to redirect the mouse events to the app instance
    friend void onMouse(int evt, int x, int y, int flags, void *userdata);

protected:
    // The actual mouse callback
    void onMouse(int evt, int x, int y, int flags);

private:
    cv::Mat lastVideoFrame;
    cv::Mat lastDepthFrame;

    // Mouse button status
    bool mouseDown;

    // Depth capture device
    openni::Device device;

    // The current stream to display
    WindowDisplayMode mode;

    // The video streams
    std::vector<std::shared_ptr<VideoStream> > videoStreams;

    // The user tracker in use
    std::shared_ptr<TargetTracker> targetTracker;

    // The motor driver
    MotorDriver motorDriver;
};


#endif
