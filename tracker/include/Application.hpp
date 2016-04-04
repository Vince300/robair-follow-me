#ifndef _APP_HPP_
#define _APP_HPP_

#define WINDOW_TITLE "Person tracker - FabLab Project"

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
#include "FeatureCapture.hpp"

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
    // Detects features inside a specific radius around a mouse event
    void updateRegionOfInterest(int x, int y);

    // The actual mouse callback
    void onMouse(int evt, int x, int y, int flags);

private:
    cv::Mat lastVideoFrame;
    cv::Mat lastDepthFrame;

    // Known keypoints
    FeatureCapture tracking;

    // Mouse button status
    bool mouseDown;

    // Depth capture device
    openni::Device device;

    // The current stream to display
    WindowDisplayMode mode;

    // The video streams
    std::vector<std::shared_ptr<VideoStream>> videoStreams;
};


#endif
