#include <iostream>
#include <iomanip>
#include <boost/timer.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Utils.hpp"
#include "DepthVideoStream.hpp"
#include "ColorVideoStream.hpp"

#include "Application.hpp"
#include <SlidingWindowTracker.hpp>

void onMouse(int evt, int x, int y, int flags, void *userdata)
{
    static_cast<Application*>(userdata)->onMouse(evt, x, y, flags);
}

Application::Application()
    : mouseDown(false), 
    device(), 
    mode(Video), 
    videoStreams(), 
    targetTracker(std::make_shared<SlidingWindowTracker>()),
    motorDriver()
{
    // Init OpenNI device
    openni::OpenNI::initialize();
    NICHECK(device.open(openni::ANY_DEVICE));

    // Init video streams
    videoStreams.push_back(std::static_pointer_cast<VideoStream>(std::make_shared<ColorVideoStream>(device)));
    videoStreams.push_back(std::static_pointer_cast<VideoStream>(std::make_shared<DepthVideoStream>(device)));

    motorDriver.setTargetTracker(targetTracker);
}

Application::~Application()
{
    // Destroy streams
    videoStreams.clear();

    // Shutdown OpenNI
    openni::OpenNI::shutdown();
}

void Application::run()
{
    // OpenCV window
    cv::namedWindow(WINDOW_TITLE, cv::WINDOW_AUTOSIZE);

    // Setup mouse callback
    cv::setMouseCallback(WINDOW_TITLE, ::onMouse, this);

    boost::timer tmstart;

    bool exit = false;
    int frameId = 0;

    while (!exit)
    {
        boost::timer timer;
        auto &vs = cvs();

        // Capture frames
        auto frame = vs.captureFrame();
        lastDepthFrame = (mode == Video ? videoStreams[Depth]->captureFrame() : frame);
        lastVideoFrame = (mode == Depth ? videoStreams[Video]->captureFrame() : frame);

        // Get frame ready for displaying
        auto displayFrame = vs.displayFrame(frame);

        // Update the target tracker
        if (targetTracker)
        {
            targetTracker->updateTarget(lastDepthFrame, lastVideoFrame);
            targetTracker->updateDisplayFrame(displayFrame);
        }

        // Display resulting image
        cv::imshow(WINDOW_TITLE, displayFrame);

        // Wait right amount of time not to exceed frame rate
        double targetPeriod = 1.0 / vs.frameRate();
        auto delayMs = (int)((targetPeriod - timer.elapsed()) / 1000.0);

        if (delayMs <= 0)
            delayMs = 1; // Min delay time when we are late behind the frame-rate

        auto key = cv::waitKey(delayMs);

        switch (key)
        {
        case 'q':
            exit = true;
            break;
        case 'd':
            mode = Depth;
            break;
        case 'v':
            mode = Video;
            break;
        }

        frameId++;
    }
}

VideoStream &Application::cvs() { return *videoStreams[mode]; }

// The actual mouse callback
void Application::onMouse(int evt, int x, int y, int flags)
{
    switch (evt)
    {
    case CV_EVENT_LBUTTONUP:
        mouseDown = false;
        break;
    case CV_EVENT_LBUTTONDOWN:
        mouseDown = true;
        if (targetTracker)
            targetTracker->updateRegionOfInterest(x, y);
        break;
    case CV_EVENT_MOUSEMOVE:
        if (mouseDown && targetTracker)
            targetTracker->updateRegionOfInterest(x, y);
        break;
    }
}

