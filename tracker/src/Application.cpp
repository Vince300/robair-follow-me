#include <iostream>
#include <boost/timer.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "Utils.hpp"
#include "DepthVideoStream.hpp"
#include "ColorVideoStream.hpp"

#include "Application.hpp"

void onMouse(int evt, int x, int y, int flags, void *userdata)
{
    static_cast<Application*>(userdata)->onMouse(evt, x, y, flags);
}

Application::Application()
    : tracking(), mouseDown(false), device(), mode(Video), videoStreams()
{
    // Init OpenNI device
    openni::OpenNI::initialize();
    NICHECK(device.open(openni::ANY_DEVICE));

    // Init video streams
    videoStreams.push_back(std::static_pointer_cast<VideoStream>(std::make_shared<ColorVideoStream>(device)));
    videoStreams.push_back(std::static_pointer_cast<VideoStream>(std::make_shared<DepthVideoStream>(device)));
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

    cvs().printStreamInfo();
    
    bool exit = false;
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
        cv::Mat m(displayFrame);

        // Perform tracking
        if (!tracking.keyPoints.empty())
        {
            // Find features
            FeatureCapture fc(lastVideoFrame);
            auto detector = cv::xfeatures2d::SurfFeatureDetector::create();
            auto descriptor = cv::xfeatures2d::SurfDescriptorExtractor::create();

            detector->detect(fc.frame, fc.keyPoints);
            descriptor->compute(fc.frame, fc.keyPoints, fc.descriptors);

            cv::FlannBasedMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match(tracking.descriptors, fc.descriptors, matches);
            cv::drawMatches(tracking.frame, tracking.keyPoints, displayFrame, fc.keyPoints, matches, m, tracking.color, cv::Scalar(255, 0, 0));
        }

        // Display resulting image
        cv::imshow(WINDOW_TITLE, m);

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
            cvs().printStreamInfo();
            break;
        case 'v':
            mode = Video;
            cvs().printStreamInfo();
            break;
        }
    }
}

VideoStream &Application::cvs() { return *videoStreams[mode]; }

// Detects features inside a specific radius around a mouse event
void Application::addRegionOfInterest(int x, int y)
{
    // Set video frame reference for tracking
    tracking.frame = lastVideoFrame;
    
    // Build a mask
    cv::Mat mask;
    try
    {
        cv::Scalar dataScalar = lastDepthFrame.at<cv::Scalar_<uint16_t>>(y, x);
        uint32_t data = dataScalar.val[0];
        if (data == 0) return;
        
        uint32_t lw = (9 * data) / 10, ub = (11 * data) / 10;
        cv::inRange(lastDepthFrame, cv::Scalar(lw), cv::Scalar(ub), mask);
    }
    catch (cv::Exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return;
    }

    // Display mask data
    cv::imshow("Mask data", mask);
    
    // Detect features
    auto detector = cv::xfeatures2d::SurfFeatureDetector::create();
    detector->detect(tracking.frame, tracking.keyPoints, mask);

    // Descriptor
    auto descriptor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    descriptor->compute(tracking.frame, tracking.keyPoints, tracking.descriptors);

    // Setup data
    tracking.color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
}

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
        addRegionOfInterest(x, y);
        break;
    case CV_EVENT_MOUSEMOVE:
        if (mouseDown)
            addRegionOfInterest(x, y);
        break;
    }
}

