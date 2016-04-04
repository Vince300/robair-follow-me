#include <iostream>
#include <boost/timer.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Utils.hpp"
#include "DepthVideoStream.hpp"
#include "ColorVideoStream.hpp"

#include "Application.hpp"
#include <iomanip>

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
        cv::Mat m(displayFrame);

        if (tracking.mean != cv::Scalar(0))
        {
            cv::Moments m = moments(tracking.mask, true);

            // The current tracking mean is used as a fallback so if the centroid has no depth data
            // we can find the contour of the tracking area beased on the depth value.
            updateRegionOfInterest(m.m10 / m.m00, m.m01 / m.m00, tracking.mean);

            // Draw the centroid on the display frame
            cv::drawMarker(displayFrame, cv::Point(tracking.poi_x, tracking.poi_y), cv::Scalar(255, 0, 255, 0));

            // Output space separated data
            // Format is hpos from -0.5 to 0.5, depth in m from camera, time in ms since start
            std::cout << std::fixed << std::setprecision(6)
                << ((double)tracking.poi_x / lastDepthFrame.cols - 0.5) << " " 
                << tracking.mean.val[0] / 1000.0 << " "
                << tmstart.elapsed() * 1000.0 << " "
                << frameId
                << std::endl;
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
            break;
        case 'v':
            mode = Video;
            break;
        }
        
        frameId++;
    }
}

VideoStream &Application::cvs() { return *videoStreams[mode]; }

// Detects features inside a specific radius around a mouse event
void Application::updateRegionOfInterest(int x, int y, cv::Scalar fallbackValue)
{
    // Build a mask
    try
    {
        // Get the color of the point at the center of interest
        cv::Scalar dataScalar = lastDepthFrame.at<ushort>(y, x);
        uint32_t data = dataScalar.val[0];

        if (data == 0) data = fallbackValue.val[0];

        // If there is no data in the selected point, abort update
        if (data == 0) return;
        
        // Create a mask using points that are +/- 10% of the selected value, store in q
        uint32_t lw = (9 * data) / 10, ub = (11 * data) / 10;
        cv::Mat q;
        cv::inRange(lastDepthFrame, cv::Scalar(lw), cv::Scalar(ub), q);

        // To exclude large objects at the same distance, mask q using a rectangle centered on the interest point

        // Width and height of the rectangle
        int w = lastDepthFrame.cols / 3, h = lastDepthFrame.rows;
        int x0 = max(0, x - w / 2);

        if (x + w / 2 > lastDepthFrame.cols)
        {
            w = lastDepthFrame.cols - x;
        }
        
        // Build the mask using a bitwise and of the rectangle and q
        cv::Mat rectFilt = cv::Mat::zeros(lastDepthFrame.size(), CV_8U);
        cv::Mat mr(rectFilt, cv::Rect(x0, 0, w, h));
        mr = cv::Scalar(255);

        cv::bitwise_and(q, rectFilt, tracking.mask);
    }
    catch (cv::Exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return;
    }

    // Display mask data
    // Debug display mask data
    // cv::imshow("Mask data", tracking.mask);
    tracking.mean = cv::mean(lastDepthFrame, tracking.mask);
    tracking.poi_x = x;
    tracking.poi_y = y;

    // Debug data output
    // std::cerr << "POI: (" << x << ", " << y << "), Depth: " << std::setprecision(2) << tracking.mean.val[0] / 1000.0 << "m" << std::endl;
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
        updateRegionOfInterest(x, y, cv::Scalar(0));
        break;
    case CV_EVENT_MOUSEMOVE:
        if (mouseDown)
            updateRegionOfInterest(x, y, cv::Scalar(0));
        break;
    }
}

