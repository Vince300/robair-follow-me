#ifndef _APP_HPP_
#define _APP_HPP_

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <OpenNI.h>

#include "VideoStream.hpp"
#include "MotorDriver.hpp"

#define WINDOW_TITLE "Person tracker - FabLab Project"
#include "TargetTracker.hpp"

/**
 * \brief Represents the type of video stream that is displayed in the main
 *        application window.
 */
typedef enum
{
    /// Display the RGB video stream
    Video,
    /// Display the depth data
    Depth
} WindowDisplayMode;

/**
 * \brief Main application class, handles the lifetime of the different
 *        components such as the tracker and motor driver.
 */
class Application
{
public:
    /**
     * \brief Initializes a new instance of the \see Application class.
     */
    Application();
    /**
     * \brief Frees the resources used by the application class.
     */
    ~Application();

    /**
     * \brief Runs the main application loop.
     */
    void run();

    /**
     * \brief Obtains the current displayed videostream.
     * 
     * \return Reference to the \see VideoStream object.
     */
    VideoStream &cvs();

    /**
     * \brief Friend method declaration to redirect the OpenCV mouse callback to this instance.
     *
     * \param evt      Mouse event type.
     * \param x        X coordinate of the mouse event.
     * \param y        Y coordinate of the mouse event.
     * \param flags    Flags of the mouse event.
     * \param userdata User data pointer. Must be set to the application instance.
     */
    friend void onMouse(int evt, int x, int y, int flags, void *userdata);

protected:
    /**
     * \brief OpenCV mouse callback implementation.
     *
     * \param evt      Mouse event type.
     * \param x        X coordinate of the mouse event.
     * \param y        Y coordinate of the mouse event.
     * \param flags    Flags of the mouse event.
     */
    void onMouse(int evt, int x, int y, int flags);

private:
    /// Last RGB frame captured from the device.
    cv::Mat lastVideoFrame;
    /// Last depth data frame captured from the device.
    cv::Mat lastDepthFrame;

    /// Mouse button status.
    bool mouseDown;

    /// Depth capture device.
    openni::Device device;

    /// The current stream to display.
    WindowDisplayMode mode;

    /// Video streams provided by the device.
    std::vector<std::shared_ptr<VideoStream> > videoStreams;

    // The user tracker in use by the application.
    std::shared_ptr<TargetTracker> targetTracker;

    /// The motor driver component.
    MotorDriver motorDriver;
};


#endif
