#ifndef _VIDEOSTREAM_HPP_
#define _VIDEOSTREAM_HPP_

#include <OpenNI.h>
#include <opencv2/core/core.hpp>

/**
 * \brief Represents a generic video stream from an OpenNI device.
 */
class VideoStream
{
protected:
    /**
     * \brief Initializes a new instance of the \see VideoStream class.
     *
     * \param device     OpenNI device to get the data from.
     * \param sensorType The type of sensor to initialize.
     */
    VideoStream(const openni::Device &device, openni::SensorType sensorType);

    /**
     * \brief Frees the resources used by this instance.
     */
    virtual ~VideoStream();

public:
    /**
     * \brief Gets the framerate of this stream.
     *
     * \return Number of frames per second.
     */
    int frameRate() const;
    /**
     * \brief Gets the number of image rows.
     */
    int rows() const;
    /**
     * \brief Gets the number of image columns.
     */
    int cols() const;

    /**
     * \brief Captures a new frame from the device and returns
     *        an OpenCV matrix representing the data.
     */
    virtual cv::Mat captureFrame() = 0;
    /**
     * \brief Performs display transformations on a matrix to
     *        render the image visible by the end-user.
     * 
     * \param source Captured image data to use as a base for the
     *               display frame.
     */
    virtual cv::Mat displayFrame(const cv::Mat &source);

    /**
     * \brief Prints the stream info to the output stream.
     *
     * \param os Output stream to print the info to.
     */
    virtual void printStreamInfo(std::ostream &os);

protected:
    /**
     * \brief Implements the actual frame data reading.
     */
    const void *readFrameData();

private:
    /// OpenNI device for this stream.
    const openni::Device &device;
    /// Video mode information for this stream.
    openni::VideoMode vm;
    /// Video stream object.
    openni::VideoStream vs;
    /// Reference to the captured video frame.
    openni::VideoFrameRef frame;
};

#endif
