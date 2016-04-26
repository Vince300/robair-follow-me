#ifndef _DEPTHVIDEOSTREAM_HPP_
#define _DEPTHVIDEOSTREAM_HPP_

#include "VideoStream.hpp"
#include "LowPassFilter.hpp"

/**
 * \brief Represents the depth data stream from an OpenNI device.
 */
class DepthVideoStream : public VideoStream
{
public:
    /**
     * \brief Initializes a new instance of the \see DepthVideoStream class.
     *
     * \param device The OpenNI device to capture the depth data from.
     */
    DepthVideoStream(const openni::Device &device);

    /**
     * \brief Frees the resources used by this instance.
     */
    ~DepthVideoStream() override;

    /**
     * \brief Captures a new frame from the video stream.
     * 
     * \return An OpenCV matrix containing the frame data.
     */
    cv::Mat captureFrame() override;

    /**
     * \brief Returns an image suitable for user display.
     * 
     * \param source Frame captured from this instance to process for displaying.
     * 
     * \return An OpenCV matrix representing the frame to display.
     */
    cv::Mat displayFrame(const cv::Mat &source) override;

private:
    /// Low pass filter for the maximum depth value.
    LowPassFilter<double> lpfMax;
};

#endif
