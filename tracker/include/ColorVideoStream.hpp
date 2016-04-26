#ifndef _COLORVIDEOSTREAM_HPP_
#define _COLORVIDEOSTREAM_HPP_

class VideoStream;

/**
 * \brief Represents the RGB video stream from the OpenNI device.
 */
class ColorVideoStream : public VideoStream
{
public:
    /**
     * \brief Initializes a new instance of the \see ColorVideoStream class.
     *
     * \param device The OpenNI device to capture the stream from.
     */
    ColorVideoStream(const openni::Device &device);

    /**
     * \brief Frees the resources used by this instance.
     */
    ~ColorVideoStream() override;

    /**
     * \brief Captures a new frame from the video stream.
     * 
     * \return An OpenCV matrix containing the frame data.
     */
    cv::Mat captureFrame() override;
};

#endif
