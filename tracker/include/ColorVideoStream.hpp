#ifndef _COLORVIDEOSTREAM_HPP_
#define _COLORVIDEOSTREAM_HPP_

class VideoStream;

class ColorVideoStream : public VideoStream
{
public:
    ColorVideoStream(const openni::Device &device);

    ~ColorVideoStream() override;

    cv::Mat captureFrame() override;
};

#endif
