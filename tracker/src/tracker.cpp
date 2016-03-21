#include "tracker.hpp"
#include "LowPassFilter.hpp"

int main(int argc, char const *argv[]) {
    openni::Device device;
    openni::VideoStream depth, color, ir;

    openni::OpenNI::initialize();
    device.open(openni::ANY_DEVICE);

    ir.create(device, openni::SENSOR_IR);
    depth.create(device, openni::SENSOR_DEPTH);
    color.create(device, openni::SENSOR_COLOR);

    openni::VideoMode vm = depth.getVideoMode();
    int colsDepth, rowsDepth;
    colsDepth = vm.getResolutionX();
    rowsDepth = vm.getResolutionY();

    vm = color.getVideoMode();
    int colsColor, rowsColor;
    colsColor = vm.getResolutionX();
    rowsColor = vm.getResolutionY();

    openni::VideoFrameRef depthFrame;
    openni::VideoFrameRef colorFrame;
    depth.start();
    color.start();

    cv::namedWindow("Depth data", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Video data", cv::WINDOW_AUTOSIZE);
    
    LowPassFilter<double> lpfMax(0.5);

    while (cv::waitKey(1) == -1)
    {
        depth.readFrame(&depthFrame);
        color.readFrame(&colorFrame);
        openni::DepthPixel* dData = (openni::DepthPixel*)depthFrame.getData();
        cv::Mat depthImage(rowsDepth, colsDepth, CV_16UC1, dData);

        double min;
        double max;
        cv::minMaxIdx(depthImage, &min, &max);
        cv::Mat adjMap;
        cv::convertScaleAbs(depthImage, adjMap, 255 / lpfMax.nextStep(max));

        openni::RGB888Pixel* cData = (openni::RGB888Pixel*)colorFrame.getData();
        cv::Mat colorImage(rowsDepth, colsDepth, CV_8UC3, cData);

        cv::imshow("Depth data", adjMap);
        cv::imshow("Video data", colorImage);
    }

    color.stop();
    depth.stop();
    openni::OpenNI::shutdown();

    return 0;
}
