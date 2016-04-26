#include <iostream>
#include <opencv2/core.hpp>
#include <algorithm>

#include "SlidingWindowTracker.hpp"
#include <opencv2/imgproc.hpp>

SlidingWindowTracker::SlidingWindowTracker()
{
}

SlidingWindowTracker::~SlidingWindowTracker()
{
}

void SlidingWindowTracker::updateDisplayFrame(cv::Mat& displayFrame)
{
    // Draw the centroid on the display frame
    cv::drawMarker(displayFrame, cv::Point(tracking.poi_x, tracking.poi_y), cv::Scalar(255, 0, 255, 0));
}

void SlidingWindowTracker::doUpdateRegionOfInterest(int x, int y)
{
    doUpdateRegionOfInterest(x, y, cv::Scalar(0));
}

bool SlidingWindowTracker::doUpdateTarget()
{
    if (tracking.mean != cv::Scalar(0))
    {
        cv::Moments m = moments(tracking.mask, true);

        // The current tracking mean is used as a fallback so if the centroid has no depth data
        // we can find the contour of the tracking area beased on the depth value.
        doUpdateRegionOfInterest(m.m10 / m.m00, m.m01 / m.m00, tracking.mean);

        /******* BEGIN UPDATE OF HEADING *******/
        double distance = tracking.mean.val[0] / 1000.0;
        double angle = 0.0,
               speed = std::max(std::min(distance - 1.0, 1.0), 0.0);

        // -1.0 is full left, 1.0 is full right
        double pos = ((double)tracking.poi_x / lastDepthFrame.cols - 0.5) * 2.0;

        // Update command values
        setCommand(speed, angle);

        return true;
    }

    return false;
}

void SlidingWindowTracker::doUpdateRegionOfInterest(int x, int y, cv::Scalar fallbackValue)
{
    // Try to build a mask from the last depth frame
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
        int x0 = std::max(0, x - w / 2);

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

    // Update tracking values
    tracking.mean = cv::mean(lastDepthFrame, tracking.mask);
    tracking.poi_x = x;
    tracking.poi_y = y;
}
