#include "ObstaclesBasicTracker.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>

ObstaclesBasicTracker::ObstaclesBasicTracker()
    : SlidingWindowTracker(),
    headAngle(0.0)
{
}

ObstaclesBasicTracker::~ObstaclesBasicTracker()
{
}

bool ObstaclesBasicTracker::doUpdateTarget()
{
    if (SlidingWindowTracker::doUpdateTarget())
    {
        double fov = 29.0 * M_PI / 180.0;
        double xTarget = ((double)tracking.poi_x / lastDepthFrame.cols - 0.5) * 2.0 * fov; // -29deg to 29deg

        double lRobert = 250.0;
        int secureDistance = 600;

        cv::Mat obstacles16;
        cv::inRange(lastDepthFrame, cv::Scalar(1), cv::Scalar(secureDistance), obstacles16);
        cv::Mat obstacles;
        obstacles16.convertTo(obstacles, CV_8U);

        // Filter for the right obstacles
        cv::Mat rectFiltRight = cv::Mat::zeros(lastDepthFrame.size(), CV_8U);
        int s = lastDepthFrame.cols / 2.0 * (1.0 + (headAngle / fov));
        cv::Mat mrr(rectFiltRight, cv::Rect(s, 0, lastDepthFrame.cols - s, lastDepthFrame.rows));
        mrr = cv::Scalar(255);

        cv::Mat rightObstacles;
        cv::bitwise_and(obstacles, rectFiltRight, rightObstacles);

        int xobsr = -1;
        for (int x = s; x < lastDepthFrame.cols; ++x)
        {
            for (int y = s; y < lastDepthFrame.rows; ++y)
            {
                if (rightObstacles.at<uchar>(y, x))
                {
                    xobsr = x;
                    break;
                }
            }

            if (xobsr > -1)
                break;
        }

        double zobsrmin, zobsrmax;
        cv::minMaxIdx(lastDepthFrame, &zobsrmin, &zobsrmax, NULL, NULL, rightObstacles);

        std::cerr << "XobsR = " << xobsr << " ZobsR = " << zobsrmin << std::endl;

        cv::imshow("Masque obstacles", rightObstacles);

        /******* BEGIN UPDATE OF HEADING *******/

        double xObsLeft;
        double zObsLeft;
        double xObsRight;
        double zObsRight;

        double commandSpeed = std::max(std::min((tracking.mean.val[0] / 1000.0) - 1.0, 1.0), 0.0);
        double commandAngle = 0.0;

        // If we can go through the left and right obstacles
        if (abs(xTarget - xObsLeft) > atan(lRobert / zObsLeft) &&
            abs(xTarget - xObsRight) > atan(lRobert / zObsRight))
        {
            // 'Full' speed towards target
            commandAngle = xTarget;
        }
        else
        {
            if (abs(xTarget - xObsLeft) < atan(lRobert / zObsLeft) &&
                abs(xTarget - xObsRight) < atan(lRobert / zObsRight))
            {
                // Stop, we can't go through
                commandAngle = 0.0;
                commandSpeed = 0.0;
            }
            else
            {
                if (abs(xTarget - xObsLeft) < atan(lRobert / zObsLeft))
                {
                    // We can't get through left
                    commandAngle = xObsLeft + atan(lRobert / zObsLeft);
                }
                else
                {
                    // We can't get through right
                    commandAngle = xObsRight - atan(lRobert / zObsRight);
                }
            }
        }

        // Update command data
        setCommand(commandSpeed, commandAngle);

        return true;
    }

    return false;
}
