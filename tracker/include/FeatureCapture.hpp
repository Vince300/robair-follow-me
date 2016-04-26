#ifndef _FEATURECAPTURE_HPP_
#define _FEATURECAPTURE_HPP_

#include <vector>

#include <opencv2/core/core.hpp>

/**
 * \brief Represents the state of mean/mask-based user tracking
 */
struct FeatureCapture
{
    /**
     * \brief Initializes a new instance of the \see FeatureCapture struct.
     */
    FeatureCapture();

    /// Current mask computed from depth data.
    cv::Mat mask;
    /// Current mean depth of the region of interest.
    cv::Scalar mean;
    /// X coordinate of the center of the region of interest.
    int poi_x;
    /// Y coordinate of the center of the region of interest.
    int poi_y;
};

#endif
