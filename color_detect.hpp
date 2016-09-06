#ifndef _COLOR_DETECT_HPP_
#define _COLOR_DETECT_HPP_

#include <vector>
#include <opencv2/core/core.hpp>

namespace color_detect
{ 
    void find_blue(const cv::Mat & img, std::vector<cv::Rect> & out, int area_ts=100, cv::Scalar lower=cv::Scalar(110, 50, 50), cv::Scalar upper=cv::Scalar(130, 255, 255));

}

#endif
