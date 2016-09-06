#include <opencv2/imgproc/imgproc.hpp>

#include "color_detect.hpp"

void color_detect::find_blue(const cv::Mat & img, std::vector<cv::Rect> & out,
    int area_ts,
    cv::Scalar lower, 
    cv::Scalar upper)
{
    cv::Mat mask, hsv;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hier;

    cv::cvtColor(img, hsv, CV_BGR2HSV);
    cv::inRange(hsv, lower, upper, mask);
	cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	for(auto contour : contours) {
		if (cv::contourArea(contour) > 100) {
			out.push_back(cv::boundingRect(contour));
		}
	}	
}
