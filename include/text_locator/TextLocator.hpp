/*********************************************************************
	The MIT License (MIT)

	Copyright (c) <2013> <Vojtech Novak>

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*********************************************************************/

#ifndef TEXT_LOCATOR_TEXTLOCATOR_HPP_
#define TEXT_LOCATOR_TEXTLOCATOR_HPP_
//#include <sensor_msgs/PointCloud.h>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include "TRecognizer.hpp"
#include "TDetector.hpp"

namespace ros_text_locator {

class TextLocator {
private:
	TRecognizer rec;
	TDetector detector;
	const bool paramDebugMode;
	const bool recognitionEnabled;
	volatile bool newdata;
	ros::NodeHandle& nodehandler;
	const bool pclEnabled;

	float rgb2IrX;
	float rgb2IrY;
	std::vector<Text2D> text2d;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Subscriber sub2;
	cv_bridge::CvImageConstPtr cv_ptr;

	void publish(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2, std::string txt);
	void saveDbgImg(const sensor_msgs::ImageConstPtr& image);
	void monoCallback(const sensor_msgs::ImageConstPtr& image);
	void depthCallback(
			const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& depth);
	int rgbToIrX(int rgb);
	int rgbToIrY(int rgb);
	void init();

public:
	TextLocator();
	TextLocator(ros::NodeHandle &handle,
			tesseract::PageSegMode pgSegMode, std::string lang,
			std::vector<ccv_swt_param_t> params, const bool enableRecognition,
			const bool paramDebug, const bool enablePcl);
	void run();

	bool isRecognitionEnabled() const {
		return recognitionEnabled;
	}

	bool isPclEnabled() const {
			return pclEnabled;
		}

	TDetector& getDetector() {
		return detector;
	}

	TRecognizer& getRecognizer() {
		return rec;
	}
};

} /* namespace ros_text_locator */
#endif /* TEXT_LOCATOR_TEXTLOCATOR_HPP_ */
