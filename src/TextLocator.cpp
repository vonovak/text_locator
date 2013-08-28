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

#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tesseract/publictypes.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "text_locator/TextLocator.hpp"
#include "text_locator/utils.hpp"

//for publisher
#include "text_locator/TextLocation.h"
#include <tf/transform_datatypes.h>
#include <stdlib.h>

using namespace sensor_msgs;

namespace ros_text_locator {

TextLocator::TextLocator(ros::NodeHandle &handle,
		tesseract::PageSegMode pgSegMode, std::string lang,
		std::vector<ccv_swt_param_t> params, bool enableRecognition,
		bool paramDebug, bool enablePcl) :
		rec(TRecognizer(pgSegMode, lang)), detector(TDetector(params)), paramDebugMode(
				paramDebug), recognitionEnabled(enableRecognition), newdata(
				false), nodehandler(handle), pclEnabled(enablePcl) {

	init();
//	nodehandler = handle;
//	detector = TDetector(params);
//	rec = TRecognizer(pgSegMode, lang);
//	paramDebugMode = paramDebug;
//	recognitionEnabled = enableRecognition;
//	pclEnabled=enablePcl;
//	locked = false;
}

void TextLocator::init() {
	using namespace ros::param;

	int dm, im;
	param<int>("/camera/driver/image_mode", im, -1);
	param<int>("/camera/driver/depth_mode", dm, -1);
	if (dm == -1 || im == -1) {
		ROS_ERROR(
				"unable to load sensor resolutions from /camera/driver/depth_mode and/or /camera/driver/image_mode");
		ros::shutdown();
	}
	std::pair<int, int> rgbRes = openniModeToRes(im);
	std::pair<int, int> irRes = openniModeToRes(dm);
	if (rgbRes.first == -1 || irRes.first == -1) {
		ROS_ERROR(
				"unknown openni mode encountered. unable to convert the mode to resolution.");
		ros::shutdown();
	}

	rgb2IrX = ((float) irRes.first) / ((float) rgbRes.first);
	rgb2IrY = ((float) irRes.second) / ((float) rgbRes.second);
}

void TextLocator::run() {
	sub = nodehandler.subscribe("/camera/rgb/image_rect", 8,
			&TextLocator::monoCallback, this);
	if (pclEnabled) {
		sub2 = nodehandler.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
				"/camera/depth_registered/points", 8,
				&TextLocator::depthCallback, this);
	}

	pub = nodehandler.advertise<text_locator::TextLocation>(
			"text_locator_topic", 8);
	if (!pub) {
		ROS_ERROR("publisher of text_locator_topic cannot be initialized");
		ros::shutdown();
	}
	if (paramDebugMode) {
		std::string h = getHome() + "/text_locator.jpg";
		ROS_INFO(
				"debug image will be written to %s. use it to fine-tune the detections parameters and then turn this feature off.", h.c_str());
	}
}

void TextLocator::monoCallback(const sensor_msgs::ImageConstPtr& image) {
	try {
		cv_ptr = cv_bridge::toCvShare(image, image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	newdata = true;

	detector.detect(cv_ptr->image, text2d);
	if (paramDebugMode) {
		saveDbgImg(image);
	}
	if (recognitionEnabled) {
		rec.recognize(cv_ptr->image, text2d);
	}
	if (!pclEnabled) {
		pcl::PointXYZRGB p; //dummy point
		for (int i = 0; i < text2d.size(); i++) {
			publish(p, p, text2d.at(i).text);
		}
	}
}

void TextLocator::depthCallback(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& depth) {
	if (!newdata) {
		return;
	}
	int y, x, y2, x2;
	pcl::PointXYZRGB p1, p2;
	if (text2d.size() == 0) {
		//ROS_INFO("nothing detected");
	}
	// IF recognition disabled then broadcast the detected textlike region coordinates in rgb image?
	for (int i = 0; i < text2d.size(); i++) {
		// coordinate translation for different ir and rgb resolutions
		y = rgbToIrY(text2d.at(i).y);
		x = rgbToIrX(text2d.at(i).x);
		y2 = rgbToIrY(text2d.at(i).y2);
		x2 = rgbToIrX(text2d.at(i).x2);

		p1 = depth->points[(y * depth->width) + x];
		p2 = depth->points[(y2 * depth->width) + x2];
		//cout << "kinect: " << text2d.at(i).text << endl;
		//cout << "kinect: " << p1.x << " " << p1.y << " " << p1.z << "\t";
		//cout << "kinect: " << p2.x << " " << p2.y << " " << p2.z << endl;
		publish(p1, p2, text2d.at(i).text);
	}
	newdata = false;
}

int TextLocator::rgbToIrX(int rgb) {
	return (int) (rgb2IrX * ((float) (rgb)));
}

int TextLocator::rgbToIrY(int rgb) {
	return (int) (rgb2IrY * ((float) (rgb)));
}

void TextLocator::publish(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2,
		std::string txt) {
	text_locator::TextLocation msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/camera/depth_registered/points";
	geometry_msgs::Point upLeft;
	upLeft.x = p1.x;
	upLeft.y = p1.y;
	upLeft.z = p1.z;
	msg.p1 = upLeft;

	geometry_msgs::Point lowRight;
	lowRight.x = p2.x;
	lowRight.y = p2.y;
	lowRight.z = p2.z;

	msg.p2 = lowRight;
	msg.text = txt;
	pub.publish(msg);
}

void TextLocator::saveDbgImg(const sensor_msgs::ImageConstPtr& image) {
	cv::Mat m;
	try {
		m = cv_bridge::toCvShare(image, image_encodings::BGR8)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Point p1, p2;
	//drawing detected areas
	for (int i = 0; i < text2d.size(); i++) {
		p1.x = text2d.at(i).x;
		p1.y = text2d.at(i).y;
		p2.x = text2d.at(i).x2;
		p2.y = text2d.at(i).y2;
		cv::rectangle(m, p1, p2, cv::Scalar(0, 0, 255));
	}
	//rectangle in the center
	p1.x = m.cols / 2;
	p1.y = m.rows / 2;
	p2.x = m.cols / 2 + 1;
	p2.y = m.rows / 2 + 1;
	cv::rectangle(m, p1, p2, cv::Scalar(0, 0, 255));
	std::string h = getHome() + "/text_locator.jpg";
	cv::imwrite(h, m);
}

} /* namespace manipulator */
