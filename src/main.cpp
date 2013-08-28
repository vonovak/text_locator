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

#include "ros/ros.h"
#include <message_filters/time_synchronizer.h>
#include "TextLocator.hpp"

using namespace ros;
using namespace std;

int main(int argc, char **argv) {

//	ccv_swt_param_t example = { 1, 0.1, 0.8, //  NO BRACES!!!
//			1, 0, 2, //  size: parameters for Canny edge detector.
//			124, //  low_thresh: parameters for Canny edge detector.
//			204, //  high_thresh: parameters for Canny edge detector.
//			300, //  max_height: the maximum height for a letter.
//			3, //  min_height: the minimum height for a letter.
//			38, 3, 10, //  aspect_ratio: the maximum aspect ratio for a letter.
//			0.83, 2.5, //  thickness_ratio: the allowable thickness variance when grouping letters.
//			7.7, //  height_ratio: the allowable height variance when grouping letters.
//			31, //  intensity_thresh: the allowable intensity variance when grouping letters.
//			5, //  distance_ratio: the allowable distance variance when grouping letters.
//			1, //  intersect_ratio: the allowable intersect variance when grouping letters.
//			3, //  letter_thresh: the allowable letter threshold.
//			1, //  elongate_ratio: the allowable elongate variance when grouping letters.
//			0, //  breakdown: if breakdown text line into words.
//			1.0 //  breakdown_ratio: apply OSTU and if inter-class variance above the threshold,
//				 //it will be break down into words.
//			};

	init(argc, argv, "text_locator_node");
	NodeHandle n;
	Rate r(3);

	vector<ccv_swt_param_t> params;
	params.push_back(ccv_swt_default_params);
	bool enableRecognition, enableDebug, enablePcl;
	param::param<bool>("enable_recognition", enableRecognition, true);
	param::param<bool>("enable_pcl", enablePcl, true);
	param::param<bool>("enable_param_debug", enableDebug, false);

	string lang, page_mode;
	param::param<std::string>("lang", lang, "eng");
	param::param<std::string>("page_mode", page_mode, "PSM_SINGLE_LINE");

	tesseract::PageSegMode pagesegmode;
	if (page_mode == "PSM_OSD_ONLY") {
		pagesegmode = tesseract::PSM_OSD_ONLY;
	} else if (page_mode == "PSM_AUTO_OSD") {
		pagesegmode = tesseract::PSM_AUTO_OSD;
	} else if (page_mode == "PSM_AUTO_ONLY") {
		pagesegmode = tesseract::PSM_AUTO_ONLY;
	} else if (page_mode == "PSM_AUTO") {
		pagesegmode = tesseract::PSM_AUTO;
	} else if (page_mode == "PSM_SINGLE_COLUMN") {
		pagesegmode = tesseract::PSM_SINGLE_COLUMN;
	} else if (page_mode == "PSM_SINGLE_BLOCK_VERT_TEXT") {
		pagesegmode = tesseract::PSM_SINGLE_BLOCK_VERT_TEXT;
	} else if (page_mode == "PSM_SINGLE_BLOCK") {
		pagesegmode = tesseract::PSM_SINGLE_BLOCK;
	} else if (page_mode == "PSM_SINGLE_LINE") {
		pagesegmode = tesseract::PSM_SINGLE_LINE;
	} else if (page_mode == "PSM_SINGLE_WORD") {
		pagesegmode = tesseract::PSM_SINGLE_WORD;
	} else if (page_mode == "PSM_SINGLE_CHAR") {
		pagesegmode = tesseract::PSM_SINGLE_CHAR;
	}

	ros_text_locator::TextLocator tl(n, pagesegmode, lang, params,
			enableRecognition, enableDebug, enablePcl);
	tl.run();

	while (ros::ok())
	{
	  ros::spinOnce();                   // Handle ROS events
	  r.sleep();
	}

	return 0;
}
