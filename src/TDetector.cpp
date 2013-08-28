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

#include "text_locator/TDetector.hpp"
#include <string>
#include <iostream>
#include <stdio.h>
#include <sstream>

namespace ros_text_locator {

TDetector::TDetector() {
	params.push_back(ccv_swt_default_params);
	ccv_enable_default_cache();
}

TDetector::TDetector(std::vector<ccv_swt_param_t> params) {
	this->params = params;
	ccv_enable_default_cache();
}

TDetector::~TDetector() {
}

void TDetector::pdetect(ccv_dense_matrix_t* image,
		std::vector<Text2D>& text2d) {
	text2d.clear();
	//text2d.erase(text2d.begin(), text2d.end());

	if (image != 0) {
		for (int i = 0; i < params.size(); i++) {
			ccv_array_t* words = ccv_swt_detect_words(image, params.at(i));

			if (words) {
				text2d.reserve(words->rnum);
				ccv_rect_t* rect;
				for (int j = 0; j < words->rnum; j++) {
					rect = (ccv_rect_t*) ccv_array_get(words, j);
					//printf("%d %d %d %d\n", rect->x, rect->y, rect->width, rect->height);
					text2d.push_back(
							Text2D(rect->x, rect->y, rect->x + rect->width,
									rect->y + rect->height, ""));
				}
				ccv_array_free(words);
			}
		}
		ccv_matrix_free(image);
	}
}

void TDetector::clearParams() {
	params.erase(params.begin(), params.end());
	params.push_back(ccv_swt_default_params);
}

void TDetector::detect(const cv::Mat &rgb, std::vector<Text2D>& text2d) {
	ccv_dense_matrix_t* image = 0;
	ccv_read(rgb.data, &image, CCV_IO_GRAY_RAW | CCV_IO_ANY_RAW | CCV_IO_GRAY,
			rgb.rows, rgb.cols, rgb.step[0]);
	TDetector::pdetect(image, text2d);
}

void TDetector::addParams(ccv_swt_param_t ccv_swt_params) {
	params.push_back(ccv_swt_params);
}

} /* namespace manipulator */

