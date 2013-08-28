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

#ifndef TEXT_LOCATOR_TDETECTOR_HPP_
#define TEXT_LOCATOR_TDETECTOR_HPP_
#include "ccv.hpp"
#include <opencv/highgui.h>
#include <vector>
#include "Text2D.hpp"

namespace ros_text_locator {

class TDetector {
private:
	//ccv_dense_matrix_t* image;
	//ccv_array_t* words;
	std::vector<ccv_swt_param_t> params;
	void pdetect(ccv_dense_matrix_t* image, std::vector<Text2D>& text2d);

public:
	TDetector();
	TDetector(std::vector<ccv_swt_param_t> params);
	virtual ~TDetector();
	void clearParams();
	void addParams(ccv_swt_param_t ccv_swt_params);
	void detect(const cv::Mat &rgb, std::vector<Text2D>& text2d);

	const std::vector<ccv_swt_param_t>& getParams() const {
		return params;
	}

	void setParams(const std::vector<ccv_swt_param_t>& params) {
		this->params = params;
	}
};

} /* namespace ros_text_locator */
#endif /* TEXT_LOCATOR_TDETECTOR_HPP_ */
