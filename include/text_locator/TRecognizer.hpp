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

#ifndef TEXT_LOCATOR_TRECOGNIZER_HPP_
#define TEXT_LOCATOR_TRECOGNIZER_HPP_
#include "TDetector.hpp"
#include "Text2D.hpp"
#include <vector>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>

namespace ros_text_locator {

class TRecognizer {
private:
	tesseract::PageSegMode mode;
	std::string lang;
	tesseract::TessBaseAPI* api;
	bool initTesseract();
	const char* tesseract(const cv::Mat &image, int left, int top, int width,
			int height);

public:
	TRecognizer(tesseract::PageSegMode pageSegMode, std::string lang);
	virtual ~TRecognizer();
	void recognize(const cv::Mat &img, std::vector<Text2D>& text2d);
	std::string recognize(const cv::Mat &img, int x, int y, int width, int height);

	const std::string& getLang() const {
		return lang;
	}

	void setLang(std::string lang) {
		this->lang = lang;
	}

	const tesseract::PageSegMode getMode() const {
		return mode;
	}

	void setMode(tesseract::PageSegMode mode) {
		this->mode = mode;
	}
};

} /* namespace manipulator */
#endif /* TEXT_LOCATOR_TRECOGNIZER_HPP_ */
