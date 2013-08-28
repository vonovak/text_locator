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

#include "text_locator/TRecognizer.hpp"
#include <opencv/highgui.h>
#include <sstream>
#include <cstdlib>
#include <text_locator/utils.hpp>

namespace ros_text_locator {

TRecognizer::TRecognizer(tesseract::PageSegMode pageSegMode, std::string lang =
		"eng") :
		mode(pageSegMode), lang(lang), api(new tesseract::TessBaseAPI()) {
}

bool TRecognizer::initTesseract() {
	if (api->Init("/usr/share/tesseract-ocr/", lang.c_str())) {
		fprintf(stderr, "Could not initialize tesseract.\n");
		std::exit(EXIT_FAILURE);
		return false;
	}
	return true;
}

TRecognizer::~TRecognizer() {
	api->Clear();
	api->End();
}

std::string TRecognizer::recognize(const cv::Mat &img, int x, int y, int width, int height) {
	initTesseract();
	std::string out = tesseract(img, x, y, width, height);
	return trim(out);
}

void TRecognizer::recognize(const cv::Mat &img, std::vector<Text2D>& text2d) {
	std::string in;
	initTesseract();
	for (size_t i = 0; i < text2d.size(); i++) {
		Text2D& r = text2d.at(i);
		in = tesseract(img, r.x, r.y, (r.x2 - r.x), (r.y2 - r.y));
		r.text = trim(in);
	}
}

const char* TRecognizer::tesseract(const cv::Mat &image, int left, int top,
		int width, int height) {
	api->SetImage(image.data, image.cols, image.rows, image.channels(),
			image.step1());
	api->SetPageSegMode(mode);
	api->SetRectangle(left, top, width, height);
	char* outText = api->GetUTF8Text();
	//std::cout << (outText) << std::endl;
	return outText;
}

} /* namespace manipulator */
