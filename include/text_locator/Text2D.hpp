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

#ifndef TEXT_LOCATOR_TEXT2D_HPP_
#define TEXT_LOCATOR_TEXT2D_HPP_
#include <string>
namespace ros_text_locator {
class Text2D {
public:
	int x, y, x2, y2;
	std::string text;
	Text2D() {
		this->x = -1;
		this->y = -1;
		this->x2 = -1;
		this->y2 = -1;
		this->text = "not initialized";
	}
	Text2D(int x, int y, int x2, int y2, std::string) {
		this->x = x;
		this->y = y;
		this->x2 = x2;
		this->y2 = y2;
		this->text = text;
	}
	virtual ~Text2D() {
	}

};
}
#endif /* TEXT_LOCATOR_TEXT2D_HPP_ */
