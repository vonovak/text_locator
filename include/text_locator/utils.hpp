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

#ifndef TEXT_LOCATOR_UTILS_HPP_
#define TEXT_LOCATOR_UTILS_HPP_
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <ccv.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <pwd.h>
#include <fstream>

namespace ros_text_locator {

inline std::pair<int, int> getTargetCoordinate(int i,
		std::vector<Text2D>& text2d) {
	std::pair<int, int> ret;
	ret.first = -1;
	ret.second = -1;
	if (i == -1) {
		return ret;
	}
	ret.first = (text2d.at(i).x2 - text2d.at(i).x) / 2 + text2d.at(i).x;
	ret.second = text2d.at(i).y;
	//ret.second = (text2d.at(i).y2 - text2d.at(i).y) / 2 + text2d.at(i).y;
	//std::cout << "coordinates: x: " << ret.first << " y: " << ret.second << std::endl;
	return ret;
}

inline std::string &trim(std::string &s) {
	boost::algorithm::trim(s);
	return s;
}

inline std::string readFile(std::string file) {
	std::ifstream in(file.c_str());
	std::string contents((std::istreambuf_iterator<char>(in)),
			std::istreambuf_iterator<char>());
	return contents;
}

inline std::pair<int, int> openni2ModeToRes(int mode) {
	std::pair<int, int> ret;
	switch (mode) {
	case 1:
	case 2:
		ret.first = 1280;
		ret.second = 1024;
		break;
	case 3:
	case 4:
		ret.first = 1280;
		ret.second = 720;
		break;
	case 5:
	case 6:
		ret.first = 640;
		ret.second = 480;
		break;
	case 7:
	case 8:
	case 9:
		ret.first = 320;
		ret.second = 240;
		break;
	case 10:
	case 11:
	case 12:
		ret.first = 160;
		ret.second = 120;
		break;
	default:
		ret.first = -1;
		ret.second = -1;
		break;
	}
	return ret;
}

inline std::pair<int, int> openniModeToRes(int mode) {
	std::pair<int, int> ret;
	switch (mode) {
	case 1:
		ret.first = 1280;
		ret.second = 1024;
		break;
	case 2:
	case 3:
		ret.first = 640;
		ret.second = 480;
		break;
	case 4:
	case 5:
	case 6:
		ret.first = 320;
		ret.second = 240;
		break;
	case 7:
	case 8:
	case 9:
		ret.first = 160;
		ret.second = 120;
		break;
	default:
		ret.first = -1;
		ret.second = -1;
		break;
	}
	return ret;
}

inline std::string exec(char* cmd) {
	FILE* pipe = popen(cmd, "r");
	if (!pipe)
		return "ERROR";
	char buffer[128];
	std::string result = "";
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}
	pclose(pipe);
	return result;
}

inline std::string getHome() {
	using namespace std;
	int myuid;
	passwd *mypasswd;
	myuid = getuid();
	mypasswd = getpwuid(myuid);
	return mypasswd->pw_dir;
}

inline int findMatch(std::string &in, std::vector<Text2D>& text2d) {
	for (size_t i = 0; i < text2d.size(); i++) {
		if (text2d.at(i).text == in) {
			std::cout << "100% match! " << std::endl;
			return i;
		}
	}
	return -1;
}
}
#endif /* TEXT_LOCATOR_UTILS_HPP_ */
