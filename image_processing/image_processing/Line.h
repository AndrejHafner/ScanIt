#pragma once
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"

class Line
{
public:
	int x1, x2, y1, y2;
	cv::Point begin, end;
	Line(int x1, int x2, int y1, int y2);
	Line(cv::Point begin, cv::Point end);

	void setData(int x1, int x2, int y1, int y2);
	int getMagnitude();
	~Line();


	
};



