#include "stdafx.h"
#include "Line.h"



Line::Line(int _x1, int _x2, int _y1, int _y2)
{
	x1 = _x1;
	x2 = _x2;
	y1 = _y1;
	y2 = _y2;
	begin = cv::Point(x1, y1);
	end = cv::Point(x2, y2);
}

Line::Line(cv::Point begin, cv::Point end)
{
	this->begin = begin;
	this->end = end;
	x1 = begin.x;
	y1 = begin.y;
	x2 = end.x;
	y2 = end.y;

}


void Line::setData(int _x1, int _x2, int _y1, int _y2)
{
	x1 = _x1;
	x2 = _x2;
	y1 = _y1;
	y2 = _y2;
	begin = cv::Point(x1, y1);
	end = cv::Point(x2, y2);
}

int Line::getMagnitude()
{
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}


Line::~Line()
{
}


