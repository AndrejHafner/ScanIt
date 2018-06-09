#pragma once
#include "Line.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"

#define LINE_SHRINK 0.05
#define WINDOW_NAME "show"
#define SCALE_RATIO 0.25
#define LINES_DELTA 10
#define MIN_MERGE_DISTANCE 40
#define MIN_MERGE_ANGLE 5
#define MAX_ANGLE_THRESH 30
#define AREA_SIZE 20

#define degreesToRadians(angleDegrees) ((angleDegrees) * CV_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / CV_PI)

using namespace cv;
using namespace std;

typedef struct LineIntersecPack
{
	Line a;
	Line b;
	cv::Point intersection;
	int score;
	int angle;
} LineIntersecPack;

typedef struct Quadrilateral
{
	cv::Point p1, p2, p3, p4;
} Quadrilateral;

struct lineXSort
{
	inline bool operator() (const Line& struct1, const Line& struct2)
	{
		return (struct1.x1 < struct2.x1);
	}
};

struct lineYSort
{
	inline bool operator() (const Line& struct1, const Line& struct2)
	{
		return (struct1.y1 < struct2.y1);
	}
};

struct pointYSort
{
	inline bool operator() (const Point& struct1, const Point& struct2)
	{
		return (struct1.y < struct2.y);
	}
};

struct pointXSort
{
	inline bool operator() (const Point& struct1, const Point& struct2)
	{
		return (struct1.x < struct2.x);
	}
};


struct quadPtsSort
{
	inline bool operator() (const Point& left, const Point& right)
	{
		return (left.x < right.x) || ((left.x == right.x) && (left.y < right.y));
	}
};