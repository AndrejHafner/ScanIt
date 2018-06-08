#include "stdafx.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "Line.h"
#include <iostream>

using namespace cv;
using namespace std;


#define WINDOW_NAME "show"
#define SCALE_RATIO 0.25
#define LINES_DELTA 10
#define MIN_MERGE_DISTANCE 40
#define MIN_MERGE_ANGLE 5
#define MAX_ANGLE_THRESH 20
#define AREA_SIZE 20

#define degreesToRadians(angleDegrees) ((angleDegrees) * CV_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / CV_PI)


int lowThreshold = 99;
int ratio = 2;
int kernel_size = 3;
bool showCanny = false;

typedef struct LineIntersecPack
{
	Line a;
	Line b;
	Point intersection;
	int score;
	int angle;
} LineIntersecPack;

vector<LineIntersecPack> combinations;
vector<vector<LineIntersecPack>> combinationResult;


vector<Line> proccesLines(vector<Vec4i> raw_lines, Mat dst, Mat resized);
vector<LineIntersecPack> findIntersections(vector<Line> all, Mat dst, Mat resized);

static double distanceBtwPoints(const cv::Point2f &a, const cv::Point2f &b)
{
	double xDiff = a.x - b.x;
	double yDiff = a.y - b.y;

	return std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

cv::Mat OpenWarpPerspective(const cv::Mat& _image, Point2f source_points[], Point2f dest_points[], cv::Mat& _transform_matrix)
{
	// todo do some checks on input.

	/*cv::Point2f source_points[4];
	cv::Point2f dest_points[4];*/

	
	/*source_points[0] = _lu;
	source_points[1] = _ru;
	source_points[2] = _rd;
	source_points[3] = _ld;

	dest_points[0] = _lu_result;
	dest_points[1] = _ru_result;
	dest_points[2] = _rd_result;
	dest_points[3] = _ld_result;*/
	
	cv::Mat dst;
	_transform_matrix = cv::getPerspectiveTransform(source_points, dest_points);
	cv::warpPerspective(_image, dst, _transform_matrix, cv::Size((int)(dest_points[1].x - dest_points[0].x), (int) (dest_points[3].y - dest_points[0].y)));

	return dst;
}


void onMouse(int event, int x, int y, int flags, void* userdata)
{
	if(event == EVENT_MOUSEMOVE)
		printf("X:%d, Y:%d\n", x, y);
}

int getMagnitude(int x1, int y1,int x2, int y2)
{
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

int distancePointLine(int px, int py, int x1, int y1, int x2, int y2)
{
	int distancePointLine,ix,iy;
	int lineMagnitude = getMagnitude(x1, y1, x2, y2);

	if (lineMagnitude < 0.00000001)
		return 9999;

	int u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)));
	int u = u1 /(int)(pow(lineMagnitude, 2));

	if (u < 0.00001 || u > 1) // closest point does not fall withing the line segment, take shorter distance
	{
		ix = getMagnitude(px, py, x1, y1);
		iy = getMagnitude(px, py, x2, y2);
		
		distancePointLine = ix > iy ? iy : ix;

	}
	else
	{
		ix = x1 + u * (x2 - x1);
		iy = y1 + u * (y2 - y1);
		distancePointLine = getMagnitude(px, py, ix, iy);
	}

	return distancePointLine;

}

int getDistance(Line line1, Line line2)
{
	int dist1, dist2, dist3, dist4;

	dist1 = distancePointLine(line1.x1, line1.y1, line2.x1, line2.y1, line2.x2, line2.y2);
	dist2 = distancePointLine(line1.x2, line1.y2, line2.x1, line2.y1, line2.x2, line2.y2);
	dist3 = distancePointLine(line2.x1, line2.y1, line1.x1, line1.y1, line1.x2, line1.y2);
	dist4 = distancePointLine(line2.x2, line2.y2, line1.x1, line1.y1, line1.x2, line1.y2);
	
	vector<int> distances = {dist1,dist2,dist3,dist4};
	std::vector<int>::iterator min = std::min_element(distances.begin(), distances.end());
	return *min;
}

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



Line mergeLineSegments(vector<Line> lines)
{
	if (lines.size() == 1)
		return lines[0];

	Line lineI = lines[0];

	double orientationI = atan2(lineI.y1 - lineI.y2, lineI.x1 - lineI.x2);

	vector<Point> points;

	for (size_t i = 0; i < lines.size(); i++)
	{
		points.push_back(lines[i].begin);
		points.push_back(lines[i].end);
	}
	int degrees = abs(radiansToDegrees(orientationI));
	if (degrees > 45 && degrees < 135)
	{
		std::sort(points.begin(), points.end(), pointYSort());

	}
	else
	{
		std::sort(points.begin(), points.end(), pointXSort());

	}
	return Line(points[0], points[points.size() - 1]);


}

vector<Line> mergeLines(vector<Line> lines)
{
	vector<vector<Line>> superLines;
	vector<Line> superLinesFinal;

	// Merge lines with similar angles to groups
	for (size_t i = 0; i < lines.size(); i++)
	{
		bool createNewGroup = true;
		bool groupUpdated = false;

		// iterate through groups
		for (size_t j = 0; j < superLines.size(); j++)
		{
			// iterate through a group
			for (size_t k = 0; k < superLines[j].size(); k++)
			{
				// first check the distance
				Line line2 = superLines[j][k];
				if (getDistance(line2, lines[i]) < MIN_MERGE_DISTANCE)
				{
					// check the angle
					double orientationI = atan2(lines[i].y1 - lines[i].y2, lines[i].x1 - lines[i].x2);
					double orientationJ = atan2(line2.y1 - line2.y2, line2.x1 - line2.x2);

					if (abs(abs(radiansToDegrees(orientationI)) - abs(radiansToDegrees(orientationI))) < MIN_MERGE_ANGLE)
					{
						superLines[j].push_back(lines[i]);
						createNewGroup = false;
						groupUpdated = true;
						break;
					}



				}
			} // through a group
			if (groupUpdated)
				break;

		} // groups


		if (createNewGroup)
		{
			vector<Line> newGroup;
			newGroup.push_back(lines[i]);

			for (size_t z = 0; z < lines.size(); z++)
			{
				Line line2 = lines[z];
				if (getDistance(lines[z], lines[i]) < MIN_MERGE_DISTANCE)
				{
					double orientationI = atan2(lines[i].y1 - lines[i].y2, lines[i].x1 - lines[i].x2);
					double orientationJ = atan2(line2.y1 - line2.y2, line2.x1 - line2.x2);

					if (abs(abs(radiansToDegrees(orientationI)) - abs(radiansToDegrees(orientationI))) < MIN_MERGE_ANGLE)
					{
						newGroup.push_back(line2);
						
					}
				}
			}
			superLines.push_back(newGroup);

		}

	} // lines

	for (size_t j = 0; j < superLines.size(); j++)
	{
		superLinesFinal.push_back(mergeLineSegments(superLines[j]));
	}

	return superLinesFinal;

}

double Slope(int x0, int y0, int x1, int y1) {
    return (double)(y0 - y1) / (x0 - x1);
}

pair<Point, Point> getFullLine(cv::Mat img, cv::Point a, cv::Point b) {

	//cv::Point p, q;

	//if (p2.x == p1.x)
	//{
	//	p = cv::Point(p1.x, 0);
	//	q = cv::Point(p1.x, (img).rows);
	//}
	//else
	//{
	//	double a = (double)(p2.y - p1.y) / (double)(p2.x - p1.x);
	//	double b = p1.y - a * p1.x;

	//	p = cv::Point(0, b);
	//	q = cv::Point((img).rows, a*img.rows + b);



	//	//clipline to the image borders. It prevents a known bug on OpenCV
	//	//versions 2.4.X when drawing 
	//	cv::clipLine(cv::Size((img).rows, (img).cols), p, q);
	//}

	double slope = Slope(a.x, a.y, b.x, b.y);

	Point p(0, 0), q(img.cols, img.rows);

	p.y = -(a.x - p.x) * slope + a.y;
	q.y = -(b.x - q.x) * slope + b.y;
	cv::clipLine(cv::Size((img).cols, (img).rows), p, q);



	/*p.y = p.y < 0 ? 0 : (p.y > img.rows ? img.rows : p.y);
	q.y = q.y < 0 ? 0 : (q.y > img.rows ? img.rows : q.y);

	p.x = p.x < 0 ? 0 : (p.x > img.cols ? img.cols : p.x);
	q.x = q.x < 0 ? 0 : (q.x > img.cols ? img.cols : q.x);*/

	return make_pair(p, q);
}

void fullLine(cv::Mat img, cv::Point p1, cv::Point p2, cv::Scalar color) {
	
	pair<Point, Point> pts = getFullLine(img, p1, p2);
	
	cv::line(img, pts.first, pts.second, color, 2);
}



double cross(Point v1, Point v2) {
	return v1.x*v2.y - v1.y*v2.x;
}

pair<Point, Point> getLineEndpoints(Line line,Size imgSize)
{
//	Point a = line.begin;
//	Point b = line.end;
//	double slope = Slope(a.x, a.y, b.x, b.y);
//	double n = a.y - slope * a.x;
//	Point p(0, 0), q(imgSize.width, imgSize.height);
//
//	p.y = -(a.x - p.x) * slope + a.y;
//	q.y = -(b.x - q.x) * slope + b.y;
//
//	/*int y1 = (0*slope) + n;
//	int y2 = ((double)slope *(double) imgSize.width) + n;
//	int x1 = (0 - n) / slope;
//	int x2 = (double)(imgSize.height - n) /(double) slope;*/
//
//	//int y1 = p.y;
//	//int y2 = q.y;
//	/*int x1 = p.x;
//	int x2 = q.x;*/
//
//
//	//return make_pair(p, q);
//	/*x1 = x1 < 0 ? 0 : (x1 > imgSize.width ? imgSize.width : x1);
//	x2 = x2 < 0 ? 0 : (x2 > imgSize.width ? imgSize.width : x2);*/
//	p.y = p.y < 0 ? 0 : (p.y > imgSize.height ? imgSize.height : p.y);
//	q.y = q.y < 0 ? 0 : (q.y > imgSize.height ? imgSize.height : q.y);
//
	cv::Point p, q;

	Point p1 = line.begin;
	Point p2 = line.end;

	//test if line is vertical, otherwise computes line equation
	//y = ax + b
	if (p2.x == p1.x)
	{
		p = cv::Point(p1.x, 0);
		q = cv::Point(p1.x, imgSize.height);
	}
	else
	{
		double a = (double)(p2.y - p1.y) / (double)(p2.x - p1.x);
		double b = p1.y - a * p1.x;

		p = cv::Point(0, b);
		q = cv::Point(imgSize.height, a*imgSize.height + b);



		//clipline to the image borders. It prevents a known bug on OpenCV
		//versions 2.4.X when drawing 
		//cv::clipLine(cv::Size((img).rows, (img).cols), p, q);
	}

	return make_pair(p, q);
}

bool getIntersectionPoint(Line a, Line b,Size imgSize, Point2f &intPnt,Mat &dst) {
	

	pair<Point, Point> aEndPoints = getLineEndpoints(a, imgSize);
	pair<Point, Point> bEndPoints = getLineEndpoints(b, imgSize);
	circle(dst, aEndPoints.first, 3, Scalar(0, 0, 255), 4);
	circle(dst, aEndPoints.second, 3, Scalar(0, 0, 255), 4);
	circle(dst, bEndPoints.first, 3, Scalar(0, 255, 0), 10);
	circle(dst, bEndPoints.second, 3, Scalar(0, 255, 0), 10);
	Point a1 = aEndPoints.first;
	Point a2 = aEndPoints.second;
	Point b1 = bEndPoints.first;
	Point b2 = bEndPoints.second;
	Point p = a1;
	Point q = b1;
	Point r(a2 - a1);
	Point s(b2 - b1);

	if (cross(r, s) == 0) { return false; }

	double t = cross(q - p, s) / cross(r, s);

	intPnt = p + t * r;
	return true;
}
void go(int offset, int k,vector<LineIntersecPack> candidates) {
	if (k == 0) {
		//pretty_print(combinations);
		combinationResult.push_back(combinations);
		return;
	}
	for (int i = offset; i <= candidates.size() - k; ++i) {
		combinations.push_back(candidates[i]);
		go(i + 1, k - 1, candidates);
		combinations.pop_back();
	}
}

bool getIntersection(Mat img, Line a, Line b, Point2f &r)
{
	/*double m1 = (a.y1 - a.y2) / (a.x1 - a.x2);
	double m2 = (b.y1 - b.y2) / (b.x1 - b.x2);

	int c1 = a.y1 - (m1 * a.x1);
	int c2 = b.y1 - (m2 * b.x1);

	int interX = (c2 - c1) / (m1 - m2);
	int interY = m1 * interX + c1;

	return Point(interX, interY);*/
	pair<Point, Point> aPts = getFullLine(img, a.begin, a.end);
	pair<Point, Point> bPts = getFullLine(img, b.begin, b.end);

	Point2f o1 = aPts.first;
	Point2f p1 = aPts.second;

	Point2f o2 = bPts.first;
	Point2f p2 = bPts.second;

	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	if (r.x < 0 || r.y < 0 || r.y > img.rows || r.x > img.cols)
		return false;
	return true;
}

vector<LineIntersecPack> filterOptions( Mat img2, vector<LineIntersecPack> options)
{
	vector<LineIntersecPack> finalOpts;
	Mat img = img2.clone();
	for (size_t i = 0; i < options.size(); i++)
	{
		options[i];
		int degreesA = radiansToDegrees(CV_PI - abs(atan(Slope(options[i].a.x1, options[i].a.y1, options[i].a.x2, options[i].a.y2)) - atan(Slope(options[i].b.x1, options[i].b.y1, options[i].b.x2, options[i].b.y2))));
		if (abs(degreesA - 90) < MAX_ANGLE_THRESH) //  find the ones that have atleast +- 10 to right angle at the intersection
		{
			finalOpts.push_back(options[i]);
			Point inter = options[i].intersection;
			Rect area(inter.x - AREA_SIZE, inter.y - AREA_SIZE, AREA_SIZE * 2, AREA_SIZE * 2);
			Mat res = img(area);
			Mat gray, detected_edges,thresh_img,dst;
			//int blockSize = 2;
			//int apertureSize = 3;
			//int thresh = 250;

			//double k = 0.05;
			//Mat dst_norm,dst_norm_scaled,gray,bi;
			//cvtColor(res, gray, COLOR_BGR2GRAY);


			//GaussianBlur(gray, detected_edges, Size(3, 3), 2, 2);

			//int lowThreshold = 90;
			//int ratio = 3;
			//int kernel_size = 3;
			///// Canny detector
			//double thresh = threshold(gray, thresh_img, 0, 255, CV_THRESH_OTSU);
			////thresh_img.copyTo(detected_edges);
			//Canny(detected_edges, detected_edges, thresh, thresh*ratio, kernel_size);

			///// Using Canny's output as a mask, we display our result
			//dst = Scalar::all(0);
			//gray.copyTo(dst, detected_edges);


			////dilate(dst, dst, Mat());
			////bilateralFilter(gray, bi, 5, 75, 75);
			////Mat dst = res.clone();
			////cornerHarris(bi, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
			////normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
			////convertScaleAbs(dst_norm, dst_norm_scaled);
			////for (int j = 0; j < dst_norm.rows; j++)
			////{
			////	for (int i = 0; i < dst_norm.cols; i++)
			////	{
			////		if ((int)dst_norm.at<float>(j, i) > thresh)
			////		{
			////			circle(res, Point(i, j), 4, Scalar(0), 1, 8, 0);
			////		}
			////	}
			////}

			//vector<Vec4i> lines;
			//imshow("i" + to_string(i), dst);
			//HoughLinesP(dst, lines, 1, CV_PI / 180, 20, 150, 10);
			///*for (size_t i = 0; i < lines.size(); i++)
			//{
			//Vec4i l = lines[i];
			//Scalar clr(rand() % 255, rand() % 255, rand() % 255);
			//line(showCanny ? dst : resized, Point(l[0], l[1]), Point(l[2], l[3]), clr, 3, CV_AA);
			//}*/
			//
			//vector<Line> linesP = proccesLines(lines, res, res.clone());

			//findIntersections(linesP, res, res);


			//imshow(to_string(i), res);
		}
	}
	return finalOpts;

}

vector<Line> proccesLines(vector<Vec4i> raw_lines, Mat dst, Mat resized)
{
	vector<Line> lines,linesX,linesY;
	// store the lines to amke them more readable
	for (size_t i = 0; i < raw_lines.size(); i++)
	{
		Vec4i l = raw_lines[i];
		lines.push_back(Line(l[0], l[2], l[1], l[3]));
	}

	for (size_t i = 0; i < lines.size(); i++)
	{
		// Retrieve the orientation and sort them into x and y
		double orientation = atan2(lines[i].y1 - lines[i].y2, lines[i].x1 - lines[i].x2);
		int degrees = abs(radiansToDegrees(orientation));
		if (degrees > 45 && degrees < 135)
			linesY.push_back(lines[i]);
		else
			linesX.push_back(lines[i]);
	}
	//for (size_t i = 0; i < linesX.size(); i++)
	//	line(dst , linesX[i].begin, linesX[i].end, Scalar(0,255,0), 2, CV_AA);
	//for (size_t i = 0; i < linesY.size(); i++)
	//	line(dst, linesY[i].begin, linesY[i].end, Scalar(0, 0,255), 2, CV_AA);

	std::sort(linesX.begin(), linesX.end(), lineXSort());
	std::sort(linesY.begin(), linesY.end(), lineYSort());

	vector<Line> mergedX = mergeLines(linesX);
	vector<Line> mergedY = mergeLines(linesY);

	for (size_t i = 0; i < mergedX.size(); i++)
		line(dst , mergedX[i].begin, mergedX[i].end, Scalar(0,255,0), 2, CV_AA);
	for (size_t i = 0; i < mergedY.size(); i++)
		line(dst, mergedY[i].begin, mergedY[i].end, Scalar(0, 0,255), 2, CV_AA);

	vector<Line> all;
	all.reserve(all.size() + mergedX.size() + mergedY.size());
	all.insert(all.end(), mergedX.begin(), mergedX.end());
	all.insert(all.end(), mergedY.begin(), mergedY.end());

	return all;

}

vector<LineIntersecPack> findIntersections(vector<Line> all,Mat dst, Mat resized)
{
	for (size_t i = 0; i < all.size(); i++)
	{
		fullLine(resized, all[i].begin, all[i].end, Scalar(255, 100, 0));
		pair<Point, Point> pts = getFullLine(dst, all[i].begin, all[i].end);
		circle(resized, pts.first, 3, Scalar(255, 255, 0), 5);
		circle(resized, pts.second, 3, Scalar(255, 255, 0), 5);



	}

	//vector<Point> intersections;
	vector<LineIntersecPack> cornerCandidates;

	for (size_t i = 0; i < all.size(); i++)
		for (size_t j = i + 1; j < all.size(); j++)
		{

			Point2f cross;
			if (getIntersection(dst, all[i], all[j], cross))
			{
				circle(dst, cross, 3, Scalar(0, 0, 255), 5);
				//intersections.push_back(cross);
				LineIntersecPack e = { all[i],all[j],cross,0 };
				cornerCandidates.push_back(e);

			}


		}


	return cornerCandidates;
	
}

void filterAndFindRectangles(vector<LineIntersecPack> candidates, Mat resized,Mat dst)
{
	vector<LineIntersecPack> filtered = filterOptions(resized, candidates);

	for (size_t i = 0; i < filtered.size(); i++)
	{
		circle(resized, filtered[i].intersection, 3, Scalar(0), 3);

	}


	cv::imshow(WINDOW_NAME, dst);
	cv::waitKey(0);
}

int main(int argc, char** argv)
{
	
	Mat src,resized,gray,detected_edges,dst,thresh_img;
	src = cv::imread("D:/Documents/Faks/ScanIt/image_processing/image_processing/list4.jpg");
	if (src.empty())
		return -1;
	namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);

	resize(src, resized, Size(src.cols * SCALE_RATIO, src.rows * SCALE_RATIO));
	cvtColor(resized, gray, COLOR_BGR2GRAY);

	//vector<vector<Point> > contours;

	//findContours(gray.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	GaussianBlur(gray, detected_edges, Size(3, 3),2,2);

	int lowThreshold = 90;
	int ratio = 3;
	int kernel_size = 3;
	/// Canny detector
	double thresh = threshold(gray, thresh_img, 0, 255, CV_THRESH_OTSU);
	//thresh_img.copyTo(detected_edges);
	Canny(detected_edges, detected_edges, thresh, thresh*ratio, kernel_size);

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);
	gray.copyTo(dst, detected_edges);


	dilate(dst, dst, Mat());

	vector<vector<Point> > contours;

	//findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//int blockSize = 2;
	//int apertureSize = 3;
	//double k = 0.05;
	//Mat dst_norm,dst_norm_scaled;
	//cornerHarris(dst, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
	//normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	//convertScaleAbs(dst_norm, dst_norm_scaled);
	//for (int j = 0; j < dst_norm.rows; j++)
	//{
	//	for (int i = 0; i < dst_norm.cols; i++)
	//	{
	//		if ((int)dst_norm.at<float>(j, i) > thresh)
	//		{
	//			circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
	//		}
	//	}
	//}

	//imshow(WINDOW_NAME, dst_norm_scaled);
	//waitKey(0);

	vector<Vec4i> lines;
	
	HoughLinesP(dst.clone(),lines,1, CV_PI / 180, 50, 150, 10);
	/*for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		Scalar clr(rand() % 255, rand() % 255, rand() % 255);
		line(showCanny ? dst : resized, Point(l[0], l[1]), Point(l[2], l[3]), clr, 3, CV_AA);
	}*/
	Mat resClone = resized.clone();
	vector<Line> linesP = proccesLines(lines, resized,resized.clone());
	
	filterAndFindRectangles(findIntersections(linesP, dst, resized),resClone,resized);

	//
	for (size_t i = 0; i < contours.size(); ++i)
	{
		RotatedRect rotatedRect = minAreaRect(Mat(contours[i]));
		
		Point2f *rectPoints = new Point2f[2];
		rotatedRect.points(rectPoints);


		Point2f pts[4];
		rotatedRect.points(pts);

		double dist0 = distanceBtwPoints(pts[0], pts[1]);
		double dist1 = distanceBtwPoints(pts[1], pts[2]);
		
		double distY = distanceBtwPoints(pts[0], pts[1]);
		double distX = distanceBtwPoints(pts[1], Point2f(427,204));

		
		/*Point2f dest_pts[4];
		dest_pts[0] = Point2f(0, 0);
		dest_pts[1] = Point2f(distX, 0);
		dest_pts[2] = Point2f(distX,distY);
		dest_pts[3] = Point2f(0, distY);

		Point2f src_pts[4];
		src_pts[0] = pts[1];
		src_pts[1] = Point2f(427, 204);
		src_pts[2] = Point2f(689, 646);
		src_pts[3] = pts[0];


		Mat transformMat;
		Mat res = OpenWarpPerspective(resized.clone(), src_pts, dest_pts, transformMat);
		namedWindow("eh", WINDOW_AUTOSIZE);

		cv::imshow("eh", res);*/
		//cv::waitKey();



		//Mat res = getPerspectiveTransform(pts,)


		if ((dist0 > 30 && dist1 > 30))
		{


			//Mat M = getRotationMatrix2D(rotatedRect.center, rotatedRect.angle, 1.0);
			//warpAffine(src, rotated, M, src.size(), INTER_CUBIC);
			//// crop the resulting image
			//getRectSubPix(rotated, rotatedRect.size , rotatedRect.center, cropped);
			//Point2f croppedCenter(cropped.cols / 2.0F, cropped.rows / 2.0F);
			//Mat rot_mat = getRotationMatrix2D(croppedCenter, 0, 1.0);
			//Mat dst, res, gray_res;
			//warpAffine(cropped, dst, rot_mat, cropped.size());
			////imshow(std::to_string(i), dst);
			////imshow("nocon", dst);
			////applyCLAHE(dst, dst);
			////imshow("con", dst);

			//cvtColor(dst, gray_res, COLOR_BGR2GRAY);
			////resize(gray_res, res, resizeSize);
			//for (int k = 0; k < res.cols; k++)
			//{
			//	res.row(0).col(k) = 255;
			//	res.row(res.rows - 1).col(k) = 255;
			//	res.row(k).col(0) = 255;
			//	res.row(k).col(res.cols - 1) = 255;


			//}
			//Point massCenter = findMassCenter(res);
			/*printf("%d %d\n", massCenter.x, massCenter.y);
			res = moveToMassCenter(res);*/
			//res.row(massCenter.y).col(massCenter.x) = 0;
			//imwrite("C:/Users/Likewse/Documents/digit-recognition/digits/" + std::to_string(cnt) + ".jpeg", res);
			//cnt++;
			for (int j = 0; j < 4; j++) {
				line(showCanny ? dst : resized, pts[j], pts[(j + 1) % 4], Scalar(0, 255, 0), 1, LINE_AA);
				circle(showCanny ? dst : resized, pts[j], 3, Scalar(0, 0, 255, 4));
			}
		}
	}

	cv::imshow(WINDOW_NAME, showCanny ? dst : resized);

	cv::waitKey(0);
}