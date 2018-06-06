#include "stdafx.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define WINDOW_NAME "Hough transform"

#define LINES_COLOR Scalar(0,255,0)
#define CIRCLES_COLOR Scalar(0,0,255)

int main(int argc, char** argv) {
	Mat src, gray, tmp, edges;

	src = imread("D:/Documents/Faks/ScanIt/image_processing/image_processing/list1.jpg", CV_LOAD_IMAGE_COLOR);
	cvtColor(src, gray, CV_BGR2GRAY);

  GaussianBlur(gray, tmp, Size(9, 9), 2, 2 );

	Canny(tmp, edges, 50, 200, 3);

  vector<Vec4i> lines;
  vector<Vec3f> circles;

  HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10 );
  //HoughCircles(tmp, circles, CV_HOUGH_GRADIENT, 1, tmp.rows / 8, 200, 100, 0, 0);

  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( src, Point(l[0], l[1]), Point(l[2], l[3]), LINES_COLOR, 3, CV_AA);
  }

  for( size_t i = 0; i < circles.size(); i++ ) {
      Point center(round(circles[i][0]), round(circles[i][1]));
      int radius = round(circles[i][2]);
      circle(src, center, radius, CIRCLES_COLOR, 3, 8, 0);
   }
  namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
  imshow(WINDOW_NAME, src);

  waitKey(0);
  return 0;

}


