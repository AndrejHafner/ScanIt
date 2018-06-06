#include "stdafx.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <iostream>

using namespace cv;
using namespace std;


#define WINDOW_NAME "show"
#define SCALE_RATIO 0.25
#define LINES_DELTA 10
int lowThreshold = 99;
int ratio = 2;
int kernel_size = 3;
bool showCanny = false;

static double distanceBtwPoints(const cv::Point2f &a, const cv::Point2f &b)
{
	double xDiff = a.x - b.x;
	double yDiff = a.y - b.y;

	return std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

int main(int argc, char** argv)
{
	Mat src,resized,gray,detected_edges,dst,thresh_img;
	src = cv::imread("D:/Documents/Faks/ScanIt/image_processing/image_processing/list2.jpg");
	if (src.empty())
		return -1;

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
	namedWindow("eh", WINDOW_AUTOSIZE);
	cv::imshow("eh", thresh_img);
	//thresh_img.copyTo(detected_edges);
	Canny(detected_edges, detected_edges, thresh, thresh*ratio, kernel_size);

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);
	gray.copyTo(dst, detected_edges);


	dilate(dst, dst, Mat());

	vector<vector<Point> > contours;

	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);



	vector<Vec4i> lines;
	
	HoughLinesP(dst.clone(),lines,1, CV_PI / 180, 50, 150, 10);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		Scalar clr(rand() % 255, rand() % 255, rand() % 255);
		line(showCanny ? dst : resized, Point(l[0], l[1]), Point(l[2], l[3]), clr, 3, CV_AA);
	}

	//
	for (size_t i = 0; i < contours.size(); ++i)
	{
		RotatedRect rotatedRect = minAreaRect(Mat(contours[i]));
		
		Point2f *rectPoints = new Point2f[2];
		rotatedRect.points(rectPoints);

		//Mat res = getPerspectiveTransform(rectPoints,)

		Point2f pts[4];
		rotatedRect.points(pts);

		double dist0 = distanceBtwPoints(pts[0], pts[1]);
		double dist1 = distanceBtwPoints(pts[1], pts[2]);

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
			for (int j = 0; j < 4; j++)
				line(showCanny ? dst : resized, pts[j], pts[(j + 1) % 4], Scalar(0, 255, 0), 1, LINE_AA);
		}
	}

	namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
	cv::imshow(WINDOW_NAME, showCanny ? dst : resized);
	cv::waitKey(0);
}