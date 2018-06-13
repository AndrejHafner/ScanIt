//
// Created by Likewise on 12. 06. 2018.
//

#include "ImageProcessing.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "Line.h"
#include <iostream>
#include <android/log.h>

using namespace cv;
using namespace std;


int lowThreshold = 99;
int ratio = 2;
int kernel_size = 3;
bool showCanny = false;



vector<Point> combinations;
vector<Line> combinationsLine;


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

vector<Line> mergeLines(vector<Line> lines, int minMergeDistance, int minMergeAngle)
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
                if (getDistance(line2, lines[i]) < minMergeDistance)
                {
                    // check the angle
                    double orientationI = atan2(lines[i].y1 - lines[i].y2, lines[i].x1 - lines[i].x2);
                    double orientationJ = atan2(line2.y1 - line2.y2, line2.x1 - line2.x2);

                    if (abs(abs(radiansToDegrees(orientationI)) - abs(radiansToDegrees(orientationI))) < minMergeAngle)
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
                if (getDistance(lines[z], lines[i]) < minMergeDistance)
                {
                    double orientationI = atan2(lines[i].y1 - lines[i].y2, lines[i].x1 - lines[i].x2);
                    double orientationJ = atan2(line2.y1 - line2.y2, line2.x1 - line2.x2);

                    if (abs(abs(radiansToDegrees(orientationI)) - abs(radiansToDegrees(orientationI))) < minMergeAngle)
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

void pointCombinations(int offset, int k,vector<Point> candidates,vector<vector<Point>> *combinationRes) {
    if (k == 0) {
            combinationRes->push_back(combinations);
        return;
    }
    for (int i = offset; i <= candidates.size() - k; ++i) {
        combinations.push_back(candidates[i]);
        pointCombinations(i + 1, k - 1, candidates,combinationRes);
        combinations.pop_back();
    }
}

void lineCombinations(int offset, int k, vector<Line> candidates, vector<vector<Line>> *combinationRes) {
    if (k == 0) {
        //pretty_print(combinations);
        combinationRes->push_back(combinationsLine);
        return;
    }
    for (int i = offset; i <= candidates.size() - k; ++i) {
        combinationsLine.push_back(candidates[i]);
        lineCombinations(i + 1, k - 1, candidates, combinationRes);
        combinationsLine.pop_back();
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

bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

Quadrilateral constructQuad(Point p1, Point p2, Point p3, Point p4)
{
    Point botleft, topleft, topright, bottomright;
    vector<Point> pts{p1,p2,p3,p4};
    std::sort(pts.begin(), pts.end(), quadPtsSort());
    // pts sorted by x min->max
    if (pts[0].y > pts[1].y)
    {
        botleft = pts[0];
        topleft = pts[1];
    }
    else
    {
        botleft = pts[1];
        topleft = pts[0];
    }

    if (pts[2].y > pts[3].y)
    {
        topright = pts[3];
        bottomright = pts[2];
    }
    else
    {
        topright = pts[2];
        bottomright = pts[3];
    }

    Quadrilateral quad = {botleft,topleft,topright,bottomright,0};
    return quad;

}


float scoreImageEdges(Point p1, Point p2, Mat cln)
{
    float score = 0;

    float angle = radiansToDegrees(atan2(p1.y - p2.y, p1.x - p2.x));
    //imshow("kek", cln);
    // Find the centre
    Point2f center = (p1 + p2) / 2;
    Size2f size(distanceBtwPoints(p1, p2), SCORE_RECT_WIDTH);
    RotatedRect rect(center, size, angle);

    Mat M, rotated, cropped, translated;
    // get angle and size from the bounding box
    Size rect_size = rect.size;
    Size cln_size = cln.size();
    // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
    if (rect.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);

    }

    Point2f translateBy(cln.cols/2, cln.rows/2);
    // get the rotation matrix

    // Translate to middle
    Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, translateBy.x, 0, 1, translateBy.y);
    warpAffine(cln, translated, trans_mat, cln_size * 2);

    // Rotate
    M = getRotationMatrix2D(rect.center + translateBy, angle, 1.0);
    warpAffine(translated, rotated, M, translated.size(), INTER_CUBIC);
    //imshow("rotated", rotated);
    // Crop the resulting image
    getRectSubPix(rotated, rect_size, rect.center + translateBy, cropped);
    cln.release();
    rotated.release();
    translated.release();
    trans_mat.release();
    M.release();

    Mat gray,thresh_img,detected_edges,dst;
    cvtColor(cropped, gray, COLOR_BGR2GRAY);

    //vector<vector<Point> > contours;

    //findContours(gray.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    GaussianBlur(gray, detected_edges, Size(3, 3), 2, 2);

    int lowThreshold = 90;
    int ratio = 1.5;
    int kernel_size = 3;
    /// Canny detector
    //imshow("ek", detected_edges.clone());

    double thresh = threshold(detected_edges, thresh_img, 0, 200, CV_THRESH_OTSU);
    //thresh_img.copyTo(detected_edges);
    Canny(detected_edges, detected_edges, thresh, thresh*ratio, kernel_size);

    dst = Scalar::all(0);
    gray.copyTo(dst, detected_edges);


    dilate(dst, dst, Mat());
    vector<vector<Point> > contours;

    findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++)
    {
        RotatedRect rotatedRect = minAreaRect(Mat(contours[i]));

        Point2f *rectPoints = new Point2f[2];
        rotatedRect.points(rectPoints);


        Point2f pts[4];
        rotatedRect.points(pts);

        for (int j = 0; j < 4; j++)
            line(dst, pts[j], pts[(j + 1) % 4], Scalar(0,0,255), 1, 8);
    }
    /*vector<Vec4i> lines;
    HoughLinesP(dst.clone(), lines, 1, CV_PI / 180, 50, 150, 10);
    vector<Line> merged = proccesLines(lines, dst, dst,30,10);
    for (size_t i = 0; i < merged.size(); i++)
        line(dst, merged[i].begin, merged[i].end, Scalar(0, 0, 255), 2, CV_AA);
*/
    //imshow("mek", dst);

    //int lowThreshold = 90;
    //int ratio = 3;
    //int kernel_size = 3;
    ///// Canny detector
    //double thresh = threshold(gray, thresh_img, 0, 255, CV_THRESH_OTSU);
    ////thresh_img.copyTo(detected_edges);
    //Canny(detected_edges, detected_edges, thresh, thresh*ratio, kernel_size);

    /// Using Canny's output as a mask, we display our result
    //dst = Scalar::all(0);
    //gray.copyTo(dst, detected_edges);


    //dilate(dst, dst, Mat());

    //waitKey(0);
    gray.release(); thresh_img.release();  detected_edges.release();  dst.release();
    return score;
}

float scoreQuadrilateral(Quadrilateral quad, Mat img)
{
    float score = 0;


    /*bool paint = false;
    Mat cln = img.clone();

    if (paint)
    {
        cv::line(cln, quad.p1, quad.p2, Scalar(255, 0, 0), 2);
        cv::line(cln, quad.p2, quad.p3, Scalar(0, 255, 0), 2);
        cv::line(cln, quad.p3, quad.p4, Scalar(0, 0, 255), 2);
        cv::line(cln, quad.p4, quad.p1, Scalar(0), 2);
    }
    score += scoreImageEdges(quad.p1, quad.p2, cln);
    cln.release();
    cln = img.clone();
    if (paint)
    {
        cv::line(cln, quad.p1, quad.p2, Scalar(255, 0, 0), 2);
        cv::line(cln, quad.p2, quad.p3, Scalar(0, 255, 0), 2);
        cv::line(cln, quad.p3, quad.p4, Scalar(0, 0, 255), 2);
        cv::line(cln, quad.p4, quad.p1, Scalar(0), 2);
    }
    score += scoreImageEdges(quad.p2, quad.p3, cln);
    cln.release();
    cln = img.clone();
    if (paint)
    {
        cv::line(cln, quad.p1, quad.p2, Scalar(255, 0, 0), 2);
        cv::line(cln, quad.p2, quad.p3, Scalar(0, 255, 0), 2);
        cv::line(cln, quad.p3, quad.p4, Scalar(0, 0, 255), 2);
        cv::line(cln, quad.p4, quad.p1, Scalar(0), 2);
    }
    score += scoreImageEdges(quad.p3, quad.p4, cln);
    cln.release();
    cln = img.clone();
    if (paint)
    {
        cv::line(cln, quad.p1, quad.p2, Scalar(255, 0, 0), 2);
        cv::line(cln, quad.p2, quad.p3, Scalar(0, 255, 0), 2);
        cv::line(cln, quad.p3, quad.p4, Scalar(0, 0, 255), 2);
        cv::line(cln, quad.p4, quad.p1, Scalar(0), 2);
    }
    score += scoreImageEdges(quad.p4, quad.p1, cln);
    cln.release();*/

    return score;
}

bool scoreBestQuadrilateral(vector<Quadrilateral> quads,Mat img, Quadrilateral &res)
{
    vector<Quadrilateral> filtered;
    for (size_t i = 0; i < quads.size(); i++)
    {
        int degrees1 = radiansToDegrees(CV_PI -
                                        abs(atan(Slope(quads[i].p1.x, quads[i].p1.y, quads[i].p2.x, quads[i].p2.y)) -
                                            atan(Slope(quads[i].p2.x, quads[i].p2.y, quads[i].p3.x, quads[i].p3.y))));
        int degrees2 = radiansToDegrees(CV_PI -
                                        abs(atan(Slope(quads[i].p2.x, quads[i].p2.y, quads[i].p3.x, quads[i].p3.y)) -
                                            atan(Slope(quads[i].p3.x, quads[i].p3.y, quads[i].p4.x, quads[i].p4.y))));
        int degrees3 = radiansToDegrees(CV_PI -
                                        abs(atan(Slope(quads[i].p3.x, quads[i].p3.y, quads[i].p4.x, quads[i].p4.y)) -
                                            atan(Slope(quads[i].p4.x, quads[i].p4.y, quads[i].p1.x, quads[i].p1.y))));
        int degrees4 = radiansToDegrees(CV_PI -
                                        abs(atan(Slope(quads[i].p4.x, quads[i].p4.y, quads[i].p1.x, quads[i].p1.y)) -
                                            atan(Slope(quads[i].p1.x, quads[i].p1.y, quads[i].p2.x, quads[i].p2.y))));
        int thresh = FILTER_ANGLE_THRESH;
        if (abs(degrees1 - 90) < thresh && abs(degrees2 - 90) < thresh && abs(degrees3 - 90) < thresh && abs(degrees4 - 90) < thresh)
        {
            vector<Point> ctr{ quads[i].p1,quads[i].p2 ,quads[i].p3 ,quads[i].p4 };
            double area = contourArea(ctr);
            quads[i].area = area;
            filtered.push_back(quads[i]);
        }

    }

    int max = 0;
    int maxIdx = -1;
    for (size_t i = 0; i < filtered.size(); i++)
    {
        if (filtered[i].area > max)
        {
            max = filtered[i].area;
            maxIdx = i;
        }
    }
    if (maxIdx < 0) return false;
    Quadrilateral finalQuad = filtered[maxIdx];
    res = finalQuad;

    cv::line(img, finalQuad.p1, finalQuad.p2, Scalar(255, 0, 0), 2);
    cv::line(img, finalQuad.p2, finalQuad.p3, Scalar(0, 255, 0), 2);
    cv::line(img, finalQuad.p3, finalQuad.p4, Scalar(0, 0, 255), 2);
    cv::line(img, finalQuad.p4, finalQuad.p1, Scalar(0), 2);
    //imshow("eeke", img);
    return true;
}

bool filterOptions(Mat img2, vector<LineIntersecPack> options,Quadrilateral &retQuad)
{
    vector<LineIntersecPack> finalOpts;
    Mat img = img2.clone();
    for (size_t i = 0; i < options.size(); i++)
    {
        //options[i];
        int degreesA = radiansToDegrees(CV_PI - abs(atan(Slope(options[i].a.x1, options[i].a.y1, options[i].a.x2, options[i].a.y2)) - atan(Slope(options[i].b.x1, options[i].b.y1, options[i].b.x2, options[i].b.y2))));
        if (abs(degreesA - 90) < MAX_ANGLE_THRESH) //  find the ones that have atleast +- 10 to right angle at the intersection
        {
            finalOpts.push_back(options[i]);

        }
    }

    vector<Point> points;
    for (size_t i = 0; i < finalOpts.size(); i++)
        points.push_back(finalOpts[i].intersection);

    combinations.clear();
    combinationsLine.clear();

    // generate all combinations and store them into combinations
    if(points.size() <= 1) return false;
    vector<vector<Point>> pointCombination;
    pointCombinations(0, 2, points, &pointCombination);
    vector<Line> allLines;
    for (size_t i = 0; i < pointCombination.size(); i++)
    {
        allLines.push_back(Line(pointCombination[i][0], pointCombination[i][1]));
    }


    vector<vector<Line>> lineComb;
    vector<Quadrilateral> quads;
    if(allLines.size() <= 1) return false;
    lineCombinations(0, 2, allLines, &lineComb);
    int count = 0;
    for (size_t i = 0; i < lineComb.size(); i++)
    {
        // (o1,p1) , (o2,p2)
        /*Point2f x = lineComb[i][1].begin - lineComb[i][0].begin;
        Point2f d1 = lineComb[i][0].end - lineComb[i][0].begin;
        Point2f d2 = lineComb[i][1].end - lineComb[i][1].begin;

        float cross = d1.x*d2.y - d1.y*d2.x;*/
        Point vecA = lineComb[i][0].begin - lineComb[i][0].end;
        Point vecB = lineComb[i][1].begin - lineComb[i][1].end;

        int degrees = radiansToDegrees(CV_PI - abs(
                atan(Slope(lineComb[i][0].x1, lineComb[i][0].y1, lineComb[i][0].x2, lineComb[i][0].y2)) -
                atan(Slope(lineComb[i][1].x1, lineComb[i][1].y1, lineComb[i][1].x2, lineComb[i][1].y2))));


        bool minAngleCondition = abs(degrees - 180) > MAX_ANGLE_THRESH;

        // Find all quadrilaterals that suffice the conditions
        if (doIntersect(lineComb[i][0].begin - vecA * LINE_SHRINK,
                        lineComb[i][0].end + vecA * LINE_SHRINK,
                        lineComb[i][1].begin - vecB * LINE_SHRINK,
                        lineComb[i][1].end + vecB * LINE_SHRINK) &&
            minAngleCondition)
        {


            Quadrilateral quad = constructQuad(lineComb[i][0].begin,
                                               lineComb[i][0].end,
                                               lineComb[i][1].begin,
                                               lineComb[i][1].end);
            quads.push_back(quad);


        }
        /*if (getIntersection(img2, lineComb[i][0], lineComb[i][1],pt))
        {
            count++;
        }*/
    }
    if(quads.size() == 0) return false;
    Quadrilateral quad = { Point(0,0),Point(0,0),Point(0,0),Point(0,0),0 };
    if (scoreBestQuadrilateral(quads, img2, quad))
    {
        retQuad = quad;
//
//        //imshow("juhej", res);
        //waitKey(0);
        return true;

    }
    return false;


}

vector<Line> proccesLines(vector<Vec4i> raw_lines, Mat dst, Mat resized, int minMergeDist,int minMergeAngle)
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

    vector<Line> mergedX = mergeLines(linesX, minMergeDist, minMergeAngle);
    vector<Line> mergedY = mergeLines(linesY, minMergeDist, minMergeAngle);

    for (size_t i = 0; i < mergedX.size(); i++)
        line(dst , mergedX[i].begin, mergedX[i].end, Scalar(0,0,255), 2, CV_AA);
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
    Quadrilateral quad;
    bool success = filterOptions(resized, candidates,quad);




    //cv::imshow(WINDOW_NAME, dst);
    //cv::waitKey(0);
}

// https://stackoverflow.com/questions/11764575/python-2-7-3-opencv-2-4-after-rotation-window-doesnt-fit-image
void myrotate(const Mat src, Mat &dest, double angle){

    int w = src.size().width, h = src.size().height;

    // resize the destination image
    Size2d new_size = Size2d(abs(w * COS((int)angle % 180)) + abs(h * SIN((int)angle % 180)), abs(w * SIN((int)angle % 180)) + abs(h * COS((int)angle % 180)));
    dest = Mat(new_size, src.type());

    // this is our rotation point
    Size2d old_size = src.size();
    Point2d rot_point = Point2d(old_size.width / 2.0, old_size.height / 2.0);

    // and this is the rotation matrix
    // same as in the opencv docs, but in 3x3 form
    double a = COS(angle), b = SIN(angle);
    Mat rot_mat   = (Mat_<double>(3,3) << a, b, (1 - a) * rot_point.x - b * rot_point.y, -1 * b, a, b * rot_point.x + (1 - a) * rot_point.y, 0, 0, 1);

    // next the translation matrix
    double offsetx = (new_size.width - old_size.width) / 2,
            offsety = (new_size.height - old_size.height) / 2;
    Mat trans_mat = (Mat_<double>(3,3) << 1, 0, offsetx , 0, 1, offsety, 0, 0, 1);

    // multiply them: we rotate first, then translate, so the order is important!
    // inverse order, so that the transformations done right
    Mat affine_mat = Mat(trans_mat * rot_mat).rowRange(0, 2);

    // now just apply the affine transformation matrix
    warpAffine(src, dest, affine_mat, new_size, INTER_LINEAR);
}


bool getDocumentOutline(Mat &src, Mat &extracted,bool extract)
{
    Mat  resized, gray, detected_edges, thresh_img;
    Mat dst(src.rows,src.cols,src.type());
    //src = cv::imread("D:/Documents/Faks/ScanIt/image_processing/image_processing/list1.jpg");
    //if (src.empty())
    //	return ;
    //namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
    __android_log_print(ANDROID_LOG_DEBUG, "ImageProcessing", "%d %d", src.cols, src.rows);
    Mat src_cln = src.clone();

    resize(src_cln, resized, Size(src.cols * SCALE_RATIO, src.rows * SCALE_RATIO));
    cvtColor(resized, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, detected_edges, Size(3, 3), 2, 2);

    int lowThreshold = 90;
    int ratio = 3;
    int kernel_size = 3;
    double thresh = threshold(gray, thresh_img, 0, 255, CV_THRESH_OTSU);
    Canny(detected_edges, detected_edges, thresh, thresh*ratio, kernel_size);
    __android_log_print(ANDROID_LOG_DEBUG, "ImageProcessing", "Made it to here!");
    __android_log_print(ANDROID_LOG_DEBUG, "ImageProcessing", "DstSize:%d %d detectedEdgessize: %d %d",dst.rows, dst.cols, detected_edges.rows,detected_edges.cols);

//    src = detected_edges;
    /// Using Canny's output as a mask, we display our result

    dst = Scalar::all(0);
    gray.copyTo(dst, detected_edges);


    dilate(dst, dst, Mat());
//    src = dst;


    vector<Vec4i> lines;

    HoughLinesP(dst.clone(), lines, 1, CV_PI / 180, 50, 150, 10);


    Mat resClone = resized.clone();
    vector<Line> linesP = proccesLines(lines, resized, resized.clone(), MIN_MERGE_DISTANCE, MIN_MERGE_ANGLE);

//    filterAndFindRectangles(findIntersections(linesP, dst, resized), resClone, resized);
    Quadrilateral quad;

    bool success = filterOptions(resized, findIntersections(linesP, dst, resized),quad);

    double thrsh = (1/SCALE_RATIO)*(1-SCALE_RATIO);

    if(success)
    {
        cv::line(src, quad.p1*(1 + thrsh), quad.p2*(1 + thrsh), Scalar(0, 0, 255), 3);
        cv::line(src, quad.p2*(1 + thrsh), quad.p3*(1 + thrsh), Scalar(0, 0, 255), 3);
        cv::line(src, quad.p3*(1 + thrsh), quad.p4*(1 + thrsh), Scalar(0, 0, 255), 3);
        cv::line(src, quad.p4*(1 + thrsh), quad.p1*(1 + thrsh), Scalar(0,0,255), 3);
    }

    if(success && extract)
    {


        double distX = distanceBtwPoints(quad.p2*(1 + thrsh), quad.p3*(1 + thrsh));
        double distY = distanceBtwPoints(quad.p1*(1 + thrsh), quad.p2*(1 + thrsh));

        Point2f dest_pts[4];
        dest_pts[0] = Point2f(0, 0);
        dest_pts[1] = Point2f(distX, 0);
        dest_pts[2] = Point2f(distX,distY);
        dest_pts[3] = Point2f(0, distY);

        Point2f src_pts[4];
        src_pts[0] = quad.p2*(1 + thrsh);
        src_pts[1] = quad.p3*(1 + thrsh);
        src_pts[2] = quad.p4*(1 + thrsh);
        src_pts[3] = quad.p1*(1 + thrsh);


        Mat transformMat;
        Mat warped = OpenWarpPerspective(src.clone(), src_pts, dest_pts, transformMat);
//        Mat M = getRotationMatrix2D(Point(warped.rows/2, warped.cols/2) ,270, 0.5);
//        Mat rotated;
//        warpAffine(warped, rotated, M, warped.size(), INTER_CUBIC);
//        resize(rotated,extracted,Size(rotated.cols*2,rotated.rows*2));
        myrotate(warped,extracted,270);

        multiply(extracted,1.2,extracted);
        Mat kernel(1,1,CV_8UC3);
        erode(extracted,extracted,kernel);

        return true;
    }
//    Mat M = getRotationMatrix2D(Point(src.rows/2, src.cols/2) ,0, 1.0);
//    warpAffine(src, src, M, src.size(), INTER_CUBIC);
    //src = resClone;
    resized.release();
    gray.release();
    detected_edges.release();
    dst.release();
    thresh_img.release();
   // lines.clear();
//    imshow("kekke", resClone);
//    waitKey(0);
    return false;
}

void testFunc(Mat &src)
{
    line(src,Point(100,100),Point(300,300),Scalar(0,0,255));
}

