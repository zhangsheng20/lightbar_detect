#ifndef DETECT_H
#define DETECT_H

#include <iostream>

#include <stdio.h>
#include "stdlib.h"


#include "opencv2/core/core.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"



using namespace std;
using namespace cv;

double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm);
double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm);
void DrawEnclosingRexts(Mat &grayImage, Mat &dstImage);
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y); //通过像素坐标就算云台需要转过的角度
float GetPixelLength(Point PixelPointO, Point PixelPointA);
Point2f PitchYawError2PixelError(float YawAngleDegree, float PitchAngleDegree);

#endif
