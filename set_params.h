#pragma once

#include "class.h"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

const u_int count = 1;
const u_int stepsize = 10;
const double robot[2] = { 15, 75 };
const double goal[2] = { 110, 675 };
const double waypoint1[2] = { 670, 80 };
const double waypoint2[2] = { 670, 610 };
const double START[3][2] = { {10, 65}, {640, 85}, {100, 600} };
const double END[3][2] = { {690, 85}, {690, 610}, {690, 690} };
const double PI = 3.14159;

node v1, v2, v3;
path pos1, pos2, pos3;

cv::Mat img = cv::imread("obstacle.jpg", cv::IMREAD_REDUCED_COLOR_2);
cv::Mat gray_img;
