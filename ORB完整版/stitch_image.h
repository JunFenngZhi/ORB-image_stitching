#pragma once
#include<iostream>
#include<stdio.h>
#include<opencv2/opencv.hpp>
#include <math.h>
#include<time.h>

using namespace cv;
using namespace std;


Mat Stitch(const Mat& H, const Mat& Pic1, const Mat& Base_Pic);
