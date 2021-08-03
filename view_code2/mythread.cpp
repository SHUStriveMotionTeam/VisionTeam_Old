#include "mythread.h"
#include<qdebug.h>
#include <iostream>
#include<core/core.hpp>  
#include<highgui/highgui.hpp>  
#include "opencv2/imgproc/imgproc.hpp"
#include <windows.h>
#include <winnt.h>
#include <Kinect.h>
#include "cv.h"   
#include "highgui.h"   
#include <stdio.h>   
#include <math.h>  
#include <string.h>   
#include "opencv2/video/background_segm.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<qdebug.h>
#include<fstream>
#include<string>
#include<QtSerialPort\qserialport.h>
#include<QtSerialPort\qserialportinfo.h>
#pragma comment( lib, "Kinect20.Lib" )

using namespace cv;
using namespace std;
HANDLE hCom;
mythread::mythread(/*QObject *parent*/)
	: QObject(/*parent*/)
{
}

mythread::~mythread()
{
}


////////////////////////////////////// ȫ�ֱ������弰��ֵ//////////////////////////////////////////////

//kinectģʽȫ�ֱ���
int point = 0;
int juli = 0;
Mat g_c8BitDepth;
vector<Mat> hsvSplit;
Mat hsv_image;
//vector<vector<Point> > contour;
//RotatedRect s;
int depth_width = 512; //depthͼ���С
int depth_height = 424;
int color_widht = 1920; //colorͼ���С
int color_height = 1080;


//����ͷģʽȫ�ֱ���
int  sign = 0;
int S = 0;
IplImage* img = 0;
IplImage* img0 = 0;
CvMemStorage* storage = 0;
CvPoint pt[4];
const char* wndname = "Square Detection Demo";
Mat bgr_frame;
cv::VideoCapture capture;
////////////////////////////////////// ȫ�ֱ������弰��ֵ//////////////////////////////////////////////

/*//////////////��������������/*/
int yuzhi = 2267;//3200
int camera = 1;

/////////////////////////////���ݸ�����///////////////////////////////////

int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
int white_h_max_t = 197, white_h_min_t = 34, white_s_max_t = 157, white_s_min_t = 0, white_v_max_t = 198, white_v_min_t = 89;
int blue_h_max_t = 116, blue_h_min_t = 16, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
int yellow_h_max_t = 71, yellow_h_min_t = 20, yellow_s_max_t = 255, yellow_s_min_t = 82, yellow_v_max_t = 255, yellow_v_min_t = 104;
int blove_h_max_t = 122, blove_h_min_t = 30, blove_s_max_t = 166, blove_s_min_t = 10, blove_v_max_t = 114, blove_v_min_t = 19;
int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
int gray_h_max_t = 69, gray_h_min_t = 46, gray_s_max_t = 132, gray_s_min_t = 32, gray_v_max_t = 129, gray_v_min_t = 7;
int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
int rect_h_max_t = 255, rect_h_min_t = 0, rect_s_max_t = 240, rect_s_min_t = 0, rect_v_max_t = 129, rect_v_min_t = 122, sign_S_t = 20000, gray_rate_t = 0, exposure_t = 6;
int whrect_h_max_t = 255, whrect_h_min_t = 0, whrect_s_max_t = 154, whrect_s_min_t = 0, whrect_v_max_t = 255, whrect_v_min_t = 76;


/*�������ĵ������
int red1_h_max_t = 10 ,red1_h_min_t = 2 ,red1_s_max_t = 220 ,red1_s_min_t = 14 ,red1_v_max_t = 255 ,red1_v_min_t = 41;
int red2_h_max_t = 188 ,red2_h_min_t = 37 ,red2_s_max_t = 255 ,red2_s_min_t = 34 ,red2_v_max_t = 255 ,red2_v_min_t = 51;
int white_h_max_t = 17 ,white_h_min_t = 0 ,white_s_max_t = 111 ,white_s_min_t = 0 ,white_v_max_t = 222 ,white_v_min_t = 0;
int yellow_h_max_t = 201 ,yellow_h_min_t = 24 ,yellow_s_max_t = 253 ,yellow_s_min_t = 84 ,yellow_v_max_t = 255 ,yellow_v_min_t = 10;
int blove_h_max_t = 159 ,blove_h_min_t = 31 ,blove_s_max_t = 223 ,blove_s_min_t = 85 ,blove_v_max_t = 172 ,blove_v_min_t = 48;

*/



////////////////////////////////////////��Ҫ��//////////////////////////////////////////
int blue_h_max = blue_h_max_t, blue_h_min = blue_h_min_t, blue_s_max = blue_s_max_t, blue_s_min = blue_s_min_t, blue_v_max = blue_v_max_t, blue_v_min = blue_v_min_t;
int blove_h_max = blove_h_max_t, blove_h_min = blove_h_min_t, blove_s_max = blove_s_max_t, blove_s_min = blove_s_min_t, blove_v_max = blove_v_max_t, blove_v_min = blove_v_min_t;
int gray_h_max = gray_h_max_t, gray_h_min = gray_h_min_t, gray_s_max = gray_s_max_t, gray_s_min = gray_s_min_t, gray_v_max = gray_v_max_t, gray_v_min = gray_v_min_t;
int red1_h_max = red1_h_max_t, red1_h_min = red1_h_min_t, red1_s_max = red1_s_max_t, red1_s_min = red1_s_min_t, red1_v_max = red1_v_max_t, red1_v_min = red1_v_min_t;
int red2_h_max = red2_h_max_t, red2_h_min = red2_h_min_t, red2_s_max = red2_s_max_t, red2_s_min = red2_s_min_t, red2_v_max = red2_v_max_t, red2_v_min = red2_v_min_t;
int orange_h_max = orange_h_max_t, orange_h_min = orange_h_min_t, orange_s_max = orange_s_max_t, orange_s_min = orange_s_min_t, orange_v_max = orange_v_max_t, orange_v_min = orange_v_min_t;
int yellow_h_max = yellow_h_max_t, yellow_h_min = yellow_h_min_t, yellow_s_max = yellow_s_max_t, yellow_s_min = yellow_s_min_t, yellow_v_max = yellow_v_max_t, yellow_v_min = yellow_v_min_t;
int lemon_h_max = lemon_h_max_t, lemon_h_min = lemon_h_min_t, lemon_s_max = lemon_s_max_t, lemon_s_min = lemon_s_min_t, lemon_v_max = lemon_v_max_t, lemon_v_min = lemon_v_min_t;
int white_h_max = white_h_max_t, white_h_min = white_h_min_t, white_s_max = white_s_max_t, white_s_min = white_s_min_t, white_v_max = white_v_max_t, white_v_min = white_v_min_t;
int rect_h_max = rect_h_max_t, rect_h_min = rect_h_min_t, rect_s_max = rect_s_max_t, rect_s_min = rect_s_min_t, rect_v_max = rect_v_max_t, rect_v_min = rect_v_min_t, gray_rate = gray_rate_t, exposure = exposure_t, sign_S= sign_S_t;
int whrect_h_max = whrect_h_max_t, whrect_h_min = whrect_h_min_t, whrect_s_max = whrect_s_max_t, whrect_s_min = whrect_s_min_t, whrect_v_max = whrect_v_max_t, whrect_v_min = whrect_v_min_t;

/////////��ɫ��λ������ʵ��///////////

void mythread::blue_reset() /////////��ɫ��λ������ʵ��//////////
{
	blue_h_max = blue_h_max_t, blue_h_min = blue_h_min_t, blue_s_max = blue_s_max_t, blue_s_min = blue_s_min_t, blue_v_max = blue_v_max_t, blue_v_min = blue_v_min_t;
}
void mythread::gray_reset() /////////��ɫ��λ������ʵ��//////////
{
	
	 gray_h_max = gray_h_max_t, gray_h_min = gray_h_min_t, gray_s_max = gray_s_max_t, gray_s_min = gray_s_min_t, gray_v_max = gray_v_max_t, gray_v_min = gray_v_min_t;
}
void mythread::red1_reset() /////////��1��λ������ʵ��//////////
{
	 red1_h_max = red1_h_max_t, red1_h_min = red1_h_min_t, red1_s_max = red1_s_max_t, red1_s_min = red1_s_min_t, red1_v_max = red1_v_max_t, red1_v_min = red1_v_min_t;
	
}
void mythread::red2_reset() /////////��2��λ������ʵ��//////////
{
	red2_h_max = red2_h_max_t, red2_h_min = red2_h_min_t, red2_s_max = red2_s_max_t, red2_s_min = red2_s_min_t, red2_v_max = red2_v_max_t, red2_v_min = red2_v_min_t;	
}
void mythread::yellow_reset() /////////��ɫ��λ������ʵ��//////////
{	
	 yellow_h_max = yellow_h_max_t, yellow_h_min = yellow_h_min_t, yellow_s_max = yellow_s_max_t, yellow_s_min = yellow_s_min_t, yellow_v_max = yellow_v_max_t, yellow_v_min = yellow_v_min_t;	
}
void mythread::lemon_reset() /////////��ɫ��λ������ʵ��//////////
{
	lemon_h_max = lemon_h_max_t, lemon_h_min = lemon_h_min_t, lemon_s_max = lemon_s_max_t, lemon_s_min = lemon_s_min_t, lemon_v_max = lemon_v_max_t, lemon_v_min = lemon_v_min_t;
}
void mythread::orange_reset() /////////��ɫ��λ������ʵ��//////////
{
	 orange_h_max = orange_h_max_t, orange_h_min = orange_h_min_t, orange_s_max = orange_s_max_t, orange_s_min = orange_s_min_t, orange_v_max = orange_v_max_t, orange_v_min = orange_v_min_t;
}
void mythread::white_reset() /////////��ɫ��λ������ʵ��//////////
{
	 white_h_max = white_h_max_t, white_h_min = white_h_min_t, white_s_max = white_s_max_t, white_s_min = white_s_min_t, white_v_max = white_v_max_t, white_v_min = white_v_min_t;
	
}
void mythread::blove_reset() /////////��ɫ��λ������ʵ��//////////
{
	blove_h_max = blove_h_max_t, blove_h_min = blove_h_min_t, blove_s_max = blove_s_max_t, blove_s_min = blove_s_min_t, blove_v_max = blove_v_max_t, blove_v_min = blove_v_min_t;

}
void mythread::rect_white_reset()
{
	
	 whrect_h_max = whrect_h_max_t, whrect_h_min = whrect_h_min_t, whrect_s_max = whrect_s_max_t, whrect_s_min = whrect_s_min_t, whrect_v_max = whrect_v_max_t, whrect_v_min = whrect_v_min_t;
}
void mythread::rect_gray_reset()
{
	 rect_h_max = rect_h_max_t, rect_h_min = rect_h_min_t, rect_s_max = rect_s_max_t, rect_s_min = rect_s_min_t, rect_v_max = rect_v_max_t, rect_v_min = rect_v_min_t;
}
//Mogen
/*
int blue_h_max_t = 116, blue_h_min_t = 16, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
int blove_h_max_t = 116, blove_h_min_t = 16, blove_s_max_t = 255, blove_s_min_t = 112, blove_v_max_t = 255, blove_v_min_t = 20;
int gray_h_max_t = 69, gray_h_min_t = 46, gray_s_max_t = 132, gray_s_min_t = 32, gray_v_max_t = 129, gray_v_min_t = 7;
int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
int yellow_h_max_t = 71, yellow_h_min_t = 0, yellow_s_max_t = 255, yellow_s_min_t = 82, yellow_v_max_t = 255, yellow_v_min_t = 124;
int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
int white_h_max_t = 144, white_h_min_t = 36, white_s_max_t = 38, white_s_min_t = 3, white_v_max_t = 246, white_v_min_t = 32;
int rect_h_max_t = 180, rect_h_min_t = 0, rect_s_max_t = 70, rect_s_min_t = 0, rect_v_max_t = 220, rect_v_min_t = 46;
int whrect_h_max_t = 178, whrect_h_min_t = 0, whrect_s_max_t = 100, whrect_s_min_t = 0, whrect_v_max_t = 255, whrect_v_min_t = 88;
*/
//Tag
/*v
int blue_h_max_t = 116, blue_h_min_t = 16, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
int blove_h_max_t = 116, blove_h_min_t = 16, blove_s_max_t = 255, blove_s_min_t = 112, blove_v_max_t = 255, blove_v_min_t = 20;
int gray_h_max_t = 69, gray_h_min_t = 46, gray_s_max_t = 132, gray_s_min_t = 32, gray_v_max_t = 129, gray_v_min_t = 7;
int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
int yellow_h_max_t = 71, yellow_h_min_t = 0, yellow_s_max_t = 255, yellow_s_min_t = 82, yellow_v_max_t = 255, yellow_v_min_t = 124;
int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
int white_h_max_t = 130, white_h_min_t = 50, white_s_max_t = 69, white_s_min_t = 8, white_v_max_t = 255, white_v_min_t = 0;
int rect_h_max_t = 180, rect_h_min_t = 0, rect_s_max_t = 70, rect_s_min_t = 0, rect_v_max_t = 220, rect_v_min_t = 46, sign_S_t = 20000,gray_rate_t = 0, exposure_t = 7;
int whrect_h_max_t = 85, whrect_h_min_t = 0, whrect_s_max_t = 127, whrect_s_min_t = 62, whrect_v_max_t = 130, whrect_v_min_t = 73;
*/
//Nicht
/*
int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
int white_h_max_t = 197, white_h_min_t = 34, white_s_max_t = 157, white_s_min_t = 0, white_v_max_t = 198, white_v_min_t = 89;
int blue_h_max_t = 116, blue_h_min_t = 16, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
int yellow_h_max_t = 71, yellow_h_min_t = 20, yellow_s_max_t = 255, yellow_s_min_t = 82, yellow_v_max_t = 255, yellow_v_min_t = 104;
int blove_h_max_t = 122, blove_h_min_t = 30, blove_s_max_t = 115, blove_s_min_t = 10, blove_v_max_t = 114, blove_v_min_t = 19;
int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
int gray_h_max_t = 69, gray_h_min_t = 46, gray_s_max_t = 132, gray_s_min_t = 32, gray_v_max_t = 129, gray_v_min_t = 7;
int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
int rect_h_max_t = 180, rect_h_min_t = 0, rect_s_max_t = 70, rect_s_min_t = 0, rect_v_max_t = 220, rect_v_min_t = 46;
int whrect_h_max_t = 178, whrect_h_min_t = 0, whrect_s_max_t = 100, whrect_s_min_t = 0, whrect_v_max_t = 255, whrect_v_min_t = 88;
*/
//Bedeckt
/*
int blue_h_max_t = 116, blue_h_min_t = 36, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
int blove_h_max_t = 124, blove_h_min_t = 61, blove_s_max_t = 179, blove_s_min_t = 71, blove_v_max_t = 255, blove_v_min_t = 20;
int gray_h_max_t = 180, gray_h_min_t = 32, gray_s_max_t = 75, gray_s_min_t = 16, gray_v_max_t = 222, gray_v_min_t = 57;
int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
int yellow_h_max_t = 81, yellow_h_min_t = 17, yellow_s_max_t = 255, yellow_s_min_t = 111, yellow_v_max_t = 255, yellow_v_min_t = 146;
int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
int white_h_max_t = 255, white_h_min_t = 30, white_s_max_t = 63, white_s_min_t = 0, white_v_max_t = 255, white_v_min_t = 130;
int rect_h_max_t = 180, rect_h_min_t = 0, rect_s_max_t = 70, rect_s_min_t = 0, rect_v_max_t = 220, rect_v_min_t = 46;
int whrect_h_max_t = 178, whrect_h_min_t = 0, whrect_s_max_t = 100, whrect_s_min_t = 0, whrect_v_max_t = 255, whrect_v_min_t = 88;
*/

//��������
/*
int blue_h_max_t = 116, blue_h_min_t = 16, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
int blove_h_max_t = 116, blove_h_min_t = 16, blove_s_max_t = 255, blove_s_min_t = 112, blove_v_max_t = 255, blove_v_min_t = 20;
int gray_h_max_t = 69, gray_h_min_t = 46, gray_s_max_t = 132, gray_s_min_t = 32, gray_v_max_t = 129, gray_v_min_t = 7;
int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
int yellow_h_max_t = 40, yellow_h_min_t = 10, yellow_s_max_t = 196, yellow_s_min_t = 98, yellow_v_max_t = 173, yellow_v_min_t = 72;
int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
int white_h_max_t = 130, white_h_min_t = 44, white_s_max_t = 69, white_s_min_t = 8, white_v_max_t = 255, white_v_min_t = 0;
int rect_h_max_t = 180, rect_h_min_t = 0, rect_s_max_t = 70, rect_s_min_t = 0, rect_v_max_t = 220, rect_v_min_t = 46, sign_S_t = 20000,gray_rate_t = 0, exposure_t = 7;
int whrect_h_max_t = 85, whrect_h_min_t = 0, whrect_s_max_t = 127, whrect_s_min_t = 62, whrect_v_max_t = 130, whrect_v_min_t = 73;
*/


////���ϲ���
//int blue_h_max_t = 116, blue_h_min_t = 16, blue_s_max_t = 255, blue_s_min_t = 112, blue_v_max_t = 255, blue_v_min_t = 20;
//int blove_h_max_t = 116, blove_h_min_t = 16, blove_s_max_t = 255, blove_s_min_t = 112, blove_v_max_t = 255, blove_v_min_t = 20;
//int gray_h_max_t = 69, gray_h_min_t = 46, gray_s_max_t = 132, gray_s_min_t = 32, gray_v_max_t = 129, gray_v_min_t = 7;
//int red1_h_max_t = 10, red1_h_min_t = 0, red1_s_max_t = 255, red1_s_min_t = 43, red1_v_max_t = 255, red1_v_min_t = 46;
//int red2_h_max_t = 180, red2_h_min_t = 156, red2_s_max_t = 255, red2_s_min_t = 43, red2_v_max_t = 255, red2_v_min_t = 46;
//int orange_h_max_t = 19, orange_h_min_t = 5, orange_s_max_t = 255, orange_s_min_t = 128, orange_v_max_t = 255, orange_v_min_t = 16;
//int yellow_h_max_t = 71, yellow_h_min_t = 0, yellow_s_max_t = 255, yellow_s_min_t = 82, yellow_v_max_t = 255, yellow_v_min_t = 124;
//int lemon_h_max_t = 129, lemon_h_min_t = 73, lemon_s_max_t = 145, lemon_s_min_t = 19, lemon_v_max_t = 163, lemon_v_min_t = 79;
//int white_h_max_t = 144, white_h_min_t = 36, white_s_max_t = 38, white_s_min_t = 3, white_v_max_t = 246, white_v_min_t = 32;
//int rect_h_max_t = 180, rect_h_min_t = 0, rect_s_max_t = 70, rect_s_min_t = 0, rect_v_max_t = 220, rect_v_min_t = 46;
//int whrect_h_max_t = 178, whrect_h_min_t = 0, whrect_s_max_t = 100, whrect_s_min_t = 0, whrect_v_max_t = 255, whrect_v_min_t = 88;
Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_8UC1);
double mythread::bluede(Mat asrc) {
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_blue = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if ((curr_r_hue[j] <= blue_h_max && curr_r_hue[j] >= blue_h_min) && (curr_r_satur[j]>blue_s_min && curr_r_satur[j]<blue_s_max) && (curr_r_value[j]>blue_v_min && curr_r_value[j]<blue_v_max))
			{
				count_blue++; count++;
			}
			else  count++;
		}
	}
	double blue;
	blue = 1.0*count_blue / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "blue   " << blue << endl;
#endif    
	return blue;
}
double mythread::grayde(Mat asrc) {
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_gray = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if ((curr_r_hue[j] <= gray_h_max && curr_r_hue[j] >= gray_h_min) && (curr_r_satur[j]>gray_s_min && curr_r_satur[j]<gray_s_max) && (curr_r_value[j]>gray_v_min && curr_r_value[j]<gray_v_max))
			{
				count_gray++; count++;
			}
			else  count++;
		}
	}
	double gray;
	gray = 1.0*count_gray / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "gray" << gray << endl;
#endif    
	return gray;
}
double mythread::orangede(Mat asrc) {
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_orange = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if (((curr_r_hue[j] <= orange_h_max && curr_r_hue[j] >= orange_h_min) && (curr_r_satur[j]>orange_s_min && curr_r_satur[j]<orange_s_max) && (curr_r_value[j]>orange_v_min && curr_r_value[j]<orange_v_max)))
			{
				count_orange++; count++;
			}
			else  count++;
		}
	}
	double orange;
	orange = 1.0*count_orange / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "orange   " << orange << endl;
#endif    

	return orange;
}
double mythread::yellowde(Mat asrc) {
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_yellow = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if (((curr_r_hue[j] <= yellow_h_max && curr_r_hue[j] >= yellow_h_min) && (curr_r_satur[j]>yellow_s_min && curr_r_satur[j]<yellow_s_max) && (curr_r_value[j]>yellow_v_min && curr_r_value[j]<yellow_v_max)))
			{
				count_yellow++; count++;
			}
			else  count++;
		}
	}
	double yellow;
	yellow = 1.0*count_yellow / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "yellow   " << yellow << endl;
#endif    

	return yellow;
}
double mythread::lemonde(Mat asrc) {
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_lemon = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if (((curr_r_hue[j] <= lemon_h_max && curr_r_hue[j] >= lemon_h_min) && (curr_r_satur[j]>lemon_s_min && curr_r_satur[j]<lemon_s_max) && (curr_r_value[j]>lemon_v_min && curr_r_value[j]<lemon_v_max)))
			{
				count_lemon++; count++;
			}
			else  count++;
		}
	}
	double lemon;
	lemon = 1.0*count_lemon / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "lemon   " << lemon << endl;
#endif    

	return lemon;
}
double mythread::redde(Mat asrc) {
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_red1 = 0;
	int count_red2 = 0;

	//Mat mask(cv::Size(num_col, num_row), CV_8U);
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if ((curr_r_hue[j] <= red1_h_max && curr_r_hue[j] >= red1_h_min) && (curr_r_satur[j]>red1_s_min && curr_r_satur[j]<red1_s_max) && (curr_r_value[j]>red1_v_min && curr_r_value[j]<red1_v_max))//����ɫ
			{
				count_red1++; count++;
			}
			else if ((curr_r_hue[j] <= red2_h_max && curr_r_hue[j] >= red2_h_min) && (curr_r_satur[j]>red2_s_min && curr_r_satur[j]<red2_s_max) && (curr_r_value[j]>red2_v_min && curr_r_value[j]<red2_v_max))
			{
				count_red2++; count++;
			}
			else  count++;
		}
	}

	double red1, red2;
	red1 = 1.0*count_red1 / count;
	red2 = 1.0*count_red2 / count;
#ifdef _debug
	cout << "red   " << red1 + red2 << endl;
#endif    

	return (red1 + red2);

}
double mythread::blovede(Mat asrc) {
	double t_b = (double)cvGetTickCount();
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_blove = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if ((curr_r_hue[j] <= blove_h_max && curr_r_hue[j] >= blove_h_min) && (curr_r_satur[j]>blove_s_min && curr_r_satur[j]<blove_s_max) && (curr_r_value[j]>blove_v_min && curr_r_value[j]<blove_v_max))
			{
				count_blove++; count++;
			}
			else  count++;
		}
	}
	double blove;
	blove = 1.0*count_blove / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "blove   " << blove << endl;
#endif
	t_b = ((double)cvGetTickCount() - t_b) / (cvGetTickFrequency() * 1000);
	cv::waitKey(10);
	cout << "����ʱ��blove: " << t_b << "ms" << endl;
	return blove;
}
double mythread::whitede(Mat asrc) {
	double t_b = (double)cvGetTickCount();
	Mat image = asrc;
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);
	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	int count = 0;
	int count_white = 0;
	for (int i = 0; i < num_row; i += 2)
	{
		const cv::Vec3b* curr_r_image = image.ptr<const cv::Vec3b>(i);
		const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
		const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
		const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
		const uchar*inData = image.ptr<uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 2) {
			if ((curr_r_hue[j] <= white_h_max && curr_r_hue[j] >= white_h_min) && (curr_r_satur[j]>white_s_min && curr_r_satur[j]<white_s_max) && (curr_r_value[j]>white_v_min && curr_r_value[j]<white_v_max))
			{
				count_white++; count++;
			}
			else  count++;
		}
	}
	double white;
	white = 1.0*count_white / count;

#ifdef _debug
	cout << "count" << count << endl;
	cout << "white   " << white << endl;
#endif
	t_b = ((double)cvGetTickCount() - t_b) / (cvGetTickFrequency() * 1000);
	cv::waitKey(10);
	cout << "����ʱ��white: " << t_b << "ms" << endl;
	return white;
}
//��һ�����Ǻ��������  Ӧ�øĳ�ʶ���ư� ��Ҫ���б���ɫ
Mat mythread::findball_1(Mat image, Mat irang)
{
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	for (int i = 0; i < num_row; i += 1)
	{
		uchar* data = irang.ptr<uchar>(i);
		uchar* curr_r_hue = channels[0].ptr< uchar>(i);
		uchar* curr_r_satur = channels[1].ptr< uchar>(i);
		uchar* curr_r_value = channels[2].ptr< uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 1) {
			if (((curr_r_hue[j] <= yellow_h_max && curr_r_hue[j] >= yellow_h_min) && (curr_r_satur[j]>yellow_s_min && curr_r_satur[j]<yellow_s_max) && (curr_r_value[j]>yellow_v_min && curr_r_value[j]<yellow_v_max)) || ((curr_r_hue[j] <= red1_h_max && curr_r_hue[j] >= red1_h_min) && (curr_r_satur[j]>red1_s_min && curr_r_satur[j]<red1_s_max) && (curr_r_value[j]>red1_v_min && curr_r_value[j]<red1_v_max)) || ((curr_r_hue[j] <= white_h_max && curr_r_hue[j] >= white_h_min) && (curr_r_satur[j]>white_s_min && curr_r_satur[j]<white_s_max) && (curr_r_value[j]>white_v_min && curr_r_value[j] < white_v_max)) || ((curr_r_hue[j] <= red2_h_max && curr_r_hue[j] >= red2_h_min) && (curr_r_satur[j]>red2_s_min && curr_r_satur[j]<red2_s_max) && (curr_r_value[j]>red2_v_min && curr_r_value[j] < red2_v_max)))
			{
				data[j] = 255;
			}
			else
			{
				data[j] = 0;
			}
		}
	}
	return irang;
}
//�ڶ������ǻư�������
Mat mythread::findball_2(Mat image, Mat irang)
{
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	for (int i = 0; i < num_row; i += 1)
	{
		uchar* data = irang.ptr<uchar>(i);
		uchar* curr_r_hue = channels[0].ptr< uchar>(i);
		uchar* curr_r_satur = channels[1].ptr< uchar>(i);
		uchar* curr_r_value = channels[2].ptr< uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 1) {
			if (((curr_r_hue[j] <= blove_h_max && curr_r_hue[j] >= blove_h_min) && (curr_r_satur[j]>blove_s_min && curr_r_satur[j]<blove_s_max) && (curr_r_value[j]>blove_v_min && curr_r_value[j]<blove_v_max)) || ((curr_r_hue[j] <= yellow_h_max && curr_r_hue[j] >= yellow_h_min) && (curr_r_satur[j]>yellow_s_min && curr_r_satur[j]<yellow_s_max) && (curr_r_value[j]>yellow_v_min && curr_r_value[j]<yellow_v_max)) || ((curr_r_hue[j] <= white_h_max && curr_r_hue[j] >= white_h_min) && (curr_r_satur[j]>white_s_min && curr_r_satur[j]<white_s_max) && (curr_r_value[j]>white_v_min && curr_r_value[j] < white_v_max)))
			{
				data[j] = 255;
			}
			else
			{
				data[j] = 0;
			}
		}
	}
	return irang;
}
//��������������ɫ����
Mat mythread::findball_3(Mat image, Mat irang)
{
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	for (int i = 0; i < num_row; i += 1)
	{
		uchar* data = irang.ptr<uchar>(i);
		uchar* curr_r_hue = channels[0].ptr< uchar>(i);
		uchar* curr_r_satur = channels[1].ptr< uchar>(i);
		uchar* curr_r_value = channels[2].ptr< uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 1) {
			if (((curr_r_hue[j] <= blue_h_max && curr_r_hue[j] >= blue_h_min) && (curr_r_satur[j]>blue_s_min && curr_r_satur[j]<blue_s_max) && (curr_r_value[j]>blue_v_min && curr_r_value[j]<blue_v_max)) || ((curr_r_hue[j] <= gray_h_max && curr_r_hue[j] >= gray_h_min) && (curr_r_satur[j]>gray_s_min && curr_r_satur[j]<gray_s_max) && (curr_r_value[j]>gray_v_min && curr_r_value[j]<gray_v_max)) || ((curr_r_hue[j] <= lemon_h_max && curr_r_hue[j] >= lemon_h_min) && (curr_r_satur[j]>lemon_s_min && curr_r_satur[j]<lemon_s_max) && (curr_r_value[j]>lemon_v_min && curr_r_value[j]<lemon_v_max)))
			{
				data[j] = 255;
			}
			else
			{
				data[j] = 0;
			}
		}
	}
	return irang;
}
//���ĸ����Ǻ�ɫ����
Mat mythread::findball_4(Mat image, Mat irang)
{
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	for (int i = 0; i < num_row; i += 1)
	{
		uchar* data = irang.ptr<uchar>(i);
		uchar* curr_r_hue = channels[0].ptr< uchar>(i);
		uchar* curr_r_satur = channels[1].ptr< uchar>(i);
		uchar* curr_r_value = channels[2].ptr< uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 1) {
			if ((curr_r_hue[j] <= orange_h_max && curr_r_hue[j] >= orange_h_min) && (curr_r_satur[j]>orange_s_min && curr_r_satur[j]<orange_s_max) && (curr_r_value[j]>orange_v_min && curr_r_value[j]<orange_v_max))
			{
				data[j] = 255;
			}
			else
			{
				data[j] = 0;
			}
		}
	}
	return irang;
}
//�����������������
Mat mythread::findball_5(Mat image, Mat irang)
{
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	for (int i = 0; i < num_row; i += 1)
	{
		uchar* data = irang.ptr<uchar>(i);
		uchar* curr_r_hue = channels[0].ptr< uchar>(i);
		uchar* curr_r_satur = channels[1].ptr< uchar>(i);
		uchar* curr_r_value = channels[2].ptr< uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 1) {
			if ((curr_r_hue[j] <= orange_h_max && curr_r_hue[j] >= orange_h_min) && (curr_r_satur[j]>orange_s_min && curr_r_satur[j]<orange_s_max) && (curr_r_value[j]>orange_v_min && curr_r_value[j]<orange_v_max))
			{
				data[j] = 255;
			}
			else if (((curr_r_hue[j] <= blue_h_max && curr_r_hue[j] >= blue_h_min) && (curr_r_satur[j] > blue_s_min && curr_r_satur[j] < blue_s_max) && (curr_r_value[j] > blue_v_min && curr_r_value[j] < blue_v_max)) || ((curr_r_hue[j] <= gray_h_max && curr_r_hue[j] >= gray_h_min) && (curr_r_satur[j] > gray_s_min && curr_r_satur[j] < gray_s_max) && (curr_r_value[j] > gray_v_min && curr_r_value[j] < gray_v_max)) || ((curr_r_hue[j] <= lemon_h_max && curr_r_hue[j] >= lemon_h_min) && (curr_r_satur[j] > lemon_s_min && curr_r_satur[j] < lemon_s_max) && (curr_r_value[j] > lemon_v_min && curr_r_value[j] < lemon_v_max)))
			{
				data[j] = 255;
			}
			else
			{
				data[j] = 0;
			}
		}
	}
	return irang;
}
//������������������
Mat mythread::findball_6(Mat image, Mat irang)
{
	cv::Mat hsv_image;        //תHSV
	hsv_image.create(image.size(), image.type());
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> channels;
	cv::split(hsv_image, channels);
	int num_row = image.rows;
	int num_col = image.cols;
	for (int i = 0; i < num_row; i += 1)
	{
		uchar* data = irang.ptr<uchar>(i);
		uchar* curr_r_hue = channels[0].ptr< uchar>(i);
		uchar* curr_r_satur = channels[1].ptr< uchar>(i);
		uchar* curr_r_value = channels[2].ptr< uchar>(i);
		//	uchar*outData = mask.ptr<uchar>(i);
		for (int j = 0; j < num_col; j += 1) {
			if (((curr_r_hue[j] <= blue_h_max && curr_r_hue[j] >= blue_h_min) && (curr_r_satur[j]>blue_s_min && curr_r_satur[j]<blue_s_max) && (curr_r_value[j]>blue_v_min && curr_r_value[j]<blue_v_max)) || ((curr_r_hue[j] <= red1_h_max && curr_r_hue[j] >= red1_h_min) && (curr_r_satur[j]>red1_s_min && curr_r_satur[j]<red1_s_max) && (curr_r_value[j]>red1_v_min && curr_r_value[j]<red1_v_max)) || ((curr_r_hue[j] <= white_h_max && curr_r_hue[j] >= white_h_min) && (curr_r_satur[j]>white_s_min && curr_r_satur[j]<white_s_max) && (curr_r_value[j]>white_v_min && curr_r_value[j] < white_v_max)) || ((curr_r_hue[j] <= red2_h_max && curr_r_hue[j] >= red2_h_min) && (curr_r_satur[j]>red2_s_min && curr_r_satur[j]<red2_s_max) && (curr_r_value[j]>red2_v_min && curr_r_value[j] < red2_v_max)))
			{
				data[j] = 255;
			}
			else if (((curr_r_hue[j] <= blove_h_max && curr_r_hue[j] >= blove_h_min) && (curr_r_satur[j]>blove_s_min && curr_r_satur[j]<blove_s_max) && (curr_r_value[j]>blove_v_min && curr_r_value[j]<blove_v_max)) || ((curr_r_hue[j] <= yellow_h_max && curr_r_hue[j] >= yellow_h_min) && (curr_r_satur[j]>yellow_s_min && curr_r_satur[j]<yellow_s_max) && (curr_r_value[j]>yellow_v_min && curr_r_value[j]<yellow_v_max)) || ((curr_r_hue[j] <= white_h_max && curr_r_hue[j] >= white_h_min) && (curr_r_satur[j]>white_s_min && curr_r_satur[j]<white_s_max) && (curr_r_value[j]>white_v_min && curr_r_value[j] < white_v_max)))
			{
				data[j] = 255;
			}
			else
			{
				data[j] = 0;
			}
		}
	}
	return irang;
}
void mythread::picprocess(Mat asrc, Mat depth, int number)
{
	double t2 = (double)cvGetTickCount();//picʱ��
	double t3 = (double)cvGetTickCount();//����ɫʱ��
	int a = number;
	Mat image = asrc;
	Mat irang_ide(depth_height, depth_width, CV_8UC1);
	//Mat irang_ide(image.size(),);
	switch (a)
	{
	case 1:
		irang_ide = findball_1(image, irang_ide);
		break;
	case 2:
		irang_ide = findball_2(image, irang_ide);
		break;
	case 3:
		irang_ide = findball_3(image, irang_ide);
		break;
	case 4:
		irang_ide = findball_4(image, irang_ide);
		break;
	case 5:
		irang_ide = findball_5(image, irang_ide);
		break;
	case 6:
		irang_ide = findball_6(image, irang_ide);
		break;
	case 7:
		irang_ide = findball_5(image, irang_ide);
		break;
	case 8:
		irang_ide = findball_6(image, irang_ide);
	default:
		break;
	}
	cv::imshow("ͨ����ɫ�ҵ���", irang_ide);
	t3 = ((double)cvGetTickCount() - t3) / (cvGetTickFrequency() * 1000);
	cout << "����ɫ����ʱ��: " << t3 << "ms" << endl;
#ifdef _debug
	imshow("3", threshold_output);
#endif
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	// �ҳ�����  opcv�еĶ���ΰ�Χ����
	findContours(irang_ide, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));//CV_CHAIN_APPROX_NONE   CV_CHAIN_APPROX_SIMPLE
	
	Mat imgROI;
	// ����αƽ����� + ��ȡ���κ�Բ�α߽��
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f>center(contours.size());
	vector<float>radius(contours.size());
	vector<Vec3f> circles;
	HoughCircles(irang_ide, circles, HOUGH_GRADIENT, 1,
		50,  // change this value to detect circles with different distances to each other
		100, 30, 1, 70 // change the last two parameters
					   // (min_radius & max_radius) to detect larger circles
	);
	Point min_center = Point(0, 0);
	int min_r = 0;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i c = circles[i];
		Point center = Point(c[0], c[1]);
		int r = c[2];
		//circle(asrc, center, (int)r, Scalar(0, 0, 255), 2, 8, 0);
		if ((center.x - r > 0) && (center.x + r < asrc.cols) && (center.y - r > 0) && (center.y + r < asrc.rows)) {
			imgROI = asrc(Rect((center.x - r), (center.y - r), 2 * r, 2 * r));
			double red, blue, yellow, lemon, orange, gray, blove, white;
			if (a == 3)
			{
				blue = bluede(imgROI);
				gray = grayde(imgROI);
				lemon = lemonde(imgROI);
				emit ball2_signal(gray, blue, lemon);
				if ((blue > 0.05) && (gray > 0.05) && (lemon > 0)) {
					//circle(drawing, center[j], (int)radius[j], color2, 2, 8, 0);//����Բ																//	circle(srcImage, center[j], 3, Scalar(0, 255, 255), -1, 8, 0);
					if (abs(center.x - 256) < abs(min_center.x - 256)) {
						min_center = center;
						min_r = r;
					}
					
				}
			}
		}
	}
	if (min_center.x > 0 && min_r >0) {
		circle(asrc, min_center, (int)min_r, Scalar(0, 0, 255), 2, 8, 0);
	}
	double red, blue, yellow, lemon, orange, gray, blove, white;
	

	//һ��ѭ�����������в��֣����б���������ĵĲ���
	int j = -1; double index = 0, temp = 0;
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, false);//��ָ�����ȱƽ���������� 
																   //	boundRect[i] = boundingRect(Mat(contours[i]));
		minEnclosingCircle(contours_poly[i], center[i], radius[i]);//�Ը����� 2D�㼯��Ѱ����С����İ�ΧԲ�� 
	}
	// ���ƶ�������� + Բ�ο�
	Mat drawing = Mat::zeros(image.size(), CV_8UC3);
	vector<RotatedRect> roRect(contours.size());
	int number_j = 0;
#ifdef _debug
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
#endif
	for (int unsigned i = 0; i < contours.size(); i++)
	{
		roRect[i] = minAreaRect(Mat(contours[i]));
		//2.5.1 ��ת������RotatedRect����Point()����������Point2f* pts������ת���ε��ĸ��˵�洢��pts.
		Point2f pts[4];
		roRect[i].points(pts);
		//2.5.2 ��line������������ת���ε��ĸ��ǵ㻭������
		line(drawing, pts[0], pts[1], Scalar(0, 0, 255), 2, 8);
		line(drawing, pts[0], pts[3], Scalar(0, 0, 255), 2, 8);
		line(drawing, pts[2], pts[1], Scalar(0, 0, 255), 2, 8);
		line(drawing, pts[2], pts[3], Scalar(0, 0, 255), 2, 8);

		//�жϾ��γ����

#ifdef _debug
		Scalar color = Scalar(0, 255, 0);//������ɫ
		drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());//��������
		circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);//����Բ
		circle(srcImage, center[i], 3, Scalar(0, 255, 255), -1, 8, 0);
		//a = 3.14*radius[i] * radius[i];
#endif

		Scalar color3 = Scalar(0, 0, 255);//���ú�ɫ
		line(image, Point(0, upperpart_threshold), Point(512, upperpart_threshold), color3);//����������ָ���
		//ȷ��������Ļ������
		if (((center[i].x - radius[i]) > 0) && ((center[i].y - radius[i]) > upperpart_threshold) &&
			((center[i].x + radius[i]) <image.cols) && ((center[i].y + radius[i]) < image.rows) && (radius[i]>10))
		{

			if ((roRect[i].size.height>(roRect[i].size.width)*0.6) && (roRect[i].size.height < (roRect[i].size.width) * 1.4))
			{
				imgROI = asrc(Rect((center[i].x - radius[i]), (center[i].y - radius[i]), 2 * radius[i], 2 * radius[i]));
				double red, blue, yellow,lemon, orange, gray,blove,white;
				if (a == 3)
				{
					blue = bluede(imgROI);
					gray = grayde(imgROI);
					lemon = lemonde(imgROI);
					emit ball3_signal(gray, blue, lemon);
					if ((blue>0.05) && (gray > 0.05) && (lemon > 0)) {
						if (number_j++ > 0)
						{
							/*if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
								j = i;
							}*/
							if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}

				else if (a == 4) {
					orange = orangede(imgROI);
					emit ball4_signal(orange);
					if (orange>0.3) {
						if (number_j++ > 0)
						{
							/*if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
								j = i;
							}*/
							if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}
				else if (a == 1) {
					yellow = yellowde(imgROI);
					red = redde(imgROI);
					white = whitede(imgROI);
					emit ball1_singal(red, yellow,white);
					if ((red> 0.05) && (yellow > 0.1) && (red< 0.5) && (white>0.03)) {
						if (number_j++ > 0)
						{
							/*if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
								j = i;
							}*/
							if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}
				else if (a == 2) {
					yellow = yellowde(imgROI);
					blove = blovede(imgROI);
					white = whitede(imgROI);
					red = redde(imgROI);
					emit ball2_signal(yellow, blove, white);
					if ((yellow > 0.11) && (blove > 0.03) && (white > 0.05) && (red < 0.05)) {
						if (number_j++ > 0)
						{
							/*if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
								j = i;
							}*/
							if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}
						}
						else
						{
							j = i;
						}
						number_j++;


					}
				}
				//cout << "�ҵ�Բ" << endl;
				else if (a == 5) {
					blue = bluede(imgROI);
					gray = grayde(imgROI);
					lemon = lemonde(imgROI);
					orange = orangede(imgROI);
					emit ball3_signal(gray, blue, lemon);
					emit ball4_signal(orange);

					if (((blue>0.05) && (gray > 0.03) && (lemon > 0))|| (orange>0.3)) {
						if (number_j++ > 0)
						{
							if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
								if (radius[i] > radius[j]) {
									j = i;
								}
							}
							/*if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}*/
							
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}
				else if (a == 6) {
					red = redde(imgROI);
					blue= bluede(imgROI);
					yellow = yellowde(imgROI);
					blove = blovede(imgROI);
					white = whitede(imgROI);
					emit ball2_signal(yellow, blove, white);
					emit ball1_singal(red, yellow,white);

					if ((yellow > 0.11) && (blove > 0.03) && (white > 0.05) && (red < 0.05) || (red> 0.05) && (yellow > 0.01) && (red< 0.5) && (white>0.03)) {
						if (number_j++ > 0)
						{
							if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
								if (radius[i] > radius[j]) {
									j = i;
								}
							}
							/*if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}*/
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}
				else if (a == 7) {
					blue = bluede(imgROI);
					gray = grayde(imgROI);
					lemon = lemonde(imgROI);
					orange = orangede(imgROI);
					emit ball3_signal(gray, blue, lemon);
					emit ball4_signal(orange);

					if (((blue>0.05) && (gray > 0.03) && (lemon > 0)) || (orange>0.3)) {
						if (number_j++ > 0)
						{
							if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
							j = i;
							}
						/*	if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}*/
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}
				else if (a == 8) {
					red = redde(imgROI);
					yellow = yellowde(imgROI);
					blove = blovede(imgROI);
					//��ɫ�Ǵ���ģ��Ǻ�ư������������ɫ�ж���Ӧ�ø�Ϊyellow
					//blue = bluede(imgROI);
					white = whitede(imgROI);
					emit ball2_signal(yellow, blove, white);
					emit ball1_singal(red, yellow, white);

					if ((yellow > 0.11) && (blove > 0.03) && (white > 0.05) && (red < 0.05) || (red> 0.05) && (blue > 0.01) && (red< 0.5) && (white>0.03)) {
						if (number_j++ > 0)
						{
							if (depthImg_show.at<uchar>(center[i].y, center[i].x)< depthImg_show.at<uchar>(center[j].y, center[j].x))
							{
							j = i;
							}
							/*if (abs(center[i].x - 256)<(abs(center[j].x - 256)))
							{
								j = i;
							}*/
						}
						else
						{
							j = i;
						}
						number_j++;

					}
				}
			}
		}
	}
	if (j > -1)
	{
		Scalar color1 = Scalar(0, 255, 0);//������ɫ
		Scalar color2 = Scalar(255, 0, 0);//������ɫ
		drawContours(drawing, contours_poly, j, color1, 1, 8, vector<Vec4i>(), 0, Point());//��������
		circle(drawing, center[j], (int)radius[j], color2, 2, 8, 0);//����Բ																//	circle(srcImage, center[j], 3, Scalar(0, 255, 255), -1, 8, 0);
		circle(asrc, center[j], (int)radius[j], color2, 2, 8, 0);
		cout << "Բ�ĵĺ����꣺" << center[j].x << endl;
#ifdef _debug
		cout << j << "��" << center[j].x << endl;
		cout << "��r  " << radius[j] << endl;
		printf("%.2d\n", g_c8BitDepth.at<uchar>(center[j].y, center[j].x));
#endif
		point = center[j].x;
		juli = depthImg_show.at<uchar>(center[j].y, center[j].x);
		juli = juli * 1.0 / 255 * 4500;
		cout << "���룺" << juli << endl;
	}
	else
	{
		point = 0;
		juli = 0;
	}
	cv::imshow("��Բ��ͼƬ", drawing);
	cv::imshow("��Բ֮��Ĳ�ɫͼ", asrc);
#ifdef _debug
	imshow("drawing", drawing);
	imshow("srcimage", srcImage);
#endif
	t2 = ((double)cvGetTickCount() - t2) / (cvGetTickFrequency() * 1000);
	cout << "pic����ʱ��: " << t2 << "ms" << endl;
}
// ת��depthͼ��cv::Mat
Mat mythread::ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
{
	Mat img(nHeight, nWidth, CV_8UC1);
	//ָ��ͷָ��
	uchar* p_mat = img.data;
	//ָ�����һ��Ԫ�ص�ָ��
	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
	while (pBuffer < pBufferEnd)  //16λ���ֵΪ65536
	{
		//��16λ����ת����8λ
		*p_mat = *pBuffer / 65536.0 * 256;
		pBuffer++;
		p_mat++;
	}
	return img;
}
double angle(CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{
	double dx1 = pt1->x - pt0->x;
	double dy1 = pt1->y - pt0->y;
	double dx2 = pt2->x - pt0->x;
	double dy2 = pt2->y - pt0->y;
	//1e-10���ǡ�aeb������ʽ����ʾa����10��b�η���   
	//����b������������a������С����   
	//?���Ҷ���CosB=(a^2+c^2-b^2)/2ac������������ļ����ƺ�������   
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}
// returns sequence of squares detected on the image.   
// the sequence is stored in the specified memory storage  
CvSeq* findSquares4(IplImage* img, CvMemStorage* storage)
{
	CvSeq* contours;
	CvSize sz = cvSize(img->width & -2, img->height & -2);//�����͸�Ϊ���� ���Ϊż��
	IplImage* timg = cvCloneImage(img); // make a copy of input image   
	IplImage* gray = cvCreateImage(sz, 8, 1);
	IplImage* pyr = cvCreateImage(cvSize(sz.width / 2, sz.height / 2), 8, 3);
	IplImage* tgray;
	CvSeq* result;

	
	CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);

	// select the maximum ROI in the image   
	// with the width and height divisible by 2   
	cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));

	// down-scale and upscale the image to filter out the noise   
	//ʹ��gaussian�������ֽ������ͼ�����²��������ȶ��������ͼ����ָ���˲������о����Ȼ��ͨ���ܾ�ż�������������²���   
	cvPyrDown(timg, pyr, 7);
	//���� cvPyrUp ʹ��Gaussian �������ֽ������ͼ�����ϲ���������ͨ����ͼ���в��� 0 ż���к�ż���У�Ȼ��Եõ���ͼ����ָ�����˲������и�˹����������˲�������4����ֵ���������ͼ��������ͼ��� 4 ����С��   
	cvPyrUp(pyr, timg, 7);
	tgray = cvCreateImage(sz, 8, 1);

	// find squares in every color plane of the image   
	for (int c = 0; c <2; c++)
	{
		// extract the c-th color plane   
		//���� cvSetImageCOI ���ڸ�����ֵ���ø���Ȥ��ͨ����ֵ 0 ��ζ�����е�ͨ������ѡ��, 1 ��ζ�ŵ�һ��ͨ����ѡ���ȵȡ�   
		cvSetImageCOI(timg, c + 1);
		cvCopy(timg, tgray, 0);

		// try several threshold levels   
		for (int l = 0; l < 1; l++)
		{
			
			//if (l == 0)
			//{
			//	// apply Canny. Take the upper threshold from slider   
			//	// and set the lower to 0 (which forces edges merging)    
			//	cvCanny(tgray, gray, 60, 180, 3);
			//	// dilate canny output to remove potential   
			//	// holes between edge segments    
			//	//ʹ������ṹԪ������ͼ��   
			//	cvDilate(gray, gray, 0, 1);
			//}
			//else
			//{
			//	 apply threshold if l!=0:   
			//	        tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0   
			//	cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );   
			//	cvThreshold(tgray, gray, 50, 255, CV_THRESH_BINARY);
			//	}			
				// find contours and store them all as a list   
			//}
			cvThreshold(tgray, gray, 50, 255, CV_THRESH_BINARY);
			cvFindContours(gray, storage, &contours, sizeof(CvContour),
				CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
			// test each contour   
			while (contours)
			{
				//��ָ�����ȱƽ����������   
				result = cvApproxPoly(contours, sizeof(CvContour), storage,////�ƽ�һ������
					CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
				          
				if (result->total == 4 &&
					fabs(cvContourArea(result, CV_WHOLE_SEQ)) > 1000 &&
					cvCheckContourConvexity(result))
				{
					for (int i = 0; i < 4; i++)
						cvSeqPush(squares,
						(CvPoint*)cvGetSeqElem(result, i));
				}

				contours = contours->h_next;
			}
		}
	}

	// release all the temporary images   
	cvReleaseImage(&gray);
	cvReleaseImage(&pyr);
	cvReleaseImage(&tgray);
	cvReleaseImage(&timg);

	return squares;/////�洢��ÿһ������
}
// the function draws all the squares in the image   
void drawSquares(IplImage* img, CvSeq* squares, int &out_x,  double &rate)
{
	CvSeqReader reader;
	IplImage* cpy = cvCloneImage(img);
	
	cvStartReadSeq(squares, &reader, 0);

	for (int i = 0; i < squares->total; i += 4)
	{
		
		CvPoint* rect = pt;
		int count = 4;

		memcpy(pt, reader.ptr, squares->elem_size);////////��ÿһ��������в���///////////
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		memcpy(pt + 1, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		memcpy(pt + 2, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		memcpy(pt + 3, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		S = (abs((rect[1].x - rect[0].x) * (rect[2].y - rect[0].y) - (rect[2].x - rect[0].x) * (rect[1].y - rect[0].y)) + abs((rect[2].x - rect[0].x) * (rect[3].y - rect[0].y) - (rect[3].x - rect[0].x) * (rect[2].y - rect[0].y)))*0.5;
		

		Mat dst;
		Mat roi = Mat::zeros(bgr_frame.size(), CV_8U);
		Mat image;
		cv::Mat hsv_image;
		if ((rect[0].x != 1)&&(S>10000) && (S<50000))
		{
			vector<vector<Point>> contour;
			vector<Point> pts;
			pts.push_back(Point(rect[0].x, rect[0].y));
			pts.push_back(Point(rect[1].x, rect[1].y));
			pts.push_back(Point(rect[2].x, rect[2].y));
			pts.push_back(Point(rect[3].x, rect[3].y));
		
			
			contour.push_back(pts);
			drawContours(roi, contour, 0, Scalar::all(255), -1);
			bgr_frame.copyTo(dst, roi);

			/////////////��ɫʶ�𲿷�///////////
			 image = dst.clone();
			hsv_image.create(image.size(), image.type());
			cv::cvtColor(image, hsv_image, CV_BGR2HSV);
			vector<cv::Mat> channels;
			cv::split(hsv_image, channels);	
			int count_gray = 0;
			int count_other = 0;
			for (int i = 0; i < image.rows; i += 2)
			{
				const uchar* curr_r_hue = channels[0].ptr<const uchar>(i);
				const uchar* curr_r_satur = channels[1].ptr<const uchar>(i);
				const uchar* curr_r_value = channels[2].ptr<const uchar>(i);
				for (int j = 0; j <image.cols; j += 2) {
					if ((curr_r_hue[j] <= rect_h_max && curr_r_hue[j] >= rect_h_min) && (curr_r_satur[j]>rect_s_min && curr_r_satur[j]<rect_s_max) && (curr_r_value[j] <= rect_v_max && curr_r_value[j] >= rect_v_min))
					{
						count_gray++; 
					}
					else if ((curr_r_hue[j] != 0) && (curr_r_satur[j] != 0) && (curr_r_value[j] != 0))
					{
						count_other++;
					}			
				}
			}
			rate = 1.0*count_gray / (count_other + count_gray);
			if (rate>=(gray_rate/100))
			{
				cvPolyLine(cpy, &rect, &count, 1, 1, CV_RGB(0, 255, 0), 3, CV_AA, 0);
				
				if (S > sign_S) {     
					sign = 1;
				}
				/*if ((rect[0].y < rect[3].y) && (rect[1].y < rect[3].y))
				{
					out_x = (rect[0].x + rect[1].x) / 2;
				}
				else if ((rect[3].y < rect[2].y) && (rect[0].y < rect[1].y))
				{
					out_x = (rect[3].x + rect[0].x) / 2;
				}
				else if ((rect[2].y < rect[1].y) && (rect[3].y < rect[0].y))
				{
					out_x = (rect[2].x + rect[3].x) / 2;
				}
				else if ((rect[1].y < rect[3].y) && (rect[2].y < rect[3].y))
				{
					out_x = (rect[1].x + rect[2].x) / 2;
				}
				*/
				//CvPoint rect_1, rect_2,rect_3,rect_4;
				//int i = 0,j=0,k=0;
				//int x_min = rect[0].x;
				//int y_min = rect[0].y;
				//for (i = 0; i < 4; i++) {
				//	if (rect[i].y < rect[0].y)
				//	{
				//		rect_1 = rect[i];
				//		j = i;
				//	}
				//} 
				//for (i = 0; (i < 4)&&(i!=j); i++) {
				//	if (rect[i].y < rect[0].y)
				//	{
				//		rect_2 = rect[i];
				//		k = i;
				//	}
				//}
				//for (i = 0; (i < 4) && (i != j)&&(i != k); i++) {
				//	if (rect[i].y < rect[0].y)
				//	{
				//		rect_3 = rect[i];
				//	}
				//}			
				//if((abs(rect_1.y-rect_2.y)<5)&&(abs(rect_3.y - rect_4.y)<5))
				out_x = (rect[0].x+ rect[1].x + rect[2].x + rect[3].x) / 4;
				
			}

		}


	}

	// show the resultant image   
	cvShowImage(wndname, cpy);
	cvReleaseImage(&cpy);
}
void on_trackbar(int a, int&out_x, double &rate)
{
	if (img)
		drawSquares(img, findSquares4(img, storage), out_x, rate);
}
char* names[] = { 0 };
void MyMainWindow_uartSendCommand(int x, int juli)
{
	DWORD wCount;//���͵��ֽ���
	char sendData[10];
	sendData[0] = '@';
	sendData[1] = '^';
	sendData[2] = 'v';
	sendData[3] = 0;
	sendData[4] = 0;
	sendData[5] = (x >> 8) & 0xff;
	sendData[6] = x & 0xff;
	sendData[7] = (juli >> 8) & 0xff;
	sendData[8] = juli & 0xff;
	sendData[9] = 0;
	for (int i = 0; i < 9; i++)
		sendData[9] += sendData[i];
	cout << int(sendData[9]) << endl;
	//currentSerialPort->write(sendData, 10);
	WriteFile(hCom, sendData, 10, &wCount, NULL);
}
void camera_on_trackbar(int,void*)
{
	 capture=camera;
}
void mythread::startthread()
{
	//////////////Kinect�ɼ�����///////////
	IKinectSensor*          m_pKinectSensor;
	IDepthFrameReader*      m_pDepthFrameReader;
	IDepthFrameSource*      pDepthFrameSource = NULL;
	IColorFrameSource*      pColorFrameSource;
	IColorFrameReader*      m_pColorFrameReader;
	IFrameDescription*      depthFrameDescription = NULL;
	IFrameDescription*      colorFrameDescription = NULL;
	ColorImageFormat        imageFormat = ColorImageFormat_None;
	ICoordinateMapper*      coordinateMapper = NULL;

	GetDefaultKinectSensor(&m_pKinectSensor);
	m_pKinectSensor->Open();
	/*if (SUCCEEDED(hr))
	{
		MessageBox(NULL, TEXT("�ɹ�"), TEXT("�ɹ�"), MB_OK);
	}
	else
	{
		MessageBox(NULL, TEXT("ʧ��"), TEXT("ʧ��"), MB_OK);
	}*/
	
	//��������Ϣ������
	m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	//��ò�ɫ��Ϣ������camera
	m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	//�������Ϣ֡��ȡ��
	pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	//�򿪲�ɫ��Ϣ֡��ȡ��
	pColorFrameSource->OpenReader(&m_pColorFrameReader);
	//////////////////////////////////////////////////////////////////////////////////////////


	////////////����ͷ�ɼ�ͼ��///////////////
	namedWindow("control", WINDOW_NORMAL);
	cvCreateTrackbar("ӳ�����ֵ", "control", &yuzhi, 5000);
	createTrackbar("����ͷ����", "control", &camera, 5, camera_on_trackbar);
	createTrackbar("��������ֵ", "control", &upperpart_threshold, 424);
	camera_on_trackbar(camera, 0);
	cv::VideoCapture capture(camera);
	cv::VideoCapture mm(0);

	int i, c;
	// create memory storage that will contain all the dynamic data   
	storage = cvCreateMemStorage(0);
	Mat t_desti;
	Mat r_desti;
	Mat desti;
	Mat bgr_frame_hsv;
	/////////////////////////////////////////////////������////////////

	//��ɫ���򻬶���
	namedWindow("control_red_volleyball", WINDOW_NORMAL);
	cvResizeWindow("control_red_volleyball", 800, 800);
	//��ɫ���򻬶���
	namedWindow("contral_yellow_volleyball", WINDOW_NORMAL);
	cvResizeWindow("contral_yellow_volleyball", 800, 800);
	//����ɫ���򻬶���
	namedWindow("control_blue_basketball", WINDOW_NORMAL);
	cvResizeWindow("control_blue_basketball", 800, 800);
	//��ɫ���򻬶���
	namedWindow("control_orange_basketball", WINDOW_NORMAL);
	cvResizeWindow("control_orange_basketball", 800, 800);
	
	////���β���������
	namedWindow("control_gray_rect", WINDOW_NORMAL);
	cvResizeWindow("control_gray_rect", 800, 800);



	while (mythreadUartState == threadUartState::ON)
	{
		while (mythreadUartState == threadUartState::ON)
		{
			//while(true)��������ģʽ��Kinectģʽ������ͷģʽ
			//��ɫ���򻬶���
			cvCreateTrackbar("y_h_max", "control_red_volleyball", &yellow_h_max, 255);
			cvCreateTrackbar("y_h_min", "control_red_volleyball", &yellow_h_min, 255);
			cvCreateTrackbar("y_s_max", "control_red_volleyball", &yellow_s_max, 255);
			cvCreateTrackbar("y_s_min", "control_red_volleyball", &yellow_s_min, 255);
			cvCreateTrackbar("y_v_max", "control_red_volleyball", &yellow_v_max, 255);
			cvCreateTrackbar("y_v_min", "control_red_volleyball", &yellow_v_min, 255);
			cvCreateTrackbar("r1_h_max", "control_red_volleyball", &red1_h_max, 255);
			cvCreateTrackbar("r1_h_min", "control_red_volleyball", &red1_h_min, 255);
			cvCreateTrackbar("r1_s_max", "control_red_volleyball", &red1_s_max, 255);
			cvCreateTrackbar("r1_s_min", "control_red_volleyball", &red1_s_min, 255);
			cvCreateTrackbar("r1_v_max", "control_red_volleyball", &red1_v_max, 255);
			cvCreateTrackbar("r1_v_min", "control_red_volleyball", &red1_v_min, 255);
			cvCreateTrackbar("r2_h_max", "control_red_volleyball", &red2_h_max, 255);
			cvCreateTrackbar("r2_h_min", "control_red_volleyball", &red2_h_min, 255);
			cvCreateTrackbar("r2_s_max", "control_red_volleyball", &red2_s_max, 255);
			cvCreateTrackbar("r2_s_min", "control_red_volleyball", &red2_s_min, 255);
			cvCreateTrackbar("r2_v_max", "control_red_volleyball", &red2_v_max, 255);
			cvCreateTrackbar("r2_v_min", "control_red_volleyball", &red2_v_min, 255);
			cvCreateTrackbar("w_h_max", "control_red_volleyball", &white_h_max, 255);
			cvCreateTrackbar("w_h_min", "control_red_volleyball", &white_h_min, 255);
			cvCreateTrackbar("w_s_max", "control_red_volleyball", &white_s_max, 255);
			cvCreateTrackbar("w_s_min", "control_red_volleyball", &white_s_min, 255);
			cvCreateTrackbar("w_v_max", "control_red_volleyball", &white_v_max, 255);
			cvCreateTrackbar("w_v_min", "control_red_volleyball", &white_v_min, 255);
		
			//��ɫ���򻬶���
			cvCreateTrackbar("b_h_max", "contral_yellow_volleyball", &blove_h_max, 255);
			cvCreateTrackbar("b_h_min", "contral_yellow_volleyball", &blove_h_min, 255);
			cvCreateTrackbar("b_s_max", "contral_yellow_volleyball", &blove_s_max, 255);
			cvCreateTrackbar("b_s_min", "contral_yellow_volleyball", &blove_s_min, 255);
			cvCreateTrackbar("b_v_max", "contral_yellow_volleyball", &blove_v_max, 255);
			cvCreateTrackbar("b_v_min", "contral_yellow_volleyball", &blove_v_min, 255);
			cvCreateTrackbar("y_h_max", "contral_yellow_volleyball", &yellow_h_max, 255);
			cvCreateTrackbar("y_h_min", "contral_yellow_volleyball", &yellow_h_min, 255);
			cvCreateTrackbar("y_s_max", "contral_yellow_volleyball", &yellow_s_max, 255);
			cvCreateTrackbar("y_s_min", "contral_yellow_volleyball", &yellow_s_min, 255);
			cvCreateTrackbar("y_v_max", "contral_yellow_volleyball", &yellow_v_max, 255);
			cvCreateTrackbar("y_v_min", "contral_yellow_volleyball", &yellow_v_min, 255);
			cvCreateTrackbar("w_h_max", "contral_yellow_volleyball", &white_h_max, 255);
			cvCreateTrackbar("w_h_min", "contral_yellow_volleyball", &white_h_min, 255);
			cvCreateTrackbar("w_s_max", "contral_yellow_volleyball", &white_s_max, 255);
			cvCreateTrackbar("w_s_min", "contral_yellow_volleyball", &white_s_min, 255);
			cvCreateTrackbar("w_v_max", "contral_yellow_volleyball", &white_v_max, 255);
			cvCreateTrackbar("w_v_min", "contral_yellow_volleyball", &white_v_min, 255);
	
			//����ɫ���򻬶���
			cvCreateTrackbar("b_h_max", "control_blue_basketball", &blue_h_max, 255);
			cvCreateTrackbar("b_h_min", "control_blue_basketball", &blue_h_min, 255);
			cvCreateTrackbar("b_s_max", "control_blue_basketball", &blue_s_max, 255);
			cvCreateTrackbar("b_s_min", "control_blue_basketball", &blue_s_min, 255);
			cvCreateTrackbar("b_v_max", "control_blue_basketball", &blue_v_max, 255);
			cvCreateTrackbar("b_v_min", "control_blue_basketball", &blue_v_min, 255);
			cvCreateTrackbar("g_h_max", "control_blue_basketball", &gray_h_max, 255);
			cvCreateTrackbar("g_h_min", "control_blue_basketball", &gray_h_min, 255);
			cvCreateTrackbar("g_s_max", "control_blue_basketball", &gray_s_max, 255);
			cvCreateTrackbar("g_s_min", "control_blue_basketball", &gray_s_min, 255);
			cvCreateTrackbar("g_v_max", "control_blue_basketball", &gray_v_max, 255);
			cvCreateTrackbar("g_v_min", "control_blue_basketball", &gray_v_min, 255);
			cvCreateTrackbar("y_h_max", "control_blue_basketball", &lemon_h_max, 255);
			cvCreateTrackbar("y_h_min", "control_blue_basketball", &lemon_h_min, 255);
			cvCreateTrackbar("y_s_max", "control_blue_basketball", &lemon_s_max, 255);
			cvCreateTrackbar("y_s_min", "control_blue_basketball", &lemon_s_min, 255);
			cvCreateTrackbar("y_v_max", "control_blue_basketball", &lemon_v_max, 255);
			cvCreateTrackbar("y_v_min", "control_blue_basketball", &lemon_v_min, 255);

		
			// ��ɫ���򻬶���
			cvCreateTrackbar("o_h_max", "control_orange_basketball", &orange_h_max, 255);
			cvCreateTrackbar("o_h_min", "control_orange_basketball", &orange_h_min, 255);
			cvCreateTrackbar("o_s_max", "control_orange_basketball", &orange_s_max, 255);
			cvCreateTrackbar("o_s_min", "control_orange_basketball", &orange_s_min, 255);
			cvCreateTrackbar("o_v_max", "control_orange_basketball", &orange_v_max, 255);
			cvCreateTrackbar("o_v_min", "control_orange_basketball", &orange_v_min, 255);

		
			//���λ�ɫ������
			cvCreateTrackbar("re_h_max", "control_gray_rect", &rect_h_max, 255);
			cvCreateTrackbar("re_h_min", "control_gray_rect", &rect_h_min, 255);
			cvCreateTrackbar("re_s_max", "control_gray_rect", &rect_s_max, 255);
			cvCreateTrackbar("re_s_min", "control_gray_rect", &rect_s_min, 255);
			cvCreateTrackbar("re_v_max", "control_gray_rect", &rect_v_max, 255);
			cvCreateTrackbar("re_v_min", "control_gray_rect", &rect_v_min, 255);
			cvCreateTrackbar("gray_rate", "control_gray_rect", &gray_rate, 100);
			cvCreateTrackbar("exposure", "control_gray_rect", &exposure, 10);
			cvCreateTrackbar("sign_S", "control_gray_rect", &sign_S, 30000);
			cvCreateTrackbar("whrect_h_max", "control_gray_rect", &whrect_h_max,255);
			cvCreateTrackbar("whrect_h_min", "control_gray_rect", &whrect_h_min, 255);
			cvCreateTrackbar("whrect_s_max", "control_gray_rect", &whrect_s_max, 255);
			cvCreateTrackbar("whrect_s_min", "control_gray_rect", &whrect_s_min, 255);
			cvCreateTrackbar("whrect_v_max", "control_gray_rect", &whrect_v_max, 255);
			cvCreateTrackbar("whrect_v_min", "control_gray_rect", &whrect_v_min, 255);
			//*judge = 'a';

			//com_data = 'a';
			emit find_singal(com_data);
			if (com_data != 'a')
			{
				//blue_h_max = 20;
				//qDebug("%d", blue_h_max);
				double ball_time = (double)cvGetTickCount();//Kinect������ʱ��

				IColorFrame*       pColorFrame = NULL;
				IDepthFrame*       pDepthFrame = NULL;

				
				//��ȡ���ͼ��
				int number = 0;
				while ((pDepthFrame == NULL) && (KinectState == KinectUartState::ON)) {
					//������ʱ���ȡ���������ѭ����ȡ�����֡
					m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
					number++;
					if (number == 4)
					{
						//MessageBox(NULL, TEXT("Kinect��ʧ��"), TEXT("Kinectʧ��"), MB_OK);
						//emit kinect_signal();

					}

				}
				if (KinectState == KinectUartState::ON)
				{
					pDepthFrame->get_FrameDescription(&depthFrameDescription);

					//��ȡ֡��������Ϣ����͸ߣ�
					depthFrameDescription->get_Width(&depth_width);
					depthFrameDescription->get_Height(&depth_height);
					printf("width=%d height=%d\n", depth_width, depth_height);
					UINT nBufferSize_depth = 0;//ͼ�����ظ���
					UINT16 *pBuffer_depth = NULL;//ָ��ͼ���ָ��
					cv::Mat pBuffer_depth1(depth_height, depth_width, CV_16SC1);
					//��ȡͼ�����ظ�����ָ��ͼ���ָ��
					pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);//Ϊ������ɫͼ�Ķ���
					pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, reinterpret_cast<UINT16**>(&pBuffer_depth1.data));//Ϊ�˵õ�depthImg_show
																															  //ת��Ϊ8λ��mat   
																															  //depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height);
																															  //���⻯��Ϊ�������ʾЧ��    
																															  //equalizeHist(depthImg_show, depthImg_show);
					pBuffer_depth1.convertTo(depthImg_show, CV_8U, 255.0f / 4500.0f, 0.0f);//depthImg_showΪչʾ���������ͼ�Լ������������õ����ͼ


																						   //��ȡ��ɫͼ��
					while (pColorFrame == NULL) {
						//������ʱ���ȡ���������ѭ����ȡ�����֡
						m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
					}
					//��ȡͼƬ������Ϣ
					pColorFrame->get_FrameDescription(&colorFrameDescription);
					int nWidth, nHeight;
					uchar *pBuffer = NULL;
					UINT nBufferSize = 0;
					colorFrameDescription->get_Width(&nWidth);
					colorFrameDescription->get_Height(&nHeight);
					//cout << "width=" << nWidth << endl;
					//	cout << "Height=" << nHeight << endl;
					pColorFrame->get_RawColorImageFormat(&imageFormat);
					//������Ϊ ColorImageFormat_Yuy2    = 5��ΪYuy2��ʽ   
					//cout << "imageformat is " << imageFormat << endl;
					//�½�һ��mat�������ڱ�������ͼ��,ע������ĸ���ǰ�����ں�
					Mat colorImg(nHeight, nWidth, CV_8UC4);
					pBuffer = colorImg.data;
					nBufferSize = colorImg.rows*colorImg.step;
					pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);


					//�����֡ӳ�䵽��ɫ�ռ� 
					m_pKinectSensor->get_CoordinateMapper(&coordinateMapper);
					ColorSpacePoint* colorSpacePoint = new ColorSpacePoint[depth_width*depth_height];
					coordinateMapper->MapDepthFrameToColorSpace(depth_width*depth_height, pBuffer_depth, depth_width*depth_height, colorSpacePoint);


					UINT16 *depthData = NULL;
					depthData = pBuffer_depth;
					Mat resultImg(depth_height, depth_width, CV_8UC4, Scalar::all(0));

					for (int i = 0; i < depth_height; i++)
					{
						for (int j = 0; j < depth_width; j++)
						{
							unsigned int index = i * depth_width + j;
							ColorSpacePoint csp = colorSpacePoint[index];
							int colorX = static_cast<int>(floor(csp.X + 0.5));
							int colorY = static_cast<int>(floor(csp.Y + 0.5));
							//ѡȡ���ڲ�ɫͼ���ϵĵ㲢��ǰ������������ֵ�ָ�,�ٴθ�����ֵ��С
							if (colorX >= 0 && colorX < nWidth && colorY >= 0 && colorY < nHeight && *depthData < yuzhi)
							{
								//������ɫ��Ϣ
								resultImg.at<cv::Vec4b>(i, j) = colorImg.at<cv::Vec4b>(colorY, colorX);//�������ص�ʱ�����к��� ����Ϊ�� ����Ϊ��
							}
							depthData++;
						}

					}

					cout << "��ɫ�Ŀ�" << resultImg.cols << endl;

					switch (com_data)
					{
					case '1':
						picprocess(resultImg, depthImg_show, 1);

						break;
					case '2':

						picprocess(resultImg, depthImg_show, 2);
						break;
					case '3':
						picprocess(resultImg, depthImg_show, 3);

						break;
					case '4':

						picprocess(resultImg, depthImg_show, 4);
						break;
					case '5':

						picprocess(resultImg, depthImg_show, 5);
						break;
					case '6':

						picprocess(resultImg, depthImg_show, 6);
						break;
					case '7':

						picprocess(resultImg, depthImg_show, 7);
						break;
					case '8':

						picprocess(resultImg, depthImg_show, 8);
						break;
					default:
						break;
					}
					//���ڷ���
					emit com_signal(0, point, juli);

					ball_time = ((double)cvGetTickCount() - ball_time) / (cvGetTickFrequency() * 1000);
					//cout << "����ʱ��: " << ball_time << "ms" << endl;
					emit ball_singal(point, juli, ball_time, upperpart_threshold);
					cv::waitKey(10);
					pDepthFrame->Release();
					pColorFrame->Release();
					delete[] colorSpacePoint;
					colorImg.release();			
				}
				depthImg_show.release();
				
				

			}
			else
			{
				capture.set(CV_CAP_PROP_EXPOSURE, (-1)*exposure);
				double rect_t = (double)cvGetTickCount();
				capture >> bgr_frame;
				if (!bgr_frame.empty())
				{
					cvtColor(bgr_frame, bgr_frame_hsv, CV_BGR2HSV);
					inRange(bgr_frame_hsv, Scalar(whrect_h_min, whrect_s_min, whrect_v_min), Scalar(whrect_h_max, whrect_s_max, whrect_v_max), t_desti);
					inRange(bgr_frame_hsv, Scalar(rect_h_min, rect_s_min, rect_v_min), Scalar(rect_h_max, rect_s_max, rect_v_max), r_desti);
					Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
					//morphologyEx(t_desti, desti, MORPH_CLOSE, element);

					dilate(t_desti, desti, element);

					//medianBlur(desti, desti, 3);
					//erode(desti, desti, element_erode);
					imshow("ʶ���ɫ", t_desti);
					imshow("ʶ���ɫ", r_desti);
					//imshow("����֮���ͼƬ", desti);
					imwrite("3.jpg", desti);
					imshow("��Ƶ", bgr_frame);
					img0 = cvLoadImage("3.jpg");
					if (!img0)
					{
						printf("Couldn't load %s\n", names[2]);
						continue;
					}
					img = cvCloneImage(img0);
					cvNamedWindow(wndname, 0);
					//qDebug(u8"��%d, �ߣ� %d", img->width, img->height);
					int out_x = 0;
					double rate = 0;
					S = 0;
					sign = 0;
					on_trackbar(0, out_x, rate);
					waitKey(10);
					rect_t = ((double)cvGetTickCount() - rect_t) / (cvGetTickFrequency() * 1000);
					emit rect_singal(out_x, rate, rect_t, S, sign);//���ݾ������ݵ��ź�
					//���ڷ�������
					if (sign == 1)
					{
						sign = 'z';
						emit com_signal(sign, out_x, 1000);
					}
					else
					{
						emit com_signal(0, out_x, 1000);
					}
					cvReleaseImage(&img);
					cvReleaseImage(&img0);
					// clear memory storage - reset free space position   
					cvClearMemStorage(storage);
				}
			}
        }
	
	}
	cvDestroyAllWindows();
	
	//cvDestroyWindow("contral_blue_volleyball");//��ɫ���򻬶���
	//cvDestroyWindow("contral_yellow_volleyball");//��ɫ���򻬶���
	//cvDestroyWindow("control_blue_basketball");//����ɫ���򻬶���
	//cvDestroyWindow("control_orange_basketball");//��ɫ���򻬶���
	//cvDestroyWindow("control_gray_rect");//���β���������
	//cvDestroyWindow(wndname);//���پ���ͼƬ

	cv::waitKey(1);


}

///////���㺯����ʵ��////////

void mythread::blue_set() /////////��ɫ���㺯����ʵ��//////////
{
	blue_h_max = 0, blue_h_min = 0, blue_s_max = 0, blue_s_min = 0, blue_v_max = 0, blue_v_min = 0;
}
void mythread::gray_set() /////////��ɫ���㺯����ʵ��//////////
{
	gray_h_max = 0, gray_h_min = 0, gray_s_max = 0, gray_s_min = 0, gray_v_max = 0, gray_v_min = 0;
}
void mythread::red1_set() /////////��1���㺯����ʵ��//////////
{
	red1_h_max = 0, red1_h_min = 0, red1_s_max = 0, red1_s_min = 0, red1_v_max = 0, red1_v_min = 0;
}
void mythread::red2_set() /////////��2���㺯����ʵ��//////////
{
	red2_h_max = 0, red2_h_min = 0, red2_s_max = 0, red2_s_min = 0, red2_v_max = 0, red2_v_min = 0;
}
void mythread::yellow_set() /////////��ɫ���㺯����ʵ��//////////
{
	yellow_h_max = 0, yellow_h_min = 0, yellow_s_max = 0, yellow_s_min = 0, yellow_v_max = 0, yellow_v_min = 0;
}
void mythread::orange_set() /////////��ɫ���㺯����ʵ��//////////
{	
	orange_h_max = 0, orange_h_min = 0, orange_s_max = 0, orange_s_min = 0, orange_v_max = 0, orange_v_min = 0;
}
void mythread::white_set() /////////��ɫ���㺯����ʵ��//////////
{
	white_h_max = 0, white_h_min = 0, white_s_max = 0, white_s_min = 0, white_v_max = 0, white_v_min = 0;
}
void mythread::blove_set()
{
	blove_h_max = 0, blove_h_min = 0, blove_s_max = 0, blove_s_min = 0, blove_v_max = 0, blove_v_min = 0;
}
void mythread::rect_white_set()
{
	whrect_h_max = 0, whrect_h_min = 0, whrect_s_max = 0, whrect_s_min = 0, whrect_v_max = 0, whrect_v_min = 0;
}
void mythread::rect_gray_set()
{
	
	rect_h_max = 0, rect_h_min = 0, rect_s_max = 0, rect_s_min = 0, rect_v_max = 0, rect_v_min = 0;
}
void mythread::lemon_set() /////////�������㺯����ʵ��//////////
{
	lemon_h_max = 0, lemon_h_min = 0, lemon_s_max = 0, lemon_s_min = 0, lemon_v_max = 0, lemon_v_min = 0;
}

///////////��λ������ʵ��/////////////
///////////һ����ĸ�λ///////////////////
void mythread::ball1_reset()
{
	yellow_reset();
	red1_reset();
	red2_reset();
	white_reset();
}
///////////������ĸ�λ///////////////////
void mythread::ball2_reset()
{
	yellow_reset();
	white_reset();
	blove_reset();
}
///////////������ĸ�λ///////////////////
void mythread::ball3_reset()
{
	lemon_reset();
	gray_reset();
	blue_reset();
}
///////////�ĺ���ĸ�λ///////////////////
void mythread::ball4_reset()
{
	orange_reset();
}


//////////���ݵı���////////////
void mythread::ball1_save()
{
	ofstream ball1_out("ball_1.txt");
	ball1_out <<"int red1_h_max_t = "<< red1_h_max<<" ,red1_h_min_t = "<< red1_h_min <<" ,red1_s_max_t = "<< red1_s_max << " ,red1_s_min_t = " << red1_s_min << " ,red1_v_max_t = "<< red1_v_max<<" ,red1_v_min_t = "<< red1_v_min<< ";" << endl;
	ball1_out << "int red2_h_max_t = "<< red2_h_max << " ,red2_h_min_t = " << red2_h_min << " ,red2_s_max_t = " << red2_s_max << " ,red2_s_min_t = " << red2_s_min << " ,red2_v_max_t = " << red2_v_max << " ,red2_v_min_t = " << red2_v_min << ";" << endl;
	ball1_out << "int white_h_max_t = " << white_h_max << " ,white_h_min_t = " << white_h_min << " ,white_s_max_t = " << white_s_max << " ,white_s_min_t = " << white_s_min << " ,white_v_max_t = " << white_v_max << " ,white_v_min_t = " << white_v_min << ";" << endl;
	ball1_out << "int yellow_h_max_t = " << yellow_h_max << " ,yellow_h_min_t = " << yellow_h_min << " ,yellow_s_max_t = " << yellow_s_max << " ,yellow_s_min_t = " << yellow_s_min << " ,yellow_v_max_t = " << yellow_v_max << " ,yellow_v_min_t = " << yellow_v_min << ";" << endl;
	ball1_out.close();
	system("start ball_1.txt");
}
void mythread::ball2_save()
{
	ofstream ball2_out("ball_2.txt");
	ball2_out << "int yellow_h_max_t = " << yellow_h_max << " ,yellow_h_min_t = " << yellow_h_min << " ,yellow_s_max_t = " << yellow_s_max << " ,yellow_s_min_t = " << yellow_s_min << " ,yellow_v_max_t = " << yellow_v_max << " ,yellow_v_min_t = " << yellow_v_min <<";"<< endl;
	ball2_out << "int white_h_max_t = " << white_h_max << " ,white_h_min_t = " << white_h_min << " ,white_s_max_t = " << white_s_max << " ,white_s_min_t = " << white_s_min << " ,white_v_max_t = " << white_v_max << " ,white_v_min_t = " << white_v_min << ";" << endl;
	ball2_out << "int blove_h_max_t = " << blove_h_max << " ,blove_h_min_t = " << blove_h_min << " ,blove_s_max_t = " << blove_s_max << " ,blove_s_min_t = " << blove_s_min << " ,blove_v_max_t = " << blove_v_max << " ,blove_v_min_t = " << blove_v_min << ";" << endl;
	ball2_out.close();
	system("start ball_2.txt");
}
void mythread::ball3_save()
{
	ofstream ball3_out("ball_3.txt");
	ball3_out << "int lemon_h_max_t = " << lemon_h_max << " ,lemon_h_min_t = " << lemon_h_min << " ,lemon_s_max_t = " << lemon_s_max << " ,lemon_s_min_t = " << lemon_s_min << " ,lemon_v_max_t = " << lemon_v_max << " ,lemon_v_min_t = " << lemon_v_min << ";" << endl;
	ball3_out << "int gray_h_max_t = " << gray_h_max << " ,gray_h_min_t = " << gray_h_min << " ,gray_s_max_t = " << gray_s_max << " ,gray_s_min_t = " << gray_s_min << " ,gray_v_max_t = " << gray_v_max << " ,gray_v_min_t = " << gray_v_min << ";" << endl;
	ball3_out << "int blue_h_max_t = " << blue_h_max << " ,blue_h_min_t = " << blue_h_min << " ,blue_s_max_t = " << blue_s_max << " ,blue_s_min_t = " << blue_s_min << " ,blue_v_max_t = " << blue_v_max << " ,blue_v_min_t = " << blue_v_min << ";" << endl;
	ball3_out.close();
	system("start ball_3.txt");
}
void mythread::ball4_save()
{
	ofstream ball4_out("ball_4.txt");
	ball4_out << "int orange_h_max_t = " << orange_h_max << " ,orange_h_min_t = " << orange_h_min << " ,orange_s_max_t = " << orange_s_max << " ,orange_s_min_t = " << orange_s_min << " ,orange_v_max_t = " << orange_v_max << " ,orange_v_min_t = " << orange_v_min << ";" << endl;
	ball4_out.close();
	system("start ball_4.txt");
}
void mythread::rect_white_save()
{
	ofstream rect_white_out("rect_white.txt");
	rect_white_out << "int whrect_h_max_t = " << whrect_h_max << " ,whrect_h_min_t = " << whrect_h_min << " ,whrect_s_max_t = " << whrect_s_max << " ,whrect_s_min_t = " << whrect_s_min << " ,whrect_v_max_t = " << whrect_v_max << " ,whrect_v_min_t = " << whrect_v_min << ";" << endl;
	rect_white_out.close();
	system("start rect_white.txt");
}
void mythread::rect_gray_save()
{
	ofstream rect_gray_out("rect_gray.txt");
	rect_gray_out << "int rect_h_max_t = " << rect_h_max << " ,rect_h_min_t = " << rect_h_min << " ,rect_s_max_t = " << rect_s_max << " ,rect_s_min_t = " << rect_s_min << " ,rect_v_max_t = " << rect_v_max << " ,rect_v_min_t = " << rect_v_min <<" ,gray_rate = "<<gray_rate_t<<" ,exposure = "<<exposure_t<<", sign_S = "<<sign_S_t<<";" << endl;
	rect_gray_out.close();
	system("start rect_gray.txt");
}
void mythread::all_save()
{
	ofstream ball_out("all.txt");
	ball_out << "int red1_h_max_t = " << red1_h_max << " ,red1_h_min_t = " << red1_h_min << " ,red1_s_max_t = " << red1_s_max << " ,red1_s_min_t = " << red1_s_min << " ,red1_v_max_t = " << red1_v_max << " ,red1_v_min_t = " << red1_v_min << ";" << endl;
	ball_out << "int red2_h_max_t = " << red2_h_max << " ,red2_h_min_t = " << red2_h_min << " ,red2_s_max_t = " << red2_s_max << " ,red2_s_min_t = " << red2_s_min << " ,red2_v_max_t = " << red2_v_max << " ,red2_v_min_t = " << red2_v_min << ";" << endl;
	ball_out << "int white_h_max_t = " << white_h_max << " ,white_h_min_t = " << white_h_min << " ,white_s_max_t = " << white_s_max << " ,white_s_min_t = " << white_s_min << " ,white_v_max_t = " << white_v_max << " ,white_v_min_t = " << white_v_min << ";" << endl;
	ball_out << "int blue_h_max_t = " << blue_h_max << " ,blue_h_min_t = " << blue_h_min << " ,blue_s_max_t = " << blue_s_max << " ,blue_s_min_t = " << blue_s_min << " ,blue_v_max_t = " << blue_v_max << " ,blue_v_min_t = " << blue_v_min << ";" << endl;
	ball_out << "int yellow_h_max_t = " << yellow_h_max << " ,yellow_h_min_t = " << yellow_h_min << " ,yellow_s_max_t = " << yellow_s_max << " ,yellow_s_min_t = " << yellow_s_min << " ,yellow_v_max_t = " << yellow_v_max << " ,yellow_v_min_t = " << yellow_v_min << ";" << endl;
	ball_out << "int blove_h_max_t = " << blove_h_max << " ,blove_h_min_t = " << blove_h_min << " ,blove_s_max_t = " << blove_s_max << " ,blove_s_min_t = " << blove_s_min << " ,blove_v_max_t = " << blove_v_max << " ,blove_v_min_t = " << blove_v_min << ";" << endl;
	ball_out << "int lemon_h_max_t = " << lemon_h_max << " ,lemon_h_min_t = " << lemon_h_min << " ,lemon_s_max_t = " << lemon_s_max << " ,lemon_s_min_t = " << lemon_s_min << " ,lemon_v_max_t = " << lemon_v_max << " ,lemon_v_min_t = " << lemon_v_min << ";" << endl;
	ball_out << "int gray_h_max_t = " << gray_h_max << " ,gray_h_min_t = " << gray_h_min << " ,gray_s_max_t = " << gray_s_max << " ,gray_s_min_t = " << gray_s_min << " ,gray_v_max_t = " << gray_v_max << " ,gray_v_min_t = " << gray_v_min << ";" << endl;
	ball_out << "int orange_h_max_t = " << orange_h_max << " ,orange_h_min_t = " << orange_h_min << " ,orange_s_max_t = " << orange_s_max << " ,orange_s_min_t = " << orange_s_min << " ,orange_v_max_t = " << orange_v_max << " ,orange_v_min_t = " << orange_v_min << ";" << endl;
	ball_out << "int rect_h_max_t = " << rect_h_max << " ,rect_h_min_t = " << rect_h_min << " ,rect_s_max_t = " << rect_s_max << " ,rect_s_min_t = " << rect_s_min << " ,rect_v_max_t = " << rect_v_max << " ,rect_v_min_t = " << rect_v_min << ";" << endl;
	ball_out << "int whrect_h_max_t = " << whrect_h_max << " ,whrect_h_min_t = " << whrect_h_min << " ,whrect_s_max_t = " << whrect_s_max << " ,whrect_s_min_t = " << whrect_s_min << " ,whrect_v_max_t = " << whrect_v_max << " ,whrect_v_min_t = " << whrect_v_min << ";" << endl;
	ball_out.close();
	system("start all.txt");
}
//////////////��ʱ���溯��////////////
void mythread::blue_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{
	blue_h_max_t = blue_h_max, blue_h_min_t = blue_h_min, blue_s_max_t = blue_s_max, blue_s_min_t = blue_s_min, blue_v_max_t = blue_v_max, blue_v_min_t = blue_v_min_t;
}
void mythread::gray_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{

	gray_h_max_t = gray_h_max, gray_h_min_t = gray_h_min, gray_s_max_t = gray_s_max, gray_s_min_t = gray_s_min, gray_v_max_t = gray_v_max, gray_v_min_t = gray_v_min_t;
}
void mythread::red1_tem() /////////��1��ʱ���溯����ʵ��//////////
{
	red1_h_max_t = red1_h_max, red1_h_min_t = red1_h_min, red1_s_max_t = red1_s_max, red1_s_min_t = red1_s_min, red1_v_max_t = red1_v_max, red1_v_min_t = red1_v_min_t;

}
void mythread::red2_tem() /////////��2��ʱ���溯����ʵ��//////////
{
	red2_h_max_t = red2_h_max, red2_h_min_t = red2_h_min, red2_s_max_t = red2_s_max, red2_s_min_t = red2_s_min, red2_v_max_t = red2_v_max, red2_v_min_t = red2_v_min_t;
}
void mythread::yellow_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{
	yellow_h_max_t = yellow_h_max, yellow_h_min_t = yellow_h_min, yellow_s_max_t = yellow_s_max, yellow_s_min_t = yellow_s_min, yellow_v_max_t = yellow_v_max, yellow_v_min_t = yellow_v_min_t;
}
void mythread::lemon_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{
	lemon_h_max_t = lemon_h_max, lemon_h_min_t = lemon_h_min, lemon_s_max_t = lemon_s_max, lemon_s_min_t = lemon_s_min, lemon_v_max_t = lemon_v_max, lemon_v_min_t = lemon_v_min_t;
}
void mythread::orange_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{
	orange_h_max_t = orange_h_max, orange_h_min_t = orange_h_min, orange_s_max_t = orange_s_max, orange_s_min_t = orange_s_min, orange_v_max_t = orange_v_max, orange_v_min_t = orange_v_min_t;
}
void mythread::white_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{
	white_h_max_t = white_h_max, white_h_min_t = white_h_min, white_s_max_t = white_s_max, white_s_min_t = white_s_min, white_v_max_t = white_v_max, white_v_min_t = white_v_min_t;

}
void mythread::blove_tem() /////////��ɫ��ʱ���溯����ʵ��//////////
{
	blove_h_max_t = blove_h_max, blove_h_min_t = blove_h_min, blove_s_max_t = blove_s_max, blove_s_min_t = blove_s_min, blove_v_max_t = blove_v_max, blove_v_min_t = blove_v_min_t;

}
/////////���ΰ�ɫ��ʱ���溯����ʵ��//////////
void mythread::rect_white_tem()
{

	whrect_h_max_t = whrect_h_max, whrect_h_min_t = whrect_h_min, whrect_s_max_t = whrect_s_max, whrect_s_min_t = whrect_s_min, whrect_v_max_t = whrect_v_max, whrect_v_min_t = whrect_v_min_t;
}
/////////���λ�ɫ��ʱ���溯����ʵ��//////////
void mythread::rect_gray_tem()
{
	rect_h_max_t = rect_h_max, rect_h_min_t = rect_h_min, rect_s_max_t = rect_s_max, rect_s_min_t = rect_s_min, rect_v_max_t = rect_v_max, rect_v_min_t = rect_v_min_t;
}
