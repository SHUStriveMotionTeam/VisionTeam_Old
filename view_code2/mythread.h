#pragma once
//2020/9/23 com_data 
#include <QObject>
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
#include<qpushbutton.h>
enum class threadUartState { ON, OFF };
enum class KinectUartState { ON, OFF };
using namespace cv;
using namespace std;
class mythread : public QObject
{
	Q_OBJECT

public:
	mythread(/*QObject *parent*/);
	~mythread();
	char com_data='3';
	int upperpart_threshold = 150;
	threadUartState mythreadUartState = threadUartState::OFF;
	KinectUartState KinectState = KinectUartState::OFF;
	///////����ɫ��������//////////////////////////////////////
	double bluede(Mat asrc);
	double grayde(Mat asrc);
	double orangede(Mat asrc);
	double yellowde(Mat asrc);
	double lemonde(Mat asrc);
	double redde(Mat asrc);
	double blovede(Mat asrc);
	double whitede(Mat asrc);
	/////////������/////////////////////////////////////////////
	//��һ�����Ǻ��������
	Mat findball_1(Mat image, Mat irang);
	//�ڶ������ǻư�������
	Mat findball_2(Mat image, Mat irang);
	//��������������ɫ����
	Mat findball_3(Mat image, Mat irang);
	//���ĸ����Ǻ�ɫ����
	Mat findball_4(Mat image, Mat irang);
	//�����������������
	Mat findball_5(Mat image, Mat irang);
	//����������������������
	Mat findball_6(Mat image, Mat irang);
	////////////////////////�����ܺ���//////////
	void picprocess(Mat asrc, Mat depth, int number);

	// ת��depthͼ��cv::Mat/////////////////////
	Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight);

	

public slots:
	void startthread();
	
	///////���㺯��//////////
	void blue_set();
	void gray_set();
	void red1_set();
	void red2_set();
	void yellow_set();
	void lemon_set();
	void orange_set();
	void white_set();
	void blove_set();
	void rect_white_set();
	void rect_gray_set();
	///////��λ����//////////
	void ball1_reset();
	void ball2_reset();
	void ball3_reset();
	void ball4_reset();

	//////////��ɫ��λ����///////////////
	void blue_reset();
	void gray_reset();
	void red1_reset();
	void red2_reset();
	void yellow_reset();
	void lemon_reset();
	void orange_reset();
	void white_reset();
	void blove_reset();
	void rect_white_reset();
	void rect_gray_reset();
	//////////////��ʱ���溯��////////////
	void blue_tem();
	void gray_tem();
	void red1_tem();
	void red2_tem();
	void yellow_tem();
	void lemon_tem();
	void orange_tem();
	void white_tem();
	void blove_tem();
	void rect_white_tem();
	void rect_gray_tem();
	/////////�����ݵı���////////
	void ball1_save();
	void ball2_save();
	void ball3_save();
	void ball4_save();
	void rect_white_save();
	void rect_gray_save();
	void all_save();
signals:
	void rect_singal(int out_x, double gray_rate, double rect_t, double rect_area,int sign);
	void find_singal(char judge);
	void ball_singal(int point_x, int dist, double ball_time,int upperpart_threshold);
    void ball1_singal(double ball1_rate1, double ball1_rate2,double rate3);
	void ball2_signal(double ball2_rate1, double ball2_rate2,double ball2_rate3);
	void ball3_signal(double ball3_rate1, double ball3_rate2, double ball3_rate3);
	void ball4_signal(double ball4_rate1);
	void com_signal(int,int,int);
	void kinect_signal();
};
