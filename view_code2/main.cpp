


#include <iostream>
// OpenCV 头文件
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// Kinect for Windows SDK 头文件
#include <Kinect.h>

using namespace std;
int main(void)
{
	// 1a.获取感应器
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);

	// 1b. 打开感应器
	pSensor->Open();

	// 2a. 取得深度数据
	IColorFrameSource* pFrameSource = nullptr;
	pSensor->get_ColorFrameSource(&pFrameSource);

	// 2b. 取得深度数据的描述信息（宽、高）
	int        iWidth = 0;
	int        iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	pFrameSource->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&iWidth);
	pFrameDescription->get_Height(&iHeight);
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 建立图像矩阵，mColorImg用来存储8位4通道的图像数据
	cv::Mat mColorImg(iHeight, iWidth, CV_8UC4);
	cv::namedWindow("ColorImage");

	// 3a. 打开深度数据阅读器
	IColorFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	// 主循环
	while (1)
	{
		// 4a. 取得最新数据
		IColorFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. 把数据存入图像矩阵中并转换成BGRA格式
			pFrame->CopyConvertedFrameDataToArray(iWidth * iHeight * 4, (BYTE*)mColorImg.data, ColorImageFormat_Bgra);
			cv::imshow("ColorImage", mColorImg);

			// 4e. 释放变量pFrame
			pFrame->Release();
		}

		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	// 3b. 释放变量pFrameReader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 2d.释放变量pFramesSource
	pFrameSource->Release();
	pFrameSource = nullptr;

	// 1c.关闭感应器
	pSensor->Close();

	// 1d.释放感应器
	pSensor->Release();
	pSensor = nullptr;
}
