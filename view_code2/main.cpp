


#include <iostream>
// OpenCV ͷ�ļ�
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// Kinect for Windows SDK ͷ�ļ�
#include <Kinect.h>

using namespace std;
int main(void)
{
	// 1a.��ȡ��Ӧ��
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);

	// 1b. �򿪸�Ӧ��
	pSensor->Open();

	// 2a. ȡ���������
	IColorFrameSource* pFrameSource = nullptr;
	pSensor->get_ColorFrameSource(&pFrameSource);

	// 2b. ȡ��������ݵ�������Ϣ�����ߣ�
	int        iWidth = 0;
	int        iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	pFrameSource->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&iWidth);
	pFrameDescription->get_Height(&iHeight);
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// ����ͼ�����mColorImg�����洢8λ4ͨ����ͼ������
	cv::Mat mColorImg(iHeight, iWidth, CV_8UC4);
	cv::namedWindow("ColorImage");

	// 3a. ����������Ķ���
	IColorFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	// ��ѭ��
	while (1)
	{
		// 4a. ȡ����������
		IColorFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. �����ݴ���ͼ������в�ת����BGRA��ʽ
			pFrame->CopyConvertedFrameDataToArray(iWidth * iHeight * 4, (BYTE*)mColorImg.data, ColorImageFormat_Bgra);
			cv::imshow("ColorImage", mColorImg);

			// 4e. �ͷű���pFrame
			pFrame->Release();
		}

		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	// 3b. �ͷű���pFrameReader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 2d.�ͷű���pFramesSource
	pFrameSource->Release();
	pFrameSource = nullptr;

	// 1c.�رո�Ӧ��
	pSensor->Close();

	// 1d.�ͷŸ�Ӧ��
	pSensor->Release();
	pSensor = nullptr;
}
