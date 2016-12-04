#pragma once
#include <Kinect.h>
#include <opencv2\highgui\highgui.hpp>
#include "ExportHeader.h"

class _EXPORT CColorFrame
{
public:
	CColorFrame();
	~CColorFrame();
	bool Initial(IKinectSensor* pSensor, int* iFrameWidth, int* iFrameHeight);
	bool UpdateFrame(cv::Mat& inputMat);

private:
	IColorFrameSource* pSource;
	IColorFrameReader* pReader;
	IColorFrame* pFrame;
	IFrameDescription* pDescription;
	int iWidth;
	int iHeight;
	UINT uBufferSize;
};
