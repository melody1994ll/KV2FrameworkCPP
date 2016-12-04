#pragma once

#include <Kinect.h>
#include <opencv2\core\core.hpp>

#include "ExportHeader.h"

class _EXPORT CDepthFrame
{
public:
	CDepthFrame();
	~CDepthFrame();

	bool Initial(IKinectSensor* pSensor, int* iFrameWidth, int* iFrameHeight);
	bool UpdateRawDepthFrame(cv::Mat& inputRawDepthMat);
	bool UpdateFrame(cv::Mat& inputMat, cv::Mat &inputRawMat);

private:
	IDepthFrameSource* pSource;
	IDepthFrameReader* pReader;
	IDepthFrame* pFrame;
	IFrameDescription* pDescription;

	int iWidth;
	int iHeight;

	UINT16 uMinDepth;
	UINT16 uMaxDepth;

	UINT uBufferSize;
};

