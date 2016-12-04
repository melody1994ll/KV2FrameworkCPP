#include "DepthFrame.h"

template <class Interface>
inline void SafeRelease(Interface*& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

CDepthFrame::CDepthFrame()
{
	pSource = nullptr;
	pReader = nullptr;
	pFrame = nullptr;
	pDescription = nullptr;

	iWidth = iHeight = uMinDepth = uMaxDepth = uBufferSize = 0;
}


CDepthFrame::~CDepthFrame()
{
	//if (pDescription)
	//	SafeRelease(pDescription);
	//
	//if (pFrame)
	//	SafeRelease(pFrame);

	//if (pReader)
	//	SafeRelease(pReader);

	//if (pSource)
	//	SafeRelease(pSource);
}


bool CDepthFrame::Initial(IKinectSensor* pSensor, int* iFrameWidth, int* iFrameHeight)
{
	if (pSensor == nullptr)
		return false;

	if (pSensor->get_DepthFrameSource(&pSource) != S_OK)
		return false;
	
	if (pSource->get_FrameDescription(&pDescription) != S_OK)
		return false;

	pDescription->get_Width(&iWidth);
	pDescription->get_Height(&iHeight);
	*iFrameWidth = iWidth;
	*iFrameHeight = iHeight;
	uBufferSize = iWidth*iHeight;

	// get min & max depth value
	pSource->get_DepthMaxReliableDistance(&uMaxDepth);
	pSource->get_DepthMinReliableDistance(&uMinDepth);

	if (pSource->OpenReader(&pReader) != S_OK)
		return false;

	// release source
	SafeRelease(pSource);

	return true;
}

bool CDepthFrame::UpdateFrame(cv::Mat& inputMat, cv::Mat &inputRawMat)
{
	pFrame = nullptr;

	if (pReader->AcquireLatestFrame(&pFrame) != S_OK)
		return false;
	else
	{
		//cv::Mat rawDepthMat(iHeight, iWidth, CV_16UC1);
		inputRawMat = cv::Mat(iHeight, iWidth, CV_16UC1);
		// copy opencv data
		if (pFrame->CopyFrameDataToArray(uBufferSize, reinterpret_cast<UINT16*>(inputRawMat.data)) != S_OK)
			return false;

		inputRawMat.convertTo(inputMat, CV_8U, 255.0f / uMaxDepth);

		pFrame->Release();
	}

	return true;
}