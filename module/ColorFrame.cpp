#include "ColorFrame.h"
#include "ColorFrame.h"

template <class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

CColorFrame::CColorFrame()
{
	pSource = nullptr;
	pReader = nullptr;
	pFrame = nullptr;
	pDescription = nullptr;
	iWidth = iHeight = 0;
	uBufferSize = 0;
}


CColorFrame::~CColorFrame()
{
	//if (pDescription)
	//	SafeRelease(pDescription);

	//if (pReader)
	//	SafeRelease(pReader);

	//if (pSource)
	//	SafeRelease(pSource);

}

bool CColorFrame::Initial(IKinectSensor* pSensor, int* iFrameWidth, int* iFrameHeight)
{
	if (pSensor == NULL)
		return false;

	if (pSensor->get_ColorFrameSource(&pSource) != S_OK)
		return false;

	if (pSource->get_FrameDescription(&pDescription) != S_OK)
		return false;
	else
	{
		// get frame width and height
		pDescription->get_Height(&iHeight);
		pDescription->get_Width(&iWidth);
		*iFrameHeight = iHeight;
		*iFrameWidth = iWidth;
		uBufferSize = iHeight*iWidth * 4 * sizeof(BYTE);
	}
	pDescription->Release();
	pDescription = nullptr;

	if (pSource->OpenReader(&pReader) != S_OK)
		return false;

	// release frame source
	SafeRelease(pSource);

	return true;
}

bool CColorFrame::UpdateFrame(cv::Mat& inputMat)
{
	pFrame = nullptr;
	// Get lase frame
	if (pReader->AcquireLatestFrame(&pFrame) != S_OK)
		return false;
	else
	{
		// copy opencv data
		if (pFrame->CopyConvertedFrameDataToArray(uBufferSize, inputMat.data, ColorImageFormat_Bgra) != S_OK)
			return false;

		// release frame
		pFrame->Release();
	}

	return true;
}
