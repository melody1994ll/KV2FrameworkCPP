#include "KinectSensor.h"
#include "KinectSensor.h"

template <class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

CKinectSensor::CKinectSensor()
{
	pSensor = nullptr;
}


CKinectSensor::~CKinectSensor()
{
	CloseSensor();
}

bool CKinectSensor::OpenSensor()
{
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
		return false;

	if (pSensor->Open() != S_OK)
		return false;

	return true;
}

void CKinectSensor::CloseSensor()
{
	if (pSensor)
	{
		// close sensor
		pSensor->Close();
		SafeRelease(pSensor);
	}
}

IKinectSensor* CKinectSensor::GetSensor()
{
	return pSensor;
}
