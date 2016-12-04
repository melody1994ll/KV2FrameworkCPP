#pragma once
#include <Kinect.h>

#include "ExportHeader.h"

class _EXPORT CKinectSensor
{
public:
	CKinectSensor();
	~CKinectSensor();

	bool OpenSensor();
	void CloseSensor();

	IKinectSensor* GetSensor();

private:
	IKinectSensor* pSensor;
};

