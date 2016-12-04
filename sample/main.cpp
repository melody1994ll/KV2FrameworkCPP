#include "Kinect.h"
#include "KinectSensor.h"
#include "ColorFrame.h"
#include "BodyFrame.h"
#include "DepthFrame.h"

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>

#include <iostream>

using namespace std;
using namespace cv;


CBodyFrame bodyFrame;
CDepthFrame depthFrame;

int iDepthWidth = 0;
int iDepthHeight = 0;
Mat depthMat(iDepthHeight, iDepthWidth, CV_8UC1);
Mat depthRawDataMat(iDepthHeight, iDepthWidth, CV_16UC1);


void mouseclick(int bevent, int x, int y, int flags, void* ustc)
{
	if (bevent == CV_EVENT_LBUTTONDOWN)
	{
		cv::Point colorPoint = cv::Point(x, y);

		Eigen::Vector3f cameraPoint(0, 0, 0);

		bodyFrame.MapColorSpace2CameraSpace(colorPoint, cameraPoint, depthRawDataMat.data);

		printf("CLP: %d, %d | CP: %f, %f, %f \n", x, y, cameraPoint[0], cameraPoint[1], cameraPoint[2]);
	}
}

void mouseclick_depth(int bevent, int x, int y, int flags, void* ustc)
{
	if (bevent == CV_EVENT_LBUTTONDOWN)
	{
		cv::Point depthPoint = cv::Point(x, y);

		int convertDepth = 0;
		float rawDepth = 0;

		int matIndex = x + y*iDepthWidth;

		convertDepth = depthMat.at<uchar>(depthPoint);
		rawDepth = static_cast<float>(depthRawDataMat.at<unsigned short>(depthPoint));

		printf("P: (%d, %d) | depth: %d | raw depth: %f \n", depthPoint.x, depthPoint.y,
			convertDepth, rawDepth);
	}
}

int main()
{
	// open sensor
	CKinectSensor sensor;
	if (!sensor.OpenSensor())
		return -1;

	IKinectSensor* pSensor = sensor.GetSensor();

	//--------------------- color frame --------------------------------
	// initial color frame
	CColorFrame colorFrame;
	int iColorWidth = 0;
	int iColorHeight = 0;
	if (!colorFrame.Initial(pSensor, &iColorWidth, &iColorHeight))
		return -1;

	// initial cv image
	Mat colorMat(iColorHeight, iColorWidth, CV_8UC4);


	//-------------------- depth frame --------------------------------
	// initial depth frame
	//CDepthFrame depthFrame;

	if (!depthFrame.Initial(pSensor, &iDepthWidth, &iDepthHeight))
		return -1;
	// initial cv image
	//Mat depthMat(iDepthHeight, iDepthWidth, CV_8UC1);
	//Mat depthRawDataMat(iDepthHeight, iDepthWidth, CV_16UC1);
	namedWindow("Depth Frame");

	//------------------- body frame --------------------------------------
	
	if (!bodyFrame.Initial(pSensor))
	{
		cout << "initial body frame failed" << endl;
		return -1;
	}
	Mat bodyMat(iColorHeight, iColorWidth, CV_8UC4);
	namedWindow("Body Frame", 0);
	resizeWindow("Body Frame", 1280, 720);

	//---------------- mouse event -------------------------------------
	setMouseCallback("Body Frame", mouseclick);
	setMouseCallback("Depth Frame", mouseclick_depth);
	while (true)
	{
		// color frame
		if (colorFrame.UpdateFrame(colorMat))
		{
			bodyMat = colorMat.clone();
		}

		// depth frame - 8bit
		if (depthFrame.UpdateFrame(depthMat, depthRawDataMat))
		{
			imshow("Depth Frame", depthMat);
		}

		// update body frame 
		int iBodyTracked = 0;
		if (bodyFrame.UpdateFrame(iBodyTracked))
		{
			bodyFrame.DrawBodyLine(bodyMat);
			imshow("Body Frame", bodyMat);
		}

		if (waitKey(30) == VK_ESCAPE)
			break;
	}
}