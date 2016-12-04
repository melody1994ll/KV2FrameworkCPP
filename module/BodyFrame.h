
#ifndef _BODYFRAME_H
#define _BODYFRAME_H

#include <Kinect.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc.hpp>

#include <Eigen\Dense>

#include "ExportHeader.h"

#define BODYFRAME_NO_TRACKED_ID 2333
#define BODYFRAME_NO_TRACKED_INDEX -1

using namespace Eigen;

class _EXPORT CBodyFrame
//class CBodyFrame
{
public:
	CBodyFrame();
	~CBodyFrame();

	bool Initial(IKinectSensor* pSensor);
	bool UpdateFrame(int& iBodyTracked);
	
	void DrawBodyLine(cv::Mat& colorMap, UINT64 userId = 0);
	JointType GetParentJoint(JointType childJoint);
	
	void GetJointPosition(int bodyIndex, JointType jointType, float& x, float& y, float& z);	// get the joint position in kinect space
	void GetJointPosition(int bodyIndex, JointType jointType, Vector3f& jointPos);				// get the joint position in kinect space
		
	void GetJointPositionInColorSpace(int bodyIndex, JointType jointType, int &x, int &y);		// get the joint position in color image space
	void GetJointPositionInColorSpace(int bodyIndex, JointType jointType, cv::Point &colorPoint);		// get the joint position in color image space

	void GetDepthValueInColorSpace(cv::Point colorPoint, float *depthValue, uchar *depthFrameData);	// get the depth value in color space
	void MapColorSpace2CameraSpace(cv::Point colorPoint, Eigen::Vector3f &cameraPoint, uchar *depthFrameData);

	Joint* GetBodyJoints(int bodyIndex);
	bool GetBodyTrackingState(int bodyIndex);

	int GetPrimaryBodyIndex();
	UINT64 GetPrimaryBodyId();

	// check wether the body gesture is the Specified gesture
	bool CheckBodyGesture(int bodyIndex);

	Vector3f CameraSpacePoint2Vec3(Joint joint);
	Vector3f CameraSpacePoint2Vec3(CameraSpacePoint csp);

	// the body between these are detected
	float AngleThread;

private:
	void UpdatePrimaryBodyId();
	bool CheckBodyPos(Vector3f bodyPos);	// check the body position, whether the body position is in the Hot Spot Area

	// body frame 
	IBodyFrameSource* pSource;
	IBodyFrameReader* pReader;
	IBodyFrame* pFrame;
	// coordinate mapper
	ICoordinateMapper* pCMapper;

	// store the whole bodies information
	IBody** aBody;
	Joint** aAllBodyJoints;		// all users body joints information, 6 users, 25 joints
	bool* bBodyTrackingState;	// indicate the user body tracking state
	UINT64* aBodyIdMap;			// if the user is tracked, set the body id, else set to 0

	// key variable
	INT32 iBodyCount;			// the body tracked ability, the kinect-V2 is 6
	UINT64 iPrimaryBodyId;		// the primary body ID
	int iPrimaryBodyIndex;		// the primary body index, 0-5

	// body position and gesture 
	Eigen::Vector4f vHotSpotArea;			// user in the hot area is the primary body, Vector4f(x_min, x_max, z_min, z_max)
};

#endif