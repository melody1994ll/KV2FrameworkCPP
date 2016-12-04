#include "BodyFrame.h"

template <class Interface>
inline void SafeRelease(Interface*& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}


CBodyFrame::CBodyFrame()
{
	pSource = nullptr;
	pReader = nullptr;
	pFrame = nullptr;
	pCMapper = nullptr;

	iBodyCount = 0;
	iPrimaryBodyId = BODYFRAME_NO_TRACKED_ID;
	iPrimaryBodyIndex = BODYFRAME_NO_TRACKED_INDEX;

	vHotSpotArea = Eigen::Vector4f(0, 0, 0, 0);

	AngleThread = -0.8f;
}

// initial working distance variable


CBodyFrame::~CBodyFrame()
{
	if (pFrame)
		SafeRelease(pFrame);

	if (pReader)
		SafeRelease(pReader);

	if (pSource)
		SafeRelease(pSource);
	
	if (aBody)
		delete[] aBody;

	if (aAllBodyJoints)
		delete[] aAllBodyJoints;

	if (bBodyTrackingState)
		delete[] bBodyTrackingState;
}

bool CBodyFrame::Initial(IKinectSensor* pSensor)
{
	if (pSensor == NULL)
		return false;

	if (pSensor->get_BodyFrameSource(&pSource) != S_OK)
		return false;

	// initial body frame value
	pSource->get_BodyCount(&iBodyCount);
	aBody = new IBody*[iBodyCount];
	aAllBodyJoints = new Joint*[iBodyCount];
	bBodyTrackingState = new bool[iBodyCount];
	aBodyIdMap = new UINT64[iBodyCount];
	
	for (int i = 0; i < iBodyCount; i++)
	{
		aBody[i] = nullptr;
		aAllBodyJoints[i] = new Joint[JointType::JointType_Count];
		bBodyTrackingState[i] = false;
		aBodyIdMap[i] = BODYFRAME_NO_TRACKED_ID;
	}

	if (pSensor->get_CoordinateMapper(&pCMapper) != S_OK)
		return false;
		
	if (pSource->OpenReader(&pReader) != S_OK)
		return false;

	SafeRelease(pSource);

	// read the hot spot area value from .yml file
	cv::FileStorage fs("HotSpotArea.yml", cv::FileStorage::READ);
	fs["min_x"] >> vHotSpotArea[0];
	fs["max_x"] >> vHotSpotArea[1];
	fs["min_z"] >> vHotSpotArea[2];
	fs["max_z"] >> vHotSpotArea[3];
	fs.release();

	return true;
}

bool CBodyFrame::UpdateFrame(int& iBodyTracked)
{
	iBodyTracked = 0;
	pFrame = nullptr;

	if (pReader->AcquireLatestFrame(&pFrame) != S_OK)
		return false;
	else
	{
		if (pFrame->GetAndRefreshBodyData(iBodyCount, aBody) != S_OK)
			return false;
		else
		{
			for (int i = 0; i < iBodyCount; i++)
			{
				BOOLEAN bTrackedState = false;

				IBody* pBody = aBody[i];
				
				if (pBody->get_IsTracked(&bTrackedState) == S_OK && bTrackedState)
				{
					// add the tracked body number
					iBodyTracked++;

					// update joint data
					pBody->GetJoints(JointType::JointType_Count, aAllBodyJoints[i]);

					// set body tracking state
					bBodyTrackingState[i] = true;

					// update body id
					UINT64 iBodyId = 0;
					pBody->get_TrackingId(&iBodyId);
					aBodyIdMap[i] = iBodyId;
				}
				else
				{
					bBodyTrackingState[i] = false;
					aBodyIdMap[i] = BODYFRAME_NO_TRACKED_ID;
				}
			}

			// search for primary user id
			UpdatePrimaryBodyId();
		}

		pFrame->Release();
	}

	return true;
}

void CBodyFrame::UpdatePrimaryBodyId()
{
	// reset primary body id and index
	iPrimaryBodyId = BODYFRAME_NO_TRACKED_ID;
	iPrimaryBodyIndex = BODYFRAME_NO_TRACKED_INDEX;

	// the closest body in the hot spot area
	Eigen::Vector3f closedBodyPos(0,0,0);

	for (int i = 0; i < iBodyCount; i++)
	{
		if (bBodyTrackingState[i])
		{
			// joints for user body postion
			Joint spineShoulder = aAllBodyJoints[i][JointType::JointType_SpineShoulder];
			Joint spineMid = aAllBodyJoints[i][JointType::JointType_SpineBase];
			Joint shoulderLeft = aAllBodyJoints[i][JointType::JointType_ShoulderLeft];
			Joint shoulderRight = aAllBodyJoints[i][JointType::JointType_ShoulderRight];

			Vector3f spinePos(0, 0, 0);
			Vector3f shoulderCenterPos(0, 0, 0);
			Vector3f bodyPos(0, 0, 0);

			// update spine pos
			if (spineShoulder.TrackingState != TrackingState_NotTracked && spineMid.TrackingState != TrackingState_NotTracked)
			{
				spinePos = (CameraSpacePoint2Vec3(spineShoulder) + CameraSpacePoint2Vec3(spineMid)) / 2;
				bodyPos = spinePos;
			}
			// update shoulder center pos
			if (shoulderLeft.TrackingState != TrackingState_NotTracked && shoulderRight.TrackingState != TrackingState_NotTracked)
			{
				shoulderCenterPos = (CameraSpacePoint2Vec3(shoulderLeft) + CameraSpacePoint2Vec3(shoulderRight)) / 2;
				bodyPos = (bodyPos + shoulderCenterPos) / 2;
			}

			// check the body position 
			if (CheckBodyPos(bodyPos) && bodyPos[2]>closedBodyPos[2])
			{
				closedBodyPos = bodyPos;
				iPrimaryBodyIndex = i;
				iPrimaryBodyId = aBodyIdMap[i];
			}
		}
	}
}

bool CBodyFrame::CheckBodyGesture(int bodyIndex)
{
	// if the body is tracked
	if (bodyIndex!=BODYFRAME_NO_TRACKED_INDEX && bBodyTrackingState[bodyIndex])
	{
		// joints for gesture and area detection
		// joints use position
		Joint elbowLeft = aAllBodyJoints[bodyIndex][JointType::JointType_ElbowLeft];
		Joint handLeft = aAllBodyJoints[bodyIndex][JointType::JointType_HandLeft];
		Joint elbowRight = aAllBodyJoints[bodyIndex][JointType::JointType_ElbowRight];
		Joint handRight = aAllBodyJoints[bodyIndex][JointType::JointType_HandRight];
		Joint spineMid = aAllBodyJoints[bodyIndex][JointType::JointType_SpineBase];
		// joints only use state
		Joint shoulderLeft = aAllBodyJoints[bodyIndex][JointType::JointType_ShoulderLeft];
		Joint shoulderRight = aAllBodyJoints[bodyIndex][JointType::JointType_ShoulderRight];
		Joint spineShoulder = aAllBodyJoints[bodyIndex][JointType::JointType_SpineShoulder];
		Joint spineBase = aAllBodyJoints[bodyIndex][JointType::JointType_SpineBase];
		Joint hipLeft = aAllBodyJoints[bodyIndex][JointType::JointType_HipLeft];
		Joint ankleLeft = aAllBodyJoints[bodyIndex][JointType::JointType_AnkleLeft];
		Joint footLeft = aAllBodyJoints[bodyIndex][JointType::JointType_FootLeft];
		Joint hipRight = aAllBodyJoints[bodyIndex][JointType::JointType_HipRight];
		Joint ankleRight = aAllBodyJoints[bodyIndex][JointType::JointType_AnkleRight];
		Joint footRight = aAllBodyJoints[bodyIndex][JointType::JointType_FootRight];

		// check the joint state
		if (elbowLeft.TrackingState != TrackingState_NotTracked && handLeft.TrackingState != TrackingState_NotTracked
			&& elbowRight.TrackingState != TrackingState_NotTracked && handRight.TrackingState != TrackingState_NotTracked
			&& spineMid.TrackingState != TrackingState_NotTracked && shoulderLeft.TrackingState != TrackingState_NotTracked
			&& shoulderRight.TrackingState != TrackingState_NotTracked && spineShoulder.TrackingState != TrackingState_NotTracked
			&& spineBase.TrackingState != TrackingState_NotTracked && hipLeft.TrackingState != TrackingState_NotTracked
			&& ankleLeft.TrackingState != TrackingState_NotTracked && footLeft.TrackingState != TrackingState_NotTracked
			&& hipRight.TrackingState != TrackingState_NotTracked && ankleRight.TrackingState != TrackingState_NotTracked
			&& footRight.TrackingState != TrackingState_NotTracked)
		{
			// get key joints position
			Vector3f elbowLeftPos = CameraSpacePoint2Vec3(elbowLeft);
			Vector3f handLeftPos = CameraSpacePoint2Vec3(handLeft);
			Vector3f elbowRightPos = CameraSpacePoint2Vec3(elbowRight);
			Vector3f handRightPos = CameraSpacePoint2Vec3(handRight);
			Vector3f spineMidPos = CameraSpacePoint2Vec3(spineMid);

			// arms direstion
			Vector3f elbow2handVec_left = elbowLeftPos - handLeftPos;
			Vector3f elbow2handVec_right = elbowRightPos - handRightPos;
			// normalize vector
			elbow2handVec_left.normalize();
			elbow2handVec_right.normalize();

			if (elbowLeftPos[1] > spineMidPos[1] && handLeftPos[1] > spineMidPos[1]
				&& elbowRightPos[1] > spineMidPos[1] && handRightPos[1] > spineMidPos[1])
			{
				// calculate the angle between two arms
				float angleBetweenArms = elbow2handVec_left.dot(elbow2handVec_right);

				printf("two arms angle: %f \n", angleBetweenArms);

				// set the primary body value, the two arm diretion is opposite
				if (angleBetweenArms > -1.2 && angleBetweenArms < AngleThread)
					return true;
			}
		}
		
	}

	return false;
}

void CBodyFrame::DrawBodyLine(cv::Mat& colorMap, UINT64 userId)
{
	for (int i = 0; i < iBodyCount; i++)
	{
		if (bBodyTrackingState[i])
		{
			// get joints array
			// draw the line between this joint and parent joint
			for (int iJointIndex = 0; iJointIndex < JointType::JointType_Count; iJointIndex++)
			{
				Joint thisJoint = aAllBodyJoints[i][iJointIndex];
				// this joint's parent joint
				Joint parentJoint = aAllBodyJoints[i][GetParentJoint(thisJoint.JointType)];

				if (thisJoint.TrackingState != TrackingState_NotTracked)
				{
					// draw a color circle at joint position
					ColorSpacePoint CSP1;
					pCMapper->MapCameraPointToColorSpace(thisJoint.Position, &CSP1);

					if (thisJoint.TrackingState == TrackingState_Tracked)
						cv::circle(colorMap, cv::Point(CSP1.X, CSP1.Y), 20, cv::Scalar(0, 255, 0), -1);
					else
						cv::circle(colorMap, cv::Point(CSP1.X, CSP1.Y), 20, cv::Scalar(255, 0, 0), -1);

					// draw a line between two adjacency joints
					if (parentJoint.TrackingState != TrackingState_NotTracked)
					{
						ColorSpacePoint CSP2;
						pCMapper->MapCameraPointToColorSpace(parentJoint.Position, &CSP2);

						cv::line(colorMap, cv::Point(CSP1.X, CSP1.Y), cv::Point(CSP2.X, CSP2.Y), cv::Scalar(0, 0, 255), 5, 4);
					}
				}
			}
		}
	}
}

JointType CBodyFrame::GetParentJoint(JointType childJoint)
{
	switch (childJoint)
	{
	case JointType_SpineBase:
		return JointType_SpineBase;

	case JointType_Neck:
		return JointType_SpineShoulder;

	case JointType_ShoulderLeft:
		return JointType_SpineShoulder;

	case JointType_ShoulderRight:
		return JointType_SpineShoulder;

	case JointType_HipLeft:
		return JointType_SpineBase;

	case JointType_HipRight:
		return JointType_SpineBase;

	case JointType_SpineShoulder:
		return JointType_SpineMid;

	case JointType_HandTipLeft:
		return JointType_HandLeft;

	case JointType_ThumbLeft:
		return JointType_WristLeft;

	case JointType_HandTipRight:
		return JointType_HandRight;

	case JointType_ThumbRight:
		return JointType_WristRight;
	default:
		break;
	}

	return JointType(childJoint - 1);
}

void CBodyFrame::GetJointPosition(int bodyIndex, JointType jointType, float& x, float& y, float& z)
{
	if (BODYFRAME_NO_TRACKED_INDEX != bodyIndex)
	{
		Joint joint = aAllBodyJoints[bodyIndex][jointType];
		x = joint.Position.X;
		y = joint.Position.Y;
		z = joint.Position.Z;
	}
}

void CBodyFrame::GetJointPosition(int bodyIndex, JointType jointType, Vector3f& jointPos)
{
	GetJointPosition(bodyIndex, jointType, jointPos[0], jointPos[1], jointPos[2]);
}

void CBodyFrame::GetJointPositionInColorSpace(int bodyIndex, JointType jointType, cv::Point &colorPoint)
{
	GetJointPositionInColorSpace(bodyIndex, jointType, colorPoint.x, colorPoint.y);
}

void CBodyFrame::GetJointPositionInColorSpace(int bodyIndex, JointType jointType, int &x, int &y)
{
	if (bodyIndex != BODYFRAME_NO_TRACKED_INDEX)
	{
		ColorSpacePoint cp;
		Joint thisJoint = aAllBodyJoints[bodyIndex][jointType];
		pCMapper->MapCameraPointToColorSpace(thisJoint.Position, &cp);

		x = cp.X;
		y = cp.Y;
	}
	else
		x = y = 0;
}

int CBodyFrame::GetPrimaryBodyIndex()
{
	return iPrimaryBodyIndex;
}

UINT64 CBodyFrame::GetPrimaryBodyId()
{
	return iPrimaryBodyId;
}

Vector3f CBodyFrame::CameraSpacePoint2Vec3(Joint joint)
{
	Vector3f v3(joint.Position.X, joint.Position.Y, joint.Position.Z);

	return v3;
}

Vector3f CBodyFrame::CameraSpacePoint2Vec3(CameraSpacePoint csp)
{
	Vector3f v3(csp.X, csp.Y, csp.Z);

	return v3;
}

bool CBodyFrame::CheckBodyPos(Eigen::Vector3f bodyPos)
{
	// TODO: compare the bodyPos to vHotSpotArea, 
	//		 if the bodyPos is in the Hot Spot Area, return true, else return false
	if (bodyPos[0] >= vHotSpotArea[0] && bodyPos[0] <= vHotSpotArea[1]
		&& bodyPos[2] >= vHotSpotArea[2] && bodyPos[2] <= vHotSpotArea[3])
		return true;
	else
		return false;
}

// TODO: optimize the depth value, 
//       not all the color point has the depth value
void CBodyFrame::GetDepthValueInColorSpace(cv::Point colorPoint, float *depthValue, uchar *depthFrameData)
{
	DepthSpacePoint *p = new DepthSpacePoint[1920 * 1080];

	pCMapper->MapColorFrameToDepthSpace(512 * 424, (UINT16*)(depthFrameData), 1920 * 1080, p);

	int depthX = (int)(p[colorPoint.x + colorPoint.y * 1920].X+0.5f);
	int depthY = (int)(p[colorPoint.x + colorPoint.y * 1920].Y+0.5f);

	if (depthX >= 0 && depthX < 515 && depthY >= 0 && depthY < 424)
		*depthValue = *(depthFrameData + (int)(depthX + depthY * 512));
	else
		*depthValue = 0;

	delete[] p;
}

void CBodyFrame::MapColorSpace2CameraSpace(cv::Point colorPoint, Eigen::Vector3f &cameraPoint, uchar *depthFrameData)
{
	CameraSpacePoint *p = new CameraSpacePoint[1920 * 1080];

	pCMapper->MapColorFrameToCameraSpace(515 * 424, (UINT16*)(depthFrameData), 1920 * 1080, p);

	long colorIndex = (long)(colorPoint.x + colorPoint.y * 1920);
	CameraSpacePoint csp = p[colorIndex];
	
	cameraPoint = CameraSpacePoint2Vec3(csp);

	delete[] p;
}

Joint* CBodyFrame::GetBodyJoints(int bodyIndex)
{
	if (bodyIndex != BODYFRAME_NO_TRACKED_INDEX && aBodyIdMap[bodyIndex])
	{
		return aAllBodyJoints[bodyIndex];
	}

	return nullptr;
}

bool CBodyFrame::GetBodyTrackingState(int bodyIndex)
{
	return bBodyTrackingState[bodyIndex];
}