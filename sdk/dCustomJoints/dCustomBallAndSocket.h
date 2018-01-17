/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dCustomBallAndSocket.h: interface for the dCustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _D_CUSTOM_BALLANDSOCKET_H_
#define _D_CUSTOM_BALLANDSOCKET_H_

#include "dCustom6dof.h"
//#include "dCustomJoint.h"

class dCustomPointToPoint: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomPointToPoint(const dVector& pivotFrame0, const dVector& pivotFrame1, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomPointToPoint();

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_distance;
	DECLARE_CUSTOM_JOINT(dCustomPointToPoint, dCustomJoint)
};


class dCustomBallAndSocket: public dCustom6dof
{
	public:
	CUSTOM_JOINTS_API dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomBallAndSocket(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	DECLARE_CUSTOM_JOINT(dCustomBallAndSocket, dCustom6dof)
};


// similar to the ball and socked 
// plus it has the ability to set joint limits
class dCustomLimitBallAndSocket: public dCustomBallAndSocket  
{
	public:
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomLimitBallAndSocket();

	CUSTOM_JOINTS_API void SetConeAngle (dFloat angle);
	CUSTOM_JOINTS_API void SetTwistAngle (dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetConeAngle () const;
	CUSTOM_JOINTS_API void GetTwistAngle (dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	//CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void SubmitConstraintsFreeDof(int freeDof, const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep, int threadIndex);

	dMatrix m_rotationOffset;
	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	dFloat m_coneAngleCos;
	dFloat m_coneAngleSin;
	dFloat m_coneAngleHalfCos;
	dFloat m_coneAngleHalfSin;	
	DECLARE_CUSTOM_JOINT(dCustomLimitBallAndSocket, dCustomBallAndSocket)
};


class dCustomBallAndSocketWithFriction: public dCustomBallAndSocket
{
	public:
	CUSTOM_JOINTS_API dCustomBallAndSocketWithFriction(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat dryFriction);
	CUSTOM_JOINTS_API void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_dryFriction;
};


class dCustomControlledBallAndSocket: public dCustomBallAndSocket  
{
	public:
	CUSTOM_JOINTS_API dCustomControlledBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomControlledBallAndSocket();

	CUSTOM_JOINTS_API void SetAngularVelocity (dFloat omegaMag);
	CUSTOM_JOINTS_API dFloat GetAngularVelocity () const;

	CUSTOM_JOINTS_API void SetPitchAngle (dFloat angle);
	CUSTOM_JOINTS_API dFloat SetPitchAngle () const;

	CUSTOM_JOINTS_API void SetYawAngle (dFloat angle);
	CUSTOM_JOINTS_API dFloat SetYawAngle () const;

	CUSTOM_JOINTS_API void SetRollAngle (dFloat angle);
	CUSTOM_JOINTS_API dFloat SetRollAngle () const;

	protected:
	void UpdateTargetMatrix ();
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	dVector m_targetAngles;
	dMatrix m_targetRotation;
	dAngularIntegration m_pitch;
	dAngularIntegration m_yaw;
	dAngularIntegration m_roll;
	dFloat m_angulaSpeed;
	DECLARE_CUSTOM_JOINT(dCustomControlledBallAndSocket, dCustomBallAndSocket)
};


#endif 

