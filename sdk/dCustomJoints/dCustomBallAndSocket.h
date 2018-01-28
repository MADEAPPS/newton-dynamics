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

#include "dCustomJoint.h"

class dCustomBallAndSocket: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomBallAndSocket();

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	void SubmitLinearRows (const dMatrix& matrix0, const dMatrix& matrix1) const;
	DECLARE_CUSTOM_JOINT(dCustomBallAndSocket, dCustomJoint)
};

// similar to the ball and socked, plus it has the ability to set joint limits
class dCustomLimitBallAndSocket: public dCustomBallAndSocket  
{
	public:
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomLimitBallAndSocket();

	CUSTOM_JOINTS_API void SetFriction (dFloat angle);
	CUSTOM_JOINTS_API void SetConeAngle (dFloat angle);
	CUSTOM_JOINTS_API void SetTwistAngle (dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetFriction () const;
	CUSTOM_JOINTS_API dFloat GetConeAngle () const;
	CUSTOM_JOINTS_API void GetTwistAngle (dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	dFloat m_friction;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	dFloat m_coneAngle;
	dFloat m_coneAngleCos;
	DECLARE_CUSTOM_JOINT(dCustomLimitBallAndSocket, dCustomBallAndSocket)
};


#endif 

