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

	CUSTOM_JOINTS_API void EnableTwist(bool state);
	CUSTOM_JOINTS_API void SetTwistLimits(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetTwistLimits(dFloat& minAngle, dFloat& maxAngle) const;

	CUSTOM_JOINTS_API void EnableCone(bool state);
	CUSTOM_JOINTS_API dFloat GetConeLimits() const;
	CUSTOM_JOINTS_API void SetConeLimits(dFloat maxAngle);

	CUSTOM_JOINTS_API void SetTwistFriction (dFloat frictionTorque);
	CUSTOM_JOINTS_API dFloat GetTwistFriction (dFloat frictionTorque) const;

	CUSTOM_JOINTS_API void SetConeFriction(dFloat frictionTorque);
	CUSTOM_JOINTS_API dFloat GetConeFriction(dFloat frictionTorque) const;

	CUSTOM_JOINTS_API void SetTwistSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper);
	

	protected:
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

		
	void SubmitConstraintTwistLimits(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& relOmega, dFloat timestep);

	dAngularIntegration m_twistAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	dFloat m_maxConeAngle;
	dFloat m_coneFriction;
	dFloat m_twistFriction;
	dOptions m_options;

	DECLARE_CUSTOM_JOINT(dCustomBallAndSocket, dCustomJoint)
};

#if 0
// similar to the ball and socked, plus it has the ability to set joint limits and friction
class dCustomLimitBallAndSocket: public dCustomBallAndSocket  
{
	public:
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dCustomLimitBallAndSocket();

	CUSTOM_JOINTS_API void SetConeAngle (dFloat angle);
	CUSTOM_JOINTS_API void SetTwistAngle (dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetConeAngle () const;
	CUSTOM_JOINTS_API void GetTwistAngle (dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	//CUSTOM_JOINTS_API dCustomLimitBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

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
	//CUSTOM_JOINTS_API dCustomControlledBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
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


/*
// similar to the ball and socked, plus it has the ability to set joint limits and friction
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



*/
#endif

#endif 

