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

#ifndef _CUSTOMBALLANDSOCKET_H_
#define _CUSTOMBALLANDSOCKET_H_

#include "dCustomJoint.h"


class dCustomPointToPoint: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomPointToPoint(const dVector& pivotFrame0, const dVector& pivotFrame1, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomPointToPoint();

	protected:
	CUSTOM_JOINTS_API dCustomPointToPoint(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo(NewtonJointRecord* const info) const;

	dFloat m_distance;
	DECLARE_CUSTOM_JOINT(dCustomPointToPoint, dCustomJoint)
};


class dCustomBallAndSocket: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomBallAndSocket(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomBallAndSocket();

	protected:
	CUSTOM_JOINTS_API dCustomBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	DECLARE_CUSTOM_JOINT(dCustomBallAndSocket, dCustomJoint)
};


// similar to the ball and socked 
// plus it has the ability to set joint limits
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
	CUSTOM_JOINTS_API dCustomLimitBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

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
	CUSTOM_JOINTS_API dCustomControlledBallAndSocket (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API void GetInfo (NewtonJointRecord* const info) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	dVector m_targetAngles;
	dMatrix m_targetRotation;
	dAngularIntegration m_pitch;
	dAngularIntegration m_yaw;
	dAngularIntegration m_roll;
	dFloat m_angulaSpeed;
	DECLARE_CUSTOM_JOINT(dCustomControlledBallAndSocket, dCustomBallAndSocket)
};



// this joint is for controlling rag dolls muscles
class dCustomRagdollMotor: public dCustomBallAndSocket
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomRagdollMotor(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dCustomRagdollMotor();

	CUSTOM_JOINTS_API void SetConeAngle(dFloat angle);
	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetConeAngle() const;
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	CUSTOM_JOINTS_API void SetJointTorque(dFloat torque);
	CUSTOM_JOINTS_API dFloat GetJointTorque() const;
	
	protected:
	CUSTOM_JOINTS_API dCustomRagdollMotor(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	
	private:
	virtual void GetInfo(NewtonJointRecord* const info) const;
	virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	
	void Submit1DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);
	void Submit2DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);
	void Submit3DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);

	dMatrix m_rotationOffset;
	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	dFloat m_coneAngleCos;
	dFloat m_coneAngleSin;
	dFloat m_coneAngleHalfCos;
	dFloat m_coneAngleHalfSin;
	dFloat m_jointTorque;
	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor, dCustomBallAndSocket)
};


#endif 

