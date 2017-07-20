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

#ifndef _D_CUSTOM_RAG_DOLL_MOTOR_H_
#define _D_CUSTOM_RAG_DOLL_MOTOR_H_

#include "dCustomJoint.h"
#include "dCustomBallAndSocket.h"


// this joint is for controlling rag dolls muscles
class dCustomRagdollMotor: public dCustomBallAndSocket
{
	public:
	class dSaveLoad
	{
		public:
		dSaveLoad() {}
		virtual ~dSaveLoad() {}
		virtual const char* GetBodyUniqueName(const NewtonBody* const body) const = 0;

		virtual NewtonBody* Load(const char* const name);
		virtual void Save(const char* const name, NewtonBody* const roobody);
	};


	CUSTOM_JOINTS_API dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);
//	CUSTOM_JOINTS_API dCustomRagdollMotor(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dCustomRagdollMotor();

	CUSTOM_JOINTS_API void SetYawAngles(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetYawAngles(dFloat& minAngle, dFloat& maxAngle) const;

	CUSTOM_JOINTS_API void SetRollAngles(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetRollAngles(dFloat& minAngle, dFloat& maxAngle) const;

	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	CUSTOM_JOINTS_API dFloat GetJointTorque() const;
	CUSTOM_JOINTS_API void SetJointTorque(dFloat torque);
	
	protected:
	CUSTOM_JOINTS_API dCustomRagdollMotor(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	
	private:
	virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	
	void Submit1DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);
	void Submit2DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);
	void Submit3DOFConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);

	dFloat m_torque;
	dFloat m_minYaw;
	dFloat m_maxYaw;
	dFloat m_minRoll;
	dFloat m_maxRoll;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	bool m_motorMode;

	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor, dCustomBallAndSocket)
};


#endif 

