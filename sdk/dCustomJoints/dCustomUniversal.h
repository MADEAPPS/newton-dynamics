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


// dCustomUniversal.h: interface for the dCustomUniversal class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOMUNIVERSAL_H_
#define _CUSTOMUNIVERSAL_H_

#include "dCustomHinge.h"

class dCustomUniversal: public dCustomHinge
{
	public:
	CUSTOM_JOINTS_API dCustomUniversal(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomUniversal(const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomUniversal();

	CUSTOM_JOINTS_API void EnableLimits2(bool state);
	CUSTOM_JOINTS_API void SetLimits2(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API dFloat GetJointAngle2() const;
	CUSTOM_JOINTS_API dVector GetPinAxis2() const;
	CUSTOM_JOINTS_API dFloat GetJointOmega2() const;
	CUSTOM_JOINTS_API dFloat GetFriction2() const;
	CUSTOM_JOINTS_API void SetFriction2(dFloat frictionTorque);
	CUSTOM_JOINTS_API void SetAsSpringDamper2(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper);

	CUSTOM_JOINTS_API void SetHardMiddleAxis(bool state);

	protected:
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;
	CUSTOM_JOINTS_API virtual void Deserialize(NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& eulers, dFloat timestep);

	void SubmitConstraintLimits(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);
	void SubmitConstraintSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);
	void SubmitConstraintLimitSpringDamper(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);

	dAngularIntegration m_curJointAngle2;
	dFloat m_minAngle2;
	dFloat m_maxAngle2;
	dFloat m_friction2;
	dFloat m_jointOmega2;
	dFloat m_spring2;
	dFloat m_damper2;
	dFloat m_springDamperRelaxation2;

	DECLARE_CUSTOM_JOINT(dCustomUniversal, dCustomHinge)
};



#endif 

