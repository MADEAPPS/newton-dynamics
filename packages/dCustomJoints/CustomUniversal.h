/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomUniversal.h: interface for the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOMUNIVERSAL_H_
#define _CUSTOMUNIVERSAL_H_

#include "CustomJoint.h"

class CustomUniversal: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomUniversal(const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomUniversal();

	CUSTOM_JOINTS_API void EnableLimit_0(bool state);
	CUSTOM_JOINTS_API void EnableLimit_1(bool state);
	CUSTOM_JOINTS_API void SetLimis_0(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void SetLimis_1(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetJointAngle_0 () const;
	CUSTOM_JOINTS_API dFloat GetJointAngle_1 () const;

	CUSTOM_JOINTS_API dVector GetPinAxis_0 () const;
	CUSTOM_JOINTS_API dVector GetPinAxis_1 () const;

	CUSTOM_JOINTS_API dFloat GetJointOmega_0 () const;
	CUSTOM_JOINTS_API dFloat GetJointOmega_1 () const;

	CUSTOM_JOINTS_API void EnableMotor_0(bool state);
	CUSTOM_JOINTS_API void EnableMotor_1(bool state);

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const {dAssert (0);} 

	bool m_limit_0_On;
	bool m_limit_1_On;
	bool m_angularMotor_0_On;
	bool m_angularMotor_1_On;

	dFloat m_minAngle_0;
	dFloat m_maxAngle_0;
	dFloat m_jointOmega_0;
	dFloat m_angularDamp_0;
	dFloat m_angularAccel_0;
	AngularIntegration m_curJointAngle_0;

	dFloat m_minAngle_1;
	dFloat m_maxAngle_1;
	dFloat m_jointOmega_1;
	dFloat m_angularDamp_1;
	dFloat m_angularAccel_1;
	AngularIntegration m_curJointAngle_1;
};

#endif 

