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


// CustomUniversal.h: interface for the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOMUNIVERSAL_H_
#define _CUSTOMUNIVERSAL_H_

#include "dCustomJoint.h"

class CustomUniversal: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomUniversal(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomUniversal();

	CUSTOM_JOINTS_API void EnableLimit_0(bool state);
	CUSTOM_JOINTS_API void EnableLimit_1(bool state);
	CUSTOM_JOINTS_API void SetLimits_0(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void SetLimits_1(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetMinAngularLimit_0() const;
	CUSTOM_JOINTS_API dFloat GetMinAngularLimit_1() const;

	CUSTOM_JOINTS_API dFloat GetMaxAngularLimit_0() const;
	CUSTOM_JOINTS_API dFloat GetMaxAngularLimit_1() const;

	CUSTOM_JOINTS_API dFloat GetJointAngle_0 () const;
	CUSTOM_JOINTS_API dFloat GetJointAngle_1 () const;

	CUSTOM_JOINTS_API dVector GetPinAxis_0 () const;
	CUSTOM_JOINTS_API dVector GetPinAxis_1 () const;

	CUSTOM_JOINTS_API dFloat GetJointOmega_0 () const;
	CUSTOM_JOINTS_API dFloat GetJointOmega_1 () const;

	CUSTOM_JOINTS_API void EnableMotor_0(bool state);
	CUSTOM_JOINTS_API void EnableMotor_1(bool state);

	CUSTOM_JOINTS_API void SetAccel_0(dFloat accel);
	CUSTOM_JOINTS_API void SetDamp_0(dFloat damp);

	CUSTOM_JOINTS_API void SetAccel_1(dFloat accel);
	CUSTOM_JOINTS_API void SetDamp_1(dFloat damp);

	protected:
	CUSTOM_JOINTS_API CustomUniversal(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	AngularIntegration m_curJointAngle_0;
	AngularIntegration m_curJointAngle_1;

	dFloat m_minAngle_0;
	dFloat m_maxAngle_0;
	dFloat m_jointOmega_0;
	dFloat m_angularDamp_0;
	dFloat m_angularAccel_0;

	dFloat m_minAngle_1;
	dFloat m_maxAngle_1;
	dFloat m_jointOmega_1;
	dFloat m_angularDamp_1;
	dFloat m_angularAccel_1;

	union {
		int m_flags;
		struct {
			unsigned m_limit_0_On		 : 1;
			unsigned m_limit_1_On		 : 1;
			unsigned m_angularMotor_0_On : 1;
			unsigned m_angularMotor_1_On : 1;
			unsigned m_actuator_0		 : 1;
			unsigned m_actuator_1		 : 1;
		};
	};

	DECLARE_CUSTOM_JOINT(CustomUniversal, dCustomJoint)
};

#endif 

