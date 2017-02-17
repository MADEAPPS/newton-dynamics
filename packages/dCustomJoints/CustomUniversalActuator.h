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


// CustomUniversalActuator.h: interface for the CustomUniversalActuator class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_UNIVERSAL_ACTUATOR_H_
#define _CUSTOM_UNIVERSAL_ACTUATOR_H_

#include "CustomUniversal.h"

class CustomUniversalActuator: public CustomUniversal
{
	public:
	CUSTOM_JOINTS_API CustomUniversalActuator (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API CustomUniversalActuator (const dMatrix& pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomUniversalActuator();

	CUSTOM_JOINTS_API bool GetEnableFlag0 () const;
	CUSTOM_JOINTS_API void SetEnableFlag0 (bool flag);

	CUSTOM_JOINTS_API dFloat GetActuatorAngle0() const;
	CUSTOM_JOINTS_API dFloat GetTargetAngle0() const;
	CUSTOM_JOINTS_API void SetTargetAngle0(dFloat angle);

	CUSTOM_JOINTS_API dFloat GetAngularRate0() const;
	CUSTOM_JOINTS_API void SetAngularRate0(dFloat rate);

	CUSTOM_JOINTS_API dFloat GetMinAngularLimit0() const;
	CUSTOM_JOINTS_API void SetMinAngularLimit0(dFloat limit);

	CUSTOM_JOINTS_API dFloat GetMaxAngularLimit0() const;
	CUSTOM_JOINTS_API void SetMaxAngularLimit0(dFloat limit);

    CUSTOM_JOINTS_API dFloat GetMaxTorquePower0() const;
    CUSTOM_JOINTS_API void SetMaxTorquePower0(dFloat force);

	CUSTOM_JOINTS_API bool GetEnableFlag1 () const;
	CUSTOM_JOINTS_API void SetEnableFlag1 (bool flag);

	CUSTOM_JOINTS_API dFloat GetActuatorAngle1() const;
	CUSTOM_JOINTS_API dFloat GetTargetAngle1() const;
	CUSTOM_JOINTS_API void SetTargetAngle1(dFloat angle);

	CUSTOM_JOINTS_API dFloat GetAngularRate1() const;
	CUSTOM_JOINTS_API void SetAngularRate1(dFloat rate);

	CUSTOM_JOINTS_API dFloat GetMinAngularLimit1() const;
	CUSTOM_JOINTS_API void SetMinAngularLimit1(dFloat limit);

	CUSTOM_JOINTS_API dFloat GetMaxAngularLimit1() const;
	CUSTOM_JOINTS_API void SetMaxAngularLimit1(dFloat limit);

    CUSTOM_JOINTS_API dFloat GetMaxTorquePower1() const;
    CUSTOM_JOINTS_API void SetMaxTorquePower1(dFloat force);

	protected:
	CUSTOM_JOINTS_API CustomUniversalActuator(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	dFloat m_angle0;
	dFloat m_minAngle0;
	dFloat m_maxAngle0;
	dFloat m_angularRate0;
    dFloat m_maxForce0;

	dFloat m_angle1;
	dFloat m_minAngle1;
	dFloat m_maxAngle1;
	dFloat m_angularRate1;
    dFloat m_maxForce1;

	bool m_flag0;
	bool m_flag1;
};

#endif 

