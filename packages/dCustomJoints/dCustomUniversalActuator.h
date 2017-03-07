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


// dCustomUniversalActuator.h: interface for the dCustomUniversalActuator class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_UNIVERSAL_ACTUATOR_H_
#define _CUSTOM_UNIVERSAL_ACTUATOR_H_

#include "dCustomUniversal.h"

class dCustomUniversalActuator: public dCustomUniversal
{
	public:
	CUSTOM_JOINTS_API dCustomUniversalActuator (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomUniversalActuator (const dMatrix& pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomUniversalActuator();

	CUSTOM_JOINTS_API bool GetEnableFlag0 () const;
	CUSTOM_JOINTS_API void SetEnableFlag0 (bool flag);

	CUSTOM_JOINTS_API dFloat GetTargetAngle0() const;
	CUSTOM_JOINTS_API void SetTargetAngle0(dFloat angle);

	CUSTOM_JOINTS_API dFloat GetAngularRate0() const;
	CUSTOM_JOINTS_API void SetAngularRate0(dFloat rate);

    CUSTOM_JOINTS_API dFloat GetMaxTorquePower0() const;
    CUSTOM_JOINTS_API void SetMaxTorquePower0(dFloat force);

	CUSTOM_JOINTS_API bool GetEnableFlag1 () const;
	CUSTOM_JOINTS_API void SetEnableFlag1 (bool flag);

	CUSTOM_JOINTS_API dFloat GetTargetAngle1() const;
	CUSTOM_JOINTS_API void SetTargetAngle1(dFloat angle);

	CUSTOM_JOINTS_API dFloat GetAngularRate1() const;
	CUSTOM_JOINTS_API void SetAngularRate1(dFloat rate);

    CUSTOM_JOINTS_API dFloat GetMaxTorquePower1() const;
    CUSTOM_JOINTS_API void SetMaxTorquePower1(dFloat force);

	protected:
	CUSTOM_JOINTS_API dCustomUniversalActuator(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	dFloat m_angle0;
	dFloat m_angularRate0;
    dFloat m_maxForce0;

	dFloat m_angle1;
	dFloat m_angularRate1;
    dFloat m_maxForce1;
};

#endif 

