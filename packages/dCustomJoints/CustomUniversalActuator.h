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


// CustomUniversalActuator.h: interface for the CustomUniversalActuator class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_UNIVERSAL_ACTUATOR_H_
#define _CUSTOM_UNIVERSAL_ACTUATOR_H_

#include "CustomUniversal.h"

class CustomUniversalActuator: public CustomUniversal
{
	public:
	NEWTON_API CustomUniversalActuator (const dMatrix& pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, NewtonBody* const child, NewtonBody* const parent = NULL);
	NEWTON_API virtual ~CustomUniversalActuator();

	NEWTON_API bool GetEnableFlag0 (bool flag) const;
	NEWTON_API void SetEnableFlag0 (bool flag);

	NEWTON_API dFloat GetActuatorAngle0() const;
	NEWTON_API dFloat GetTargetAngle0() const;
	NEWTON_API void SetTargetAngle0(dFloat angle);

	NEWTON_API dFloat GetAngularRate0() const;
	NEWTON_API void SetAngularRate0(dFloat rate);

	NEWTON_API dFloat GetMinAngularLimit0() const;
	NEWTON_API void SetMinAngularLimit0(dFloat limit);

	NEWTON_API dFloat GetMaxAngularLimit0() const;
	NEWTON_API void SetMaxAngularLimit0(dFloat limit);


	NEWTON_API bool GetEnableFlag1 (bool flag) const;
	NEWTON_API void SetEnableFlag1 (bool flag);

	NEWTON_API dFloat GetActuatorAngle1() const;
	NEWTON_API dFloat GetTargetAngle1() const;
	NEWTON_API void SetTargetAngle1(dFloat angle);

	NEWTON_API dFloat GetAngularRate1() const;
	NEWTON_API void SetAngularRate1(dFloat rate);

	NEWTON_API dFloat GetMinAngularLimit1() const;
	NEWTON_API void SetMinAngularLimit1(dFloat limit);

	NEWTON_API dFloat GetMaxAngularLimit1() const;
	NEWTON_API void SetMaxAngularLimit1(dFloat limit);


	protected:
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_angle0;
	dFloat m_minAngle0;
	dFloat m_maxAngle0;
	dFloat m_angularRate0;

	dFloat m_angle1;
	dFloat m_minAngle1;
	dFloat m_maxAngle1;
	dFloat m_angularRate1;

	bool m_flag0;
	bool m_flag1;
};

#endif 

