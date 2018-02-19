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

	CUSTOM_JOINTS_API dFloat GetActuatorAngle1() const;

	CUSTOM_JOINTS_API dFloat GetTargetAngle1() const;
	CUSTOM_JOINTS_API void SetTargetAngle1(dFloat angle);

	CUSTOM_JOINTS_API dFloat GetMinAngularLimit1() const;
	CUSTOM_JOINTS_API void SetMinAngularLimit1(dFloat limit);

	CUSTOM_JOINTS_API dFloat GetMaxAngularLimit1() const;
	CUSTOM_JOINTS_API void SetMaxAngularLimit1(dFloat limit);

	CUSTOM_JOINTS_API dFloat GetAngularRate1() const;
	CUSTOM_JOINTS_API void SetAngularRate1(dFloat rate);

	CUSTOM_JOINTS_API dFloat GetMaxTorque1() const;
	CUSTOM_JOINTS_API void SetMaxTorque1(dFloat torque);

	protected:
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Deserialize(NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& eulers, dFloat timestep);

	dAngularIntegration m_targetAngle0;
	dAngularIntegration m_targetAngle1;
	dFloat m_maxTorque0;
	dFloat m_maxTorque1;

	DECLARE_CUSTOM_JOINT(dCustomUniversalActuator, dCustomUniversal)
};

#endif 

