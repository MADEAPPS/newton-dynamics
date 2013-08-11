/* Copyright (c) <2003-2013> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/


#ifndef _CUSTOM_HINGE_ACTUATOR_H_
#define _CUSTOM_HINGE_ACTUATOR_H_

#include "CustomJoint.h"
#include "CustomHinge.h"

class CustomHingeActuator: public CustomHinge
{
	public:
	NEWTON_API CustomHingeActuator(const dMatrix& pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, NewtonBody* const child, NewtonBody* const parent = NULL);

	NEWTON_API dFloat GetActuatorAngle() const;
	NEWTON_API bool GetEnableFlag (bool flag) const;
	NEWTON_API void SetEnableFlag (bool flag);

	NEWTON_API dFloat GetTargetAngle() const;
	NEWTON_API void SetTargetAngle(dFloat angle);

	NEWTON_API dFloat GetMinAngularLimit() const;
	NEWTON_API void SetMinAngularLimit(dFloat limit);

	NEWTON_API dFloat GetMaxAngularLimit() const;
	NEWTON_API void SetMaxAngularLimit(dFloat limit);

	NEWTON_API dFloat GetAngularRate() const;
	NEWTON_API void SetAngularRate(dFloat rate);

	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	private:
	dFloat m_angle;
	dFloat m_minAngle;
	dFloat m_maxAngle;
	dFloat m_angularRate;
	bool m_flag;
};

#endif
