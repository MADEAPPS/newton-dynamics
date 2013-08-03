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

#ifndef _D_NEWTON_ACTUATOR_JOINTS_H_
#define _D_NEWTON_ACTUATOR_JOINTS_H_

#include "dStdAfxNewton.h"
#include "dNewtonJoint.h"


class dNewtonHingeActuator: public dNewtonHingeJoint
{
	public:
	CNEWTON_API dNewtonHingeActuator(const dFloat* const pinAndPivotFrame, dFloat minAngle, dFloat maxAngle, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonHingeJoint(pinAndPivotFrame, body0, body1)
		,m_angle(0.0f)
		,m_minAngle(minAngle)
		,m_maxAngle(maxAngle)
	{
	}

	CNEWTON_API dFloat GetTargetAngle() const
	{
		return m_angle;
	}

	CNEWTON_API void SetTargetAngle(dFloat angle)
	{
		m_angle = dClamp (angle, m_minAngle, m_maxAngle);
	}
	
	CNEWTON_API dFloat GetActuatorAngle() const;
	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
	dFloat m_angle;
	dFloat m_minAngle;
	dFloat m_maxAngle;
};

#endif
