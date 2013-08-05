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


#ifndef _CUSTOMHINGE_ACTUATOR_H_
#define _CUSTOMHINGE_ACTUATOR_H_

#include "CustomJoint.h"
#include "CustomHinge.h"

class CustomHingeActuator: public CustomHinge
{
	public:
	NEWTON_API CustomHingeActuator(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL, dFloat minAngle = 30.0f * 3.141592f / 180.0f, dFloat maxAngle = 30.0f * 3.141592f / 180.0f)
		:CustomHinge (pinAndPivotFrame, child, parent)
		,m_targetAngle(0.0f)
		,m_minAngle(minAngle)
		,m_maxAngle(maxAngle)
		,m_flag(true)
	{
		EnableLimits(false);
	}

	NEWTON_API dFloat GetTargetAngle() const
	{
		return m_targetAngle;
	}

	NEWTON_API void SetTargetAngle(dFloat angle)
	{
		m_targetAngle = dClamp (angle, m_minAngle, m_maxAngle);
	}

	void SetEnableFlag (bool flag)
	{
		m_flag = flag;
	}

	bool GetEnableFlag (bool flag) const
	{
		return m_flag;
	}
	
	NEWTON_API dFloat GetActuatorAngle() const
	{
		return GetJointAngle();
	}

	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	private:
	dFloat m_targetAngle;
	dFloat m_minAngle;
	dFloat m_maxAngle;
	bool m_flag;
};

#endif
