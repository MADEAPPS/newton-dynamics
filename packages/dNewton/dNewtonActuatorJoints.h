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
		,m_flag(true)
	{
		m_type = m_hingeActuator;
	}

	CNEWTON_API dFloat GetTargetAngle() const
	{
		return m_angle;
	}

	CNEWTON_API void SetTargetAngle(dFloat angle)
	{
		m_angle = dClamp (angle, m_minAngle, m_maxAngle);
	}

	void SetEnableFlag (bool flag)
	{
		m_flag = flag;
	}

	bool GetEnableFlag (bool flag) const
	{
		return m_flag;
	}

	
	CNEWTON_API dFloat GetActuatorAngle() const;
	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
	dFloat m_angle;
	dFloat m_minAngle;
	dFloat m_maxAngle;
	bool m_flag;
};


class dNewtonUniversalActuator: public dNewtonUniversalJoint
{
	public:
	CNEWTON_API dNewtonUniversalActuator(const dFloat* const pinAndPivotFrame, dFloat minAngle0, dFloat maxAngle0, dFloat minAngle1, dFloat maxAngle1, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonUniversalJoint(pinAndPivotFrame, body0, body1)
		,m_angle0(0.0f)
		,m_minAngle0(minAngle0)
		,m_maxAngle0(maxAngle0)
		,m_angle1(0.0f)
		,m_minAngle1(minAngle1)
		,m_maxAngle1(maxAngle1)
		,m_flag0(true)
		,m_flag1(true)
	{
		m_type = m_universalActuator;
	}

	CNEWTON_API dFloat GetTargetAngle0() const
	{
		return m_angle0;
	}

	CNEWTON_API void SetTargetAngle0(dFloat angle)
	{
		m_angle0 = dClamp (angle, m_minAngle0, m_maxAngle0);
	}

	CNEWTON_API dFloat GetTargetAngle1() const
	{
		return m_angle1;
	}

	CNEWTON_API void SetTargetAngle1(dFloat angle)
	{
		m_angle0 = dClamp (angle, m_minAngle1, m_maxAngle1);
	}

	void SetEnableFlag0 (bool flag)
	{
		m_flag0 = flag;
	}

	bool GetEnableFlag0 (bool flag) const
	{
		return m_flag0;
	}

	void SetEnableFlag1 (bool flag)
	{
		m_flag1 = flag;
	}

	bool GetEnableFlag (bool flag) const
	{
		return m_flag1;
	}


	CNEWTON_API dFloat GetActuatorAngle0() const;
	CNEWTON_API dFloat GetActuatorAngle1() const;
	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
	dFloat m_angle0;
	dFloat m_minAngle0;
	dFloat m_maxAngle0;

	dFloat m_angle1;
	dFloat m_minAngle1;
	dFloat m_maxAngle1;
	bool m_flag0;
	bool m_flag1;
};



class dNewtonSliderActuator: public dNewtonSliderJoint
{
	public:
	CNEWTON_API dNewtonSliderActuator(const dFloat* const pinAndPivotFrame, dFloat minPosit, dFloat maxPosit, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonSliderJoint(pinAndPivotFrame, body0, body1)
		,m_posit(0.0f)
		,m_minPosit(minPosit)
		,m_maxPosit(maxPosit)
		,m_flag(true)
	{
		m_type = m_sliderActuator;
	}

	CNEWTON_API dFloat GetTargetPosit() const
	{
		return m_posit;
	}

	CNEWTON_API void SetTargetPosit(dFloat posit)
	{
		m_posit = dClamp (posit, m_minPosit, m_maxPosit);
	}

	void SetEnableFlag0 (bool flag)
	{
		m_flag = flag;
	}

	bool GetEnableFlag0 (bool flag) const
	{
		return m_flag;
	}

	CNEWTON_API dFloat GetActuatorPosit() const;
	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
	dFloat m_posit;
	dFloat m_minPosit;
	dFloat m_maxPosit;
	bool m_flag;
};




#endif
