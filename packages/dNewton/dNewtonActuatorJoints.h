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
	CNEWTON_API dNewtonHingeActuator(const dFloat* const pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonHingeJoint(pinAndPivotFrame, body0, body1)
		,m_angle(0.0f)
		,m_minAngle(minAngle)
		,m_maxAngle(maxAngle)
		,m_angularRate(angularRate)
		,m_flag(true)
	{
		m_type = m_hingeActuator;
	}

	CNEWTON_API bool GetEnableFlag (bool flag) const
	{
		return m_flag;
	}

	CNEWTON_API dFloat GetTargetAngle() const
	{
		return m_angle;
	}

	CNEWTON_API dFloat GetMinAngularLimit() const
	{
		return m_minAngle;
	}

	CNEWTON_API dFloat GetMaxAngularLimit() const
	{
		return m_maxAngle;
	}

	CNEWTON_API dFloat GetAngularRate() const
	{
		return m_angularRate;
	}


	CNEWTON_API void SetMinAngularLimit(dFloat limit)
	{
		m_minAngle = limit;
	}

	CNEWTON_API void SetMaxAngularLimit(dFloat limit)
	{
		m_maxAngle = limit;
	}


	CNEWTON_API void SetAngularRate(dFloat rate)
	{
		m_angularRate = rate;
	}

	CNEWTON_API void SetTargetAngle(dFloat angle)
	{
		m_angle = dClamp (angle, m_minAngle, m_maxAngle);
	}

	CNEWTON_API void SetEnableFlag (bool flag)
	{
		m_flag = flag;
	}
	
	CNEWTON_API dFloat GetActuatorAngle() const
	{
		CustomHinge* const hinge = (CustomHinge*) m_joint;
		return hinge->GetJointAngle();
	}


	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
	dFloat m_angle;
	dFloat m_minAngle;
	dFloat m_maxAngle;
	dFloat m_angularRate;
	bool m_flag;
};





class dNewtonSliderActuator: public dNewtonSliderJoint
{
	public:
	CNEWTON_API dNewtonSliderActuator(const dFloat* const pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonSliderJoint(pinAndPivotFrame, body0, body1)
		,m_linearRate(speed)
		,m_posit(0.0f)
		,m_minPosit(minPosit)
		,m_maxPosit(maxPosit)
		,m_flag(true)
	{
		m_type = m_sliderActuator;
	}

	CNEWTON_API bool GetEnableFlag (bool flag) const
	{
		return m_flag;
	}

	CNEWTON_API dFloat GetTargetPosit() const
	{
		return m_posit;
	}

	CNEWTON_API dFloat GetLinearRate() const
	{
		return m_linearRate;
	}


	CNEWTON_API dFloat GetMinPositLimit() const
	{
		return m_minPosit;
	}

	CNEWTON_API dFloat GetMaxPositLimit() const
	{
		return m_maxPosit;
	}


	CNEWTON_API void SetTargetPosit(dFloat posit)
	{
		m_posit = dClamp (posit, m_minPosit, m_maxPosit);
	}


	CNEWTON_API void SetMinPositLimit(dFloat limit)
	{
		m_minPosit = limit;
	}

	CNEWTON_API void SetMaxPositLimit(dFloat limit)
	{
		m_maxPosit = limit;
	}

	CNEWTON_API void SetLinearRate(dFloat rate)
	{
		m_linearRate = rate;
	}

	void SetEnableFlag (bool flag)
	{
		m_flag = flag;
	}


	CNEWTON_API dFloat GetActuatorPosit() const;
	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
	dFloat m_linearRate;
	dFloat m_posit;
	dFloat m_minPosit;
	dFloat m_maxPosit;
	bool m_flag;
};


class dNewtonUniversalActuator: public dNewtonUniversalJoint
{
	public:
	CNEWTON_API dNewtonUniversalActuator(const dFloat* const pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonUniversalJoint(pinAndPivotFrame, body0, body1)
		,m_angle0(0.0f)
		,m_minAngle0(minAngle0)
		,m_maxAngle0(maxAngle0)
		,m_angularRate0(angularRate0)
		,m_angle1(0.0f)
		,m_minAngle1(minAngle1)
		,m_maxAngle1(maxAngle1)
		,m_angularRate1(angularRate1)
		,m_flag0(true)
		,m_flag1(true)
	{
		m_type = m_universalActuator;
		// disable limit on both axis
		EnableLimit_0(false);
		EnableLimit_1(false);
	}

	CNEWTON_API bool GetEnableFlag0 (bool flag) const
	{
		return m_flag0;
	}

	CNEWTON_API dFloat GetTargetAngle0() const
	{
		return m_angle0;
	}

	CNEWTON_API dFloat GetAngularRate0() const
	{
		return m_angularRate0;
	}

	CNEWTON_API dFloat GetMinAngularLimit0() const
	{
		return m_minAngle0;
	}

	CNEWTON_API dFloat GetMaxAngularLimit0() const
	{
		return m_maxAngle0;
	}

	bool GetEnableFlag1 (bool flag) const
	{
		return m_flag1;
	}

	CNEWTON_API dFloat GetTargetAngle1() const
	{
		return m_angle1;
	}

	CNEWTON_API dFloat GetAngularRate1() const
	{
		return m_angularRate1;
	}

	CNEWTON_API dFloat GetMinAngularLimit1() const
	{
		return m_minAngle1;
	}

	CNEWTON_API dFloat GetMaxAngularLimit1() const
	{
		return m_maxAngle1;
	}


	CNEWTON_API void SetEnableFlag0 (bool flag)
	{
		m_flag0 = flag;
	}

	CNEWTON_API void SetTargetAngle0(dFloat angle)
	{
		m_angle0 = dClamp (angle, m_minAngle0, m_maxAngle0);
	}

	CNEWTON_API void SetMinAngularLimit0(dFloat limit)
	{
		m_minAngle0 = limit;
	}

	CNEWTON_API void SetMaxAngularLimit0(dFloat limit)
	{
		m_maxAngle0 = limit;
	}

	CNEWTON_API void SetAngularRate0(dFloat rate)
	{
		m_angularRate0 = rate;
	}

	CNEWTON_API void SetEnableFlag1 (bool flag)
	{
		m_flag1 = flag;
	}

	CNEWTON_API void SetTargetAngle1(dFloat angle)
	{
		m_angle1 = dClamp (angle, m_minAngle1, m_maxAngle1);
	}


	CNEWTON_API void SetAngularRate1(dFloat rate)
	{
		m_angularRate1 = rate;
	}

	CNEWTON_API void SetMinAngularLimit1(dFloat limit)
	{
		m_minAngle1 = limit;
	}

	CNEWTON_API void SetMaxAngularLimit1(dFloat limit)
	{
		m_maxAngle1 = limit;
	}

	CNEWTON_API dFloat GetActuatorAngle0() const;
	CNEWTON_API dFloat GetActuatorAngle1() const;
	CNEWTON_API virtual void OnSubmitConstraint (dFloat timestep, int threadIndex);

	private:
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

