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


class dNewtonHingeActuator: public dNewtonJoint
{
	public:
	CNEWTON_API dNewtonHingeActuator(const dFloat* const pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonJoint(m_hingeActuator)
	{
		SetJoint (new CustomHingeActuator (pinAndPivotFrame, angularRate, minAngle, maxAngle, body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
	}

	CNEWTON_API bool GetEnableFlag () const
	{
		return ((CustomHingeActuator*)m_joint)->GetEnableFlag();
	}

	CNEWTON_API dFloat GetTargetAngle() const
	{
		return ((CustomHingeActuator*)m_joint)->GetTargetAngle();
	}

	CNEWTON_API dFloat GetMinAngularLimit() const
	{
		return ((CustomHingeActuator*)m_joint)->GetMinAngularLimit();
	}

	CNEWTON_API dFloat GetMaxAngularLimit() const
	{
		return ((CustomHingeActuator*)m_joint)->GetMaxAngularLimit();
	}

	CNEWTON_API dFloat GetAngularRate() const
	{
		return ((CustomHingeActuator*)m_joint)->GetAngularRate();
	}


	CNEWTON_API void SetMinAngularLimit(dFloat limit)
	{
		((CustomHingeActuator*)m_joint)->SetMinAngularLimit(limit);
	}

	CNEWTON_API void SetMaxAngularLimit(dFloat limit)
	{
		((CustomHingeActuator*)m_joint)->SetMaxAngularLimit(limit);
	}


	CNEWTON_API void SetAngularRate(dFloat rate)
	{
		((CustomHingeActuator*)m_joint)->SetAngularRate(rate);
	}

	CNEWTON_API void SetTargetAngle(dFloat angle)
	{
		((CustomHingeActuator*)m_joint)->SetTargetAngle(angle);
	}

	CNEWTON_API void SetEnableFlag (bool flag)
	{
		((CustomHingeActuator*)m_joint)->SetEnableFlag (flag);
	}
	
	CNEWTON_API dFloat GetActuatorAngle() const
	{
		return ((CustomHingeActuator*)m_joint)->GetActuatorAngle();
	}
};



class dNewtonSliderActuator: public dNewtonJoint
{
	public:
	CNEWTON_API dNewtonSliderActuator(const dFloat* const pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonJoint(m_sliderActuator)
	{
		SetJoint (new CustomSliderActuator (pinAndPivotFrame, speed, minPosit, maxPosit, body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL)); 
	}

	CNEWTON_API bool GetEnableFlag () const
	{
		return ((CustomSliderActuator*)m_joint)->GetEnableFlag ();
	}

	CNEWTON_API dFloat GetTargetPosit() const
	{
		return ((CustomSliderActuator*)m_joint)->GetTargetPosit();
	}

	CNEWTON_API dFloat GetLinearRate() const
	{
		return ((CustomSliderActuator*)m_joint)->GetLinearRate();
	}


	CNEWTON_API dFloat GetMinPositLimit() const
	{
		return ((CustomSliderActuator*)m_joint)->GetMinPositLimit();
	}

	CNEWTON_API dFloat GetMaxPositLimit() const
	{
		return ((CustomSliderActuator*)m_joint)->GetMaxPositLimit();
	}


	CNEWTON_API void SetTargetPosit(dFloat posit)
	{
		((CustomSliderActuator*)m_joint)->SetTargetPosit(posit);
	}


	CNEWTON_API void SetMinPositLimit(dFloat limit)
	{
		((CustomSliderActuator*)m_joint)->SetMinPositLimit(limit);
	}

	CNEWTON_API void SetMaxPositLimit(dFloat limit)
	{
		((CustomSliderActuator*)m_joint)->SetMaxPositLimit(limit);
	}

	CNEWTON_API void SetLinearRate(dFloat rate)
	{
		((CustomSliderActuator*)m_joint)->SetLinearRate (rate);
	}

	void SetEnableFlag (bool flag)
	{
		((CustomSliderActuator*)m_joint)->SetEnableFlag (flag);
	}

	CNEWTON_API dFloat GetActuatorPosit() const
	{
		return ((CustomSliderActuator*)m_joint)->GetActuatorPosit();
	}
};


class dNewtonUniversalActuator: public dNewtonJoint 
{
	public:
	CNEWTON_API dNewtonUniversalActuator(const dFloat* const pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
		:dNewtonJoint(m_universalActuator)
	{
		SetJoint (new CustomUniversalActuator (pinAndPivotFrame, angularRate0, minAngle0, maxAngle0, angularRate1, minAngle1, maxAngle1, body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
	}

	CNEWTON_API void SetEnableFlag0 (bool flag)
	{
		((CustomUniversalActuator*)m_joint)->SetEnableFlag0 (flag);
	}

	CNEWTON_API void SetTargetAngle0(dFloat angle)
	{
		((CustomUniversalActuator*)m_joint)->SetTargetAngle0 (angle);
	}

	CNEWTON_API void SetMinAngularLimit0(dFloat limit)
	{
		((CustomUniversalActuator*)m_joint)->SetMinAngularLimit0 (limit);
	}

	CNEWTON_API void SetMaxAngularLimit0(dFloat limit)
	{
		((CustomUniversalActuator*)m_joint)->SetMaxAngularLimit0(limit);
	}

	CNEWTON_API void SetAngularRate0(dFloat rate)
	{
		((CustomUniversalActuator*)m_joint)->SetAngularRate0(rate);
	}


	CNEWTON_API void SetEnableFlag1 (bool flag)
	{
		((CustomUniversalActuator*)m_joint)->SetEnableFlag1 (flag);
	}

	CNEWTON_API void SetTargetAngle1(dFloat angle)
	{
		((CustomUniversalActuator*)m_joint)->SetTargetAngle1 (angle);
	}

	CNEWTON_API void SetMinAngularLimit1(dFloat limit)
	{
		((CustomUniversalActuator*)m_joint)->SetMinAngularLimit1 (limit);
	}

	CNEWTON_API void SetMaxAngularLimit1(dFloat limit)
	{
		((CustomUniversalActuator*)m_joint)->SetMaxAngularLimit1(limit);
	}

	CNEWTON_API void SetAngularRate1(dFloat rate)
	{
		((CustomUniversalActuator*)m_joint)->SetAngularRate1(rate);
	}



	CNEWTON_API bool GetEnableFlag0 () const
	{
		return ((CustomUniversalActuator*)m_joint)->GetEnableFlag0();
	}

	CNEWTON_API dFloat GetTargetAngle0() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetTargetAngle0();
	}

	CNEWTON_API dFloat GetAngularRate0() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetAngularRate0();
	}

	CNEWTON_API dFloat GetMinAngularLimit0() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetMinAngularLimit0();
	}

	CNEWTON_API dFloat GetMaxAngularLimit0() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetMaxAngularLimit0();
	}



	CNEWTON_API bool GetEnableFlag1 () const
	{
		return ((CustomUniversalActuator*)m_joint)->GetEnableFlag1();
	}

	CNEWTON_API dFloat GetTargetAngle1() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetTargetAngle1();
	}

	CNEWTON_API dFloat GetAngularRate1() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetAngularRate1();
	}

	CNEWTON_API dFloat GetMinAngularLimit1() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetMinAngularLimit1();
	}

	CNEWTON_API dFloat GetMaxAngularLimit1() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetMaxAngularLimit1();
	}

	CNEWTON_API dFloat GetActuatorAngle0() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetActuatorAngle0();
	}

	CNEWTON_API dFloat GetActuatorAngle1() const
	{
		return ((CustomUniversalActuator*)m_joint)->GetActuatorAngle1();
	}
};



#endif

