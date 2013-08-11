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



// CustomSliderActuator.h: interface for the CustomSliderActuator class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_SLIDER_ACTUATOR_H__
#define _CUSTOM_SLIDER_ACTUATOR_H__

#include "CustomJoint.h"
#include "CustomSlider.h"

class CustomSliderActuator: public CustomSlider
{
	public:
	NEWTON_API CustomSliderActuator (const dMatrix& pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, NewtonBody* const child, NewtonBody* const parent = NULL);
	NEWTON_API virtual ~CustomSliderActuator();

	NEWTON_API bool GetEnableFlag (bool flag) const;
	NEWTON_API void SetEnableFlag (bool flag);

	NEWTON_API dFloat GetActuatorPosit() const;
	NEWTON_API dFloat GetTargetPosit() const;
	NEWTON_API void SetTargetPosit(dFloat posit);

	NEWTON_API dFloat GetLinearRate() const;
	NEWTON_API void SetLinearRate(dFloat rate);

	NEWTON_API dFloat GetMinPositLimit() const;
	NEWTON_API void SetMinPositLimit(dFloat limit);

	NEWTON_API dFloat GetMaxPositLimit() const;
	NEWTON_API void SetMaxPositLimit(dFloat limit);

	protected:
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_linearRate;
	dFloat m_posit;
	dFloat m_minPosit;
	dFloat m_maxPosit;
	bool m_flag;
};

#endif

