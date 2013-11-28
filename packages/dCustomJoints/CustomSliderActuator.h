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
	CUSTOM_JOINTS_API CustomSliderActuator (const dMatrix& pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomSliderActuator();

	CUSTOM_JOINTS_API bool GetEnableFlag () const;
	CUSTOM_JOINTS_API void SetEnableFlag (bool flag);

	CUSTOM_JOINTS_API dFloat GetActuatorPosit() const;
	CUSTOM_JOINTS_API dFloat GetTargetPosit() const;
	CUSTOM_JOINTS_API void SetTargetPosit(dFloat posit);

	CUSTOM_JOINTS_API dFloat GetLinearRate() const;
	CUSTOM_JOINTS_API void SetLinearRate(dFloat rate);

	CUSTOM_JOINTS_API dFloat GetMinPositLimit() const;
	CUSTOM_JOINTS_API void SetMinPositLimit(dFloat limit);

	CUSTOM_JOINTS_API dFloat GetMaxPositLimit() const;
	CUSTOM_JOINTS_API void SetMaxPositLimit(dFloat limit);

    CUSTOM_JOINTS_API dFloat GetMaxForcePower() const;
    CUSTOM_JOINTS_API void SetMaxForcePower(dFloat force);

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_linearRate;
	dFloat m_posit;
	dFloat m_minPosit;
	dFloat m_maxPosit;
    dFloat m_maxForce;
	bool m_flag;
};

#endif

