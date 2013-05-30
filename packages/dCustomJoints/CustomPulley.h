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


// CustomPulley.h: interface for the CustomPulley class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CustomPulley_H__
#define _CustomPulley_H__

#include "CustomJoint.h"

// this joint is for used in conjunction with Slider of other linear joints
// is is useful for establishing synchronization between the rel;ative position 
// or relative linear velocity of two sliding bodies according to the law of pulley
// velErro = -(v0 + n * v1)
// where v0 and v1 are the linear velocity
// n is the number of turn on the pulley, in the case of this joint N coudl
// be a value with fraction, as this joint is a generalization of the pulley idea.
class CustomPulley: public CustomJoint  
{
	public:
	//dAddRtti(CustomJoint);

	NEWTON_API CustomPulley(dFloat pulleyRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* const parenPin, NewtonBody* const parent);
	NEWTON_API virtual ~CustomPulley();


	protected:
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	dFloat m_gearRatio;
};

#endif 

