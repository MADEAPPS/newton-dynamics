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


// CustomGear.h: interface for the CustomGear class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CustomGear_H__
#define _CustomGear_H__

#include "CustomJoint.h"

// this joint is for used in conjunction with Hinge of other spherical joints
// is is usefully for establishing synchronization between the phase angle other the 
// relative angular velocity of two spinning disk according to the law of gears
// velErro = -(W0 * r0 + W1 *  r1)
// where w0 and W1 are the angular velocity
// r0 and r1 are the radius of the spinning disk
class CustomGear: public CustomJoint  
{
	public:
	//dAddRtti(CustomJoint);

	CustomGear(dFloat gearRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* const parenPin, NewtonBody* const parent);
	virtual ~CustomGear();


	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	dFloat m_gearRatio;
};

#endif // !defined(AFX_CustomGear_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

