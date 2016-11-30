/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomUpVector.h: interface for the CustomUpVector class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _DRY_ROLLOING_FRICTION3D1046F4_
#define _DRY_ROLLOING_FRICTION3D1046F4_


#include "CustomJoint.h"


// this joint is usefully to simulate the rolling friction of a rolling ball over 
// a flat surface.
// normally this is not important for non spherical objects, but for games like 
// poll, pinball, bolling, golf or any other where the movement of balls is the main objective
// the rolling friction is a real big problem.
class CustomDryRollingFriction: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomDryRollingFriction(NewtonBody* const child, dFloat radius, dFloat coefficient);
	CUSTOM_JOINTS_API virtual ~CustomDryRollingFriction();

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dFloat m_frictionCoef;
	dFloat m_frictionTorque;
	
};



#endif // !defined(AFX_DRY_ROLLOING_FRICTION3D1046F4_H)

