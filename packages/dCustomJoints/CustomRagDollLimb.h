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


// "CustomRagDollLimb.h".h: interface for the "CustomRagDollLimb.h" class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_RAGDOLL_H__ 
#define _CUSTOM_RAGDOLL_H__

#include "CustomJoint.h"
#include "CustomBallAndSocket.h"

class CustomRagDollLimb: public CustomLimitBallAndSocket  
{
	public:
	dAddRtti(CustomRagDollLimb);

	CustomRagDollLimb (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);

	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
};


#endif
