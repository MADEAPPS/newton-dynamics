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

#include "CustomJointLibraryStdAfx.h"

#include "CustomRagDollLimb.h"


dInitRtti(CustomRagDollLimb);


CustomRagDollLimb::CustomRagDollLimb (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomLimitBallAndSocket(pinAndPivotFrame, child, parent)
{
}

void CustomRagDollLimb::SubmitConstraints (dFloat timestep, int threadIndex)
{
	CustomLimitBallAndSocket::SubmitConstraints (timestep, threadIndex);
}


