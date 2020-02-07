/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/



// dCustomPathFollow.h: interface for the dCustomPathFollow class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOM_PATH_FOLLOW_H_H)
#define AFX_CUSTOM_PATH_FOLLOW_H_H

#include "dCustomJoint.h"

class dCustomPathFollow: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomPathFollow (const dMatrix& pinAndPivotFrame, NewtonBody* const body, NewtonBody* const parentPath);
	CUSTOM_JOINTS_API virtual ~dCustomPathFollow();

	virtual void GetPointAndTangentAtLocation(const dVector& location, dVector& positOut, dVector& tangentOut) const { dAssert(0);}

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	DECLARE_CUSTOM_JOINT(dCustomPathFollow, dCustomJoint)
};

#endif

