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



// CustomPathFollow.h: interface for the CustomPathFollow class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOM_PATH_FOLLOW_H_H)
#define AFX_CUSTOM_PATH_FOLLOW_H_H

#include "dCustomJoint.h"

class CustomPathFollow: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomPathFollow (const dMatrix& pinAndPivotFrame, NewtonBody* const body, NewtonBody* const parentPath);
	CUSTOM_JOINTS_API virtual ~CustomPathFollow();

	virtual void GetPointAndTangentAtLocation (const dVector& location,  dVector& positOut, dVector& tangentOut) const = 0;
//	CUSTOM_JOINTS_API void SetPathTarget (const dVector& posit, const dVector& tangent);

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

//	dVector m_pathTangent;
//	dVector m_pointOnPath;
};

#endif

