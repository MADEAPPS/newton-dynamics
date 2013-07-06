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



// CustomPathFollow.h: interface for the CustomPathFollow class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOM_PATH_FOLLOW_H_H)
#define AFX_CUSTOM_PATH_FOLLOW_H_H

#include "CustomJoint.h"

class CustomPathFollow: public CustomJoint  
{
	public:
	NEWTON_API CustomPathFollow (const dMatrix& pinAndPivotFrame, NewtonBody* const body);
	NEWTON_API virtual ~CustomPathFollow();

	NEWTON_API virtual dMatrix EvalueCurve (const dVector& posit);

	protected:
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
};

#endif // !defined(AFX_CUSTOM_PATH_FOLLOW_H_H)

