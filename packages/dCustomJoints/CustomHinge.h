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

// CustomHinge.h: interface for the CustomHinge class.
//
//////////////////////////////////////////////////////////////////////


#ifndef _CUSTOMHINGE_H_
#define _CUSTOMHINGE_H_

#include "CustomJoint.h"

class CustomHinge: public CustomJoint  
{
	public:
	dAddRtti(CustomJoint);
	CustomHinge (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);

	// this is a special contributor that create a hinge with an error between the two matrices, the error is reduce to zero after few iterations 
	// the error can not be too great, this is more for hinges with wiggle room
	CustomHinge (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	virtual ~CustomHinge();

	void EnableLimits(bool state);
	void SetLimis(dFloat minAngle, dFloat maxAngle);
	dFloat GetJointAngle () const;
	dVector GetPinAxis () const;
	dFloat GetJointOmega () const;

	protected:
	virtual void GetInfo (NewtonJointRecord* const info) const;
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsOn;
	dFloat m_minAngle;
	dFloat m_maxAngle;
	dFloat m_jointOmega;
	AngularIntegration m_curJointAngle;
};

#endif // !defined(AFX_CUSTOMHINGE_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

