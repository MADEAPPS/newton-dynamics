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
	NEWTON_API CustomHinge (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);

	// this is a special contributor that create a hinge with an error between the two matrices, the error is reduce to zero after few iterations 
	// the error can not be too great, this is more for hinges with wiggle room
	NEWTON_API CustomHinge (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	NEWTON_API virtual ~CustomHinge();

	NEWTON_API void EnableLimits(bool state);
	NEWTON_API void SetLimis(dFloat minAngle, dFloat maxAngle);
	NEWTON_API dFloat GetJointAngle () const;
	NEWTON_API dVector GetPinAxis () const;
	NEWTON_API dFloat GetJointOmega () const;
	void CalculateGlobalMatrix(dMatrix& matrix0, dMatrix& matrix1) const
	{
		CustomJoint::CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);
	}

	protected:
	void CalculatePitchAngle (const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const;
	NEWTON_API virtual void ProjectError () const;
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsOn;
	dFloat m_minAngle;
	dFloat m_maxAngle;
	dFloat m_jointOmega;
	AngularIntegration m_curJointAngle;
};

#endif 

