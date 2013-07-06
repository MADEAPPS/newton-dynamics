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



// CustomSlidingContact.h: interface for the CustomSlidingContact class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_SLIDING_H_
#define _CUSTOM_SLIDING_H_

#include "CustomJoint.h"

class CustomSlidingContact: public CustomJoint  
{
	public:
	NEWTON_API CustomSlidingContact (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	NEWTON_API virtual ~CustomSlidingContact();

	NEWTON_API void EnableLinearLimits(bool state);
	NEWTON_API void EnableAngularLimits(bool state);
	NEWTON_API void SetLinearLimis(dFloat minAngle, dFloat maxAngle);
	NEWTON_API void SetAngularLimis(dFloat minAngle, dFloat maxAngle);

	protected:
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsLinearOn;
	bool m_limitsAngularOn;
	dFloat m_minLinearDist;
	dFloat m_maxLinearDist;
	dFloat m_minAngularDist;
	dFloat m_maxAngularDist;

};

#endif // !defined(AFX_CustomSlidingContact_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

