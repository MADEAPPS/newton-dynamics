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
	CUSTOM_JOINTS_API CustomSlidingContact (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomSlidingContact();

	CUSTOM_JOINTS_API void EnableLinearLimits(bool state);
	CUSTOM_JOINTS_API void EnableAngularLimits(bool state);
	CUSTOM_JOINTS_API void SetLinearLimis(dFloat minDist, dFloat maxDist);
	CUSTOM_JOINTS_API void SetAngularLimis(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetSpeed() const;
	CUSTOM_JOINTS_API dFloat GetPosition() const;

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	AngularIntegration m_curJointAngle;
	dFloat m_speed;
	dFloat m_posit;
	dFloat m_minLinearDist;
	dFloat m_maxLinearDist;
	dFloat m_minAngularDist;
	dFloat m_maxAngularDist;
	bool m_limitsLinearOn;
	bool m_limitsAngularOn;
};

#endif // !defined(AFX_CustomSlidingContact_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

