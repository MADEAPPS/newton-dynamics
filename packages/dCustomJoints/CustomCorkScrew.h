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


// CustomCorkScrew.h: interface for the CustomCorkScrew class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOMCORKSCREW_H_
#define _CUSTOMCORKSCREW_H_

#include "CustomJoint.h"

class CustomCorkScrew: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomCorkScrew (const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomCorkScrew();

	CUSTOM_JOINTS_API void EnableLinearLimits(bool state);
	CUSTOM_JOINTS_API void EnableAngularLimits(bool state);
	CUSTOM_JOINTS_API void SetLinearLimis(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void SetAngularLimis(dFloat minAngle, dFloat maxAngle);

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsLinearOn;
	bool m_limitsAngularOn;
	dFloat m_minLinearDist;
	dFloat m_maxLinearDist;
	dFloat m_minAngularDist;
	dFloat m_maxAngularDist;

	bool m_angularmotorOn;
	dFloat m_angularDamp;
	dFloat m_angularAccel;
	AngularIntegration m_curJointAngle;
};

#endif // !defined(AFX_CUSTOMCORKSCREW_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE_H)

