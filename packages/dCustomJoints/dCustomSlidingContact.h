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



// dCustomSlidingContact.h: interface for the dCustomSlidingContact class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_SLIDING_H_
#define _CUSTOM_SLIDING_H_

#include "dCustomJoint.h"

class dCustomSlidingContact: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustomSlidingContact (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomSlidingContact();

	CUSTOM_JOINTS_API void EnableLinearLimits(bool state);
	CUSTOM_JOINTS_API void EnableAngularLimits(bool state);
	CUSTOM_JOINTS_API void SetLinearLimits(dFloat minDist, dFloat maxDist);
	CUSTOM_JOINTS_API void SetAngularLimits(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetSpeed() const;
	CUSTOM_JOINTS_API dFloat GetPosition() const;
	CUSTOM_JOINTS_API void SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper);

	protected:
	CUSTOM_JOINTS_API dCustomSlidingContact(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	AngularIntegration m_curJointAngle;
	dFloat m_speed;
	dFloat m_posit;
	dFloat m_minLinearDist;
	dFloat m_maxLinearDist;
	dFloat m_minAngularDist;
	dFloat m_maxAngularDist;

	dFloat m_spring;
	dFloat m_damper;
	dFloat m_springDamperRelaxation;
	union
	{
		int m_flags;
		struct
		{
			unsigned m_limitsLinearOn	 : 1;
			unsigned m_limitsAngularOn	 : 1;
			unsigned m_setAsSpringDamper : 1;
			unsigned m_lastRowWasUsed	 : 1;
		};
	};
	DECLARE_CUSTOM_JOINT(dCustomSlidingContact, dCustomJoint)
};

#endif 

