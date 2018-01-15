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
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_SLIDING_H_
#define _CUSTOM_SLIDING_H_


#include "dCustom6dof.h"

class dCustomSlidingContact: public dCustom6dof
{
	public:
	CUSTOM_JOINTS_API dCustomSlidingContact (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API dCustomSlidingContact (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomSlidingContact();

	CUSTOM_JOINTS_API void EnableLinearLimits(bool state);
	CUSTOM_JOINTS_API void EnableAngularLimits(bool state);
	CUSTOM_JOINTS_API void SetLinearLimits(dFloat minDist, dFloat maxDist);
	CUSTOM_JOINTS_API void SetAngularLimits(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetSpeed() const;
	CUSTOM_JOINTS_API dFloat GetPosition() const;
	CUSTOM_JOINTS_API void SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper);

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData); 
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraintsFreeDof(int freeDof, const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep, int threadIndex);
	
	dFloat m_speed;
	dFloat m_posit;
	dFloat m_spring;
	dFloat m_damper;
	dFloat m_springDamperRelaxation;
	union
	{
		int m_options;
		struct
		{
			unsigned m_limitsLinearOn	 : 1;
			unsigned m_limitsAngularOn	 : 1;
			unsigned m_setAsSpringDamper : 1;
		};
	};
	DECLARE_CUSTOM_JOINT(dCustomSlidingContact, dCustom6dof)
};

#endif 

