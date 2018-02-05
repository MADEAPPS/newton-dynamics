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


// dCustomCorkScrew.h: interface for the dCustomCorkScrew class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOMCORKSCREW_H_
#define _CUSTOMCORKSCREW_H_

#include "dCustomSlider.h"

class dCustomCorkScrew: public dCustomSlider
{
	public:
	CUSTOM_JOINTS_API dCustomCorkScrew (const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent = NULL);
	CUSTOM_JOINTS_API dCustomCorkScrew (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomCorkScrew();

	CUSTOM_JOINTS_API void EnableAngularLimits(bool state);
	CUSTOM_JOINTS_API void SetAngularLimis(dFloat minAngle, dFloat maxAngle);

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	//CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	protected:
	CUSTOM_JOINTS_API virtual void SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep);


	dAngularIntegration m_curJointAngle;
	dFloat m_minAngularDist;
	dFloat m_maxAngularDist;
	dFloat m_angularDamp;
	dFloat m_angularAccel;
	bool m_limitsAngularOn;
	bool m_angularmotorOn;

	DECLARE_CUSTOM_JOINT(dCustomCorkScrew, dCustomSlider)
};

#endif

