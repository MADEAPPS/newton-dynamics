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



// CustomSlider.h: interface for the CustomSlider class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_SLIDER_H__
#define _CUSTOM_SLIDER_H__

#include "CustomJoint.h"

class CustomSlider: public CustomJoint  
{
	public:
	CUSTOM_JOINTS_API CustomSlider (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~CustomSlider();

	CUSTOM_JOINTS_API void EnableLimits(bool state);
	CUSTOM_JOINTS_API void SetLimits(dFloat mindist, dFloat maxdist);

	CUSTOM_JOINTS_API dFloat GetJointPosit () const;
	CUSTOM_JOINTS_API dFloat GetJointSpeed () const;
	
	protected:
	CUSTOM_JOINTS_API CustomSlider (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 

	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void SubmitConstraintsFreeDof (dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1);

	dFloat m_speed;
	dFloat m_posit;
	dFloat m_minDist;
	dFloat m_maxDist;
	bool m_limitsOn;
	bool m_lastRowWasUsed;
	DECLARE_CUSTOM_JOINT(CustomSlider, CustomJoint)
};

#endif

