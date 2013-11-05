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
	CUSTOM_JOINTS_API void SetLimis(dFloat mindist, dFloat maxdist);
	CUSTOM_JOINTS_API bool JoinHitLimit () const ;

	CUSTOM_JOINTS_API dFloat GetJointPosit () const;
	CUSTOM_JOINTS_API dFloat GetJointSpeed () const;
	void CalculateGlobalMatrix(dMatrix& matrix0, dMatrix& matrix1) const
	{
		CustomJoint::CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);
	}

	
	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	dFloat m_speed;
	dFloat m_posit;
	dFloat m_minDist;
	dFloat m_maxDist;

	bool m_limitsOn;
	bool m_hitLimitOnLastUpdate;
};

#endif

