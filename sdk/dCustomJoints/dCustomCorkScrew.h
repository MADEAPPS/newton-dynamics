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

#include "dCustom6DOF.h"

class dCustomCorkScrew: public dCustom6DOF
{
	public:
	CUSTOM_JOINTS_API dCustomCorkScrew (const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustomCorkScrew();

	CUSTOM_JOINTS_API void EnableLinearLimits(bool state);
	CUSTOM_JOINTS_API void EnableAngularLimits(bool state);
	CUSTOM_JOINTS_API void SetLinearLimis(dFloat minDist, dFloat maxDist);
	CUSTOM_JOINTS_API void SetAngularLimis(dFloat minAngle, dFloat maxAngle);

	protected:
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const; 
	CUSTOM_JOINTS_API virtual void SubmitConstraintsFreeDof(int freeDof, const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep, int threadIndex);

	dFloat m_minLinearDist;
	dFloat m_maxLinearDist;
	dFloat m_minAngularDist;
	dFloat m_maxAngularDist;
	dFloat m_angularDamp;
	dFloat m_angularVeloc;
	bool m_limitsLinearOn;
	bool m_limitsAngularOn;
	bool m_angularmotorOn;

	DECLARE_CUSTOM_JOINT(dCustomCorkScrew, dCustom6DOF)
};

#endif 

