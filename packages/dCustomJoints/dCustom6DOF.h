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



// dCustom6DOF.h: interface for the dCustom6DOF class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_6DOF_H_
#define _CUSTOM_6DOF_H_

#include "dCustomJoint.h"

class dCustom6DOF: public dCustomJoint  
{
	public:
	CUSTOM_JOINTS_API dCustom6DOF (const dMatrix& pinsAndPivotChildFrame, const dMatrix& pinsAndPivotParentFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustom6DOF();

	CUSTOM_JOINTS_API void SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits);
	CUSTOM_JOINTS_API void SetAngularLimits (const dVector& minAngularLimits, const dVector& maxAngularLimits);
	CUSTOM_JOINTS_API void GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits);
	CUSTOM_JOINTS_API void GetAngularLimits (dVector& minAngularLimits, dVector& maxAngularLimits);

	protected:
	CUSTOM_JOINTS_API dCustom6DOF (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dVector m_minLinearLimits;
	dVector m_maxLinearLimits;
	dVector m_minAngularLimits;
	dVector m_maxAngularLimits;
	AngularIntegration m_pitch;
	AngularIntegration m_yaw;
	AngularIntegration m_roll;
	DECLARE_CUSTOM_JOINT(dCustom6DOF, dCustomJoint)
};

#endif // !defined(AFX_Custom6DOF_H__B631F556_B7D7_F85ECF3E9ADE_H)

