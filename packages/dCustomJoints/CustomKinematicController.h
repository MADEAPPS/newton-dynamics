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



// CustomKinematicController.h: interface for the CustomKinematicController class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CUSTOM_KINEMATIC_CONTROLLER_H__
#define __CUSTOM_KINEMATIC_CONTROLLER_H__


#include "CustomJoint.h"

class CustomKinematicController: public CustomJoint
{
	public:
	dAddRtti(CustomJoint);
	CustomKinematicController (NewtonBody* const body, const dVector& attachmentPointInGlobalSpace);
	virtual ~CustomKinematicController();

	void SetPickMode (int mode);
	void SetMaxLinearFriction(dFloat accel); 
	void SetMaxAngularFriction(dFloat alpha); 
	
	void SetTargetRotation (const dQuaternion& rotation); 
	void SetTargetPosit (const dVector& posit); 
	void SetTargetMatrix (const dMatrix& matrix); 

	dMatrix GetTargetMatrix () const;

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;

	dVector m_localHandle;
	dVector m_targetPosit;
	dQuaternion m_targetRot;
	int m_pickMode;
	int m_autoSlepState;
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;
};

#endif // !defined(AFX_CustomKinematicController_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4_H)

