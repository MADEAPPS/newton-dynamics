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
	//dAddRtti(CustomJoint);

	NEWTON_API CustomKinematicController (NewtonBody* const body, const dVector& attachmentPointInGlobalSpace);
	NEWTON_API virtual ~CustomKinematicController();

	NEWTON_API void SetPickMode (int mode);
	NEWTON_API void SetMaxLinearFriction(dFloat accel); 
	NEWTON_API void SetMaxAngularFriction(dFloat alpha); 
	
	NEWTON_API void SetTargetRotation (const dQuaternion& rotation); 
	NEWTON_API void SetTargetPosit (const dVector& posit); 
	NEWTON_API void SetTargetMatrix (const dMatrix& matrix); 

	NEWTON_API dMatrix GetTargetMatrix () const;

	protected:
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;

	dVector m_localHandle;
	dVector m_targetPosit;
	dQuaternion m_targetRot;
	int m_pickMode;
	int m_autoSlepState;
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;
};

#endif // !defined(AFX_CustomKinematicController_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4_H)

