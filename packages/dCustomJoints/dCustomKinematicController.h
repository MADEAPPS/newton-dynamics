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



// dCustomKinematicController.h: interface for the dCustomKinematicController class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CUSTOM_KINEMATIC_CONTROLLER_H__
#define __CUSTOM_KINEMATIC_CONTROLLER_H__


#include "dCustomJoint.h"

class dCustomKinematicController: public dCustomJoint
{
	public:
	CUSTOM_JOINTS_API dCustomKinematicController (NewtonBody* const body, const dVector& attachmentPointInGlobalSpace);
	CUSTOM_JOINTS_API virtual ~dCustomKinematicController();

	CUSTOM_JOINTS_API void SetPickMode (int mode);
	CUSTOM_JOINTS_API void SetMaxLinearFriction(dFloat accel); 
	CUSTOM_JOINTS_API void SetMaxAngularFriction(dFloat alpha); 
	
	CUSTOM_JOINTS_API void SetTargetRotation (const dQuaternion& rotation); 
	CUSTOM_JOINTS_API void SetTargetPosit (const dVector& posit); 
	CUSTOM_JOINTS_API void SetTargetMatrix (const dMatrix& matrix); 

	CUSTOM_JOINTS_API dMatrix GetTargetMatrix () const;

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const {dAssert (0);} 

	dVector m_localHandle;
	dVector m_targetPosit;
	dQuaternion m_targetRot;
	int m_pickMode;
	int m_autoSlepState;
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;
};

#endif // !defined(AFX_CustomKinematicController_H__EAE1E36C_6FDF_4D86_B4EE_855E3D1046F4_H)

