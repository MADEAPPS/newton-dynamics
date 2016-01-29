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


//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_DYNAMIC_RAGDOLL_MANAGER_H_
#define D_CUSTOM_DYNAMIC_RAGDOLL_MANAGER_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomArcticulatedTransformManager.h>


class CustomDynamicRagDollManager: public CustomArticulaledTransformManager
{
	public:
	CUSTOM_JOINTS_API CustomDynamicRagDollManager(NewtonWorld* const world, bool applyLocalTransform);
	CUSTOM_JOINTS_API virtual ~CustomDynamicRagDollManager();

	CUSTOM_JOINTS_API virtual void Debug () const;

	CUSTOM_JOINTS_API virtual CustomArticulatedTransformController* CreateTransformController (void* const userData);

	CUSTOM_JOINTS_API virtual void OnPreUpdate (CustomArticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const;
	CUSTOM_JOINTS_API virtual void OnUpdateTransform (const CustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
};


#endif 

