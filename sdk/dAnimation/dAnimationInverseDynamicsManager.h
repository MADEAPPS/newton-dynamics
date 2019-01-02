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


#ifndef __D_ANIMATION_CHARACTER_RIG_MANAGER_H__
#define __D_ANIMATION_CHARACTER_RIG_MANAGER_H__

#include "dAnimationStdAfx.h"
#include "dAnimationRigJoint.h"
#include "dAnimationAcyclicSolver.h"
#include "dAnimationInverseDynamicsController.h"

#define D_ANINAMTION_INVERSE_DYNAMICS_MANAGER	"__dAnimationInverseDynamicsManager__"

class dAnimationInverseDynamicsManager: public dCustomControllerManager<dAnimationInverseDynamicsController>
{
	public:
	dAnimationInverseDynamicsManager(NewtonWorld* const world);
	virtual ~dAnimationInverseDynamicsManager();

	virtual dAnimationInverseDynamicsController* CreateCharacterRig(NewtonBody* const body, const dMatrix& localFrame);
	virtual dAnimationInverseDynamicsController* CreateCharacterRig(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag);

	virtual void DestroyController(dAnimationInverseDynamicsController* const controller);
	protected:
	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);
	virtual void OnUpdateTransform (const dAnimationRigJoint* const bone, const dMatrix& localMatrix) const{}	

	friend class dAnimationRigJoint;
	friend class dAnimationInverseDynamicsController;
};
#endif 

