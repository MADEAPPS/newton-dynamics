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
#include "dAnimIDRigJoint.h"
#include "dAnimIKController.h"
#include "dAnimAcyclicSolver.h"


#define D_ANIM_IK_MANAGER	"__dAnimIKManager__"

class dAnimIKManager: public dCustomControllerManager<dAnimIKController>
{
	public:
	dAnimIKManager(NewtonWorld* const world);
	virtual ~dAnimIKManager();

	virtual dAnimIKController* CreateIKController();
//	virtual dAnimIKController* CreateCharacterRig(NewtonBody* const body, const dMatrix& localFrame);
//	virtual dAnimIKController* CreateCharacterRig(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag);

	virtual void DestroyController(dAnimIKController* const controller);
	protected:
	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);
	virtual void OnUpdateTransform (const dAnimIDRigJoint* const bone, const dMatrix& localMatrix) const{}	

//	friend class dAnimIDRigJoint;
	friend class dAnimIKController;
};
#endif 

