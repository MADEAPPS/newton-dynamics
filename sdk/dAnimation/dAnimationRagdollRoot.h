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


#ifndef __D_ANIMATION_RAGDOLL_ROOT_H__
#define __D_ANIMATION_RAGDOLL_ROOT_H__

#include "dAnimationStdAfx.h"
#include "dAnimationJointRoot.h"

class dAnimationRagdollRoot: public dAnimationJointRoot
{
	public:
	dAnimationRagdollRoot(NewtonBody* const body, const dMatrix& bindMarix);
	virtual ~dAnimationRagdollRoot();

	dVector CalculateCenterOfMass() const;

	virtual void PreUpdate(dFloat timestep);
};


#endif 

