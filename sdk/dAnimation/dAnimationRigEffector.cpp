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

#include "dAnimationStdAfx.h"
#include "dAnimationRigLimb.h"
#include "dAnimationRigEffector.h"
#include "dAnimationCharacterRigManager.h"


dAnimationRigEffector::dAnimationRigEffector(dAnimationRigLimb* const parent)
	:dAnimationKinematicLoopJoint()
{
	dAssert (!parent->m_effector);
	parent->m_effector = this;
	dAnimationCharacterRig* const root = parent->GetRoot();
	Init(parent->GetBody(), root->GetStaticWorld());
	SetOwners(parent, root);
}

dAnimationRigEffector::~dAnimationRigEffector()
{
}


