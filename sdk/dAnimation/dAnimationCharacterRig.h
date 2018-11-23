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


#ifndef __D_ANIMATION_CHARACTER_RIG_H__
#define __D_ANIMATION_CHARACTER_RIG_H__

#include "dAnimationStdAfx.h"
#include "dAnimationRigJoint.h"
#include "dAnimationAcyclicSolver.h"

class dAnimationCharacterRig: public dCustomControllerBase, public dAnimationRigJoint
{
	public:
	dAnimationCharacterRig ();
	~dAnimationCharacterRig ();
	
	void Finalize();

	protected:
	virtual void PreUpdate(dFloat timestep, int threadIndex);
	virtual void PostUpdate(dFloat timestep, int threadIndex);
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	void Init(NewtonBody* const body);
	
	NewtonBody* GetNewtonBody() const;
	dComplementaritySolver::dBodyState* GetStaticWorld() {return &m_staticWorld;}

	dComplementaritySolver::dBodyState m_staticWorld;
	dAnimationAcyclicSolver m_solver;
	friend class dAnimationCharacterRigManager;
};


#endif 

