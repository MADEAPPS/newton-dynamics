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

#ifndef __D_ACYCLIC_JOINT_H__
#define __D_ACYCLIC_JOINT_H__

#include "dAnimationStdAfx.h"

class dAnimationRigLimb;
class dAnimationRigJoint;
class dAnimationKinematicLoopJoint;

class dAnimationAcyclicJoint: public dContainersAlloc
{
	public:
	dAnimationAcyclicJoint(dAnimationAcyclicJoint* const parent);
	virtual ~dAnimationAcyclicJoint();

	void* GetUserData();
	void SetUserData(void* const userData);
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	NewtonWorld* GetWorld() const { return m_world; }
	void SetWorld(NewtonWorld* const world) { m_world = world; }

	dAnimationAcyclicJoint* GetParent() const { return m_parent; }
	const dList<dAnimationAcyclicJoint*>& GetChildren() const { return m_children; }

	int GetIndex() const { return m_solverIndex; }
	void SetIndex(int index) { m_solverIndex = index; }

	bool IsLoopNode() const { return m_isLoop; }
	void SetLoopNode(bool staste) { m_isLoop = staste; }

	dComplementaritySolver::dBodyState* GetBody() { return &m_body; }
	virtual dComplementaritySolver::dBilateralJoint* GetJoint() { return m_joint; }

	virtual void ApplyExternalForce(dFloat timestep);
	virtual int GetKinematicLoops(dAnimationKinematicLoopJoint** const jointArray);

	virtual dAnimationRigLimb* GetAsRigLimb() {return NULL;}
	virtual dAnimationRigJoint* GetAsRigJoint() {return NULL;}

	protected:
	virtual void UpdateJointAcceleration();

	dComplementaritySolver::dBodyState m_body;
	dComplementaritySolver::dBilateralJoint* m_joint;
	dAnimationAcyclicJoint* m_parent;
	void* m_userData;
	NewtonWorld* m_world;
	int m_solverIndex;
	bool m_isLoop;
	dList<dAnimationAcyclicJoint*> m_children;

	friend class dAnimationAcyclicSolver;
};

#endif

