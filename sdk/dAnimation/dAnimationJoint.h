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

#ifndef __D_ANIMATION_JOINT_H__
#define __D_ANIMATION_JOINT_H__

#include "dAnimationStdAfx.h"

//class dAnimIDRigLimb;
//class dAnimIDRigJoint;
//class dAnimIDRigKinematicLoopJoint;
/*
class dAnimAcyclicJoint: public dContainersAlloc
{
	public:
	dAnimAcyclicJoint(dAnimAcyclicJoint* const parent);
	virtual ~dAnimAcyclicJoint();

	void* GetUserData();
	void SetUserData(void* const userData);
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	NewtonWorld* GetWorld() const { return m_world; }
	void SetWorld(NewtonWorld* const world) { m_world = world; }

	dAnimAcyclicJoint* GetParent() const { return m_parent; }
	dList<dAnimAcyclicJoint*>& GetChildren() { return m_children;} 
	const dList<dAnimAcyclicJoint*>& GetChildren() const { return m_children; }

	int GetIndex() const { return m_solverIndex; }
	void SetIndex(int index) { m_solverIndex = index; }

	bool IsLoopNode() const { return m_isLoop; }
	void SetLoopNode(bool staste) { m_isLoop = staste; }

	dComplementaritySolver::dBodyState* GetProxyBody() { return &m_proxyBody; }
	const dComplementaritySolver::dBodyState* GetProxyBody() const { return &m_proxyBody; }
	virtual dComplementaritySolver::dBilateralJoint* GetProxyJoint() { return m_proxyJoint; }

	virtual void ApplyExternalForce(dFloat timestep);
	virtual int GetKinematicLoops(dAnimIDRigKinematicLoopJoint** const jointArray);

	virtual dAnimIDRigLimb* GetAsRigLimb() {return NULL;}
	virtual dAnimIDRigJoint* GetAsRigJoint() {return NULL;}
	virtual dAnimAcyclicJoint* GetAsAcyclicJoint() { return this; }

	protected:
	virtual void Finalize();
	virtual void UpdateJointAcceleration();

	dComplementaritySolver::dBodyState m_proxyBody;
	dComplementaritySolver::dBilateralJoint* m_proxyJoint;
	dAnimAcyclicJoint* m_parent;
	void* m_userData;
	NewtonWorld* m_world;
	int m_solverIndex;
	bool m_isLoop;
	dList<dAnimAcyclicJoint*> m_children;

	friend class dAnimAcyclicSolver;
};
*/

class dAnimationJoint;
class dAnimationModelManager;

class dAnimationJointChildren: public dList<dAnimationJoint*>
{
};

class dAnimationJoint: public dCustomAlloc
{
	public:
	dAnimationJoint(NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent);
	virtual ~dAnimationJoint();
	
	dAnimationJoint* GetParent() const;
	const dMatrix& GetBindMatrix() const;

	void* GetUserData() const;
	void SetUserData(void* const data);
	
	dAnimationJointChildren& GetChidren();
	const dAnimationJointChildren& GetChidren() const;

	NewtonBody* GetBody() const;
	dCustomJoint* GetJoint() const;

	protected:
	void PostUpdate(dAnimationModelManager* const manager, dFloat timestep) const;

	dMatrix m_bindMatrix;
	void* m_userData;
	NewtonBody* m_body;
	dCustomJoint* m_joint;
	dAnimationJoint* m_parent;
	dAnimationJointChildren m_children;
};

inline dAnimationJoint::dAnimationJoint(NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent)
	:dCustomAlloc()
	,m_bindMatrix(bindMarix)
	,m_userData(NULL)
	,m_body(body)
	,m_joint(NULL)
	,m_parent(parent)
	,m_children()
{
}

inline dAnimationJoint::~dAnimationJoint()
{
}

inline dAnimationJoint* dAnimationJoint::GetParent() const
{
	return m_parent;
}

inline const dMatrix& dAnimationJoint::GetBindMatrix() const
{ 
	return m_bindMatrix; 
}

inline void* dAnimationJoint::GetUserData() const 
{ 
	return m_userData; 
}

inline void dAnimationJoint::SetUserData(void* const data) 
{ 
	m_userData = data; 
}

inline NewtonBody* dAnimationJoint::GetBody() const
{ 
	return m_body; 
}

inline dCustomJoint* dAnimationJoint::GetJoint() const
{
	return NULL;
}

inline const dAnimationJointChildren& dAnimationJoint::GetChidren() const
{ 
	return m_children; 
}

inline dAnimationJointChildren& dAnimationJoint::GetChidren()
{
	return m_children; 
}


#endif

