/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

class dAnimationJoint;
class dAnimationJointRoot;
class dAnimationModelManager;

class dAnimationJointChildren: public dList<dAnimationJoint*>
{
};

class dAnimationBody: public dComplementaritySolver::dBodyState
{
	public:
	dAnimationBody ()
		:dComplementaritySolver::dBodyState()
		,m_owner(NULL)
		,m_index(-1)
	{
	}

	const int GetIndex() const {dAssert (m_index >= 0); return m_index;}

	dAnimationJoint* m_owner;
	int m_index;
};

class dAnimationContraint: public dComplementaritySolver::dBilateralJoint
{
	public:
	dAnimationContraint()
		:dComplementaritySolver::dBilateralJoint()
	{
	}
};

class dAnimationJoint: public dCustomAlloc
{
	public:
	dAnimationJoint(NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent);
	virtual ~dAnimationJoint();
	
	dAnimationJoint* GetParent() const;
	const dMatrix& GetBindMatrix() const;

	virtual dAnimationJoint* GetAsLeaf();
	virtual dAnimationJointRoot* GetAsRoot();

	dAnimationJointRoot* GetRoot() const;

	void* GetUserData() const;
	void SetUserData(void* const data);
	
	dAnimationJointChildren& GetChildren();
	const dAnimationJointChildren& GetChildren() const;
	const dAnimationJointChildren::dListNode* GetNode() const;

	NewtonBody* GetBody() const;
	dCustomJoint* GetJoint() const;
	dAnimationBody* GetProxyBody();
	dAnimationContraint* GetProxyJoint();

	void CopyRigidBodyMassToStates();

	protected:
	void CopyRigidBodyMassToStatesLow();
	virtual void RigidBodyToStates();
	virtual void UpdateJointAcceleration();
	virtual void ApplyExternalForce(dFloat timestep);

	dAnimationBody m_proxyBody;
	dMatrix m_bindMatrix;
	void* m_userData;
	NewtonBody* m_body;
	dCustomJoint* m_joint;
	dAnimationJoint* m_parent;
	dAnimationContraint* m_proxyJoint;
	dAnimationJointChildren::dListNode* m_node;
	dAnimationJointChildren m_children;

	friend class dAnimationJointSolver;
};

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

inline dAnimationBody* dAnimationJoint::GetProxyBody()
{
	return &m_proxyBody;
}

inline dAnimationContraint* dAnimationJoint::GetProxyJoint()
{
	return m_proxyJoint;
}

inline const dAnimationJointChildren& dAnimationJoint::GetChildren() const
{ 
	return m_children; 
}

inline dAnimationJointChildren& dAnimationJoint::GetChildren()
{
	return m_children; 
}

inline const dAnimationJointChildren::dListNode* dAnimationJoint::GetNode() const
{
	return m_node;
}

inline dAnimationJointRoot* dAnimationJoint::GetAsRoot()
{ 
	return NULL; 
}

inline dAnimationJoint* dAnimationJoint::GetAsLeaf()
{ 
	return !m_children.GetCount() ? this : NULL; 
}

inline dAnimationJointRoot* dAnimationJoint::GetRoot() const
{
	const dAnimationJoint* root = this;
	while (root->GetParent()) {
		root = root->GetParent();
	}
	return (dAnimationJointRoot*)root;
}

#endif

