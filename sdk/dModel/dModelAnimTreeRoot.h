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


#ifndef __D_MODEL_ANIM_TREE_ROOT_H__
#define __D_MODEL_ANIM_TREE_ROOT_H__

#include "dCustomJoint.h"

class dEffectorTreeInterface: public dCustomAlloc
{
	public:
	class dEffectorTransform
	{
		public:
		dVector m_posit;
		dQuaternion m_rotation;
		dCustomInverseDynamicsEffector* m_effector;
	};

	class dEffectorPose: public dList<dEffectorTransform>
	{
		public:
		dEffectorPose(): dList<dEffectorTransform>(), m_childNode(NULL)	{}
		dEffectorTreeInterface* m_childNode;
	};

	dEffectorTreeInterface(NewtonBody* const rootBody)
		:dCustomAlloc()
		,m_rootBody(rootBody)
	{
	}

	virtual ~dEffectorTreeInterface()
	{
	}

	virtual void Evaluate(dEffectorPose& output, dFloat timestep)
	{
	}

	NewtonBody* GetRootBody() const 
	{
		return m_rootBody;
	}

	NewtonBody* m_rootBody;
};


class dEffectorTreeRoot: public dEffectorTreeInterface
{
	public:
	dEffectorTreeRoot(NewtonBody* const rootBody, dEffectorTreeInterface* const childNode)
		:dEffectorTreeInterface(rootBody)
	{
		m_pose.m_childNode = childNode;
	}
	CUSTOM_JOINTS_API virtual ~dEffectorTreeRoot();

	dEffectorPose& GetPose() {return m_pose;}

	CUSTOM_JOINTS_API void Update(dFloat timestep);
	CUSTOM_JOINTS_API void Evaluate(dEffectorPose& output, dFloat timestep);

	protected:
	dEffectorPose m_pose;
};

class dEffectorTreePose: public dEffectorTreeInterface
{
	public:
	dEffectorTreePose(NewtonBody* const rootBody)
		:dEffectorTreeInterface(rootBody)
	{
	}

	void Evaluate(dEffectorPose& output, dFloat timestep)
	{
		dAssert(0);
	}
};


class dEffectorTreeTwoWayBlender: public dEffectorTreeInterface
{
	public:
	dEffectorTreeTwoWayBlender(NewtonBody* const rootBody, dEffectorTreeInterface* const node0, dEffectorTreeInterface* const node1)
		:dEffectorTreeInterface(rootBody)
		,m_node0(node0)
		,m_node1(node1)
		,m_param(1.0f)
	{
	}

	~dEffectorTreeTwoWayBlender()
	{
		delete m_node0;
		delete m_node1;
	}

	protected:
	CUSTOM_JOINTS_API void Evaluate(dEffectorPose& output, dFloat timestep);

	dEffectorTreeInterface* m_node0;
	dEffectorTreeInterface* m_node1;
	dFloat m_param;
};




#endif 


