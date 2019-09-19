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


#ifndef __D_MODEL_ANIM_TREE_H__
#define __D_MODEL_ANIM_TREE_H__

#include "dCustomJoint.h"

class dModelRootNode;

class dModelAnimTreeKeyFrame
{
	public:
	dVector m_posit;
	dQuaternion m_rotation;
//	dCustomInverseDynamicsEffector* m_effector;
};

class dModelAnimTreePose: public dList<dModelAnimTreeKeyFrame>
{
	public:
	dModelAnimTreePose() : dList<dModelAnimTreeKeyFrame>(), m_childNode(NULL) 
		:dList<dModelAnimTreeKeyFrame>()
	{
	}
	dModelAnimTree* m_childNode;
};


class dModelAnimTree: public dCustomAlloc
{
	public:
	dModelAnimTree(dModelRootNode* const model)
		:dCustomAlloc()
		,m_model(model)
	{
	}

	virtual ~dModelAnimTree()
	{
	}

	virtual void Evaluate(dModelAnimTreePose& output, dFloat timestep)
	{
	}

	dModelRootNode* m_model;
};

#endif 


