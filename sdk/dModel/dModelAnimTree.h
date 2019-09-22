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

#include "dModelStdAfx.h"

class dModelRootNode;

class dModelKeyFrame
{
	public:
	void SetMatrix(const dMatrix& matrix)
	{
		m_posit = matrix.m_posit;
		m_rotation = dQuaternion(matrix);
	}

	dVector m_posit;
	dQuaternion m_rotation;
	dCustomKinematicController* m_effector;
};

class dModelKeyFramePose: public dList<dModelKeyFrame>
{
	public:
	dModelKeyFramePose()
		:dList<dModelKeyFrame>()
	{
	}

	dModelKeyFramePose(const dModelKeyFramePose& src)
		:dList<dModelKeyFrame>()
	{
		for (dListNode* node = src.GetFirst(); node; node = node->GetNext()) {
			Append (node->GetInfo());
		}
	}

	void CopyKeyFrames(dModelKeyFramePose& output) const
	{
		dAssert (GetCount() == output.GetCount());
		for (dListNode* srcNode = GetFirst(), *dstNode = output.GetFirst(); srcNode; srcNode = srcNode->GetNext(), dstNode = dstNode->GetNext()) {
			dModelKeyFrame& dst = dstNode->GetInfo();
			const dModelKeyFrame& src = srcNode->GetInfo();
			
			dst.m_posit = src.m_posit;
			dst.m_rotation = src.m_rotation;
			dAssert(dst.m_effector == src.m_effector);
		}
	}
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
	
	virtual void Evaluate(dFloat timestep) = 0;
	virtual void GeneratePose(dModelKeyFramePose& output) = 0;

	protected:
	dModelRootNode* m_model;
};

#endif 


