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


#ifndef __D_MODEL_ANIM_TREE_POSE_H__
#define __D_MODEL_ANIM_TREE_POSE_H__

#include "dModelAnimTree.h"

class dModelAnimTreePose: public dModelAnimTree
{
	public:
	dModelAnimTreePose(dModelRootNode* const model, const dModelKeyFramePose& pose)
		:dModelAnimTree(model)
		,m_pose(pose)
	{
	}

	dModelKeyFramePose& GetPose() 
	{
		return m_pose;
	}

	virtual void Evaluate(dFloat timestep)
	{
	}

	virtual void GeneratePose(dModelKeyFramePose& output)
	{
		m_pose.CopyKeyFrames(output);
	}

	protected:
	dModelKeyFramePose m_pose;
};


#endif 


