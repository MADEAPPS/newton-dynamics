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


#ifndef __D_MODEL_ANIM_TREE_FIX_POSE_H__
#define __D_MODEL_ANIM_TREE_FIX_POSE_H__

#include "dModelAnimTreePose.h"
class dModelAnimTreeFixPose : public dModelAnimTreePose
{
	public:
	dModelAnimTreeFixPose(dModelRootNode* const model)
		:dModelAnimTreePose(rootBody)
	{
	}

	virtual dModelAnimTreePose& GetPose() { return m_pose; }

	protected:
	virtual void Evaluate(dModelAnimTreePose& output, dFloat timestep);
	dModelAnimTreePose m_pose;
};

#endif 


