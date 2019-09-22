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


#ifndef __D_MODEL_ANIM_TREE_POSE_BLENDER_H__
#define __D_MODEL_ANIM_TREE_POSE_BLENDER_H__

#include "dModelAnimTree.h"
#include "dModelAnimTreePose.h"

class dModelAnimTreePoseBlender: public dModelAnimTree
{
	public:
	dModelAnimTreePoseBlender(dModelRootNode* const model, dModelAnimTreePose* const pose0, dModelAnimTreePose* const pose1)
		:dModelAnimTree(model)
		,m_pose0(pose0)
		,m_pose1(pose1)
		,m_param(1.0f)
	{
	}

	~dModelAnimTreePoseBlender()
	{
		delete m_pose0;
		delete m_pose1;
	}

	dModelAnimTreePoseBlender& operator=(const dModelAnimTreePoseBlender& src);

	dFloat GetParam() const
	{
		return m_param;
	}

	void SetParam(dFloat param)
	{
		m_param = dClamp(param, dFloat (0.0f), dFloat (1.0f));
	}

	virtual void Evaluate(dFloat timestep)
	{
		m_pose0->Evaluate(timestep);
		m_pose1->Evaluate(timestep);
	}

	virtual void GeneratePose(dModelKeyFramePose& output)
	{
		if (m_param < 0.001f) {
			m_pose0->GeneratePose(output);
		} else if (m_param > 0.999f) {
			m_pose1->GeneratePose(output);
		} else {
			int index = 0;
			m_pose1->GeneratePose(output);
			dModelKeyFrame* const tmpKeyFrame = dAlloca(dModelKeyFrame, output.GetCount());
			for (dModelKeyFramePose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
				tmpKeyFrame[index] = node->GetInfo();
				index++;
			}
			index = 0;
			m_pose0->GeneratePose(output);
			for (dModelKeyFramePose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
				dModelKeyFrame& dst = node->GetInfo();
				const dModelKeyFrame& src = tmpKeyFrame[index];
				index++;

				dst.m_posit = dst.m_posit.Scale(1.0f - m_param) + src.m_posit.Scale(m_param);
				dQuaternion srcRotation(src.m_rotation);
				srcRotation.Scale(dSign(dst.m_rotation.DotProduct(src.m_rotation)));
				dst.m_rotation = dst.m_rotation.Slerp(srcRotation, m_param);
				dst.m_posit.m_w = 1.0f;
			}
		}
	}

	protected:
	dModelAnimTreePose* const m_pose0;
	dModelAnimTreePose* const m_pose1;
	dFloat m_param;
};


#endif 


