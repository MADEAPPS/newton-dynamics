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


#ifndef __D_ANIMATION_JOINT_ROOT_H__
#define __D_ANIMATION_JOINT_ROOT_H__

#include "dAnimationStdAfx.h"
#include "dAnimationJoint.h"


class dAnimationJointRoot: public dAnimationJoint
{
	public:
	dAnimationJointRoot(NewtonBody* const body)
		:dAnimationJoint(body, NULL)
		,m_calculateLocalTransform(true)
	{
	}

	~dAnimationJointRoot()
	{
	}

	dAnimationJoint* AddBone(NewtonBody* const bone, const dMatrix& bindMatrix, dAnimationJoint* const parentBone);

	void SetCalculateLocalTransforms(bool val) { m_calculateLocalTransform = val; }
	bool GetCalculateLocalTransforms() const { return m_calculateLocalTransform; }

	private:
	//void PostUpdate(dCustomTransformManager* const manager, dFloat timestep) const;

	bool m_calculateLocalTransform;

	friend class dCustomTransformManager;
};

#endif 

