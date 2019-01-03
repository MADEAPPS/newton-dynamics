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

#ifndef __D_ANIM_IK_BLEND_NODE_h__
#define __D_ANIM_IK_BLEND_NODE_h__

//class dAnimIKBlendNode;
//class dAnimIDRigEffector;
class dAnimIKController;
#include "dAnimationStdAfx.h"

class dAnimIKBlendNode: public dCustomAlloc
{
	public:
	class dAnimationTransform
	{
		public:
		dVector m_posit;
		dQuaternion m_rotation;
//		dAnimIDRigEffector* m_effector;
	};

	class dAnimationPose : public dList<dAnimationTransform>
	{
		public:
		dAnimationPose(dAnimIKController* const character);

		void SetTargetPose(dAnimIKController* const character) const;
		void CopySource(const dAnimationPose& source);
	};


	dAnimIKBlendNode(dAnimIKController* const character, dAnimIKBlendNode* const child);
	virtual ~dAnimIKBlendNode();

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		if (m_child) {
			m_child->Debug(debugContext);
		}
	}

	virtual void Evaluate(dAnimationPose& output, dFloat timestep)
	{
		if (m_child) {
			m_child->Evaluate(output, timestep);
		}
	}

	dAnimIKController* m_character;
	dAnimIKBlendNode* m_child;
};

#endif