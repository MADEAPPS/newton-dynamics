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

#ifndef __D_ANIMATION_EFFECTOR_BLEND_NODE_h__
#define __D_ANIMATION_EFFECTOR_BLEND_NODE_h__

class dAnimationEffectorBlendNode;
class dAnimationRigEffector;
class dAnimationCharacterRig;

class dAnimationTransform
{
	public:
	dVector m_posit;
	dQuaternion m_rotation;
	dAnimationRigEffector* m_effector;
};

class dAnimationPose: public dList<dAnimationTransform>
{
	public:
	dAnimationPose(dAnimationCharacterRig* const character);
};

class dAnimationEffectorBlendNode : public dCustomAlloc
{
	public:
	dAnimationEffectorBlendNode(dAnimationCharacterRig* const character);
	virtual ~dAnimationEffectorBlendNode();

	virtual void Evaluate(dAnimationPose& output, dFloat timestep)
	{
	}

//	NewtonBody* GetRootBody() const
//	{
//		return m_rootBody;
//	}

	dAnimationCharacterRig* m_character;
};


#endif