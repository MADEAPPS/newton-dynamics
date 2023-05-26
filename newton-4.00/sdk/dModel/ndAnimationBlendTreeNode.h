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

#ifndef __D_ANIM_ID_BLENDTREE_NODE_h__
#define __D_ANIM_ID_BLENDTREE_NODE_h__

class ndAnimationPose;

class ndAnimationBlendTreeNode: public ndClassAlloc
{
	public:
	ndAnimationBlendTreeNode(ndAnimationBlendTreeNode* const input);
	virtual ~ndAnimationBlendTreeNode();

	virtual void Evaluate(ndAnimationPose& output)
	{
		if (m_input) 
		{
			m_input->Evaluate(output);
		}
	}

	ndAnimationBlendTreeNode* m_input;
};

#endif