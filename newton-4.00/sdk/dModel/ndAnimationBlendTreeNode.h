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

#ifndef __ND_ANIM_ID_BLENDTREE_NODE_h__
#define __ND_ANIM_ID_BLENDTREE_NODE_h__

class ndAnimationPose;

class ndAnimationBlendTreeNode: public ndContainersFreeListAlloc<ndAnimationBlendTreeNode>
{
	public:
	ndAnimationBlendTreeNode(const ndSharedPtr<ndAnimationBlendTreeNode>& input);
	virtual ~ndAnimationBlendTreeNode();

	virtual void SetTime(ndFloat32 dt);
	virtual void Update(ndFloat32 dt);
	virtual void Evaluate(ndAnimationPose& output, ndVector& veloc);

	//ndAnimationBlendTreeNode* m_input;
	ndSharedPtr<ndAnimationBlendTreeNode> m_input;
};

#endif