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

#include "ndSandboxStdafx.h"
#include "ndAnimationPose.h"

ndAnimationPose::ndAnimationPose()
	:ndArray<ndAnimKeyframe>()
{
}

//ndAnimationPose::ndAnimationPose(const ndAnimationPose& source)
ndAnimationPose::ndAnimationPose(const ndAnimationPose&)
	:ndArray<ndAnimKeyframe>()
{
	dAssert(0);
}

void ndAnimationPose::Clear() 
{ 
//	RemoveAll(); 
}

//void ndAnimationPose::CopySource(const ndAnimationPose& source)
void ndAnimationPose::CopySource(const ndAnimationPose&)
{
	dAssert(0);
//	dListNode* destNode = GetFirst();
//	for (dListNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext()) {
//		const ndAnimKeyframe& srcFrame = sourceNode->GetInfo();
//		ndAnimKeyframe& dstFrame = destNode->GetInfo();
//		dAssert(srcFrame.m_userData == dstFrame.m_userData);
//		dstFrame.m_rotation = srcFrame.m_rotation;
//		dstFrame.m_posit = srcFrame.m_posit;
//		destNode = destNode->GetNext();
//	}
}

