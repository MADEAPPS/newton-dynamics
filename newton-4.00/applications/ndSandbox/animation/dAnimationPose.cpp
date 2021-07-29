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
#include "dAnimationPose.h"

dAnimationPose::dAnimationPose()
	:dArray<dAnimKeyframe>()
{
}

//dAnimationPose::dAnimationPose(const dAnimationPose& source)
dAnimationPose::dAnimationPose(const dAnimationPose&)
	:dArray<dAnimKeyframe>()
{
	dAssert(0);
}

void dAnimationPose::Clear() 
{ 
//	RemoveAll(); 
}

//void dAnimationPose::CopySource(const dAnimationPose& source)
void dAnimationPose::CopySource(const dAnimationPose&)
{
	dAssert(0);
//	dListNode* destNode = GetFirst();
//	for (dListNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext()) {
//		const dAnimKeyframe& srcFrame = sourceNode->GetInfo();
//		dAnimKeyframe& dstFrame = destNode->GetInfo();
//		dAssert(srcFrame.m_userData == dstFrame.m_userData);
//		dstFrame.m_rotation = srcFrame.m_rotation;
//		dstFrame.m_posit = srcFrame.m_posit;
//		destNode = destNode->GetNext();
//	}
}

