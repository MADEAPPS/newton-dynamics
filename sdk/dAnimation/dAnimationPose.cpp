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

#include "dAnimationStdAfx.h"
#include "dAnimationPose.h"

dAnimationPose::dAnimationPose()
	:dArray<dAnimKeyframe>()
{
}

dAnimationPose::dAnimationPose(const dAnimationPose& source)
	:dArray<dAnimKeyframe>()
{
//	for (dListNode* node = source.GetFirst(); node; node = node->GetNext()) {
//		Append(node->GetInfo());
//	}
}

void dAnimationPose::Clear() 
{ 
//	RemoveAll(); 
}

void dAnimationPose::CopySource(const dAnimationPose& source)
{
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

