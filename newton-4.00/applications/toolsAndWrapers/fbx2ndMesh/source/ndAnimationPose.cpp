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

#include "ndModelStdafx.h"
#include "ndAnimationPose.h"

ndAnimationPose::ndAnimationPose()
	:ndArray<ndAnimKeyframe>()
{
}

//ndAnimationPose::ndAnimationPose(const ndAnimationPose& source)
ndAnimationPose::ndAnimationPose(const ndAnimationPose&)
	:ndArray<ndAnimKeyframe>()
{
	ndAssert(0);
}

void ndAnimationPose::Clear() 
{ 
//	RemoveAll(); 
}

void ndAnimationPose::CopySource(const ndAnimationPose& src)
{
	SetCount(0);
	for (ndInt32 i = 0; i < src.GetCount(); ++i)
	{
		PushBack(src[i]);
	}
}

