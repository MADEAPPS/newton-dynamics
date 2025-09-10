/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderSceneNodeInstance.h"

ndRenderSceneNodeInstance::ndRenderSceneNodeInstance(const ndMatrix& matrix)
	:ndRenderSceneNode(matrix)
{
}

void ndRenderSceneNodeInstance::Render(const ndRender* const owner, ndFloat32 timeStep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const
{
	ndRenderSceneNode::Render(owner, timeStep, parentMatrix, renderMode);
}
