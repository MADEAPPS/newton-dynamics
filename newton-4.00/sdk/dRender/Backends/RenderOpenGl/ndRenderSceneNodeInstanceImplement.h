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
#ifndef __ND_RENDER_SCENE_NODE_INSTANCE_IMPLEMENT_H__
#define __ND_RENDER_SCENE_NODE_INSTANCE_IMPLEMENT_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"
#include "ndRenderSceneNode.h"
#include "ndRenderOpenGlUtil.h"

class ndRenderSceneNodeInstance;
class ndRenderSceneNodeInstanceImplement : public ndContainersFreeListAlloc<ndRenderSceneNodeInstanceImplement>
{
	public:
	ndRenderSceneNodeInstanceImplement(ndRenderSceneNodeInstance* const owner);

	void Finalize();
	void Render(const ndRender* const owner, ndFloat32 timeStep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const;

	ndRenderSceneNodeInstance* m_owner;
};

#endif