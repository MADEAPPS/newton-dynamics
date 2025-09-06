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
#include "ndRenderSceneNode.h"
#include "ndRenderPassShadows.h"

ndRenderPassShadows::ndRenderPassShadows(ndRender* const owner)
	:ndRenderPass(owner)
{
}

ndRenderPassShadows::~ndRenderPassShadows()
{
}

void ndRenderPassShadows::RenderScene(ndFloat32 timestep)
{
	//m_owner->m_context->SetCollorPassRenderStates();
	//
	//const ndMatrix globalMatrix(ndGetIdentityMatrix());
	//ndList<ndSharedPtr<ndRenderSceneNode>>& scene = m_owner->m_scene;
	//for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = scene.GetFirst(); node; node = node->GetNext())
	//{
	//	ndRenderSceneNode* const sceneNode = *node->GetInfo();
	//	sceneNode->Render(sceneNode->m_owner, timestep, globalMatrix);
	//}
}
