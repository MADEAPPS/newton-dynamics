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
#include "ndRenderPassShadowsImplement.h"

ndRenderPassShadows::ndRenderPassShadows(ndRender* const owner)
	:ndRenderPass(owner)
	,m_implement(new ndRenderPassShadowsImplement(*m_owner->m_context))
{
	owner->m_cachedShadowPass = *m_implement;
}

ndRenderPassShadows::~ndRenderPassShadows()
{
	m_owner->m_cachedShadowPass = nullptr;
}

void ndRenderPassShadows::RenderScene(ndFloat32)
{
	//const ndRenderSceneCamera* const camera = *m_owner->m_camera;
	const ndRenderSceneCamera* const camera = m_owner->m_camera->FindCameraNode();
	m_implement->RenderScene(camera);
}
