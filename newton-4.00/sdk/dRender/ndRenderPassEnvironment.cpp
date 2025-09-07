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
#include "ndRenderTexture.h"
#include "ndRenderPassEnvironment.h"
#include "ndRenderPassEnvironmentImplement.h"

ndRenderPassEnvironment::ndRenderPassEnvironment(ndRender* const owner, ndSharedPtr<ndRenderTexture>& cubeMap)
	:ndRenderPass(owner)
	,m_cubeMap(cubeMap)
	,m_implement(new ndRenderPassEnvironmentImplement(*m_owner->m_context))
{
	owner->m_cachedEnvironmentPass = this;
}

ndRenderPassEnvironment::~ndRenderPassEnvironment()
{
	m_owner->m_cachedShadowPass = nullptr;
}

void ndRenderPassEnvironment::RenderScene(ndFloat32)
{
	const ndRenderSceneCamera* const camera = *m_owner->m_camera;
	m_implement->RenderScene(camera, *m_cubeMap);
}

