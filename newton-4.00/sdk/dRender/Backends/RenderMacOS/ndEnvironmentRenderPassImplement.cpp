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
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderShaderCache.h"
#include "ndRenderTextureImage.h"
#include "ndEnvironmentRenderPassImplement.h"

ndEnvironmentRenderPassImplement::ndEnvironmentRenderPassImplement(ndRenderContext* const context)
	:ndClassAlloc()
	,m_context(context)
{
}

ndEnvironmentRenderPassImplement::~ndEnvironmentRenderPassImplement()
{
}

void ndEnvironmentRenderPassImplement::RenderScene(const ndRenderSceneCamera* const camera, const ndRenderTexture* const texture)
{
}

