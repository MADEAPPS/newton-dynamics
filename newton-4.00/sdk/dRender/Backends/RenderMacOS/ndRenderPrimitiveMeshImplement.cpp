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
#include "ndRenderShaderCache.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPrimitiveMesh.h"

#include "ndRenderPrimitiveMeshImplement.h"

ndRenderPrimitiveMeshImplement::ndRenderPrimitiveMeshImplement(
	const ndRender* const render,
	const ndShapeInstance* const collision, 
	const ndRenderPrimitiveMeshMaterial& material,
	ndRenderPrimitiveMesh::ndUvMapingMode mapping,
	const ndMatrix& uvMatrix, 
	bool stretchMaping)
	:ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>()
	,m_context(*render->m_context)
	,m_segments()
{
}

ndRenderPrimitiveMeshImplement::~ndRenderPrimitiveMeshImplement()
{
	ResetOptimization();
}

void ndRenderPrimitiveMeshImplement::ResetOptimization()
{
}

void ndRenderPrimitiveMeshImplement::OptimizeForRender(
	const glPositionNormalUV* const points, ndInt32 pointCount,
	const ndInt32* const indices, ndInt32 indexCount)
{
}

void ndRenderPrimitiveMeshImplement::Render(const ndRender* const render, const ndMatrix& modelMatrix) const
{
}