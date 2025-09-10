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
#include "ndRenderTexture.h"
#include "ndRenderPrimitiveMesh.h"
#include "ndRenderPassShadowsImplement.h"
#include "ndRenderPrimitiveMeshImplement.h"

ndRenderPrimitiveMeshMaterial::ndRenderPrimitiveMeshMaterial()
	:m_diffuse(ndFloat32(1.0f))
	,m_specular(ndFloat32(1.0f))
	,m_reflection(ndFloat32(0.15f))
	,m_specularPower(ndFloat32(250.0f))
	,m_opacity(ndFloat32(1.0f))
	,m_texture(nullptr)
	,m_castShadows(true)
{
}

ndRenderPrimitiveMeshSegment::ndRenderPrimitiveMeshSegment()
	:m_material()
	,m_indexCount(0)
	,m_segmentStart(0)
	,m_hasTranparency(0)
{
}

ndRenderPrimitiveMesh::ndRenderPrimitiveMesh()
	:ndRenderPrimitive()
	,m_implementation(nullptr)
{
}

ndRenderPrimitiveMesh::~ndRenderPrimitiveMesh()
{
}

ndSharedPtr<ndRenderPrimitive> ndRenderPrimitiveMesh::CreateSetZbufferCollisionShape(
	const ndRender* const render,
	const ndShapeInstance* const collision)
{
	ndRenderPrimitiveMesh* const primtive = new ndRenderPrimitiveMesh();
	primtive->m_implementation = ndSharedPtr<ndRenderPrimitiveMeshImplement>(new ndRenderPrimitiveMeshImplement(primtive, render, collision, ndRenderPrimitiveMeshImplement::m_hidenLines));
	return ndSharedPtr<ndRenderPrimitive>(primtive);
}

ndSharedPtr<ndRenderPrimitive> ndRenderPrimitiveMesh::CreateWireFrameFromCollisionShape(
	const ndRender* const render,
	const ndShapeInstance* const collision)
{
	ndRenderPrimitiveMesh* const primtive = new ndRenderPrimitiveMesh();
	primtive->m_implementation = ndSharedPtr<ndRenderPrimitiveMeshImplement>(new ndRenderPrimitiveMeshImplement(primtive, render, collision, ndRenderPrimitiveMeshImplement::m_wireFrame));
	return ndSharedPtr<ndRenderPrimitive>(primtive);
}

ndSharedPtr<ndRenderPrimitive> ndRenderPrimitiveMesh::CreateFromCollisionShape(
	const ndRender* const render,
	const ndShapeInstance* const collision)
{
	ndRenderPrimitiveMesh* const primtive = new ndRenderPrimitiveMesh();
	primtive->m_implementation = ndSharedPtr<ndRenderPrimitiveMeshImplement>(new ndRenderPrimitiveMeshImplement(primtive, render, collision, ndRenderPrimitiveMeshImplement::m_solid));
	return ndSharedPtr<ndRenderPrimitive>(primtive);
}

ndSharedPtr<ndRenderPrimitive> ndRenderPrimitiveMesh::CreateFromCollisionShape(
	const ndRender* const render,
	const ndShapeInstance* const collision,
	const ndRenderPrimitiveMeshMaterial& material,
	ndUvMapingMode mapping,
	const ndMatrix& uvMatrix, bool stretchMaping)
{
	ndRenderPrimitiveMesh* const primtive = new ndRenderPrimitiveMesh();
	primtive->m_implementation = ndSharedPtr<ndRenderPrimitiveMeshImplement>(new ndRenderPrimitiveMeshImplement(primtive, render, collision, material, mapping, uvMatrix, stretchMaping));
	return ndSharedPtr<ndRenderPrimitive>(primtive);
}

void ndRenderPrimitiveMesh::Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const
{
	m_implementation->Render(render, modelViewMatrix, renderPassMode);
}
