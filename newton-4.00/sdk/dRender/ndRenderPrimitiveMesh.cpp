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
#include "ndRenderSceneNode.h"
#include "ndRenderPrimitiveMesh.h"
#include "ndRenderPassShadowsImplement.h"
#include "ndRenderPrimitiveMeshImplement.h"


ndRenderPrimitiveMeshSegment::ndRenderPrimitiveMeshSegment()
	:m_material()
	,m_indexCount(0)
	,m_segmentStart(0)
	,m_hasTranparency(0)
{
}

ndRenderPrimitiveMeshSegment::ndRenderPrimitiveMeshSegment(const ndRenderPrimitiveMeshSegment& src)
	:m_material(src.m_material)
	,m_indexCount(src.m_indexCount)
	,m_segmentStart(src.m_segmentStart)
	,m_hasTranparency(src.m_hasTranparency)
{
}

ndRenderPrimitiveMesh::ndRenderPrimitiveMesh()
	:ndRenderPrimitive()
	,m_implement(nullptr)
{
}

ndRenderPrimitiveMesh::ndRenderPrimitiveMesh(const ndRenderPrimitiveMesh& src)
	:ndRenderPrimitive(src)
	,m_implement(nullptr)
{
	m_implement = ndSharedPtr<ndRenderPrimitiveMeshImplement>(src.m_implement->Clone(this));

	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = src.m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		m_segments.Append(ndRenderPrimitiveMeshSegment(segment));
	}
}

ndRenderPrimitiveMesh::~ndRenderPrimitiveMesh()
{
}

ndRenderPrimitive* ndRenderPrimitiveMesh::Clone()
{
	return new ndRenderPrimitiveMesh(*this);
}

ndSharedPtr<ndRenderPrimitive> ndRenderPrimitiveMesh::CreateMeshPrimitive(const ndDescriptor& descriptor)
{
	ndRenderPrimitiveMesh* const primitive = new ndRenderPrimitiveMesh();
	primitive->m_implement = ndSharedPtr<ndRenderPrimitiveMeshImplement>(new ndRenderPrimitiveMeshImplement(primitive, descriptor));
	return ndSharedPtr<ndRenderPrimitive>(primitive);
}

bool ndRenderPrimitiveMesh::IsSKinnedMesh() const
{
	//return m_implement->m_skinSceneNode ? true : false;
	return m_implement->IsSKinnedMesh();
}

void ndRenderPrimitiveMesh::Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const
{
	m_implement->Render(render, modelViewMatrix, renderPassMode);
}
