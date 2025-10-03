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
#include "ndRenderPrimitive.h"
#include "ndRenderSceneNode.h"
#include "ndRenderPrimitiveImplement.h"

ndRenderPrimitiveMaterial::ndRenderPrimitiveMaterial()
	:m_diffuse(ndFloat32(1.0f))
	,m_specular(ndFloat32(1.0f))
	,m_reflection(ndFloat32(0.5f))
	,m_specularPower(ndFloat32(250.0f))
	,m_opacity(ndFloat32(1.0f))
	,m_texture(nullptr)
	,m_castShadows(true)
{
}

ndRenderPrimitiveMaterial::ndRenderPrimitiveMaterial(const ndRenderPrimitiveMaterial& src)
	:m_diffuse(src.m_diffuse)
	,m_specular(src.m_specular)
	,m_reflection(src.m_reflection)
	,m_specularPower(src.m_specularPower)
	,m_opacity(src.m_opacity)
	,m_texture(src.m_texture)
	,m_castShadows(src.m_castShadows)
{
}

ndRenderPrimitiveSegment::ndRenderPrimitiveSegment()
	:m_material()
	,m_indexCount(0)
	,m_segmentStart(0)
	,m_hasTranparency(0)
{
}

ndRenderPrimitiveSegment::ndRenderPrimitiveSegment(const ndRenderPrimitiveSegment& src)
	:m_material(src.m_material)
	,m_indexCount(src.m_indexCount)
	,m_segmentStart(src.m_segmentStart)
	,m_hasTranparency(src.m_hasTranparency)
{
}

ndRenderPrimitive::ndDescriptor::ndDescriptor(ndRender* const render)
	:m_render(render)
	,m_meshNode(nullptr)
	,m_collision(nullptr)
	,m_skeleton(nullptr)
	,m_materials()
	,m_mapping(m_box)
	,m_uvMatrix(ndGetIdentityMatrix())
	,m_meshBuildMode(m_simplePrimitve)
	,m_numberOfInstances(0)
	,m_stretchMaping(true)
{
}

ndRenderPrimitive::ndDescriptor::ndDescriptor(const ndDescriptor& src)
	:m_render(src.m_render)
	,m_meshNode(src.m_meshNode)
	,m_collision(src.m_collision)
	,m_skeleton(nullptr)
	,m_materials()
	,m_mapping(src.m_mapping)
	,m_uvMatrix(src.m_uvMatrix)
	,m_meshBuildMode(src.m_meshBuildMode)
	,m_numberOfInstances(src.m_numberOfInstances)
	,m_stretchMaping(src.m_stretchMaping)
{
	for (ndList<ndRenderPrimitiveMaterial>::ndNode* node = src.m_materials.GetFirst(); node; node = node->GetNext())
	{
		m_materials.Append(node->GetInfo());
	}
}

ndRenderPrimitiveMaterial& ndRenderPrimitive::ndDescriptor::AddMaterial(const ndSharedPtr<ndRenderTexture>& texture)
{
	ndList<ndRenderPrimitiveMaterial>::ndNode* const node = m_materials.Append();
	node->GetInfo().m_texture = texture;
	return node->GetInfo();
}

ndRenderPrimitive::ndRenderPrimitive()
	:ndContainersFreeListAlloc<ndRenderPrimitive>()
	,m_implement(nullptr)
{
}

ndRenderPrimitive::ndRenderPrimitive(const ndDescriptor& descriptor)
	:m_implement(nullptr)
{
	m_implement = ndSharedPtr<ndRenderPrimitiveImplement>(new ndRenderPrimitiveImplement(this, descriptor));
}

ndRenderPrimitive::ndRenderPrimitive(const ndRenderPrimitive& src)
	:m_implement(nullptr)
{
	m_implement = ndSharedPtr<ndRenderPrimitiveImplement>(src.m_implement->Clone(this, nullptr));

	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = src.m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& srcSegment = node->GetInfo();
		m_segments.Append(ndRenderPrimitiveSegment(srcSegment));
	}
}

ndRenderPrimitive::ndRenderPrimitive(const ndRenderPrimitive& src, const ndRenderSceneNode* const skeleton)
	:ndContainersFreeListAlloc<ndRenderPrimitive>()
	,m_implement(nullptr)
{
	m_implement = ndSharedPtr<ndRenderPrimitiveImplement>(src.m_implement->Clone(this, skeleton));

	for (ndList<ndRenderPrimitiveSegment>::ndNode* srcNode = src.m_segments.GetFirst(); srcNode; srcNode = srcNode->GetNext())
	{
		ndRenderPrimitiveSegment& srcSegment = srcNode->GetInfo();
		m_segments.Append(ndRenderPrimitiveSegment(srcSegment));
	}
}

ndRenderPrimitive::~ndRenderPrimitive()
{
}

bool ndRenderPrimitive::IsSKinnedMesh() const
{
	return m_implement->IsSKinnedMesh();
}

void ndRenderPrimitive::UpdateSkinPaletteMatrix()
{
	m_implement->UpdateSkinPaletteMatrix();
}

void ndRenderPrimitive::Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const
{
	m_implement->Render(render, modelViewMatrix, renderPassMode);
}
