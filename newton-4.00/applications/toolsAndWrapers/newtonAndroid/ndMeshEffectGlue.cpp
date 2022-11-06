/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndNewton.h"
#include "ndMatrixGlue.h"
#include "ndMeshEffectGlue.h"
#include "ndShapeInstanceGlue.h"

ndMeshEffectGlue::ndMeshEffectGlue()
	:ndContainersFreeListAlloc<ndMeshEffectGlue>()
	,m_materialHandle(nullptr)
	,m_meshEffect(new ndMeshEffect())
{
}

ndMeshEffectGlue::ndMeshEffectGlue(const ndShapeInstanceGlue& shapeInstance)
	:ndContainersFreeListAlloc<ndMeshEffectGlue>()
	,m_materialHandle(nullptr)
	,m_meshEffect(new ndMeshEffect(*(shapeInstance.m_shapeInstance)))
{
}

ndMeshEffectGlue::~ndMeshEffectGlue()
{
	delete m_meshEffect;
}

int ndMeshEffectGlue::GetVertexSize()
{
	return int (m_meshEffect->GetPropertiesCount());
}

void ndMeshEffectGlue::GetVertexPosit(float data[], int startOffsetInfloats, int strideInFloats)
{
	m_meshEffect->GetVertexChannel(strideInFloats * sizeof(float), &data[startOffsetInfloats]);
}

void ndMeshEffectGlue::GetVertexNormal(float data[], int startOffsetInfloats, int strideInFloats)
{
	m_meshEffect->GetNormalChannel(strideInFloats * sizeof(float), &data[startOffsetInfloats]);
}

void ndMeshEffectGlue::GetVertexUV0(float data[], int startOffsetInfloats, int strideInFloats)
{
	m_meshEffect->GetUV0Channel(strideInFloats * sizeof(float), &data[startOffsetInfloats]);
}

void ndMeshEffectGlue::MaterialBegin()
{
	m_materialHandle = m_meshEffect->MaterialGeometryBegin();
}

int ndMeshEffectGlue::GetFirstMaterial()
{
	ndAssert(m_materialHandle);
	return m_meshEffect->GetFirstMaterial(m_materialHandle);
}

int ndMeshEffectGlue::GetNextMaterial(int currentMaterial)
{
	ndAssert(m_materialHandle);
	return m_meshEffect->GetNextMaterial(m_materialHandle, currentMaterial);
}

int ndMeshEffectGlue::GetMaterialIndexCount(int materialIndex)
{
	ndAssert(m_materialHandle);
	return m_meshEffect->GetMaterialIndexCount(m_materialHandle, materialIndex);
}

void ndMeshEffectGlue::GetMaterialGetIndexStream(int materialIndex, short data[], int startOffsetInShorts)
{
	ndAssert(m_materialHandle);
	m_meshEffect->GetMaterialGetIndexStream(m_materialHandle, materialIndex, &data[startOffsetInShorts]);
}
	
void ndMeshEffectGlue::MaterialEnd()
{
	ndAssert(m_materialHandle);
	m_meshEffect->MaterialGeometryEnd(m_materialHandle);
	m_materialHandle = nullptr;
}

void ndMeshEffectGlue::SphericalMapping(int materialIndex, const ndMatrixGlue& textureMatrix)
{
	m_meshEffect->SphericalMapping(materialIndex, textureMatrix);
}

void ndMeshEffectGlue::UniformBoxMapping(int materialIndex, const ndMatrixGlue& textureMatrix)
{
	m_meshEffect->UniformBoxMapping(materialIndex, textureMatrix);
}

