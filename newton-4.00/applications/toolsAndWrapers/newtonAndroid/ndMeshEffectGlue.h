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

#ifndef _ND_MESH_EFFECT_GLUE_H_
#define _ND_MESH_EFFECT_GLUE_H_

#include "ndMeshEffect.h"
#include "ndShapeInstance.h"
#include "ndShapeInstanceGlue.h"

class ndMeshEffectGlue
{
	public:
	ndMeshEffectGlue()
		:m_materialHandle(nullptr)
		,m_meshEffect(new ndMeshEffect())
	{
	}

	ndMeshEffectGlue(const ndShapeInstanceGlue& shapeInstance)
		:m_materialHandle(nullptr)
		,m_meshEffect(new ndMeshEffect(*shapeInstance.m_instance))
	{
	}

	~ndMeshEffectGlue()
	{
	}

	int GetVertexSize()
	{
		return int (m_meshEffect->GetPropertiesCount());
	}

	void GetVertexPosit(float data[], int startOffsetInfloats, int strideInFloats)
	{
		m_meshEffect->GetVertexChannel(strideInFloats * sizeof(float), &data[startOffsetInfloats]);
	}

	void GetVertexNormal(float data[], int startOffsetInfloats, int strideInFloats)
	{
		m_meshEffect->GetNormalChannel(strideInFloats * sizeof(float), &data[startOffsetInfloats]);
	}

	void GetVertexUV0(float data[], int startOffsetInfloats, int strideInFloats)
	{
		m_meshEffect->GetUV0Channel(strideInFloats * sizeof(float), &data[startOffsetInfloats]);
	}

	void MaterialBegin()
	{
		m_materialHandle = m_meshEffect->MaterialGeometryBegin();
	}

	int GetFirstMaterial()
	{
		ndAssert(m_materialHandle);
		return m_meshEffect->GetFirstMaterial(m_materialHandle);
	}

	int GetNextMaterial(int currentMaterial)
	{
		ndAssert(m_materialHandle);
		return m_meshEffect->GetNextMaterial(m_materialHandle, currentMaterial);
	}

	int GetMaterialIndexCount(int materialIndex)
	{
		ndAssert(m_materialHandle);
		return m_meshEffect->GetMaterialIndexCount(m_materialHandle, materialIndex);
	}

	void GetMaterialGetIndexStream(int materialIndex, short data[], int startOffsetInShorts)
	{
		ndAssert(m_materialHandle);
		m_meshEffect->GetMaterialGetIndexStream(m_materialHandle, materialIndex, &data[startOffsetInShorts]);
	}
	
	void MaterialEnd()
	{
		ndAssert(m_materialHandle);
		m_meshEffect->MaterialGeometryEnd(m_materialHandle);
		m_materialHandle = nullptr;
	}

	ndIndexArray* m_materialHandle;
	ndSharedPtr<ndMeshEffect> m_meshEffect;
};

#endif 

