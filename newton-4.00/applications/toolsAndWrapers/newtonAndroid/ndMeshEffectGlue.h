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

#include "ndClassAlloc.h"
#include "ndContainersAlloc.h"

class ndIndexArray;
class ndMeshEffect;
class ndMatrixGlue;
class ndShapeInstanceGlue;

class ndMeshEffectGlue : public ndContainersFreeListAlloc<ndMeshEffectGlue>
{
	public:
	ndMeshEffectGlue();
	ndMeshEffectGlue(const ndShapeInstanceGlue& shapeInstance);
	~ndMeshEffectGlue();
	int GetVertexSize();
	void GetVertexPosit(float data[], int startOffsetInfloats, int strideInFloats);
	void GetVertexNormal(float data[], int startOffsetInfloats, int strideInFloats);
	void GetVertexUV0(float data[], int startOffsetInfloats, int strideInFloats);
	void MaterialBegin();
	int GetFirstMaterial();
	int GetNextMaterial(int currentMaterial);
	int GetMaterialIndexCount(int materialIndex);
	void GetMaterialGetIndexStream(int materialIndex, short data[], int startOffsetInShorts);
	void MaterialEnd();

	void SphericalMapping(int materialIndex, const ndMatrixGlue& textureMatrix);
	void UniformBoxMapping(int materialIndex, const ndMatrixGlue& textureMatrix);

	private:
	ndMeshEffect* m_meshEffect;
	ndIndexArray* m_materialHandle;
};

#endif 

