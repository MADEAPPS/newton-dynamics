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

#ifndef _D_DEMO_SKIN_MESH_H_
#define _D_DEMO_SKIN_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"

class ndDemoMesh;
class ndDemoEntity;
class glSkinVertex;
class ndShaderPrograms;
class ndDemoEntityManager;

class ndDemoSkinMesh: public ndDemoMesh
{
	public:
	struct dWeightBoneIndex
	{
		dInt32 m_boneIndex[4];
	};

	ndDemoSkinMesh(ndDemoEntity* const owner, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);
	~ndDemoSkinMesh();

	protected: 
	virtual ndDemoMeshInterface* Clone(ndDemoEntity* const owner);
	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	void CreateRenderMesh(
		const glSkinVertex* const points, dInt32 pointCount,
		const dInt32* const indices, dInt32 indexCount);

	ndDemoEntity* m_entity; 
	dArray<glMatrix> m_bindingMatrixArray;
	dInt32 m_nodeCount; 
};

#endif 


