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

//class ndDemoSkinMesh: public ndDemoMesh
class ndDemoSkinMesh: public ndDemoMeshInterface
{
	public:
	struct dWeightBoneIndex
	{
		ndInt32 m_boneIndex[4];
	};

	ndDemoSkinMesh(const ndDemoSkinMesh& source, ndDemoEntity* const owner);
	ndDemoSkinMesh(ndDemoEntity* const owner, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);
	~ndDemoSkinMesh();

	protected: 
	virtual ndDemoMeshInterface* Clone(ndDemoEntity* const owner);
	void Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	void CreateRenderMesh(
		const glSkinVertex* const points, ndInt32 pointCount,
		const ndInt32* const indices, ndInt32 indexCount);

	ndInt32 CalculateMatrixPalette(ndMatrix* const bindMatrix) const;

	ndDemoMesh* m_shareMesh;
	ndDemoEntity* m_ownerEntity; 
	ndArray<ndMatrix> m_bindingMatrixArray;
	GLuint m_shader;
	ndInt32 m_nodeCount; 
	ndInt32 m_matrixPalette;
};

#endif 


