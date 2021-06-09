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
#include "ndDemoMeshInterface.h"

class ndDemoMesh;
class ndDemoEntity;
class ndShaderPrograms;
class ndDemoEntityManager;


class ndDemoSkinMesh: public ndDemoMeshInterface
{
	public:
	struct dWeightBoneIndex
	{
		dInt32 m_boneIndex[4];
	};

	ndDemoSkinMesh(const ndDemoSkinMesh& clone, ndDemoEntity* const owner);
	//ndDemoSkinMesh(dScene* const scene, ndDemoEntity* const owner, dScene::dNode* const meshNode, const dTree<ndDemoEntity*, dScene::dNode*>& boneMap, const ndShaderPrograms& shaderCache);
	~ndDemoSkinMesh();

	void Render (ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	//NewtonMesh* CreateNewtonMesh(NewtonWorld* const world, const dMatrix& meshMatrix);

	protected: 
	virtual ndDemoMeshInterface* Clone(ndDemoEntity* const owner);
	dInt32 CalculateMatrixPalette(dMatrix* const bindMatrix) const;
	void ConvertToGlMatrix(dInt32 count, const dMatrix* const bindMatrix, GLfloat* const glMatrices) const;
	//dGeometryNodeSkinClusterInfo* FindSkinModifier(dScene* const scene, dScene::dNode* const meshNode) const;
	void OptimizeForRender(const ndDemoSubMesh& segment, const dVector* const pointWeights, const dWeightBoneIndex* const pointSkinBone) const;

	ndDemoMesh* m_mesh;
	ndDemoEntity* m_entity; 
	dMatrix* m_bindingMatrixArray;
	dInt32 m_nodeCount; 
	dInt32 m_shader;
};

#endif 


