/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __DEMO_DEBRI_ENTITY_H__
#define __DEMO_DEBRI_ENTITY_H__

#include "ndDemoMesh.h"
#include "ndDemoEntity.h"


class ndDemoDebriEntity;



class ndDemoDebriMesh____ : public ndDemoMesh
{
	public:
	ndDemoDebriMesh____(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);

	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	ndDemoSubMeshMaterial m_material[2];
	dInt32 m_textureLocation1;
};






class ndDemoDebriMesh : public ndDemoMesh
{
	public:
	//ndDemoDebriMesh(const char* const name, const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat32 opacity = 1.0f, const dMatrix& uvMatrix = dGetIdentityMatrix());
	ndDemoDebriMesh(const char* const name, dArray<ndMeshPointUV>& vertexArrayOut, dArray<dInt32>& indexArrayOut, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);
	~ndDemoDebriMesh();

	//virtual void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	//void SetTransforms(dInt32 count, const dMatrix* const matrixArray);

	private:
	//void RenderBatch(dInt32 start, ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	//
	//const dMatrix* m_offsets;
	//dInt32 m_instanceCount;
	//dInt32 m_maxInstanceCount;
	//GLuint m_matrixOffsetBuffer;
	
};


class ndDemoDebriEntity: public ndDemoEntity
{
	public:
	ndDemoDebriEntity();
	ndDemoDebriEntity(const ndDemoDebriEntity& copyFrom);
	virtual ~ndDemoDebriEntity(void);

	void FinalizeConstruction(dArray<ndMeshPointUV>& vertexArrayOut, dArray<dInt32>& indexArrayOut);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	//ndDemoMeshIntance* m_instanceMesh;
};

#endif