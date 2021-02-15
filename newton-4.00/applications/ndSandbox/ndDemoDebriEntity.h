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


struct DebriPoint
{
	ndMeshVector4 m_posit;
	ndMeshVector m_normal;
	ndMeshUV m_uv;
};

class ndDemoDebriEntityRoot;

class ndDemoDebriMesh: public ndDemoMesh
{
	public:
	ndDemoDebriMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);

	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	ndDemoSubMeshMaterial m_material[2];
	dInt32 m_textureLocation1;
};

class ndDemoDebriMesh2 : public ndDemoMesh
{
	public:
	ndDemoDebriMesh2(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);
	~ndDemoDebriMesh2();

	private:
	ndDemoSubMeshMaterial m_material[2];
	dInt32 m_textureLocation1;
};

class ndDemoDebriEntityRoot: public ndDemoEntity
{
	public:
	ndDemoDebriEntityRoot();
	ndDemoDebriEntityRoot(const ndDemoDebriEntityRoot& copyFrom);
	virtual ~ndDemoDebriEntityRoot(void);

	void FinalizeConstruction(dArray<DebriPoint>& vertexArray);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	dInt32 m_vertexCount;
	dInt32 m_buffRefCount;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
};

class ndDemoDebriEntity : public ndDemoEntity
{
	public:
	ndDemoDebriEntity(ndMeshEffect* const meshNode, dArray<DebriPoint>& vertexArray, ndDemoDebriEntityRoot* const parent, const ndShaderPrograms& shaderCache);
	ndDemoDebriEntity(const ndDemoDebriEntity& copyFrom);
	virtual ~ndDemoDebriEntity();
	dNodeBaseHierarchy* CreateClone() const;

	dInt32 m_vertexOffestBase;
};


#endif