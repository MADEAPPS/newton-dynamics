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

class ndDemoDebriRootEntity;

class ndDemoDebriMesh : public ndDemoMesh
{
	public:
	ndDemoDebriMesh(ndDemoDebriMesh* const srcMesh, const dArray<DebriPoint>& vertexArray);
	ndDemoDebriMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache, dInt32 offsetBase, dArray<DebriPoint>& vertexArray);

	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	private:
	ndDemoSubMeshMaterial m_material[2];
	dInt32 m_textureLocation1;

	friend class ndDemoDebriRootEntity;
};

class ndDemoDebriRootEntity: public ndDemoEntity
{
	public:
	ndDemoDebriRootEntity();
	ndDemoDebriRootEntity(const ndDemoDebriRootEntity& copyFrom);
	virtual ~ndDemoDebriRootEntity(void);

	void FinalizeConstruction(const dArray<DebriPoint>& vertexArray);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	//dInt32 m_vertexCount;
	//dInt32 m_buffRefCount;
	//GLuint m_vertexBuffer;
	//GLuint m_vertextArrayBuffer;
};

class ndDemoDebriEntity : public ndDemoEntity
{
	public:
	ndDemoDebriEntity(ndMeshEffect* const meshNode, dArray<DebriPoint>& vertexArray, ndDemoDebriRootEntity* const parent, const ndShaderPrograms& shaderCache);
	ndDemoDebriEntity(const ndDemoDebriEntity& copyFrom);
	virtual ~ndDemoDebriEntity();
	dNodeBaseHierarchy* CreateClone() const;

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	//dInt32 m_vertexOffsetBase;
};


#endif