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

#ifndef __DEMO_DEBRIS_ENTITY_H__
#define __DEMO_DEBRIS_ENTITY_H__

#include "ndDemoMesh.h"
#include "ndDemoEntity.h"


class glDebrisPoint
{
	public:
	glVector4 m_posit;
	glVector3 m_normal;
	glUV m_uv;
};

class ndDemoDebrisRootEntity;

class ndDemoDebrisMesh : public ndDemoMesh
{
	public:
	ndDemoDebrisMesh(ndDemoDebrisMesh* const srcMesh, const ndArray<glDebrisPoint>& vertexArray);
	ndDemoDebrisMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache, ndInt32 offsetBase, ndArray<glDebrisPoint>& vertexArray);

	void Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);

	private:
	ndDemoSubMeshMaterial m_material[2];
	ndInt32 m_textureLocation1;

	friend class ndDemoDebrisRootEntity;
};

class ndDemoDebrisRootEntity: public ndDemoEntity
{
	public:
	ndDemoDebrisRootEntity();
	ndDemoDebrisRootEntity(const ndDemoDebrisRootEntity& copyFrom);
	virtual ~ndDemoDebrisRootEntity(void);

	void FinalizeConstruction(const ndArray<glDebrisPoint>& vertexArray);

	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	//ndInt32 m_vertexCount;
	//ndInt32 m_buffRefCount;
	//GLuint m_vertexBuffer;
	//GLuint m_vertextArrayBuffer;
};

class ndDemoDebrisEntity : public ndDemoEntity
{
	public:
	ndDemoDebrisEntity(ndMeshEffect* const meshNode, ndArray<glDebrisPoint>& vertexArray, ndDemoDebrisRootEntity* const parent, const ndShaderPrograms& shaderCache);
	ndDemoDebrisEntity(const ndDemoDebrisEntity& copyFrom);
	virtual ~ndDemoDebrisEntity();
	ndNodeBaseHierarchy* CreateClone() const;

	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	//ndInt32 m_vertexOffsetBase;
};


#endif