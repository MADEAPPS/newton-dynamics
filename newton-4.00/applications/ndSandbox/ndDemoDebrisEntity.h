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
	ndDemoDebrisMesh(ndDemoDebrisMesh* const srcMesh, const dArray<glDebrisPoint>& vertexArray);
	ndDemoDebrisMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache, dInt32 offsetBase, dArray<glDebrisPoint>& vertexArray);

	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	private:
	ndDemoSubMeshMaterial m_material[2];
	dInt32 m_textureLocation1;

	friend class ndDemoDebrisRootEntity;
};

class ndDemoDebrisRootEntity: public ndDemoEntity
{
	public:
	ndDemoDebrisRootEntity();
	ndDemoDebrisRootEntity(const ndDemoDebrisRootEntity& copyFrom);
	virtual ~ndDemoDebrisRootEntity(void);

	void FinalizeConstruction(const dArray<glDebrisPoint>& vertexArray);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	//dInt32 m_vertexCount;
	//dInt32 m_buffRefCount;
	//GLuint m_vertexBuffer;
	//GLuint m_vertextArrayBuffer;
};

class ndDemoDebrisEntity : public ndDemoEntity
{
	public:
	ndDemoDebrisEntity(ndMeshEffect* const meshNode, dArray<glDebrisPoint>& vertexArray, ndDemoDebrisRootEntity* const parent, const ndShaderPrograms& shaderCache);
	ndDemoDebrisEntity(const ndDemoDebrisEntity& copyFrom);
	virtual ~ndDemoDebrisEntity();
	dNodeBaseHierarchy* CreateClone() const;

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	//dInt32 m_vertexOffsetBase;
};


#endif