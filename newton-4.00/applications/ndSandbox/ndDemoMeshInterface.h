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

#ifndef _D_MESH_INTERFACE_H_
#define _D_MESH_INTERFACE_H_

#include "ndSandboxStdafx.h"

class ndDemoEntity;
class ndDemoEntityManager;

class ndMeshVector
{
	public:
	ndMeshVector() {}
	ndMeshVector(GLfloat x, GLfloat y, GLfloat z)
		:m_x(x), m_y(y), m_z(z)
	{
	}

	GLfloat m_x;
	GLfloat m_y;
	GLfloat m_z;
};

class ndMeshVector4: public ndMeshVector
{
	public:
	GLfloat m_w;
};

class ndMeshMatrix
{
	public:
	ndMeshVector4 m_array[4];
};

class ndMeshUV
{
	public:
	GLfloat m_u;
	GLfloat m_v;
};

class ndPointNormal
{
	public:
	ndMeshVector m_posit;
	ndMeshVector m_normal;
};

class ndMeshPointUV: public ndPointNormal
{
	public:
	ndMeshUV m_uv;
};

class ndDemoSubMeshMaterial
{
	public:
	ndDemoSubMeshMaterial();
	~ndDemoSubMeshMaterial();

	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	dFloat32 m_opacity;
	dFloat32 m_shiness;
	dUnsigned32 m_textureHandle;
	char  m_textureName[32];
};

class ndDemoSubMesh
{
	public:
	ndDemoSubMesh();
	~ndDemoSubMesh();
	void SetOpacity(dFloat32 opacity);

	ndDemoSubMeshMaterial m_material;
	dInt32 m_indexCount;
	dInt32 m_segmentStart;
	bool m_hasTranparency;
};

class ndDemoMeshInterface: public dClassAlloc, public dRefCounter<ndDemoMeshInterface>
{
	public:
	ndDemoMeshInterface();
	~ndDemoMeshInterface();
	const dString& GetName () const;

	dInt32 Release();
	bool GetVisible () const;
	void SetVisible (bool visibilityFlag);

	virtual ndDemoMeshInterface* Clone(ndDemoEntity* const owner) { dAssert(0); return nullptr; }

	virtual void Render (ndDemoEntityManager* const scene, const dMatrix& modelMatrix) = 0;
	virtual void RenderTransparency(ndDemoEntityManager* const scene, const dMatrix& modelMatrix) {}

	dString m_name;
	bool m_isVisible;
};


#endif 


