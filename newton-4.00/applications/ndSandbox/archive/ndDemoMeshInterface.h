/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndOpenGlUtil.h"

class ndDemoMesh;
class ndDemoEntity;
class ndDemoSkinMesh;
class ndDemoEntityManager;
class ndShadowMapRenderPass;

class ndDemoSubMeshMaterial
{
	public:
	ndDemoSubMeshMaterial();
	~ndDemoSubMeshMaterial();

	GLint GetTexture() const;
	void SetTexture(GLint textureHandle);

	const char* GetTextureName() const;
	void SetTextureName(const char* const name);
	
	glVector4 m_ambient;
	glVector4 m_diffuse;
	glVector4 m_specular;
	GLfloat m_opacity;
	GLfloat m_shiness;
	protected:
	GLint m_textureHandle;
	char  m_textureName[32];
};

class ndDemoSubMesh
{
	public:
	ndDemoSubMesh();
	~ndDemoSubMesh();
	void SetOpacity(ndFloat32 opacity);

	ndDemoSubMeshMaterial m_material;
	ndInt32 m_indexCount;
	ndInt32 m_segmentStart;
	bool m_hasTranparency;
};

class ndDemoMeshInterface : public ndClassAlloc
{
	public:
	ndDemoMeshInterface();
	virtual ~ndDemoMeshInterface();
	const ndString& GetName () const;
	
	bool GetVisible () const;
	void SetVisible (bool visibilityFlag);

	virtual ndDemoMesh* GetAsDemoMesh();
	virtual ndDemoSkinMesh* GetAsDemoSkinMesh();
	virtual ndDemoMeshInterface* Clone(ndDemoEntity* const entityOwner);

	virtual void RenderShadowMap(ndShadowMapRenderPass* const, const ndMatrix&) {}
	virtual void RenderTransparency(ndDemoEntityManager* const, const ndMatrix&) {}
	virtual void Render (ndDemoEntityManager* const scene, const ndMatrix& modelMatrix) = 0;

	ndString m_name;
	bool m_isVisible;
};

inline ndDemoMesh* ndDemoMeshInterface::GetAsDemoMesh() 
{ 
	return nullptr; 
}

inline ndDemoSkinMesh* ndDemoMeshInterface::GetAsDemoSkinMesh() 
{ 
	return nullptr; 
}

inline ndDemoMeshInterface* ndDemoMeshInterface::Clone(ndDemoEntity* const) 
{ 
	ndAssert(0); 
	return nullptr; 
}


#endif 


