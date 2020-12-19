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

#ifndef _D_DEMO_MESH_H_
#define _D_DEMO_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoMeshInterface.h"

class ndDemoMesh;
class ndDemoEntity;
class ndShaderPrograms;
class ndDemoEntityManager;

class ndDemoMesh: public ndDemoMeshInterface, public dList<ndDemoSubMesh>
{
	public:
	ndDemoMesh(const char* const name);
	ndDemoMesh(const ndDemoMesh& mesh, const ndShaderPrograms& shaderCache);
	ndDemoMesh(const char* const name, dMeshEffect* const meshNode, const ndShaderPrograms& shaderCache);
	ndDemoMesh(const char* const name, const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat32 opacity = 1.0f, const dMatrix& uvMatrix = dGetIdentityMatrix());

	virtual ndDemoMeshInterface* Clone(ndDemoEntity* const owner) 
	{ 
		AddRef(); 
		return this;
	}

	ndDemoSubMesh* AddSubMesh();
	virtual const char* GetTextureName (const ndDemoSubMesh* const subMesh) const;

	virtual void RenderNormals();
	virtual void Render (ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	virtual void RenderTransparency(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	void OptimizeForRender(const dArray<ndMeshPointUV>& points, const dArray<dInt32>& indices);

	void GetVertexArray(dArray<dVector>& points) const;

	protected:
	virtual ~ndDemoMesh();
	void ResetOptimization();
	void RenderGeometry(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	dInt32 m_indexCount;
	dInt32 m_vertexCount;
	dInt32 m_textureLocation;
	dInt32 m_transparencyLocation;
	dInt32 m_normalMatrixLocation;
	dInt32 m_projectMatrixLocation;
	dInt32 m_viewModelMatrixLocation;
	dInt32 m_directionalLightDirLocation;

	dInt32 m_materialAmbientLocation;
	dInt32 m_materialDiffuseLocation;
	dInt32 m_materialSpecularLocation;

	GLuint m_shader;
	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vetextArrayBuffer;
	bool m_hasTransparency;
	friend class ndDemoEntity;
};

#endif 


