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

#ifndef _D_DEMO_MESH_H_
#define _D_DEMO_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoMeshInterface.h"

class ndDemoMesh;
class ndDemoEntity;
class ndShaderCache;
class ndDemoEntityManager;

class ndDemoMesh: public ndDemoMeshInterface, public ndList<ndDemoSubMesh>
{
	public:
	ndDemoMesh(const char* const name);
	ndDemoMesh(const ndDemoMesh& mesh, const ndShaderCache& shaderCache);
	ndDemoMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderCache& shaderCache);
	ndDemoMesh(const char* const name, const ndShaderCache& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const texture1, const char* const texture2, ndFloat32 opacity = 1.0f, const ndMatrix& uvMatrix = ndGetIdentityMatrix(), bool stretchMaping = true);
	virtual ~ndDemoMesh();

	ndDemoMesh* GetAsDemoMesh();
	ndDemoSubMesh* AddSubMesh();
	virtual const char* GetTextureName (const ndDemoSubMesh* const subMesh) const;

	virtual void RenderNormals();
	virtual void Render (ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	virtual void RenderTransparency(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	virtual void RenderShadowMap(ndShadowMapRenderPass* const shadowMap, const ndMatrix& modelMatrixProjection);

	void OptimizeForRender(const glPositionNormalUV* const points, ndInt32 pointCount,
						   const ndInt32* const indices, ndInt32 indexCount);
	void GetVertexArray(ndArray<ndVector>& points) const;
	void GetIndexArray(ndArray<ndInt32>& indexList) const;

	protected:
	void ResetOptimization();
	void RenderGeometry(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);

	void RenderDifusse(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	void RenderDifusseShadow(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);

	GLint m_indexCount;
	GLint m_vertexCount;

	GLuint m_shader;
	GLint m_textureLocation;
	GLint m_transparencyLocation;
	GLint m_normalMatrixLocation;
	GLint m_projectMatrixLocation;
	GLint m_viewModelMatrixLocation;
	GLint m_materialAmbientLocation;
	GLint m_materialDiffuseLocation;
	GLint m_materialSpecularLocation;
	GLint m_directionalLightDirLocation;
	
	GLuint m_shaderShadow;
	GLint m_textureLocationShadow;
	GLint m_transparencyLocationShadow;
	GLint m_projectMatrixLocationShadow;
	GLint m_viewModelMatrixLocationShadow;
	GLint m_materialAmbientLocationShadow;
	GLint m_materialDiffuseLocationShadow;
	GLint m_materialSpecularLocationShadow;
	GLint m_directionalLightDirLocationShadow;

	GLint m_worldMatrix;
	GLint m_shadowSlices;
	GLint m_depthMapTexture;
	GLint m_directionLightViewProjectionMatrixShadow;

	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	bool m_hasTransparency;

	friend class ndDemoEntity;
	friend class ndDemoSkinMesh;
	friend class ndDemoDebrisRootEntity;
};

#endif 


