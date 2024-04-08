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

#ifndef __DEMO_INSTANCE_ENTITY_H__
#define __DEMO_INSTANCE_ENTITY_H__

#include "ndDemoMesh.h"
#include "ndDemoEntity.h"

class ndDemoMeshIntance : public ndDemoMesh
{
	public:
	ndDemoMeshIntance(const char* const name, const ndShaderCache& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const texture1, const char* const texture2, ndFloat32 opacity = 1.0f, const ndMatrix& uvMatrix = ndGetIdentityMatrix());
	~ndDemoMeshIntance();

	virtual void Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	virtual void RenderShadowMap(ndShadowMapRenderPass* const shadowMap, const ndMatrix& modelMatrixProjection);
	void SetTransforms(ndInt32 count, const ndMatrix* const matrixArray);

	private:
	void RenderBatch(ndInt32 start, ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	void RenderShadowMapBatch(ndInt32 start, ndShadowMapRenderPass* const shadowMap, const ndMatrix& modelMatrixProjection);

	const ndMatrix* m_offsets;
	ndInt32 m_instanceCount;
	ndInt32 m_maxInstanceCount;
	GLuint m_matrixOffsetBuffer;
};

class ndDemoInstanceEntity: public ndDemoEntity
{
	public:
	ndDemoInstanceEntity(ndSharedPtr<ndDemoMeshIntance> instanceMesh);
	ndDemoInstanceEntity(const ndDemoInstanceEntity& copyFrom);
	virtual ~ndDemoInstanceEntity(void);

	virtual void RenderShadowMap(ndShadowMapRenderPass* const shadowMap, const ndMatrix& matrix);
	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;
	
	static ndArray<ndMatrix>& GetMatrixStack();

	ndSharedPtr<ndDemoMeshIntance> m_instanceMesh;
};

#endif