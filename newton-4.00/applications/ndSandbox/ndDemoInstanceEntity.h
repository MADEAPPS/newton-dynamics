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

#ifndef __DEMO_INSTANCE_ENTITY_H__
#define __DEMO_INSTANCE_ENTITY_H__

#include "ndDemoMesh.h"
#include "ndDemoEntity.h"

class ndDemoMeshIntance : public ndDemoMesh
{
	public:
	ndDemoMeshIntance(const char* const name, const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat32 opacity = 1.0f, const dMatrix& uvMatrix = dGetIdentityMatrix());
	~ndDemoMeshIntance();

	virtual void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);
	void SetTransforms(dInt32 count, const dMatrix* const matrixArray);

	private:
	void RenderBatch(dInt32 start, ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	const dMatrix* m_offsets;
	dInt32 m_instanceCount;
	dInt32 m_maxInstanceCount;
	GLuint m_matrixOffsetBuffer;
};


class ndDemoInstanceEntity: public ndDemoEntity
{
	public:
	ndDemoInstanceEntity(ndDemoMeshIntance* const instanceMesh);
	ndDemoInstanceEntity(const ndDemoInstanceEntity& copyFrom);
	virtual ~ndDemoInstanceEntity(void);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	ndDemoMeshIntance* m_instanceMesh;
};

#endif