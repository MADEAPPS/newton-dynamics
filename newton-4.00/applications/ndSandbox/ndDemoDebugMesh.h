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

#ifndef _D_DEMO_DEBUG_MESH_H_
#define _D_DEMO_DEBUG_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoMeshInterface.h"

//class ndDemoMesh;
//class ndDemoEntity;
class ndShaderPrograms;
//class ndDemoEntityManager;


class ndFlatShadedDebugMesh: public ndDemoMeshInterface
{
	public:
	ndFlatShadedDebugMesh(const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision);
	~ndFlatShadedDebugMesh();

	void SetColor(const ndVector& color)
	{
		m_color = color;
	}
	void Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);

	ndVector m_color;
	dInt32 m_indexCount;
	dInt32 m_shadeColorLocation;
	dInt32 m_normalMatrixLocation;
	dInt32 m_projectMatrixLocation;
	dInt32 m_viewModelMatrixLocation;

	GLuint m_shader;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLuint m_triangleIndexBuffer;
};

class ndWireFrameDebugMesh: public ndDemoMeshInterface
{
	public:
	ndWireFrameDebugMesh(const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, ndShapeDebugNotify::ndEdgeType edgeType = ndShapeDebugNotify::ndEdgeType::m_shared);
	~ndWireFrameDebugMesh();

	void SetColor(const ndVector& color)
	{
		m_color = color;
	}

	void Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);

	ndVector m_color;
	dInt32 m_indexCount;
	dInt32 m_shadeColorLocation;
	dInt32 m_projectionViewModelMatrixLocation;

	GLuint m_shader;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLuint m_lineIndexBuffer;
};

#endif 


