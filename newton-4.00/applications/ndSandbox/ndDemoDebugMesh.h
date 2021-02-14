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

	void SetColor(const dVector& color)
	{
		m_color.m_x = GLfloat(color.m_x);
		m_color.m_y = GLfloat(color.m_y);
		m_color.m_z = GLfloat(color.m_z);
		m_color.m_w = GLfloat(color.m_w);
	}
	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	ndMeshVector4 m_color;
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
	ndWireFrameDebugMesh(const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision);
	~ndWireFrameDebugMesh();

	void SetColor(const dVector& color)
	{
		m_color.m_x = GLfloat(color.m_x);
		m_color.m_y = GLfloat(color.m_y);
		m_color.m_z = GLfloat(color.m_z);
		m_color.m_w = GLfloat(color.m_w);
	}

	void Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix);

	ndMeshVector4 m_color;
	dInt32 m_indexCount;
	dInt32 m_shadeColorLocation;
	dInt32 m_projectionViewModelMatrixLocation;

	GLuint m_shader;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLuint m_lineIndexBuffer;
};

#endif 


