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

#ifndef _D_DEMO_SPLINE_CURVE_MESH_H_
#define _D_DEMO_SPLINE_CURVE_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoMeshInterface.h"

class ndDemoMesh;
class ndDemoEntity;
class ndShaderCache;
class ndDemoEntityManager;

class ndDemoSplinePathMesh: public ndDemoMeshInterface
{
	public:
	ndDemoSplinePathMesh(const ndBezierSpline& curve, const ndShaderCache& shaderCache, ndInt32 resolution);
	~ndDemoSplinePathMesh();

	void SetColor(const ndVector& color);
	ndInt32 GetRenderResolution() const;
	void SetRenderResolution(ndInt32 breaks);

	virtual void Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix);
	
	ndBezierSpline m_curve;
	ndVector m_color;
	ndInt32 m_renderResolution;
	GLuint m_shader;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLuint m_shadeColorLocation;
	GLuint m_projectionViewModelMatrixLocation;
};

#endif 


