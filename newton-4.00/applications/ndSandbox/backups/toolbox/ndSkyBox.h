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


// RenderPrimitive.h: interface for the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __SKY_BOX_H_
#define __SKY_BOX_H_

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoEntityManager;

class ndSkyBox: public ndDemoEntity
{
	public:
	ndSkyBox(GLuint shader);
	~ndSkyBox();

	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	private:
	ndMatrix m_textureMatrix;
	GLuint m_shader;
	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_texturecubemap;
	GLuint m_vertextArrayBuffer;
	GLint m_invViewModelProjectionTextureMatrix;
};

#endif 

