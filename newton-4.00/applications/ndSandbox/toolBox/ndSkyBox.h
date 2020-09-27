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


// RenderPrimitive.h: interface for the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __SKY_BOX_H_
#define __SKY_BOX_H_

#include "ndSandboxStdafx.h"
#include "../ndDemoEntity.h"

class ndDemoEntityManager;


#ifdef USING_GLES_4

class ndSkyBox: public ndDemoEntity
{
	public:
	ndSkyBox(GLuint shader);
	~ndSkyBox();

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	private:
	void SetupCubeMap();

	void LoadCubeTexture(GLenum face, char* const filename);

	int m_uvOffest;
	GLuint m_shader;
	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_texturecubemap;
	GLuint m_vao;
	GLuint m_ibo;
	GLint matrixUniformLocation;

	GLint m_textureLocation;
	GLint m_attribVertexPosition;
	GLint m_attribVertexTexCoord;
};

#else
class ndSkyBox: public ndDemoEntity
{
	public:
	ndSkyBox(GLuint shader);
	~ndSkyBox();

	void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	private:
	void DrawMesh() const;

	GLuint m_displayList;
	GLuint m_shader;
	GLuint m_textures[6];
};
#endif

#endif 

