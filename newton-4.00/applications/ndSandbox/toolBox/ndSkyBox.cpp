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

#include "ndSandboxStdafx.h"
#include "ndTargaToOpenGl.h"
#include "ndSkyBox.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////




ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity (dGetIdentityMatrix(), nullptr)
	,m_shader(shader)
{
	m_textures[0] = LoadTexture("NewtonSky0001.tga");
	m_textures[1] = LoadTexture("NewtonSky0002.tga");
	m_textures[2] = LoadTexture("NewtonSky0003.tga");
	m_textures[3] = LoadTexture("NewtonSky0004.tga");
	m_textures[4] = LoadTexture("NewtonSky0005.tga");
	m_textures[5] = LoadTexture("NewtonSky0006.tga");

	GLfloat size = 200.0f;
	static GLfloat vertices[] = 
	{
		size, size, size,  -size, size, size,  -size,-size, size,  size,-size, size, // v0,v1,v2,v3 (front)
		size, size, size,   size,-size, size,   size,-size,-size,  size, size,-size, // v0,v3,v4,v5 (right)
		size, size, size,   size, size,-size,  -size, size,-size, -size, size, size, // v0,v5,v6,v1 (top)
		-size, size, size,  -size, size,-size,  -size,-size,-size, -size,-size, size, // v1,v6,v7,v2 (left)
		-size,-size,-size,   size,-size,-size,   size,-size, size, -size,-size, size, // v7,v4,v3,v2 (bottom)
		size,-size,-size,  -size,-size,-size,  -size, size,-size,  size, size,-size  // v4,v7,v6,v5 (back)
	};
	//m_uvOffest = sizeof(vertices);

	// texture coordinate array
	static GLfloat texCoords[] = 
	{
		1, 0,   0, 0,   0, 1,   1, 1,               // v0,v1,v2,v3 (front)
		0, 0,   0, 1,   1, 1,   1, 0,               // v0,v3,v4,v5 (right)
		1, 1,   1, 0,   0, 0,   0, 1,               // v0,v5,v6,v1 (top)
		1, 0,   0, 0,   0, 1,   1, 1,               // v1,v6,v7,v2 (left)
		0, 1,   1, 1,   1, 0,   0, 0,               // v7,v4,v3,v2 (bottom)
		0, 1,   1, 1,   1, 0,   0, 0                // v4,v7,v6,v5 (back)
	};

	static GLuint indices[] = 
	{
		0, 1, 2,   2, 3, 0,    // v0-v1-v2, v2-v3-v0 (front)
		4, 5, 6,   6, 7, 4,    // v0-v3-v4, v4-v5-v0 (right)
		8, 9,10,  10,11, 8,    // v0-v5-v6, v6-v1-v0 (top)
		12,13,14,  14,15,12,    // v1-v6-v7, v7-v2-v1 (left)
		16,17,18,  18,19,16,    // v7-v4-v3, v3-v2-v7 (bottom)
		20,21,22,  22,23,20     // v4-v7-v6, v6-v5-v4 (back)
	};

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof (vertices), vertices, GL_STATIC_DRAW);

	glGenBuffers(1, &m_uvBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_uvBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords), texCoords, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Create and set-up the vertex array object
	glGenVertexArrays(1, &m_vertexArrayHandle);
	glBindVertexArray(m_vertexArrayHandle);
	glEnableVertexAttribArray(0);  // Vertex position
	glEnableVertexAttribArray(1);  // Vertex uv

	glBindVertexBuffer(0, m_vertexBuffer, 0, 3 * sizeof(GLfloat));
	glBindVertexBuffer(1, m_uvBuffer, 0, 2 * sizeof(GLfloat));

	glVertexAttribFormat(0, 3, GL_FLOAT, GL_FALSE, 0);
	glVertexAttribBinding(0, 0);
	glVertexAttribFormat(1, 2, GL_FLOAT, GL_FALSE, 0);
	glVertexAttribBinding(1, 1);
	glBindVertexArray(0);

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

ndSkyBox::~ndSkyBox()
{
	// delete VBO when program terminated
	glDeleteBuffers(1, &m_uvBuffer);
	glDeleteBuffers(1, &m_indexBuffer);
	glDeleteBuffers(1, &m_vertexBuffer);
	glDeleteBuffers(1, &m_vertexArrayHandle);
	
	for (int i = 0; i < int(sizeof (m_textures) / sizeof (m_textures[0])); i++) 
	{
		ReleaseTexture(m_textures[i]);
	}
}

void ndSkyBox::Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix__) const
{
	dMatrix matrix;

	// get the model viewMatrix; 
	glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);

	dMatrix skyMatrix (dGetIdentityMatrix());
	skyMatrix.m_posit = matrix.UntransformVector (dVector (0.0f, 0.25f, 0.0f, 1.0f));
skyMatrix.m_posit = matrix.UntransformVector(dVector(0.0f, 0.25f, -800.0f, 1.0f));

	glPushMatrix();
	glMultMatrix(&skyMatrix[0][0]);
	
	glUseProgram(m_shader);
	// activate attributes

	glUniform1i(m_textureLocation, 0);
	glBindVertexArray(GL_ARRAY_BUFFER, m_vertexArrayHandle);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0); 

	// unbind
	glBindVertexArray(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(0);

	glPopMatrix();
}
