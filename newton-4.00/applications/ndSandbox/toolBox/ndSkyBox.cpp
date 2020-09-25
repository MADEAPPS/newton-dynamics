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
	,m_displayList(0)
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
	m_uvOffest = sizeof(vertices);

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

	// index array for glDrawElements()
	// A cube requires 36 indices = 6 sides * 2 tris * 3 verts
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
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices) + sizeof(texCoords), 0, GL_STATIC_DRAW);

	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(vertices), sizeof(texCoords), texCoords);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	m_displayList = glGenLists(1);
	glNewList(m_displayList, GL_COMPILE);
	DrawMesh ();
	glEndList();
}

ndSkyBox::~ndSkyBox()
{

	// delete VBO when program terminated
	glDeleteBuffers(1, &m_vertexBuffer);

	if (m_displayList) 
	{
		glDeleteLists (m_displayList, 1);
	}
	for (int i = 0; i < int(sizeof (m_textures) / sizeof (m_textures[0])); i++) 
	{
		ReleaseTexture(m_textures[i]);
	}
}

void ndSkyBox::DrawMesh () const
{
	glUseProgram(m_shader);
	glUniform1i(glGetUniformLocation(m_shader, "texture"), 0);

	glColor3f(1.0f, 1.0f, 1.0f);

	GLfloat padd = 1.0e-3f;
	dFloat32 size = 200.0f;

	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	// front
	glBindTexture(GL_TEXTURE_2D, m_textures[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size), GLfloat(size), -GLfloat(size));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size), GLfloat(size), -GLfloat(size));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size), -GLfloat(size), -GLfloat(size));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size), -GLfloat(size), -GLfloat(size));
	glEnd();

	// left
	glBindTexture(GL_TEXTURE_2D, m_textures[3]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size), GLfloat(size), -GLfloat(size));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size), GLfloat(size), GLfloat(size));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size), -GLfloat(size), GLfloat(size));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size), -GLfloat(size), -GLfloat(size));
	glEnd();

	// right
	glBindTexture(GL_TEXTURE_2D, m_textures[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size), GLfloat(size), GLfloat(size));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size), GLfloat(size), -GLfloat(size));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size), -GLfloat(size), -GLfloat(size));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size), -GLfloat(size), GLfloat(size));
	glEnd();

	// back
	glBindTexture(GL_TEXTURE_2D, m_textures[2]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size), GLfloat(size), GLfloat(size));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size), GLfloat(size), GLfloat(size));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size), -GLfloat(size), GLfloat(size));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size), -GLfloat(size), GLfloat(size));
	glEnd();

	// top
	glBindTexture(GL_TEXTURE_2D, m_textures[4]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size), GLfloat(size), -GLfloat(size));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size), GLfloat(size), -GLfloat(size));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size), GLfloat(size), GLfloat(size));
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size), GLfloat(size), GLfloat(size));
	glEnd();

	// bottom
	glBindTexture(GL_TEXTURE_2D, m_textures[5]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size), -GLfloat(size), GLfloat(size));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size), -GLfloat(size), GLfloat(size));
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size), -GLfloat(size), -GLfloat(size));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size), -GLfloat(size), -GLfloat(size));
	glEnd();

	//	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	glUseProgram(0);
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

	if (m_displayList) 
//	if (0)
	{
		glCallList(m_displayList);
	} 
	else 
	{
		//DrawMesh();
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

		GLint attribVertexPosition;
		GLint attribVertexTexCoord;

		glUseProgram(m_shader);
		attribVertexPosition = glGetAttribLocation(m_shader, "vertexPosition");
		attribVertexTexCoord = glGetAttribLocation(m_shader, "vertexTexCoord");

		// activate attributes
		glEnableVertexAttribArray(attribVertexPosition);
		glEnableVertexAttribArray(attribVertexTexCoord);

		// set attrib arrays using glVertexAttribPointer()
		glVertexAttribPointer(attribVertexPosition, 3, GL_FLOAT, false, 0, 0);
		glVertexAttribPointer(attribVertexTexCoord, 2, GL_FLOAT, false, 0, (void*)m_uvOffest);

		glBindTexture(GL_TEXTURE_2D, m_textures[0]);
		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)0); 
		
		glDisableVertexAttribArray(attribVertexPosition);
		glDisableVertexAttribArray(attribVertexTexCoord);

		// unbind
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glUseProgram(0);
	}

	glPopMatrix();
}
