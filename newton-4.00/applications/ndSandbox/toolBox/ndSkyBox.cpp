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


// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndTargaToOpenGl.h"
#include "ndSkyBox.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity (dGetIdentityMatrix(), NULL)
	,m_shader(shader)
	,m_displayList(0)
{
	m_textures[0] = LoadTexture("NewtonSky0001.tga");
	m_textures[1] = LoadTexture("NewtonSky0002.tga");
	m_textures[2] = LoadTexture("NewtonSky0003.tga");
	m_textures[3] = LoadTexture("NewtonSky0004.tga");
	m_textures[4] = LoadTexture("NewtonSky0005.tga");
	m_textures[5] = LoadTexture("NewtonSky0006.tga");

	//m_displayList = glGenLists(1);
	//glNewList(m_displayList, GL_COMPILE);
	//DrawMesh ();
	//glEndList();
}

ndSkyBox::~ndSkyBox()
{
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
	dVector size (200.0f);

/*
	glUseProgram(m_shader);
	glUniform1i(glGetUniformLocation(m_shader, "texture"), 0);

glFrontFace(GL_CW);
	glColor3f(1.0f, 1.0f, 1.0f);

	GLfloat padd = 1.0e-3f;
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	// front
	glBindTexture(GL_TEXTURE_2D, m_textures[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glEnd();

	// left
	glBindTexture(GL_TEXTURE_2D, m_textures[3]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glEnd();

	// right
	glBindTexture(GL_TEXTURE_2D, m_textures[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glEnd();

	// back
	glBindTexture(GL_TEXTURE_2D, m_textures[2]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glEnd();

	// top
	glBindTexture(GL_TEXTURE_2D, m_textures[4]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glEnd();

	// bottom
	glBindTexture(GL_TEXTURE_2D, m_textures[5]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glEnd();

	//	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	glUseProgram(0);
*/

glMatrixMode(GL_MODELVIEW);
glLoadIdentity();

glTranslatef(1.5f, 0.0f, -7.0f);

glBegin(GL_QUADS);


//// Front face  (z = 1.0f)
glColor3f(1.0f, 0.0f, 0.0f);
glVertex3f(1.0f, 1.0f, 1.0f);
glVertex3f(-1.0f, 1.0f, 1.0f);
glVertex3f(-1.0f, -1.0f, 1.0f);
glVertex3f(1.0f, -1.0f, 1.0f);

//glColor3f(0.0f, 1.0f, 0.0f);
//glVertex3f(1.0f, 1.0f, -1.0f);
//glVertex3f(-1.0f, 1.0f, -1.0f);
//glVertex3f(-1.0f, 1.0f, 1.0f);
//glVertex3f(1.0f, 1.0f, 1.0f);

//// Bottom face (y = -1.0f)
//glColor3f(1.0f, 0.5f, 0.0f);
//glVertex3f(1.0f, -1.0f, 1.0f);
//glVertex3f(-1.0f, -1.0f, 1.0f);
//glVertex3f(-1.0f, -1.0f, -1.0f);
//glVertex3f(1.0f, -1.0f, -1.0f);


//// Back face (z = -1.0f)
//glColor3f(1.0f, 1.0f, 0.0f);
//glVertex3f(1.0f, -1.0f, -1.0f);
//glVertex3f(-1.0f, -1.0f, -1.0f);
//glVertex3f(-1.0f, 1.0f, -1.0f);
//glVertex3f(1.0f, 1.0f, -1.0f);
//
//// Left face (x = -1.0f)
//glColor3f(0.0f, 0.0f, 1.0f);
//glVertex3f(-1.0f, 1.0f, 1.0f);
//glVertex3f(-1.0f, 1.0f, -1.0f);
//glVertex3f(-1.0f, -1.0f, -1.0f);
//glVertex3f(-1.0f, -1.0f, 1.0f);
//
//// Right face (x = 1.0f)
//glColor3f(1.0f, 0.0f, 1.0f);
//glVertex3f(1.0f, 1.0f, -1.0f);
//glVertex3f(1.0f, 1.0f, 1.0f);
//glVertex3f(1.0f, -1.0f, 1.0f);
//glVertex3f(1.0f, -1.0f, -1.0f);
glEnd();

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
	{
		glCallList(m_displayList);
	} 
	else 
	{
		DrawMesh();
	}

	// render the rest of the hierarchy
	glPopMatrix();
}
	



