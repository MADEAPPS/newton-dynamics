/* Copyright (c) <2009> <Newton Game Dynamics>
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
#include <toolbox_stdafx.h>
#include "TargaToOpenGl.h"
#include "SkyBox.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


SkyBox::SkyBox()
	:DemoEntity (GetIdentityMatrix(), NULL)
{
	dFloat boxsize;

	boxsize = 200.0f;

	m_size = dVector (boxsize, boxsize, boxsize);
	m_textures[0] = LoadTexture("NewtonSky0001.tga");
	m_textures[1] = LoadTexture("NewtonSky0002.tga");
	m_textures[2] = LoadTexture("NewtonSky0003.tga");
	m_textures[3] = LoadTexture("NewtonSky0004.tga");
	m_textures[4] = LoadTexture("NewtonSky0005.tga");
	m_textures[5] = LoadTexture("NewtonSky0006.tga");
}


void SkyBox::Render(dFloat timeStep) const
{
	dMatrix matrix;

	// get the model viewMatrix; 
	glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);

	dMatrix skyMatrix (GetIdentityMatrix());
	skyMatrix.m_posit = matrix.UnrotateVector (matrix.m_posit.Scale (-1.0f));
	skyMatrix.m_posit.m_y += 25.0f; 

	glPushMatrix();
	glMultMatrix(&skyMatrix[0][0]);
	

	dVector color (1.0, 1.0f, 1.0f);

	dFloat padd = 1.0e-3f;
//	glDisable(GL_BLEND);
//	glDisable (GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);	

	// front
	glBindTexture(GL_TEXTURE_2D, m_textures[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f( m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f( m_size.m_x, -m_size.m_y, -m_size.m_z);
	glEnd();

	// left
	glBindTexture(GL_TEXTURE_2D, m_textures[3]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glEnd();

	// right
	glBindTexture(GL_TEXTURE_2D, m_textures[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(m_size.m_x,   m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(m_size.m_x,   m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(m_size.m_x,  -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(m_size.m_x,  -m_size.m_y,  m_size.m_z);
	glEnd();


	// back
	glBindTexture(GL_TEXTURE_2D, m_textures[2]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f( m_size.m_x,  m_size.m_y, m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f( m_size.m_x, -m_size.m_y, m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, m_size.m_z);
	glEnd();


	// top
	glBindTexture(GL_TEXTURE_2D, m_textures[4]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f( m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f( m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y,  m_size.m_z);
	glEnd();	 

	// bottom
	glBindTexture(GL_TEXTURE_2D, m_textures[5]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f( m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f( m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glEnd();	 	

//	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
//	glEnable (GL_LIGHTING);

	// render the rest of the hierarchy
	glPopMatrix();

}
	
SkyBox::~SkyBox()
{
	for (int i = 0; i < sizeof (m_textures) / sizeof (m_textures[0]); i ++) {
		ReleaseTexture(m_textures[i]);
	}
}



