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



// OpenGlUtil.cpp: implementation of the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndOpenGlUtil.h"
#include "ndPngToOpenGl.h"

	
static dMOUSE_POINT mouse0;

void GetCursorPos(dMOUSE_POINT& point)
{
	point = mouse0;
}

void ShowMousePicking(const ndVector&, const ndVector&, const ndVector&, const ndVector&)
{
	ndTrace((__FUNCTION__));
	// set up the cube's texture
	//glDisable(GL_TEXTURE_2D);
	//glColor3f(GLfloat(lineColor.m_x), GLfloat(lineColor.m_y), GLfloat(lineColor.m_z));
	//glLineWidth(2.0f);
	//glBegin(GL_LINES);
	//glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	//glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
	//glEnd();
	//glLineWidth(1.0f);
	//
	//// draw the bone points
	//glColor3f(GLfloat(originColor.m_x), GLfloat(originColor.m_y), GLfloat(originColor.m_z));
	//glPointSize(8.0f);
	//
	//glBegin(GL_POINTS);
	//glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	//glEnd();
	//glPointSize(1.0f);
	//
	//glColor3f(1.0f, 1.0f, 0.0f);
}



