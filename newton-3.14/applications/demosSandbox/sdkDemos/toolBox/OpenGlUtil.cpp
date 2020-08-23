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



// OpenGlUtil.cpp: implementation of the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////
#include "toolbox_stdafx.h"
#include "OpenGlUtil.h"
#include "TargaToOpenGl.h"


#ifdef _MSC_VER
#if (_MSC_VER < 1300) 
		//VC7 or later, building with pre-VC7 runtime libraries
		//defined by VC6 C libs
		extern "C" long _ftol (double); 
		extern "C" long _ftol2( double dblSource ) 
		{ 
			return long( dblSource ); 
		}
#endif

#if (_MSC_VER < 1400) 
		extern "C" long _ftol2_sse() 
		{ 
			long val;
			_asm fistp qword ptr val;
			return val; 
		}
#endif
#endif



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
	
static dMOUSE_POINT mouse0;

void GetCursorPos(dMOUSE_POINT& point)
{
	point = mouse0;
}

void ShowMousePicking (const dVector& p0, const dVector& p1, const dVector& originColor, const dVector& lineColor)
{
	// set up the cube's texture
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(GLfloat(lineColor.m_x), GLfloat(lineColor.m_y), GLfloat(lineColor.m_z));
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
	glEnd();
	glLineWidth(1.0f);

	// draw the bone points
	glColor3f(GLfloat(originColor.m_x), GLfloat(originColor.m_y), GLfloat(originColor.m_z));
	glPointSize(8.0f);
//glPointSize(16.0f);
	glBegin(GL_POINTS);
	glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	//glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
	glEnd();
	glPointSize(1.0f);

	glColor3f(1.0f, 1.0f, 0.0f);
	glEnable (GL_LIGHTING);
}



