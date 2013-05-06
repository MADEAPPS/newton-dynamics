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



// OpenGlUtil.cpp: implementation of the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include "OpenGlUtil.h"
#include "TargaToOpenGl.h"


int keyBoardTraceMode = 1;
#ifdef RECORD_LOG
	FILE * file;
#else 
	#ifdef READ_LOG
		FILE * file;
	#endif
#endif


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
static char keybuffer[512];

void KeyDown (int key) 
{
	keybuffer[key % (sizeof (keybuffer) - 1)] = 1;
}

void KeyUp(int key) 
{
	keybuffer[key % (sizeof (keybuffer) - 1)] = 0;
}


#ifdef RECORD_LOG
char outPathName[2048];
GetWorkingFileName ("log.log", outPathName);
file = fopen (outPathName, "wb");
#endif

#ifdef READ_LOG
char outPathName[2048];
GetWorkingFileName ("log.log", outPathName);
file = fopen (outPathName, "rb");
#endif




void GetCursorPos(dMOUSE_POINT& point)
{
	point = mouse0;

	if (keyBoardTraceMode){
#ifdef RECORD_LOG
		fwrite (& point, sizeof (dMOUSE_POINT), 1, file);
		fflush (file);
#endif

#ifdef READ_LOG
		fread (& point, sizeof (dMOUSE_POINT), 1, file);
#endif
	}
}

int dGetKeyState(int key)
{
	int code;
	code = keybuffer[key];

	if (keyBoardTraceMode){
#ifdef RECORD_LOG
		fwrite (&code, sizeof (int), 1, file);
		fflush (file);
#endif

#ifdef READ_LOG
		fread (&code, sizeof (int), 1, file);
#endif
	}
	return code;
}


dMatrix GetModelViewMatrix ()
{
	int i;
	int j;

	GLdouble modelview[16]; 
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 
	dMatrix camMatrix (GetIdentityMatrix());
	for (i = 0; i < 4; i ++) {
		for (j = 0; j < 4; j ++) {
			camMatrix[i][j] = dFloat (modelview[i * 4 + j]);
		}
	}
	return camMatrix;
}





dVector WorldToScreen (const dVector& world)
{
	_ASSERTE (0);
return dVector (0, 0, 0, 0);
/*
	int win[4]; 
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	// get a point on the display arai of the windows
	GLUI_Master.get_viewport_area(&win[0], &win[1], &win[2], &win[3]);
	viewport[0] = GLint (win[0]);
	viewport[1] = GLint (win[1]);
	viewport[2] = GLint (win[2]);
	viewport[3] = GLint (win[3]);


	//Where the 16 doubles of the matrix are to be stored
	GLdouble modelview[16]; 

	//Retrieve the matrix
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 

	GLdouble projection[16]; 
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLdouble winX;
	GLdouble winY;
	GLdouble winZ; //The coordinates to pass in

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	GLdouble objx = world.m_x;
	GLdouble objy = world.m_y;
	GLdouble objz = world.m_z;

	// use the real GL view port
	glGetIntegerv(GL_VIEWPORT, viewport); 
	gluProject (objx, objy, objz, modelview, projection, viewport, &winX, &winY, &winZ);

	winY = (dFloat)viewport[3] - winY; 

	return dVector (dFloat(winX), dFloat (winY), dFloat(winZ), 0.0f);
*/
}



dVector ScreenToWorld (const dVector& screen)
{
	//Where the values will be stored
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	// get a point on the display array of the windows
	glGetIntegerv(GL_VIEWPORT, viewport); 

	//Where the 16 doubles of the matrix are to be stored

	//Retrieve the matrix
	GLdouble modelview[16]; 
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 

	GLdouble projection[16]; 
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLdouble winX = 0.0;
	GLdouble winY = 0.0;
	GLdouble winZ = 0.0; //The coordinates to pass in

	winX = screen.m_x; //Store the x cord
	winY = screen.m_y; //Store the y cord
	winZ = screen.m_z; //Store the Z cord

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	winY = (dFloat)viewport[3] - winY; 

	GLdouble objx;
	GLdouble objy;
	GLdouble objz;

	// use the real GL view port
	glGetIntegerv(GL_VIEWPORT, viewport); 
	gluUnProject (winX, winY, winZ, modelview, projection, viewport, &objx, &objy, &objz);

	return dVector (dFloat(objx), dFloat(objy), dFloat(objz));
}

void ShowMousePicking (const dVector& p0, const dVector& p1, const dVector& originColor, const dVector& lineColor)
{
	// set up the cube's texture
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(lineColor.m_x, lineColor.m_y, lineColor.m_z);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glVertex3f(p0.m_x, p0.m_y, p0.m_z);
	glVertex3f(p1.m_x, p1.m_y, p1.m_z);
	glEnd();
	glLineWidth(1.0f);

	// draw the bone points
	glColor3f(originColor.m_x, originColor.m_y, originColor.m_z);
	glPointSize(6.0f);
	glBegin(GL_POINTS);
	glVertex3f(p0.m_x, p0.m_y, p0.m_z);
	glVertex3f(p1.m_x, p1.m_y, p1.m_z);
	glEnd();
	glPointSize(1.0f);

	glColor3f(1.0f, 1.0f, 0.0f);
	glEnable (GL_LIGHTING);
}



