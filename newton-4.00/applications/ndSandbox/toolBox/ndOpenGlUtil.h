/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
	

#if !defined(AFX_OPENGLUTIL_H_H)
#define AFX_OPENGLUTIL_H_H

#include "ndSandboxStdafx.h"

#ifdef _NEWTON_USE_DOUBLE
	inline void glMaterialParam(GLenum face, GLenum pname, const dFloat32 *params)
	{
		GLfloat tmp[4] = { GLfloat(params[0]), GLfloat(params[1]), GLfloat(params[2]), GLfloat(params[3]) };
		glMaterialfv(face, pname, &tmp[0]);
	}
	#define glMultMatrix(x) glMultMatrixd(x)
	#define glLoadMatrix(x) glMultMatrixd(x)
	#define glGetFloat(x,y) glGetDoublev(x,(GLdouble *)y) 
#else 
	#define glMaterialParam glMaterialfv
	#define glMultMatrix(x) glMultMatrixf(x)
	#define glLoadMatrix(x) glMultMatrixf(x)
	#define glGetFloat(x,y) glGetFloatv(x, (GLfloat*)y) 
#endif

class glVector : public dFixSizeArray<GLfloat, 4>
{
	public:
	glVector(const dVector& v)
	{
		(*this)[0] = GLfloat(v[0]);
		(*this)[1] = GLfloat(v[1]);
		(*this)[2] = GLfloat(v[2]);
		(*this)[3] = GLfloat(v[3]);
	}
};

class glMatrix : public dFixSizeArray<GLfloat, 16>
{
	public:
	glMatrix(const dMatrix& matrix)
	{
		for (dInt32 i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				(*this)[i * 4 + j] = GLfloat(matrix[i][j]);
			}
		}
	}
};

class dMOUSE_POINT
{
	public:
	dInt32 x;
	dInt32 y;
};

void ShowMousePicking (const dVector& p0, const dVector& p1, const dVector& originColor = dVector (1.0f, 1.0f, 0.0f, 0.0f), const dVector& lineColor =  dVector (1.0f, 1.0f, 1.0f, 0.0f)); 
void HandleResize(dInt32 width, dInt32 height);
dInt32 dGetKeyState(dInt32 key);
void GetCursorPos(dMOUSE_POINT& point);

#endif 

