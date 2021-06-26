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
	
#ifndef __OPENGLUTIL_H__
#define __OPENGLUTIL_H__

#include "ndSandboxStdafx.h"

#ifdef D_NEWTON_USE_DOUBLE
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

class glVector
{
	public:
	glVector()
	{
		m_data[0] = GLfloat(0.0f);
		m_data[1] = GLfloat(0.0f);
		m_data[2] = GLfloat(0.0f);
		m_data[3] = GLfloat(0.0f);
	}

	glVector(dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
	{
		m_data[0] = GLfloat(x);
		m_data[1] = GLfloat(y);
		m_data[2] = GLfloat(z);
		m_data[3] = GLfloat(w);
	}

	glVector(const dVector& v)
	{
		m_data[0] = GLfloat(v[0]);
		m_data[1] = GLfloat(v[1]);
		m_data[2] = GLfloat(v[2]);
		m_data[3] = GLfloat(v[3]);
	}

	GLfloat& operator[] (dInt32 i)
	{
		dAssert(i >= 0);
		dAssert(i < sizeof m_data / sizeof(m_data[0]));
		return m_data[i];
	}

	const GLfloat& operator[] (dInt32 i) const
	{
		dAssert(i >= 0);
		dAssert(i < sizeof m_data / sizeof(m_data[0]));
		return m_data[i];
	}

	GLfloat m_data[4];
};

class glMatrix
{
	public:
	glMatrix(const dMatrix& matrix)
	{
		for (dInt32 i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				m_data[i * 4 + j] = GLfloat(matrix[i][j]);
			}
		}
	}

	GLfloat& operator[] (dInt32 i)
	{
		dAssert(i >= 0);
		dAssert(i < sizeof m_data / sizeof(m_data[0]));
		return m_data[i];
	}

	const GLfloat& operator[] (dInt32 i) const
	{
		dAssert(i >= 0);
		dAssert(i < sizeof m_data/sizeof (m_data[0]));
		return m_data[i];
	}

	GLfloat m_data[16];
};

class ndMeshVector
{
	public:
	ndMeshVector() {}
	ndMeshVector(GLfloat x, GLfloat y, GLfloat z)
		:m_x(x), m_y(y), m_z(z)
	{
	}

	GLfloat m_x;
	GLfloat m_y;
	GLfloat m_z;
};

class ndMeshVector4 : public ndMeshVector
{
	public:
	GLfloat m_w;
};

class ndMeshMatrix
{
	public:
	ndMeshVector4 m_array[4];
};

class ndMeshUV
{
	public:
	GLfloat m_u;
	GLfloat m_v;
};

class ndPointNormal
{
	public:
	ndMeshVector m_posit;
	ndMeshVector m_normal;
};

class ndMeshPointUV : public ndPointNormal
{
	public:
	ndMeshUV m_uv;
};


class dMOUSE_POINT
{
	public:
	dInt32 x;
	dInt32 y;
};

void GetCursorPos(dMOUSE_POINT& point);
void ShowMousePicking (const dVector& p0, const dVector& p1, const dVector& originColor = dVector (1.0f, 1.0f, 0.0f, 0.0f), const dVector& lineColor =  dVector (1.0f, 1.0f, 1.0f, 0.0f)); 

#endif 

