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

// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently


#ifndef _TOLLBOX_STDAFX_H_
#define _TOLLBOX_STDAFX_H_

// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#define DOMMY_API 

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <ctype.h>

#define USE_STATIC_MESHES_DEBUG_COLLISION
//#define USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK


#ifdef _WIN32
	#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

	#undef APIENTRY
	#define GLFW_EXPOSE_NATIVE_WIN32
	#define GLFW_EXPOSE_NATIVE_WGL

	#include <windows.h>
	#include <commctrl.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <malloc.h>
	#include <memory.h>
	#include <time.h>
	#include <tchar.h>
	#include <crtdbg.h>

	#include <glatter.h>
	#include <GL/glu.h>
	#include <GL/gl.h>
	#include <imgui.h>
	#include <GLFW/glfw3.h>
	#include <GLFW/glfw3native.h>
#endif
	
#if (defined (_POSIX_VER) || defined (_POSIX_VER_64))
	#include <stdlib.h>
	#include <unistd.h>
	#include <time.h>

	#include <glatter.h>
	#include <GL/glu.h>
	#include <GL/gl.h>
	#include <imgui.h>
	#include <GLFW/glfw3.h>

	// audio library support
	#include <AL/al.h>
	#include <AL/alc.h>
#endif

#ifdef _MACOSX_VER
	#include <CoreFoundation/CoreFoundation.h> 
	#include <unistd.h>
	#include <GL/glew.h>
	#include <OpenGL/glu.h>
	#include <OpenGL/gl.h>

	// audio library support
	#include <OpenAl/al.h>
	#include <OpenAl/alc.h>
#endif

// SDK includes
#include <ndNewton.h>

/*
#ifdef _NEWTON_USE_DOUBLE
	#define glMultMatrix(x) glMultMatrixd(x)
	#define glLoadMatrix(x) glMultMatrixd(x)
	#define glGetFloat(x,y) glGetDoublev(x,(GLdouble *)y) 
#else
	#define glMultMatrix(x) glMultMatrixf(x)
	#define glLoadMatrix(x) glMultMatrixf(x)
	#define glGetFloat(x,y) glGetFloatv(x,(dFloat32  *)y) 
#endif
*/

#ifdef _NEWTON_USE_DOUBLE
inline void glMaterialParam (GLenum face, GLenum pname, const dFloat32 *params)
{
	GLfloat tmp[4] = {GLfloat(params[0]), GLfloat(params[1]), GLfloat(params[2]), GLfloat(params[3])};
	glMaterialfv (face, pname, &tmp[0]);
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


#ifndef _MSC_VER
	#ifndef stricmp
		#define stricmp strcasecmp
	#endif

	#ifndef _strlwr
		inline char *_strlwr_ (char *a) 
		{ 
			char *ret = a; 
			while (*a != '\0') 
			{ 
				//if (isupper (*a)) 
				*a = char (tolower (*a)); 
				++a; 
			} 
			return ret; 
		}

		#define strlwr(a) _strlwr_ (a) 
	#endif
#endif


//#define DEMO_CHECK_ASYN_UPDATE
#define dRAND_MAX		0x0fffffff

unsigned dRand ();
void dSetRandSeed (unsigned seed);
dFloat32 dGaussianRandom (dFloat32 amp);

inline int dTwosPower (int x)
{
	int rval=1;
	for (; rval < x; rval *= 2);
	return rval;
}


// for some reason specifying a relative does not seem to work in Linus
// and i have to specify a absolute path
// #define ASSETS_PATH "."
//void GetAplicationDirectory (char* const aplicationDir);
void dGetWorkingFileName (const char* const name, char* const outPathName);


// little Indian/big Indian conversion
unsigned SWAP_INT32(unsigned x);
unsigned short SWAP_INT16(unsigned short x);
void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count);

#endif 

