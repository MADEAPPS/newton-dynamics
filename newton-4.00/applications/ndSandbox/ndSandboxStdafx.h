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

// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently


#ifndef _TOLLBOX_STDAFX_H_
#define _TOLLBOX_STDAFX_H_

// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <float.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#if (defined(WIN32) || defined(_WIN32))
	#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

	#undef APIENTRY
	#define GLFW_EXPOSE_NATIVE_WIN32
	#define GLFW_EXPOSE_NATIVE_WGL

	#include <windows.h>
	#include <commctrl.h>
	#include <crtdbg.h>
	
	#include <glatter.h>
	#include <GL/glu.h>
	#include <GL/gl.h>
	#include <GLFW/glfw3.h>
	#include <GLFW/glfw3native.h>
#endif
	
#if defined (__linux__ )
	#include <unistd.h>

	#include <glatter.h>
	#include <GL/glu.h>
	#include <GL/gl.h>
	#include <GLFW/glfw3.h>
#endif

#if defined (__APPLE__)
	#include <CoreFoundation/CoreFoundation.h> 
	#include <unistd.h>
	//#include <GL/glew.h>
	//#include <OpenGL/glu.h>
	//#include <OpenGL/gl.h>
	#include <OpenGL/gl3.h>
#endif

// some third party libraries includes here
#include <VHACD.h>
#include <imgui.h>
#include <lodepng.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// SDK includes
#include <ndNewton.h>
#include <ndModelInc.h>
#include <ndBrainInc.h>

void ndGetWorkingFileName (const char* const name, char* const outPathName);

// endian conversion
ndUnsigned32 SWAP_INT32(ndUnsigned32 x);
ndUnsigned16 SWAP_INT16(ndUnsigned16 x);
ndUnsigned16 ndIndian16(ndUnsigned16 x);
ndUnsigned32 ndIndian32(ndUnsigned32 x);

#endif 

