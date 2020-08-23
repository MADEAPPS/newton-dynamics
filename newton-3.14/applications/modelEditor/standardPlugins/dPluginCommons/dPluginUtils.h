/////////////////////////////////////////////////////////////////////////////
// Name:        dPluginStdafx.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////



#ifndef __D_PLUGIN_UTILS_H__
#define __D_PLUGIN_UTILS_H__


#ifdef _DPLUGIN_COMMON_DLL_EXPORT
	#ifdef _WIN32
		#define DPLUGIN_API __declspec (dllexport)
	#else
		#define DPLUGIN_API __attribute__ ((visibility("default")))
	#endif
#else
	#ifdef _WIN32
		#define DPLUGIN_API __declspec (dllimport)
	#else
		#define NEWTON_API
	#endif
#endif


class dPluginAlloc  
{
	public:
	dPluginAlloc  ()
	{
	}

	virtual ~dPluginAlloc  () 
	{
	}

	DPLUGIN_API void *operator new (size_t size);
	DPLUGIN_API void operator delete (void* ptr);
};


#ifdef _NEWTON_USE_DOUBLE
	inline void glMaterialParam (GLenum face, GLenum pname, const dFloat *params)
	{
		GLfloat tmp[4] = {params[0], params[1], params[2], params[3]};
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




// for some reason specifying a relative does not seem to work in Linus
// and i have to specify a absolute path
// #define ASSETS_PATH "."
DPLUGIN_API void GetMediaDirectory (char* const mediaDirOut);
DPLUGIN_API void GetAplicationDirectory (char* const aplicationDirOut);
DPLUGIN_API void GetWorkingFileName (const char* const name, char* const outPathNameOut);
DPLUGIN_API const char* GetNameFromPath (const char* const fullName);
DPLUGIN_API void ExtractPathFromFullName (const char* const fullName, char* const path);


// TODO: reference additional headers your program requires here

#endif