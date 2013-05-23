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

// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently


#ifndef _TOLLBOX_STDAFX_H_
#define _TOLLBOX_STDAFX_H_

// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers


typedef char dInt8;
typedef unsigned char dUnsigned8;

typedef short dInt16;
typedef unsigned short dUnsigned16;

typedef int dInt32;
typedef unsigned dUnsigned32;
typedef unsigned int dUnsigned32;


typedef long long unsigned64;


#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <ctype.h>


#define USE_STATIC_MESHES_DEBUG_COLLISION
//#define USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK


#ifdef _MSC_VER
	#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
	#include <windows.h>
	#include <Commctrl.h>
	#include <stdlib.h>
	#include <malloc.h>
	#include <memory.h>
	#include <time.h>
	#include <tchar.h>
	#include <crtdbg.h>

	// opengl stuff
	#include <GL/glew.h>
	#include <GL/wglew.h>
	#include <gl/gl.h>
	#include <gl/glu.h>

	// font library for open gl text
	#include <ft2build.h>
	#include FT_FREETYPE_H

	// audio library support
	#include <al/al.h>
	#include <al/alc.h>

	// gui library includes
	#include <wx/wx.h>
	#include <wx/event.h>
	#include <wx/dcclient.h>
	#include <wx/glcanvas.h>


#endif
	
#ifdef _POSIX_VER
	#include <stdlib.h>
	#include <unistd.h>
	#include <time.h>
	#include <GL/glxew.h>
	#include <GL/glew.h>
	#include <GL/glu.h>
	#include <GL/gl.h>

	// audio library support
	#include <AL/al.h>
	#include <AL/alc.h>

	// gui library includes
	#include <wx/wx.h>
	#include <wx/event.h>
	#include <wx/dcclient.h>
	#include <wx/glcanvas.h>


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

	// gui library includes
	#include <wx/wx.h>
	#include <wx/event.h>
	#include <wx/dcclient.h>
	#include <wx/glcanvas.h>
#endif



#ifdef _MSC_VER
	#pragma warning (disable: 4100) //unreferenced formal parameter
	#pragma warning (disable: 4505) //unreferenced local function has been removed
	#pragma warning (disable: 4201) //nonstandard extension used : nameless struct/union
	#pragma warning (disable: 4127) //conditional expression is constant

	#if (_MSC_VER >= 1400)
		#pragma warning (disable: 4996) // for 2005 users declared deprecated
	#endif
#endif



// SDK includes
#include <Newton.h>
#include <dVector.h>
#include <dMatrix.h>
#include <dQuaternion.h>
#include <dMathDefines.h>
#include <NewtonDebuggerServer.h>
#include <CustomControllerManager.h>

// AI wrapper 
#include <dAI.h>
#include <dAIAgent.h>
#include <dAIAgentState.h>
#include <dAIAgentVehicleController.h>
#include <dAIAgentCharacterController.h>

#include <dCRC.h>
#include <dHeap.h>
#include <dList.h>
#include <dTree.h>
#include <dRtti.h>
#include <dClassInfo.h>
#include <dRefCounter.h>
#include <dBaseHierarchy.h>

#include "dSceneStdafx.h"
#include "dScene.h"
#include "dRootNodeInfo.h"
#include "dBoneNodeInfo.h"
#include "dSceneNodeInfo.h"
#include "dMeshNodeInfo.h"
#include "dTextureNodeInfo.h"
#include "dMaterialNodeInfo.h"
#include "dRigidbodyNodeInfo.h"
#include "dCollisionNodeInfo.h"
#include "dCollisionBoxNodeInfo.h"
#include "dCollisionSphereNodeInfo.h"
#include "dCollisionConvexHullNodeInfo.h"
#include "dGeometryNodeSkinModifierInfo.h"





#ifdef __USE_DOUBLE_PRECISION__
	#define glMultMatrix(x) glMultMatrixd(x)
	#define glLoadMatrix(x) glMultMatrixd(x)
//	#define glGetFloat(x,y) glGetDoublev(x,(GLdouble *)y) 
#else
	#define glMultMatrix(x) glMultMatrixf(x)
	#define glLoadMatrix(x) glMultMatrixf(x)
	#define glGetFloat(x,y) glGetFloatv(x,(dFloat  *)y) 
#endif



#ifndef _MSC_VER
	#ifndef stricmp
		#define stricmp strcasecmp
	#endif

	#ifndef strlwr
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



#define dRAND_MAX		0x0fffffff

unsigned dRand ();
dFloat RandomVariable(dFloat amp);


// for some reason specifying a relative does not seem to work in Linus
// and i have to specify a absolute path
// #define ASSETS_PATH "."
//void GetAplicationDirectory (char* const aplicationDir);
void GetWorkingFileName (const char* const name, char* const outPathName);


// little Indian/big Indian conversion
unsigned SWAP_INT32(unsigned x);
unsigned short SWAP_INT16(unsigned short x);
void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count);



#endif 

