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
// or project specific include files that are used frequently, but
// are changed infrequently
//

#ifndef __NEWTON_ANIMATION_STDAFX__
#define __NEWTON_ANIMATION_STDAFX__

#include <stdlib.h>
#include <string.h>

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

// TODO: reference additional headers your program requires here

#if _MSC_VER
#include <windows.h>
#include <crtdbg.h> 
	#pragma warning (disable: 4996) // for 2005 users declared deprecated
	#pragma warning (disable: 4100) // unreferenced formal parameter
	#pragma warning (disable: 4505) // unreferenced local function has been removed
#endif


#include <dCRC.h>
#include <dList.h>
#include <dTree.h>
#include <dRefCounter.h>
#include <dMathDefines.h>
#include <dVector.h>
#include <dMatrix.h>
#include <dQuaternion.h>
#include <Newton.h>
#include <dBaseHierarchy.h>

//#define D_LOAD_SAVE_XML

#endif
