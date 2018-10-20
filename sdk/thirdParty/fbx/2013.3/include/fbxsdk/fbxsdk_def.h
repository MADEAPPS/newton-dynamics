/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

/** \file fbxsdk_def.h
  * FBX SDK environment definition.
  *
  * This file is the principal FBX SDK environment definition. It is used at the top of
  * every header and source file so that every unit is using the same definitions.
  */
#ifndef _FBXSDK_DEFINITION_H_
#define _FBXSDK_DEFINITION_H_

//---------------------------------------------------------------------------------------
//System Includes
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <wchar.h>
#include <locale.h>
#include <float.h>
#include <math.h>
#include <time.h>

//---------------------------------------------------------------------------------------
//Define Version and Namespace
#include <fbxsdk/fbxsdk_version.h>

//---------------------------------------------------------------------------------------
//Define Architecture
#include <fbxsdk/core/arch/fbxarch.h>
#include <fbxsdk/core/arch/fbxtypes.h>
#include <fbxsdk/core/arch/fbxdebug.h>
#include <fbxsdk/core/arch/fbxalloc.h>
#include <fbxsdk/core/arch/fbxnew.h>
#include <fbxsdk/core/arch/fbxstdcompliant.h>

//---------------------------------------------------------------------------------------
//Platform Standardization
#ifndef NULL
	#if defined(__GNUG__) && (__GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ >= 8))
		#define NULL (__null)
	#else	
    	#if defined(__cplusplus)
    		#define NULL 0
    	#else
    		#define NULL ((void*)0)
    	#endif
    #endif
#endif

#if !defined(_MAX_PATH)
	#define _MAX_PATH 260
#endif

#if defined(FBXSDK_ENV_WIN)
	#define snprintf _snprintf // for stdio.h platform compatibility
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN  // Defined to speed up compilation
	#endif
#endif

#if !defined(FBXSDK_COMPILER_MSC)
	#ifndef strcmpi
		#define strcmpi strcasecmp
	#endif
	#ifndef stricmp
		#define stricmp strcasecmp
	#endif
	#ifndef strncmpi
		#define strncmpi strncasecmp
	#endif
	#ifndef strnicmp
		#define strnicmp strncasecmp
	#endif
#endif

//---------------------------------------------------------------------------------------
//Compiler Specifics
#if defined(FBXSDK_COMPILER_MSC)
	#pragma warning(disable : 4251)	//'identifier' : class 'type' needs to have dll-interface to be used by clients of class 'type2'
    #if _MSC_VER >= 1300 // 7.1
        #define FBX_DEPRECATED __declspec(deprecated)
    #else
        #define FBX_DEPRECATED
    #endif
#elif defined(FBXSDK_COMPILER_GNU)
    #define FBX_DEPRECATED __attribute__((deprecated))
#elif defined(FBXSDK_COMPILER_INTEL)
    #if __INTEL_COMPILER >= 810
        #define FBX_DEPRECATED __declspec(deprecated)
    #else
        #define FBX_DEPRECATED
    #endif
#else
	#error Unsupported compiler!
#endif

//---------------------------------------------------------------------------------------
//Useful Macros
#define FBX_SAFE_DELETE(p)			{FbxDelete(p);(p)=NULL;}
#define FBX_SAFE_DELETE_ARRAY(a)	{FbxDeleteArray(a);(a)=NULL;}
#define FBX_SAFE_DESTROY(p)			if(p){(p)->Destroy();(p)=NULL;}
#define FBX_SAFE_FREE(p)			if(p){FbxFree(p);(p)=NULL;}

#endif /* _FBXSDK_DEFINITION_H_ */
