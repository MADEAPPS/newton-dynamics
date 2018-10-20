/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

/** \file fbxsdk_version.h
  * FBX SDK version definition.
  *
  * This file defines the version string and numbers for this release of the FBX SDK.
  * \note This file should never be included directly, please include fbxsdk_def.h
  * instead.
  */
#ifndef _FBXSDK_VERSION_H_
#define _FBXSDK_VERSION_H_

//FBX SDK version defines
#define FBXSDK_VERSION_MAJOR	2013		//integer
#define FBXSDK_VERSION_MINOR	3			//integer
#define FBXSDK_VERSION_POINT	0			//integer
#define FBXSDK_VERSION_NAME		"Release"	//string, ex: Alpha1, Alpha2, Beta1, Beta2, RC, Release
#define FBXSDK_VERSION_DATE		20120911	//yyyymmdd

#ifndef FBXSDK_VERSION_REVISION
	#define FBXSDK_VERSION_REVISION	0		//set by environment, do not change here!
#endif

//FBX SDK version string macros
#define FBXSDK_DEF_TO_STR(x)	#x
#define FBXSDK_STRINGIFY(x)		FBXSDK_DEF_TO_STR(x)

#if FBXSDK_VERSION_POINT == 0
	#define FBXSDK_VER_TO_STR(a, b, c)	FBXSDK_DEF_TO_STR(a.b)
#else
	#define FBXSDK_VER_TO_STR(a, b, c)	FBXSDK_DEF_TO_STR(a.b.c)
#endif

//FBX SDK version string
#define FBXSDK_VERSION_STRING		FBXSDK_VER_TO_STR(FBXSDK_VERSION_MAJOR, FBXSDK_VERSION_MINOR, FBXSDK_VERSION_POINT)
#define FBXSDK_VERSION_STRING_FULL	FBXSDK_VERSION_STRING" "FBXSDK_VERSION_NAME" ("FBXSDK_STRINGIFY(FBXSDK_VERSION_REVISION)")"

//FBX SDK namespace definition
#ifndef FBXSDK_DEFINE_NAMESPACE
	#define FBXSDK_DEFINE_NAMESPACE 1
#endif

#if FBXSDK_DEFINE_NAMESPACE == 1
	#define FBXSDK_NAMESPACE fbxsdk_2013_3
#else
	#define FBXSDK_NAMESPACE
#endif

#endif /* _FBXSDK_VERSION_H_ */
