/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

/** \file fbxarch.h
  * Architecture definition.
  * 
  * List of available preprocessor defines that can appear on various systems:
  *
  * Operating System:
  *    FBXSDK_ENV_WIN (Windows)
  *    FBXSDK_ENV_MAC (MacOSX)
  *    FBXSDK_ENV_LINUX (Linux)
  *
  * Architecture:
  *    FBXSDK_ARCH_32 (Intel x86)
  *    FBXSDK_ARCH_64 (AMD64)
  *
  * Compiler:
  *    FBXSDK_COMPILER_MSC (Microsoft Compiler)
  *    FBXSDK_COMPILER_GNU (GNU Compiler)
  *    FBXSDK_COMPILER_INTEL (Intel Compiler)
  *
  * These definitions are based on the information found here:
  * http://predef.sourceforge.net/index.php
  */
#ifndef _FBXSDK_CORE_ARCH_ARCH_H_
#define _FBXSDK_CORE_ARCH_ARCH_H_

//------------------------------------------------------------------------------------

#if defined(_WIN32) || defined(_WIN64)	//Windows

	#define FBXSDK_ENV_WIN 1

	#if defined(_M_X64)
		#define FBXSDK_ARCH_64 1
	#elif defined(_M_IX86)
		#define FBXSDK_ARCH_32 1
	#else
		#error Unsupported architecture!
	#endif

	#if defined(_MSC_VER)
		#define FBXSDK_COMPILER_MSC 1
	#elif defined(__GNUC__)
		#define FBXSDK_COMPILER_GNU 1
	#elif defined(__ICL)
		#define FBXSDK_COMPILER_INTEL 1
	#else
		#error Unsupported compiler!
	#endif

#elif defined(__APPLE__) || defined(__MACH__)	//MacOS/X

	#define FBXSDK_ENV_MAC 1

	#if defined(__i386__)
		#define FBXSDK_ARCH_32 1
	#elif defined(__x86_64__) || defined(__x86_64)
		#define FBXSDK_ARCH_64 1
	#else
		#error Unsupported architecture!
	#endif

	#define FBXSDK_COMPILER_GNU 1

#elif defined(__linux__) || defined(__CYGWIN__)	//Linux

	#define FBXSDK_ENV_LINUX 1

	#if defined(__i386__)
		#define FBXSDK_ARCH_32 1
	#elif defined(__x86_64__) || defined(__x86_64)
		#define FBXSDK_ARCH_64 1
	#else
		#error Unsupported architecture!
	#endif

	#define FBXSDK_COMPILER_GNU 1

#else
	#error Unsupported platform!
#endif

#if defined(FBXSDK_SHARED) && defined(FBXSDK_ENV_WIN)
	#define FBXSDK_DLLIMPORT __declspec(dllimport)
	#define FBXSDK_DLLEXPORT __declspec(dllexport)
#else
	#define FBXSDK_DLLIMPORT
	#define FBXSDK_DLLEXPORT
#endif

#ifndef FBXSDK_DLL
	#define FBXSDK_DLL FBXSDK_DLLIMPORT
#endif

#endif /* _FBXSDK_CORE_ARCH_ARCH_H_ */
