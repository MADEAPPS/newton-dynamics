/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_CUDA_STDAFX_H__
#define __ND_CUDA_STDAFX_H__

#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
	#include <io.h>
	#include <stdio.h>
	#include <stdint.h>
	#include <direct.h>
	#include <malloc.h>
	#include <stdarg.h>
	#include <process.h>

	#pragma warning (push, 3)
	#include <windows.h>
	#include <crtdbg.h>
	#pragma warning (pop)
#else
	#include <sys/stat.h>
#endif

#ifdef _D_CUDA_EXPORT_DLL
	#define D_CUDA_API __declspec(dllexport)
#else
	#define D_CUDA_API __declspec(dllimport)
#endif

#endif 

