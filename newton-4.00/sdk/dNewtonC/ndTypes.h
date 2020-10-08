/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_TYPES_C_H__
#define __D_TYPES_C_H__

#if defined(_MSC_VER)
	#define ND_LIBRARY_EXPORT __declspec(dllexport)
	#define ND_LIBRARY_IMPORT __declspec(dllimport)
#else
	#define ND_LIBRARY_EXPORT __attribute__((visibility("default")))
	#define ND_LIBRARY_IMPORT __attribute__((visibility("default")))
#endif

#ifdef _D_NEWTON_BUILD_DLL
	#define NEWTON_API ND_LIBRARY_EXPORT
#else
	#define NEWTON_API ND_LIBRARY_IMPORT
#endif


#ifdef __cplusplus 
extern "C" {
#endif

	typedef void* (*ndMalloc) (size_t sizeInBytes);
	typedef void(*ncFree) (void* const ptr);

	NEWTON_API size_t ndGetMemoryUsed();
	NEWTON_API void ndSetAllocators(ndMalloc malloc, ncFree free);

#ifdef __cplusplus 
}
#endif

#endif
