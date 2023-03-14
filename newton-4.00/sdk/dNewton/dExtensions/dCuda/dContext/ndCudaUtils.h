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

#ifndef __ND_CUDA_UTILS_H__
#define __ND_CUDA_UTILS_H__

#include "ndCudaStdafx.h"

#ifdef D_DISABLE_ASSERT
	#define ndAssert(x)
#else 
	#ifdef _DEBUG
		#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
			#define ndAssert(x) _ASSERTE(x)
		#else
			#define ndAssert(x) assert(x)
	#endif
	#else 
		#define ndAssert(x)
	#endif
#endif

#ifdef _MSC_VER 
	#ifdef _DEBUG 
		#define CUDA_TRACE
	#endif
#endif

void cudaExpandTraceMessage(const char* const fmt, ...);

#ifdef CUDA_TRACE
	#define cuTrace(x) cudaExpandTraceMessage x;
#else
	#define cuTrace(x);
#endif

typedef void* (*ndMemAllocCallback) (size_t size);
typedef void (*ndMemFreeCallback) (void* const ptr);

D_CUDA_API void* ndCudaMalloc(size_t size);
D_CUDA_API void ndCudaFree(void* const ptr);

#define ndAlloca(type, count) (type*) alloca (sizeof (type) * (count))

#define D_CUDA_OPERATOR_NEW_AND_DELETE		\
inline void *operator new (size_t size)		\
{											\
	return ndCudaMalloc(size);				\
}											\
											\
inline void *operator new[](size_t size) 	\
{											\
	return ndCudaMalloc(size);				\
}											\
											\
inline void operator delete (void* ptr)		\
{											\
	ndCudaFree(ptr);						\
}											\
											\
inline void operator delete[](void* ptr)	\
{											\
	ndCudaFree(ptr);						\
}

#endif