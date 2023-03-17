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

//int bits = 1;
////for (int k = 2; k <= blockStride; k = k << 1)
//for (int k = 2; k <= 64; k = k << 1)
//{
//	int highMask = -bits;
//	int lowMask = ~highMask;
//	for (int j = k >> 1; j > 0; j = j >> 1)
//	{
//		//for (int threadId = 0; threadId < blockStride / 2; ++threadId)
//		for (int threadId = 0; threadId < 64 / 2; ++threadId)
//		{
//			//int id0 = threadId;
//			int lowIndex = threadId & lowMask;
//			int highIndex = (threadId & highMask) * 2;
//
//			int id0 = highIndex + lowIndex;
//			int id1 = highIndex + lowIndex + bits;
//			//cuTrace(("%d ", id0));
//			cuTrace(("%d ", id1));
//			//int id1 = id0 ^ j;
//			//if (id1 > id0)
//			//{
//			//	const int a = sortedRadix[id0];
//			//	const int b = sortedRadix[id1];
//			//	const int mask0 = -(id0 & k);
//			//	const int mask1 = -(a > b);
//			//	const int mask2 = (mask0 ^ mask1) & 0x80000000;
//			//	if (mask2)
//			//	{
//			//		sortedRadix[id0] = b;
//			//		sortedRadix[id1] = a;
//			//	}
//			//}
//		}
//		cuTrace(("\n"));
//	}
//	bits *= 2;
//}


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