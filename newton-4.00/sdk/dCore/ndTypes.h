/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_TYPES_H__
#define __ND_TYPES_H__

#ifdef _MSC_VER 
	#if _MSC_VER >= 1400
		#define HAVE_STRUCT_TIMESPEC
	#endif
#endif

#define ND_NEWTON_VERSION	403

#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
	#define NOMINMAX
	#include <io.h>
	#include <stdint.h>
	#include <direct.h>
	#include <malloc.h>
	#include <stdarg.h>
	#include <process.h>
	#include <windows.h>
	#include <crtdbg.h>
#else
  #include <sys/stat.h>
#endif

#include <new>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <random> 
#include <math.h>
#include <fenv.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <locale.h>
//#include <semaphore> only in cpp 20 or later
#include <condition_variable>

// weird workaround but i am not sure how a global compiler flag would affect the rest of the code
// probably useless since this functionality will not be used/implemented but avoid compiler errors
#if defined(__STRICT_ANSI__) && (defined (__MINGW32__) || defined (__MINGW64__))
	#pragma push_macro("__STRICT_ANSI__")
	#undef __STRICT_ANSI__
#endif

#include <float.h>
#if (defined (__MINGW32__) || defined (__MINGW64__))
	#pragma pop_macro("__STRICT_ANSI__")
#endif

#if (defined (__MINGW32__) || defined (__MINGW64__))
	#include <io.h> 
	#include <direct.h> 
	#include <malloc.h>
	#include <windows.h>
	#include <process.h>
#endif

#if (defined (__linux__ ) || defined (__MINGW32__) || defined (__MINGW64__))
	//CMake defines NDEBUG for _not_ debug builds. Therefore, set
	//Newton's _DEBUG flag only when NDEBUG is not defined.

	#ifndef NDEBUG
	#define _DEBUG 1
	#endif

	#include <unistd.h>
	#include <assert.h>
	#include <cstdint>
	// it was __ARMCC_VERSION before, it should be __ARM__ or aarch64, otherwise cross compiling in gcc fails.
	#if (!defined(__arm__) && !defined(__aarch64__)) 
		extern "C" 
		{ 
			// for SSE3 and up
			#include <immintrin.h>
			#include <pmmintrin.h> 
			#include <emmintrin.h> 
			#include <mmintrin.h> 
		} 
	#endif
#endif

#if defined (__x86_64) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
	#include <immintrin.h>
#endif

#if defined(__arm__) && defined(__aarch64__) 
	#include <arm_neon.h>
#endif

#if defined (__APPLE__)
	#include <unistd.h>
	#include <sys/sysctl.h>
    #include <assert.h> 
    #if (defined __i386__ || defined __x86_64__)
		#include <immintrin.h>
		#include <pmmintrin.h> 
		#include <emmintrin.h>  //sse3
        #include <mmintrin.h> 
    #endif
#endif

// uncommemt this line to enable memory corruption checks, (very slow and use lots of memory)
//#define D_MEMORY_SANITY_CHECK

// uncomment this to inforce double precision 
//#define D_NEWTON_USE_DOUBLE

// uncomment this for Scalar floating point 
// alternatively the end application can use a command line option to enable this define
//#define D_SCALAR_VECTOR_CLASS

// uncomment out D_FORCE_PROFILE_PHYSICS to enable profiler frame capture profiler traces
// alternatively the end application can use a command line option to enable this define
//#define D_FORCE_PROFILE_PHYSICS

// uncomment this for Scalar floating point 
// alternatively the end application can use a command line option to enable this define
//#define __ANDROID__

// by default newton run on a separate thread concurrent with the calling thread.
// it also uses a thread job pool for multi core systems.
// uncoment this define for platforms that do not support hardware multi threading 
// or applications that wants to control threading at the application level. 
//#define D_USE_FORCE_THREAD_EMULATION

#ifdef D_USE_FORCE_THREAD_EMULATION
	#ifndef D_USE_THREAD_EMULATION
		#define D_USE_THREAD_EMULATION
	#endif
#endif

#ifdef D_FORCE_PROFILE_PHYSICS
	#ifndef D_PROFILER
		#define D_PROFILER
	#endif
#endif

//************************************************************
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

//#define D_NEWTON_ENFORCE_CLASS_ALIGNMENT

#if defined(D_NEWTON_ENFORCE_CLASS_ALIGNMENT)
	// enfoce compiler decide the aligments
	#if defined(_MSC_VER)
		#define	D_GCC_NEWTON_CLASS_ALIGN_16 	 
		#define	D_MSV_NEWTON_CLASS_ALIGN_16		__declspec(align(16))
		#define	D_GCC_NEWTON_CLASS_ALIGN_32 
		#define	D_MSV_NEWTON_CLASS_ALIGN_32		__declspec(align(32))
	#else
		#define	D_GCC_NEWTON_CLASS_ALIGN_16     __attribute__((aligned (16)))
		#define	D_MSV_NEWTON_CLASS_ALIGN_16
		#define	D_GCC_NEWTON_CLASS_ALIGN_32     __attribute__((aligned (32)))
		#define	D_MSV_NEWTON_CLASS_ALIGN_32
	#endif
#else
	// let the compiler decide the aligments
	#define	D_GCC_NEWTON_CLASS_ALIGN_16
	#define	D_MSV_NEWTON_CLASS_ALIGN_16
	#define	D_GCC_NEWTON_CLASS_ALIGN_32
	#define	D_MSV_NEWTON_CLASS_ALIGN_32
#endif

#if defined(_MSC_VER)
	#define D_LIBRARY_EXPORT __declspec(dllexport)
	#define D_LIBRARY_IMPORT __declspec(dllimport)
#else
	#define D_LIBRARY_EXPORT __attribute__((visibility("default")))
	#define D_LIBRARY_IMPORT __attribute__((visibility("default")))
#endif

#ifdef _D_CORE_DLL
	#ifdef _D_CORE_EXPORT_DLL
		#define D_CORE_API D_LIBRARY_EXPORT
	#else
		#define D_CORE_API D_LIBRARY_IMPORT
	#endif
#else
	#define D_CORE_API 
#endif

typedef float ndReal;
typedef int8_t ndInt8;
typedef uint8_t ndUnsigned8;
typedef int16_t ndInt16;
typedef uint16_t ndUnsigned16;
typedef int32_t ndInt32;
typedef uint32_t ndUnsigned32;
typedef int64_t ndInt64;
typedef uint64_t ndUnsigned64;

typedef double ndFloat64;
#ifdef D_NEWTON_USE_DOUBLE
	typedef double ndFloat32;
#else
	typedef float ndFloat32;
#endif
	
#define ndPi	 		ndFloat32 (3.141592f)
#define ndEpsilon	  	ndFloat32 (1.0e-5f)
#define ndDegreeToRad	ndFloat32 (ndPi / 180.0f)
#define ndRadToDegree  	ndFloat32 (180.0f / ndPi)

extern D_CORE_API ndFloat32 ndExp_VS__Fix(ndFloat64 x);

#define ndSqrt(x)		ndFloat32 (sqrt(x))	
#define ndSin(x)		ndFloat32 (sin(x))
#define ndCos(x)		ndFloat32 (cos(x))
#define ndAsin(x)		ndFloat32 (asin(x))
#define ndAcos(x)		ndFloat32 (acos(x))
#define ndLog(x)		ndFloat32 (log(x))
#define ndCeil(x)		ndFloat32 (ceil(x))
#define ndFloor(x)		ndFloat32 (floor(x))	
//#define ndExp(x)		ndFloat32 (exp(x))
#define ndExp(x)		ndExp_VS__Fix(x)
#define ndPow(x,y)		ndFloat32 (pow(x,y))
#define ndFmod(x,y)		ndFloat32 (fmod(x,y))
#define ndTan(x)		ndFloat32 (tan(x))
#define ndTanh(x)		ndFloat32 (tanh(x))
#define ndAtan(x)		ndFloat32 (atan(x))
#define ndAtan2(x,y)	ndFloat32 (atan2(x,y))
#define ndRsqrt(x)		(ndFloat32 (1.0f) / ndSqrt(x))

#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
	#define ndCheckFloat(x) (1)
	//#define ndCheckFloat(x) (_finite(x) && !_isnan(x))
#else
	//#define ndCheckFloat(x) (isfinite(x) && !isnan(x))
	#define ndCheckFloat(x) (1)
#endif

#ifdef D_NEWTON_USE_DOUBLE
	union ndFloatSign
	{
		ndInt64 m_iVal;
		ndFloat64 m_fVal;
	};
#else
	union ndFloatSign
	{
		ndInt32 m_iVal;
		ndFloat32 m_fVal;
	};
#endif

union ndIntPtr
{
	ndIntPtr()
		:m_int(0)
	{
	}
	void* m_ptr;
	ndInt64 m_int;
};

union ndDoubleInt
{
	struct 
	{
		ndInt32 m_intL;
		ndInt32 m_intH;
	};
	void* m_ptr;
	ndInt64 m_int;
	ndFloat64 m_float;
};

class ndTriplex
{
	public:
	ndFloat32 m_x;
	ndFloat32 m_y;
	ndFloat32 m_z;
};

#define D_BASE_CLASS_REFLECTION(Class)			\
	virtual const char* ClassName() const		\
	{											\
		return #Class;							\
	}											\
	static const char* StaticClassName()		\
	{											\
		return #Class;							\
	}											\
	virtual const char* SuperClassName() const	\
	{											\
		return #Class;							\
	}

#define D_CLASS_REFLECTION(Class,SuperClass)			\
	virtual const char* ClassName() const override		\
	{													\
		return #Class;									\
	}													\
	static const char* StaticClassName()				\
	{													\
		return #Class;									\
	}													\
	virtual const char* SuperClassName() const override	\
	{													\
		return #SuperClass;								\
	}

#define D_OPERATOR_NEW_AND_DELETE			\
inline void *operator new (size_t size)		\
{											\
	return ndMemory::Malloc(size);			\
}											\
											\
inline void *operator new[](size_t size) 	\
{											\
	return ndMemory::Malloc(size);			\
}											\
											\
inline void operator delete (void* ptr)		\
{											\
	ndMemory::Free(ptr);					\
}											\
											\
inline void operator delete[](void* ptr)	\
{											\
	ndMemory::Free(ptr);					\
}

#endif

