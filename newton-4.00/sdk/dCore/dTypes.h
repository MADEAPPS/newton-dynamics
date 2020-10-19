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

#ifndef __D_TYPES_H__
#define __D_TYPES_H__

#ifdef _MSC_VER 
	#ifdef _M_ARM
		#ifndef _ARM_VER
			#define _ARM_VER
		#endif
	#elif defined (_M_X64)
		#ifndef _WIN_64_VER
			#define _WIN_64_VER
		#endif
	#elif defined (_M_IX86)
		#ifndef _WIN_32_VER
			#define _WIN_32_VER
		#endif
	#else 
		#error target platform not defined
	#endif

	#if _MSC_VER >= 1400
		#define HAVE_STRUCT_TIMESPEC
	#endif
#endif

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER) || (defined (_MSC_VER ) && defined (_ARM_VER)) )
	#include <io.h> 
	#include <stdint.h>
	#include <direct.h> 
	#include <malloc.h>
	#include <stdarg.h>
	#include <process.h>

	#pragma warning (push, 3) 
		#include <windows.h>
		#include <crtdbg.h>
	#pragma warning (pop) 
#endif

#include <new>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <math.h>
#include <float.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <condition_variable>

#if (defined (__MINGW32__) || defined (__MINGW64__))
	#include <io.h> 
	#include <direct.h> 
	#include <malloc.h>
	#include <float.h>
	#include <windows.h>
	#include <process.h>
#endif

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#include <intrin.h>
	#include <emmintrin.h> 
	#include <pmmintrin.h>
#endif

#if (defined (_POSIX_VER) || defined (_POSIX_VER_64) || defined (__MINGW32__) || defined (__MINGW64__))
  // CMake defines NDEBUG for _not_ debug builds. Therefore, set
  // Newton's _DEBUG flag only when NDEBUG is not defined.

  #ifndef NDEBUG
    #define _DEBUG 1
  #endif

	#include <unistd.h>
	#include <assert.h>
	#if (!defined(__arm__) && !defined(__aarch64__)) // it was __ARMCC_VERSION before, it should be __ARM__ or aarch64, otherwise cross compiling in gcc fails.
		extern "C" 
		{ 
			// for SSE3 and up
			#include <pmmintrin.h> 
			#include <emmintrin.h> 
			#include <mmintrin.h> 
		} 
	#endif
#endif

#ifdef _MACOSX_VER
	#include <unistd.h>
	#include <sys/sysctl.h>
    #include <assert.h> 
    #if (defined __i386__ || defined __x86_64__)
		#include <fenv.h>
		#include <pmmintrin.h> 
		#include <emmintrin.h>  //sse3
        #include <mmintrin.h> 
    #endif
#endif

//#define DG_PROFILE_PHYSICS

// uncomment out D_PROFILER to enable profiler frame capture profiler traces
// alternatively the end application can use a command line option to enable this define
//#define D_PROFILER

// uncomment this for Scalar floating point 
// alternatively the end application can use a command line option to enable this define
//#define D_SCALAR_VECTOR_CLASS

// uncomment this for Scalar floating point 
// alternatively the end application can use a command line option to enable this define
//#define __ANDROID__

// by default newton run on a separate thread and 
// optionally concurrent with the calling thread,
// it also uses a thread job pool for multi core systems.
// define D_USE_THREAD_EMULATION on the command line for 
// platform that do not support hardware multi threading or 
// if the and application want to control threading at the application level 
//#define D_USE_THREAD_EMULATION

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#if _MSC_VER < 1700
		#ifndef D_USE_THREAD_EMULATION
			#define D_USE_THREAD_EMULATION
		#endif
	#endif
#endif

//************************************************************
#ifdef D_DISABLE_ASSERT
	#define dAssert(x)
#else 
	#if defined (_WIN_32_VER) || defined (_WIN_64_VER) 
		#define dAssert(x) _ASSERTE(x)
	#elif defined (_MSC_VER ) && defined (_ARM_VER) 
		#define dAssert(x) _ASSERTE(x)
	#else
		#ifdef _DEBUG
			#define dAssert(x) assert(x)
		#else 
			#define dAssert(x)
		#endif
	#endif
#endif

#ifdef _DEBUG
//#define __ENABLE_D_CONTAINERS_SANITY_CHECK 
#endif

#ifdef _DEBUG
	#define D_INLINE inline
#else
	#if defined(_MSC_VER)
		#define D_INLINE __forceinline 
	#else 
		#define D_INLINE	inline
		//#define D_INLINE	 __attribute__((always_inline))
	#endif
#endif

#if defined (D_USE_VECTOR_AVX)
	#if defined(_MSC_VER)
		#define	D_MSV_NEWTON_ALIGN_16    __declspec(align(16))
		#define	D_GCC_NEWTON_ALIGN_16 	 

		#define	D_MSV_NEWTON_ALIGN_32    __declspec(align(32))
		#define	D_GCC_NEWTON_ALIGN_32 	 
	#else
		#define	D_MSV_NEWTON_ALIGN_16
		#define	D_GCC_NEWTON_ALIGN_16     __attribute__((aligned (16)))

		#define	D_MSV_NEWTON_ALIGN_32
		#define	D_GCC_NEWTON_ALIGN_32     __attribute__((aligned (32)))
	#endif
#else
	#if defined(_MSC_VER)
		#define	D_GCC_NEWTON_ALIGN_16 	 
		#define	D_MSV_NEWTON_ALIGN_16    __declspec(align(16))
	#else
		#define	D_GCC_NEWTON_ALIGN_16     __attribute__((aligned (16)))
		#define	D_MSV_NEWTON_ALIGN_16

		#define	D_GCC_NEWTON_ALIGN_32     __attribute__((aligned (32)))
		#define	D_MSV_NEWTON_ALIGN_32
	#endif

	#define	D_MSV_NEWTON_ALIGN_32 D_MSV_NEWTON_ALIGN_16   
	#define	D_GCC_NEWTON_ALIGN_32 D_GCC_NEWTON_ALIGN_16	 
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

#if ((defined (_WIN_32_VER) || defined (_WIN_64_VER)) && (_MSC_VER  >= 1600))
	typedef int8_t dInt8;
	typedef uint8_t dUnsigned8;

	typedef int16_t dInt16;
	typedef uint16_t dUnsigned16;

	typedef int32_t dInt32;
	typedef uint32_t dUnsigned32;

	typedef int64_t dInt64;
	typedef uint64_t dUnsigned64;
#else
	typedef char dInt8;
	typedef unsigned char dUnsigned8;

	typedef short dInt16;
	typedef unsigned short dUnsigned16;

	typedef int dInt32;
	typedef unsigned dUnsigned32;
	typedef unsigned int dUnsigned32;

	typedef long long dInt64;
	typedef unsigned long long dUnsigned64;
	typedef double dFloat64;
#endif

typedef double dFloat64;

#ifdef D_NEWTON_USE_DOUBLE
	typedef double dFloat32;
#else
	typedef float dFloat32;
#endif

class dTriplex
{
	public:
	dFloat32 m_x;
	dFloat32 m_y;
	dFloat32 m_z;
};

#define dPi		 		dFloat32 (3.141592f)
//#define dPi2		 	dFloat32 (dPi * 2.0f)
#define dEXP		 	dFloat32 (2.71828f)
#define dEpsilon	  	dFloat32 (1.0e-5f)
#define dDegreeToRad	dFloat32 (dPi / 180.0f)
#define dRadToDegree  	dFloat32 (180.0f / dPi)

#define dSqrt(x)		dFloat32 (sqrt(x))	
#define dSin(x)			dFloat32 (sin(x))
#define dCos(x)			dFloat32 (cos(x))
#define dAsin(x)		dFloat32 (asin(x))
#define dAcos(x)		dFloat32 (acos(x))
#define dLog(x)			dFloat32 (log(x))
#define dCeil(x)		dFloat32 (ceil(x))
#define dFloor(x)		dFloat32 (floor(x))	
#define dPow(x,y)		dFloat32 (pow(x,y))
#define dFmod(x,y)		dFloat32 (fmod(x,y))
#define dTan(x)			dFloat32 (tan(x))
#define dAtan2(x,y)		dFloat32 (atan2(x,y))
#define dRsqrt(x)		(dFloat32 (1.0f) / dSqrt(x))
#define dClearFP()		_clearfp() 
#define dControlFP(x,y)	_controlfp(x,y)

class dBigVector;
#ifndef D_NEWTON_USE_DOUBLE
	class dVector;
#endif 

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#define dCheckFloat(x) (_finite(x) && !_isnan(x))
//	#define dCheckFloat(x) 1
#else
	#define dCheckFloat(x) (isfinite(x) && !isnan(x))
//		#define dCheckFloat(x) 1
#endif

//typedef void (*dDeserialize) (void* const userData, void* buffer, dInt32 size);
//typedef void (*dSerialize) (void* const userData, const void* const buffer, dInt32 size);
//typedef bool (*dReportProgress) (dFloat32 progressNormalzedPercent, void* const userData);

// assume this function returns memory aligned to 16 bytes
#define dAlloca(type, count) (type*) alloca (sizeof (type) * (count))

D_INLINE dInt32 dExp2 (dInt32 x)
{
	dInt32 exp;
	for (exp = -1; x; x >>= 1) 
	{
		exp ++;
	}
	return exp;
}

D_INLINE dInt32 dBitReversal(dInt32 v, dInt32 base)
{
	dInt32 x = 0;
	dInt32 power = dExp2 (base) - 1;
	do 
	{
		x += (v & 1) << power;
		v >>= 1;
		power--;
	} while (v);
	dAssert(x < base);
	return x;
}

template <class T>
T dMod(T val, T mod)
{
	return T(fmod(T(val), T(mod)));
}

template <class T> 
D_INLINE T dMin(T A, T B)
{
	return (A < B) ? A : B; 
}

template <class T> 
D_INLINE T dMax(T A, T B)
{
	return (A > B) ? A : B; 
}

template <class T>
D_INLINE T dMin(T A, T B, T C)
{
	return dMin(dMin (A, B), C);
}

template <class T>
D_INLINE T dMax(T A, T B, T C)
{
	return dMax(dMax (A, B), C);
}

template <class T>
D_INLINE T dClamp(T val, T min, T max)
{
	return dMax (min, dMin (max, val));
}

template <class T> 
D_INLINE void dSwap(T& A, T& B)
{
	T tmp (A);
	A = B;
	B = tmp;
}	

template <class T>
D_INLINE T dAbs(T A)
{
	// according to Intel this is better because is does not read after write
	return (A >= T(0)) ? A : -A;
}

template <class T>
D_INLINE T dSign(T A)
{
	return (A >= T(0)) ? T(1) : T(-1);
}

template <class T> 
D_INLINE bool dAreEqual(T A, T B, T tol)
{
	if ((dAbs(A) < tol) && (dAbs(B) < tol)) {
		return true;
	}
/*
	dInt32 exp0;
	dFloat64 mantissa0 = frexp(dFloat64 (A), &exp0);

	dInt32 exp1;
	dFloat64 mantissa1 = frexp(dFloat64(B), &exp1);

	if ((exp0 < -12) && (exp1 < -12)) {
		return true;
	}

	if (exp0 != exp1) {
		return false;
	}
	return dAbs(mantissa0 - mantissa1) < tol;
*/	
	T den = dMax(dAbs(A), dAbs(B)) + tol;
	A /= den;
	B /= den;
	return dAbs(A - B) < tol;
}

template <class T>
D_INLINE T AnglesAdd (T angleInRadiand1, T angleInRadiand0)
{
	T s1 = T(dSin(angleInRadiand1));
	T c1 = T(dCos(angleInRadiand1));
	T s0 = T(dSin(angleInRadiand0));
	T c0 = T(dCos(angleInRadiand0));

	T s = s1 * c0 + s0 * c1;
	T c = c1 * c0 - s0 * s1;
	return T(dAtan2(s, c));
}

#ifdef D_NEWTON_USE_DOUBLE
	union dFloatSign
	{
		struct {
			dInt32 m_dommy;
			dInt32 m_iVal;
		} m_integer;
		dFloat64 m_fVal;
	};
#else
	union dFloatSign
	{
		struct {
			dInt32 m_iVal;
		} m_integer;
		dFloat32 m_fVal;
	};
#endif

union dDoubleInt
{
	struct {
		dInt32 m_intL;
		dInt32 m_intH;
	};
	void* m_ptr;
	dInt64 m_int;
	dFloat64 m_float;
};

#define PointerToInt(x) ((size_t)x)
#define IntToPointer(x) ((void*)(size_t(x)))

#ifndef _MSC_VER 
	#define _stricmp(x,y) strcasecmp(x,y)
#endif

enum dSerializeRevisionNumber
{
	m_firstRevision = 100,
	// add new serialization revision number here
	m_currentRevision 
};

D_CORE_API dUnsigned64 dGetTimeInMicrosenconds();

#ifdef D_USE_THREAD_EMULATION
	template<class T>
	class dAtomic
	{
		public:
		dAtomic<T>(T val)
			: m_val(val)
		{
		}

		T load() const
		{
			return m_val;
		}

		void store(T val)
		{
			m_val = val;
		}

		T exchange(T val)
		{
			dSwap(val, m_val);
			return val;
		}

		T fetch_add(T val)
		{
			m_val += val;
			return m_val - val;
		}

		T fetch_sub(T val)
		{
			m_val -= val;
			return m_val + val;
		}

		private:
		T m_val;
	};
#else
	template<class T>
	class dAtomic: public std::atomic<T>
	{
		public: 
		dAtomic<T>(T val)
			:std::atomic<T>(val)
		{
		}
	};
#endif

class dSpinLock
{
	public:
	dSpinLock()
#ifndef D_USE_THREAD_EMULATION	
		:m_lock(0)
#endif
	{
	}

	void Lock()
	{
#ifndef D_USE_THREAD_EMULATION	
		while (m_lock.exchange(1)) 
		{
			std::this_thread::yield();
		}
#endif
	}

	void Unlock()
	{
#ifndef D_USE_THREAD_EMULATION	
		m_lock.exchange(0);
#endif
	}

#ifndef D_USE_THREAD_EMULATION	
	private:
	dAtomic<dUnsigned32> m_lock;
#endif
};

class dScopeSpinLock
{
	public:
	dScopeSpinLock(dSpinLock& spinLock)
		:m_spinLock(spinLock)
	{
		m_spinLock.Lock();
	}

	~dScopeSpinLock()
	{
		m_spinLock.Unlock();
	}

	dSpinLock& m_spinLock;
};


//void dGetMinMax (dBigVector &Min, dBigVector &Max, const dFloat64* const vArray, dInt32 vCount, dInt32 strideInBytes);
D_CORE_API dInt32 dVertexListToIndexList (dFloat64* const vertexList, dInt32 strideInBytes, dInt32 compareCount,     dInt32 vertexCount,         dInt32* const indexListOut, dFloat64 tolerance = dEpsilon);
D_CORE_API dInt32 dVertexListToIndexList (dFloat32* const vertexList, dInt32 strideInBytes, dInt32 floatSizeInBytes, dInt32 unsignedSizeInBytes, dInt32 vertexCount, dInt32* const indexListOut, dFloat32 tolerance = dEpsilon);
//dFloat64 dRoundToFloat(dFloat64 val);
//void dSerializeMarker(dSerialize serializeCallback, void* const userData);
//dInt32 dDeserializeMarker(dDeserialize serializeCallback, void* const userData);

class dFloatExceptions
{
	public:
	#ifdef _MSC_VER
		#define D_FLOAT_EXECTIONS_MASK	(EM_INVALID | EM_DENORMAL | EM_ZERODIVIDE)
	#else 
		#define D_FLOAT_EXECTIONS_MASK	0
	#endif

	dFloatExceptions(dUnsigned32 mask = D_FLOAT_EXECTIONS_MASK);
	~dFloatExceptions();

	private:
	//#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	#if defined (_MSC_VER)
		dUnsigned32 m_mask;
	#endif
};

class dSetPrecisionDouble 
{
	public:
	dSetPrecisionDouble();
	~dSetPrecisionDouble();
	#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	dInt32 m_mask; 
	#endif
};


#ifdef _MACOSX_VER
#include <sys/time.h>
#ifndef CLOCK_REALTIME
	#define CLOCK_REALTIME 0
#endif
#ifndef CLOCK_MONOTONIC
	#define CLOCK_MONOTONIC 0
#endif
//clock_gettime is not implemented on OSX
D_INLINE int clock_gettime(int /*clk_id*/, struct timespec* t) {
    struct timeval now;
    int rv = gettimeofday(&now, nullptr);
    if (rv) return rv;
    t->tv_sec  = now.tv_sec;
    t->tv_nsec = now.tv_usec * 1000;
    return 0;
}
#endif

#endif

