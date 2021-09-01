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

#ifndef __D_TYPES_H__
#define __D_TYPES_H__

#ifdef _MSC_VER 
	#if _MSC_VER >= 1400
		#define HAVE_STRUCT_TIMESPEC
	#endif
#endif

#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
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
#else
  #include <sys/stat.h>
#endif

#include <new>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <locale.h>
#include <tinyxml.h>
#include <immintrin.h>
#include <condition_variable>

// we need _clearfp() and _controlfp() from float.h which are excluded if __STRICT_ANSI__ is defined
// weird workaround but i'm not sure how a global compiler flag would affect the rest of the code
// probably useless since this functionality will not be used/implemented but to avoid compiler errors
#if defined(__STRICT_ANSI__) && (defined (__MINGW32__) || defined (__MINGW64__))
	#pragma push_macro("__STRICT_ANSI__")
	#undef __STRICT_ANSI__
#endif

// include without __STRICT_ANSI__
#include <float.h>
// restore __STRICT_ANSI__ back
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

#if defined (__APPLE__)
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

#if (defined(WIN32) || defined(_WIN32))
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
	#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
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

#if defined(_MSC_VER)
	#define	D_GCC_NEWTON_ALIGN_16 	 
	#define	D_MSV_NEWTON_ALIGN_16	__declspec(align(16))

	#define	D_GCC_NEWTON_ALIGN_32 
	#define	D_MSV_NEWTON_ALIGN_32	__declspec(align(32))
#else
	#define	D_GCC_NEWTON_ALIGN_16     __attribute__((aligned (16)))
	#define	D_MSV_NEWTON_ALIGN_16

	#define	D_GCC_NEWTON_ALIGN_32     __attribute__((aligned (32)))
	#define	D_MSV_NEWTON_ALIGN_32
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

typedef int8_t dInt8;
typedef uint8_t dUnsigned8;
typedef int16_t dInt16;
typedef uint16_t dUnsigned16;
typedef int32_t dInt32;
typedef uint32_t dUnsigned32;
typedef int64_t dInt64;
typedef uint64_t dUnsigned64;

typedef double dFloat64;
#ifdef D_NEWTON_USE_DOUBLE
	typedef double dFloat32;
#else
	typedef float dFloat32;
#endif

#define dPi		 		dFloat32 (3.141592f)
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

#if (defined (WIN32) || defined(_WIN32) || defined (_M_ARM) || defined (_M_ARM64))
	#define dCheckFloat(x) (_finite(x) && !_isnan(x))
#else
	#define dCheckFloat(x) (isfinite(x) && !isnan(x))
#endif

#ifdef D_NEWTON_USE_DOUBLE
	union dFloatSign
	{
		struct 
		{
			dInt32 m_dommy;
			dInt32 m_iVal;
		} m_integer;
		dFloat64 m_fVal;
	};
#else
	union dFloatSign
	{
		struct 
		{
			dInt32 m_iVal;
		} m_integer;
		dFloat32 m_fVal;
	};
#endif

union dDoubleInt
{
	struct 
	{
		dInt32 m_intL;
		dInt32 m_intH;
	};
	void* m_ptr;
	dInt64 m_int;
	dFloat64 m_float;
};

class dTriplex
{
	public:
	dFloat32 m_x;
	dFloat32 m_y;
	dFloat32 m_z;
};

#ifndef _MSC_VER 
	#define _stricmp(x,y) strcasecmp(x,y)
#endif

#ifdef D_USE_THREAD_EMULATION
	/// wrapper over standard atomic operations
	template<class T>
	class dAtomic
	{
		public:
		dAtomic<T>()
			: m_val(T(0))
		{
		}

		dAtomic<T>(T val)
			: m_val(val)
		{
		}

		operator T() const
		{
			return m_val;
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
			T ret = m_val;
			m_val += val;
			return ret;
		}

		T fetch_sub(T val)
		{
			T ret = m_val;
			m_val -= val;
			return ret;
		}

		bool compare_exchange_weak(T oldValue, T newValue)
		{
			if (m_val == oldValue)
			{
				m_val = newValue;
				return true;
			}
			return false;
		}

		private:
		T m_val;
	};
#else
	/// wrapper over standard atomic operations
	template<class T>
	class dAtomic : public std::atomic<T>
	{
		public:
		dAtomic<T>()
			: std::atomic<T>(T(0))
		{
		}

		dAtomic<T>(T val)
			: std::atomic<T>(val)
		{
		}

		dAtomic<T>(const dAtomic<T>& copy)
			: std::atomic<T>(copy)
		{
		}

		T operator=(T value)
		{
			return std::atomic<T>::operator=(value);
		}
	};
#endif

/// Simple spin lock for synchronizing threads for very short period of time.
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
		dInt32 exp = 1;
		for (dUnsigned32 test = 0; !m_lock.compare_exchange_weak(test, 1); test = 0)
		{
			Delay(exp);
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
	D_CORE_API void Delay(dInt32& exp);

	dAtomic<dUnsigned32> m_lock;
	#endif
};

/// Simple scope based spin lock.
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


#endif

