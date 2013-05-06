/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __DGTYPES_H__
#define __DGTYPES_H__



#if !(defined (__USE_DOUBLE_PRECISION__) || defined (__ppc__) || defined (ANDROID) || defined (IOS) || defined (__APPLE__))
	#define DG_BUILD_SIMD_CODE
#endif


#ifdef _MSC_VER 
	#ifdef _M_X64
		#ifndef _WIN_64_VER
			#define _WIN_64_VER
		#endif
	#else 		
		#ifndef _WIN_32_VER
			#define _WIN_32_VER
		#endif
	#endif
#endif


#ifdef _MSC_VER 
	#if !(defined (_WIN_32_VER) || defined (_WIN_64_VER))
		#error "most define _WIN_32_VER or _WIN_64_VER for a correct CPU target"
	#endif
#endif

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))

	#pragma warning (disable: 4100) //unreferenced formal parameter
	#pragma warning (disable: 4201) //nonstandard extension used : nameless struct/union
	#pragma warning (disable: 4324) //structure was padded due to __declspec(align())

	#include <io.h> 
	#include <direct.h> 
	#include <malloc.h>
	#include <float.h>
	#include <stdarg.h>
	#include <process.h>
	
	#if (_MSC_VER >= 1600) 
		// VS 2010 or better supports AVX instruction
		#ifdef DG_BUILD_SIMD_CODE
			#define DG_BUILD_SIMD_256_CODE
		#endif
	#endif


	#ifdef _DEBUG
		#pragma warning (disable: 4127)	//conditional expression is constant
	#endif

	#pragma warning (push, 3) 
		#include <windows.h>
		#include <crtdbg.h>
		#include <tlhelp32.h>
	#pragma warning (pop) 
#endif

#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>



#if (defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
	#include <io.h> 
	#include <direct.h> 
	#include <malloc.h>
	#include <float.h>
	#include <windows.h>
	#include <process.h>
#endif




#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#include <intrin.h>
#endif



#ifdef __ppc__
	#include <vecLib/veclib.h>
#endif

#ifdef _POSIX_VER
	#include <unistd.h>
	#include <assert.h>
	#ifdef DG_BUILD_SIMD_CODE
		extern "C" 
		{ 
			// for SSE3 and up
			// #include <pmmintrin.h> 
			// #include <immintrin.h>   
			// #include <smmintrin.h> 
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
		#include <pmmintrin.h> 
		#include <emmintrin.h>  //sse3
        #include <mmintrin.h>    
    #endif
#endif

//************************************************************
#if defined (_WIN_32_VER) || defined (_WIN_64_VER)
	#define dgAssert(x) _ASSERTE(x)
#else 
	#define dgAssert(x) assert(x)
#endif


#define	DG_MAX_THREADS_HIVE_COUNT		16

#ifdef _DEBUG
//#define __ENABLE_SANITY_CHECK 
#endif


#ifdef DLL_DECLSPEC
#undef DLL_DECLSPEC
#endif

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#define DG_INLINE	__forceinline 
#else 
	#define DG_INLINE	inline
	//#define DG_INLINE	 __attribute__((always_inline))
#endif


#define DG_VECTOR_SIMD_SIZE		16
#define DG_VECTOR_AVX_SIZE		32

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#define	DG_MSC_VECTOR_ALIGMENT			__declspec(align(DG_VECTOR_SIMD_SIZE))
	#define	DG_MSC_VECTOR_AVX_ALIGMENT		__declspec(align(DG_VECTOR_AVX_SIZE))

	#define	DG_GCC_VECTOR_ALIGMENT	
	#define	DG_GCC_VECTOR_AVX_ALIGMENT	
#else
	#define	DG_MSC_VECTOR_ALIGMENT			
	#define	DG_MSC_VECTOR_AVX_ALIGMENT

	#define	DG_GCC_VECTOR_ALIGMENT			__attribute__ ((aligned (DG_VECTOR_SIMD_SIZE)))
	#define	DG_GCC_VECTOR_AVX_ALIGMENT		__attribute__ ((aligned (DG_VECTOR_AVX_SIZE)))
#endif


#define DG_SIMD_WORD_SIZE		dgInt32 (sizeof (dgSimd) / sizeof (dgFloat32))
//#define DG_SIMD_256_WORD_SIZE	(DG_SIMD_WORD_SIZE * 2)


typedef char dgInt8;
typedef unsigned char dgUnsigned8;

typedef short dgInt16;
typedef unsigned short dgUnsigned16;

typedef int dgInt32;
typedef unsigned dgUnsigned32;
typedef unsigned int dgUnsigned32;

typedef long long dgInt64;
typedef unsigned long long dgUnsigned64;


typedef double dgFloat64;

#ifdef __USE_DOUBLE_PRECISION__
	typedef double dgFloat32;
#else
	typedef float dgFloat32;
#endif


class dgTriplex
{
	public:
	dgFloat32 m_x;
	dgFloat32 m_y;
	dgFloat32 m_z;
};


#define dgPI			 	dgFloat32 (3.14159f)
#define dgPI2			 	dgFloat32 (dgPI * 2.0f)
#define dgEXP			 	dgFloat32 (2.71828f)
#define dgEPSILON	  	 	dgFloat32 (1.0e-5f)
#define dgGRAVITY	  	 	dgFloat32 (9.8f)
#define dgDEG2RAD	  	 	dgFloat32 (dgPI / 180.0f)
#define dgRAD2DEG	  	 	dgFloat32 (180.0f / dgPI)
#define dgKMH2MPSEC		 	dgFloat32 (0.278f)


class dgVector;
class dgBigVector;

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#define dgApi __cdecl 	
	#define dgStdApi __stdcall 	
#else
	#define dgApi 	
	#define dgStdApi
#endif



#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	#define dgCheckFloat(x) (_finite(x) && !_isnan(x))
//	#define dgCheckFloat(x) 1
#else
	#define dgCheckFloat(x) (isfinite(x) && !isnan(x))
//		#define dgCheckFloat(x) 1
#endif



typedef void (dgApi *dgDeserialize) (void* const userData, void* buffer, size_t size);
typedef void (dgApi *dgSerialize) (void* const userData, const void* const buffer, size_t size);
typedef void (dgApi *dgReportProgress) (dgFloat32 progressNormalzedPercent);

DG_INLINE dgInt32 dgExp2 (dgInt32 x)
{
	dgInt32 exp;
	for (exp = -1; x; x >>= 1) {
		exp ++;
	}
	return exp;
}

template <class T> 
DG_INLINE T dgMin(T A, T B)
{
	return (A < B) ? A : B; 
}

template <class T> 
DG_INLINE T dgMax(T A, T B)
{
	return (A > B) ? A : B; 
}

template <class T>
DG_INLINE T dgMin(T A, T B, T C)
{
	return dgMin(dgMin (A, B), C);
}

template <class T>
DG_INLINE T dgMax(T A, T B, T C)
{
	return dgMax(dgMax (A, B), C);
}

template <class T>
DG_INLINE T dgClamp(T val, T min, T max)
{
	return dgMax (min, dgMin (max, val));
}

template <class T> 
DG_INLINE void dgSwap(T& A, T& B)
{
	T tmp (A);
	A = B;
	B = tmp;
}	

template <class T>
DG_INLINE T dgSign(T A)
{
//	T sign (1.0f);
//	if (A < T (0.0f)) {
//		sign = T (-1.0f);
//	}
//	return sign;
	return (A >= T(0)) ? T(1) : T(-1);
}

template <class T> 
dgInt32 dgBinarySearch (T const* array, dgInt32 elements, dgInt32 entry)
{
	dgInt32 index0 = 0;
	dgInt32 index2 = elements - 1;
	dgInt32 entry0 = array[index0].m_Key;
	dgInt32 entry2 = array[index2].m_Key;

	while ((index2 - index0) > 1) {
		dgInt32 index1 = (index0 + index2) >> 1;
		dgInt32 entry1 = array[index1].m_Key;
		if (entry1 == entry) {
			dgAssert (array[index1].m_Key <= entry);
			dgAssert (array[index1 + 1].m_Key >= entry);
			return index1;
		} else if (entry < entry1) {
			index2 = index1;
			entry2 = entry1;
		} else {
			index0 = index1;
			entry0 = entry1;
		}
	}

	if (array[index0].m_Key > index0) {
		index0 --;
	}

	dgAssert (array[index0].m_Key <= entry);
	dgAssert (array[index0 + 1].m_Key >= entry);
	return index0;
}




template <class T> 
void dgRadixSort (T* const array, T* const tmpArray, dgInt32 elements, dgInt32 radixPass, 
				  dgInt32 (*getRadixKey) (const T* const  A, void* const context), void* const context = NULL)
{
	dgInt32 scanCount[256]; 
	dgInt32 histogram[256][4];

	dgAssert (radixPass >= 1);
	dgAssert (radixPass <= 4);
	
	memset (histogram, 0, sizeof (histogram));
	for (dgInt32 i = 0; i < elements; i ++) {
		dgInt32 key = getRadixKey (&array[i], context);
		for (dgInt32 j = 0; j < radixPass; j ++) {
			dgInt32 radix = (key >> (j << 3)) & 0xff;
			histogram[radix][j] = histogram[radix][j] + 1;
		}
	}

	for (dgInt32 radix = 0; radix < radixPass; radix += 2) {
		scanCount[0] = 0;
		for (dgInt32 i = 1; i < 256; i ++) {
			scanCount[i] = scanCount[i - 1] + histogram[i - 1][radix];
		}
		dgInt32 radixShift = radix << 3;
		for (dgInt32 i = 0; i < elements; i ++) {
			dgInt32 key = (getRadixKey (&array[i], context) >> radixShift) & 0xff;
			dgInt32 index = scanCount[key];
			tmpArray[index] = array[i];
			scanCount[key] = index + 1;
		}

		if ((radix + 1) < radixPass) { 
			scanCount[0] = 0;
			for (dgInt32 i = 1; i < 256; i ++) {
				scanCount[i] = scanCount[i - 1] + histogram[i - 1][radix + 1];
			}
			
			dgInt32 radixShift = (radix + 1) << 3;
			for (dgInt32 i = 0; i < elements; i ++) {
				dgInt32 key = (getRadixKey (&array[i], context) >> radixShift) & 0xff;
				dgInt32 index = scanCount[key];
				array[index] = tmpArray[i];
				scanCount[key] = index + 1;
			}
		} else {
			memcpy (array, tmpArray, elements * sizeof (T)); 
		}
	}


#ifdef _DEBUG
	for (dgInt32 i = 0; i < (elements - 1); i ++) {
		dgAssert (getRadixKey (&array[i], context) <= getRadixKey (&array[i + 1], context));
	}
#endif

}


template <class T> 
void dgSort (T* const array, dgInt32 elements, dgInt32 (*compare) (const T* const  A, const T* const B, void* const context), void* const context = NULL)
{
	dgInt32 stride = 8;
	dgInt32 stack[1024][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	dgInt32 stackIndex = 1;
	while (stackIndex) {
		stackIndex --;
		dgInt32 lo = stack[stackIndex][0];
		dgInt32 hi = stack[stackIndex][1];
		if ((hi - lo) > stride) {
			dgInt32 i = lo;
			dgInt32 j = hi;
			T val (array[(lo + hi) >> 1]);
			do {    
				while (compare (&array[i], &val, context) < 0) i ++;
				while (compare (&array[j], &val, context) > 0) j --;

				if (i <= j)	{
					T tmp (array[i]);
					array[i] = array[j]; 
					array[j] = tmp;
					i++; 
					j--;
				}
			} while (i <= j);

			if (i < hi) {
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex ++;
			}
			if (lo < j) {
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex ++;
			}
			dgAssert (stackIndex < dgInt32 (sizeof (stack) / (2 * sizeof (stack[0][0]))));
		}
	}

	stride = stride * 2;
	if (elements < stride) {
		stride = elements;
	}
	for (dgInt32 i = 1; i < stride; i ++) {
		if (compare (&array[0], &array[i], context) > 0) {
			T tmp (array[0]);
			array[0] = array[i];
			array[i] = tmp;
		}
	}

	for (dgInt32 i = 1; i < elements; i ++) {
		dgInt32 j = i;
		T tmp (array[i]);
		//for (; j && (compare (&array[j - 1], &tmp, context) > 0); j --) {
		for (; compare (&array[j - 1], &tmp, context) > 0; j --) {
			dgAssert (j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

#ifdef _DEBUG
	for (dgInt32 i = 0; i < (elements - 1); i ++) {
		dgAssert (compare (&array[i], &array[i + 1], context) <= 0);
	}
#endif
}


template <class T> 
void dgSortIndirect (T** const array, dgInt32 elements, dgInt32 (*compare) (const T* const  A, const T* const B, void* const context), void* const context = NULL)
{
	dgInt32 stride = 8;
	dgInt32 stack[1024][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	dgInt32 stackIndex = 1;
	while (stackIndex) {
		stackIndex --;
		dgInt32 lo = stack[stackIndex][0];
		dgInt32 hi = stack[stackIndex][1];
		if ((hi - lo) > stride) {
			dgInt32 i = lo;
			dgInt32 j = hi;
			T* val (array[(lo + hi) >> 1]);
			do {    
				while (compare (array[i], val, context) < 0) i ++;
				while (compare (array[j], val, context) > 0) j --;

				if (i <= j)	{
					T* tmp (array[i]);
					array[i] = array[j]; 
					array[j] = tmp;
					i++; 
					j--;
				}
			} while (i <= j);

			if (i < hi) {
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex ++;
			}
			if (lo < j) {
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex ++;
			}
			dgAssert (stackIndex < dgInt32 (sizeof (stack) / (2 * sizeof (stack[0][0]))));
		}
	}

	stride = stride * 2;
	if (elements < stride) {
		stride = elements;
	}
	for (dgInt32 i = 1; i < stride; i ++) {
		if (compare (&array[0], &array[i], context) > 0) {
			T tmp (array[0]);
			array[0] = array[i];
			array[i] = tmp;
		}
	}

	for (dgInt32 i = 1; i < elements; i ++) {
		dgInt32 j = i;
		T* tmp (array[i]);
		//for (; j && (compare (array[j - 1], tmp, context) > 0); j --) {
		for (; compare (&array[j - 1], &tmp, context) > 0; j --) {
			dgAssert (j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

#ifdef _DEBUG
	for (dgInt32 i = 0; i < (elements - 1); i ++) {
		dgAssert (compare (array[i], array[i + 1], context) <= 0);
	}
#endif
}



#ifdef __USE_DOUBLE_PRECISION__
	union dgFloatSign
	{
		struct {
			dgInt32 m_dommy;
			dgInt32 m_iVal;
		} m_integer;
		dgFloat64 m_fVal;
	};
#else
	union dgFloatSign
	{
		struct {
			dgInt32 m_iVal;
		} m_integer;
		dgFloat32 m_fVal;
	};
#endif

union dgDoubleInt
{
	struct {
		dgInt32 m_intL;
		dgInt32 m_intH;
	};
	dgInt64 m_int;
	dgFloat64 m_float;
};



void GetMinMax (dgVector &Min, dgVector &Max, const dgFloat32* const vArray, dgInt32 vCount, dgInt32 StrideInBytes);
void GetMinMax (dgBigVector &Min, dgBigVector &Max, const dgFloat64* const vArray, dgInt32 vCount, dgInt32 strideInBytes);

dgInt32 dgVertexListToIndexList (dgFloat32* const vertexList, dgInt32 strideInBytes, dgInt32 floatSizeInBytes,
				       dgInt32 unsignedSizeInBytes, dgInt32 vertexCount, dgInt32* const indexListOut, dgFloat32 tolerance = dgEPSILON);

dgInt32 dgVertexListToIndexList (dgFloat64* const vertexList, dgInt32 strideInBytes, dgInt32 compareCount, dgInt32 vertexCount, dgInt32* const indexListOut, dgFloat64 tolerance = dgEPSILON);



enum dgCpuClass
{
	dgNoSimdPresent = 0,
	dgSimdPresent,
};


#define PointerToInt(x) ((size_t)x)
#define IntToPointer(x) ((void*)(size_t(x)))


#ifndef _MSC_VER 
	#define _stricmp(x,y) strcasecmp(x,y)
#endif


DG_INLINE dgFloat32 dgAbsf(dgFloat32 x)
{
#if 0
	dgDoubleInt val;
	val.m_float = x;
	val.m_intH &= ~(dgUnsigned64 (1)<<31);
	dgAssert (val.m_float == fabs (x));
	return dgFloat32 (val.m_float);
#else
	// according to Intel this is better because is doe not read after write
	return (x >= dgFloat32 (0.0f)) ? x : -x;
#endif
}


DG_INLINE dgInt32 dgFastInt (dgFloat64 x)
{
	dgInt32 i = dgInt32 (x);
	if (dgFloat64 (i) > x) {
		i --;
	}
	return i;
}

DG_INLINE dgInt32 dgFastInt (dgFloat32 x)
{
#if 0
	volatile dgDoubleInt val;
	volatile dgDoubleInt round;
	const dgFloat64 conversionMagicConst = ((dgFloat64 (dgInt64(1)<<52)) * dgFloat64 (1.5f));
	val.m_float = dgFloat64 (x) + conversionMagicConst; 
	round.m_float = x - dgFloat64 (val.m_intL);
	dgInt32 ret = val.m_intL + (round.m_intH >> 31);
	dgAssert (ret == dgInt32 (floor (x)));
	return ret;

#else
	dgInt32 i = dgInt32 (x);
	if (dgFloat32 (i) > x) {
		i --;
	}
	return i;
#endif
}

DG_INLINE dgFloat32 dgFloor(dgFloat32 x)
{
#ifdef _MSC_VER
	dgFloat32 ret = dgFloat32 (dgFastInt (x));
	dgAssert (ret == floor (x));
	return  ret;
#else 
	return floor (x);
#endif
}

DG_INLINE dgFloat32 dgCeil(dgFloat32 x)
{
#ifdef _MSC_VER
	dgFloat32 ret = dgFloor(x);
	if (ret < x) {
		ret += dgFloat32 (1.0f);
	}
	dgAssert (ret == ceil (x));
	return  ret;
#else 
	return ceil (x);
#endif
}

#define dgSqrt(x)			dgFloat32 (sqrt(x))	
#define dgRsqrt(x)			(dgFloat32 (1.0f) / dgSqrt(x))		
#define dgSin(x)			dgFloat32 (sin(x))
#define dgCos(x)			dgFloat32 (cos(x))
#define dgAsin(x)			dgFloat32 (asin(x))
#define dgAcos(x)			dgFloat32 (acos(x))
#define dgAtan2(x,y)		dgFloat32 (atan2(x,y))
#define dgLog(x)			dgFloat32 (log(x))
#define dgPow(x,y)			dgFloat32 (pow(x,y))
#define dgFmod(x,y)			dgFloat32 (fmod(x,y))
#define dgClearFP()			_clearfp() 
#define dgControlFP(x,y)	_controlfp(x,y)


void dgSerializeMarker(dgSerialize serializeCallback, void* const userData);
void dgDeserializeMarker(dgDeserialize serializeCallback, void* const userData);


typedef dgUnsigned32 (dgApi *OnGetPerformanceCountCallback) ();

dgCpuClass dgApi dgGetCpuType ();


class dgFloatExceptions
{
	public:
	//#define DG_FLOAT_EXECTIONS_MASK (_EM_INVALID | _EM_DENORMAL | _EM_ZERODIVIDE | _EM_OVERFLOW | _EM_UNDERFLOW| _EM_INEXACT)
	#define DG_FLOAT_EXECTIONS_MASK (_EM_INVALID | _EM_DENORMAL | _EM_ZERODIVIDE)

	dgFloatExceptions()
		:m_mask (0)
	{
//		#if (defined (_MSC_VER) && defined (_DEBUG))
//			dgClearFP();
//			m_mask = dgControlFP(0, 0);
//			dgControlFP(m_mask & dgUnsigned32 (~DG_FLOAT_EXECTIONS_MASK), _MCW_EM);
//		#endif
	}

	~dgFloatExceptions()
	{
//		#if (defined (_MSC_VER) && defined (_DEBUG))
//			dgClearFP();
//			dgControlFP(m_mask | dgUnsigned32(DG_FLOAT_EXECTIONS_MASK), _MCW_EM);
//		#endif
	}

	dgInt32 m_mask; 
};


class dgSetPrecisionDouble 
{
	public:
	dgSetPrecisionDouble()
	{
		#if (defined (_MSC_VER) && defined (_WIN_32_VER))
			dgClearFP();
			m_mask = dgControlFP(0, 0);
			dgControlFP (_PC_53, _MCW_PC);
		#endif
	}

	~dgSetPrecisionDouble()
	{
		#if (defined (_MSC_VER) && defined (_WIN_32_VER))
			dgClearFP();
			dgControlFP (m_mask, _MCW_PC);
		#endif
	}
	dgInt32 m_mask; 
};




DG_INLINE dgInt32 dgAtomicExchangeAndAdd (dgInt32* const addend, dgInt32 amount)
{
	// it is a pity that pthread does not supports cross platform atomics, it would be nice if it did
	#if (defined (_WIN_32_VER) || defined (_WIN_64_VER) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
		dgInt32 ret = _InterlockedExchangeAdd((long*) addend, long (amount));
		return ret;
	#endif

	#if (defined (_POSIX_VER) || defined (_MACOSX_VER))
		dgInt32 ret = __sync_fetch_and_add ((int32_t*)addend, amount );
		return ret;
	#endif
}

DG_INLINE dgInt32 dgInterlockedExchange(dgInt32* const ptr, dgInt32 value)
{
	// it is a pity that pthread does not supports cross platform atomics, it would be nice if it did
	#if (defined (_WIN_32_VER) || defined (_WIN_64_VER) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
		dgInt32 ret = _InterlockedExchange((long*) ptr, value);
		return ret;
	#endif

	#if (defined (_POSIX_VER) || defined (_MACOSX_VER))
		//__sync_synchronize();
		dgInt32 ret = __sync_lock_test_and_set((int32_t*)ptr, value);
		return ret;
	#endif
}

DG_INLINE void dgThreadYield()
{
	sched_yield();
}


#endif

