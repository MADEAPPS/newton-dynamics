/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __dMathDefined__
#define __dMathDefined__

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef _MSC_VER
	#include <windows.h>
	#ifdef _DEBUG
		#include <stdarg.h>
		inline void dExpandTraceMessage (const char* const fmt, ...)
		{
			va_list v_args;
			char text[4096];

			text[0] = 0;
			va_start (v_args, fmt);     
			vsprintf(text, fmt, v_args);
			va_end (v_args);            

			OutputDebugStringA (text);
		}

		#define dTrace(x)										\
		{														\
			dExpandTraceMessage x;								\
		}																	
	#else
		#define dTrace(x)
	#endif
#else
	#define dTrace(x)
#endif

#if ( defined (_MSC_VER) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER) )
	#include <crtdbg.h>
	#define dAssert(x) _ASSERTE(x)
#else 
	#define dAssert(x) assert(x)
#endif



#ifndef dFloat
	#ifdef _NEWTON_USE_DOUBLE
		typedef double dFloat;
	#else 
		typedef float dFloat;
	#endif
#endif 

#ifndef dFloat64
	typedef double dFloat64;
#endif 


// transcendental functions
#define	dAbs(x)		dFloat (fabs (dFloat(x))) 
#define	dSqrt(x)	dFloat (sqrt (dFloat(x))) 
#define	dFloor(x)	dFloat (floor (dFloat(x))) 
#define	dCiel(x)	dFloat (ceil (dFloat(x))) 
#define	dMod(x,y)	dFloat (fmod (dFloat(x), dFloat(y))) 
#define	dPow(x,y)	dFloat (pow (dFloat(x), dFloat(y))) 

#define dSin(x)		dFloat (sin (dFloat(x)))
#define dCos(x)		dFloat (cos (dFloat(x)))
#define dTan(x)		dFloat (tan (dFloat(x)))
#define dAsin(x)	dFloat (asin (dFloat(x)))
#define dAcos(x)	dFloat (acos (dFloat(x)))
#define	dAtan2(x,y) dFloat (atan2 (dFloat(x), dFloat(y)))


#define	D_MSC_VECTOR_ALIGMENT

enum dEulerAngleOrder
{
	m_pitchYawRoll = (0 << 8) + (1 << 4) + (2 << 0),
	m_rollYawpitch = (2 << 8) + (1 << 4) + (0 << 0),
};



template <class T>
T dSign (T A)
{
	return (A >= T(0)) ? T(1) : T(-1);
}

template <class T> 
void dSwap(T& A, T& B)
{
	T tmp (A);
	A = B;
	B = tmp;
}	

template <class T>
T dMax(T A, T B)
{
	return (A > B) ? A : B; 
}

template <class T>
T dMin(T A, T B)
{
	return (A < B) ? A : B; 
}

template <class T>
T dClamp(T val, T min, T max)
{
	return dMax (min, dMin (max, val));
}

template <class T> 
void dSort (T* const array, int elements, int (*compare) (const T* const  A, const T* const B, void* const context), void* const context = NULL)
{
	int stride = 8;
	int stack[1024][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	int stackIndex = 1;
	while (stackIndex) {
		stackIndex --;
		int lo = stack[stackIndex][0];
		int hi = stack[stackIndex][1];
		if ((hi - lo) > stride) {
			int i = lo;
			int j = hi;
			T val (array[(lo + hi) >> 1]);
			do {    
				while (compare (&array[i], &val, context) < 0) i ++;
				while (compare (&array[j], &val, context) > 0) j --;

				if (i <= j)	{
					dSwap(array[i], array[j]);
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
			dAssert (stackIndex < int (sizeof (stack) / (2 * sizeof (stack[0][0]))));
		}
	}

	stride = stride * 2;
	if (elements < stride) {
		stride = elements;
	}
	for (int i = 1; i < stride; i ++) {
		if (compare (&array[0], &array[i], context) > 0) {
			dSwap(array[0], array[i]);
		}
	}

	for (int i = 1; i < elements; i ++) {
		int j = i;
		T tmp (array[i]);
		for (; compare (&array[j - 1], &tmp, context) > 0; j --) {
			dAssert (j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

#ifdef _DEBUG
	for (int i = 0; i < (elements - 1); i ++) {
		dAssert (compare (&array[i], &array[i + 1], context) <= 0);
	}
#endif
}

#endif

