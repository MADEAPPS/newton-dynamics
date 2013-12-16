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

#define dSin(x)		dFloat (sin (dFloat(x)))
#define dCos(x)		dFloat (cos (dFloat(x)))
#define dAsin(x)	dFloat (asin (dFloat(x)))
#define dAcos(x)	dFloat (acos (dFloat(x)))
#define	dAtan2(x,y) dFloat (atan2 (dFloat(x), dFloat(y)))


#define	D_MSC_VECTOR_ALIGMENT

enum dEulerAngleOrder
{
	m_pitchYawRoll = (0 << 8) + (1 << 4) + (2 << 0),
	m_rollYawpitch = (2 << 8) + (1 << 4) + (0 << 0),
};




#if ( defined (_MSC_VER) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER) )
	#define dAssert(x) _ASSERTE(x)
#else 
	#define dAssert(x) assert(x)
#endif




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




#ifdef _MSC_VER
	#ifdef _DEBUG
		#include <windows.h>
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


#endif

