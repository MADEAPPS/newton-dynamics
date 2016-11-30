/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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

#define dAlloca(type, size) (type*) alloca ((size) * sizeof (type))

template <class T>
T dAbs(T A)
{
	// according to Intel this is better because is does not read after write
	return (A >= T(0.0f)) ? A : -A;
}

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


// return dot product
template<class T>
T dDotProduct(int size, const T* const A, const T* const B)
{
	T val(0.0f);
	for (int i = 0; i < size; i++) {
		val = val + A[i] * B[i];
	}
	return val;
}


template<class T>
void dMatrixTimeVector(int size, const T* const matrix, const T* const v, T* const out)
{
	int stride = 0;
	for (int i = 0; i < size; i++) {
		out[i] = dDotProduct(size, &matrix[stride], v);
		stride += size;
	}
}


template<class T>
bool dCholeskyFactorizationAddRow(int size, int n, T* const matrix)
{
	T* const rowN = &matrix[size * n];

	int stride = 0;
	for (int j = 0; j <= n; j++) {
		T s(0.0f);
		T* const rowJ = &matrix[stride];
		for (int k = 0; k < j; k++) {
			s += rowN[k] * rowJ[k];
		}

		if (n == j) {
			T diag = rowN[n] - s;
			if (diag <= T(1.0e-10f)) {
				dAssert(0);
				return false;
			}
			rowN[n] = T(sqrt(diag));
		}
		else {
			rowN[j] = (T(1.0f) / rowJ[j] * (rowN[j] - s));
		}

		stride += size;
	}

	return true;
}

template<class T>
void dCholeskyFactorization(int size, T* const matrix)
{
	for (int i = 0; i < size; i++) {
		dCholeskyFactorizationAddRow(size, i, matrix);
	}
}


template<class T>
void dCholeskyRestore(int size, T* const matrix, const T* const diagonal, int n, int subSize)
{
	int stride = n * size;
	for (int i = n; i < subSize; i++) {
		T* const row = &matrix[stride];
		row[i] = diagonal[i];
		for (int j = 0; j < i; j++) {
			row[j] = matrix[size * j + i];
		}
		stride += size;
	}
}


template<class T>
void dCholeskySolve(int size, const T* const matrix, T* const x, int n)
{
	int stride = 0;
	for (int i = 0; i < n; i++) {
		T acc(0.0f);
		const T* const row = &matrix[stride];
		for (int j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (x[i] - acc) / row[i];
		stride += size;
	}

	for (int i = n - 1; i >= 0; i--) {
		T acc = 0.0f;
		for (int j = i + 1; j < n; j++) {
			acc = acc + matrix[size * j + i] * x[j];
		}
		x[i] = (x[i] - acc) / matrix[size * i + i];
	}
}

// calculate delta_r = A * delta_x
template<class T>
void dCalculateDelta_r(int size, int n, const T* const matrix, const T* const delta_x, T* const delta_r)
{
	int stride = n * size;
	for (int i = n; i < size; i++) {
		delta_r[i] = dDotProduct(size, &matrix[stride], delta_x);
		stride += size;
	}
}

template<class T>
void dCalculateDelta_x(int size, T dir, int n, const T* const matrix, T* const delta_x)
{
	const T* const row = &matrix[size * n];
	for (int i = 0; i < n; i++) {
		delta_x[i] = -row[i] * dir;
	}
	dCholeskySolve(size, matrix, delta_x, n);
	delta_x[n] = dir;
}


template<class T>
void dPermuteRows(int size, int i, int j, T* const matrix, T* const x, T* const r, T* const low, T* const high, T* const diagonal, short* const permute)
{
	if (i != j) {
		T* const A = &matrix[size * i];
		T* const B = &matrix[size * j];
		for (int k = 0; k < size; k++) {
			dSwap(A[k], B[k]);
		}

		int stride = 0;
		for (int k = 0; k < size; k++) {
			dSwap(matrix[stride + i], matrix[stride + j]);
			stride += size;
		}

		dSwap(x[i], x[j]);
		dSwap(r[i], r[j]);
		dSwap(low[i], low[j]);
		dSwap(high[i], high[j]);
		dSwap(diagonal[i], diagonal[j]);
		dSwap(permute[i], permute[j]);
	}
}


#define D_LCP_MAX_VALUE (1.0e12f)
// solve a general Linear complementary program (LCP)
// A * x = b + r
// subjected to constraints
// x(i) = low(i),  if r(i) >= 0  
// x(i) = high(i), if r(i) <= 0  
// low(i) <= x(i) <= high(i),  if r(i) == 0  
//
// return true is the system has a solution.
// in return 
// x is the solution,
// r is return in vector b
// note: although the system is called LCP, the solve is far more general than a strict LCP
// to solve a strict LCP set the following
// low(i) = 0
// high(i) = infinity.
// this the same as enforcing the constrain: x(i) * r(i) = 0
template <class T>
bool dSolveDantzigLCP(int size, T* const matrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const x0 = dAlloca(T, size);
	T* const r0 = dAlloca(T, size);
	T* const delta_r = dAlloca(T, size);
	short* const permute = dAlloca(short, size);

	T* const delta_x = b;
	T* const diagonal = x;

	int stride = 0;
	for (int i = 0; i < size; i++) {
		x0[i] = dClamp(x[i], low[i], high[i]);
		if ((low[i] > T(-D_LCP_MAX_VALUE)) || (high[i] < T(D_LCP_MAX_VALUE))) {
			dAssert (0);
			low[i] -= x0[i];
			high[i] -= x0[i];
		}
		permute[i] = short(i);
		diagonal[i] = matrix[stride + i];
		stride += size;
	}

	dMatrixTimeVector(size, matrix, x0, r0);
	for (int i = 0; i < size; i++) {
		r0[i] -= b[i];
	}

	int index = 0;
	int count = size;

	for (int i = 0; i < count; i++) {
		if ((low[i] <= T(-D_LCP_MAX_VALUE)) && (high[i] >= T(D_LCP_MAX_VALUE))) {
			if (!dCholeskyFactorizationAddRow(size, index, matrix)) {
				return false;
			}
			index++;
		} else {
			dPermuteRows(size, i, count - 1, matrix, x0, r0, low, high, diagonal, permute);
			i--;
			count--;
		}
	}

	if (index > 0) {
		dCholeskySolve(size, matrix, r0, index);
		for (int i = 0; i < index; i++) {
			x0[i] -= r0[i];
			r0[i] = T(0.0f);
		}
		dCalculateDelta_r(size, index, matrix, x0, delta_r);
		for (int i = index; i < size; i++) {
			r0[i] += delta_r[i];
		}
	}
	count = size - index;
	for (int i = 0; i < size; i++) {
		delta_x[i] = T(0.0f);
		delta_r[i] = T(0.0f);
	}

	const int start = index;
	int clampedIndex = size;
	while (count) {
		bool loop = true;
		bool calculateDelta_x = true;

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			int swapIndex = -1;

			if (dAbs(r0[index]) > T(1.0e-12f)) {

				if (calculateDelta_x) {
					//T dir = dSign(r0[index]);
					T dir = 1.0f;
					dCalculateDelta_x(size, dir, index, matrix, delta_x);
				}

				calculateDelta_x = true;
				dCalculateDelta_r(size, index, matrix, delta_x, delta_r);
				dAssert(delta_r[index] != T(0.0f));
				dAssert(dAbs(delta_x[index]) == T(1.0f));

				T s = - r0[index] / delta_r[index];
				dAssert(dAbs (s) >= T(0.0f));

				for (int i = start; i <= index; i++) {
					T x1 = x0[i] + s * delta_x[i];
					if (x1 > high[i]) {
						swapIndex = i;
						clamp_x = high[i];
						s = (high[i] - x0[i]) / delta_x[i];
					} else if (x1 < low[i]) {
						swapIndex = i;
						clamp_x = low[i];
						s = (low[i] - x0[i]) / delta_x[i];
					}
				}
				dAssert(dAbs (s) >= T(0.0f));

				for (int i = clampedIndex; (i < size) && (dAbs(s) > T(1.0e-12f)); i++) {
					T r1 = r0[i] + s * delta_r[i];
					if ((r1 * r0[i]) < T(0.0f)) {
						dAssert(dAbs(delta_r[i]) > T(0.0f));
						T s1 = - r0[i] / delta_r[i];
						dAssert(dAbs(s1) >= T(0.0f));
						dAssert(dAbs(s1) <= dAbs(s));
						if (dAbs(s1) < dAbs(s)) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				dAssert(dAbs (s) >= T(0.0f));
				for (int i = 0; i < size; i++) {
					dAssert((x0[i] + dAbs(x0[i]) * T(1.0e-4f)) >= low[i]);
					dAssert((x0[i] - dAbs(x0[i]) * T(1.0e-4f)) <= high[i]);

					x0[i] += s * delta_x[i];
					r0[i] += s * delta_r[i];

					dAssert((x0[i] + dAbs(x0[i]) * T(1.0e-4f)) >= low[i]);
					dAssert((x0[i] - dAbs(x0[i]) * T(1.0e-4f)) <= high[i]);
				}
			}

			if (swapIndex == -1) {
				r0[index] = T(0.0f);
				delta_r[index] = T(0.0f);
				if (!dCholeskyFactorizationAddRow(size, index, matrix)) {
					return false;
				}
				index++;
				count--;
				loop = false;
			} else if (swapIndex == index) {
				count--;
				clampedIndex--;
				x0[index] = clamp_x;
				dPermuteRows(size, index, clampedIndex, matrix, x0, r0, low, high, diagonal, permute);
				loop = count ? true : false;
			} else if (swapIndex > index) {
				loop = true;
				r0[swapIndex] = T(0.0f);
				dAssert(swapIndex < size);
				dAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) {
					count--;
					clampedIndex--;
					dPermuteRows(size, clampedIndex, swapIndex, matrix, x0, r0, low, high, diagonal, permute);
					dAssert(clampedIndex >= index);
				} else {
					count++;
					dAssert(clampedIndex < size);
					dPermuteRows(size, clampedIndex, swapIndex, matrix, x0, r0, low, high, diagonal, permute);
					clampedIndex++;
					dAssert(clampedIndex <= size);
					dAssert(clampedIndex >= index);
				}
				calculateDelta_x = false;
			} else {
				dAssert(index > 0);
				x0[swapIndex] = clamp_x;
				delta_x[index] = T(0.0f);

				dCholeskyRestore(size, matrix, diagonal, swapIndex, index);
				dPermuteRows(size, swapIndex, index - 1, matrix, x0, r0, low, high, diagonal, permute);
				dPermuteRows(size, index - 1, index, matrix, x0, r0, low, high, diagonal, permute);
				dPermuteRows(size, clampedIndex - 1, index, matrix, x0, r0, low, high, diagonal, permute);

				clampedIndex--;
				index--;
				for (int i = swapIndex; i < index; i++) {
					dCholeskyFactorizationAddRow(size, i, matrix);
				}
				loop = true;
			}
		}
	}

	for (int i = 0; i < size; i++) {
		int j = permute[i];
		x[j] = x0[i];
		b[j] = r0[i];
	}

	return true;
}



#endif

