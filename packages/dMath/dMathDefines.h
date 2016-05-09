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


#define D_LCP_MAX_VALUE (1.0e12f)

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

#define dAlloca(type, size) (type*) alloca ((size) * sizeof (type))


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
T DotProduct(int size, const T* const A, const T* const B)
{
	T val(0.0f);
	for (int i = 0; i < size; i++) {
		val = val + A[i] * B[i];
	}
	return val;
}


template<class T>
void MatrixTimeVector(int size, const T* const matrix, const T* const v, T* const out)
{
	int stride = 0;
	for (int i = 0; i < size; i++) {
		out[i] = DotProduct(size, &matrix[stride], v);
		stride += size;
	}
}


template<class T>
bool CholeskyFactorizationAddRow(int size, int n, T* const matrix)
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
			if (diag < T(0.0f)) {
				dAssert(0);
				return false;
			}
			rowN[n] = T(sqrt(diag));
		} else {
			rowN[j] = (T(1.0f) / rowJ[j] * (rowN[j] - s));
		}

		stride += size;
	}

	return true;
}


template<class T>
void CholeskyRestore(int size, T* const matrix, const T* const diagonal, int n, int subSize)
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
void CholeskySolve(int size, const T* const matrix, const T* const b, T* const x, int n)
{
	int stride = 0;
	for (int i = 0; i < n; i++) {
		T acc(0.0f);
		const T* const row = &matrix[stride];
		for (int j = 0; j < i; j++) {
			acc = acc + row[j] * x[j];
		}
		x[i] = (b[i] - acc) / row[i];
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
void CalculateDelta_r(int size, int n, const T* const matrix, const T* const m_delta_x, T* const m_delta_r)
{
	int stride = n * size;
	for (int i = 0; i < n; i ++) {
		m_delta_r[i] = T(0.0f);
	}
	for (int i = n; i < size; i++) {
		m_delta_r[i] = DotProduct(size, &matrix[stride], m_delta_x);
		stride += size;
	}
}

template<class T>
void CalculateDelta_x(int size, T dir, int n, const T* const matrix, T* const m_delta_x, T* const m_tmp)
{
	const T* const row = &matrix[size * n];
	for (int i = 0; i < n; i++) {
		m_tmp[i] = -row[i] * dir;
	}
	CholeskySolve(size, matrix, m_tmp, m_delta_x, n);
	m_delta_x[n] = dir;
	for (int i = n + 1; i < size; i++) {
		m_delta_x[i] = T(0.0f);
	}
}


template<class T>
void PermuteRows(int size, int i, int j, T* const matrix, T* const m_x, T* const m_r, T* const m_low, T* const m_high, T* const m_diagonal, short* const m_permute)
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

		dSwap(m_x[i], m_x[j]);
		dSwap(m_r[i], m_r[j]);
		dSwap(m_low[i], m_low[j]);
		dSwap(m_high[i], m_high[j]);
		dSwap(m_diagonal[i], m_diagonal[j]);
		dSwap(m_permute[i], m_permute[j]);
	}
}


template <class T> 
bool SolveDantzigLCP(int size, T* const matrix, T* const x, T* const b, T* const low, T* const high)
{
	T* const m_x = dAlloca(T, size);
	T* const m_r = dAlloca(T, size);
	T* const m_delta_x = dAlloca(T, size);
	T* const m_delta_r = dAlloca(T, size);
	T* const diagonal = dAlloca(T, size);
	T* const m_tmp = dAlloca(T, size);
	short* const permute = dAlloca(short, size);

//static int xxx;
//xxx ++;

	int stride = 0;
	for (int i = 0; i < size; i++) {
		m_x[i] = dClamp(x[i], low[i], high[i]);
		permute[i] = short(i);
		diagonal[i] = matrix[stride + i];
		stride += size;
	}

	MatrixTimeVector(size, matrix, m_x, m_r);
	for (int i = 0; i < size; i++) {
		m_r[i] -= b[i];
	}


	int index = 0;
	int count = size;

	for (int i = 0; i < count; i++) {
		if ((low[i] <= T(-D_LCP_MAX_VALUE)) && (high[i] >= T(D_LCP_MAX_VALUE))) {
			CholeskyFactorizationAddRow(size, index, matrix);
			index++;
		} else {
			PermuteRows(size, i, count - 1, matrix, m_x, m_r, low, high, diagonal, permute);
			i--;
			count--;
		}
	}

	if (index > 0) {
		CholeskySolve(size, matrix, m_r, m_delta_x, index);
		for (int i = 0; i < index; i++) {
			m_x[i] -= m_delta_x[i];
			m_r[i] = T(0.0f);
		}
		for (int i = index; i < size; i++) {
			m_delta_x[i] = T(0.0f);
		}

		CalculateDelta_r(size, index, matrix, m_delta_x, m_delta_r);
		for (int i = index; i < size; i++) {
			m_r[i] -= m_delta_r[i];
		}
	}
	count = size - index;

	const int start = index;
	int clampedIndex = size;
	while (count) {
		bool loop = true;
		bool calculateDelta_x = true;
		T dir (0.0f);

		while (loop) {
			loop = false;
			T clamp_x(0.0f);
			int swapIndex = -1;

			if (T(fabs(m_r[index]) > T(1.0e-12f))) {

				if (calculateDelta_x) {
					dir = (m_r[index] <= T(0.0f)) ? T(1.0f) : T(-1.0f);
					CalculateDelta_x(size, dir, index, matrix, m_delta_x, m_tmp);
				}

				calculateDelta_x = true;
				CalculateDelta_r(size, index, matrix, m_delta_x, m_delta_r);
				dAssert(m_delta_r[index] != T(0.0f));
				dAssert(T(fabs(m_delta_x[index])) == T(1.0f));

				T s = -m_r[index] / m_delta_r[index];
				dAssert(s >= T(0.0f));

				for (int i = start; i <= index; i++) {
					T x1 = m_x[i] + s * m_delta_x[i];
					if (x1 > high[i]) {
						swapIndex = i;
						clamp_x = high[i];
						s = (high[i] - m_x[i]) / m_delta_x[i];
					} else if (x1 < low[i]) {
						swapIndex = i;
						clamp_x = low[i];
						s = (low[i] - m_x[i]) / m_delta_x[i];
					}
				}
				dAssert(s >= T(0.0f));
				//dgAssert(s <= -m_r[index] / m_delta_r[index]);

				for (int i = clampedIndex; (i < size) && (s > T(1.0e-12f)); i++) {
					T r1 = m_r[i] + s * m_delta_r[i];
					if ((r1 * m_r[i]) < T(0.0f)) {
						dAssert(T(fabs(m_delta_r[i]) > T(0.0f)));
						T s1 = -m_r[i] / m_delta_r[i];
						dAssert(s1 >= T(0.0f));
						if (s1 < s) {
							s = s1;
							swapIndex = i;
						}
					}
				}

				if (s > T(1.0e-12f)) {
					for (int i = 0; i < size; i++) {
						dAssert((m_x[i] + T(1.0e-4f)) >= low[i]);
						dAssert((m_x[i] - T(1.0e-4f)) <= high[i]);
						m_x[i] += s * m_delta_x[i];
						m_r[i] += s * m_delta_r[i];
						dAssert((m_x[i] + T(1.0e-4f)) >= low[i]);
						dAssert((m_x[i] - T(1.0e-4f)) <= high[i]);
					}
				}
			}

			if (swapIndex == -1) {
				m_r[index] = T(0.0f);
				m_delta_r[index] = T(0.0f);
				if (!CholeskyFactorizationAddRow(size, index, matrix)) {
					return false;
				}
				index++;
				count--;
				loop = false;
			} else if (swapIndex == index) {
				count--;
				clampedIndex--;
				m_x[index] = clamp_x;
				m_delta_x[index] = T(0.0f);
				PermuteRows(size, index, clampedIndex, matrix, m_x, m_r, low, high, diagonal, permute);
				loop = count ? true : false;
			} else if (swapIndex > index) {
				loop = true;
				m_r[swapIndex] = T(0.0f);
				dAssert(swapIndex < size);
				dAssert(clampedIndex <= size);
				if (swapIndex < clampedIndex) {
					count--;
					clampedIndex--;
					PermuteRows(size, clampedIndex, swapIndex, matrix, m_x, m_r, low, high, diagonal, permute);
					dAssert(clampedIndex >= index);
				} else {
					count++;
					dAssert(clampedIndex < size);
					PermuteRows(size, clampedIndex, swapIndex, matrix, m_x, m_r, low, high, diagonal, permute);
					clampedIndex++;
					dAssert(clampedIndex <= size);
					dAssert(clampedIndex >= index);
				}
				calculateDelta_x = false;

			} else {
				m_x[swapIndex] = clamp_x;

				CholeskyRestore(size, matrix, diagonal, swapIndex, index);

				dAssert(index > 0);
				PermuteRows(size, swapIndex, index - 1, matrix, m_x, m_r, low, high, diagonal, permute);
				PermuteRows(size, index - 1, index, matrix, m_x, m_r, low, high, diagonal, permute);
				clampedIndex--;
				PermuteRows(size, clampedIndex, index, matrix, m_x, m_r, low, high, diagonal, permute);

				index--;
				for (int i = swapIndex; i < index; i++) {
					CholeskyFactorizationAddRow(size, i, matrix);
				}
				loop = true;
			}
		}
	}

	for (int i = 0; i < size; i++) {
		int j = permute[i];
		x[j] = m_x[i];
		b[j] = m_r[i];
	}

	return true;
}

#endif

