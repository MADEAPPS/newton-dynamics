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

#ifndef __ND_GENERAL_VECTOR_H__
#define __ND_GENERAL_VECTOR_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"

template <class T>
T ndSQRH(const T num, const T den)
{
	T r(num / den);
	return T(sqrt(T(1.0f) + r * r));
}

template <class T>
T ndPythag(const T a, const T b)
{
	T absa(ndAbs(a));
	T absb(ndAbs(b));
	return (absa > absb) ? (absa * ndSQRH(absb, absa)) : ((absb == T(0.0f) ? T(0.0f) : (absb * ndSQRH(absa, absb))));
}

template <class T>
T ndSign(const T a, const T b)
{
	return (b >= T(0.0f)) ? (a >= T(0.0f) ? a : -a) : (a >= T(0.0f) ? -a : a);
}

// return dot product
template<class T>
T ndDotProduct(ndInt32 size, const T* const A, const T* const B)
{
	T val(0.0f);
	for (ndInt32 i = 0; i < size; ++i) 
	{
		val = val + A[i] * B[i];
		ndAssert(ndCheckFloat(val));
	}
	return val;
}

template<class T>
void ndScaleSet(ndInt32 size, T* const X, const T* const A, T scale)
{
	for (ndInt32 i = 0; i < size; ++i)
	{
		X[i] = A[i] * scale;
		ndAssert(ndCheckFloat(X[i]));
	}
}

template<class T>
void ndAdd(ndInt32 size, T* const X, const T* const A, const T* const B)
{
	for (ndInt32 i = 0; i < size; ++i) 
	{
		X[i] = A[i] + B[i];
		ndAssert(ndCheckFloat(X[i]));
	}
}

template<class T>
void ndSub(ndInt32 size, T* const X, const T* const A, const T* const B)
{
	for (ndInt32 i = 0; i < size; ++i) 
	{
		X[i] = A[i] - B[i];
		ndAssert(ndCheckFloat(X[i]));
	}
}

template<class T>
void ndMul(ndInt32 size, T* const X, const T* const A, const T* const B)
{
	for (ndInt32 i = 0; i < size; ++i)
	{
		X[i] = A[i] * B[i];
		ndAssert(ndCheckFloat(X[i]));
	}
}

template<class T>
void ndScaleAdd(ndInt32 size, T* const X, const T* const A, const T* const B, T C)
{
	for (ndInt32 i = 0; i < size; ++i) 
	{
		X[i] = A[i] + B[i] * C;
		ndAssert(ndCheckFloat(X[i]));
	}
}

template<class T>
void ndMulAdd(ndInt32 size, T* const X, const T* const A, const T* const B, const T* const C)
{
	for (ndInt32 i = 0; i < size; ++i)
	{
		X[i] = A[i] + B[i] * C[i];
		ndAssert(ndCheckFloat(X[i]));
	}
}

template<class T>
void ndMulSub(ndInt32 size, T* const X, const T* const A, const T* const B, const T* const C)
{
	for (ndInt32 i = 0; i < size; ++i)
	{
		X[i] = A[i] - B[i] * C[i];
		ndAssert(ndCheckFloat(X[i]));
	}
}

#endif


