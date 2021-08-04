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

#ifndef __D_UTILS_H__
#define __D_UTILS_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dStack.h"
#include "dMemory.h"

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
	// deal with too small and de normal values.
	if ((dAbs(A) < tol) && (dAbs(B) < tol)) 
	{
		return true;
	}

	dInt32 exp0;
	dFloat64 mantissa0 = frexp(dFloat64 (A), &exp0);
	
	dInt32 exp1;
	dFloat64 mantissa1 = frexp(dFloat64(B), &exp1);
	if (dAbs(exp0 - exp1) > 1)
	{
		return false;
	}
	if (exp0 != exp1)
	{
		if (exp0 > exp1)
		{
			mantissa0 *= dFloat32(2.0f);
		}
		else
		{
			mantissa1 *= dFloat32(2.0f);
		}
	}
	return dAbs(mantissa0 - mantissa1) < tol;
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

/// Returns the time in micro seconds since application started 
D_CORE_API dUnsigned64 dGetTimeInMicrosenconds();

/// Round a 64 bit float to a 32 bit float by truncating the mantissa a 24 bit 
/// \param dFloat64 val: 64 bit float 
/// \return a 64 bit double precision with a 32 bit mantissa
D_CORE_API dFloat64 dRoundToFloat(dFloat64 val);

D_CORE_API dInt32 dVertexListToIndexList(dFloat64* const vertexList, dInt32 strideInBytes, dInt32 compareCount, dInt32 vertexCount, dInt32* const indexListOut, dFloat64 tolerance = dEpsilon);

template <class T>
dInt32 dVertexListToIndexList(T* const vertexList, dInt32 strideInBytes, dInt32 compareCount, dInt32 vertexCount, dInt32* const indexListOut, T tolerance = dEpsilon)
{
	dInt32 stride = dInt32(strideInBytes / sizeof(T));
	dStack<dFloat64> pool(vertexCount * stride);

	dFloat64* const data = &pool[0];
	for (dInt32 i = 0; i < vertexCount; i++)
	{
		dFloat64* const dst = &data[i * stride];
		const T* const src = &vertexList[i * stride];
		for (dInt32 j = 0; j < stride; j++)
		{
			dst[j] = src[j];
		}
	}

	dInt32 count = dVertexListToIndexList(data, dInt32(stride * sizeof(dFloat64)), compareCount, vertexCount, indexListOut, dFloat64(tolerance));
	for (dInt32 i = 0; i < count; i++)
	{
		const dFloat64* const src = &data[i * stride];
		T* const dst = &vertexList[i * stride];
		for (dInt32 j = 0; j < stride; j++)
		{
			dst[j] = T(src[j]);
		}
	}

	return count;
}

/// Set cpu floating point exceptions, the original exception state is restored when the destructor is called.
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
	#if (defined(WIN32) || defined(_WIN32))
		dUnsigned32 m_mask;
	#endif
};

/// Set cpu floating point precision mode, the original mode is restored when the destructor is called.
class dSetPrecisionDouble 
{
	public:
	dSetPrecisionDouble();
	~dSetPrecisionDouble();
	#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	dInt32 m_mask; 
	#endif
};

#endif

