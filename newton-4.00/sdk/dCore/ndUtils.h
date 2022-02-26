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

#ifndef __ND_UTILS_H__
#define __ND_UTILS_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndStack.h"
#include "ndMemory.h"

// assume this function returns memory aligned to 16 bytes
#define dAlloca(type, count) (type*) alloca (sizeof (type) * (count))

inline ndInt32 dExp2 (ndInt32 x)
{
	ndInt32 exp;
	for (exp = -1; x; x >>= 1) 
	{
		exp ++;
	}
	return exp;
}

inline ndInt32 dBitReversal(ndInt32 v, ndInt32 base)
{
	ndInt32 x = 0;
	ndInt32 power = dExp2 (base) - 1;
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
inline T dMin(T A, T B)
{
	return (A < B) ? A : B; 
}

template <class T> 
inline T dMax(T A, T B)
{
	return (A > B) ? A : B; 
}

template <class T>
inline T dClamp(T val, T min, T max)
{
	return dMax (min, dMin (max, val));
}

template <class T> 
inline void dSwap(T& A, T& B)
{
	T tmp (A);
	A = B;
	B = tmp;
}	

template <class T>
inline T dAbs(T A)
{
	// according to Intel this is better because is does not read after write
	return (A >= T(0)) ? A : -A;
}

template <class T>
inline T dSign(T A)
{
	return (A >= T(0)) ? T(1) : T(-1);
}

template <class T> 
inline bool dAreEqual(T A, T B, T tol)
{
	// deal with too small and de normal values.
	if ((dAbs(A) < tol) && (dAbs(B) < tol)) 
	{
		return true;
	}

	ndInt32 exp0;
	ndFloat64 mantissa0 = frexp(ndFloat64 (A), &exp0);
	
	ndInt32 exp1;
	ndFloat64 mantissa1 = frexp(ndFloat64(B), &exp1);
	if (dAbs(exp0 - exp1) > 1)
	{
		return false;
	}
	if (exp0 != exp1)
	{
		if (exp0 > exp1)
		{
			mantissa0 *= ndFloat32(2.0f);
		}
		else
		{
			mantissa1 *= ndFloat32(2.0f);
		}
	}
	return dAbs(mantissa0 - mantissa1) < tol;
}

template <class T>
inline T AnglesAdd (T angleInRadiand1, T angleInRadiand0)
{
	T s1 = T(ndSin(angleInRadiand1));
	T c1 = T(ndCos(angleInRadiand1));
	T s0 = T(ndSin(angleInRadiand0));
	T c0 = T(ndCos(angleInRadiand0));

	T s = s1 * c0 + s0 * c1;
	T c = c1 * c0 - s0 * s1;
	return T(ndAtan2(s, c));
}

/// Returns the time in micro seconds since application started 
D_CORE_API ndUnsigned64 dGetTimeInMicroseconds();

D_CORE_API ndFloat32 dRand();
D_CORE_API ndInt32 dRandInt();
D_CORE_API void dSetRandSeed(ndUnsigned32 seed);
D_CORE_API ndFloat32 dGaussianRandom(ndFloat32 amp);

/// Round a 64 bit float to a 32 bit float by truncating the mantissa a 24 bit 
/// \param ndFloat64 val: 64 bit float 
/// \return a 64 bit double precision with a 32 bit mantissa
D_CORE_API ndFloat64 dRoundToFloat(ndFloat64 val);

D_CORE_API ndInt32 dVertexListToIndexList(ndFloat64* const vertexList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, ndFloat64 tolerance = ndEpsilon);

template <class T>
ndInt32 dVertexListToIndexList(T* const vertexList, ndInt32 strideInBytes, ndInt32 compareCount, ndInt32 vertexCount, ndInt32* const indexListOut, T tolerance = ndEpsilon)
{
	ndInt32 stride = ndInt32(strideInBytes / sizeof(T));
	ndStack<ndFloat64> pool(vertexCount * stride);

	ndFloat64* const data = &pool[0];
	for (ndInt32 i = 0; i < vertexCount; i++)
	{
		ndFloat64* const dst = &data[i * stride];
		const T* const src = &vertexList[i * stride];
		for (ndInt32 j = 0; j < stride; j++)
		{
			dst[j] = src[j];
		}
	}

	ndInt32 count = dVertexListToIndexList(data, ndInt32(stride * sizeof(ndFloat64)), compareCount, vertexCount, indexListOut, ndFloat64(tolerance));
	for (ndInt32 i = 0; i < count; i++)
	{
		const ndFloat64* const src = &data[i * stride];
		T* const dst = &vertexList[i * stride];
		for (ndInt32 j = 0; j < stride; j++)
		{
			dst[j] = T(src[j]);
		}
	}

	return count;
}

/// Set cpu floating point exceptions, the original exception state is restored when the destructor is called.
class ndFloatExceptions
{
	public:
	#ifdef _MSC_VER
		#define D_FLOAT_EXECTIONS_MASK	(EM_INVALID | EM_DENORMAL | EM_ZERODIVIDE)
	#else 
		#define D_FLOAT_EXECTIONS_MASK	0
	#endif

	ndFloatExceptions(ndUnsigned32 mask = D_FLOAT_EXECTIONS_MASK);
	~ndFloatExceptions();

	private:
	#if (defined(WIN32) || defined(_WIN32))
		ndUnsigned32 m_mask;
	#endif
};

/// Set cpu floating point precision mode, the original mode is restored when the destructor is called.
class ndSetPrecisionDouble 
{
	public:
	ndSetPrecisionDouble();
	~ndSetPrecisionDouble();
	#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	ndInt32 m_mask; 
	#endif
};

#endif

