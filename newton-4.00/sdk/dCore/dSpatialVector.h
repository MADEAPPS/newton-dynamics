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

#ifndef __D_SPATIAL_VECTOR_H__
#define __D_SPATIAL_VECTOR_H__

#include "dCoreStdafx.h"
#include "dTypes.h"

D_MSV_NEWTON_ALIGN_32
class dSpatialVector
{
	public:
	D_INLINE dSpatialVector() = default;

	D_INLINE dSpatialVector(const dFloat64 a)
		:m{a, a}
	{
	}

	D_INLINE dSpatialVector(const dSpatialVector& copy)
		:m{copy.m.low, copy.m.high}
	{
	}

	D_INLINE dSpatialVector(const dBigVector& low, const dBigVector& high)
        :m{low, high}
	{
	}

	D_INLINE dFloat64& operator[] (dInt32 i)
	{
		dAssert(i >= 0);
		dAssert(i < sizeof (m_f) / sizeof (m_f[0]));
		return ((dFloat64*)&m_f)[i];
	}

	D_INLINE const dFloat64& operator[] (dInt32 i) const
	{
		dAssert(i >= 0);
		dAssert(i < sizeof(m_f) / sizeof(m_f[0]));
		return ((dFloat64*)&m_f)[i];
	}

	D_INLINE dSpatialVector operator+ (const dSpatialVector& A) const
	{
		return dSpatialVector(m.low + A.m.low, m.high + A.m.high);
	}

	D_INLINE dSpatialVector operator*(const dSpatialVector& A) const
	{
		return dSpatialVector(m.low * A.m.low, m.high * A.m.high);
	}

	D_INLINE dFloat64 DotProduct(const dSpatialVector& v) const
	{
		dAssert(m_f[6] == dFloat32(0.0f));
		dAssert(m_f[7] == dFloat32(0.0f));
		dBigVector tmp(m.low * v.m.low + m.high * v.m.high);
		return tmp.AddHorizontal().GetScalar();
	}

	D_INLINE dSpatialVector Scale(dFloat64 s) const
	{
		dBigVector tmp(s);
		return dSpatialVector(m.low * tmp, m.high * tmp);
	}

    struct Bisect
    {
        D_INLINE Bisect() = default;
        D_INLINE Bisect(const dBigVector & l, const dBigVector & h)
            :low(l), high(h)
        {
        }

        dBigVector low;
        dBigVector high;
    };

	union
	{
		dFloat64 m_f[8];
        Bisect m;
	};

	D_CORE_API static dSpatialVector m_zero;
} D_GCC_NEWTON_ALIGN_32;

#endif
