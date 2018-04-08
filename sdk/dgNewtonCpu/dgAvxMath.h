/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_SCALAR_H_
#define _DG_SCALAR_H_
#include "dgNewtonCpuStdafx.h"


DG_MSC_AVX_ALIGMENT
class dgAvxFloat
{
	#define PERMUTE_MASK(w, z, y, x)		_MM_SHUFFLE (w, z, y, x)
	public:
	DG_INLINE dgAvxFloat()
	{
	}

	DG_INLINE dgAvxFloat(float val)
		:m_type(_mm256_set1_ps (val))
	{
	}

	DG_INLINE dgAvxFloat(const __m256 type)
		:m_type(type)
	{
	}

	DG_INLINE dgAvxFloat(const dgAvxFloat& copy)
		:m_type(copy.m_type)
	{
	}

	DG_INLINE dgAvxFloat (const dgVector& low, const dgVector& high)
		:m_type (_mm256_loadu2_m128 (&high.m_x, &low.m_x))
	{
	}

	DG_INLINE static void Transpose4x8(dgAvxFloat& src0, dgAvxFloat& src1, dgAvxFloat& src2, dgAvxFloat& src3)
	{
		__m256 tmp0(_mm256_unpacklo_ps(src0.m_type, src1.m_type));
		__m256 tmp1(_mm256_unpacklo_ps(src2.m_type, src3.m_type));
		__m256 tmp2(_mm256_unpackhi_ps(src0.m_type, src1.m_type));
		__m256 tmp3(_mm256_unpackhi_ps(src2.m_type, src3.m_type));

		src0 = _mm256_shuffle_ps(tmp0, tmp1, PERMUTE_MASK(1, 0, 1, 0));
		src1 = _mm256_shuffle_ps(tmp0, tmp1, PERMUTE_MASK(3, 2, 3, 2));
		src2 = _mm256_shuffle_ps(tmp2, tmp3, PERMUTE_MASK(1, 0, 1, 0));
		src3 = _mm256_shuffle_ps(tmp2, tmp3, PERMUTE_MASK(3, 2, 3, 2));
	}


	union
	{
		__m256 m_type;
		__m256i m_typeInt;
		float m_f[8];
		int m_i[8];
	};
} DG_GCC_AVX_ALIGMENT;


class dgAvxScalar
{
	public:
	dgAvxScalar (dgMemoryAllocator* const allocator)
		:m_val(allocator)
	{
	}
	void Reserve (dgInt32 count)
	{
		m_val.ResizeIfNecessary(count);
	}

	dgArray<dgAvxFloat> m_val;
};

class dgAvxVector3
{
	public:
	dgAvxVector3 (dgMemoryAllocator* const allocator)
		:m_x(allocator)
		,m_y(allocator)
		,m_z(allocator)
	{
	}
	void Reserve (dgInt32 count)
	{
		m_x.ResizeIfNecessary(count);
		m_y.ResizeIfNecessary(count);
		m_z.ResizeIfNecessary(count);
	}

	dgArray<dgAvxFloat> m_x;
	dgArray<dgAvxFloat> m_y;
	dgArray<dgAvxFloat> m_z;
};

#endif

