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

	DG_INLINE dgAvxFloat operator* (const dgAvxFloat& A) const
	{
		return _mm256_mul_ps(m_type, A.m_type);
	}

	DG_INLINE dgAvxFloat operator+ (const dgAvxFloat& A) const
	{
		return _mm256_add_ps(m_type, A.m_type);
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

DG_MSC_AVX_ALIGMENT
class dgAvxVector3
{
	public:
	DG_INLINE dgAvxVector3()
		:m_x()
		,m_y()
		,m_z()
	{
	}

	DG_INLINE dgAvxVector3(const dgAvxVector3& val)
		:m_x(val.m_x)
		,m_y(val.m_y)
		,m_z(val.m_z)
	{
	}

	DG_INLINE dgAvxVector3(const dgAvxFloat& x, const dgAvxFloat& y, const dgAvxFloat& z)
		:m_x(x)
		,m_y(y)
		,m_z(z)
	{
	}

	DG_INLINE dgAvxVector3(const dgVector& v0, const dgVector& v1, const dgVector& v2, const dgVector& v3, const dgVector& v4, const dgVector& v5, const dgVector& v6, const dgVector& v7)
		:m_x()
		,m_y()
		,m_z()
	{
		dgAvxFloat r0(v0, v4);
		dgAvxFloat r1(v1, v5);
		dgAvxFloat r2(v2, v6);
		dgAvxFloat r3(v3, v7);
		dgAvxFloat::Transpose4x8(r0, r1, r2, r3);
		m_x = r0;
		m_y = r1;
		m_z = r2;
	}

	DG_INLINE dgAvxVector3 Scale(const dgAvxFloat& a) const
	{
		return dgAvxVector3(m_x * a, m_y * a, m_z * a);
	}

	DG_INLINE dgAvxVector3 operator* (const dgAvxVector3& a) const
	{
		return dgAvxVector3(m_x * a.m_x, m_y * a.m_y, m_z * a.m_z);
	}

	dgAvxFloat m_x;
	dgAvxFloat m_y;
	dgAvxFloat m_z;
} DG_GCC_AVX_ALIGMENT;

DG_MSC_AVX_ALIGMENT
class dgAvxMatrix3x3
{
	public:
	DG_INLINE dgAvxMatrix3x3()
		:m_right()
		,m_up()
		,m_front()
	{
	}

	DG_INLINE dgAvxMatrix3x3(const dgAvxMatrix3x3& val)
		:m_right(val.m_right)
		,m_up(val.m_up)
		,m_front(val.m_front)
	{
	}

	DG_INLINE dgAvxMatrix3x3(const dgAvxVector3& x, const dgAvxVector3& y, const dgAvxVector3& z)
		:m_right(x)
		,m_up(y)
		,m_front(z)
	{
	}

	DG_INLINE dgAvxMatrix3x3(const dgMatrix& matrix0, const dgMatrix& matrix1, const dgMatrix& matrix2, const dgMatrix& matrix3, const dgMatrix& matrix4, const dgMatrix& matrix5, const dgMatrix& matrix6, const dgMatrix& matrix7)
		:m_right(matrix0[0], matrix1[0], matrix2[0], matrix3[0], matrix4[0], matrix5[0], matrix6[0], matrix7[0])
		,m_up(matrix0[1], matrix1[1], matrix2[1], matrix3[1], matrix4[1], matrix5[1], matrix6[1], matrix7[1])
		,m_front(matrix0[2], matrix1[2], matrix2[2], matrix3[2], matrix4[2], matrix5[2], matrix6[2], matrix7[2])
	{
	}

	DG_INLINE dgAvxMatrix3x3 Transpose () const
	{
		dgAvxFloat tmp;
		dgAvxMatrix3x3 ret (*this);
		dgAvxFloat::Transpose4x8(ret.m_front.m_x, ret.m_front.m_y, ret.m_front.m_z, tmp);
		dgAvxFloat::Transpose4x8(ret.m_up.m_x, ret.m_up.m_y, ret.m_up.m_z, tmp);
		dgAvxFloat::Transpose4x8(ret.m_right.m_x, ret.m_right.m_y, ret.m_right.m_z, tmp);
		return ret;
	}

	DG_INLINE dgAvxMatrix3x3 operator* (const dgAvxMatrix3x3& a) const
	{
		return dgAvxMatrix3x3();
	}

	DG_INLINE dgAvxVector3 RotateVector(const dgAvxVector3& a) const
	{
		//dgAvxFloat x(((a.m_x * m_front.m_x).MultAdd(a.m_y, m_up.m_x)).MultAdd(a.m_z, m_right.m_x));
		//dgAvxFloat y(((a.m_x * m_front.m_y).MultAdd(a.m_y, m_up.m_y)).MultAdd(a.m_z, m_right.m_y));
		//dgAvxFloat z(((a.m_x * m_front.m_z).MultAdd(a.m_y, m_up.m_z)).MultAdd(a.m_z, m_right.m_z));
		dgAvxFloat x(a.m_x * m_front.m_x + a.m_y * m_up.m_x + a.m_z * m_right.m_x);
		dgAvxFloat y(a.m_x * m_front.m_y + a.m_y * m_up.m_y + a.m_z * m_right.m_y);
		dgAvxFloat z(a.m_x * m_front.m_z + a.m_y * m_up.m_z + a.m_z * m_right.m_z);
		return dgAvxVector3(x, y, z);
	}

	DG_INLINE dgAvxVector3 UnrotateVector(const dgAvxVector3& a) const
	{
		//dgAvxFloat x(((a.m_x * m_front.m_x).MultAdd (a.m_y, m_front.m_y)).MultAdd(a.m_z, m_front.m_z));
		//dgAvxFloat y(((a.m_x * m_up.m_x).MultAdd(a.m_y, m_up.m_y)).MultAdd(a.m_z, m_up.m_z));
		//dgAvxFloat z(((a.m_x * m_right.m_x).MultAdd(a.m_y, m_right.m_y)).MultAdd(a.m_z, m_right.m_z));

		dgAvxFloat x(a.m_x * m_front.m_x + a.m_y * m_front.m_y + a.m_z * m_front.m_z);
		dgAvxFloat y(a.m_x * m_up.m_x    + a.m_y * m_up.m_y    + a.m_z * m_up.m_z);
		dgAvxFloat z(a.m_x * m_right.m_x + a.m_y * m_right.m_y + a.m_z * m_right.m_z);
		return dgAvxVector3(x, y, z);
	}
	
	dgAvxVector3 m_front;
	dgAvxVector3 m_up;
	dgAvxVector3 m_right;
	
} DG_GCC_AVX_ALIGMENT;

#endif

