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

#ifndef _DG_MATH_AVX2_H_
#define _DG_MATH_AVX2_H_
#include "dgNewtonPluginStdafx.h"
#include "dgMathAvx.h"

DG_MSC_AVX_ALIGMENT
class dgFloatAvx2: public dgFloatAvx
{
	public:
	DG_INLINE dgFloatAvx2()
		:dgFloatAvx()
	{
	}

	DG_INLINE dgFloatAvx2(const __m256 type)
		:dgFloatAvx(type)
	{
	}

	DG_INLINE dgFloatAvx2(const dgFloatAvx& copy)
		:dgFloatAvx(copy)
	{
	}

	DG_INLINE dgFloatAvx2(const dgVector& low, const dgVector& high)
		: dgFloatAvx(low, high)
	{
	}

	DG_INLINE dgFloatAvx2 operator+ (const dgFloatAvx2& A) const
	{
		//return _mm256_fmadd_ps(A.m_type, m_zero.m_type, m_type);
		return _mm256_add_ps(m_type, A.m_type);
	}

	DG_INLINE dgFloatAvx2 operator* (const dgFloatAvx2& A) const
	{
		//return _mm256_fmadd_ps(A.m_type, m_type, m_zero.m_type);
		return _mm256_mul_ps(m_type, A.m_type);
	}

	DG_INLINE dgFloatAvx2 MulAdd(const dgFloatAvx2& A, const dgFloatAvx2& B) const
	{
		return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
	}

	static dgFloatAvx2 m_one;
	static dgFloatAvx2 m_zero;
} DG_GCC_AVX_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgVector3Avx2
{
	public:
	DG_INLINE dgVector3Avx2()
		:m_x()
		,m_y()
		,m_z()
	{
	}

	DG_INLINE dgVector3Avx2(const dgVector3Avx2& val)
		:m_x(val.m_x)
		,m_y(val.m_y)
		,m_z(val.m_z)
	{
	}

	DG_INLINE dgVector3Avx2(const dgFloatAvx2& x, const dgFloatAvx2& y, const dgFloatAvx2& z)
		:m_x(x)
		,m_y(y)
		,m_z(z)
	{
	}

	DG_INLINE dgVector3Avx2(const dgVector& v0, const dgVector& v1, const dgVector& v2, const dgVector& v3, const dgVector& v4, const dgVector& v5, const dgVector& v6, const dgVector& v7)
		:m_x()
		,m_y()
		,m_z()
	{
		dgFloatAvx2 r0(v0, v4);
		dgFloatAvx2 r1(v1, v5);
		dgFloatAvx2 r2(v2, v6);
		dgFloatAvx2 r3(v3, v7);
		dgFloatAvx2::Transpose4x8(r0, r1, r2, r3);
		m_x = r0;
		m_y = r1;
		m_z = r2;
	}

	DG_INLINE void Store(dgVector3Avx2* const ptr) const
	{
		m_x.Store(&ptr->m_x);
		m_y.Store(&ptr->m_y);
		m_z.Store(&ptr->m_z);
	}

	DG_INLINE dgVector3Avx2 Scale(const dgFloatAvx2& a) const
	{
		return dgVector3Avx2(m_x * a, m_y * a, m_z * a);
	}

	DG_INLINE dgVector3Avx2 operator* (const dgVector3Avx2& a) const
	{
		return dgVector3Avx2(m_x * a.m_x, m_y * a.m_y, m_z * a.m_z);
	}

	dgFloatAvx2 m_x;
	dgFloatAvx2 m_y;
	dgFloatAvx2 m_z;
} DG_GCC_AVX_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgVector6Avx2
{
	public:
	dgVector3Avx2 m_linear;
	dgVector3Avx2 m_angular;

} DG_GCC_AVX_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgMatrix3x3Avx2
{
	public:
	DG_INLINE dgMatrix3x3Avx2()
		:m_right()
		,m_up()
		,m_front()
	{
	}

	DG_INLINE dgMatrix3x3Avx2(const dgMatrix3x3Avx2& val)
		:m_right(val.m_right)
		,m_up(val.m_up)
		,m_front(val.m_front)
	{
	}

	DG_INLINE dgMatrix3x3Avx2(const dgVector3Avx2& x, const dgVector3Avx2& y, const dgVector3Avx2& z)
		:m_right(x)
		,m_up(y)
		,m_front(z)
	{
	}

	DG_INLINE dgMatrix3x3Avx2(const dgMatrix& matrix0, const dgMatrix& matrix1, const dgMatrix& matrix2, const dgMatrix& matrix3, const dgMatrix& matrix4, const dgMatrix& matrix5, const dgMatrix& matrix6, const dgMatrix& matrix7)
		:m_right(matrix0[0], matrix1[0], matrix2[0], matrix3[0], matrix4[0], matrix5[0], matrix6[0], matrix7[0])
		,m_up(matrix0[1], matrix1[1], matrix2[1], matrix3[1], matrix4[1], matrix5[1], matrix6[1], matrix7[1])
		,m_front(matrix0[2], matrix1[2], matrix2[2], matrix3[2], matrix4[2], matrix5[2], matrix6[2], matrix7[2])
	{
	}

	DG_INLINE void Store(dgMatrix3x3Avx2* const ptr) const
	{
		m_front.Store(&ptr->m_front);
		m_up.Store(&ptr->m_up);
		m_right.Store(&ptr->m_right);
	}

	DG_INLINE dgMatrix3x3Avx2 Transposed() const
	{
		return dgMatrix3x3Avx2(dgVector3Avx2(m_front.m_x, m_up.m_x, m_right.m_x),
							  dgVector3Avx2(m_front.m_y, m_up.m_y, m_right.m_y),
							  dgVector3Avx2(m_front.m_z, m_up.m_z, m_right.m_z));
	}

	DG_INLINE dgMatrix3x3Avx2 operator* (const dgMatrix3x3Avx2& a) const
	{
		return dgMatrix3x3Avx2(a.RotateVector(m_front), a.RotateVector(m_up), a.RotateVector(m_right));
	}

	DG_INLINE dgVector3Avx2 RotateVector(const dgVector3Avx2& a) const
	{
		//dgFloatAvx2 x(dgFloatAvx2::m_zero.MulAdd(a.m_x, m_front.m_x).MulAdd(a.m_y, m_up.m_x).MulAdd(a.m_z, m_right.m_x));
		//dgFloatAvx2 y(dgFloatAvx2::m_zero.MulAdd(a.m_x, m_front.m_y).MulAdd(a.m_y, m_up.m_y).MulAdd(a.m_z, m_right.m_y));
		//dgFloatAvx2 z(dgFloatAvx2::m_zero.MulAdd(a.m_x, m_front.m_z).MulAdd(a.m_y, m_up.m_z).MulAdd(a.m_z, m_right.m_z));
		dgFloatAvx2 x((a.m_x * m_front.m_x).MulAdd(a.m_y, m_up.m_x).MulAdd(a.m_z, m_right.m_x));
		dgFloatAvx2 y((a.m_x * m_front.m_y).MulAdd(a.m_y, m_up.m_y).MulAdd(a.m_z, m_right.m_y));
		dgFloatAvx2 z((a.m_x * m_front.m_z).MulAdd(a.m_y, m_up.m_z).MulAdd(a.m_z, m_right.m_z));
		return dgVector3Avx2(x, y, z);
	}

	DG_INLINE dgVector3Avx2 UnrotateVector(const dgVector3Avx2& a) const
	{
		//dgFloatAvx2 x(dgFloatAvx2::m_zero.MulAdd(a.m_x, m_front.m_x).MulAdd(a.m_y, m_front.m_y).MulAdd(a.m_z, m_front.m_z));
		//dgFloatAvx2 y(dgFloatAvx2::m_zero.MulAdd(a.m_x, m_up.m_x).MulAdd(a.m_y, m_up.m_y).MulAdd(a.m_z, m_up.m_z));
		//dgFloatAvx2 z(dgFloatAvx2::m_zero.MulAdd(a.m_x, m_right.m_x).MulAdd(a.m_y, m_right.m_y).MulAdd(a.m_z, m_right.m_z));
		dgFloatAvx2 x((a.m_x * m_front.m_x).MulAdd(a.m_y, m_front.m_y).MulAdd(a.m_z, m_front.m_z));
		dgFloatAvx2 y((a.m_x * m_up.m_x).MulAdd(a.m_y, m_up.m_y).MulAdd(a.m_z, m_up.m_z));
		dgFloatAvx2 z((a.m_x * m_right.m_x).MulAdd(a.m_y, m_right.m_y).MulAdd(a.m_z, m_right.m_z));
		return dgVector3Avx2(x, y, z);
	}
	
	dgVector3Avx2 m_front;
	dgVector3Avx2 m_up;
	dgVector3Avx2 m_right;
	
} DG_GCC_AVX_ALIGMENT;

#endif

