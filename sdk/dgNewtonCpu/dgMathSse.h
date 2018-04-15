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

#ifndef _DG_MATH_SSE_H_
#define _DG_MATH_SSE_H_
#include "dgNewtonPluginStdafx.h"

DG_MSC_AVX_ALIGMENT
class dgFloatSse
{
	#define PERMUTE_MASK(w, z, y, x)		_MM_SHUFFLE (w, z, y, x)
	public:
	DG_INLINE dgFloatSse()
	{
	}

	DG_INLINE dgFloatSse(float val)
		:m_type(_mm_set1_ps (val))
	{
	}

	DG_INLINE dgFloatSse(const __m128 type)
		:m_type(type)
	{
	}

	DG_INLINE dgFloatSse(const dgFloatSse& copy)
		:m_type(copy.m_type)
	{
	}

	DG_INLINE dgFloatSse(const dgVector& low)
		:m_type(low.m_type)
	{
	}

	DG_INLINE static void ClearFlops()
	{
		#ifdef _DEBUG
		m_flopsCount = 0;
		#endif
	}

	DG_INLINE static void IncFlops()
	{
		#ifdef _DEBUG
		m_flopsCount++;
		#endif
	}

	DG_INLINE static dgUnsigned32 GetFlops()
	{
#ifdef _DEBUG
		return m_flopsCount;
#else 
		return 0;
#endif
	}

	DG_INLINE void Store (dgFloatSse* const ptr ) const
	{
		_mm_store_ps((float*) ptr, m_type);
	}

	DG_INLINE dgFloatSse operator+ (const dgFloatSse& A) const
	{
		IncFlops();
		return _mm_add_ps(m_type, A.m_type);
	}

	DG_INLINE dgFloatSse operator* (const dgFloatSse& A) const
	{
		IncFlops();
		return _mm_mul_ps(m_type, A.m_type);
	}

	DG_INLINE static void Transpose4x4(dgFloatSse& src0, dgFloatSse& src1, dgFloatSse& src2, dgFloatSse& src3)
	{
		__m128 tmp0(_mm_unpacklo_ps(src0.m_type, src1.m_type));
		__m128 tmp1(_mm_unpacklo_ps(src2.m_type, src3.m_type));
		__m128 tmp2(_mm_unpackhi_ps(src0.m_type, src1.m_type));
		__m128 tmp3(_mm_unpackhi_ps(src2.m_type, src3.m_type));

		src0 = _mm_shuffle_ps(tmp0, tmp1, PERMUTE_MASK(1, 0, 1, 0));
		src1 = _mm_shuffle_ps(tmp0, tmp1, PERMUTE_MASK(3, 2, 3, 2));
		src2 = _mm_shuffle_ps(tmp2, tmp3, PERMUTE_MASK(1, 0, 1, 0));
		src3 = _mm_shuffle_ps(tmp2, tmp3, PERMUTE_MASK(3, 2, 3, 2));
	}

	union
	{
		__m128 m_type;
		__m128i m_typeInt;
		float m_f[8];
		int m_i[8];
	};

	static dgFloatSse m_one;
	static dgFloatSse m_zero;
#ifdef _DEBUG
	static dgUnsigned32 m_flopsCount;
#endif
} DG_GCC_AVX_ALIGMENT;

DG_MSC_AVX_ALIGMENT
class dgVector3Sse
{
	public:
	DG_INLINE dgVector3Sse()
		:m_x()
		,m_y()
		,m_z()
	{
	}

	DG_INLINE dgVector3Sse(const dgVector3Sse& val)
		:m_x(val.m_x)
		,m_y(val.m_y)
		,m_z(val.m_z)
	{
	}

	DG_INLINE dgVector3Sse(const dgFloatSse& x, const dgFloatSse& y, const dgFloatSse& z)
		:m_x(x)
		,m_y(y)
		,m_z(z)
	{
	}

	DG_INLINE dgVector3Sse(const dgVector& v0, const dgVector& v1, const dgVector& v2, const dgVector& v3)
		:m_x()
		,m_y()
		,m_z()
	{
		dgFloatSse r0(v0);
		dgFloatSse r1(v1);
		dgFloatSse r2(v2);
		dgFloatSse r3(v3);
		dgFloatSse::Transpose4x4(r0, r1, r2, r3);
		m_x = r0;
		m_y = r1;
		m_z = r2;
	}

	DG_INLINE void Store(dgVector3Sse* const ptr) const
	{
		m_x.Store(&ptr->m_x);
		m_y.Store(&ptr->m_y);
		m_z.Store(&ptr->m_z);
	}

	DG_INLINE dgVector3Sse Scale(const dgFloatSse& a) const
	{
		return dgVector3Sse(m_x * a, m_y * a, m_z * a);
	}

	DG_INLINE dgVector3Sse operator* (const dgVector3Sse& a) const
	{
		return dgVector3Sse(m_x * a.m_x, m_y * a.m_y, m_z * a.m_z);
	}

	dgFloatSse m_x;
	dgFloatSse m_y;
	dgFloatSse m_z;
} DG_GCC_AVX_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgVector6Sse
{
	public:
	dgVector3Sse m_linear;
	dgVector3Sse m_angular;

} DG_GCC_AVX_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgMatrix3x3Sse
{
	public:
	DG_INLINE dgMatrix3x3Sse()
		:m_right()
		,m_up()
		,m_front()
	{
	}

	DG_INLINE dgMatrix3x3Sse(const dgMatrix3x3Sse& val)
		:m_right(val.m_right)
		,m_up(val.m_up)
		,m_front(val.m_front)
	{
	}

	DG_INLINE dgMatrix3x3Sse(const dgVector3Sse& x, const dgVector3Sse& y, const dgVector3Sse& z)
		:m_right(x)
		,m_up(y)
		,m_front(z)
	{
	}

	DG_INLINE dgMatrix3x3Sse(const dgMatrix& matrix0, const dgMatrix& matrix1, const dgMatrix& matrix2, const dgMatrix& matrix3)
		:m_right(matrix0[0], matrix1[0], matrix2[0], matrix3[0])
		,m_up(matrix0[1], matrix1[1], matrix2[1], matrix3[1])
		,m_front(matrix0[2], matrix1[2], matrix2[2], matrix3[2])
	{
	}

	DG_INLINE void Store(dgMatrix3x3Sse* const ptr) const
	{
		m_front.Store(&ptr->m_front);
		m_up.Store(&ptr->m_up);
		m_right.Store(&ptr->m_right);
	}

	DG_INLINE dgMatrix3x3Sse Transposed() const
	{
		return dgMatrix3x3Sse(dgVector3Sse(m_front.m_x, m_up.m_x, m_right.m_x),
							  dgVector3Sse(m_front.m_y, m_up.m_y, m_right.m_y),
							  dgVector3Sse(m_front.m_z, m_up.m_z, m_right.m_z));
	}

	DG_INLINE dgMatrix3x3Sse operator* (const dgMatrix3x3Sse& a) const
	{
		return dgMatrix3x3Sse(a.RotateVector(m_front), a.RotateVector(m_up), a.RotateVector(m_right));
	}

	DG_INLINE dgVector3Sse RotateVector(const dgVector3Sse& a) const
	{
		dgFloatSse x(a.m_x * m_front.m_x + a.m_y * m_up.m_x + a.m_z * m_right.m_x);
		dgFloatSse y(a.m_x * m_front.m_y + a.m_y * m_up.m_y + a.m_z * m_right.m_y);
		dgFloatSse z(a.m_x * m_front.m_z + a.m_y * m_up.m_z + a.m_z * m_right.m_z);
		return dgVector3Sse(x, y, z);
	}

	DG_INLINE dgVector3Sse UnrotateVector(const dgVector3Sse& a) const
	{
		dgFloatSse x(a.m_x * m_front.m_x + a.m_y * m_front.m_y + a.m_z * m_front.m_z);
		dgFloatSse y(a.m_x * m_up.m_x    + a.m_y * m_up.m_y    + a.m_z * m_up.m_z);
		dgFloatSse z(a.m_x * m_right.m_x + a.m_y * m_right.m_y + a.m_z * m_right.m_z);
		return dgVector3Sse(x, y, z);
	}
	
	dgVector3Sse m_front;
	dgVector3Sse m_up;
	dgVector3Sse m_right;
	
} DG_GCC_AVX_ALIGMENT;

#endif

