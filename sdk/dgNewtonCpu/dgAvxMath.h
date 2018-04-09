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

template<class T>
class dgAvxArray: public dgArray<T>
{
	public:
	dgAvxArray (dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes = DG_MEMORY_GRANULARITY)
		:dgArray<T>(allocator, aligmentInBytes)
		,m_ptr(NULL)
	{
	}

	void Reserve(dgInt32 count)
	{
		ResizeIfNecessary(count);
		dgArray<T>& me = *this;
		m_ptr = &me[0];
	}

	DG_INLINE T& operator[] (dgInt32 i)
	{
		return m_ptr[i];
	}

	DG_INLINE const T& operator[] (dgInt32 i) const
	{
		return m_ptr[i];
	}

	T* m_ptr;
};


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

	DG_INLINE dgAvxFloat MultAdd(const dgAvxFloat& a, const dgAvxFloat& b) const
	{
		return _mm256_fmadd_ps(a.m_type, b.m_type, m_type);
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
class dgAvxVector3_local
{
	public:
	DG_INLINE dgAvxVector3_local()
		:m_x()
		,m_y()
		,m_z()
	{
	}

	DG_INLINE dgAvxVector3_local(const dgAvxVector3_local& val)
		:m_x(val.m_x)
		,m_y(val.m_y)
		,m_z(val.m_z)
	{
	}

	DG_INLINE dgAvxVector3_local(const dgAvxFloat& x, const dgAvxFloat& y, const dgAvxFloat& z)
		:m_x(x)
		,m_y(y)
		,m_z(z)
	{
	}

	DG_INLINE dgAvxVector3_local Scale(const dgAvxFloat& a) const
	{
		return dgAvxVector3_local(m_x * a, m_y * a, m_z * a);
	}

	DG_INLINE dgAvxVector3_local operator* (const dgAvxVector3_local& a) const
	{
		return dgAvxVector3_local(m_x * a.m_x, m_y * a.m_y, m_z * a.m_z);
	}

	DG_INLINE dgAvxVector3_local MultAdd (const dgAvxVector3_local& a, const dgAvxVector3_local& b) const
	{
		return dgAvxVector3_local(m_x.MultAdd (a.m_x, b.m_x), m_y.MultAdd(a.m_y, b.m_y), m_z.MultAdd(a.m_z, b.m_z));
	}


	dgAvxFloat m_x;
	dgAvxFloat m_y;
	dgAvxFloat m_z;
} DG_GCC_AVX_ALIGMENT;

DG_MSC_AVX_ALIGMENT
class dgAvxMatrix3x3_local
{
	public:
	DG_INLINE dgAvxMatrix3x3_local()
		:m_right()
		,m_up()
		,m_front()
	{
	}

	DG_INLINE dgAvxMatrix3x3_local(const dgAvxMatrix3x3_local& val)
		:m_right(val.m_right)
		,m_up(val.m_up)
		,m_front(val.m_front)
	{
	}

	DG_INLINE dgAvxMatrix3x3_local(const dgAvxVector3_local& x, const dgAvxVector3_local& y, const dgAvxVector3_local& z)
		:m_right(x)
		,m_up(y)
		,m_front(z)
	{
	}

	DG_INLINE dgAvxVector3_local RotateVector(const dgAvxVector3_local& a) const
	{
		//dgAvxFloat x(((a.m_x * m_front.m_x).MultAdd(a.m_y, m_up.m_x)).MultAdd(a.m_z, m_right.m_x));
		//dgAvxFloat y(((a.m_x * m_front.m_y).MultAdd(a.m_y, m_up.m_y)).MultAdd(a.m_z, m_right.m_y));
		//dgAvxFloat z(((a.m_x * m_front.m_z).MultAdd(a.m_y, m_up.m_z)).MultAdd(a.m_z, m_right.m_z));
		dgAvxFloat x(a.m_x * m_front.m_x + a.m_y * m_up.m_x + a.m_z * m_right.m_x);
		dgAvxFloat y(a.m_x * m_front.m_y + a.m_y * m_up.m_y + a.m_z * m_right.m_y);
		dgAvxFloat z(a.m_x * m_front.m_z + a.m_y * m_up.m_z + a.m_z * m_right.m_z);
		return dgAvxVector3_local(x, y, z);
	}

	DG_INLINE dgAvxVector3_local UnrotateVector(const dgAvxVector3_local& a) const
	{
		//dgAvxFloat x(((a.m_x * m_front.m_x).MultAdd (a.m_y, m_front.m_y)).MultAdd(a.m_z, m_front.m_z));
		//dgAvxFloat y(((a.m_x * m_up.m_x).MultAdd(a.m_y, m_up.m_y)).MultAdd(a.m_z, m_up.m_z));
		//dgAvxFloat z(((a.m_x * m_right.m_x).MultAdd(a.m_y, m_right.m_y)).MultAdd(a.m_z, m_right.m_z));

		dgAvxFloat x(a.m_x * m_front.m_x + a.m_y * m_front.m_y + a.m_z * m_front.m_z);
		dgAvxFloat y(a.m_x * m_up.m_x    + a.m_y * m_up.m_y    + a.m_z * m_up.m_z);
		dgAvxFloat z(a.m_x * m_right.m_x + a.m_y * m_right.m_y + a.m_z * m_right.m_z);
		return dgAvxVector3_local(x, y, z);
	}
	
	dgAvxVector3_local m_front;
	dgAvxVector3_local m_up;
	dgAvxVector3_local m_right;
	
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
		m_val.Reserve(count);
	}

	dgAvxArray<dgAvxFloat> m_val;
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
		m_x.Reserve(count);
		m_y.Reserve(count);
		m_z.Reserve(count);
	}

	void InitVector3(dgInt32 index, const dgVector& v0, const dgVector& v1, const dgVector& v2, const dgVector& v3, const dgVector& v4, const dgVector& v5, const dgVector& v6, const dgVector& v7)
	{
		dgAvxFloat r0(v0, v4);
		dgAvxFloat r1(v1, v5);
		dgAvxFloat r2(v2, v6);
		dgAvxFloat r3(v3, v7);
		dgAvxFloat::Transpose4x8(r0, r1, r2, r3);
		m_x[index] = r0;
		m_y[index] = r1;
		m_z[index] = r2;
	}

	DG_INLINE void Set (dgInt32 index, const dgAvxVector3_local& val)
	{
		m_x[index] = val.m_x;
		m_y[index] = val.m_y;
		m_z[index] = val.m_z;
	}

	DG_INLINE dgAvxVector3_local Get(dgInt32 index) const
	{
		return dgAvxVector3_local(m_x[index], m_y[index], m_z[index]);
	}

	dgAvxArray<dgAvxFloat> m_x;
	dgAvxArray<dgAvxFloat> m_y;
	dgAvxArray<dgAvxFloat> m_z;
};

class dgAvxMatrix3x3
{
	public:
	dgAvxMatrix3x3(dgMemoryAllocator* const allocator)
		:m_front(allocator)
		,m_up(allocator)
		,m_right(allocator)
	{
	}

	void Reserve (dgInt32 count)
	{
		m_front.Reserve(count);
		m_up.Reserve(count);
		m_right.Reserve(count);
	}

	DG_INLINE dgAvxMatrix3x3_local Get(dgInt32 index) const
	{
		return dgAvxMatrix3x3_local(m_front.Get(index), m_up.Get(index), m_right.Get(index));
	}

	DG_INLINE void Set(dgInt32 index, const dgAvxMatrix3x3_local& val)
	{
		m_front.Set(index, val.m_front);
		m_up.Set(index, val.m_up);
		m_right.Set(index, val.m_right);
	}

	dgAvxVector3 m_front;
	dgAvxVector3 m_up;
	dgAvxVector3 m_right;
};

#endif

