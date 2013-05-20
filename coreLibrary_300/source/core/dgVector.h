/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __dgVector__
#define __dgVector__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgMemory.h"
#include "dgSimd.h"


//#define DG_SIMD_VECTOR_CLASS

#define dgCheckVector(x) (dgCheckFloat(x[0]) && dgCheckFloat(x[1]) && dgCheckFloat(x[2]) && dgCheckFloat(x[3]))


template<class T>
class dgTemplateVector
{
	public:
	DG_INLINE dgTemplateVector () 
	{
	}
	
	DG_INLINE dgTemplateVector (const T* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (0.0f)
	{
		//	dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgTemplateVector (T x, T y, T z, T w) 
		:m_x(x), m_y(y), m_z(z), m_w (w)
	{
	}
	
	DG_INLINE T& operator[] (dgInt32 i)
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return (&m_x)[i];
	}	

	DG_INLINE const T& operator[] (dgInt32 i) const
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return (&m_x)[i];
	}

	DG_INLINE dgTemplateVector<T> Scale3 (T scale) const
	{
		return dgTemplateVector<T> (m_x * scale, m_y * scale, m_z * scale, m_w);
	}

	DG_INLINE dgTemplateVector<T> Scale4 (T scale) const
	{
		return dgTemplateVector<T> (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	DG_INLINE dgTemplateVector<T> operator+ (const dgTemplateVector<T> &B) const
	{
		return dgTemplateVector<T> (m_x + B.m_x, m_y + B.m_y, m_z + B.m_z, m_w);
	}

	DG_INLINE dgTemplateVector<T>& operator+= (const dgTemplateVector<T> &A) 
	{
		m_x += A.m_x;
		m_y += A.m_y;
		m_z += A.m_z;
		//	dgAssert (dgCheckVector ((*this)));
		return *this;
	}

	DG_INLINE dgTemplateVector<T> operator- (const dgTemplateVector<T> &A) const
	{
		return dgTemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w);
	}

	DG_INLINE dgTemplateVector<T>& operator-= (const dgTemplateVector<T> &A) 
	{
		m_x -= A.m_x;
		m_y -= A.m_y;
		m_z -= A.m_z;
		//	dgAssert (dgCheckVector ((*this)));
		return *this;
	}

	// return dot product
	DG_INLINE T operator% (const dgTemplateVector<T> &A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z;
	}

	// return cross product
	DG_INLINE dgTemplateVector<T> operator* (const dgTemplateVector<T> &B) const
	{
		return dgTemplateVector<T> (m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	DG_INLINE dgTemplateVector<T> Add4 (const dgTemplateVector &A) const
	{
		return dgTemplateVector<T> (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	DG_INLINE dgTemplateVector<T> Sub4 (const dgTemplateVector &A) const
	{
		return dgTemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	// return dot 4d dot product
	DG_INLINE T DotProduct4 (const dgTemplateVector &A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w;
	}

	
	DG_INLINE dgTemplateVector<T> CrossProduct4 (const dgTemplateVector &A, const dgTemplateVector &B) const
	{
		T cofactor[3][3];
		T array[4][4];

		const dgTemplateVector<T>& me = *this;
		for (dgInt32 i = 0; i < 4; i ++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = T (1.0f);
		}

		dgTemplateVector<T> normal;
		T sign = T (-1.0f);
		for (dgInt32 i = 0; i < 4; i ++)  {

			for (dgInt32 j = 0; j < 3; j ++) {
				dgInt32 k0 = 0;
				for (dgInt32 k = 0; k < 4; k ++) {
					if (k != i) {
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			T x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			T y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			T z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			T det = x + y + z;

			normal[i] = sign * det;
			sign *= T (-1.0f);
		}

		return normal;
	}

	// component wise multiplication
	DG_INLINE dgTemplateVector<T> CompProduct3 (const dgTemplateVector<T> &A) const
	{
		return dgTemplateVector<T> (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
	}

	// component wise 4d multiplication
	DG_INLINE dgTemplateVector<T> CompProduct4 (const dgTemplateVector<T> &A) const
	{
		return dgTemplateVector<T> (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}


	// check validity of floats
#ifdef _DEBUG
	void Trace () const
	{
		dgTrace (("%f %f %f %f\n", m_x, m_y, m_z, m_w));
	}
#endif

	DG_CLASS_ALLOCATOR(allocator)

	T m_x;
	T m_y;
	T m_z;
	T m_w;
};

class dgVector;

// *****************************************************************************************
//
// 256 bit double precision vector class declaration
//
// *****************************************************************************************
DG_MSC_VECTOR_ALIGMENT
class dgBigVector: public dgTemplateVector<dgFloat64>
{
	public:
	DG_INLINE dgBigVector()
		:dgTemplateVector<dgFloat64>()
	{
	}

	DG_INLINE dgBigVector (const dgVector& v)
		:dgTemplateVector<dgFloat64>(((dgTemplateVector<dgFloat32>&)v).m_x, ((dgTemplateVector<dgFloat32>&)v).m_y, ((dgTemplateVector<dgFloat32>&)v).m_z, ((dgTemplateVector<dgFloat32>&)v).m_w)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgBigVector (const dgTemplateVector<dgFloat64>& v)
		:dgTemplateVector<dgFloat64>(v)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgBigVector (const dgFloat32* const ptr)
		:dgTemplateVector<dgFloat64>(ptr[0], ptr[1], ptr[2], dgFloat64 (0.0f))
	{
		dgAssert (dgCheckVector ((*this)));
	}

#ifndef __USE_DOUBLE_PRECISION__
	DG_INLINE dgBigVector (const dgFloat64* const ptr)
		:dgTemplateVector<dgFloat64>(ptr)
	{
		dgAssert (dgCheckVector ((*this)));
	}
#endif

	DG_INLINE dgBigVector (dgFloat64 x, dgFloat64 y, dgFloat64 z, dgFloat64 w) 
		:dgTemplateVector<dgFloat64>(x, y, z, w)
	{
		dgAssert (dgCheckVector ((*this)));
	}

} DG_GCC_VECTOR_ALIGMENT;



// *****************************************************************************************
//
// 128 bit single precision vector class declaration
//
// *****************************************************************************************
#ifndef DG_SIMD_VECTOR_CLASS

DG_MSC_VECTOR_ALIGMENT
class dgVector: public dgTemplateVector<dgFloat32>
{
	public:
	DG_INLINE dgVector(const dgSimd& val)
#ifndef _MSC_VER
		// for GCC only
		:dgTemplateVector<dgFloat32>(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f))
#endif
	{
		dgAssert ((dgUnsigned64(this) & 0x0f) == 0);
		(dgSimd&) *this = val;
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector()
		:dgTemplateVector<dgFloat32>()
	{
	}

	DG_INLINE dgVector (const dgTemplateVector<dgFloat32>& v)
		:dgTemplateVector<dgFloat32>(v)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (const dgFloat32* const ptr)
		:dgTemplateVector<dgFloat32>(ptr)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (dgFloat32 x, dgFloat32 y, dgFloat32 z, dgFloat32 w) 
		:dgTemplateVector<dgFloat32>(x, y, z, w)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (const dgBigVector& copy)
		:dgTemplateVector<dgFloat32>(dgFloat32 (copy.m_x), dgFloat32 (copy.m_y), dgFloat32 (copy.m_z), dgFloat32 (copy.m_w))
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector dgVector::operator= (const dgSimd& val)
	{
		(dgSimd&)*this = val;
		return *this;
	}


	DG_INLINE dgFloat32 dgVector::DotProductSimd (const dgVector& A) const
	{
		dgFloat32 dot;
		dgSimd temp (((dgSimd&)*this).DotProduct((dgSimd&)A));
		temp.StoreScalar (&dot);
		return dot;
	}

	DG_INLINE dgVector dgVector::CrossProductSimd (const dgVector &e10) const
	{
		return ((dgSimd&)*this).CrossProduct((dgSimd&)e10);
	}

	DG_INLINE dgVector dgVector::CompProductSimd (const dgVector &A) const
	{
		return ((dgSimd&)*this) * (dgSimd&)A;
	}


	// logical operations
//	DG_INLINE dgVector operator& (const dgVector& data) const
//	{
//		return _mm_and_ps (m_type, data.m_type);	
//	}

//	DG_INLINE dgVector operator| (const dgVector& data) const
//	{
//		return _mm_or_ps (m_type, data.m_type);	
//	}

//	DG_INLINE dgVector AndNot (const dgVector& data) const
//	{
//		return _mm_andnot_ps (m_type, data.m_type);	
//	}
/*
	DG_INLINE dgVector MoveLow (const dgVector& data) const
	{
		return dgSimd (m_x, m_y, data.m_x, data.m_y); 
	}

	DG_INLINE dgVector MoveHigh (const dgVector& data) const
	{
		return dgSimd (data.m_z, data.m_w, m_z, m_w); 
	}

	DG_INLINE dgVector PackLow (const dgVector& data) const
	{
		return dgSimd (m_x, data.m_x, m_y, data.m_y); 
	}

	DG_INLINE dgVector PackHigh (const dgVector& data) const
	{
		return dgSimd (m_z, data.m_z, m_w, data.m_w); 
	}

	DG_INLINE static void Transpose4x4 (dgVector& dst0, dgVector& dst1, dgVector& dst2, dgVector& dst3, 
		const dgVector& src0, const dgVector& src1, const dgVector& src2, const dgVector& src3)
	{
		dgVector tmp0 (src0.PackLow(src1));
		dgVector tmp1 (src2.PackLow(src3));
		dgVector tmp2 (src0.PackHigh(src1));
		dgVector tmp3 (src2.PackHigh(src3));

		dst0 = tmp0.MoveLow (tmp1);
		dst1 = tmp1.MoveHigh (tmp0);
		dst2 = tmp2.MoveLow (tmp3);
		dst3 = tmp3.MoveHigh (tmp2);
	}
*/

	static dgVector m_wOne;
	static dgVector m_triplexMask;

}DG_GCC_VECTOR_ALIGMENT;


#else

DG_MSC_VECTOR_ALIGMENT
class dgVector
{
	protected:
	public:
	DG_INLINE dgVector() 
	{
	}

	DG_INLINE dgVector(const __m128 type)
		:m_type (type)
	{
	}

	DG_INLINE dgVector (const dgFloat32 a)
		: m_type(_mm_set_ps1(a)) 
	{
	}

	DG_INLINE dgVector(const dgSimd& val)
		:m_type (val.m_type)
	{
	}

	DG_INLINE dgVector (const dgFloat32* const ptr)
		:m_type(_mm_loadu_ps (ptr))
	{
		m_type = _mm_and_ps (m_type, m_triplexMask.m_type);
	}

	DG_INLINE dgVector (const dgBigVector& copy)
		:m_type(_mm_set_ps(dgFloat32 (copy.m_w), dgFloat32 (copy.m_z), dgFloat32 (copy.m_y), dgFloat32 (copy.m_x)))
	{
	}

	DG_INLINE dgVector (dgFloat32 x, dgFloat32 y, dgFloat32 z, dgFloat32 w)
		:m_type(_mm_set_ps(w, z, y, x))
	{
	}

	DG_INLINE dgVector (dgInt32 ix, dgInt32 iy, dgInt32 iz, dgInt32 iw)
		:m_type(_mm_set_ps(*(dgFloat32*)&iw, *(dgFloat32*)&iz, *(dgFloat32*)&iy, *(dgFloat32*)&ix))
	{
	}


	DG_INLINE dgVector operator= (const dgSimd& val)
	{
		m_type = val.m_type;
		return *this;
	}

//	DG_INLINE dgFloat32 DotProductSimd (const dgVector& A) const;
//	DG_INLINE dgVector CrossProductSimd (const dgVector &A) const;
//	DG_INLINE dgVector CompProductSimd (const dgVector &A) const;

	DG_INLINE dgVector Scale3 (dgFloat32 s) const
	{
		dgVector tmp (s, s, s, dgFloat32 (1.0f));
		return _mm_mul_ps (m_type, tmp.m_type);
	}

	DG_INLINE dgVector Scale4 (dgFloat32 s) const
	{
		return _mm_mul_ps (m_type, _mm_set_ps1(s));
	}


	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return m_f[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return m_f[i];
	}

	DG_INLINE dgVector operator+ (const dgVector &A) const
	{
//		dgVector tmp (A & m_triplexMask);
//		return _mm_add_ps (m_type, tmp.m_type);	
		return _mm_add_ps (m_type, A.m_type);	
	}

	DG_INLINE dgVector Add4 (const dgVector &A) const
	{
		return _mm_add_ps (m_type, A.m_type);	
	}

	DG_INLINE dgVector operator- (const dgVector &A) const 
	{
		dgVector tmp (A & m_triplexMask);
		return _mm_sub_ps (m_type, tmp.m_type);	
	}

	DG_INLINE dgVector Sub4 (const dgVector &A) const
	{
		return _mm_sub_ps (m_type, A.m_type);	
	}

	DG_INLINE dgVector &operator+= (const dgVector &A)
	{
		dgVector tmp (A & m_triplexMask);
		m_type = _mm_add_ps (m_type, tmp.m_type);
		return *this;
	}

	DG_INLINE dgVector &operator-= (const dgVector &A)
	{
		dgVector tmp (A & m_triplexMask);
		m_type = _mm_sub_ps (m_type, tmp.m_type);
		return *this;
	}

	// return dot product
	DG_INLINE dgFloat32 operator% (const dgVector &A) const
	{
		dgVector tmp (A & m_triplexMask);
		dgAssert ((m_w * tmp.m_w) == dgFloat32 (0.0f));
		return CompProduct4(tmp).AddHorizontal().m_x;
	}

	DG_INLINE dgFloat32 DotProduct4 (const dgVector &A) const
	{
		return CompProduct4(A).AddHorizontal().m_x;
	}

	// return cross product
	DG_INLINE dgVector operator* (const dgVector &B) const
	{
		return _mm_sub_ps (_mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 0, 2, 1)), _mm_shuffle_ps (B.m_type, B.m_type, PURMUT_MASK(3, 1, 0, 2))),
						   _mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 1, 0, 2)), _mm_shuffle_ps (B.m_type, B.m_type, PURMUT_MASK(3, 0, 2, 1))));

	}

	DG_INLINE dgVector CrossProduct4 (const dgVector &A, const dgVector &B) const
	{
		dgFloat32 cofactor[3][3];
		dgFloat32 array[4][4];

		const dgVector& me = *this;
		for (dgInt32 i = 0; i < 4; i ++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dgFloat32 (1.0f);
		}

		dgVector normal;
		dgFloat32  sign = dgFloat32 (-1.0f);
		for (dgInt32 i = 0; i < 4; i ++)  {

			for (dgInt32 j = 0; j < 3; j ++) {
				dgInt32 k0 = 0;
				for (dgInt32 k = 0; k < 4; k ++) {
					if (k != i) {
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			dgFloat32  x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dgFloat32  y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dgFloat32  z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dgFloat32  det = x + y + z;

			normal[i] = sign * det;
			sign *= dgFloat32 (-1.0f);
		}

		return normal;
	}

	// component wise multiplication
	DG_INLINE dgVector CompProduct3 (const dgVector &A) const
	{
		dgVector tmp ((A & m_triplexMask) | m_wOne);
		return _mm_mul_ps (m_type, tmp.m_type);
	}

	// component wise multiplication
	DG_INLINE dgVector CompProduct4 (const dgVector &A) const
	{
		return _mm_mul_ps (m_type, A.m_type);
	}

	DG_INLINE dgVector AddHorizontal () const
	{
		dgSimd tmp (_mm_hadd_ps (m_type, m_type));
		return _mm_hadd_ps (tmp.m_type, tmp.m_type);
	}

	// logical operations
	DG_INLINE dgVector operator& (const dgVector& data) const
	{
		return _mm_and_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector operator| (const dgVector& data) const
	{
		return _mm_or_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector AndNot (const dgVector& data) const
	{
		return _mm_andnot_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector MoveLow (const dgVector& data) const
	{
		return _mm_movelh_ps (m_type, data.m_type);
	}

	DG_INLINE dgVector MoveHigh (const dgVector& data) const
	{
		return _mm_movehl_ps (m_type, data.m_type);
	}

	DG_INLINE dgVector PackLow (const dgVector& data) const
	{
		return _mm_unpacklo_ps (m_type, data.m_type);
	}

	DG_INLINE dgVector PackHigh (const dgVector& data) const
	{
		return _mm_unpackhi_ps (m_type, data.m_type);
	}


	DG_INLINE static void Transpose4x4 (dgVector& dst0, dgVector& dst1, dgVector& dst2, dgVector& dst3, 
										const dgVector& src0, const dgVector& src1, const dgVector& src2, const dgVector& src3)
	{
		dgVector tmp0 (src0.PackLow(src1));
		dgVector tmp1 (src2.PackLow(src3));
		dgVector tmp2 (src0.PackHigh(src1));
		dgVector tmp3 (src2.PackHigh(src3));

		dst0 = tmp0.MoveLow (tmp1);
		dst1 = tmp1.MoveHigh (tmp0);
		dst2 = tmp2.MoveLow (tmp3);
		dst3 = tmp3.MoveHigh (tmp2);
	}


	DG_CLASS_ALLOCATOR(allocator)
	
	union {
		__m128 m_type;
		dgFloat32 m_f[4];
		struct {
			dgFloat32 m_x;
			dgFloat32 m_y;
			dgFloat32 m_z;
			dgFloat32 m_w;
		};
		struct {
			dgFloat32 m_ix;
			dgFloat32 m_iy;
			dgFloat32 m_iz;
			dgFloat32 m_iw;
		};
	};

	static dgVector m_wOne;
	static dgVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;



#endif





#endif

