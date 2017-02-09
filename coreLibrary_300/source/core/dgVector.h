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

#ifndef __dgVector__
#define __dgVector__

#include "dgStdafx.h"
#include "dgTypes.h"
#include "dgDebug.h"
#include "dgMemory.h"

#define dgCheckVector(x) (dgCheckFloat(x[0]) && dgCheckFloat(x[1]) && dgCheckFloat(x[2]) && dgCheckFloat(x[3]))

template<class T>
class dgTemplateVector
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	DG_INLINE dgTemplateVector () 
	{
	}
	
	DG_INLINE dgTemplateVector (const T* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (T(0.0f))
	{
		//	dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgTemplateVector (const dgTemplateVector<T>& copy)
		:m_x(copy.m_x), m_y(copy.m_y), m_z(copy.m_z), m_w (copy.m_w)
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

	DG_INLINE dgTemplateVector<T> operator+ (const dgTemplateVector<T>& B) const
	{
		return dgTemplateVector<T> (m_x + B.m_x, m_y + B.m_y, m_z + B.m_z, m_w + B.m_w);
	}

	DG_INLINE dgTemplateVector<T>& operator+= (const dgTemplateVector<T>& A) 
	{
		return (*this = dgTemplateVector<T> (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	DG_INLINE dgTemplateVector<T> operator- (const dgTemplateVector<T>& A) const
	{
		return dgTemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	DG_INLINE dgTemplateVector<T>& operator-= (const dgTemplateVector<T>& A) 
	{
		return (*this = dgTemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	DG_INLINE dgTemplateVector<T> AddHorizontal() const
	{
		T val(m_x + m_y + m_z + m_w);
		return dgTemplateVector<T>(val, val, val, val);
	}

	// return dot product
	DG_INLINE T DotProduct3 (const dgTemplateVector<T>& A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z;
	}

	// return cross product
	DG_INLINE dgTemplateVector<T> CrossProduct3 (const dgTemplateVector<T>& B) const
	{
		return dgTemplateVector<T> (m_y * B.m_z - m_z * B.m_y,
									m_z * B.m_x - m_x * B.m_z,
									m_x * B.m_y - m_y * B.m_x, m_w);
	}

	// return dot 4d dot product
	DG_INLINE dgTemplateVector<T> DotProduct4 (const dgTemplateVector &A) const
	{
		T val (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
		return dgTemplateVector<T> (val, val, val, val);
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
	DG_INLINE dgTemplateVector<T> CompProduct3 (const dgTemplateVector<T>& A) const
	{
		return dgTemplateVector<T> (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
	}

	// component wise 4d multiplication
	DG_INLINE dgTemplateVector<T> CompProduct4 (const dgTemplateVector<T>& A) const
	{
		return dgTemplateVector<T> (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	DG_INLINE dgTemplateVector<T> GetMax(const dgTemplateVector<T>& data) const
	{
		return dgTemplateVector<T>((m_x > data.m_x) ? m_x : data.m_x, (m_y > data.m_y) ? m_y : data.m_y, (m_z > data.m_z) ? m_z : data.m_z,	(m_w > data.m_w) ? m_w : data.m_w);
	}

	DG_INLINE dgTemplateVector<T> GetMin(const dgTemplateVector<T>& data) const
	{
		return dgTemplateVector<T>((m_x < data.m_x) ? m_x : data.m_x, (m_y < data.m_y) ? m_y : data.m_y, (m_z < data.m_z) ? m_z : data.m_z,	(m_w < data.m_w) ? m_w : data.m_w);
	}

	// check validity of floats
#ifdef _DEBUG
	void Trace () const
	{
		dgTrace (("%f %f %f %f\n", m_x, m_y, m_z, m_w));
	}
#endif

	T m_x;
	T m_y;
	T m_z;
	T m_w;
};



#ifdef DG_SCALAR_VECTOR_CLASS


// *****************************************************************************************
//
// 4 x 1 single precision vector class declaration
//
// *****************************************************************************************
#ifdef _NEWTON_USE_DOUBLE
	#define dgVector dgBigVector
#else

class dgBigVector;
DG_MSC_VECTOR_ALIGMENT
class dgVector
{
	public:
	DG_INLINE dgVector()
	{
	}

	DG_INLINE dgVector(dgFloat32 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	DG_INLINE dgVector (const dgVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
		//dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (const dgFloat32* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (dgFloat32 (0.0f))
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (dgFloat32 x, dgFloat32 y, dgFloat32 z, dgFloat32 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (dgInt32 ix, dgInt32 iy, dgInt32 iz, dgInt32 iw)
		:m_x(*((dgFloat32*)&ix)), m_y(*((dgFloat32*)&iy)), m_z(*((dgFloat32*)&iz)), m_w(*((dgFloat32*)&iw))
	{
	}

#ifndef  _NEWTON_USE_DOUBLE 
	DG_INLINE dgVector (const dgBigVector& copy)
		//		:m_x(dgFloat32 (copy.m_x)), m_y(dgFloat32 (copy.m_y)), m_z(dgFloat32 (copy.m_z)), m_w(dgFloat32 (copy.m_w))
	{
		dgAssert (0);
		const dgFloat64* const ptr = (dgFloat64*) &copy;
		m_x = dgFloat32 (ptr[0]);
		m_y = dgFloat32 (ptr[1]);
		m_z = dgFloat32 (ptr[2]);
		m_w = dgFloat32 (ptr[3]);
		dgAssert (dgCheckVector ((*this)));
	}
#endif

	DG_INLINE dgFloat32 GetScalar () const
	{
		return m_x;
	}

	DG_INLINE void Store (dgFloat32* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	DG_INLINE dgVector BroadcastX () const
	{
		return dgVector (m_x);
	}

	DG_INLINE dgVector BroadcastY () const
	{
		return dgVector (m_y);
	}

	DG_INLINE dgVector BroadcastZ () const
	{
		return dgVector (m_z);
	}

	DG_INLINE dgVector BroadcastW () const
	{
		return dgVector (m_w);
	}


	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return (&m_x)[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return (&m_x)[i];
	}

	DG_INLINE dgVector operator+ (const dgVector& A) const
	{
		return dgVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	DG_INLINE dgVector operator- (const dgVector& A) const 
	{
		return dgVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	DG_INLINE dgVector &operator+= (const dgVector& A)
	{
		return (*this = dgVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	DG_INLINE dgVector &operator-= (const dgVector& A)
	{
		return (*this = dgVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	DG_INLINE dgVector AddHorizontal () const
	{
		return dgVector (m_x + m_y + m_z + m_w);
	}

	DG_INLINE dgVector Scale3 (dgFloat32 scale) const
	{
		return dgVector (m_x * scale, m_y * scale, m_z * scale, m_w);
	}

	DG_INLINE dgVector Scale4 (dgFloat32 scale) const
	{
		return dgVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// component wise multiplication
	DG_INLINE dgVector CompProduct3 (const dgVector& A) const
	{
		return dgVector (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
	}

	// component wise 4d multiplication
	DG_INLINE dgVector CompProduct4 (const dgVector& A) const
	{
		return dgVector (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}


	// return dot product
	DG_INLINE dgFloat32 DotProduct3 (const dgVector& A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z;
	}

	// return cross product
	DG_INLINE dgVector CrossProduct3 (const dgVector& B) const
	{
		return dgVector (m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	DG_INLINE dgVector CrossProduct4 (const dgVector& A, const dgVector& B) const
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
			dgFloat32 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dgFloat32 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dgFloat32 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dgFloat32 det = x + y + z;

			normal[i] = sign * det;
			sign *= dgFloat32 (-1.0f);
		}

		return normal;
	}

	DG_INLINE dgVector GetInt () const
	{
		return dgVector (dgInt32 (dgFloor (m_x)), dgInt32(dgFloor (m_y)), dgInt32(dgFloor (m_z)), dgInt32 (dgFloor (m_w)));
	}

	DG_INLINE dgVector TestZero() const
	{
		const dgInt32* const a = (dgInt32*)&m_x;
		return dgVector ((a[0] == 0) ? dgFloat32 (-1.0f) : dgFloat32 (1.0f),
			(a[1] == 0) ? dgFloat32 (-1.0f) : dgFloat32 (1.0f),
			(a[2] == 0) ? dgFloat32 (-1.0f) : dgFloat32 (1.0f),
			(a[3] == 0) ? dgFloat32 (-1.0f) : dgFloat32 (1.0f));
	}


	DG_INLINE dgVector Floor () const
	{
		return dgVector (dgFloor (m_x), dgFloor (m_y), dgFloor (m_z), dgFloor (m_w));
	}

	DG_INLINE dgVector DotProduct4 (const dgVector &A) const
	{
		return dgVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	DG_INLINE dgVector InvMagSqrt () const
	{
		return dgVector (dgRsqrt (DotProduct4(*this).m_x));
	}

	DG_INLINE dgVector Reciproc () const
	{
		return dgVector (dgFloat32 (1.0f) / m_x, dgFloat32 (1.0f) / m_y, dgFloat32 (1.0f) / m_z, dgFloat32 (1.0f) / m_w);
	}

	DG_INLINE dgVector Sqrt () const
	{
		return dgVector (dgSqrt (m_x), dgSqrt (m_y), dgSqrt (m_z), dgSqrt (m_w));
	}

	DG_INLINE dgVector InvSqrt () const
	{
		return dgVector (dgRsqrt (m_x), dgRsqrt (m_y), dgRsqrt (m_z), dgRsqrt (m_w));
	}

	dgVector Abs () const
	{
		return dgVector ((m_x > dgFloat32 (0.0f)) ? m_x : -m_x,
			(m_y > dgFloat32 (0.0f)) ? m_y : -m_y,
			(m_z > dgFloat32 (0.0f)) ? m_z : -m_z,
			(m_w > dgFloat32 (0.0f)) ? m_w : -m_w);
	}

	dgVector GetMax (const dgVector& data) const
	{
		return dgVector ((m_x > data.m_x) ? m_x : data.m_x,
			(m_y > data.m_y) ? m_y : data.m_y,
			(m_z > data.m_z) ? m_z : data.m_z,
			(m_w > data.m_w) ? m_w : data.m_w);
	}

	dgVector GetMin (const dgVector& data) const
	{
		return dgVector ((m_x < data.m_x) ? m_x : data.m_x,
			(m_y < data.m_y) ? m_y : data.m_y,
			(m_z < data.m_z) ? m_z : data.m_z,
			(m_w < data.m_w) ? m_w : data.m_w);
	}


	// relational operators
	DG_INLINE dgVector operator== (const dgVector& data) const
	{
		return dgVector ((m_x == data.m_x) ? dgInt32 (0xffffffff) : 0,
			(m_y == data.m_y) ? dgInt32 (0xffffffff) : 0,
			(m_z == data.m_z) ? dgInt32 (0xffffffff) : 0,
			(m_w == data.m_w) ? dgInt32 (0xffffffff) : 0);
	}

	DG_INLINE dgVector operator> (const dgVector& data) const
	{
		return dgVector ((m_x > data.m_x) ? dgInt32 (0xffffffff) : 0,
			(m_y > data.m_y) ? dgInt32 (0xffffffff) : 0,
			(m_z > data.m_z) ? dgInt32 (0xffffffff) : 0,
			(m_w > data.m_w) ? dgInt32 (0xffffffff) : 0);
	}

	DG_INLINE dgVector operator< (const dgVector& data) const
	{
		return dgVector ((m_x < data.m_x) ? dgInt32 (0xffffffff) : 0,
			(m_y < data.m_y) ? dgInt32 (0xffffffff) : 0,
			(m_z < data.m_z) ? dgInt32 (0xffffffff) : 0,
			(m_w < data.m_w) ? dgInt32 (0xffffffff) : 0);
	}

	DG_INLINE dgVector operator>= (const dgVector& data) const
	{
		return dgVector ((m_x >= data.m_x) ? dgInt32 (0xffffffff) : 0, 
			(m_y >= data.m_y) ? dgInt32 (0xffffffff) : 0,
			(m_z >= data.m_z) ? dgInt32 (0xffffffff) : 0,
			(m_w >= data.m_w) ? dgInt32 (0xffffffff) : 0);
	}

	DG_INLINE dgVector operator<= (const dgVector& data) const
	{
		return dgVector ((m_x <= data.m_x) ? dgInt32 (0xffffffff) : 0,
			(m_y <= data.m_y) ? dgInt32 (0xffffffff) : 0,
			(m_z <= data.m_z) ? dgInt32 (0xffffffff) : 0,
			(m_w <= data.m_w) ? dgInt32 (0xffffffff) : 0);
	}


	// logical operations
	DG_INLINE dgVector operator& (const dgVector& data) const
	{
		const dgInt32* const a = (dgInt32*)&m_x;
		const dgInt32* const b = (dgInt32*)&data.m_x;
		return dgVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	DG_INLINE dgVector operator| (const dgVector& data) const
	{
		const dgInt32* const a = (dgInt32*)&m_x;
		const dgInt32* const b = (dgInt32*)&data.m_x;
		return dgVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	DG_INLINE dgVector operator^ (const dgVector& data) const
	{
		const dgInt32* const a = (dgInt32*)&m_x;
		const dgInt32* const b = (dgInt32*)&data.m_x;
		return dgVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	DG_INLINE dgVector AndNot (const dgVector& data) const
	{
		const dgInt32* const a = (dgInt32*)&m_x;
		const dgInt32* const b = (dgInt32*)&data.m_x;
		return dgVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	DG_INLINE dgInt32 GetSignMask() const
	{
		const dgInt32* const a = (dgInt32*)&m_x;
		return (((a[0] & 0x80000000) ? 1 : 0) | ((a[1] & 0x80000000) ? 2 : 0) | ((a[2] & 0x80000000) ? 4 : 0) | ((a[3] & 0x80000000) ? 8 : 0));
	} 

	DG_INLINE dgVector ShiftTripleRight () const
	{
		return dgVector (m_z, m_x, m_y, m_w); 
	}

	DG_INLINE dgVector ShiftTripleLeft () const
	{
		return dgVector (m_y, m_z, m_x, m_w); 
	}

	DG_INLINE dgVector ShiftRightLogical (int bits) const
	{
		return dgVector (dgInt32 (dgUnsigned32 (m_ix) >> bits), dgInt32 (dgUnsigned32 (m_iy) >> bits), dgInt32 (dgUnsigned32 (m_iz) >> bits), dgInt32 (dgUnsigned32 (m_iw) >> bits)); 
	}

	DG_INLINE static void Transpose4x4 (dgVector& dst0, dgVector& dst1, dgVector& dst2, dgVector& dst3, const dgVector& src0, const dgVector& src1, const dgVector& src2, const dgVector& src3)
	{
		dgVector tmp0 (src0);
		dgVector tmp1 (src1);
		dgVector tmp2 (src2);
		dgVector tmp3 (src3);

		dst0 = dgVector (tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = dgVector (tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = dgVector (tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = dgVector (tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	DG_CLASS_ALLOCATOR(allocator)

	union {
		struct {
			dgFloat32 m_x;
			dgFloat32 m_y;
			dgFloat32 m_z;
			dgFloat32 m_w;
		};
		struct {
			dgInt32 m_ix;
			dgInt32 m_iy;
			dgInt32 m_iz;
			dgInt32 m_iw;
		};
	};

	static dgVector m_zero;
	static dgVector m_one;
	static dgVector m_wOne;
	static dgVector m_half;
	static dgVector m_two;
	static dgVector m_three;
	static dgVector m_negOne;
	static dgVector m_xMask;
	static dgVector m_yMask;
	static dgVector m_zMask;
	static dgVector m_wMask;
	static dgVector m_signMask;
	static dgVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;

#endif

DG_MSC_VECTOR_ALIGMENT
class dgBigVector
{
	public:
	DG_INLINE dgBigVector()
	{
	}

	DG_INLINE dgBigVector(dgFloat64 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	DG_INLINE dgBigVector (const dgBigVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

#ifndef _NEWTON_USE_DOUBLE
	DG_INLINE dgBigVector (const dgVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	DG_INLINE dgBigVector (const dgFloat32* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (dgFloat32 (0.0f))
	{
		dgAssert (dgCheckVector ((*this)));
	}
#endif

	DG_INLINE dgBigVector (const dgFloat64* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (dgFloat32 (0.0f))
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgBigVector (dgFloat64 x, dgFloat64 y, dgFloat64 z, dgFloat64 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgBigVector (dgInt32 ix, dgInt32 iy, dgInt32 iz, dgInt32 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	DG_INLINE dgBigVector (dgInt64 ix, dgInt64 iy, dgInt64 iz, dgInt64 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	DG_INLINE dgFloat64 GetScalar () const
	{
		return m_x;
	}

	DG_INLINE void Store (dgFloat64* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	DG_INLINE dgBigVector BroadcastX () const
	{
		return dgBigVector (m_x);
	}

	DG_INLINE dgBigVector BroadcastY () const
	{
		return dgBigVector (m_y);
	}

	DG_INLINE dgBigVector BroadcastZ () const
	{
		return dgBigVector (m_z);
	}

	DG_INLINE dgBigVector BroadcastW () const
	{
		return dgBigVector (m_w);
	}


	DG_INLINE dgFloat64& operator[] (dgInt32 i)
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return (&m_x)[i];
	}

	DG_INLINE const dgFloat64& operator[] (dgInt32 i) const
	{
		dgAssert (i < 4);
		dgAssert (i >= 0);
		return (&m_x)[i];
	}

	DG_INLINE dgBigVector operator+ (const dgBigVector& A) const
	{
		return dgBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	DG_INLINE dgBigVector operator- (const dgBigVector& A) const 
	{
		return dgBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	DG_INLINE dgBigVector &operator+= (const dgBigVector& A)
	{
		return (*this = dgBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	DG_INLINE dgBigVector &operator-= (const dgBigVector& A)
	{
		return (*this = dgBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	DG_INLINE dgBigVector AddHorizontal () const
	{
		return dgBigVector (m_x + m_y + m_z + m_w);
	}

	DG_INLINE dgBigVector Scale3 (dgFloat64 scale) const
	{
		return dgBigVector (m_x * scale, m_y * scale, m_z * scale, m_w);
	}

	DG_INLINE dgBigVector Scale4 (dgFloat64 scale) const
	{
		return dgBigVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// component wise multiplication
	DG_INLINE dgBigVector CompProduct3 (const dgBigVector& A) const
	{
		return dgBigVector (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
	}

	// component wise 4d multiplication
	DG_INLINE dgBigVector CompProduct4 (const dgBigVector& A) const
	{
		return dgBigVector (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}


	// return dot product
	DG_INLINE dgFloat64 DotProduct3 (const dgBigVector& A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z;
	}

	// return cross product
	DG_INLINE dgBigVector CrossProduct3 (const dgBigVector& B) const
	{
		return dgBigVector (m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	DG_INLINE dgBigVector CrossProduct4 (const dgBigVector& A, const dgBigVector& B) const
	{
		dgFloat64 cofactor[3][3];
		dgFloat64 array[4][4];

		const dgBigVector& me = *this;
		for (dgInt32 i = 0; i < 4; i ++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dgFloat32 (1.0f);
		}

		dgBigVector normal;
		dgFloat64  sign = dgFloat64 (-1.0f);
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
			dgFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dgFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dgFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dgFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= dgFloat64 (-1.0f);
		}

		return normal;
	}

	DG_INLINE dgBigVector GetInt () const
	{
		return dgBigVector (dgInt64 (floor (m_x)), dgInt64(floor (m_y)), dgInt64(floor (m_z)), dgInt64 (floor (m_w)));
	}

	DG_INLINE dgBigVector TestZero() const
	{
		const dgInt64* const a = (dgInt64*)&m_x;
		return dgBigVector ((a[0] == 0) ? dgFloat64 (-1.0f) : dgFloat64 (1.0f),
			(a[1] == 0) ? dgFloat64 (-1.0f) : dgFloat64 (1.0f),
			(a[2] == 0) ? dgFloat64 (-1.0f) : dgFloat64 (1.0f),
			(a[3] == 0) ? dgFloat64 (-1.0f) : dgFloat64 (1.0f));
	}


	DG_INLINE dgBigVector Floor () const
	{
		return dgBigVector (floor (m_x), floor (m_y), floor (m_z), floor (m_w));
	}

	DG_INLINE dgBigVector DotProduct4 (const dgBigVector &A) const
	{
		return dgBigVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	DG_INLINE dgBigVector InvMagSqrt () const
	{
		return dgBigVector (dgFloat64 (1.0f) / sqrt (DotProduct4(*this).m_x));
	}

	DG_INLINE dgBigVector Reciproc () const
	{
		return dgBigVector (dgFloat64 (1.0f) / m_x, dgFloat64 (1.0f) / m_y, dgFloat64 (1.0f) / m_z, dgFloat64 (1.0f) / m_w);
	}

	DG_INLINE dgBigVector Sqrt () const
	{
		return dgBigVector (sqrt (m_x), sqrt (m_y), sqrt (m_z), sqrt (m_w));
	}

	DG_INLINE dgBigVector InvSqrt () const
	{
		return dgBigVector (dgFloat64 (1.0f) / sqrt (m_x), dgFloat64 (1.0f) / sqrt (m_y), dgFloat64 (1.0f) / sqrt (m_z), dgFloat64 (1.0f) / sqrt (m_w));
	}

	dgBigVector Abs () const
	{
		return dgBigVector ((m_x > dgFloat64 (0.0f)) ? m_x : -m_x,
			(m_y > dgFloat64 (0.0f)) ? m_y : -m_y,
			(m_z > dgFloat64 (0.0f)) ? m_z : -m_z,
			(m_w > dgFloat64 (0.0f)) ? m_w : -m_w);
	}

	dgBigVector GetMax (const dgBigVector& data) const
	{
		return dgBigVector ((m_x > data.m_x) ? m_x : data.m_x,
							(m_y > data.m_y) ? m_y : data.m_y,
							(m_z > data.m_z) ? m_z : data.m_z,
							(m_w > data.m_w) ? m_w : data.m_w);
					}

	dgBigVector GetMin (const dgBigVector& data) const
	{
		return dgBigVector ((m_x < data.m_x) ? m_x : data.m_x,
							(m_y < data.m_y) ? m_y : data.m_y,
							(m_z < data.m_z) ? m_z : data.m_z,
							(m_w < data.m_w) ? m_w : data.m_w);
	}

	// relational operators
	DG_INLINE dgBigVector operator== (const dgBigVector& data) const
	{
		return dgBigVector ((m_x == data.m_x) ? dgInt64 (-1) : dgInt64 (0),
							(m_y == data.m_y) ? dgInt64 (-1) : dgInt64 (0),
							(m_z == data.m_z) ? dgInt64 (-1) : dgInt64 (0),
							(m_w == data.m_w) ? dgInt64 (-1) : dgInt64 (0));
	}

	DG_INLINE dgBigVector operator> (const dgBigVector& data) const
	{
		return dgBigVector ((m_x > data.m_x) ? dgInt64 (-1) : dgInt64 (0),
							(m_y > data.m_y) ? dgInt64 (-1) : dgInt64 (0),
							(m_z > data.m_z) ? dgInt64 (-1) : dgInt64 (0),
							(m_w > data.m_w) ? dgInt64 (-1) : dgInt64 (0));
	}

	DG_INLINE dgBigVector operator< (const dgBigVector& data) const
	{
		return dgBigVector ((m_x < data.m_x) ? dgInt64 (-1) : dgInt64 (0),
							(m_y < data.m_y) ? dgInt64 (-1) : dgInt64 (0),
							(m_z < data.m_z) ? dgInt64 (-1) : dgInt64 (0),
							(m_w < data.m_w) ? dgInt64 (-1) : dgInt64 (0));
	}

	DG_INLINE dgBigVector operator>= (const dgBigVector& data) const
	{
		return dgBigVector ((m_x >= data.m_x) ? dgInt64 (-1) : dgInt64 (0), 
							(m_y >= data.m_y) ? dgInt64 (-1) : dgInt64 (0),
							(m_z >= data.m_z) ? dgInt64 (-1) : dgInt64 (0),
							(m_w >= data.m_w) ? dgInt64 (-1) : dgInt64 (0));
	}

	DG_INLINE dgBigVector operator<= (const dgBigVector& data) const
	{
		return dgBigVector ((m_x <= data.m_x) ? dgInt64 (-1) : dgInt64 (0),
							(m_y <= data.m_y) ? dgInt64 (-1) : dgInt64 (0),
							(m_z <= data.m_z) ? dgInt64 (-1) : dgInt64 (0),
							(m_w <= data.m_w) ? dgInt64 (-1) : dgInt64 (0));
	}


	// logical operations
	DG_INLINE dgBigVector operator& (const dgBigVector& data) const
	{
		const dgInt64* const a = (dgInt64*)&m_x;
		const dgInt64* const b = (dgInt64*)&data.m_x;
		return dgBigVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	DG_INLINE dgBigVector operator| (const dgBigVector& data) const
	{
		const dgInt64* const a = (dgInt64*)&m_x;
		const dgInt64* const b = (dgInt64*)&data.m_x;
		return dgBigVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	DG_INLINE dgBigVector operator^ (const dgBigVector& data) const
	{
		const dgInt64* const a = (dgInt64*)&m_x;
		const dgInt64* const b = (dgInt64*)&data.m_x;
		return dgBigVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	DG_INLINE dgBigVector AndNot (const dgBigVector& data) const
	{
		const dgInt64* const a = (dgInt64*)&m_x;
		const dgInt64* const b = (dgInt64*)&data.m_x;
		return dgBigVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	DG_INLINE dgInt32 GetSignMask() const
	{
		const dgInt64* const a = (dgInt64*)&m_x;
		return (((a[0]>>63) ? 1 : 0) | ((a[1]>>63) ? 2 : 0) | ((a[2]>>63) ? 4 : 0) | ((a[3]>>63) ? 8 : 0));
	} 

	DG_INLINE dgBigVector ShiftTripleRight () const
	{
		return dgBigVector (m_z, m_x, m_y, m_w); 
	}

	DG_INLINE dgBigVector ShiftTripleLeft () const
	{
		return dgBigVector (m_y, m_z, m_x, m_w); 
	}

	DG_INLINE dgBigVector ShiftRightLogical (int bits) const
	{
		return dgBigVector (dgInt64 (dgUnsigned64 (m_ix) >> bits), dgInt64 (dgUnsigned64 (m_iy) >> bits), dgInt64 (dgUnsigned64 (m_iz) >> bits), dgInt64 (dgUnsigned64 (m_iw) >> bits)); 
	}

	DG_INLINE static void Transpose4x4 (dgBigVector& dst0, dgBigVector& dst1, dgBigVector& dst2, dgBigVector& dst3, const dgBigVector& src0, const dgBigVector& src1, const dgBigVector& src2, const dgBigVector& src3)
	{
		dgBigVector tmp0 (src0);
		dgBigVector tmp1 (src1);
		dgBigVector tmp2 (src2);
		dgBigVector tmp3 (src3);

		dst0 = dgBigVector (tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = dgBigVector (tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = dgBigVector (tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = dgBigVector (tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	DG_CLASS_ALLOCATOR(allocator)

	union 
	{
		struct 
		{
			dgFloat64 m_x;
			dgFloat64 m_y;
			dgFloat64 m_z;
			dgFloat64 m_w;
		};
		struct 
		{
			dgInt64 m_ix;
			dgInt64 m_iy;
			dgInt64 m_iz;
			dgInt64 m_iw;
		};
	};

	static dgBigVector m_zero;
	static dgBigVector m_one;
	static dgBigVector m_wOne;
	static dgBigVector m_half;
	static dgBigVector m_two;
	static dgBigVector m_three;
	static dgBigVector m_negOne;
	static dgBigVector m_xMask;
	static dgBigVector m_yMask;
	static dgBigVector m_zMask;
	static dgBigVector m_wMask;
	static dgBigVector m_signMask;
	static dgBigVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;






#else

#ifdef _NEWTON_USE_DOUBLE
	#define dgVector dgBigVector
#else

class dgBigVector;
// *****************************************************************************************
//
// 4 x 1 single precision SSE vector class declaration
//
// *****************************************************************************************
DG_MSC_VECTOR_ALIGMENT
class dgVector
{
	#define PURMUT_MASK(w, z, y, x)		_MM_SHUFFLE (w, z, y, x)
	public:
	DG_INLINE dgVector() 
	{
	}

	DG_INLINE dgVector(const __m128 type)
		:m_type (type)
	{
	}

	DG_INLINE dgVector(const __m128i type)
		:m_typeInt (type)
	{
	}

	DG_INLINE dgVector (const dgFloat32 a)
		: m_type(_mm_set_ps1(a)) 
	{
	}

	DG_INLINE dgVector (const dgFloat32* const ptr)
		:m_type(_mm_loadu_ps (ptr))
	{
		m_type = _mm_and_ps (m_type, m_triplexMask.m_type);
	}

	DG_INLINE dgVector (const dgVector& copy)
		:m_type(copy.m_type)
	{
	}

	DG_INLINE dgVector (const dgBigVector& copy)
		:m_type(_mm_shuffle_ps (_mm_cvtpd_ps (((__m128d*)&copy)[0]), _mm_cvtpd_ps (((__m128d*)&copy)[1]), PURMUT_MASK(1, 0, 1, 0)))
	{
		dgAssert (dgCheckVector ((*this)));
	}

	DG_INLINE dgVector (dgFloat32 x, dgFloat32 y, dgFloat32 z, dgFloat32 w)
		:m_type(_mm_set_ps(w, z, y, x))
	{
	}

	DG_INLINE dgVector (dgInt32 ix, dgInt32 iy, dgInt32 iz, dgInt32 iw)
		:m_type(_mm_set_ps(*(dgFloat32*)&iw, *(dgFloat32*)&iz, *(dgFloat32*)&iy, *(dgFloat32*)&ix))
	{
	}


	DG_INLINE dgFloat32 GetScalar () const
	{
		//dgFloat32 scalar;
		//_mm_store_ss(&scalar, m_type);
		return m_x;
	}

	DG_INLINE void Store (dgFloat32* const dst) const
	{
		_mm_storeu_ps(dst, m_type);
	}

	DG_INLINE dgVector BroadcastX () const
	{
		return _mm_shuffle_ps (m_type, m_type, PURMUT_MASK(0, 0, 0, 0));
	}

	DG_INLINE dgVector BroadcastY () const
	{
		return _mm_shuffle_ps (m_type, m_type, PURMUT_MASK(1, 1, 1, 1));
	}

	DG_INLINE dgVector BroadcastZ () const
	{
		return _mm_shuffle_ps (m_type, m_type, PURMUT_MASK(2, 2, 2, 2));
	}

	DG_INLINE dgVector BroadcastW () const
	{
		return _mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 3, 3, 3));
	}

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

	DG_INLINE dgVector operator+ (const dgVector& A) const
	{
		return _mm_add_ps (m_type, A.m_type);	
	}

	DG_INLINE dgVector operator- (const dgVector& A) const 
	{
		return _mm_sub_ps (m_type, A.m_type);	
	}

	DG_INLINE dgVector &operator+= (const dgVector& A)
	{
		return (*this = _mm_add_ps (m_type, A.m_type));
	}

	DG_INLINE dgVector &operator-= (const dgVector& A)
	{
		return (*this = _mm_sub_ps (m_type, A.m_type));
	}

	// return dot product
	DG_INLINE dgFloat32 DotProduct3 (const dgVector& A) const
	{
		#ifdef DG_SSE4_INSTRUCTIONS_SET 
			return dgVector (_mm_dp_ps (m_type, A.m_type, 0x77)).GetScalar(); 
		#else
			dgVector tmp (A & m_triplexMask);
			dgAssert ((m_w * tmp.m_w) == dgFloat32 (0.0f));
			return CompProduct4(tmp).AddHorizontal().GetScalar();
		#endif
	}

	// return cross product
	DG_INLINE dgVector CrossProduct3 (const dgVector& B) const
	{
		return _mm_sub_ps (_mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 0, 2, 1)), _mm_shuffle_ps (B.m_type, B.m_type, PURMUT_MASK(3, 1, 0, 2))),
			_mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 1, 0, 2)), _mm_shuffle_ps (B.m_type, B.m_type, PURMUT_MASK(3, 0, 2, 1))));
	}

	DG_INLINE dgVector DotProduct4(const dgVector& A) const
	{
		#ifdef DG_SSE4_INSTRUCTIONS_SET 
			return _mm_dp_ps(m_type, A.m_type, 0xff);
		#else 
			return CompProduct4(A).AddHorizontal();
		#endif
	}

	DG_INLINE dgVector CrossProduct4 (const dgVector& A, const dgVector& B) const
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
	DG_INLINE dgVector CompProduct3 (const dgVector& A) const
	{
		dgVector tmp ((A & m_triplexMask) | m_wOne);
		return _mm_mul_ps (m_type, tmp.m_type);
	}

	DG_INLINE dgVector Reciproc () const
	{
		return _mm_div_ps (m_one.m_type, m_type);
	}

	// component wise multiplication
	DG_INLINE dgVector CompProduct4 (const dgVector& A) const
	{
		return _mm_mul_ps (m_type, A.m_type);
	}

	DG_INLINE dgVector AddHorizontal () const
	{
		dgVector tmp (_mm_hadd_ps (m_type, m_type));
		return _mm_hadd_ps (tmp.m_type, tmp.m_type);
	}

	DG_INLINE dgVector Abs () const
	{
		return _mm_and_ps (m_type, m_signMask.m_type);
	}

	dgVector GetMax (const dgVector& data) const
	{
		return _mm_max_ps (m_type, data.m_type);
	}

	dgVector GetMin (const dgVector& data) const
	{
		return _mm_min_ps (m_type, data.m_type);
	}

	DG_INLINE dgVector GetInt () const
	{
		return dgVector(_mm_cvtps_epi32(Floor().m_type));
	}

	DG_INLINE dgVector TestZero() const
	{
		return dgVector (_mm_cmpeq_epi32 (m_typeInt, m_zero.m_typeInt)) & m_negOne;
	}

	DG_INLINE dgVector Floor () const
	{
		#ifdef DG_SSE4_INSTRUCTIONS_SET
			return _mm_floor_ps(m_type);
		#else
			dgVector truncated (_mm_cvtepi32_ps (_mm_cvttps_epi32 (m_type)));
			dgVector ret (truncated - (dgVector::m_one & (*this < truncated)));
			dgAssert (ret.m_f[0] == dgFloor(m_f[0]));
			dgAssert (ret.m_f[1] == dgFloor(m_f[1]));
			dgAssert (ret.m_f[2] == dgFloor(m_f[2]));
			dgAssert (ret.m_f[3] == dgFloor(m_f[3]));
			return ret;
		#endif
	}

	DG_INLINE dgVector Sqrt () const
	{
		return dgVector (_mm_sqrt_ps(m_type));
	}

	DG_INLINE dgVector InvSqrt () const
	{
		dgVector tmp0 (_mm_rsqrt_ps(m_type));
		return m_half.CompProduct4(tmp0).CompProduct4((m_three - CompProduct4(tmp0).CompProduct4(tmp0)));
	}

	DG_INLINE dgVector InvMagSqrt () const
	{
		return DotProduct4(*this).InvSqrt();
	}

	// relational operators
	DG_INLINE dgVector operator> (const dgVector& data) const
	{
		return _mm_cmpgt_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector operator== (const dgVector& data) const
	{
		return _mm_cmpeq_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector operator< (const dgVector& data) const
	{
		return _mm_cmplt_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector operator>= (const dgVector& data) const
	{
		return _mm_cmpge_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector operator<= (const dgVector& data) const
	{
		return _mm_cmple_ps (m_type, data.m_type);	
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

	DG_INLINE dgVector operator^ (const dgVector& data) const
	{
		return _mm_xor_ps (m_type, data.m_type);	
	}

	DG_INLINE dgVector AndNot (const dgVector& data) const
	{
		return _mm_andnot_ps (data.m_type, m_type);	
	}

	DG_INLINE dgInt32 GetSignMask() const
	{
		return _mm_movemask_ps(m_type);
	} 

	DG_INLINE dgVector ShiftTripleRight () const
	{
		return _mm_shuffle_ps(m_type, m_type, PURMUT_MASK(3, 1, 0, 2));
	}

	DG_INLINE dgVector ShiftTripleLeft () const
	{
		return _mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 0, 2, 1));
	}

	DG_INLINE dgVector ShiftRightLogical (int bits) const
	{
		return dgVector (_mm_srli_epi32(m_typeInt, bits)); 
	}

	DG_INLINE static void Transpose4x4 (dgVector& dst0, dgVector& dst1, dgVector& dst2, dgVector& dst3, const dgVector& src0, const dgVector& src1, const dgVector& src2, const dgVector& src3)
	{
		__m128 tmp0 (_mm_unpacklo_ps (src0.m_type, src1.m_type));
		__m128 tmp1 (_mm_unpacklo_ps (src2.m_type, src3.m_type));
		__m128 tmp2 (_mm_unpackhi_ps (src0.m_type, src1.m_type));
		__m128 tmp3 (_mm_unpackhi_ps (src2.m_type, src3.m_type));

		dst0 = dgVector (_mm_movelh_ps (tmp0, tmp1));
		dst1 = dgVector (_mm_movehl_ps (tmp1, tmp0));
		dst2 = dgVector (_mm_movelh_ps (tmp2, tmp3));
		dst3 = dgVector (_mm_movehl_ps (tmp3, tmp2));
	}

	DG_CLASS_ALLOCATOR(allocator)

	union {
		dgFloat32 m_f[4];
		__m128 m_type;
		__m128i m_typeInt;
		struct {
			dgFloat32 m_x;
			dgFloat32 m_y;
			dgFloat32 m_z;
			dgFloat32 m_w;
		};
		struct {
			dgInt32 m_ix;
			dgInt32 m_iy;
			dgInt32 m_iz;
			dgInt32 m_iw;
		};
	};

	static dgVector m_zero;
	static dgVector m_one;
	static dgVector m_wOne;
	static dgVector m_two;
	static dgVector m_half;
	static dgVector m_three;
	static dgVector m_negOne;
	static dgVector m_xMask;
	static dgVector m_yMask;
	static dgVector m_zMask;
	static dgVector m_wMask;
	static dgVector m_signMask;
	static dgVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;
#endif




// *****************************************************************************************
//
// 4 x 1 double precision SSE2 vector class declaration
//
// *****************************************************************************************
DG_MSC_VECTOR_ALIGMENT
class dgBigVector
{
	#define PURMUT_MASK_DOUBLE(y, x)	_MM_SHUFFLE2 (y, x)

	public:
	DG_INLINE dgBigVector()
	{
	}

	DG_INLINE dgBigVector(const dgFloat64 a)
		:m_typeLow(_mm_set1_pd(a))
		,m_typeHigh(_mm_set1_pd(a))
	{
	}


#ifdef _NEWTON_USE_DOUBLE
	DG_INLINE dgBigVector(const dgVector& v)
		:m_typeLow(v.m_typeLow)
		,m_typeHigh(v.m_typeHigh)
	{
		//dgAssert(dgCheckVector((*this)));
	}

	DG_INLINE dgBigVector (const dgFloat32* const ptr)
		:m_typeLow(_mm_loadu_pd(ptr))
		,m_typeHigh(_mm_set_pd(dgFloat64(0.0f), ptr[2]))
	{
		dgAssert (dgCheckVector ((*this)));
	}
#else

	DG_INLINE dgBigVector(const dgVector& v)
		:m_typeLow(_mm_cvtps_pd (v.m_type))
		,m_typeHigh(_mm_cvtps_pd (_mm_shuffle_ps (v.m_type, v.m_type, PURMUT_MASK(3, 2, 3, 2))))
	{
		dgAssert(dgCheckVector((*this)));
	}

	DG_INLINE dgBigVector(const dgFloat64* const ptr)
		:m_typeLow(_mm_loadu_pd(ptr))
		,m_typeHigh(_mm_set_pd(dgFloat64(0.0f), ptr[2]))
	{
	}
#endif

	DG_INLINE dgBigVector(const __m128d typeLow, const __m128d typeHigh)
		:m_typeLow(typeLow)
		,m_typeHigh(typeHigh)
	{
	}

	DG_INLINE dgBigVector(dgFloat64 x, dgFloat64 y, dgFloat64 z, dgFloat64 w)
		: m_typeLow(_mm_set_pd(y, x))
		, m_typeHigh(_mm_set_pd(w, z))
	{
	}

	DG_INLINE dgBigVector(const __m128i typeLow, const __m128i typeHigh)
		: m_typeIntLow(typeLow)
		, m_typeIntHigh(typeHigh)
	{
	}

	DG_INLINE dgBigVector(dgInt32 ix, dgInt32 iy, dgInt32 iz, dgInt32 iw)
	{
		dgInt64 x = ix;
		dgInt64 y = iy;
		dgInt64 z = iz;
		dgInt64 w = iw;
		m_typeLow = _mm_set_pd(*(dgFloat32*)&y, *(dgFloat32*)&x);
		m_typeHigh = _mm_set_pd(*(dgFloat32*)&w, *(dgFloat32*)&z);
	}

	DG_INLINE dgFloat64& operator[] (dgInt32 i)
	{
		dgAssert(i < 4);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE const dgFloat64& operator[] (dgInt32 i) const
	{
		dgAssert(i < 4);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE dgFloat64 GetScalar() const
	{
		return m_x;
	}

	DG_INLINE dgBigVector operator+ (const dgBigVector& A) const
	{
		return dgBigVector(_mm_add_pd(m_typeLow, A.m_typeLow), _mm_add_pd(m_typeHigh, A.m_typeHigh));
	}

	DG_INLINE dgBigVector operator- (const dgBigVector& A) const
	{
		return dgBigVector(_mm_sub_pd(m_typeLow, A.m_typeLow), _mm_sub_pd(m_typeHigh, A.m_typeHigh));
	}

	DG_INLINE dgBigVector &operator+= (const dgBigVector& A)
	{
		m_typeLow = _mm_add_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_add_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	DG_INLINE dgBigVector &operator-= (const dgBigVector& A)
	{
		m_typeLow = _mm_sub_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_sub_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	DG_INLINE dgFloat64 DotProduct3(const dgBigVector& A) const
	{
		dgFloat64 ret;
		__m128d tmp0(_mm_mul_pd(m_typeLow, A.m_typeLow));
		__m128d tmp1(_mm_and_pd(m_typeHigh, dgBigVector::m_triplexMask.m_typeHigh));
		__m128d tmp2(_mm_mul_pd(tmp1, A.m_typeHigh));
		__m128d tmp3(_mm_add_pd(tmp0, tmp2));
		__m128d dot(_mm_hadd_pd(tmp3, tmp3));
		_mm_store_sd(&ret, dot);
		return ret;
	}

	// return cross product
	DG_INLINE dgBigVector CrossProduct3(const dgBigVector& B) const
	{
		dgBigVector tmp0(ShiftTripleLeft());
		dgBigVector tmp1(B.ShiftTripleRight());
		dgBigVector tmp2(ShiftTripleRight());
		dgBigVector tmp3(B.ShiftTripleLeft());
		return tmp0.CompProduct4(tmp1) - tmp2.CompProduct4(tmp3);
	}

	DG_INLINE dgBigVector DotProduct4(const dgBigVector& A) const
	{
		__m128d tmp0(_mm_mul_pd(m_typeLow, A.m_typeLow));
		__m128d tmp1(_mm_mul_pd(m_typeHigh, A.m_typeHigh));
		__m128d tmp2(_mm_add_pd(tmp0, tmp1));
		__m128d dot(_mm_hadd_pd(tmp2, tmp2));
		return dgBigVector(dot, dot);
	}

	DG_INLINE dgBigVector AddHorizontal() const
	{
		__m128d tmp0(_mm_add_pd(m_typeHigh, m_typeLow));
		__m128d tmp1(_mm_hadd_pd(tmp0, tmp0));
		return dgBigVector(tmp1, tmp1);
	}

	// component wise multiplication
	DG_INLINE dgBigVector CompProduct3(const dgBigVector& A) const
	{
		return dgBigVector(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_and_pd(_mm_mul_pd(m_typeHigh, A.m_typeHigh), m_triplexMask.m_typeHigh));
	}

	// component wide multiplication
	DG_INLINE dgBigVector CompProduct4(const dgBigVector& A) const
	{
		return dgBigVector(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
	}

	DG_INLINE dgBigVector BroadcastX() const
	{
		return dgBigVector(m_x);
	}

	DG_INLINE dgBigVector BroadcastY() const
	{
		return dgBigVector(m_y);
	}

	DG_INLINE dgBigVector BroadcastZ() const
	{
		return dgBigVector(m_z);
	}

	DG_INLINE dgBigVector BroadcastW() const
	{
		return dgBigVector(m_w);
	}

	DG_INLINE dgBigVector Scale3(dgFloat64 s) const
	{
		__m128d tmp0(_mm_set1_pd(s));
		__m128d tmp1(_mm_set_pd(dgFloat32(1.0f), s));
		return dgBigVector(_mm_mul_pd(m_typeLow, tmp0), _mm_mul_pd(m_typeHigh, tmp1));
	}

	DG_INLINE dgBigVector Scale4(dgFloat64 s) const
	{
		__m128d tmp0(_mm_set1_pd(s));
		return dgBigVector(_mm_mul_pd(m_typeLow, tmp0), _mm_mul_pd(m_typeHigh, tmp0));
	}

	DG_INLINE dgBigVector Abs() const
	{
		//		return _mm_and_ps (m_type, m_signMask.m_type);
		return dgBigVector(_mm_and_pd(m_typeLow, m_signMask.m_typeLow), _mm_and_pd(m_typeHigh, m_signMask.m_typeLow));
	}

	DG_INLINE dgBigVector Reciproc() const
	{
		return dgBigVector(_mm_div_pd(m_one.m_typeLow, m_typeLow), _mm_div_pd(m_one.m_typeHigh, m_typeHigh));
	}

	DG_INLINE dgBigVector Sqrt() const
	{
		return dgBigVector(_mm_sqrt_pd(m_typeLow), _mm_sqrt_pd(m_typeHigh));
	}

	DG_INLINE dgBigVector InvSqrt() const
	{
		return Sqrt().Reciproc();
	}

	DG_INLINE dgBigVector InvMagSqrt() const
	{
		return (DotProduct4(*this)).InvSqrt();
	}

	dgBigVector GetMax(const dgBigVector& data) const
	{
		return dgBigVector(_mm_max_pd(m_typeLow, data.m_typeLow), _mm_max_pd(m_typeHigh, data.m_typeHigh));
	}

	dgBigVector GetMin(const dgBigVector& data) const
	{
		return dgBigVector(_mm_min_pd(m_typeLow, data.m_typeLow), _mm_min_pd(m_typeHigh, data.m_typeHigh));
	}


	DG_INLINE dgBigVector GetInt() const
	{
		dgBigVector temp(Floor());
		dgInt64 x = _mm_cvtsd_si32(temp.m_typeLow);
		dgInt64 y = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeLow, temp.m_typeLow, PURMUT_MASK_DOUBLE(1, 1)));
		dgInt64 z = _mm_cvtsd_si32(temp.m_typeHigh);
		dgInt64 w = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeHigh, temp.m_typeHigh, PURMUT_MASK_DOUBLE(1, 1)));
		return dgBigVector(_mm_set_pd(*(dgFloat32*)&y, *(dgFloat32*)&x), _mm_set_pd(*(dgFloat32*)&w, *(dgFloat32*)&z));
	}

	// relational operators
	DG_INLINE dgBigVector operator> (const dgBigVector& data) const
	{
		return dgBigVector(_mm_cmpgt_pd(m_typeLow, data.m_typeLow), _mm_cmpgt_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector operator== (const dgBigVector& data) const
	{
		return dgBigVector(_mm_cmpeq_pd(m_typeLow, data.m_typeLow), _mm_cmpeq_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector operator< (const dgBigVector& data) const
	{
		return dgBigVector(_mm_cmplt_pd(m_typeLow, data.m_typeLow), _mm_cmplt_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector operator>= (const dgBigVector& data) const
	{
		return dgBigVector(_mm_cmpge_pd(m_typeLow, data.m_typeLow), _mm_cmpge_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector operator<= (const dgBigVector& data) const
	{
		return dgBigVector(_mm_cmple_pd(m_typeLow, data.m_typeLow), _mm_cmple_pd(m_typeHigh, data.m_typeHigh));
	}

	// logical operations
	DG_INLINE dgBigVector operator& (const dgBigVector& data) const
	{
		return dgBigVector(_mm_and_pd(m_typeLow, data.m_typeLow), _mm_and_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector operator| (const dgBigVector& data) const
	{
		return dgBigVector(_mm_or_pd(m_typeLow, data.m_typeLow), _mm_or_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector operator^ (const dgBigVector& data) const
	{
		return dgBigVector(_mm_xor_pd(m_typeLow, data.m_typeLow), _mm_xor_pd(m_typeHigh, data.m_typeHigh));
	}

	DG_INLINE dgBigVector AndNot(const dgBigVector& data) const
	{
		return dgBigVector(_mm_andnot_pd(data.m_typeLow, m_typeLow), _mm_andnot_pd(data.m_typeHigh, m_typeHigh));
	}

	DG_INLINE dgBigVector ShiftTripleRight() const
	{
		return dgBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PURMUT_MASK_DOUBLE(0, 0)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PURMUT_MASK_DOUBLE(1, 1)));
	}

	DG_INLINE dgBigVector ShiftTripleLeft() const
	{
		return dgBigVector(_mm_shuffle_pd(m_typeLow, m_typeHigh, PURMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PURMUT_MASK_DOUBLE(1, 0)));
	}

	DG_INLINE dgBigVector ShiftRightLogical(int bits) const
	{
		return dgBigVector(_mm_srli_epi64(m_typeIntLow, bits), _mm_srli_epi64(m_typeIntHigh, bits));
	}


	DG_INLINE dgInt32 GetSignMask() const
	{
		return _mm_movemask_pd(m_typeLow) | (_mm_movemask_pd(m_typeHigh) << 2);
	}

	DG_INLINE dgBigVector Floor() const
	{
#ifdef DG_SSE4_INSTRUCTIONS_SET
		return dgBigVector(_mm_floor_pd(m_typeLow), _mm_floor_pd(m_typeHigh));
#else 
		return dgBigVector(floor(m_x), floor(m_y), floor(m_z), floor(m_w));
#endif
	}

	DG_INLINE dgBigVector CrossProduct4(const dgBigVector& A, const dgBigVector& B) const
	{
		dgFloat64 array[4][4];
		dgFloat64 cofactor[3][3];

		const dgBigVector& me = *this;
		for (dgInt32 i = 0; i < 4; i++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dgFloat64(1.0f);
		}

		dgBigVector normal;
		dgFloat64 sign = dgFloat32(-1.0f);
		for (dgInt32 i = 0; i < 4; i++) {
			for (dgInt32 j = 0; j < 3; j++) {
				dgInt32 k0 = 0;
				for (dgInt32 k = 0; k < 4; k++) {
					if (k != i) {
						cofactor[j][k0] = array[j][k];
						k0++;
					}
				}
			}
			dgFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dgFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dgFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dgFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= dgFloat64(-1.0f);
		}

		return normal;
	}

	DG_INLINE dgBigVector TestZero() const
	{
		return dgBigVector(_mm_cmpeq_epi64(m_typeIntLow, m_zero.m_typeIntLow), _mm_cmpeq_epi64(m_typeIntHigh, m_zero.m_typeIntHigh)) & m_negOne;
	}

	DG_INLINE static void Transpose4x4(dgBigVector& dst0, dgBigVector& dst1, dgBigVector& dst2, dgBigVector& dst3,
		const dgBigVector& src0, const dgBigVector& src1, const dgBigVector& src2, const dgBigVector& src3)
	{
		dgBigVector tmp0(src0);
		dgBigVector tmp1(src1);
		dgBigVector tmp2(src2);
		dgBigVector tmp3(src3);

		dst0 = dgBigVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = dgBigVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = dgBigVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = dgBigVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	DG_CLASS_ALLOCATOR(allocator)

	union
	{
		dgFloat64 m_f[4];
		struct
		{
			__m128d m_typeLow;
			__m128d m_typeHigh;
		};
		struct
		{
			__m128i m_typeIntLow;
			__m128i m_typeIntHigh;
		};
		struct
		{
			dgFloat64 m_x;
			dgFloat64 m_y;
			dgFloat64 m_z;
			dgFloat64 m_w;
		};
		struct
		{
			dgInt64 m_ix;
			dgInt64 m_iy;
			dgInt64 m_iz;
			dgInt64 m_iw;
		};
	};

	static dgBigVector m_zero;
	static dgBigVector m_one;
	static dgBigVector m_wOne;
	static dgBigVector m_two;
	static dgBigVector m_half;
	static dgBigVector m_three;
	static dgBigVector m_negOne;
	static dgBigVector m_xMask;
	static dgBigVector m_yMask;
	static dgBigVector m_zMask;
	static dgBigVector m_wMask;
	static dgBigVector m_signMask;
	static dgBigVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;


#endif



/*
DG_MSC_VECTOR_ALIGMENT
class dgSpatialVector
{
	public:
	DG_INLINE dgSpatialVector()
	{
		SetZero();
	}

	DG_INLINE dgSpatialVector(const dgVector& low, const dgVector& high)
	{
		m_v[0] = low[0];
		m_v[1] = low[1];
		m_v[2] = low[2];
		m_v[3] = high[0];
		m_v[4] = high[1];
		m_v[5] = high[2];
	}

	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert(i < 6);
		dgAssert(i >= 0);
		return m_v[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i < 6);
		dgAssert(i >= 0);
		return m_v[i];
	}

	DG_INLINE void SetZero()
	{
		m_v[0] = dgFloat32(0.0f);
		m_v[1] = dgFloat32(0.0f);
		m_v[2] = dgFloat32(0.0f);
		m_v[3] = dgFloat32(0.0f);
		m_v[4] = dgFloat32(0.0f);
		m_v[5] = dgFloat32(0.0f);
	}

	DG_INLINE dgFloat32 DotProduct(const dgSpatialVector& v) const
	{
		return m_v[0] * v[0] + m_v[1] * v[1] + m_v[2] * v[2] + m_v[3] * v[3] + m_v[4] * v[4] + m_v[5] * v[5];
	}

	DG_INLINE void Scale(dgFloat32 s, dgSpatialVector& dst) const
	{
		dst.m_v[0] = m_v[0] * s;
		dst.m_v[1] = m_v[1] * s;
		dst.m_v[2] = m_v[2] * s;
		dst.m_v[3] = m_v[3] * s;
		dst.m_v[4] = m_v[4] * s;
		dst.m_v[5] = m_v[5] * s;
	}

	DG_INLINE void ScaleAdd(dgFloat32 s, const dgSpatialVector& b, dgSpatialVector& dst) const
	{
		dst.m_v[0] = b[0] + m_v[0] * s;
		dst.m_v[1] = b[1] + m_v[1] * s;
		dst.m_v[2] = b[2] + m_v[2] * s;
		dst.m_v[3] = b[3] + m_v[3] * s;
		dst.m_v[4] = b[4] + m_v[4] * s;
		dst.m_v[5] = b[5] + m_v[5] * s;
	}

	dgFloat32 m_v[6];
} DG_GCC_VECTOR_ALIGMENT;
*/



// *****************************************************************************************
//
// 6 x 1 vector class declaration
//
// *****************************************************************************************
DG_MSC_VECTOR_ALIGMENT
class dgSpatialVector
{
	public:
	DG_INLINE dgSpatialVector()
	{
	}

	DG_INLINE dgSpatialVector(const dgFloat32 a)
		:m_l(a)
		,m_h(a)
	{
	}

	DG_INLINE dgSpatialVector(const dgVector& low, const dgVector& high, int x)
		:m_l(low)
		,m_h(dgVector::m_zero)
	{
		m_l[3] = high[0];
		m_h[0] = high[1];
		m_h[1] = high[2];
	}

	DG_INLINE dgSpatialVector(const dgVector& low, const dgVector& high)
		:m_l(low)
		,m_h(high)
	{
	}

	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert(i < 6);
		dgAssert(i >= 0);
		return (&m_l.m_x)[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i < 6);
		dgAssert(i >= 0);
		return (&m_l.m_x)[i];
	}

	DG_INLINE dgSpatialVector operator+ (const dgSpatialVector& A) const
	{
		return dgSpatialVector(m_l + A.m_l, m_h + A.m_h);
	}

	DG_INLINE dgFloat32 DotProduct(const dgSpatialVector& v) const
	{
		dgAssert (v.m_h[2] == dgFloat32 (0.0f));
		dgAssert (v.m_h[3] == dgFloat32 (0.0f));
		dgVector p (m_l.CompProduct4(v.m_l) + m_h.CompProduct4(v.m_h));
		return (p.AddHorizontal()).GetScalar(); 
	}

	DG_INLINE dgSpatialVector Scale(dgFloat32 s) const
	{
		return dgSpatialVector(m_l.Scale4(s), m_h.Scale4(s));
	}

	dgVector m_l;
	dgVector m_h;

} DG_GCC_VECTOR_ALIGMENT;


#endif
