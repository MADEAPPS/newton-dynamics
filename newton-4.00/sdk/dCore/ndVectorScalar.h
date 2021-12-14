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

#ifndef __ND_VECTOR_SCALAR_H__
#define __ND_VECTOR_SCALAR_H__

// *****************************************************************************************
//
// 4 x 1 single precision vector class declaration
//
// *****************************************************************************************
#ifdef D_NEWTON_USE_DOUBLE
	#define ndVector ndBigVector
#else

class ndBigVector;
class ndVector
{
	public:
	inline ndVector()
	{
	}

	inline ndVector(ndFloat32 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline ndVector (const ndVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline ndVector (const ndFloat32* const ptr)
		:m_x(ptr[0])
		,m_y(ptr[1])
		,m_z(ptr[2])
		,m_w(ptr[3])
	{
		dAssert (dCheckVector ((*this)));
	}

	// emulate gather instruction
	inline ndVector(const ndFloat32* const baseAddr, const ndInt32* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}

#ifndef	D_NEWTON_USE_DOUBLE
	inline ndVector(const ndFloat64* const ptr)
		:m_x(ndFloat32(ptr[0]))
		,m_y(ndFloat32(ptr[1]))
		,m_z(ndFloat32(ptr[2]))
		,m_w(ndFloat32(ptr[3]))
	{
	}
#endif


	inline ndVector (ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert (dCheckVector ((*this)));
	}

	inline ndVector (ndInt32 ix, ndInt32 iy, ndInt32 iz, ndInt32 iw)
		:m_x(*((ndFloat32*)&ix)), m_y(*((ndFloat32*)&iy)), m_z(*((ndFloat32*)&iz)), m_w(*((ndFloat32*)&iw))
	{
	}

#ifndef  D_NEWTON_USE_DOUBLE 
	inline ndVector (const ndBigVector& copy)
		:m_x(ndFloat32 (((ndFloat64*)&copy)[0])) 
		,m_y(ndFloat32 (((ndFloat64*)&copy)[1])) 
		,m_z(ndFloat32 (((ndFloat64*)&copy)[2])) 
		,m_w(ndFloat32 (((ndFloat64*)&copy)[3])) 
	{
		dAssert (dCheckVector ((*this)));
	}
#endif

	inline ndFloat32 GetScalar () const
	{
		return m_x;
	}

	inline void Store (ndFloat32* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	inline ndVector BroadcastX () const
	{
		return ndVector (m_x);
	}

	inline ndVector BroadcastY () const
	{
		return ndVector (m_y);
	}

	inline ndVector BroadcastZ () const
	{
		return ndVector (m_z);
	}

	inline ndVector BroadcastW () const
	{
		return ndVector (m_w);
	}


	inline ndFloat32& operator[] (ndInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline const ndFloat32& operator[] (ndInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline ndVector operator+ (const ndVector& A) const
	{
		return ndVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline ndVector operator- (const ndVector& A) const 
	{
		return ndVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline ndVector operator* (const ndVector& A) const
	{
		return ndVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline ndVector& operator+= (const ndVector& A)
	{
		return (*this = ndVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline ndVector& operator-= (const ndVector& A)
	{
		return (*this = ndVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline ndVector& operator*= (const ndVector& A)
	{
		return (*this = ndVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline ndVector MulAdd(const ndVector& A, const ndVector& B) const
	{
		return *this + A * B;
	}

	inline ndVector MulSub(const ndVector& A, const ndVector& B) const
	{
		return *this - A * B;
	}

	inline ndVector AddHorizontal () const
	{
		return ndVector (m_x + m_y + m_z + m_w);
	}

	inline ndVector Scale (ndFloat32 scale) const
	{
		return ndVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline ndVector CrossProduct (const ndVector& B) const
	{
		return ndVector (m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline ndVector CrossProduct (const ndVector& A, const ndVector& B) const
	{
		ndFloat32 cofactor[3][3];
		ndFloat32 array[4][4];

		const ndVector& me = *this;
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = ndFloat32 (1.0f);
		}

		ndVector normal;
		ndFloat32  sign = ndFloat32 (-1.0f);
		for (ndInt32 i = 0; i < 4; i ++)  
		{
			for (ndInt32 j = 0; j < 3; j ++) 
			{
				ndInt32 k0 = 0;
				for (ndInt32 k = 0; k < 4; k ++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			ndFloat32 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			ndFloat32 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			ndFloat32 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			ndFloat32 det = x + y + z;

			normal[i] = sign * det;
			sign *= ndFloat32 (-1.0f);
		}

		return normal;
	}

	inline ndVector GetInt () const
	{
		return ndVector (ndInt32 (ndFloor (m_x)), ndInt32(ndFloor (m_y)), ndInt32(ndFloor (m_z)), ndInt32 (ndFloor (m_w)));
	}

	inline ndVector TestZero() const
	{
		const ndInt32* const a = (ndInt32*)&m_x;
		return ndVector ((a[0] == 0) ? ndFloat32 (-1.0f) : ndFloat32 (1.0f),
						 (a[1] == 0) ? ndFloat32 (-1.0f) : ndFloat32 (1.0f),
						 (a[2] == 0) ? ndFloat32 (-1.0f) : ndFloat32 (1.0f),
						 (a[3] == 0) ? ndFloat32 (-1.0f) : ndFloat32 (1.0f));
	}


	inline ndVector Floor () const
	{
		return ndVector (ndFloor (m_x), ndFloor (m_y), ndFloor (m_z), ndFloor (m_w));
	}

	inline ndVector DotProduct (const ndVector &A) const
	{
		return ndVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline ndVector Reciproc () const
	{
		return ndVector (ndFloat32 (1.0f) / m_x, ndFloat32 (1.0f) / m_y, ndFloat32 (1.0f) / m_z, ndFloat32 (1.0f) / m_w);
	}

	inline ndVector Sqrt () const
	{
		return ndVector (ndSqrt (m_x), ndSqrt (m_y), ndSqrt (m_z), ndSqrt (m_w));
	}

	inline ndVector InvSqrt () const
	{
		return ndVector (ndRsqrt (m_x), ndRsqrt (m_y), ndRsqrt (m_z), ndRsqrt (m_w));
	}

	inline ndVector InvMagSqrt () const
	{
		return ndVector (ndRsqrt (DotProduct(*this).m_x));
	}

	inline ndVector Normalize () const
	{
		dAssert (m_w == ndFloat32 (0.0f));
		const ndVector& me = *this; 
		return me * InvMagSqrt();
	}

	ndVector Abs () const
	{
		return ndVector ((m_x > ndFloat32 (0.0f)) ? m_x : -m_x,
			(m_y > ndFloat32 (0.0f)) ? m_y : -m_y,
			(m_z > ndFloat32 (0.0f)) ? m_z : -m_z,
			(m_w > ndFloat32 (0.0f)) ? m_w : -m_w);
	}

	ndFloat32 GetMax () const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	ndVector GetMax (const ndVector& data) const
	{
		return ndVector ((m_x > data.m_x) ? m_x : data.m_x,
			(m_y > data.m_y) ? m_y : data.m_y,
			(m_z > data.m_z) ? m_z : data.m_z,
			(m_w > data.m_w) ? m_w : data.m_w);
	}

	ndVector GetMin (const ndVector& data) const
	{
		return ndVector ((m_x < data.m_x) ? m_x : data.m_x,
			(m_y < data.m_y) ? m_y : data.m_y,
			(m_z < data.m_z) ? m_z : data.m_z,
			(m_w < data.m_w) ? m_w : data.m_w);
	}


	// relational operators
	inline ndVector operator== (const ndVector& data) const
	{
		return ndVector ((m_x == data.m_x) ? ndInt32 (0xffffffff) : 0,
			(m_y == data.m_y) ? ndInt32 (0xffffffff) : 0,
			(m_z == data.m_z) ? ndInt32 (0xffffffff) : 0,
			(m_w == data.m_w) ? ndInt32 (0xffffffff) : 0);
	}

	inline ndVector operator> (const ndVector& data) const
	{
		return ndVector ((m_x > data.m_x) ? ndInt32 (0xffffffff) : 0,
			(m_y > data.m_y) ? ndInt32 (0xffffffff) : 0,
			(m_z > data.m_z) ? ndInt32 (0xffffffff) : 0,
			(m_w > data.m_w) ? ndInt32 (0xffffffff) : 0);
	}

	inline ndVector operator< (const ndVector& data) const
	{
		return ndVector ((m_x < data.m_x) ? ndInt32 (0xffffffff) : 0,
			(m_y < data.m_y) ? ndInt32 (0xffffffff) : 0,
			(m_z < data.m_z) ? ndInt32 (0xffffffff) : 0,
			(m_w < data.m_w) ? ndInt32 (0xffffffff) : 0);
	}

	inline ndVector operator>= (const ndVector& data) const
	{
		return ndVector ((m_x >= data.m_x) ? ndInt32 (0xffffffff) : 0, 
			(m_y >= data.m_y) ? ndInt32 (0xffffffff) : 0,
			(m_z >= data.m_z) ? ndInt32 (0xffffffff) : 0,
			(m_w >= data.m_w) ? ndInt32 (0xffffffff) : 0);
	}

	inline ndVector operator<= (const ndVector& data) const
	{
		return ndVector ((m_x <= data.m_x) ? ndInt32 (0xffffffff) : 0,
			(m_y <= data.m_y) ? ndInt32 (0xffffffff) : 0,
			(m_z <= data.m_z) ? ndInt32 (0xffffffff) : 0,
			(m_w <= data.m_w) ? ndInt32 (0xffffffff) : 0);
	}


	// logical operations
	inline ndVector operator& (const ndVector& data) const
	{
		const ndInt32* const a = (ndInt32*)&m_x;
		const ndInt32* const b = (ndInt32*)&data.m_x;
		return ndVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	inline ndVector operator| (const ndVector& data) const
	{
		const ndInt32* const a = (ndInt32*)&m_x;
		const ndInt32* const b = (ndInt32*)&data.m_x;
		return ndVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	inline ndVector operator^ (const ndVector& data) const
	{
		const ndInt32* const a = (ndInt32*)&m_x;
		const ndInt32* const b = (ndInt32*)&data.m_x;
		return ndVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	inline ndVector AndNot (const ndVector& data) const
	{
		const ndInt32* const a = (ndInt32*)&m_x;
		const ndInt32* const b = (ndInt32*)&data.m_x;
		return ndVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	inline ndVector Select (const ndVector& data, const ndVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	inline ndInt32 GetSignMask() const
	{
		const ndInt32* const a = (ndInt32*)&m_x;
		return (((a[0] & 0x80000000) ? 1 : 0) | ((a[1] & 0x80000000) ? 2 : 0) | ((a[2] & 0x80000000) ? 4 : 0) | ((a[3] & 0x80000000) ? 8 : 0));
	} 

	inline ndVector ShiftRight() const
	{
		return ndVector (m_w, m_x, m_y, m_z); 
	}

	inline ndVector ShiftTripleRight () const
	{
		return ndVector (m_z, m_x, m_y, m_w); 
	}

	inline ndVector ShiftTripleLeft () const
	{
		return ndVector (m_y, m_z, m_x, m_w); 
	}

	inline ndVector ShiftRightLogical (ndInt32 bits) const
	{
		return ndVector (ndInt32 (ndUnsigned32 (m_ix) >> bits), ndInt32 (ndUnsigned32 (m_iy) >> bits), ndInt32 (ndUnsigned32 (m_iz) >> bits), ndInt32 (ndUnsigned32 (m_iw) >> bits)); 
	}

	inline static void Transpose4x4 (ndVector& dst0, ndVector& dst1, ndVector& dst2, ndVector& dst3, const ndVector& src0, const ndVector& src1, const ndVector& src2, const ndVector& src3)
	{
		ndVector tmp0 (src0);
		ndVector tmp1 (src1);
		ndVector tmp2 (src2);
		ndVector tmp3 (src3);

		dst0 = ndVector (tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = ndVector (tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = ndVector (tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = ndVector (tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	union 
	{
		ndInt32 m_i[4];
		struct 
		{
			ndFloat32 m_x;
			ndFloat32 m_y;
			ndFloat32 m_z;
			ndFloat32 m_w;
		};
		struct 
		{
			ndInt32 m_ix;
			ndInt32 m_iy;
			ndInt32 m_iz;
			ndInt32 m_iw;
		};
	};

	D_CORE_API static ndVector m_zero;
	D_CORE_API static ndVector m_one;
	D_CORE_API static ndVector m_wOne;
	D_CORE_API static ndVector m_half;
	D_CORE_API static ndVector m_two;
	D_CORE_API static ndVector m_three;
	D_CORE_API static ndVector m_negOne;
	D_CORE_API static ndVector m_xMask;
	D_CORE_API static ndVector m_yMask;
	D_CORE_API static ndVector m_zMask;
	D_CORE_API static ndVector m_wMask;
	D_CORE_API static ndVector m_xyzwMask;
	D_CORE_API static ndVector m_epsilon;
	D_CORE_API static ndVector m_signMask;
	D_CORE_API static ndVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

class ndBigVector
{
	public:
	inline ndBigVector()
	{
	}

	inline ndBigVector(ndFloat64 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline ndBigVector (const ndBigVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

#ifndef D_NEWTON_USE_DOUBLE
	inline ndBigVector (const ndVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline ndBigVector (const ndFloat32* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (ndFloat32 (0.0f))
	{
		dAssert (dCheckVector ((*this)));
	}
#endif

	inline ndBigVector (const ndFloat64* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (ptr[3])
	{
		dAssert (dCheckVector ((*this)));
	}

	inline ndBigVector (ndFloat64 x, ndFloat64 y, ndFloat64 z, ndFloat64 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert (dCheckVector ((*this)));
	}

	inline ndBigVector (ndInt32 ix, ndInt32 iy, ndInt32 iz, ndInt32 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline ndBigVector (ndInt64 ix, ndInt64 iy, ndInt64 iz, ndInt64 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline ndFloat64 GetScalar () const
	{
		return m_x;
	}

	inline void Store (ndFloat64* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	inline ndBigVector BroadcastX () const
	{
		return ndBigVector (m_x);
	}

	inline ndBigVector BroadcastY () const
	{
		return ndBigVector (m_y);
	}

	inline ndBigVector BroadcastZ () const
	{
		return ndBigVector (m_z);
	}

	inline ndBigVector BroadcastW () const
	{
		return ndBigVector (m_w);
	}


	inline ndFloat64& operator[] (ndInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline const ndFloat64& operator[] (ndInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline ndBigVector operator+ (const ndBigVector& A) const
	{
		return ndBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline ndBigVector operator- (const ndBigVector& A) const 
	{
		return ndBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline ndBigVector operator* (const ndBigVector& A) const
	{
		return ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline ndBigVector& operator+= (const ndBigVector& A)
	{
		return (*this = ndBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline ndBigVector& operator-= (const ndBigVector& A)
	{
		return (*this = ndBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline ndBigVector& operator*= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline ndBigVector MulAdd(const ndBigVector& A, const ndBigVector& B) const
	{
		return *this + A * B;
	}

	inline ndBigVector MulSub(const ndVector& A, const ndBigVector& B) const
	{
		return *this - A * B;
	}


	inline ndBigVector AddHorizontal () const
	{
		return ndBigVector (m_x + m_y + m_z + m_w);
	}

	inline ndBigVector Scale (ndFloat64 scale) const
	{
		return ndBigVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline ndBigVector CrossProduct (const ndBigVector& B) const
	{
		return ndBigVector (m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline ndBigVector CrossProduct (const ndBigVector& A, const ndBigVector& B) const
	{
		ndFloat64 cofactor[3][3];
		ndFloat64 array[4][4];

		const ndBigVector& me = *this;
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = ndFloat32 (1.0f);
		}

		ndBigVector normal;
		ndFloat64  sign = ndFloat64 (-1.0f);
		for (ndInt32 i = 0; i < 4; i ++)  
		{
			for (ndInt32 j = 0; j < 3; j ++) 
			{
				ndInt32 k0 = 0;
				for (ndInt32 k = 0; k < 4; k ++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			ndFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			ndFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			ndFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			ndFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= ndFloat64 (-1.0f);
		}

		return normal;
	}

	inline ndBigVector GetInt () const
	{
		return ndBigVector (ndInt64 (floor (m_x)), ndInt64(floor (m_y)), ndInt64(floor (m_z)), ndInt64 (floor (m_w)));
	}

	inline ndBigVector TestZero() const
	{
		const ndInt64* const a = (ndInt64*)&m_x;
		return ndBigVector ((a[0] == 0) ? ndFloat64 (-1.0f) : ndFloat64 (1.0f),
							(a[1] == 0) ? ndFloat64 (-1.0f) : ndFloat64 (1.0f),
							(a[2] == 0) ? ndFloat64 (-1.0f) : ndFloat64 (1.0f),
							(a[3] == 0) ? ndFloat64 (-1.0f) : ndFloat64 (1.0f));
	}


	inline ndBigVector Floor () const
	{
		return ndBigVector (floor (m_x), floor (m_y), floor (m_z), floor (m_w));
	}

	inline ndBigVector DotProduct (const ndBigVector &A) const
	{
		return ndBigVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline ndBigVector Reciproc () const
	{
		return ndBigVector (ndFloat64 (1.0f) / m_x, ndFloat64 (1.0f) / m_y, ndFloat64 (1.0f) / m_z, ndFloat64 (1.0f) / m_w);
	}

	inline ndBigVector Sqrt () const
	{
		return ndBigVector (sqrt (m_x), sqrt (m_y), sqrt (m_z), sqrt (m_w));
	}

	inline ndBigVector InvSqrt () const
	{
		return ndBigVector (ndFloat64 (1.0f) / sqrt (m_x), ndFloat64 (1.0f) / sqrt (m_y), ndFloat64 (1.0f) / sqrt (m_z), ndFloat64 (1.0f) / sqrt (m_w));
	}

	inline ndBigVector InvMagSqrt () const
	{
		return ndBigVector (ndFloat64 (1.0f) / sqrt (DotProduct(*this).m_x));
	}

	inline ndBigVector Normalize() const
	{
		dAssert (m_w == ndFloat64 (0.0f));
		//const ndBigVector& me = *this;
		//return *this * ndBigVector (ndRsqrt(DotProduct(*this).m_x));
		return *this * InvMagSqrt();
	}

	ndBigVector Abs () const
	{
		return ndBigVector ((m_x > ndFloat64 (0.0f)) ? m_x : -m_x,
							(m_y > ndFloat64 (0.0f)) ? m_y : -m_y,
							(m_z > ndFloat64 (0.0f)) ? m_z : -m_z,
							(m_w > ndFloat64 (0.0f)) ? m_w : -m_w);
	}

	ndFloat64 GetMax () const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	ndBigVector GetMax (const ndBigVector& data) const
	{
		return ndBigVector ((m_x > data.m_x) ? m_x : data.m_x,
							(m_y > data.m_y) ? m_y : data.m_y,
							(m_z > data.m_z) ? m_z : data.m_z,
							(m_w > data.m_w) ? m_w : data.m_w);
					}

	ndBigVector GetMin (const ndBigVector& data) const
	{
		return ndBigVector ((m_x < data.m_x) ? m_x : data.m_x,
							(m_y < data.m_y) ? m_y : data.m_y,
							(m_z < data.m_z) ? m_z : data.m_z,
							(m_w < data.m_w) ? m_w : data.m_w);
	}

	// relational operators
	inline ndBigVector operator== (const ndBigVector& data) const
	{
		return ndBigVector ((m_x == data.m_x) ? ndInt64 (-1) : ndInt64 (0),
							(m_y == data.m_y) ? ndInt64 (-1) : ndInt64 (0),
							(m_z == data.m_z) ? ndInt64 (-1) : ndInt64 (0),
							(m_w == data.m_w) ? ndInt64 (-1) : ndInt64 (0));
	}

	inline ndBigVector operator> (const ndBigVector& data) const
	{
		return ndBigVector ((m_x > data.m_x) ? ndInt64 (-1) : ndInt64 (0),
							(m_y > data.m_y) ? ndInt64 (-1) : ndInt64 (0),
							(m_z > data.m_z) ? ndInt64 (-1) : ndInt64 (0),
							(m_w > data.m_w) ? ndInt64 (-1) : ndInt64 (0));
	}

	inline ndBigVector operator< (const ndBigVector& data) const
	{
		return ndBigVector ((m_x < data.m_x) ? ndInt64 (-1) : ndInt64 (0),
							(m_y < data.m_y) ? ndInt64 (-1) : ndInt64 (0),
							(m_z < data.m_z) ? ndInt64 (-1) : ndInt64 (0),
							(m_w < data.m_w) ? ndInt64 (-1) : ndInt64 (0));
	}

	inline ndBigVector operator>= (const ndBigVector& data) const
	{
		return ndBigVector ((m_x >= data.m_x) ? ndInt64 (-1) : ndInt64 (0), 
							(m_y >= data.m_y) ? ndInt64 (-1) : ndInt64 (0),
							(m_z >= data.m_z) ? ndInt64 (-1) : ndInt64 (0),
							(m_w >= data.m_w) ? ndInt64 (-1) : ndInt64 (0));
	}

	inline ndBigVector operator<= (const ndBigVector& data) const
	{
		return ndBigVector ((m_x <= data.m_x) ? ndInt64 (-1) : ndInt64 (0),
							(m_y <= data.m_y) ? ndInt64 (-1) : ndInt64 (0),
							(m_z <= data.m_z) ? ndInt64 (-1) : ndInt64 (0),
							(m_w <= data.m_w) ? ndInt64 (-1) : ndInt64 (0));
	}

	// logical operations
	inline ndBigVector operator& (const ndBigVector& data) const
	{
		const ndInt64* const a = (ndInt64*)&m_x;
		const ndInt64* const b = (ndInt64*)&data.m_x;
		return ndBigVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	inline ndBigVector operator| (const ndBigVector& data) const
	{
		const ndInt64* const a = (ndInt64*)&m_x;
		const ndInt64* const b = (ndInt64*)&data.m_x;
		return ndBigVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	inline ndBigVector operator^ (const ndBigVector& data) const
	{
		const ndInt64* const a = (ndInt64*)&m_x;
		const ndInt64* const b = (ndInt64*)&data.m_x;
		return ndBigVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	inline ndBigVector AndNot (const ndBigVector& data) const
	{
		const ndInt64* const a = (ndInt64*)&m_x;
		const ndInt64* const b = (ndInt64*)&data.m_x;
		return ndBigVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	inline ndBigVector Select(const ndBigVector& data, const ndBigVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	inline ndInt32 GetSignMask() const
	{
		const ndInt64* const a = (ndInt64*)&m_x;
		return (((a[0]>>63) ? 1 : 0) | ((a[1]>>63) ? 2 : 0) | ((a[2]>>63) ? 4 : 0) | ((a[3]>>63) ? 8 : 0));
	} 

	inline ndVector ShiftRight() const
	{
		return ndBigVector (m_w, m_x, m_y, m_z); 
	}

	inline ndBigVector ShiftTripleRight () const
	{
		return ndBigVector (m_z, m_x, m_y, m_w); 
	}

	inline ndBigVector ShiftTripleLeft () const
	{
		return ndBigVector (m_y, m_z, m_x, m_w); 
	}

	inline ndBigVector ShiftRightLogical (ndInt32 bits) const
	{
		return ndBigVector (ndInt64 (ndUnsigned64 (m_ix) >> bits), ndInt64 (ndUnsigned64 (m_iy) >> bits), ndInt64 (ndUnsigned64 (m_iz) >> bits), ndInt64 (ndUnsigned64 (m_iw) >> bits)); 
	}

	inline static void Transpose4x4 (ndBigVector& dst0, ndBigVector& dst1, ndBigVector& dst2, ndBigVector& dst3, const ndBigVector& src0, const ndBigVector& src1, const ndBigVector& src2, const ndBigVector& src3)
	{
		ndBigVector tmp0 (src0);
		ndBigVector tmp1 (src1);
		ndBigVector tmp2 (src2);
		ndBigVector tmp3 (src3);

		dst0 = ndBigVector (tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = ndBigVector (tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = ndBigVector (tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = ndBigVector (tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	union 
	{
		ndInt64 m_i[4];
		struct 
		{
			ndFloat64 m_x;
			ndFloat64 m_y;
			ndFloat64 m_z;
			ndFloat64 m_w;
		};
		struct 
		{
			ndInt64 m_ix;
			ndInt64 m_iy;
			ndInt64 m_iz;
			ndInt64 m_iw;
		};
	};

	D_CORE_API static ndBigVector m_zero;
	D_CORE_API static ndBigVector m_one;
	D_CORE_API static ndBigVector m_wOne;
	D_CORE_API static ndBigVector m_half;
	D_CORE_API static ndBigVector m_two;
	D_CORE_API static ndBigVector m_three;
	D_CORE_API static ndBigVector m_negOne;
	D_CORE_API static ndBigVector m_xMask;
	D_CORE_API static ndBigVector m_yMask;
	D_CORE_API static ndBigVector m_zMask;
	D_CORE_API static ndBigVector m_wMask;
	D_CORE_API static ndBigVector m_xyzwMask;
	D_CORE_API static ndBigVector m_epsilon;
	D_CORE_API static ndBigVector m_signMask;
	D_CORE_API static ndBigVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32;


#endif
