/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_VECTOR_SCALAR_H__
#define __D_VECTOR_SCALAR_H__

// *****************************************************************************************
//
// 4 x 1 single precision vector class declaration
//
// *****************************************************************************************
#ifdef D_NEWTON_USE_DOUBLE
	#define dVector dBigVector
#else

class dBigVector;
class dVector
{
	public:
	D_INLINE dVector()
	{
	}

	D_INLINE dVector(dFloat32 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	D_INLINE dVector (const dVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
		//dAssert (dCheckVector ((*this)));
	}

	D_INLINE dVector (const dFloat32* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (ptr[3])
	{
		dAssert (dCheckVector ((*this)));
	}

#ifndef	D_NEWTON_USE_DOUBLE
	D_INLINE dVector(const dFloat64* const ptr)
		:m_x(dFloat32(ptr[0]))
		,m_y(dFloat32(ptr[1]))
		,m_z(dFloat32(ptr[2]))
		,m_w(dFloat32(ptr[3]))
	{
	}
#endif


	D_INLINE dVector (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert (dCheckVector ((*this)));
	}

	D_INLINE dVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_x(*((dFloat32*)&ix)), m_y(*((dFloat32*)&iy)), m_z(*((dFloat32*)&iz)), m_w(*((dFloat32*)&iw))
	{
	}

#ifndef  D_NEWTON_USE_DOUBLE 
	D_INLINE dVector (const dBigVector& copy)
		:m_x(dFloat32 (((dFloat64*)&copy)[0])) 
		,m_y(dFloat32 (((dFloat64*)&copy)[1])) 
		,m_z(dFloat32 (((dFloat64*)&copy)[2])) 
		,m_w(dFloat32 (((dFloat64*)&copy)[3])) 
	{
		dAssert (dCheckVector ((*this)));
	}
#endif

	D_INLINE dFloat32 GetScalar () const
	{
		return m_x;
	}

	D_INLINE void Store (dFloat32* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	D_INLINE dVector BroadcastX () const
	{
		return dVector (m_x);
	}

	D_INLINE dVector BroadcastY () const
	{
		return dVector (m_y);
	}

	D_INLINE dVector BroadcastZ () const
	{
		return dVector (m_z);
	}

	D_INLINE dVector BroadcastW () const
	{
		return dVector (m_w);
	}


	D_INLINE dFloat32& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	D_INLINE const dFloat32& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	D_INLINE dVector operator+ (const dVector& A) const
	{
		return dVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	D_INLINE dVector operator- (const dVector& A) const 
	{
		return dVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	D_INLINE dVector operator* (const dVector& A) const
	{
		return dVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	D_INLINE dVector& operator+= (const dVector& A)
	{
		return (*this = dVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	D_INLINE dVector& operator-= (const dVector& A)
	{
		return (*this = dVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	D_INLINE dVector& operator*= (const dVector& A)
	{
		return (*this = dVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	D_INLINE dVector MulAdd(const dVector& A, const dVector& B) const
	{
		return *this + A * B;
	}

	D_INLINE dVector MulSub(const dVector& A, const dVector& B) const
	{
		return *this - A * B;
	}

	D_INLINE dVector AddHorizontal () const
	{
		return dVector (m_x + m_y + m_z + m_w);
	}

	D_INLINE dVector Scale (dFloat32 scale) const
	{
		return dVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	D_INLINE dVector CrossProduct (const dVector& B) const
	{
		return dVector (m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	D_INLINE dVector CrossProduct (const dVector& A, const dVector& B) const
	{
		dFloat32 cofactor[3][3];
		dFloat32 array[4][4];

		const dVector& me = *this;
		for (dInt32 i = 0; i < 4; i ++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dFloat32 (1.0f);
		}

		dVector normal;
		dFloat32  sign = dFloat32 (-1.0f);
		for (dInt32 i = 0; i < 4; i ++)  
		{
			for (dInt32 j = 0; j < 3; j ++) 
			{
				dInt32 k0 = 0;
				for (dInt32 k = 0; k < 4; k ++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			dFloat32 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dFloat32 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dFloat32 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dFloat32 det = x + y + z;

			normal[i] = sign * det;
			sign *= dFloat32 (-1.0f);
		}

		return normal;
	}

	D_INLINE dVector GetInt () const
	{
		return dVector (dInt32 (dFloor (m_x)), dInt32(dFloor (m_y)), dInt32(dFloor (m_z)), dInt32 (dFloor (m_w)));
	}

	D_INLINE dVector TestZero() const
	{
		const dInt32* const a = (dInt32*)&m_x;
		return dVector ((a[0] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f),
						 (a[1] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f),
						 (a[2] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f),
						 (a[3] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f));
	}


	D_INLINE dVector Floor () const
	{
		return dVector (dFloor (m_x), dFloor (m_y), dFloor (m_z), dFloor (m_w));
	}

	D_INLINE dVector DotProduct (const dVector &A) const
	{
		return dVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	D_INLINE dVector Reciproc () const
	{
		return dVector (dFloat32 (1.0f) / m_x, dFloat32 (1.0f) / m_y, dFloat32 (1.0f) / m_z, dFloat32 (1.0f) / m_w);
	}

	D_INLINE dVector Sqrt () const
	{
		return dVector (dSqrt (m_x), dSqrt (m_y), dSqrt (m_z), dSqrt (m_w));
	}

	D_INLINE dVector InvSqrt () const
	{
		return dVector (dRsqrt (m_x), dRsqrt (m_y), dRsqrt (m_z), dRsqrt (m_w));
	}

	D_INLINE dVector InvMagSqrt () const
	{
		return dVector (dRsqrt (DotProduct(*this).m_x));
	}

	D_INLINE dVector Normalize () const
	{
		dAssert (m_w == dFloat32 (0.0f));
		//return *this * dVector (dRsqrt (DotProduct(*this).m_x));
		//return Scale (dRsqrt (DotProduct(*this).GetScalar()));
		const dVector& me = *this; 
		return me * InvMagSqrt();
	}

	dVector Abs () const
	{
		return dVector ((m_x > dFloat32 (0.0f)) ? m_x : -m_x,
			(m_y > dFloat32 (0.0f)) ? m_y : -m_y,
			(m_z > dFloat32 (0.0f)) ? m_z : -m_z,
			(m_w > dFloat32 (0.0f)) ? m_w : -m_w);
	}

	dFloat32 GetMax () const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	dVector GetMax (const dVector& data) const
	{
		return dVector ((m_x > data.m_x) ? m_x : data.m_x,
			(m_y > data.m_y) ? m_y : data.m_y,
			(m_z > data.m_z) ? m_z : data.m_z,
			(m_w > data.m_w) ? m_w : data.m_w);
	}

	dVector GetMin (const dVector& data) const
	{
		return dVector ((m_x < data.m_x) ? m_x : data.m_x,
			(m_y < data.m_y) ? m_y : data.m_y,
			(m_z < data.m_z) ? m_z : data.m_z,
			(m_w < data.m_w) ? m_w : data.m_w);
	}


	// relational operators
	D_INLINE dVector operator== (const dVector& data) const
	{
		return dVector ((m_x == data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y == data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z == data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w == data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	D_INLINE dVector operator> (const dVector& data) const
	{
		return dVector ((m_x > data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y > data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z > data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w > data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	D_INLINE dVector operator< (const dVector& data) const
	{
		return dVector ((m_x < data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y < data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z < data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w < data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	D_INLINE dVector operator>= (const dVector& data) const
	{
		return dVector ((m_x >= data.m_x) ? dInt32 (0xffffffff) : 0, 
			(m_y >= data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z >= data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w >= data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	D_INLINE dVector operator<= (const dVector& data) const
	{
		return dVector ((m_x <= data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y <= data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z <= data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w <= data.m_w) ? dInt32 (0xffffffff) : 0);
	}


	// logical operations
	D_INLINE dVector operator& (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	D_INLINE dVector operator| (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	D_INLINE dVector operator^ (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	D_INLINE dVector AndNot (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	D_INLINE dVector Select (const dVector& data, const dVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	D_INLINE dInt32 GetSignMask() const
	{
		const dInt32* const a = (dInt32*)&m_x;
		return (((a[0] & 0x80000000) ? 1 : 0) | ((a[1] & 0x80000000) ? 2 : 0) | ((a[2] & 0x80000000) ? 4 : 0) | ((a[3] & 0x80000000) ? 8 : 0));
	} 

	D_INLINE dVector ShiftRight() const
	{
		return dVector (m_w, m_x, m_y, m_z); 
	}

	D_INLINE dVector ShiftTripleRight () const
	{
		return dVector (m_z, m_x, m_y, m_w); 
	}

	D_INLINE dVector ShiftTripleLeft () const
	{
		return dVector (m_y, m_z, m_x, m_w); 
	}

	D_INLINE dVector ShiftRightLogical (dInt32 bits) const
	{
		return dVector (dInt32 (dUnsigned32 (m_ix) >> bits), dInt32 (dUnsigned32 (m_iy) >> bits), dInt32 (dUnsigned32 (m_iz) >> bits), dInt32 (dUnsigned32 (m_iw) >> bits)); 
	}

	D_INLINE static void Transpose4x4 (dVector& dst0, dVector& dst1, dVector& dst2, dVector& dst3, const dVector& src0, const dVector& src1, const dVector& src2, const dVector& src3)
	{
		dVector tmp0 (src0);
		dVector tmp1 (src1);
		dVector tmp2 (src2);
		dVector tmp3 (src3);

		dst0 = dVector (tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = dVector (tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = dVector (tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = dVector (tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	union 
	{
		dInt32 m_i[4];
		struct 
		{
			dFloat32 m_x;
			dFloat32 m_y;
			dFloat32 m_z;
			dFloat32 m_w;
		};
		struct 
		{
			dInt32 m_ix;
			dInt32 m_iy;
			dInt32 m_iz;
			dInt32 m_iw;
		};
	};

	static dVector m_zero;
	static dVector m_one;
	static dVector m_wOne;
	static dVector m_half;
	static dVector m_two;
	static dVector m_three;
	static dVector m_negOne;
	static dVector m_xMask;
	static dVector m_yMask;
	static dVector m_zMask;
	static dVector m_wMask;
	static dVector m_epsilon;
	static dVector m_signMask;
	static dVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

class dBigVector
{
	public:
	D_INLINE dBigVector()
	{
	}

	D_INLINE dBigVector(dFloat64 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	D_INLINE dBigVector (const dBigVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

#ifndef D_NEWTON_USE_DOUBLE
	D_INLINE dBigVector (const dVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	D_INLINE dBigVector (const dFloat32* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (dFloat32 (0.0f))
	{
		dAssert (dCheckVector ((*this)));
	}
#endif

	D_INLINE dBigVector (const dFloat64* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (ptr[3])
	{
		dAssert (dCheckVector ((*this)));
	}

	D_INLINE dBigVector (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert (dCheckVector ((*this)));
	}

	D_INLINE dBigVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	D_INLINE dBigVector (dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	D_INLINE dFloat64 GetScalar () const
	{
		return m_x;
	}

	D_INLINE void Store (dFloat64* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	D_INLINE dBigVector BroadcastX () const
	{
		return dBigVector (m_x);
	}

	D_INLINE dBigVector BroadcastY () const
	{
		return dBigVector (m_y);
	}

	D_INLINE dBigVector BroadcastZ () const
	{
		return dBigVector (m_z);
	}

	D_INLINE dBigVector BroadcastW () const
	{
		return dBigVector (m_w);
	}


	D_INLINE dFloat64& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	D_INLINE const dFloat64& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	D_INLINE dBigVector operator+ (const dBigVector& A) const
	{
		return dBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	D_INLINE dBigVector operator- (const dBigVector& A) const 
	{
		return dBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	D_INLINE dBigVector operator* (const dBigVector& A) const
	{
		return dBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	D_INLINE dBigVector& operator+= (const dBigVector& A)
	{
		return (*this = dBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	D_INLINE dBigVector& operator-= (const dBigVector& A)
	{
		return (*this = dBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	D_INLINE dBigVector& operator*= (const dBigVector& A)
	{
		return (*this = dBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	D_INLINE dBigVector MulAdd(const dBigVector& A, const dBigVector& B) const
	{
		return *this + A * B;
	}

	D_INLINE dBigVector MulSub(const dVector& A, const dBigVector& B) const
	{
		return *this - A * B;
	}


	D_INLINE dBigVector AddHorizontal () const
	{
		return dBigVector (m_x + m_y + m_z + m_w);
	}

	D_INLINE dBigVector Scale (dFloat64 scale) const
	{
		return dBigVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	D_INLINE dBigVector CrossProduct (const dBigVector& B) const
	{
		return dBigVector (m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	D_INLINE dBigVector CrossProduct (const dBigVector& A, const dBigVector& B) const
	{
		dFloat64 cofactor[3][3];
		dFloat64 array[4][4];

		const dBigVector& me = *this;
		for (dInt32 i = 0; i < 4; i ++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dFloat32 (1.0f);
		}

		dBigVector normal;
		dFloat64  sign = dFloat64 (-1.0f);
		for (dInt32 i = 0; i < 4; i ++)  
		{
			for (dInt32 j = 0; j < 3; j ++) 
			{
				dInt32 k0 = 0;
				for (dInt32 k = 0; k < 4; k ++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			dFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= dFloat64 (-1.0f);
		}

		return normal;
	}

	D_INLINE dBigVector GetInt () const
	{
		return dBigVector (dInt64 (floor (m_x)), dInt64(floor (m_y)), dInt64(floor (m_z)), dInt64 (floor (m_w)));
	}

	D_INLINE dBigVector TestZero() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return dBigVector ((a[0] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f),
							(a[1] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f),
							(a[2] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f),
							(a[3] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f));
	}


	D_INLINE dBigVector Floor () const
	{
		return dBigVector (floor (m_x), floor (m_y), floor (m_z), floor (m_w));
	}

	D_INLINE dBigVector DotProduct (const dBigVector &A) const
	{
		return dBigVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	D_INLINE dBigVector Reciproc () const
	{
		return dBigVector (dFloat64 (1.0f) / m_x, dFloat64 (1.0f) / m_y, dFloat64 (1.0f) / m_z, dFloat64 (1.0f) / m_w);
	}

	D_INLINE dBigVector Sqrt () const
	{
		return dBigVector (sqrt (m_x), sqrt (m_y), sqrt (m_z), sqrt (m_w));
	}

	D_INLINE dBigVector InvSqrt () const
	{
		return dBigVector (dFloat64 (1.0f) / sqrt (m_x), dFloat64 (1.0f) / sqrt (m_y), dFloat64 (1.0f) / sqrt (m_z), dFloat64 (1.0f) / sqrt (m_w));
	}

	D_INLINE dBigVector InvMagSqrt () const
	{
		return dBigVector (dFloat64 (1.0f) / sqrt (DotProduct(*this).m_x));
	}

	D_INLINE dBigVector Normalize() const
	{
		dAssert (m_w == dFloat64 (0.0f));
		//const dBigVector& me = *this;
		//return *this * dBigVector (dRsqrt(DotProduct(*this).m_x));
		return *this * InvMagSqrt();
	}

	dBigVector Abs () const
	{
		return dBigVector ((m_x > dFloat64 (0.0f)) ? m_x : -m_x,
							(m_y > dFloat64 (0.0f)) ? m_y : -m_y,
							(m_z > dFloat64 (0.0f)) ? m_z : -m_z,
							(m_w > dFloat64 (0.0f)) ? m_w : -m_w);
	}

	dFloat64 GetMax () const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	dBigVector GetMax (const dBigVector& data) const
	{
		return dBigVector ((m_x > data.m_x) ? m_x : data.m_x,
							(m_y > data.m_y) ? m_y : data.m_y,
							(m_z > data.m_z) ? m_z : data.m_z,
							(m_w > data.m_w) ? m_w : data.m_w);
					}

	dBigVector GetMin (const dBigVector& data) const
	{
		return dBigVector ((m_x < data.m_x) ? m_x : data.m_x,
							(m_y < data.m_y) ? m_y : data.m_y,
							(m_z < data.m_z) ? m_z : data.m_z,
							(m_w < data.m_w) ? m_w : data.m_w);
	}

	// relational operators
	D_INLINE dBigVector operator== (const dBigVector& data) const
	{
		return dBigVector ((m_x == data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y == data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z == data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w == data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	D_INLINE dBigVector operator> (const dBigVector& data) const
	{
		return dBigVector ((m_x > data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y > data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z > data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w > data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	D_INLINE dBigVector operator< (const dBigVector& data) const
	{
		return dBigVector ((m_x < data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y < data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z < data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w < data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	D_INLINE dBigVector operator>= (const dBigVector& data) const
	{
		return dBigVector ((m_x >= data.m_x) ? dInt64 (-1) : dInt64 (0), 
							(m_y >= data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z >= data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w >= data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	D_INLINE dBigVector operator<= (const dBigVector& data) const
	{
		return dBigVector ((m_x <= data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y <= data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z <= data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w <= data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	// logical operations
	D_INLINE dBigVector operator& (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	D_INLINE dBigVector operator| (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	D_INLINE dBigVector operator^ (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	D_INLINE dBigVector AndNot (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	D_INLINE dBigVector Select(const dBigVector& data, const dBigVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	D_INLINE dInt32 GetSignMask() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return (((a[0]>>63) ? 1 : 0) | ((a[1]>>63) ? 2 : 0) | ((a[2]>>63) ? 4 : 0) | ((a[3]>>63) ? 8 : 0));
	} 

	D_INLINE dVector ShiftRight() const
	{
		return dBigVector (m_w, m_x, m_y, m_z); 
	}

	D_INLINE dBigVector ShiftTripleRight () const
	{
		return dBigVector (m_z, m_x, m_y, m_w); 
	}

	D_INLINE dBigVector ShiftTripleLeft () const
	{
		return dBigVector (m_y, m_z, m_x, m_w); 
	}

	D_INLINE dBigVector ShiftRightLogical (dInt32 bits) const
	{
		return dBigVector (dInt64 (dUnsigned64 (m_ix) >> bits), dInt64 (dUnsigned64 (m_iy) >> bits), dInt64 (dUnsigned64 (m_iz) >> bits), dInt64 (dUnsigned64 (m_iw) >> bits)); 
	}

	D_INLINE static void Transpose4x4 (dBigVector& dst0, dBigVector& dst1, dBigVector& dst2, dBigVector& dst3, const dBigVector& src0, const dBigVector& src1, const dBigVector& src2, const dBigVector& src3)
	{
		dBigVector tmp0 (src0);
		dBigVector tmp1 (src1);
		dBigVector tmp2 (src2);
		dBigVector tmp3 (src3);

		dst0 = dBigVector (tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = dBigVector (tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = dBigVector (tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = dBigVector (tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	union 
	{
		dInt64 m_i[4];
		struct 
		{
			dFloat64 m_x;
			dFloat64 m_y;
			dFloat64 m_z;
			dFloat64 m_w;
		};
		struct 
		{
			dInt64 m_ix;
			dInt64 m_iy;
			dInt64 m_iz;
			dInt64 m_iw;
		};
	};

	static dBigVector m_zero;
	static dBigVector m_one;
	static dBigVector m_wOne;
	static dBigVector m_half;
	static dBigVector m_two;
	static dBigVector m_three;
	static dBigVector m_negOne;
	static dBigVector m_xMask;
	static dBigVector m_yMask;
	static dBigVector m_zMask;
	static dBigVector m_wMask;
	static dBigVector m_epsilon;
	static dBigVector m_signMask;
	static dBigVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32;

class dSpatialVector
{
	public:
	D_INLINE dSpatialVector()
	{
	}

	D_INLINE dSpatialVector(const dFloat32 a)
	{
		for (dInt32 i = 0; i < 6; i ++) 
		{
			m_d[i] = a;
		}
	}

	D_INLINE dSpatialVector(const dVector& low, const dVector& high)
	{
		for (dInt32 i = 0; i < 3; i ++) 
		{
			m_d[i] = low[i];
			m_d[i + 3] = high[i];
		}
	}

	D_INLINE dSpatialVector(const dSpatialVector& src)
	{
		for (dInt32 i = 0; i < 6; i++) 
		{
			m_d[i] = src[i];
		}
	}

	D_INLINE dFloat64& operator[] (dInt32 i)
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_d[i];
	}

	D_INLINE const dFloat64& operator[] (dInt32 i) const
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_d[i];
	}

	D_INLINE dSpatialVector operator+ (const dSpatialVector& A) const
	{
		dSpatialVector tmp;
		for (dInt32 i = 0; i < 6; i++) 
		{
			tmp[i] = m_d[i] + A.m_d[i];
		}
		return tmp;
	}

	D_INLINE dSpatialVector operator* (const dSpatialVector& A) const
	{
		dSpatialVector tmp;
		for (dInt32 i = 0; i < 6; i++) 
		{
			tmp[i] = m_d[i] * A.m_d[i];
		}
		return tmp;
	}

	D_INLINE dFloat64 DotProduct(const dSpatialVector& v) const
	{
		dFloat64 ret = dFloat64 (0.0f);
		for (dInt32 i = 0; i < 6; i++) 
		{
			ret += m_d[i] * v.m_d[i];
		}
		return ret;
	}

	D_INLINE dSpatialVector Scale(dFloat64 s) const
	{
		dSpatialVector tmp;
		for (dInt32 i = 0; i < 6; i++) 
		{
			tmp[i] = m_d[i] * s;
		}
		return tmp;
	}

	dFloat64 m_d[6];
	static dSpatialVector m_zero;
} D_GCC_NEWTON_ALIGN_32 ;

#endif
