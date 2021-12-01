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
	#define dVector dBigVector
#else

class dBigVector;
class dVector
{
	public:
	inline dVector()
	{
	}

	inline dVector(dFloat32 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline dVector (const dVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline dVector (const dFloat32* const ptr)
		:m_x(ptr[0])
		,m_y(ptr[1])
		,m_z(ptr[2])
		,m_w(ptr[3])
	{
		dAssert (dCheckVector ((*this)));
	}

	// emulate gather instruction
	inline dVector(const dFloat32* const baseAddr, const dInt32* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}

#ifndef	D_NEWTON_USE_DOUBLE
	inline dVector(const dFloat64* const ptr)
		:m_x(dFloat32(ptr[0]))
		,m_y(dFloat32(ptr[1]))
		,m_z(dFloat32(ptr[2]))
		,m_w(dFloat32(ptr[3]))
	{
	}
#endif


	inline dVector (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert (dCheckVector ((*this)));
	}

	inline dVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_x(*((dFloat32*)&ix)), m_y(*((dFloat32*)&iy)), m_z(*((dFloat32*)&iz)), m_w(*((dFloat32*)&iw))
	{
	}

#ifndef  D_NEWTON_USE_DOUBLE 
	inline dVector (const dBigVector& copy)
		:m_x(dFloat32 (((dFloat64*)&copy)[0])) 
		,m_y(dFloat32 (((dFloat64*)&copy)[1])) 
		,m_z(dFloat32 (((dFloat64*)&copy)[2])) 
		,m_w(dFloat32 (((dFloat64*)&copy)[3])) 
	{
		dAssert (dCheckVector ((*this)));
	}
#endif

	inline dFloat32 GetScalar () const
	{
		return m_x;
	}

	inline void Store (dFloat32* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	inline dVector BroadcastX () const
	{
		return dVector (m_x);
	}

	inline dVector BroadcastY () const
	{
		return dVector (m_y);
	}

	inline dVector BroadcastZ () const
	{
		return dVector (m_z);
	}

	inline dVector BroadcastW () const
	{
		return dVector (m_w);
	}


	inline dFloat32& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline const dFloat32& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline dVector operator+ (const dVector& A) const
	{
		return dVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline dVector operator- (const dVector& A) const 
	{
		return dVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline dVector operator* (const dVector& A) const
	{
		return dVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline dVector& operator+= (const dVector& A)
	{
		return (*this = dVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline dVector& operator-= (const dVector& A)
	{
		return (*this = dVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline dVector& operator*= (const dVector& A)
	{
		return (*this = dVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline dVector MulAdd(const dVector& A, const dVector& B) const
	{
		return *this + A * B;
	}

	inline dVector MulSub(const dVector& A, const dVector& B) const
	{
		return *this - A * B;
	}

	inline dVector AddHorizontal () const
	{
		return dVector (m_x + m_y + m_z + m_w);
	}

	inline dVector Scale (dFloat32 scale) const
	{
		return dVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline dVector CrossProduct (const dVector& B) const
	{
		return dVector (m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline dVector CrossProduct (const dVector& A, const dVector& B) const
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

	inline dVector GetInt () const
	{
		return dVector (dInt32 (dFloor (m_x)), dInt32(dFloor (m_y)), dInt32(dFloor (m_z)), dInt32 (dFloor (m_w)));
	}

	inline dVector TestZero() const
	{
		const dInt32* const a = (dInt32*)&m_x;
		return dVector ((a[0] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f),
						 (a[1] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f),
						 (a[2] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f),
						 (a[3] == 0) ? dFloat32 (-1.0f) : dFloat32 (1.0f));
	}


	inline dVector Floor () const
	{
		return dVector (dFloor (m_x), dFloor (m_y), dFloor (m_z), dFloor (m_w));
	}

	inline dVector DotProduct (const dVector &A) const
	{
		return dVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline dVector Reciproc () const
	{
		return dVector (dFloat32 (1.0f) / m_x, dFloat32 (1.0f) / m_y, dFloat32 (1.0f) / m_z, dFloat32 (1.0f) / m_w);
	}

	inline dVector Sqrt () const
	{
		return dVector (dSqrt (m_x), dSqrt (m_y), dSqrt (m_z), dSqrt (m_w));
	}

	inline dVector InvSqrt () const
	{
		return dVector (dRsqrt (m_x), dRsqrt (m_y), dRsqrt (m_z), dRsqrt (m_w));
	}

	inline dVector InvMagSqrt () const
	{
		return dVector (dRsqrt (DotProduct(*this).m_x));
	}

	inline dVector Normalize () const
	{
		dAssert (m_w == dFloat32 (0.0f));
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
	inline dVector operator== (const dVector& data) const
	{
		return dVector ((m_x == data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y == data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z == data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w == data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	inline dVector operator> (const dVector& data) const
	{
		return dVector ((m_x > data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y > data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z > data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w > data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	inline dVector operator< (const dVector& data) const
	{
		return dVector ((m_x < data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y < data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z < data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w < data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	inline dVector operator>= (const dVector& data) const
	{
		return dVector ((m_x >= data.m_x) ? dInt32 (0xffffffff) : 0, 
			(m_y >= data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z >= data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w >= data.m_w) ? dInt32 (0xffffffff) : 0);
	}

	inline dVector operator<= (const dVector& data) const
	{
		return dVector ((m_x <= data.m_x) ? dInt32 (0xffffffff) : 0,
			(m_y <= data.m_y) ? dInt32 (0xffffffff) : 0,
			(m_z <= data.m_z) ? dInt32 (0xffffffff) : 0,
			(m_w <= data.m_w) ? dInt32 (0xffffffff) : 0);
	}


	// logical operations
	inline dVector operator& (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	inline dVector operator| (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	inline dVector operator^ (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	inline dVector AndNot (const dVector& data) const
	{
		const dInt32* const a = (dInt32*)&m_x;
		const dInt32* const b = (dInt32*)&data.m_x;
		return dVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	inline dVector Select (const dVector& data, const dVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	inline dInt32 GetSignMask() const
	{
		const dInt32* const a = (dInt32*)&m_x;
		return (((a[0] & 0x80000000) ? 1 : 0) | ((a[1] & 0x80000000) ? 2 : 0) | ((a[2] & 0x80000000) ? 4 : 0) | ((a[3] & 0x80000000) ? 8 : 0));
	} 

	inline dVector ShiftRight() const
	{
		return dVector (m_w, m_x, m_y, m_z); 
	}

	inline dVector ShiftTripleRight () const
	{
		return dVector (m_z, m_x, m_y, m_w); 
	}

	inline dVector ShiftTripleLeft () const
	{
		return dVector (m_y, m_z, m_x, m_w); 
	}

	inline dVector ShiftRightLogical (dInt32 bits) const
	{
		return dVector (dInt32 (dUnsigned32 (m_ix) >> bits), dInt32 (dUnsigned32 (m_iy) >> bits), dInt32 (dUnsigned32 (m_iz) >> bits), dInt32 (dUnsigned32 (m_iw) >> bits)); 
	}

	inline static void Transpose4x4 (dVector& dst0, dVector& dst1, dVector& dst2, dVector& dst3, const dVector& src0, const dVector& src1, const dVector& src2, const dVector& src3)
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

	D_CORE_API static dVector m_zero;
	D_CORE_API static dVector m_one;
	D_CORE_API static dVector m_wOne;
	D_CORE_API static dVector m_half;
	D_CORE_API static dVector m_two;
	D_CORE_API static dVector m_three;
	D_CORE_API static dVector m_negOne;
	D_CORE_API static dVector m_xMask;
	D_CORE_API static dVector m_yMask;
	D_CORE_API static dVector m_zMask;
	D_CORE_API static dVector m_wMask;
	D_CORE_API static dVector m_xyzwMask;
	D_CORE_API static dVector m_epsilon;
	D_CORE_API static dVector m_signMask;
	D_CORE_API static dVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

class dBigVector
{
	public:
	inline dBigVector()
	{
	}

	inline dBigVector(dFloat64 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline dBigVector (const dBigVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

#ifndef D_NEWTON_USE_DOUBLE
	inline dBigVector (const dVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline dBigVector (const dFloat32* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (dFloat32 (0.0f))
	{
		dAssert (dCheckVector ((*this)));
	}
#endif

	inline dBigVector (const dFloat64* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (ptr[3])
	{
		dAssert (dCheckVector ((*this)));
	}

	inline dBigVector (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w) 
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert (dCheckVector ((*this)));
	}

	inline dBigVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline dBigVector (dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline dFloat64 GetScalar () const
	{
		return m_x;
	}

	inline void Store (dFloat64* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	inline dBigVector BroadcastX () const
	{
		return dBigVector (m_x);
	}

	inline dBigVector BroadcastY () const
	{
		return dBigVector (m_y);
	}

	inline dBigVector BroadcastZ () const
	{
		return dBigVector (m_z);
	}

	inline dBigVector BroadcastW () const
	{
		return dBigVector (m_w);
	}


	inline dFloat64& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline const dFloat64& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	inline dBigVector operator+ (const dBigVector& A) const
	{
		return dBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline dBigVector operator- (const dBigVector& A) const 
	{
		return dBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline dBigVector operator* (const dBigVector& A) const
	{
		return dBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline dBigVector& operator+= (const dBigVector& A)
	{
		return (*this = dBigVector (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline dBigVector& operator-= (const dBigVector& A)
	{
		return (*this = dBigVector (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline dBigVector& operator*= (const dBigVector& A)
	{
		return (*this = dBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline dBigVector MulAdd(const dBigVector& A, const dBigVector& B) const
	{
		return *this + A * B;
	}

	inline dBigVector MulSub(const dVector& A, const dBigVector& B) const
	{
		return *this - A * B;
	}


	inline dBigVector AddHorizontal () const
	{
		return dBigVector (m_x + m_y + m_z + m_w);
	}

	inline dBigVector Scale (dFloat64 scale) const
	{
		return dBigVector (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline dBigVector CrossProduct (const dBigVector& B) const
	{
		return dBigVector (m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline dBigVector CrossProduct (const dBigVector& A, const dBigVector& B) const
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

	inline dBigVector GetInt () const
	{
		return dBigVector (dInt64 (floor (m_x)), dInt64(floor (m_y)), dInt64(floor (m_z)), dInt64 (floor (m_w)));
	}

	inline dBigVector TestZero() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return dBigVector ((a[0] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f),
							(a[1] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f),
							(a[2] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f),
							(a[3] == 0) ? dFloat64 (-1.0f) : dFloat64 (1.0f));
	}


	inline dBigVector Floor () const
	{
		return dBigVector (floor (m_x), floor (m_y), floor (m_z), floor (m_w));
	}

	inline dBigVector DotProduct (const dBigVector &A) const
	{
		return dBigVector (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline dBigVector Reciproc () const
	{
		return dBigVector (dFloat64 (1.0f) / m_x, dFloat64 (1.0f) / m_y, dFloat64 (1.0f) / m_z, dFloat64 (1.0f) / m_w);
	}

	inline dBigVector Sqrt () const
	{
		return dBigVector (sqrt (m_x), sqrt (m_y), sqrt (m_z), sqrt (m_w));
	}

	inline dBigVector InvSqrt () const
	{
		return dBigVector (dFloat64 (1.0f) / sqrt (m_x), dFloat64 (1.0f) / sqrt (m_y), dFloat64 (1.0f) / sqrt (m_z), dFloat64 (1.0f) / sqrt (m_w));
	}

	inline dBigVector InvMagSqrt () const
	{
		return dBigVector (dFloat64 (1.0f) / sqrt (DotProduct(*this).m_x));
	}

	inline dBigVector Normalize() const
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
	inline dBigVector operator== (const dBigVector& data) const
	{
		return dBigVector ((m_x == data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y == data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z == data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w == data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	inline dBigVector operator> (const dBigVector& data) const
	{
		return dBigVector ((m_x > data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y > data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z > data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w > data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	inline dBigVector operator< (const dBigVector& data) const
	{
		return dBigVector ((m_x < data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y < data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z < data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w < data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	inline dBigVector operator>= (const dBigVector& data) const
	{
		return dBigVector ((m_x >= data.m_x) ? dInt64 (-1) : dInt64 (0), 
							(m_y >= data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z >= data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w >= data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	inline dBigVector operator<= (const dBigVector& data) const
	{
		return dBigVector ((m_x <= data.m_x) ? dInt64 (-1) : dInt64 (0),
							(m_y <= data.m_y) ? dInt64 (-1) : dInt64 (0),
							(m_z <= data.m_z) ? dInt64 (-1) : dInt64 (0),
							(m_w <= data.m_w) ? dInt64 (-1) : dInt64 (0));
	}

	// logical operations
	inline dBigVector operator& (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]); 
	}

	inline dBigVector operator| (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]); 
	}

	inline dBigVector operator^ (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]); 
	}

	inline dBigVector AndNot (const dBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return dBigVector (a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]); 
	}

	inline dBigVector Select(const dBigVector& data, const dBigVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	inline dInt32 GetSignMask() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return (((a[0]>>63) ? 1 : 0) | ((a[1]>>63) ? 2 : 0) | ((a[2]>>63) ? 4 : 0) | ((a[3]>>63) ? 8 : 0));
	} 

	inline dVector ShiftRight() const
	{
		return dBigVector (m_w, m_x, m_y, m_z); 
	}

	inline dBigVector ShiftTripleRight () const
	{
		return dBigVector (m_z, m_x, m_y, m_w); 
	}

	inline dBigVector ShiftTripleLeft () const
	{
		return dBigVector (m_y, m_z, m_x, m_w); 
	}

	inline dBigVector ShiftRightLogical (dInt32 bits) const
	{
		return dBigVector (dInt64 (dUnsigned64 (m_ix) >> bits), dInt64 (dUnsigned64 (m_iy) >> bits), dInt64 (dUnsigned64 (m_iz) >> bits), dInt64 (dUnsigned64 (m_iw) >> bits)); 
	}

	inline static void Transpose4x4 (dBigVector& dst0, dBigVector& dst1, dBigVector& dst2, dBigVector& dst3, const dBigVector& src0, const dBigVector& src1, const dBigVector& src2, const dBigVector& src3)
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

	D_CORE_API static dBigVector m_zero;
	D_CORE_API static dBigVector m_one;
	D_CORE_API static dBigVector m_wOne;
	D_CORE_API static dBigVector m_half;
	D_CORE_API static dBigVector m_two;
	D_CORE_API static dBigVector m_three;
	D_CORE_API static dBigVector m_negOne;
	D_CORE_API static dBigVector m_xMask;
	D_CORE_API static dBigVector m_yMask;
	D_CORE_API static dBigVector m_zMask;
	D_CORE_API static dBigVector m_wMask;
	D_CORE_API static dBigVector m_xyzwMask;
	D_CORE_API static dBigVector m_epsilon;
	D_CORE_API static dBigVector m_signMask;
	D_CORE_API static dBigVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32;


#endif
