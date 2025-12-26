/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_VECTOR16_H__
#define __ND_VECTOR16_H__

#include "ndVector8.h"

#define ND_SIMD16_WORK_GROUP_SIZE	16 

D_MSV_NEWTON_CLASS_ALIGN_32
class ndVector16
{
	public:
	inline ndVector16()
	{
	}

	inline ndVector16(const ndFloat32 val)
		:m_low(val)
		,m_high(val)
	{
	}

	inline ndVector16(const ndInt32 val)
		:m_low(val)
		,m_high(val)
	{
	}

	inline ndVector16(const ndVector16& copy)
		:m_low(copy.m_low)
		,m_high(copy.m_high)
	{
	}

	inline ndVector16(const ndVector8& low, const ndVector8& high)
		:m_low(low)
		,m_high(high)
	{
	}

	inline ndVector16(const ndVector& v0, const ndVector& v1, const ndVector& v2, const ndVector& v3)
		:m_low(v0, v1)
		,m_high(v2, v3)
	{
	}

	inline ndVector16(const ndVector16* const baseAddr, const ndVector16& index)
		:m_low(&baseAddr->m_low, index.m_low)
		,m_high(&baseAddr->m_low, index.m_high)
	{
	}

	inline ndFloat32& operator[] (ndInt32 i)
	{
		ndAssert(i >= 0);
		ndAssert(i < ND_SIMD16_WORK_GROUP_SIZE);
		ndFloat32* const data = &m_low[0];
		return data[i];
	}

	inline const ndFloat32& operator[] (ndInt32 i) const
	{
		ndAssert(i >= 0);
		ndAssert(i < ND_SIMD16_WORK_GROUP_SIZE);
		const ndFloat32* const data = &m_low[0];
		return data[i];
	}

	inline ndVector16& operator= (const ndVector16& A)
	{
		m_low = A.m_low;
		m_high = A.m_high;
		return *this;
	}

	inline ndVector16 operator+ (const ndVector16& A) const
	{
		return ndVector16(m_low + A.m_low, m_high + A.m_high);
	}

	inline ndVector16 operator- (const ndVector16& A) const
	{
		return ndVector16(m_low - A.m_low, m_high - A.m_high);
	}

	inline ndVector16 operator* (const ndVector16& A) const
	{
		return ndVector16(m_low * A.m_low, m_high * A.m_high);
	}

	inline ndVector16 MulAdd(const ndVector16& A, const ndVector16& B) const
	{
		return ndVector16(m_low.MulAdd(A.m_low, B.m_low), m_high.MulAdd(A.m_high, B.m_high));
	}

	inline ndVector16 MulSub(const ndVector16& A, const ndVector16& B) const
	{
		return ndVector16(m_low.MulSub(A.m_low, B.m_low), m_high.MulSub(A.m_high, B.m_high));
	}

	inline ndVector16 operator> (const ndVector16& A) const
	{
		return ndVector16(m_low > A.m_low, m_high > A.m_high);
	}

	inline ndVector16 operator< (const ndVector16& A) const
	{
		return ndVector16(m_low < A.m_low, m_high < A.m_high);
	}

	inline ndVector16 operator| (const ndVector16& A) const
	{
		return ndVector16(m_low | A.m_low, m_high | A.m_high);
	}

	inline ndVector16 operator& (const ndVector16& A) const
	{
		return ndVector16(m_low & A.m_low, m_high & A.m_high);
	}

	inline ndVector16 GetMin(const ndVector16& A) const
	{
		return ndVector16(m_low.GetMin(A.m_low), m_high.GetMin(A.m_high));
	}

	inline ndVector16 GetMax(const ndVector16& A) const
	{
		return ndVector16(m_low.GetMax(A.m_low), m_high.GetMax(A.m_high));
	}

	inline ndVector16 Select(const ndVector16& data, const ndVector16& mask) const
	{
		return ndVector16(m_low.Select(data.m_low, mask.m_low), m_high.Select(data.m_high, mask.m_high));
	}

	inline ndVector8 GetLow() const
	{
		return m_low;
	}

	inline ndVector8 GetHigh() const
	{
		return m_high;
	}

	inline ndFloat32 GetMax() const
	{
		return ndMax (m_low.GetMax(), m_high.GetMax());
	}

	inline ndFloat32 AddHorizontal() const
	{
		return (m_low + m_high).AddHorizontal();
	}

	static inline void FlushRegisters()
	{
	}

	ndVector8 m_low;
	ndVector8 m_high;

	D_CORE_API static ndVector16 m_one;
	D_CORE_API static ndVector16 m_zero;
	D_CORE_API static ndVector16 m_mask;
	D_CORE_API static ndVector16 m_ordinals;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif