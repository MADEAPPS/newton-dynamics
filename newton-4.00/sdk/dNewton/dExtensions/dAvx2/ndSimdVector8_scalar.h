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

#ifndef __ND_SIMD_VECTOR8_SCALAR_H__
#define __ND_SIMD_VECTOR8_SCALAR_H__

#include "ndCore.h"

#define ND_SIMD8_WORK_GROUP_SIZE	8 

#ifdef D_NEWTON_USE_DOUBLE
D_MSV_NEWTON_CLASS_ALIGN_32
class ndVector8
{
	public:
	inline ndVector8()
	{
	}

	inline ndVector8(const ndFloat32 val)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = val;
		}
	}

	inline ndVector8(const ndInt32 val)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_int[i] = ndInt64(val);
		}
	}

	inline ndVector8(const ndVector8& src)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = src.m_float[i];
		}
	}

	inline ndVector8(const ndVector& low, const ndVector& high)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE / 2; ++i)
		{
			m_float[i] = low[i];
			m_float[i + ND_SIMD8_WORK_GROUP_SIZE / 2] = high[i];
		}
	}

	inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
	{
		const ndFloat32* const base = (ndFloat32*)baseAddr;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = base[index.m_int[i]];
		}
	}

	inline ndFloat32& operator[] (ndInt32 i)
	{
		ndAssert(i >= 0);
		ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
		return m_float[i];
	}

	inline const ndFloat32& operator[] (ndInt32 i) const
	{
		ndAssert(i >= 0);
		ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
		return m_float[i];
	}

	inline ndVector8& operator= (const ndVector8& A)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = A.m_float[i];
		}
		return *this;
	}

	inline ndVector8 operator+ (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] + A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 operator- (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] - A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 operator* (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] * A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] + B.m_float[i] * A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] - B.m_float[i] * A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 operator> (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_float[i] > A.m_float[i] ? ndInt64(-1) : ndInt64(0);
		}
		return ret;
	}

	inline ndVector8 operator< (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_float[i] < A.m_float[i] ? ndInt64(-1) : ndInt64(0);
		}
		return ret;
	}

	inline ndVector8 operator| (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_int[i] | A.m_int[i];
		}
		return ret;
	}

	inline ndVector8 operator& (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_int[i] & A.m_int[i];
		}
		return ret;
	}

	inline ndVector8 GetMin(const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = ndMin(m_float[i], A.m_float[i]);
		}
		return ret;
	}

	inline ndVector8 GetMax(const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = ndMax(m_float[i], A.m_float[i]);
		}
		return ret;
	}

	inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = (mask.m_int[i] == 0) ? m_float[i] : data.m_float[i];
		}
		return ret;
	}

	inline ndVector GetLow() const
	{
		return m_vector8.m_linear;
	}

	inline ndVector GetHigh() const
	{
		return m_vector8.m_angular;
	}

	inline ndFloat32 GetMax() const
	{
		ndFloat32 ret = ndFloat32(-1.0e20f);
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret = ndMax(ret, m_float[i]);
		}
		return ret;
	}

	inline ndFloat32 AddHorizontal() const
	{
		ndFloat32 ret = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret += m_float[i];
		}
		return ret;
	}

	static inline void FlushRegisters()
	{
	}

	static inline void Transpose(
		ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
		ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
		const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
		const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
	{
		const ndMatrix off01(src0.m_vector8.m_angular, src1.m_vector8.m_angular, src2.m_vector8.m_angular, src3.m_vector8.m_angular);
		const ndMatrix off10(src4.m_vector8.m_linear, src5.m_vector8.m_linear, src6.m_vector8.m_linear, src7.m_vector8.m_linear);

		ndVector::Transpose4x4(
			dst0.m_vector8.m_linear, dst1.m_vector8.m_linear, dst2.m_vector8.m_linear, dst3.m_vector8.m_linear,
			src0.m_vector8.m_linear, src1.m_vector8.m_linear, src2.m_vector8.m_linear, src3.m_vector8.m_linear);

		ndVector::Transpose4x4(
			dst0.m_vector8.m_angular, dst1.m_vector8.m_angular, dst2.m_vector8.m_angular, dst3.m_vector8.m_angular,
			off10[0], off10[1], off10[2], off10[3]);

		ndVector::Transpose4x4(
			dst4.m_vector8.m_linear, dst5.m_vector8.m_linear, dst6.m_vector8.m_linear, dst7.m_vector8.m_linear,
			off01[0], off01[1], off01[2], off01[3]);

		ndVector::Transpose4x4(
			dst4.m_vector8.m_angular, dst5.m_vector8.m_angular, dst6.m_vector8.m_angular, dst7.m_vector8.m_angular,
			src4.m_vector8.m_angular, src5.m_vector8.m_angular, src6.m_vector8.m_angular, src7.m_vector8.m_angular);
	}

	union
	{
		ndFloat64 m_float[ND_SIMD8_WORK_GROUP_SIZE];
		ndInt64 m_int[ND_SIMD8_WORK_GROUP_SIZE];
		struct
		{
			ndBigVector m_linear;
			ndBigVector m_angular;
		} m_vector8;
	};

	static ndVector8 m_one;
	static ndVector8 m_zero;
	static ndVector8 m_mask;
	static ndVector8 m_ordinals;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#else
D_MSV_NEWTON_CLASS_ALIGN_32
class ndVector8
{
	public:
	inline ndVector8()
	{
	}

	inline ndVector8(const ndFloat32 val)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = val;
		}
	}

	inline ndVector8(const ndInt32 val)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_int[i] = val;
		}
	}

	inline ndVector8(const ndVector8& src)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = src.m_float[i];
		}
	}

	inline ndVector8(const ndVector& low, const ndVector& high)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE/2; ++i)
		{
			m_float[i] = low[i];
			m_float[i + ND_SIMD8_WORK_GROUP_SIZE / 2] = high[i];
		}
	}

	inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
	{
		const ndFloat32* const base = (ndFloat32*)baseAddr;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = base[index.m_int[i]];
		}
	}

	inline ndFloat32& operator[] (ndInt32 i)
	{
		ndAssert(i >= 0);
		ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
		return m_float[i];
	}

	inline const ndFloat32& operator[] (ndInt32 i) const
	{
		ndAssert(i >= 0);
		ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
		return m_float[i];
	}

	inline ndVector8& operator= (const ndVector8& A)
	{
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			m_float[i] = A.m_float[i];
		}
		return *this;
	}

	inline ndVector8 operator+ (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] + A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 operator- (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] - A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 operator* (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] * A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] + B.m_float[i] * A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = m_float[i] - B.m_float[i] * A.m_float[i];
		}
		return ret;
	}

	inline ndVector8 operator> (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_float[i] > A.m_float[i] ? -1 : 0;
		}
		return ret;
	}

	inline ndVector8 operator< (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_float[i] < A.m_float[i] ? -1 : 0;
		}
		return ret;
	}

	inline ndVector8 operator| (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_int[i] | A.m_int[i];
		}
		return ret;
	}

	inline ndVector8 operator& (const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_int[i] = m_int[i] & A.m_int[i];
		}
		return ret;
	}

	inline ndVector8 GetMin(const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = ndMin(m_float[i], A.m_float[i]);
		}
		return ret;
	}

	inline ndVector8 GetMax(const ndVector8& A) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = ndMax(m_float[i], A.m_float[i]);
		}
		return ret;
	}

	inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
	{
		ndVector8 ret;
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret.m_float[i] = (mask.m_int[i] == 0) ? m_float[i] : data.m_float[i];
		}
		return ret;
	}

	inline ndVector GetLow() const
	{
		return m_vector8.m_linear;
	}

	inline ndVector GetHigh() const
	{
		return m_vector8.m_angular;
	}

	inline ndFloat32 GetMax() const
	{
		ndFloat32 ret = ndFloat32(-1.0e20f);
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret  = ndMax(ret, m_float[i]);
		}
		return ret;
	}

	inline ndFloat32 AddHorizontal() const
	{
		ndFloat32 ret = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
		{
			ret += m_float[i];
		}
		return ret;
	}

	static inline void FlushRegisters()
	{
	}

	static inline void Transpose(
		ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
		ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
		const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
		const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
	{
		const ndMatrix off01(src0.m_vector8.m_angular, src1.m_vector8.m_angular, src2.m_vector8.m_angular, src3.m_vector8.m_angular);
		const ndMatrix off10(src4.m_vector8.m_linear, src5.m_vector8.m_linear, src6.m_vector8.m_linear, src7.m_vector8.m_linear);

		ndVector::Transpose4x4(
			dst0.m_vector8.m_linear, dst1.m_vector8.m_linear, dst2.m_vector8.m_linear, dst3.m_vector8.m_linear,
			src0.m_vector8.m_linear, src1.m_vector8.m_linear, src2.m_vector8.m_linear, src3.m_vector8.m_linear);

		ndVector::Transpose4x4(
			dst0.m_vector8.m_angular, dst1.m_vector8.m_angular, dst2.m_vector8.m_angular, dst3.m_vector8.m_angular,
			off10[0], off10[1], off10[2], off10[3]);

		ndVector::Transpose4x4(
			dst4.m_vector8.m_linear, dst5.m_vector8.m_linear, dst6.m_vector8.m_linear, dst7.m_vector8.m_linear,
			off01[0], off01[1], off01[2], off01[3]);

		ndVector::Transpose4x4(
			dst4.m_vector8.m_angular, dst5.m_vector8.m_angular, dst6.m_vector8.m_angular, dst7.m_vector8.m_angular,
			src4.m_vector8.m_angular, src5.m_vector8.m_angular, src6.m_vector8.m_angular, src7.m_vector8.m_angular);
	}

	union
	{
		ndFloat32 m_float[ND_SIMD8_WORK_GROUP_SIZE];
		ndInt32 m_int[ND_SIMD8_WORK_GROUP_SIZE];
		struct
		{
			ndVector m_linear;
			ndVector m_angular;
		} m_vector8;
	};

	static ndVector8 m_one;
	static ndVector8 m_zero;
	static ndVector8 m_mask;
	static ndVector8 m_ordinals;
} D_GCC_NEWTON_CLASS_ALIGN_32;
#endif

#endif

