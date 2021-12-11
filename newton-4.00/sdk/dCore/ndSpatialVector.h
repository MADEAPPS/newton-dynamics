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

#ifndef __ND_SPATIAL_VECTOR_H__
#define __ND_SPATIAL_VECTOR_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"

D_MSV_NEWTON_ALIGN_32
class ndSpatialVector
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	inline ndSpatialVector()
	{
	}

	inline ndSpatialVector(const dFloat64 a)
		:m_data(a)
	{
	}

	inline ndSpatialVector(const ndSpatialVector& copy)
		:m_data(copy.m_data)
	{
	}

	inline ndSpatialVector(const ndBigVector& low, const ndBigVector& high)
		:m_data(low, high)
	{
	}

	inline ~ndSpatialVector()
	{
	}

	inline dFloat64& operator[] (dInt32 i)
	{
		dAssert(i >= 0);
		dAssert(i < dInt32(sizeof(m_f) / sizeof(m_f[0])));
		return ((dFloat64*)&m_f)[i];
	}

	inline const dFloat64& operator[] (dInt32 i) const
	{
		dAssert(i >= 0);
		dAssert(i < dInt32 (sizeof(m_f) / sizeof(m_f[0])));
		return ((dFloat64*)&m_f)[i];
	}

	inline ndSpatialVector operator+ (const ndSpatialVector& A) const
	{
		return ndSpatialVector(m_data.m_low + A.m_data.m_low, m_data.m_high + A.m_data.m_high);
	}

	inline ndSpatialVector operator*(const ndSpatialVector& A) const
	{
		return ndSpatialVector(m_data.m_low * A.m_data.m_low, m_data.m_high * A.m_data.m_high);
	}

	inline dFloat64 DotProduct(const ndSpatialVector& v) const
	{
		dAssert(m_f[6] == dFloat32(0.0f));
		dAssert(m_f[7] == dFloat32(0.0f));
		ndBigVector tmp(m_data.m_low * v.m_data.m_low + m_data.m_high * v.m_data.m_high);
		return tmp.AddHorizontal().GetScalar();
	}

	inline ndSpatialVector Scale(dFloat64 s) const
	{
		ndBigVector tmp(s);
		return ndSpatialVector(m_data.m_low * tmp, m_data.m_high * tmp);
	}

	struct ndData
	{
		inline ndData(const dFloat64 a)
			:m_low(a)
			,m_high(a)
		{
		}

		inline ndData(const ndData& data)
			:m_low(data.m_low)
			,m_high(data.m_high)
		{
		}

		inline ndData(const ndBigVector& low, const ndBigVector& high)
			:m_low(low)
			,m_high(high)
		{
		}

		ndBigVector m_low;
		ndBigVector m_high;
	};

	union
	{
		dFloat64 m_f[8];
		ndData m_data;
	};

	D_CORE_API static ndSpatialVector m_zero;
} D_GCC_NEWTON_ALIGN_32;

#endif
