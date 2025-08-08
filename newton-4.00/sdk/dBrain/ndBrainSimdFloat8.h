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

#ifndef _ND_BRAIN_FLOAT4_H__
#define _ND_BRAIN_FLOAT4_H__

#include "ndBrainStdafx.h"

// Minimal simd support.
// this class is only defined for avx instructions 
// this is because the neural net code make use of misaligned read and writes, 
// which cause segment fault when using SS, but not when using AVX.
// the second reason it is the only way I found to make use of some 
// simd intrinsics in visual studio, like tanh, which is expensive.
 
// the good part is that visual studio 2022 is capable of generate the simd version of the class without having to 
// use explicit simd code. 
// This is much better since is let the compiler do other optimizations.

D_MSV_NEWTON_CLASS_ALIGN_32
class ndBrainSimdFloat8 
{
	public: 
	ndBrainSimdFloat8();
	ndBrainSimdFloat8(const ndBrainFloat a);
	ndBrainSimdFloat8(const ndBrainFloat* const ptr);

	void Store(ndBrainFloat* const ptr) const;

	ndBrainSimdFloat8 Tanh() const;
	ndBrainSimdFloat8 Min(const ndBrainSimdFloat8& src) const;
	ndBrainSimdFloat8 Max(const ndBrainSimdFloat8& src) const;
	ndBrainSimdFloat8 Clamp(const ndBrainSimdFloat8& min, const ndBrainSimdFloat8& max) const;
	
	ndBrainSimdFloat8 operator+ (const ndBrainSimdFloat8& A) const;
	ndBrainSimdFloat8 operator- (const ndBrainSimdFloat8& A) const;
	ndBrainSimdFloat8 operator* (const ndBrainSimdFloat8& A) const;
	
	// logical operations;
	ndBrainSimdFloat8 operator~ () const;
	ndBrainSimdFloat8 operator& (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator| (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator> (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator< (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator>= (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator<= (const ndBrainSimdFloat8& data) const;

	union
	{
		ndBrainFloat m_f[8];
		ndInt32 m_i[8];
	};

} D_GCC_NEWTON_CLASS_ALIGN_32;

inline ndBrainSimdFloat8::ndBrainSimdFloat8()
{
}

inline ndBrainSimdFloat8::ndBrainSimdFloat8(const ndBrainFloat a)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = a;
	}
}

inline ndBrainSimdFloat8::ndBrainSimdFloat8(const ndBrainFloat* const ptr)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = ptr[i];
	}
}

inline void ndBrainSimdFloat8::Store(ndBrainFloat* const ptr) const
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		ptr[i] = m_f[i];
	}
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Clamp(const ndBrainSimdFloat8& min, const ndBrainSimdFloat8& max) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndClamp(m_f[i], min.m_f[i], max.m_f[i]);
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Min(const ndBrainSimdFloat8& min) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndMin(m_f[i], min.m_f[i]);
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Max(const ndBrainSimdFloat8& max) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndMax(m_f[i], max.m_f[i]);
		ndAssert(tmp.m_f[i] <= ndFloat32(1000.0f));
		ndAssert(tmp.m_f[i] >= ndFloat32(-1000.0f));
	}
	return tmp;

}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Tanh() const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndBrainFloat(ndTanh(m_f[i]));
		ndAssert(tmp.m_f[i] <= ndFloat32(1000.0f));
		ndAssert(tmp.m_f[i] >= ndFloat32(-1000.0f));
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator+ (const ndBrainSimdFloat8& A) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = m_f[i] + A.m_f[i];
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator- (const ndBrainSimdFloat8& A) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = m_f[i] - A.m_f[i];
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator* (const ndBrainSimdFloat8& A) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = m_f[i] * A.m_f[i];
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator> (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] > data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator>= (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] >= data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator< (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] < data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator<= (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] <= data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator~ () const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = ~m_i[i];
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator| (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_i[i] | data.m_i[i];
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator& (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_i[i] & data.m_i[i];
	}
	return tmp;
}

#endif 
