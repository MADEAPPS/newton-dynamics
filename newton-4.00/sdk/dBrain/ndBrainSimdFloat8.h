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

class ndBrainSimdFloat8 
{
	public: 
	ndBrainSimdFloat8();
	ndBrainSimdFloat8(const ndBrainFloat a);

	ndBrainSimdFloat8 Tanh() const;
	ndBrainSimdFloat8 Min(const ndBrainSimdFloat8& src) const;
	ndBrainSimdFloat8 Max(const ndBrainSimdFloat8& src) const;
	ndBrainSimdFloat8 Clamp(const ndBrainSimdFloat8& min, const ndBrainSimdFloat8& max) const;

	//ndBrainSimdFloat8(const ndBrainFloat* const ptr);
	//ndBrainSimdFloat8(ndInt32 x, ndInt32 y, ndInt32 z, ndInt32 w);
	//ndBrainSimdFloat8(ndBrainFloat x, ndBrainFloat y, ndBrainFloat z, ndBrainFloat w);
	//~ndBrainSimdFloat8();
	//
	//ndBrainSimdFloat8& operator= (const ndBrainSimdFloat8& A);
	
	ndBrainSimdFloat8 operator+ (const ndBrainSimdFloat8& A) const;
	ndBrainSimdFloat8 operator- (const ndBrainSimdFloat8& A) const;
	//ndBrainSimdFloat8 operator* (const ndBrainSimdFloat8& A) const;
	//
	//ndBrainFloat HorizontalAdd() const;
	//ndBrainSimdFloat8 Min(const ndBrainSimdFloat8& A) const;
	//ndBrainSimdFloat8 Max(const ndBrainSimdFloat8& A) const;
	//ndBrainSimdFloat8 MulAdd(const ndBrainSimdFloat8& A, const ndBrainSimdFloat8& B) const;
	//ndBrainSimdFloat8 MulSub(const ndBrainSimdFloat8& A, const ndBrainSimdFloat8& B) const;
	
	// logical operations;
	ndBrainSimdFloat8 operator& (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator| (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator> (const ndBrainSimdFloat8& data) const;
	ndBrainSimdFloat8 operator< (const ndBrainSimdFloat8& data) const;

//#ifdef D_NEWTON_USE_AVX2_OPTION
#if 0
	ndBrainSimdFloat8(const __m256 type);
	union
	{
		ndBrainFloat m_f[8];

		__m256 m_type;
	};
#else
	union
	{
		ndBrainFloat m_f[8];
		ndInt32 m_i[8];
	};
#endif
};

inline ndBrainSimdFloat8::ndBrainSimdFloat8()
{
}

//#ifdef D_NEWTON_USE_AVX2_OPTION
#if 0
inline ndBrainSimdFloat8::ndBrainSimdFloat8(const __m256 type)
	:m_type(type)
{
}

inline ndBrainSimdFloat8::ndBrainSimdFloat8(const ndBrainFloat a)
	:m_type(_mm256_set1_ps(a))
{
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Max(const ndBrainSimdFloat8& src) const
{
	return _mm256_max_ps(m_type, src.m_type);
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Clamp(const ndBrainSimdFloat8& min, const ndBrainSimdFloat8& max) const
{
	return _mm256_max_ps(_mm256_min_ps(m_type, max.m_type), min.m_type);
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Tanh() const
{
	return _mm256_tanh_ps(m_type);
}

//inline ndBrainSimdFloat8::ndBrainSimdFloat8(ndInt32 x, ndInt32 y, ndInt32 z, ndInt32 w)
//	:m_typeInt(_mm_set_epi32(w, z, y, x))
//{
//	ndAssert(0);
//}
//
//inline ndBrainSimdFloat8::ndBrainSimdFloat8(ndBrainFloat x, ndBrainFloat y, ndBrainFloat z, ndBrainFloat w)
//	:m_type(_mm_set_ps(w, z, y, x))
//{
//	ndAssert(0);
//}
//
//inline ndBrainSimdFloat8::ndBrainSimdFloat8(const ndBrainFloat* const ptr)
//	:m_type(_mm_loadu_ps(ptr))
//{
//}
//
//inline ndBrainSimdFloat8::~ndBrainSimdFloat8()
//{
//}
//
//inline ndBrainSimdFloat8& ndBrainSimdFloat8::operator= (const ndBrainSimdFloat8& A)
//{
//	m_type = A.m_type;
//	return *this;
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator+ (const ndBrainSimdFloat8& A) const
//{
//	return _mm_add_ps(m_type, A.m_type);
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator- (const ndBrainSimdFloat8& A) const
//{
//	return _mm_sub_ps(m_type, A.m_type);
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator* (const ndBrainSimdFloat8& A) const
//{
//	return _mm_mul_ps(m_type, A.m_type);
//}
//
//inline ndBrainFloat ndBrainSimdFloat8::HorizontalAdd() const
//{
//	__m128 tmp(_mm_hadd_ps(m_type, m_type));
//	return _mm_cvtss_f32(_mm_hadd_ps(tmp, tmp));
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::MulAdd(const ndBrainSimdFloat8& A, const ndBrainSimdFloat8& B) const
//{
//	return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::MulSub(const ndBrainSimdFloat8& A, const ndBrainSimdFloat8& B) const
//{
//	return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator& (const ndBrainSimdFloat8& data) const
//{
//	return _mm_and_ps(m_type, data.m_type);
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator| (const ndBrainSimdFloat8& data) const
//{
//	return _mm_or_ps(m_type, data.m_type);
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator> (const ndBrainSimdFloat8& data) const
//{
//	return _mm_cmpgt_ps(m_type, data.m_type);
//}
//
//inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator< (const ndBrainSimdFloat8& data) const
//{
//	return _mm_cmplt_ps(m_type, data.m_type);
//}

#else

inline ndBrainSimdFloat8::ndBrainSimdFloat8(const ndBrainFloat a)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = a;
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

inline ndBrainSimdFloat8 ndBrainSimdFloat8::Tanh() const
{
	ndBrainSimdFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndTanh(m_f[i]);
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

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator> (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	const ndBrainSimdFloat8 diff(data - *this);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = diff.m_i[i] >> 31;
	}
	return tmp;
}

inline ndBrainSimdFloat8 ndBrainSimdFloat8::operator< (const ndBrainSimdFloat8& data) const
{
	ndBrainSimdFloat8 tmp;
	ndBrainSimdFloat8 diff (*this - data);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = diff.m_i[i] >> 31;
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


#endif 
