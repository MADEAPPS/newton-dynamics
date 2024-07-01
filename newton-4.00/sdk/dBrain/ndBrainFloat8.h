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

#ifndef _ND_BRAIN_FLOAT8_H__
#define _ND_BRAIN_FLOAT8_H__

#include "ndBrainStdafx.h"

class ndBrainFloat8 
{
	public: 
	ndBrainFloat8();
#ifdef D_NEWTON_USE_AVX2_OPTION
	ndBrainFloat8(const __m256 type);
#endif
	ndBrainFloat8(const ndBrainFloat a);
	ndBrainFloat8(const ndBrainFloat8& src);
	//ndBrainFloat8(const ndBrainFloat* const ptr);
	ndBrainFloat8(ndBrainFloat x0, ndBrainFloat x1, ndBrainFloat x2, ndBrainFloat x3,
				  ndBrainFloat x4, ndBrainFloat x5, ndBrainFloat x6, ndBrainFloat x7);
	~ndBrainFloat8();

	ndBrainFloat8& operator= (const ndBrainFloat8& A);

	ndBrainFloat8 operator+ (const ndBrainFloat8& A) const;
	ndBrainFloat8 operator- (const ndBrainFloat8& A) const;
	ndBrainFloat8 operator* (const ndBrainFloat8& A) const;

	ndBrainFloat HorizontalAdd() const;
	ndBrainFloat8 MulAdd(const ndBrainFloat8& A, const ndBrainFloat8& B) const;
	ndBrainFloat8 MulSub(const ndBrainFloat8& A, const ndBrainFloat8& B) const;

	union
	{
		ndBrainFloat m_f[8];
		#ifdef D_NEWTON_USE_AVX2_OPTION
		__m256 m_type;
		#endif
	};
};

// this class is only defined for avx instructions 
// this is because the neural net code make use of misaligned read and writes, 
// which cause segment fault when using SS, but not when using AVX.
#ifdef D_NEWTON_USE_AVX2_OPTION
inline ndBrainFloat8::ndBrainFloat8()
{
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat8& src)
	:m_type(src.m_type)
{
}

inline ndBrainFloat8::ndBrainFloat8(const __m256 type)
	:m_type(type)
{
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat a)
	:m_type(_mm256_set1_ps(a))
{
}

inline ndBrainFloat8::ndBrainFloat8(
	ndBrainFloat x0, ndBrainFloat x1, ndBrainFloat x2, ndBrainFloat x3,
	ndBrainFloat x4, ndBrainFloat x5, ndBrainFloat x6, ndBrainFloat x7)
{
	ndAssert(0);
	m_f[0] = x0;
	m_f[1] = x1;
	m_f[2] = x2;
	m_f[3] = x3;
	m_f[4] = x4;
	m_f[5] = x5;
	m_f[6] = x6;
	m_f[7] = x7;
}

inline ndBrainFloat8::~ndBrainFloat8()
{
}

inline ndBrainFloat8& ndBrainFloat8::operator= (const ndBrainFloat8& A)
{
	m_type = A.m_type;
	return *this;
}

inline ndBrainFloat8 ndBrainFloat8::operator+ (const ndBrainFloat8& A) const
{
	return _mm256_add_ps(m_type, A.m_type);
}

inline ndBrainFloat8 ndBrainFloat8::operator- (const ndBrainFloat8& A) const
{
	return _mm256_sub_ps(m_type, A.m_type);
}

inline ndBrainFloat8 ndBrainFloat8::operator* (const ndBrainFloat8& A) const
{
	return _mm256_mul_ps(m_type, A.m_type);
}

inline ndBrainFloat ndBrainFloat8::HorizontalAdd() const
{
	__m128 tmp0(_mm_add_ps(_mm256_castps256_ps128(m_type), _mm256_extractf128_ps(m_type, 1)));
	__m128 tmp1(_mm_add_ps(tmp0, _mm_shuffle_ps(tmp0, tmp0, PERMUTE_MASK(1, 0, 3, 2))));
	__m128 tmp2(_mm_add_ps(tmp1, _mm_shuffle_ps(tmp1, tmp1, PERMUTE_MASK(2, 3, 0, 1))));
	return _mm_cvtss_f32(tmp2);
}

inline ndBrainFloat8 ndBrainFloat8::MulAdd(const ndBrainFloat8& A, const ndBrainFloat8& B) const
{
	//return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
}

inline ndBrainFloat8 ndBrainFloat8::MulSub(const ndBrainFloat8& A, const ndBrainFloat8& B) const
{
	//return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	return _mm256_fnmadd_ps(A.m_type, B.m_type, m_type);
}

#else

inline ndBrainFloat8::ndBrainFloat8()
{
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat8& src)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = src.m_f[i];
	}
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat a)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = a;
	}
}

inline ndBrainFloat8::ndBrainFloat8(
	ndBrainFloat x0, ndBrainFloat x1, ndBrainFloat x2, ndBrainFloat x3,
	ndBrainFloat x4, ndBrainFloat x5, ndBrainFloat x6, ndBrainFloat x7)
{
	m_f[0] = x0;
	m_f[1] = x1;
	m_f[2] = x2;
	m_f[3] = x3;
	m_f[4] = x4;
	m_f[5] = x5;
	m_f[6] = x6;
	m_f[7] = x7;
}

inline ndBrainFloat8::~ndBrainFloat8()
{
}

inline ndBrainFloat8& ndBrainFloat8::operator= (const ndBrainFloat8& src)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = src.m_f[i];
	}
	return *this;
}

inline ndBrainFloat8 ndBrainFloat8::operator+ (const ndBrainFloat8& A) const
{
	return ndBrainFloat8(
		m_f[0] + A.m_f[0], m_f[1] + A.m_f[1], m_f[2] + A.m_f[2], m_f[3] + A.m_f[3],
		m_f[4] + A.m_f[4], m_f[5] + A.m_f[5], m_f[6] + A.m_f[6], m_f[7] + A.m_f[7]);
}

inline ndBrainFloat8 ndBrainFloat8::operator- (const ndBrainFloat8& A) const
{
	return ndBrainFloat8(
		m_f[0] - A.m_f[0], m_f[1] - A.m_f[1], m_f[2] - A.m_f[2], m_f[3] - A.m_f[3],
		m_f[4] - A.m_f[4], m_f[5] - A.m_f[5], m_f[6] - A.m_f[6], m_f[7] - A.m_f[7]);
}

inline ndBrainFloat8 ndBrainFloat8::operator* (const ndBrainFloat8& A) const
{
	return ndBrainFloat8(
		m_f[0] * A.m_f[0], m_f[1] * A.m_f[1], m_f[2] * A.m_f[2], m_f[3] * A.m_f[3],
		m_f[4] * A.m_f[4], m_f[5] * A.m_f[5], m_f[6] * A.m_f[6], m_f[7] * A.m_f[7]);
}

inline ndBrainFloat ndBrainFloat8::HorizontalAdd() const
{
	ndBrainFloat acc = 0.0f;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		acc += m_f[i];
	}
	return acc;
}

inline ndBrainFloat8 ndBrainFloat8::MulAdd(const ndBrainFloat8& A, const ndBrainFloat8& B) const
{
	return *this + A * B;
}

inline ndBrainFloat8 ndBrainFloat8::MulSub(const ndBrainFloat8& A, const ndBrainFloat8& B) const
{
	return *this - A * B;
}

//inline ndBrainFloat8 ndBrainFloat8::operator& (const ndBrainFloat8& data) const
//{
//	return ndBrainFloat8 (m_ix & data.m_ix, m_iy & data.m_iy, m_iz & data.m_iz, m_iw & data.m_iw);
//}
//
//inline ndBrainFloat8 ndBrainFloat8::operator| (const ndBrainFloat8& data) const
//{
//	return ndBrainFloat8(m_ix | data.m_ix, m_iy | data.m_iy, m_iz | data.m_iz, m_iw | data.m_iw);
//}
//
//inline ndBrainFloat8 ndBrainFloat8::operator> (const ndBrainFloat8& data) const
//{
//	return ndBrainFloat8((m_x <= data.m_x) - 1, (m_y <= data.m_y) - 1, (m_z <= data.m_z) - 1, (m_w <= data.m_w) - 1);
//}
//
//inline ndBrainFloat8 ndBrainFloat8::operator< (const ndBrainFloat8& data) const
//{
//	return ndBrainFloat8((m_x >= data.m_x) - 1, (m_y >= data.m_y) - 1, (m_z >= data.m_z) - 1, (m_w >= data.m_w) - 1);
//}

#endif
#endif
