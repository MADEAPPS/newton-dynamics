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

class ndBrainFloat4 
{
	public: 
	ndBrainFloat4();
#ifdef D_NEWTON_USE_AVX2_OPTION
	ndBrainFloat4(const __m128 type);
#endif
	ndBrainFloat4(const ndBrainFloat a);
	ndBrainFloat4(const ndBrainFloat4& src);
	ndBrainFloat4(const ndBrainFloat* const ptr);
	ndBrainFloat4(ndInt32 x, ndInt32 y, ndInt32 z, ndInt32 w);
	ndBrainFloat4(ndBrainFloat x, ndBrainFloat y, ndBrainFloat z, ndBrainFloat w);
	~ndBrainFloat4();

	ndBrainFloat4& operator= (const ndBrainFloat4& A);

	ndBrainFloat4 operator+ (const ndBrainFloat4& A) const;
	ndBrainFloat4 operator- (const ndBrainFloat4& A) const;
	ndBrainFloat4 operator* (const ndBrainFloat4& A) const;

	ndBrainFloat HorizontalAdd() const;
	ndBrainFloat4 MulAdd(const ndBrainFloat4& A, const ndBrainFloat4& B) const;
	ndBrainFloat4 MulSub(const ndBrainFloat4& A, const ndBrainFloat4& B) const;

	// logical operations;
	ndBrainFloat4 operator& (const ndBrainFloat4& data) const;
	ndBrainFloat4 operator| (const ndBrainFloat4& data) const;
	ndBrainFloat4 operator> (const ndBrainFloat4& data) const;
	ndBrainFloat4 operator< (const ndBrainFloat4& data) const;

	union
	{
		ndBrainFloat m_f[4];
		ndInt32 m_i[4];
		struct
		{
			ndBrainFloat m_x;
			ndBrainFloat m_y;
			ndBrainFloat m_z;
			ndBrainFloat m_w;
		};
		struct
		{
			ndInt32 m_ix;
			ndInt32 m_iy;
			ndInt32 m_iz;
			ndInt32 m_iw;
		};

		#ifdef D_NEWTON_USE_AVX2_OPTION
		__m128 m_type;
		__m128i m_typeInt;
		#endif
	};
};

// this class is only defined for avx instructions 
// this is because the neural net code make use of misaligned read and writes, 
// which cause segment fault when using SS, but not when using AVX.
#ifdef D_NEWTON_USE_AVX2_OPTION
inline ndBrainFloat4::ndBrainFloat4()
{
}

inline ndBrainFloat4::ndBrainFloat4(const ndBrainFloat4& src)
	:m_type(src.m_type)
{
}

inline ndBrainFloat4::ndBrainFloat4(const __m128 type)
	:m_type(type)
{
}

inline ndBrainFloat4::ndBrainFloat4(const ndBrainFloat a)
	:m_type(_mm_set1_ps(a))
{
}

inline ndBrainFloat4::ndBrainFloat4(ndInt32 x, ndInt32 y, ndInt32 z, ndInt32 w)
	:m_typeInt(_mm_set_epi32(w, z, y, x))
{
	ndAssert(0);
}

inline ndBrainFloat4::ndBrainFloat4(ndBrainFloat x, ndBrainFloat y, ndBrainFloat z, ndBrainFloat w)
	:m_type(_mm_set_ps(w, z, y, x))
{
	ndAssert(0);
}

inline ndBrainFloat4::ndBrainFloat4(const ndBrainFloat* const ptr)
	:m_type(_mm_loadu_ps(ptr))
{
}

inline ndBrainFloat4::~ndBrainFloat4()
{
}

inline ndBrainFloat4& ndBrainFloat4::operator= (const ndBrainFloat4& A)
{
	m_type = A.m_type;
	return *this;
}

inline ndBrainFloat4 ndBrainFloat4::operator+ (const ndBrainFloat4& A) const
{
	return _mm_add_ps(m_type, A.m_type);
}

inline ndBrainFloat4 ndBrainFloat4::operator- (const ndBrainFloat4& A) const
{
	return _mm_sub_ps(m_type, A.m_type);
}

inline ndBrainFloat4 ndBrainFloat4::operator* (const ndBrainFloat4& A) const
{
	return _mm_mul_ps(m_type, A.m_type);
}

inline ndBrainFloat ndBrainFloat4::HorizontalAdd() const
{
	__m128 tmp(_mm_hadd_ps(m_type, m_type));
	return _mm_cvtss_f32(_mm_hadd_ps(tmp, tmp));
}

inline ndBrainFloat4 ndBrainFloat4::MulAdd(const ndBrainFloat4& A, const ndBrainFloat4& B) const
{
	return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
}

inline ndBrainFloat4 ndBrainFloat4::MulSub(const ndBrainFloat4& A, const ndBrainFloat4& B) const
{
	return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
}

inline ndBrainFloat4 ndBrainFloat4::operator& (const ndBrainFloat4& data) const
{
	return _mm_and_ps(m_type, data.m_type);
}

inline ndBrainFloat4 ndBrainFloat4::operator| (const ndBrainFloat4& data) const
{
	return _mm_or_ps(m_type, data.m_type);
}

inline ndBrainFloat4 ndBrainFloat4::operator> (const ndBrainFloat4& data) const
{
	return _mm_cmpgt_ps(m_type, data.m_type);
}

inline ndBrainFloat4 ndBrainFloat4::operator< (const ndBrainFloat4& data) const
{
	return _mm_cmplt_ps(m_type, data.m_type);
}

#else

inline ndBrainFloat4::ndBrainFloat4()
{
}

inline ndBrainFloat4::ndBrainFloat4(const ndBrainFloat4& src)
	:m_x(src.m_x), m_y(src.m_y), m_z(src.m_z), m_w(src.m_w)
{
}

inline ndBrainFloat4::ndBrainFloat4(const ndBrainFloat a)
	:m_x(a), m_y(a), m_z(a), m_w(a)
{
}

inline ndBrainFloat4::ndBrainFloat4(ndInt32 x, ndInt32 y, ndInt32 z, ndInt32 w)
	:m_ix(x), m_iy(y), m_iz(z), m_iw(w)
{

}

inline ndBrainFloat4::ndBrainFloat4(ndBrainFloat x, ndBrainFloat y, ndBrainFloat z, ndBrainFloat w)
	:m_x(x), m_y(y), m_z(z), m_w(w)
{
}

inline ndBrainFloat4::ndBrainFloat4(const ndBrainFloat* const ptr)
	: m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(ptr[3])
{
}

inline ndBrainFloat4::~ndBrainFloat4()
{
}

inline ndBrainFloat4& ndBrainFloat4::operator= (const ndBrainFloat4& src)
{
	m_x = src.m_x;
	m_y = src.m_y;
	m_z = src.m_z;
	m_w = src.m_w;
	return *this;
}

inline ndBrainFloat4 ndBrainFloat4::operator+ (const ndBrainFloat4& A) const
{
	return ndBrainFloat4(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
}

inline ndBrainFloat4 ndBrainFloat4::operator- (const ndBrainFloat4& A) const
{
	return ndBrainFloat4(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
}

inline ndBrainFloat4 ndBrainFloat4::operator* (const ndBrainFloat4& A) const
{
	return ndBrainFloat4(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
}

inline ndBrainFloat ndBrainFloat4::HorizontalAdd() const
{
	return m_x + m_y + m_z + m_w;
}

inline ndBrainFloat4 ndBrainFloat4::MulAdd(const ndBrainFloat4& A, const ndBrainFloat4& B) const
{
	return *this + A * B;
}

inline ndBrainFloat4 ndBrainFloat4::MulSub(const ndBrainFloat4& A, const ndBrainFloat4& B) const
{
	return *this - A * B;
}

inline ndBrainFloat4 ndBrainFloat4::operator& (const ndBrainFloat4& data) const
{
	return ndBrainFloat4 (m_ix & data.m_ix, m_iy & data.m_iy, m_iz & data.m_iz, m_iw & data.m_iw);
}

inline ndBrainFloat4 ndBrainFloat4::operator| (const ndBrainFloat4& data) const
{
	return ndBrainFloat4(m_ix | data.m_ix, m_iy | data.m_iy, m_iz | data.m_iz, m_iw | data.m_iw);
}

inline ndBrainFloat4 ndBrainFloat4::operator> (const ndBrainFloat4& data) const
{
	return ndBrainFloat4((m_x <= data.m_x) - 1, (m_y <= data.m_y) - 1, (m_z <= data.m_z) - 1, (m_w <= data.m_w) - 1);
}

inline ndBrainFloat4 ndBrainFloat4::operator< (const ndBrainFloat4& data) const
{
	return ndBrainFloat4((m_x >= data.m_x) - 1, (m_y >= data.m_y) - 1, (m_z >= data.m_z) - 1, (m_w >= data.m_w) - 1);
}

#endif


#endif 
