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
	ndBrainFloat4(const __m128 type);
	ndBrainFloat4(const ndBrainFloat a);
	ndBrainFloat4(const ndBrainFloat4& src);
	ndBrainFloat4(const ndBrainFloat* const ptr);
	~ndBrainFloat4();

	ndBrainFloat4& operator= (const ndBrainFloat4& A);

	ndBrainFloat4 operator+ (const ndBrainFloat4& A) const;
	ndBrainFloat4 operator- (const ndBrainFloat4& A) const;
	ndBrainFloat4 operator* (const ndBrainFloat4& A) const;

	ndBrainFloat4 MulAdd(const ndBrainFloat4& A, const ndBrainFloat4& B) const;
	ndBrainFloat4 MulSub(const ndBrainFloat4& A, const ndBrainFloat4& B) const;

	union
	{
		ndBrainFloat m_f[4];
		ndInt32 m_i[4];
		__m128 m_type;
		__m128i m_typeInt;
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
	};
};

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

inline ndBrainFloat4 ndBrainFloat4::MulAdd(const ndBrainFloat4& A, const ndBrainFloat4& B) const
{
	return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
}

inline ndBrainFloat4 ndBrainFloat4::MulSub(const ndBrainFloat4& A, const ndBrainFloat4& B) const
{
	return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
}


#endif 
