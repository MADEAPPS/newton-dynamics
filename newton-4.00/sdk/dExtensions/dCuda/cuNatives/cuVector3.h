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

#ifndef __ND_CUVECTOR3_H__
#define __ND_CUVECTOR3_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>
#include <device_launch_parameters.h>

class cuVector3
{
	public:
	inline __device__ cuVector3()
	{
	}

	inline __device__ cuVector3(float x)
		:m_x(x)
		,m_y(x)
		,m_z(x)
	{
	}

	inline __device__ cuVector3(float x, float y, float z)
		:m_x(x)
		,m_y(y)
		,m_z(z)
	{
	}

	inline __device__ cuVector3(const cuVector3& src)
		:m_x(src.m_x)
		,m_y(src.m_y)
		,m_z(src.m_z)
	{
	}

	inline cuVector3(const ndVector& src)
		:m_x(src.m_x)
		,m_y(src.m_y)
		,m_z(src.m_z)
	{
	}

	inline float __device__ GetElement (int i) const
	{
		return (&m_x)[i];
	}

	inline void __device__ SetElement(int i, float val)
	{
		(&m_x)[i] = val;
	}

	inline float __device__ AddHorizontal() const
	{
		return m_x + m_y + m_z;
	}

	inline cuVector3 __device__ Scale (float s) const
	{
		return cuVector3(m_x * s, m_y * s , m_z * s);
	}

	inline cuVector3 __device__ CrossProduct(const cuVector3& B) const
	{
		return cuVector3(m_y * B.m_z - m_z * B.m_y,
						 m_z * B.m_x - m_x * B.m_z,
						 m_x * B.m_y - m_y * B.m_x);
	}

	inline float __device__ DotProduct(const cuVector3& B) const
	{
		return (*this * B).AddHorizontal();
	}

	inline cuVector3 __device__ operator+ (const cuVector3& A) const
	{
		return cuVector3(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z);
	}

	inline cuVector3 __device__ operator- (const cuVector3& A) const
	{
		return cuVector3(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z);
	}

	inline cuVector3 __device__ operator* (const cuVector3& A) const
	{
		return cuVector3(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z);
	}

	float m_x;
	float m_y;
	float m_z;
};

#endif