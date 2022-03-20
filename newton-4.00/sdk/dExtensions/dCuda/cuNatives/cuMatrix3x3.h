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

#ifndef __ND_CUMATRIX3x3_H__
#define __ND_CUMATRIX3x3_H__

#include "cuVector3.h"

class cuMatrix3x3
{
	public:
	//inline __device__ cuMatrix3x3(float x, float y, float z)
	//	:m_x(x)
	//	,m_y(y)
	//	,m_z(z)
	//{
	//}
	
	inline __device__ cuMatrix3x3(const cuMatrix3x3& src)
		:m_front(src.m_front)
		,m_up(src.m_up)
		,m_right(src.m_right)
	{
	}

	inline __device__ cuMatrix3x3(const cuVector3& front, const cuVector3& up, const cuVector3& right)
		:m_front(front)
		,m_up(up)
		,m_right(right)
	{
	}
	
	//inline cuMatrix3x3(const ndVector& src)
	//	:m_x(src.m_x)
	//	,m_y(src.m_y)
	//	,m_z(src.m_z)
	//{
	//}
	//
	//
	//inline cuMatrix3x3 __device__ operator+ (const cuMatrix3x3& A) const
	//{
	//	return cuMatrix3x3(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z);
	//}

	cuVector3 m_front;
	cuVector3 m_up;
	cuVector3 m_right;
};

#endif