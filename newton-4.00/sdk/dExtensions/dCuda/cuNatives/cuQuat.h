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

#ifndef __ND_CUQUAT_H__
#define __ND_CUQUAT_H__

#include "cuVector4.h"
#include "cuMatrix3x3.h"

class cuQuat: public cuVector4
{
	public:
	inline __device__ cuQuat()
		:cuVector4(0, 0, 0, 1)
	{
	}

	inline __device__ cuQuat(float x, float y, float z, float w)
		:cuVector4(x, y, z, w)
	{
	}

	inline __device__ cuQuat(const cuQuat& src)
		:cuVector4(src)
	{
	}

	inline __device__ cuQuat(const cuVector4& src)
		:cuVector4(src)
	{
	}

	inline __device__ cuQuat(const cuVector3& unitAxis, float angle)
		:cuVector4()
	{
		angle = angle * float(0.5f);
		m_w = cos(angle);
		float sinAng = sin(angle);

		m_x = unitAxis.m_x * sinAng;
		m_y = unitAxis.m_y * sinAng;
		m_z = unitAxis.m_z * sinAng;
	}

	inline cuQuat(const ndVector& src)
		:cuVector4(src)
	{
	}

	inline cuMatrix3x3 __device__ GetMatrix3x3 () const
	{
		//const dQuaternion quat0(quat);
		//const dQuaternion quat1(quat0.Scale (float(2.0f)));
		const cuQuat quat0 = *this;
		const cuQuat quat1 (quat0.Scale (2.0));
		
		float x2 = quat0.m_x * quat1.m_x;
		float y2 = quat0.m_y * quat1.m_y;
		float z2 = quat0.m_z * quat1.m_z;

		float xy = quat0.m_x * quat1.m_y;
		float xz = quat0.m_x * quat1.m_z;
		float xw = quat0.m_x * quat1.m_w;
		float yz = quat0.m_y * quat1.m_z;
		float yw = quat0.m_y * quat1.m_w;
		float zw = quat0.m_z * quat1.m_w;

		const cuVector3 front(1.0f - y2 - z2, xy + zw, xz - yw);
		const cuVector3 up   (xy - zw, 1.0f - x2 - z2, yz + xw);
		const cuVector3 right(xz + yw, yz - xw, 1.0f - x2 - y2);
		return cuMatrix3x3(front, up, right);
	}

	inline cuQuat __device__ Normalize() const
	{
		return cuVector4::Normalize();
	}

	inline cuQuat __device__ operator* (const cuQuat &q) const
	{
		const cuVector4 x( q.m_w,  q.m_z, -q.m_y, -q.m_x);
		const cuVector4 y(-q.m_z,  q.m_w,  q.m_x, -q.m_y);
		const cuVector4 z( q.m_y, -q.m_x,  q.m_w, -q.m_z);
		const cuVector4 w(q);
		return x * cuVector4(m_x) + y * cuVector4(m_y) + z * cuVector4(m_z) + w * cuVector4(m_w);
	}
};

#endif