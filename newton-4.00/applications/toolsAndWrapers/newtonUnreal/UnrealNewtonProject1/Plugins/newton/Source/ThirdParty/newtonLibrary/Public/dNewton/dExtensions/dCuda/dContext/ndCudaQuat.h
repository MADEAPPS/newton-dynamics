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

#ifndef __ND_CUDA_QUAT_H__
#define __ND_CUDA_QUAT_H__

#include "ndCudaVector.h"
#include "ndCudaMatrix3x3.h"

class ndCudaQuat: public ndCudaVector
{
	public:
	inline __device__ __host__ ndCudaQuat()
		:ndCudaVector(0.0f, 0.0f, 0.0f, 1.0f)
	{
	}

	inline __device__ __host__ ndCudaQuat(float x, float y, float z, float w)
		:ndCudaVector(x, y, z, w)
	{
	}

	inline __device__ __host__ ndCudaQuat(const ndCudaQuat& src)
		:ndCudaVector(src)
	{
	}

	inline __device__ __host__ ndCudaQuat(const ndCudaVector& src)
		:ndCudaVector(src)
	{
	}

	inline __device__ __host__ ndCudaQuat(const ndCudaVector& unitAxis, float angle)
		:ndCudaVector()
	{
		angle = angle * 0.5f;
		w = cosf(angle);
		const float sinAng = sinf(angle);
		x = unitAxis.x * sinAng;
		y = unitAxis.y * sinAng;
		z = unitAxis.z * sinAng;
	}

	//inline ndCudaQuat(const ndVector& src)
	//	:ndCudaVector(src)
	//{
	//}

	inline ndCudaMatrix3x3 __device__ __host__ GetMatrix3x3 () const
	{
		//const ndCudaQuat quat0 = *this;
		const ndCudaQuat quat1 (Scale (2.0));
		
		const float x2 = x * quat1.x;
		const float y2 = y * quat1.y;
		const float z2 = z * quat1.z;

		const float xy = x * quat1.y;
		const float xz = x * quat1.z;
		const float xw = x * quat1.w;
		const float yz = y * quat1.z;
		const float yw = y * quat1.w;
		const float zw = z * quat1.w;

		const ndCudaVector front(1.0f - y2 - z2, xy + zw, xz - yw, 0.0f);
		const ndCudaVector up   (xy - zw, 1.0f - x2 - z2, yz + xw, 0.0f);
		const ndCudaVector right(xz + yw, yz - xw, 1.0f - x2 - y2, 0.0f);
		return ndCudaMatrix3x3(front, up, right);
	}

	inline ndCudaQuat __device__ __host__ Normalize() const
	{
		return ndCudaVector::Normalize();
	}

	inline ndCudaQuat __device__ __host__ operator* (const ndCudaQuat &q) const
	{
		const ndCudaVector x_( q.w,  q.z, -q.y, -q.x);
		const ndCudaVector y_(-q.z,  q.w,  q.x, -q.y);
		const ndCudaVector z_( q.y, -q.x,  q.w, -q.z);
		const ndCudaVector w_(q);
		return x_ * ndCudaVector(x) + y_ * ndCudaVector(y) + z_ * ndCudaVector(z) + w_ * ndCudaVector(w);
	}
};

#endif