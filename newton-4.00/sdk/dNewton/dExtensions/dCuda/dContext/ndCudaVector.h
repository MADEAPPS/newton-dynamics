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

#ifndef __ND_CUDA_VECTOR_H__
#define __ND_CUDA_VECTOR_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>
#include "ndCudaIntrinsics.h"

class ndCudaVector: public float4
{
	public:
	inline __device__ __host__ ndCudaVector()
	{
	}

	inline __device__ __host__ ndCudaVector(float val)
	{
		x = val;
		y = val;
		z = val;
		w = val;
	}

	inline __device__ __host__ ndCudaVector(float val_x, float val_y, float val_z, float val_w)
	{
		x = val_x;
		y = val_y;
		z = val_z;
		w = val_w;
	}

	inline __device__ __host__ ndCudaVector(const ndCudaVector& src)
	{
		x = src.x;
		y = src.y;
		z = src.z;
		w = src.w;
	}

	//inline ndCudaVector(const ndVector& src)
	//{
	//	x = src.m_x;
	//	y = src.m_y;
	//	z = src.m_z;
	//	w = src.m_w;
	//}
	//
	//inline ndVector ToNdVector(const ndCudaVector& src) const
	//{
	//	return ndVector(src.x, src.y, src.z, src.w);
	//}

	inline float __device__ __host__ GetElement(int i) const
	{
		return (&this->x)[i];
	}

	inline void __device__ __host__ SetElement(int i, float val)
	{
		(&this->x)[i] = val;
	}
	
	inline ndCudaVector __device__ __host__ operator+ (const ndCudaVector& A) const
	{
		return ndCudaVector(x + A.x, y + A.y, z + A.z, w + A.w);
	}

	inline ndCudaVector __device__ __host__ operator- (const ndCudaVector& A) const
	{
		return ndCudaVector(x - A.x, y - A.y, z - A.z, w - A.w);
	}

	inline ndCudaVector __device__ __host__ operator* (const ndCudaVector& A) const
	{
		return ndCudaVector(x * A.x, y * A.y, z * A.z, w * A.w);
	}

	inline ndCudaVector __device__ __host__ operator> (const ndCudaVector& A) const
	{
		return ndCudaVector(
			cuSelect(x > A.x, 1.0f, 0.0f),
			cuSelect(y > A.y, 1.0f, 0.0f),
			cuSelect(z > A.z, 1.0f, 0.0f),
			cuSelect(w > A.w, 1.0f, 0.0f));
	}

	inline ndCudaVector __device__ __host__ operator< (const ndCudaVector& A) const
	{
		return ndCudaVector(
			cuSelect(x < A.x, 1.0f, 0.0f),
			cuSelect(y < A.y, 1.0f, 0.0f),
			cuSelect(z < A.z, 1.0f, 0.0f),
			cuSelect(w < A.w, 1.0f, 0.0f));
	}

	inline ndCudaVector __device__ __host__ Abs() const
	{
		return ndCudaVector(cuAbs(x), cuAbs(y), cuAbs(z), cuAbs(w));
	}

	inline ndCudaVector __device__ __host__ Select(const ndCudaVector& test, const ndCudaVector& A) const
	{
		return ndCudaVector(*this * test + A * (ndCudaVector(1.0f) - test));
	}

	inline ndCudaVector __device__ __host__ Min (const ndCudaVector& A) const
	{
		return ndCudaVector(cuMin(x, A.x), cuMin(y, A.y), cuMin(z, A.z), cuMin(w, A.w));
	}

	inline ndCudaVector __device__ __host__ Max(const ndCudaVector& A) const
	{
		return ndCudaVector(cuMax(x, A.x), cuMax(y, A.y), cuMax(z, A.z), cuMax(w, A.w));
	}

	inline ndCudaVector __device__ __host__ Floor() const
	{
		return ndCudaVector(cuFloor(x), cuFloor(y), cuFloor(z), cuFloor(w));
	}

	inline ndCudaVector __device__ __host__ Scale(float s) const
	{
		return ndCudaVector(x * s, y * s, z * s, w * s);
	}

	inline float __device__ __host__ AddHorizontal() const
	{
		return x + y + z + w;
	}

	inline float __device__ __host__ DotProduct(const ndCudaVector& B) const
	{
		return (*this * B).AddHorizontal();
	}

	inline ndCudaVector __device__ __host__ CrossProduct(const ndCudaVector& B) const
	{
		return ndCudaVector(
			y * B.z - z * B.y,
			z * B.x - x * B.z,
			x * B.y - y * B.x,
			w);
	}

	inline ndCudaVector __device__ __host__ Normalize() const
	{
		float den = 1.0f / sqrtf(DotProduct(*this));
		return ndCudaVector(x * den, y * den, z * den, w * den);
	}
};

#endif