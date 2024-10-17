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

#ifndef __ND_CUDA_INTRINSICS_H__
#define __ND_CUDA_INTRINSICS_H__

#include <cuda.h>
#include <cuda_runtime.h>

#define D_GRANULARITY			(1024 * 64)
#define D_LOG_BANK_COUNT_GPU	5
#define D_BANK_COUNT_GPU		(1<<D_LOG_BANK_COUNT_GPU)

template <class T>
inline T __device__ __host__ cuAbs(T A)
{
	return fabsf(A);
}

template <class T>
inline T __device__ __host__ cuFloor(T A)
{
	return floorf(A);
}

template <class T>
inline T __device__ __host__ cuMax(T A, T B)
{
	return fmaxf(A, B);
}

template <class T>
inline T __device__ __host__ cuMin(T A, T B)
{
	return fminf(A, B);
}

template <class T>
inline T __device__ __host__ cuSelect(bool test, T A, T B)
{
	return test ? A : B;
}

template <class T>
inline void __device__ __host__ cuSwap(T& A, T& B)
{
	T tmpA(A);
	T tmpB(B);
	A = tmpB;
	B = tmpA;
}

template <typename T, int size>
class cuBankFreeArray
{
	public:
	__device__ cuBankFreeArray()
	{
	}

	__device__ T& operator[] (int address)
	{
		int low = address & (D_BANK_COUNT_GPU - 1);
		int high = address >> D_LOG_BANK_COUNT_GPU;
		int dst = high * (D_BANK_COUNT_GPU + 1) + low;
		return m_array[dst];
	}

	T m_array[((size + D_BANK_COUNT_GPU - 1) >> D_LOG_BANK_COUNT_GPU) * (D_BANK_COUNT_GPU + 1)];
};

#endif