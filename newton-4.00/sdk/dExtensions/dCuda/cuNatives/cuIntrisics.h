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

#ifndef __ND_INTRISICS_H__
#define __ND_INTRISICS_H__

#include <cuda.h>
#include <cuda_runtime.h>

#define D_GRANULARITY			(1024 * 32)
#define D_THREADS_PER_BLOCK		256

template <class T>
inline T __device__ cuAbs(T A)
{
	// according to Intel this is better because is does not read after write
	return (A >= T(0)) ? A : -A;
}

template <class T>
inline void __device__ cuSwap(T& A, T& B)
{
	T tmp(A);
	A = B;
	B = tmp;
}

#endif