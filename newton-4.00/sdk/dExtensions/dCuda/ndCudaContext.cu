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

#include "ndCudaContext.h"

ndCudaContext::ndCudaContext()
	:ndClassAlloc()
	,m_bodyBuffer()
{
	cudaError_t cudaStatus;
	cudaStatus = cudaGetDeviceProperties(&m_prop, 0);
	dAssert(cudaStatus == cudaSuccess);

	cudaStatus = cudaSetDevice(0);
	dAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}

ndCudaContext::~ndCudaContext()
{
}

ndCudaContext* ndCudaContext::CreateContext()
{
	cudaError_t cudaStatus = cudaSetDevice(0);
	return (cudaStatus == cudaSuccess) ? new ndCudaContext() : nullptr;
}
