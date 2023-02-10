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

#include "ndCudaStdafx.h"
#include "ndCudaUtils.h"
#include "ndCudaContext.h"
#include "ndCudaContextImplement.h"

#define D_CUDA_MIN_COMPUTE_CAP	610

void CudaSetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free);

ndCudaContext::ndCudaContext()
	:m_device (new ndCudaDevice)
	,m_implement(nullptr)
{
	int capability = m_device->m_prop.major * 100 + m_device->m_prop.minor * 10;
	// go as far back as 5.2 Maxwell GeForce GTX 960 or better.
	// go as far back as 6.1 Pascal GeForce GTX 1050 or better.
	if (capability >= D_CUDA_MIN_COMPUTE_CAP)
	{
		cudaError_t cudaStatus = cudaSetDevice(0);
		if (cudaStatus == cudaSuccess)
		{
			m_implement = new ndCudaContextImplement(m_device);
		}
	}
}

ndCudaContext::~ndCudaContext()
{
	if (m_implement)
	{
		delete m_implement;
	}
	delete m_device;
}

void* ndCudaContext::operator new (size_t size)
{
	return ndCudaMalloc(size);
}

void ndCudaContext::operator delete (void* ptr)
{
	ndCudaFree(ptr);
}

void ndCudaContext::SetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free)
{
	CudaSetMemoryAllocators(alloc, free);
}

bool ndCudaContext::IsValid() const 
{
	return m_implement ? true : false;
}

const char* ndCudaContext::GetStringId() const
{
	ndAssert(m_implement);
	return m_implement->GetStringId();
}

#if 0
ndCudaSpatialVector* ndCudaContext::GetTransformBuffer()
{
	return m_implement->GetTransformBuffer();
}


void ndCudaContext::ResizeBuffers(int size)
{
	m_implement->ResizeBuffers(size);
}

void ndCudaContext::LoadBodyData(const ndCudaBodyProxy* const src, int size)
{
	m_implement->LoadBodyData(src, size);
}

void ndCudaContext::ValidateContextBuffers()
{
	m_implement->ValidateContextBuffers();
}

void ndCudaContext::InitBodyArray()
{
	m_implement->InitBodyArray();
}

void ndCudaContext::IntegrateBodies(float timestep)
{
	m_implement->IntegrateBodies(timestep);
}

void ndCudaContext::IntegrateUnconstrainedBodies(float timestep)
{
	m_implement->IntegrateUnconstrainedBodies(timestep);
}

void ndCudaContext::UpdateTransform()
{
	m_implement->UpdateTransform();
}
#endif

void ndCudaContext::Begin()
{
	m_implement->Begin();
}

void ndCudaContext::End()
{
	m_implement->End();
}

void ndCudaContext::Cleanup()
{
	m_implement->Cleanup();
}

void ndCudaContext::PrepareCleanup()
{
	m_implement->PrepareCleanup();
}