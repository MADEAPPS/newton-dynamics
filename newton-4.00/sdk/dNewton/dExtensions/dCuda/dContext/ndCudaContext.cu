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

ndCudaContext::ndCudaContext()
	:m_device(new ndCudaDevice)
	,m_implement(nullptr)
{
	int campbility = m_device->m_prop.major * 100 + m_device->m_prop.minor;
	// go as far back as 5.2 Maxwell GeForce GTX 960 or better.
	if (campbility >= 600)
	//if ((cudaStatus == cudaSuccess) && (campbility >= 700))
	{
		cudaError_t cudaStatus = cudaSetDevice(0);
		m_implement = (cudaStatus == cudaSuccess) ? new ndCudaContextImplement(m_device) : nullptr;
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

bool ndCudaContext::IsValid() const 
{
	return m_implement ? true : false;
}

const char* ndCudaContext::GetStringId() const
{
	return IsValid() ? &m_device->m_prop.name[0] : "no cuda support";
}

double ndCudaContext::GetGPUTime() const
{
	return IsValid() ? m_implement->GetTimeInMilisecunds() : 0.0;
}

ndCudaSpatialVector* ndCudaContext::GetTransformBuffer0()
{
	return m_implement->GetTransformBuffer0();
}

ndCudaSpatialVector* ndCudaContext::GetTransformBuffer1()
{
	return m_implement->GetTransformBuffer1();
}

void ndCudaContext::Begin()
{
	m_implement->Begin();
}

void ndCudaContext::End()
{
	m_implement->End();
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