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

#include <ndSyclStdafx.h>
#include "ndSyclUtils.h"
#include "ndSyclContextImpl.h"

using namespace sycl;

ndSyclContextImpl::ndSyclContextImpl(sycl::device device)
	:m_device(device)
	,m_queue(device)
{
	std::string name(m_device.get_info<info::device::name>());
	memset(m_deviceName, 0, sizeof(m_deviceName));
	strncpy(m_deviceName, name.c_str(), name.size());
}

ndSyclContextImpl::~ndSyclContextImpl()
{
}

const char* ndSyclContextImpl::GetStringId() const
{
	return m_deviceName;
}

#if 0
double ndCudaContext::GetGPUTime() const
{
	return IsValid() ? m_implement->GetTimeInSeconds() : 0.0;
}

ndCudaSpatialVector* ndCudaContext::GetTransformBuffer()
{
	return m_implement->GetTransformBuffer();
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