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
#include "ndStlContainers.h"
#include "ndSyclContextImpl.h"

using namespace sycl;

ndSyclContextImpl::ndSyclContextImpl(sycl::device device)
	:m_device(device)
	,m_queue(device)
	,m_computeUnits(0)
	,m_sortPrefixBuffer(range<1>(64 * 1024))
{
	std::string deviceName(m_device.get_info<info::device::name>());
	std::string platformName(m_device.get_platform().get_info<info::platform::name>());
	sprintf(m_deviceName, "%s: %s", platformName.c_str(), deviceName.c_str());

	m_computeUnits = device.get_info<sycl::info::device::max_compute_units>();
}

ndSyclContextImpl::~ndSyclContextImpl()
{
}

const char* ndSyclContextImpl::GetStringId() const
{
	return m_deviceName;
}


void ndSyclContextImpl::Begin()
{
	StlVector<int> xxx;
	for (int i = 0; i < 16; i++)
	{
		xxx.push_back(1);
	}
	xxx.push_back(1);
	xxx.push_back(1);
	xxx.push_back(1);
}

#if 0
double ndSyclContextImpl::GetGPUTime() const
{
	return IsValid() ? m_implement->GetTimeInSeconds() : 0.0;
}

ndCudaSpatialVector* ndSyclContextImpl::GetTransformBuffer()
{
	return m_implement->GetTransformBuffer();
}


void ndSyclContextImpl::End()
{
	m_implement->End();
}

void ndSyclContextImpl::ResizeBuffers(int size)
{
	m_implement->ResizeBuffers(size);
}

void ndSyclContextImpl::LoadBodyData(const ndCudaBodyProxy* const src, int size)
{
	m_implement->LoadBodyData(src, size);
}

void ndSyclContextImpl::ValidateContextBuffers()
{
	m_implement->ValidateContextBuffers();
}

void ndSyclContextImpl::InitBodyArray()
{
	m_implement->InitBodyArray();
}

void ndSyclContextImpl::IntegrateBodies(float timestep)
{
	m_implement->IntegrateBodies(timestep);
}

void ndSyclContextImpl::IntegrateUnconstrainedBodies(float timestep)
{
	m_implement->IntegrateUnconstrainedBodies(timestep);
}

void ndSyclContextImpl::UpdateTransform()
{
	m_implement->UpdateTransform();
}
#endif