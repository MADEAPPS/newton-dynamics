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

#define ND_SORT_SCAN_BUFFER_SIZE (256 * 256)

ndSyclContextImpl::ndSyclContextImpl(sycl::device& device)
	:m_device(device)
	,m_queue(device)
	,m_computeUnits(0)
	,m_localMemorySize(0)
	,m_maxWorkGroupSize(0)
	,m_sortPrefixBuffer(range<1>(ND_SORT_SCAN_BUFFER_SIZE))
	,m_cpuBuffer0()
	,m_cpuBuffer1()
	,m_cpuBuffer2()
	,m_buf0(range<1>(1000))
	,m_buf1(range<1>(1000))
{
	m_computeUnits = device.get_info<sycl::info::device::max_compute_units>();
	m_localMemorySize = device.get_info<sycl::info::device::local_mem_size>();
	m_maxWorkGroupSize = device.get_info<sycl::info::device::max_work_group_size>();
	
	std::string deviceName(m_device.get_info<info::device::name>());
	std::string platformName(m_device.get_platform().get_info<info::platform::name>());
	sprintf(m_deviceName, "%s: %s", platformName.c_str(), deviceName.c_str());

	// debuging code
	for (int i = 0; i < m_buf0.size(); i++)
	{
		m_cpuBuffer0.push_back(0);
	}
	m_cpuBuffer1.resize(m_cpuBuffer0.size());
	m_cpuBuffer2.resize(m_sortPrefixBuffer.size());
	
	for (int i = 0; i < 100; i++)
	{
		m_cpuBuffer0[i] = rand() % 0x3;
	}
	
	host_accessor m_buffer0(m_buf0);
	host_accessor m_buffer1(m_buf1);
	for (int i = 0; i < m_buf0.size(); ++i)
	{
		m_buffer0[i] = m_cpuBuffer0[i];
		m_buffer1[i] = m_cpuBuffer1[i];
	}
}

ndSyclContextImpl::~ndSyclContextImpl()
{
}

const char* ndSyclContextImpl::GetStringId() const
{
	return m_deviceName;
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


void ndSyclContextImpl::Begin()
{
	class CountDigit
	{
		public:
		int GetCount(const int& item) const
		{
			return item & 0xff;
		}
	};

	StlVector<unsigned> xxxxxxxx;
	xxxxxxxx.resize(64 * 1024);
	ndCountingSort<int, CountDigit, 8>(m_cpuBuffer0, m_cpuBuffer1, xxxxxxxx);

	CountingSort<int, CountDigit, 8>(m_buf0, m_buf1);
	m_queue.wait();
	
	host_accessor result(m_sortPrefixBuffer);
	for (int i = 0; i < result.size(); i++)
	{
		m_cpuBuffer2[i] = result[i];
	}
	//ndAssert(0);
}