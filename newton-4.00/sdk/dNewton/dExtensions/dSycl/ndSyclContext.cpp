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
#include "ndSyclSort.h"
#include "ndSyclContext.h"
#include "ndStlContainers.h"
#include "ndSyclContextImpl.h"

using namespace sycl;

ndSyclContext::ndSyclContext(bool selectCpu)
	:m_impl(nullptr)
{
	EnumDevices(selectCpu);

	StlVector<int> buffer0;
	StlVector<int> buffer1;
	for (int i = 0; i < 1000; i++)
	{
		buffer0.push_back(rand() & 0xff);
	}
	buffer1.resize(buffer0.size());

	class CountDigit
	{
		public:
		int GetCount(const int& item) const
		{
			return item & 0xff;
		}
	};

	buffer buf0(buffer0);
	buffer buf1(buffer1);
	ndCountingSort<int, CountDigit, 8>(m_impl, buf0, buf1);
	m_impl->m_queue.wait();

	StlVector<int> buffer2;
	host_accessor result(m_impl->m_sortPrefixBuffer);
	for (int i = 0; i < result.size(); i++)
	{
		buffer2.push_back(result[i]);
	}
	//ndAssert(0);
}

ndSyclContext::~ndSyclContext()
{
	if (m_impl)
	{
		delete m_impl;
	}
}

void* ndSyclContext::operator new (size_t size)
{
	return ndSyclMalloc(size);
}

void ndSyclContext::operator delete (void* ptr)
{
	ndSyclFree(ptr);
}

void ndSyclContext::SetMemoryAllocators(ndMemAllocCallback alloc, ndMemFreeCallback free)
{
	ndSyclSetMemoryAllocators(alloc, free);
}

bool ndSyclContext::IsValid() const 
{
	return m_impl ? true : false;
}

const char* ndSyclContext::GetStringId() const
{
	ndAssert(m_impl);
	return m_impl->GetStringId();
}

void ndSyclContext::EnumDevices(bool selectCpu)
{
	std::vector<device> devices;
	std::vector<platform> platforms(platform::get_platforms());
	for (int i = 0; i < platforms.size(); ++i)
	{
		platform& plat = platforms[i];
		std::vector<device> devList(plat.get_devices());
		for (int j = 0; j < devList.size(); ++j)
		{
			device& dev = devList[j];
			if (selectCpu)
			{
				if (dev.is_cpu())
				{
					devices.push_back(dev);
				}
			}
			else
			{
				if (dev.is_gpu())
				{
					devices.push_back(dev);
				}
			}
		}
	}

	if (devices.size())
	{
		device bestDevice = devices[0];
		for (int i = 0; i < devices.size(); ++i)
		{
			std::string platformName(devices[i].get_platform().get_info<info::platform::name>());
			for (int j = 0; j < platformName.size(); ++j)
			{
				platformName[j] = char(tolower(platformName[j]));
			}
			if (platformName.find("opencl") >= 0)
			{
				bestDevice = devices[i];
			}
		}

		m_impl = new ndSyclContextImpl(bestDevice);
	}
}

void ndSyclContext::Begin()
{
	m_impl->Begin();
}


#if 0
double ndSyclContext::GetGPUTime() const
{
	return IsValid() ? m_impl->GetTimeInSeconds() : 0.0;
}

ndCudaSpatialVector* ndSyclContext::GetTransformBuffer()
{
	return m_impl->GetTransformBuffer();
}


void ndSyclContext::End()
{
	m_impl->End();
}

void ndSyclContext::ResizeBuffers(int size)
{
	m_impl->ResizeBuffers(size);
}

void ndSyclContext::LoadBodyData(const ndCudaBodyProxy* const src, int size)
{
	m_impl->LoadBodyData(src, size);
}

void ndSyclContext::ValidateContextBuffers()
{
	m_impl->ValidateContextBuffers();
}

void ndSyclContext::InitBodyArray()
{
	m_impl->InitBodyArray();
}

void ndSyclContext::IntegrateBodies(float timestep)
{
	m_impl->IntegrateBodies(timestep);
}

void ndSyclContext::IntegrateUnconstrainedBodies(float timestep)
{
	m_impl->IntegrateUnconstrainedBodies(timestep);
}

void ndSyclContext::UpdateTransform()
{
	m_impl->UpdateTransform();
}
#endif