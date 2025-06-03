/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndBrainStdafx.h"
#include <CL/opencl.hpp>
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"

#include <string>
#include <vector>

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
{
	m_devicedInitialized = false;
	ndMemSet(m_modules, (void*)nullptr, sizeof(m_modules) / sizeof(m_modules[0]));

	// get all devices of all platforms
	std::vector<cl::Device> cl_devices; 
	{
		std::vector<cl::Platform> cl_platforms; 
		cl::Platform::get(&cl_platforms);
		for (size_t i = 0; i < cl_platforms.size(); i++)
		{
			std::vector<cl::Device> cl_devices_available;
			cl_platforms[i].getDevices(CL_DEVICE_TYPE_ALL, &cl_devices_available);
			for (size_t j = 0u; j < cl_devices_available.size(); j++)
			{
				cl_devices.push_back(cl_devices_available[j]);
			}
		}
	}

	// select fastest available device
	{
		size_t best_i = 0;
		size_t bestCapacity = 0;
		for (size_t i = 0u; i < cl_devices.size(); i++)
		{
			// find device with highest (estimated) floating point performance
			//const std::string name (cl_devices[i].getInfo<CL_DEVICE_NAME>()); // device name
			//const std::string vendor (cl_devices[i].getInfo<CL_DEVICE_VENDOR>()); // device vendor
			const size_t compute_units = cl_devices[i].getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>(); // compute units (CUs) can contain multiple cores depending on the microarchitecture
			const size_t clock_frequency = cl_devices[i].getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>(); // in MHz

			size_t capacity = compute_units * clock_frequency;
			// estimated device floating point performance in TeraFLOPs/s
			if (capacity > bestCapacity)
			{
				bestCapacity = capacity;
				best_i = i;
			}
		}

		const std::string name(cl_devices[best_i].getInfo<CL_DEVICE_NAME>());
		m_device = cl_devices[best_i];
		//print_info(name); // print device name
		ndExpandTraceMessage("opencl accelerator: %s\n", name.c_str());
		m_devicedInitialized = true;
	}
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

bool ndBrainGpuContext::HasGpuSupport() 
{ 
	return true; 
}

ndBrainContext::ndContextType ndBrainGpuContext::GetType() const
{
	return ndBrainContext::m_gpu;
}
