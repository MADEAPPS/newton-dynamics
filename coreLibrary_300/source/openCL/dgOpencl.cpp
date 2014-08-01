/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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



#include "dgOpencl.h"


#define NEWTON_OPENCL_SOURCE	"NEWTON_DYNAMICS"

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER) || defined (_POSIX_VER) || defined (_POSIX_VER_64) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
	#include <CL\cl.h>
	#include <CL\cl_ext.h>
#else 
	#ifdef _MACOSX_VER
			#error "No OpenCL SDK" 
	#else 
		#error "No OpenCL SDK" 
	#endif
#endif



dgOpencl* dgOpencl::GetOpenCL(dgMemoryAllocator* allocator)
{
	static dgOpencl opencl(allocator);
	return &opencl;
}


dgOpencl::dgOpencl(dgMemoryAllocator* const allocator)
	:m_allocator(allocator)
	,m_currentPlatform(NULL)
	,m_context(NULL)
//	,m_cmd_queue(NULL)
	,m_program(NULL)
	,m_platformsCount(0)
	,m_aligment(0)
{
	char buffer[2048];
	DWORD varCount = GetEnvironmentVariable(NEWTON_OPENCL_SOURCE, buffer, sizeof (buffer));

	if (varCount) {
		cl_uint count;
		if (!clGetPlatformIDs(0, NULL, &count)) {
			m_platformsCount = count;
			_ASSERTE (m_platformsCount < sizeof (m_platforms) / sizeof (m_platforms[0]));
			clGetPlatformIDs(m_platformsCount, (cl_platform_id*)m_platforms, NULL);
		}
	}

SelectPlaform(0);
}


dgOpencl::~dgOpencl(void)
{
	CleanUp();
}


dgInt32 dgOpencl::GetPlatformsCount() const
{
	return m_platformsCount;
}

void dgOpencl::CleanUp()
{
	if (m_program) {
		clReleaseProgram(cl_program (m_program));
		m_program = NULL;
	}

	if (m_context) {
//		clReleaseCommandQueue (cl_command_queue (m_cmd_queue));
		clReleaseContext (cl_context (m_context) ); 
		m_context = NULL;
//		m_cmd_queue = NULL;
	}
}

void dgOpencl::GetVendorString(dgInt32 deviceIndex, char* const name, dgInt32 maxlength) const
{
	deviceIndex = dgClamp(deviceIndex, 0, m_platformsCount - 1);
	//clGetPlatformInfo(cl_platform_id (m_platforms[deviceIndex]), CL_PLATFORM_NAME, maxlength, name, NULL);
	clGetPlatformInfo(cl_platform_id (m_platforms[deviceIndex]), CL_PLATFORM_VERSION, maxlength, name, NULL);
}


void dgOpencl::SelectPlaform(dgInt32 platform)
{
	dgInt32 index = dgClamp(platform, 0, dgInt32 (m_platformsCount - 1)); 
	m_currentPlatform = m_platforms[index];

	// get the devices for this platform
	cl_uint numDevices;
	clGetDeviceIDs(cl_platform_id (m_currentPlatform), CL_DEVICE_TYPE_ALL, 0, NULL, &numDevices);
	_ASSERTE (numDevices < sizeof (m_devices)/ sizeof (m_devices[0]));
	m_devicesCount = numDevices;
	clGetDeviceIDs(cl_platform_id (m_currentPlatform), CL_DEVICE_TYPE_ALL, numDevices, (cl_device_id*)m_devices, NULL);

	// create the context
	cl_int errcode_ret;
	m_context = clCreateContext (NULL, numDevices, (cl_device_id*)m_devices, NULL, NULL, &errcode_ret);
	_ASSERTE (!errcode_ret);

	// get the memory alignment
	cl_uint alignment;
	clGetDeviceInfo (cl_device_id (m_devices[0]), CL_DEVICE_MEM_BASE_ADDR_ALIGN, sizeof(cl_uint), &alignment, NULL);
	m_aligment = alignment/8; //in bytes

	cl_uint computeUnits;
	clGetDeviceInfo (cl_device_id (m_devices[0]), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), &computeUnits, NULL);

//	cl_uint cpuSize;
//	clGetDeviceInfo (cl_device_id (m_devices[0]), CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, sizeof(cl_uint), &cpuSize, NULL);

	cl_uint workGroupSize;
	clGetDeviceInfo (cl_device_id (m_devices[0]), CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(cl_uint), &workGroupSize, NULL);

	cl_uint workItemDim;
	clGetDeviceInfo (cl_device_id (m_devices[0]), CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(cl_uint), &workItemDim, NULL);

	cl_uint workItemSize[8];
	clGetDeviceInfo (cl_device_id (m_devices[0]), CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(workItemSize), workItemSize, NULL);
		  


	// create command queue
//	m_cmd_queue = clCreateCommandQueue(cl_context (m_context), cl_device_id (m_devices[0]), 0, NULL);

	// compile all programs
	CompileProgram ();

	// create all kernels
//	BuildKernels ();

}


void dgOpencl::CompileProgram ()
{
	char tmp[2048];
	char path[2048];
	char fullPathName[2048];
	GetEnvironmentVariable(NEWTON_OPENCL_SOURCE, tmp, sizeof (tmp));
	sprintf (path, "%s/coreLibrary_300/source/openCL", tmp);  
	
	char* programSource[256];

	dgInt32 programCount = 0;
	sprintf (fullPathName, "%s/*.*", path);
	_finddata_t data;
	intptr_t handle = _findfirst (fullPathName, &data);
	if (handle != -1) {
		do {
			char tmpPath[2048];
			sprintf (tmpPath, "%s/%s", path, data.name);
			if (strstr (data.name, ".cl")) {
				FILE* const file = fopen (tmpPath, "rb");
				_ASSERTE (file);
				fseek (file, 0, SEEK_END);
				dgInt32 size = ftell(file); 
				fseek (file, 0, SEEK_SET);
				programSource[programCount] = (char*)dgMallocStack (size + 1);
				fread (programSource[programCount], 1, size, file);
				programSource[programCount][size] = 0;
				fclose(file);
				programCount ++;
				dgAssert (programCount < (sizeof (programSource) / sizeof(programSource[0])));
			}
		} while (_findnext (handle, &data) == 0);
		_findclose (handle);
	}

	cl_int errcode_ret;
	m_program = clCreateProgramWithSource(cl_context (m_context), programCount, (const char**)&programSource[0], NULL, &errcode_ret);
	_ASSERTE (!errcode_ret);

	for (dgInt32 i = 0; i < programCount; i ++) {
		dgFreeStack (programSource[i]);
	}

	errcode_ret = clBuildProgram(cl_program (m_program), m_devicesCount, (cl_device_id*)m_devices, NULL, NULL, NULL);
	_ASSERTE (errcode_ret ==  CL_SUCCESS);
}

