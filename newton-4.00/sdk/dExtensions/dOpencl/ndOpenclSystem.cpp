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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndOpenclSystem.h"
#include "ndDynamicsUpdateOpencl.h"
#include "ndJointBilateralConstraint.h"


const char* ndOpenclSystem::m_kernelSource = R""""(

struct ndOpenclBodyBuffer
{ 
	float4* m_rotation; 
	float4* m_posit; 
	float4* m_omega;
	float4* m_veloc;
	float4* m_alpha;
	float4* m_accel;
}; 

float DotProduct(float4 a, float4 b)
{
	float4 mag2 = a * b;
	return mag2.x + mag2.y + mag2.z + mag2.w;
}

float4 MakeQuat(float4 axis, float angle)
{
	angle = angle * 0.5f;
	float4 quat = axis * ((float4) (sin (angle)));
	quat.w = cos (angle);
	return quat;
}

float4 MultiplyQuat(float4 r, float4 q)
{
	float4 x = (float4)( q.w,  q.z, -q.y, -q.x);
	float4 y = (float4)(-q.z,  q.w,  q.x, -q.y);
	float4 z = (float4)( q.y, -q.x,  q.w, -q.z);
	return x * ((float4)(r.x)) + y * ((float4)(r.y)) + z * ((float4)(r.z)) + q * ((float4)(r.w));
}

float4 NormalizeQuat(float4 r)
{
	float invMag = 1.0f / sqrt(DotProduct(r, r));
	return r * ((float4) (invMag));
}

__kernel void IntegrateBodies(
	float timestepIn, 
	int bodyCount, 
	__global float4* rotationBuffer, 
	__global float4* centerOfMassBuffer,
	__global float4* omegaBuffer,
	__global float4* velocBuffer,
	__global float4* alphaBuffer,
	__global float4* accelBuffer) 
{ 
	const int globalIndex = get_global_id(0); 
	if (globalIndex >= bodyCount) 
	{
		return;
	}	

	float4 timestep = (float4) (timestepIn);
	float4 invTimestep = (float4) (1.0f / timestepIn);
	
	float4 veloc = velocBuffer[globalIndex];
	float4 omega = omegaBuffer[globalIndex];
	float4 accel = accelBuffer[globalIndex];
	float4 alpha = alphaBuffer[globalIndex];
	float4 com = centerOfMassBuffer[globalIndex];
	float4 rotation = rotationBuffer[globalIndex]; 
	
	com = com + timestep * veloc;
	accel = invTimestep * (veloc - accel);
	alpha = invTimestep * (omega - alpha);
	
	float4 omega2 = omega * omega;
	float tmpMag2 = omega2.x + omega2.y + omega2.z;
	
	float tol = (0.0125f * 3.141592f / 180.0f);
	float tol2 = tol * tol;
	float omegaMag2 = (tmpMag2 > tol2) ? tmpMag2 : tol2;
	
	float invOmegaMag = 1.0f / sqrt(omegaMag2);
	float omegaAngle = invOmegaMag * omegaMag2 * timestepIn;
	float4 omegaAxis = omega * invOmegaMag;
	float4 rotationStep = MakeQuat(omegaAxis, omegaAngle);
	float4 newRotation = MultiplyQuat(rotation, rotationStep);
	rotation = NormalizeQuat(newRotation);
	
	accelBuffer[globalIndex] = accel;
	alphaBuffer[globalIndex] = alpha;
	centerOfMassBuffer[globalIndex] = com;
	rotationBuffer[globalIndex] = rotation; 
} 

__kernel void IntegrateUnconstrainedBodies(float timestep, int bodyCount, __global struct ndOpenclBodyBuffer* inputArray, __global struct ndOpenclBodyBuffer* outputArray) 
{ 
	const int globalIndex = get_global_id(0); 
	if (globalIndex >= bodyCount) 
	{
		return;
	}	
} 


)"""";



ndOpenclBodyBuffer::ndOpenclBodyBuffer()
	:m_rotation(CL_MEM_READ_WRITE)
	, m_posit(CL_MEM_READ_WRITE)
	, m_omega(CL_MEM_READ_WRITE)
	, m_veloc(CL_MEM_READ_WRITE)
	, m_alpha(CL_MEM_READ_WRITE)
	, m_accel(CL_MEM_READ_WRITE)
{
}

ndOpenclBodyBuffer::~ndOpenclBodyBuffer()
{
}

void ndOpenclBodyBuffer::Cleanup()
{
	m_rotation.Cleanup();
	m_posit.Cleanup();
	m_omega.Cleanup();
	m_veloc.Cleanup();
	m_alpha.Cleanup();
	m_accel.Cleanup();
}

void ndOpenclBodyBuffer::Resize(cl_context context, dArray<ndBodyKinematic*>& bodyArray)
{
	if (m_rotation.GetCapacity() < bodyArray.GetCount())
	{
		dInt32 size = dMax(m_rotation.GetCapacity(), D_OPENCL_BUFFER_SIZE);
		while (size < bodyArray.GetCount())
		{
			size *= 2;
		}
		m_rotation.SyncSize(context, size);
		m_posit.SyncSize(context, size);
		m_omega.SyncSize(context, size);
		m_veloc.SyncSize(context, size);
		m_alpha.SyncSize(context, size);
		m_accel.SyncSize(context, size);
	}
}

void ndOpenclBodyBuffer::CopyToGpu(cl_command_queue commandQueue, const dArray<ndBodyKinematic*>& bodyArray)
{
	const dInt32 items = bodyArray.GetCount();
	m_rotation.SetCount(items);
	m_posit.SetCount(items);
	m_omega.SetCount(items);
	m_veloc.SetCount(items);
	m_alpha.SetCount(items);
	m_accel.SetCount(items);

	dVector* const posit = (dVector*)&m_posit[0];
	dVector* const omega = (dVector*)&m_omega[0];
	dVector* const veloc = (dVector*)&m_veloc[0];
	dVector* const alpha = (dVector*)&m_alpha[0];
	dVector* const accel = (dVector*)&m_accel[0];
	dQuaternion* const rotation = (dQuaternion*)&m_rotation[0];

	for (dInt32 i = 0; i < items; i++)
	{
		ndBodyDynamic* const body = bodyArray[i]->GetAsBodyDynamic();
		rotation[i] = body->GetRotation();
		posit[i] = body->GetGlobalGetCentreOfMass();
		omega[i] = body->GetOmega();
		veloc[i] = body->GetVelocity();
		alpha[i] = body->GetAlpha();
		accel[i] = body->GetAccel();
	}

	m_rotation.WriteData(commandQueue);
	m_posit.WriteData(commandQueue);
	m_omega.WriteData(commandQueue);
	m_veloc.WriteData(commandQueue);
	m_alpha.WriteData(commandQueue);
	m_accel.WriteData(commandQueue);
}

#ifdef D_DEBUG_GPU_KERNELS
dVector ndOpenclBodyBuffer::MakeQuat(const dVector& axis, float angle)
{
	angle = angle * 0.5f;
	dVector quat = axis * dVector(sin(angle));
	quat.m_w = cos(angle);
	return quat;
}

dVector ndOpenclBodyBuffer::MultiplyQuat(const dVector& r, const dVector& q)
{
	dVector x = dVector(q.m_w, q.m_z, -q.m_y, -q.m_x);
	dVector y = dVector(-q.m_z, q.m_w, q.m_x, -q.m_y);
	dVector z = dVector(q.m_y, -q.m_x, q.m_w, -q.m_z);
	return x * dVector(r.m_x) + y * dVector(r.m_y) + z * dVector(r.m_z) + q * dVector(r.m_w);
}

dVector ndOpenclBodyBuffer::NormalizeQuat(const dVector& r)
{
	dVector mag2Vec = r * r;
	float invMag = 1.0f / sqrt(mag2Vec.m_x + mag2Vec.m_y + mag2Vec.m_z + mag2Vec.m_w);
	return r * dVector(invMag);
}

void ndOpenclBodyBuffer::DebudKernel(dFloat32 timestepIn, const dArray<ndBodyKinematic*>& bodyArray)
{
	dVector* const omegaBuffer = (dVector*)&m_omega[0];
	dVector* const velocBuffer = (dVector*)&m_veloc[0];
	dVector* const alphaBuffer = (dVector*)&m_alpha[0];
	dVector* const accelBuffer = (dVector*)&m_accel[0];
	dVector* const centerOfMassBuffer = (dVector*)&m_posit[0];
	dQuaternion* const rotationBuffer = (dQuaternion*)&m_rotation[0];

	const dInt32 items = bodyArray.GetCount();
	for (dInt32 globalIndex = 0; globalIndex < items; globalIndex++)
	{
		dVector timestep(timestepIn);
		dVector invTimestep(1.0f / timestepIn);
		dVector veloc = velocBuffer[globalIndex];
		dVector omega = omegaBuffer[globalIndex];
		dVector accel = accelBuffer[globalIndex];
		dVector alpha = alphaBuffer[globalIndex];
		dVector com = centerOfMassBuffer[globalIndex];
		dVector rotation = rotationBuffer[globalIndex];

		com = com + timestep * veloc;
		accel = invTimestep * (veloc - accel);
		alpha = invTimestep * (omega - alpha);

		dVector omega2 = omega * omega;
		float tmpMag2 = omega2.m_x + omega2.m_y + omega2.m_z;

		float tol = (0.0125f * 3.141592f / 180.0f);
		float tol2 = tol * tol;
		float omegaMag2 = (tmpMag2 > tol2) ? tmpMag2 : tol2;

		float invOmegaMag = 1.0f / sqrt(omegaMag2);
		float omegaAngle = invOmegaMag * omegaMag2 * timestepIn;
		dVector omegaAxis = omega * invOmegaMag;
		dVector rotationStep = MakeQuat(omegaAxis, omegaAngle);
		dVector newRotation = MultiplyQuat(rotation, rotationStep);
		rotation = NormalizeQuat(newRotation);

		accelBuffer[globalIndex] = accel;
		alphaBuffer[globalIndex] = alpha;
		centerOfMassBuffer[globalIndex] = com;
		rotationBuffer[globalIndex] = rotation;
	}
}
#endif

void ndOpenclBodyBuffer::SetKernelParameters(cl_kernel kernel, dFloat32 timestep, const dArray<ndBodyKinematic*>& bodyArray)
{
	cl_int err = CL_SUCCESS;

	err = clSetKernelArg(kernel, 0, sizeof(cl_float), &timestep);
	dAssert(err == CL_SUCCESS);

	cl_int bodyCount = bodyArray.GetCount();
	err = clSetKernelArg(kernel, 1, sizeof(cl_int), &bodyCount);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(kernel, 2, sizeof(cl_mem), &m_rotation.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(kernel, 3, sizeof(cl_mem), &m_posit.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(kernel, 4, sizeof(cl_mem), &m_omega.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(kernel, 5, sizeof(cl_mem), &m_veloc.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(kernel, 6, sizeof(cl_mem), &m_alpha.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(kernel, 7, sizeof(cl_mem), &m_accel.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);
}



ndOpenclSystem::ndOpenclSystem(cl_context context, cl_platform_id)
	:m_context(context)
	,m_bodyArray()
	,m_integrateBodies()
{
	cl_int err = CL_SUCCESS;

	// get the device
	err = clGetContextInfo(m_context, CL_CONTEXT_DEVICES, sizeof(cl_device_id), &m_device, nullptr);
	dAssert(err == CL_SUCCESS);

	// get vendor driver support
	size_t stringLength;

	char deviceName[1024];
	err = clGetDeviceInfo(m_device, CL_DEVICE_NAME, sizeof(deviceName), deviceName, &stringLength);
	dAssert(err == CL_SUCCESS);

	char driverVersion[1024];
	err = clGetDeviceInfo(m_device, CL_DEVICE_VERSION, sizeof(driverVersion), driverVersion, &stringLength);
	dAssert(err == CL_SUCCESS);
	dAssert((strlen(deviceName) + strlen(driverVersion) + 8) < sizeof(m_platformName));

	// linux does not like this;
	//sprintf(m_platformName, "%s: %s", deviceName, driverVersion);
	strcpy(m_platformName, deviceName);
	strcat(m_platformName, ": ");
	strcat(m_platformName, driverVersion);

	// create command queue
	cl_command_queue_properties properties = CL_QUEUE_PROFILING_ENABLE;
	m_commandQueue = clCreateCommandQueue(m_context, m_device, properties, &err);
	dAssert(err == CL_SUCCESS);

	m_solverProgram = CompileProgram();

	SetKernel("IntegrateBodies", m_integrateBodies);
	//SetKernel("IntegrateUnconstrainedBodies", m_integrateBodies);
}

ndOpenclSystem::~ndOpenclSystem()
{
	cl_int err = CL_SUCCESS;

	m_bodyArray.Cleanup();

	err = clReleaseKernel(m_integrateBodies.m_kernel);
	dAssert(err == CL_SUCCESS);

	//err = clReleaseKernel(m_integrateUnconstrainedBodies);
	//dAssert(err == CL_SUCCESS);

	err = clReleaseProgram(m_solverProgram);
	dAssert(err == CL_SUCCESS);

	err = clReleaseCommandQueue(m_commandQueue);
	dAssert(err == CL_SUCCESS);

	err = clReleaseDevice(m_device);
	dAssert(err == CL_SUCCESS);

	err = clReleaseContext(m_context);
	dAssert(err == CL_SUCCESS);
}

ndOpenclSystem* ndOpenclSystem::Singleton(dInt32 driveNumber)
{
	cl_uint numPlatforms = 0;
	cl_int err = clGetPlatformIDs(0, nullptr, &numPlatforms);
	if ((err != CL_SUCCESS) || (numPlatforms == 0))
	{
		return nullptr;
	}

	dAssert(numPlatforms < 16);
	cl_platform_id platforms[16];
	err = clGetPlatformIDs(numPlatforms, &platforms[0], nullptr);
	if (err != CL_SUCCESS)
	{
		return nullptr;
	}

	dInt32 driveIndex = 0;
	cl_platform_id bestPlatform = 0;
	for (cl_uint i = 0; i < numPlatforms; i++)
	{
		cl_uint numDevices = 0;
		err = clGetDeviceIDs(platforms[i], CL_DEVICE_TYPE_GPU, 0, nullptr, &numDevices);
		if (!((err != CL_SUCCESS) || (numDevices == 0)))
		{
			bestPlatform = platforms[i];
			if (driveIndex == driveNumber)
			{
				break;
			}
			driveIndex++;
		}
	}

	if (bestPlatform == nullptr)
	{
		return nullptr;
	}

	cl_context_properties contextProperties[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)bestPlatform, 0 };

	cl_context context = clCreateContextFromType(contextProperties, CL_DEVICE_TYPE_GPU, nullptr, nullptr, &err);
	if ((CL_SUCCESS != err) || (context == nullptr))
	{
		return nullptr;
	}

	return new ndOpenclSystem(context, bestPlatform);
}

cl_program ndOpenclSystem::CompileProgram()
{
	cl_int err = CL_SUCCESS;
	const char* source[1];
	source[0] = m_kernelSource;
	size_t sourceSize = strlen(m_kernelSource);
	cl_program program = clCreateProgramWithSource(m_context, 1, source, &sourceSize, &err);
	dAssert(err == CL_SUCCESS);

	err = clBuildProgram(program, 1, &m_device, "", nullptr, nullptr);
	if (err == CL_BUILD_PROGRAM_FAILURE)
	{
		size_t log_size = 0;
		clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);

		char* const build_log = dAlloca(char, log_size + 4096);
		clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, log_size, build_log, nullptr);
		dTrace((build_log));
	}
	dAssert(err == CL_SUCCESS);
	return program;
}

void ndOpenclSystem::SetKernel(const char* const name, ndKernel& kerner)
{
	cl_int err = CL_SUCCESS;

	kerner.m_kernel = clCreateKernel(m_solverProgram, name, &err);
	dAssert(err == CL_SUCCESS);

	err = clGetKernelWorkGroupInfo(kerner.m_kernel, m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(kerner.m_workWroupSize), &kerner.m_workWroupSize, nullptr);
	dAssert(err == CL_SUCCESS);
}

void ndOpenclSystem::Resize(dArray<ndBodyKinematic*>& bodyArray)
{
	m_bodyArray.Resize(m_context, bodyArray);
}

void ndOpenclSystem::CopyToGpu(const dArray<ndBodyKinematic*>& bodyArray)
{
	m_bodyArray.CopyToGpu(m_commandQueue, bodyArray);
}

void ndOpenclSystem::Finish()
{
	cl_int err = CL_SUCCESS;
	err = clFinish(m_commandQueue);
	dAssert(err == CL_SUCCESS);
}

void ndOpenclSystem::ExecuteIntegrateBody(dFloat32 timestep, const dArray<ndBodyKinematic*>& bodyArray)
{
	// let the driver decide the local work group size.
	//size_t local = bodyArray.GetCount();
	size_t global = bodyArray.GetCount();
	m_bodyArray.SetKernelParameters(m_integrateBodies.m_kernel, timestep, bodyArray);
	cl_int err = clEnqueueNDRangeKernel(
		m_commandQueue, m_integrateBodies.m_kernel, 1,
		nullptr, &global, nullptr, 0, nullptr, nullptr);
	dAssert(err == CL_SUCCESS);

	// enqueue to read the body buffers results
	m_bodyArray.m_alpha.ReadData(m_commandQueue);
	m_bodyArray.m_accel.ReadData(m_commandQueue);
	m_bodyArray.m_posit.ReadData(m_commandQueue);
	m_bodyArray.m_rotation.ReadData(m_commandQueue);

#ifdef D_DEBUG_GPU_KERNELS
	m_bodyArray.DebudKernel(timestep, bodyArray);
#endif
}
