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
#include <CL/cl.h>
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdateOpencl.h"
#include "ndJointBilateralConstraint.h"

#define D_GPU_WORK_GROUP		128

#define D_MAX_BODY_RADIX_BIT		9
#define D_MAX_BODY_RADIX_DIGIT		(1<<D_MAX_BODY_RADIX_BIT)
#define D_MAX_BODY_RADIX_MASK		(D_MAX_BODY_RADIX_DIGIT-1)

#if 0
template<class T>
class dOpenclBuffer: public dArray<T>
{
	public:
	dOpenclBuffer(cl_mem_flags flags)
		:dArray<T>(D_DEFAULT_BUFFER_SIZE)
		,m_flags(flags)
		,m_gpuBuffer(nullptr)
	{
		//SetCount(D_DEFAULT_BUFFER_SIZE);
		dArray<T>::SetCount(D_DEFAULT_BUFFER_SIZE * 10);
	}

	~dOpenclBuffer()
	{
		if (m_gpuBuffer)
		{
			cl_int err;
			err = clReleaseMemObject(m_gpuBuffer);
			dAssert(err == CL_SUCCESS);
		}
	}

	void SyncSize(cl_context context, dInt32 size)
	{
		cl_int err;

		if (m_gpuBuffer == nullptr)
		{
			if (m_flags & CL_MEM_USE_HOST_PTR)
			{
				void* const hostBuffer = &(*this)[0];
				m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * dArray<T>::GetCapacity(), hostBuffer, &err);
			}
			else
			{
				m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * dArray<T>::GetCapacity(), nullptr, &err);
			}
			dAssert(err == CL_SUCCESS);
		}

		if (dArray<T>::GetCapacity() < size)
		{
			dAssert(0);
		}
		dArray<T>::SetCount(size);
	}

	void ReadData(cl_command_queue commandQueue)
	{
		void* const source = &(*this)[0];
		cl_int err = clEnqueueReadBuffer(commandQueue, m_gpuBuffer,
			CL_FALSE, 0, sizeof(T) * dArray<T>::GetCount(), source,
			0, nullptr, nullptr);
		dAssert(err == CL_SUCCESS);
	}

	void WriteData(cl_command_queue commandQueue)
	{
		cl_int err;
		const void* const source = &(*this)[0];
		err = clEnqueueWriteBuffer(commandQueue, m_gpuBuffer,
			CL_FALSE, 0, sizeof(T) * dArray<T>::GetCount(), source,
			0, nullptr, nullptr);
		dAssert(err == CL_SUCCESS);
	}

	cl_mem_flags m_flags;
	cl_mem m_gpuBuffer;
};

class ndOpenclBodyProxy
{
	public:
	cl_float4 m_rotation;
	cl_float3 m_position;
	cl_float3 m_veloc;
	cl_float3 m_omega;
	cl_float4 m_invMass;
};

//class ndOpenclOutBodyProxy
//{
//	public:
//	cl_float3 m_matrix[3];
//	cl_float3 m_position;
//	cl_float4 m_rotation;
//	cl_float3 m_veloc;
//	cl_float3 m_omega;
//};

class ndOpenclMatrix3x3
{
	//cl_float3 m_matrix[3];
};


class ndOpenclBodyWorkingBuffer
{
	public:
	//ndOpenclMatrix3x3 m_matrix;
	cl_float4 m_rotation;
	cl_float3 m_position;
	cl_float3 m_veloc;
	cl_float3 m_omega;
};

#endif

class OpenclSystem: public dClassAlloc
{
	public:
	OpenclSystem(cl_context context, cl_platform_id platform)
		:m_context(context)
		////,m_bodyArray(CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR)
		//,m_bodyArray(CL_MEM_READ_ONLY)
		////,m_outBodyArray(CL_MEM_WRITE_ONLY)
		//,m_bodyWorkingArray(CL_MEM_READ_WRITE)
		,m_integrateUnconstrainedBodies(nullptr)
	{
		cl_int err;
		// get the device
		err = clGetContextInfo(m_context, CL_CONTEXT_DEVICES, sizeof(cl_device_id), &m_device, nullptr);
		dAssert(err == CL_SUCCESS);
		
		// get vendor driver support
		size_t stringLength = 0;
		err = clGetPlatformInfo(platform, CL_PLATFORM_NAME, 0, nullptr, &stringLength);
		dAssert(err == CL_SUCCESS);
		dAssert(stringLength < sizeof(m_platformName));
		err = clGetPlatformInfo(platform, CL_PLATFORM_NAME, stringLength, m_platformName, nullptr);
		dAssert(err == CL_SUCCESS);
		
		// get opencl version
		err = clGetDeviceInfo(m_device, CL_DEVICE_VERSION, 0, NULL, &stringLength);
		dAssert(err == CL_SUCCESS);
		dAssert(stringLength < sizeof(m_platformName));
		err = clGetDeviceInfo(m_device, CL_DEVICE_VERSION, stringLength, m_platformName, nullptr);
		dAssert(err == CL_SUCCESS);
		
		// create command queue
		cl_command_queue_properties properties = CL_QUEUE_PROFILING_ENABLE;
		m_commandQueue = clCreateCommandQueue(m_context, m_device, properties, &err);
		dAssert(err == CL_SUCCESS);
		
		m_solverProgram = CompileProgram();
		
		m_integrateUnconstrainedBodies = clCreateKernel(m_solverProgram, "IntegrateUnconstrainedBodies", &err);
		dAssert(err == CL_SUCCESS);
	}

	~OpenclSystem()
	{
		cl_int err;
		
		err = clReleaseKernel(m_integrateUnconstrainedBodies);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseProgram(m_solverProgram);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseCommandQueue(m_commandQueue);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseDevice(m_device);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseContext(m_context);
		dAssert(err == CL_SUCCESS);
	}

	static OpenclSystem* Singleton(dInt32 driveNumber)
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

		return new OpenclSystem(context, bestPlatform);
	}

	cl_program CompileProgram()
	{
		dInt32 errorCode;
		const char* source[1];
		source[0] = m_kernelSource;
		size_t sourceSize = strlen(m_kernelSource);
		cl_program program = clCreateProgramWithSource(m_context, 1, source, &sourceSize, &errorCode);
		dAssert(errorCode == CL_SUCCESS);

		errorCode = clBuildProgram(program, 1, &m_device, "", nullptr, nullptr);
		if (errorCode == CL_BUILD_PROGRAM_FAILURE)
		{
			size_t log_size = 0;
			clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);

			char* const build_log = dAlloca(char, log_size + 4096);
			clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, log_size, build_log, nullptr);
			dTrace((build_log));
		}
		dAssert(errorCode == CL_SUCCESS);
		return program;
	}

	#if 0	
	//cl_kernel        kernel;				// hold the kernel handler
	

	dOpenclBuffer<ndOpenclBodyProxy> m_bodyArray;
	dOpenclBuffer<ndOpenclBodyWorkingBuffer> m_bodyWorkingArray;
//	dOpenclBuffer<ndOpenclOutBodyProxy> m_outBodyArray;


#endif

	char m_platformName[128];

	// Regular OpenCL objects:
	cl_context m_context;					// hold the context handler
	cl_device_id m_device;					// hold the selected device handler
	cl_program	m_solverProgram;			// hold the program handler
	cl_command_queue m_commandQueue;		// hold the commands-queue handler

	cl_kernel m_integrateUnconstrainedBodies;
	static const char* m_kernelSource;
};

const char* OpenclSystem::m_kernelSource = R""""(

struct ndOpenclMatrix3x3
{
	float3 m_element[3];
}; 

struct ndOpenclBodyProxy 
{ 
	float4 m_rotation; 
	float3 m_position; 
	float3 m_veloc;	
	float3 m_omega;	
	float4 m_invMass; 
}; 

struct ndOpenclBodyWorkingBuffer 
{ 
	struct ndOpenclMatrix3x3 m_matrix; 
	float4 m_rotation; 
	float3 m_position; 
	float3 m_veloc;	
	float3 m_omega;	
}; 

struct ndOpenclMatrix3x3 QuatToMatrix(float4 rotation) 
{ 
	struct ndOpenclMatrix3x3 matrix; 
	return matrix; 
} 

__kernel void IntegrateUnconstrainedBodies(float timestep, int bodyCount, __global struct ndOpenclBodyProxy* inputArray, __global struct ndOpenclBodyWorkingBuffer* outputArray) 
{ 
	const int index = get_global_id(0); 

	struct ndOpenclBodyProxy body; 

	// load all variable into registers.
	if (index < bodyCount) 
	{ 
		body = inputArray[index];
	} 
	barrier(CLK_LOCAL_MEM_FENCE); 

	if (index < bodyCount) 
	{ 
		struct ndOpenclMatrix3x3 matrix; 
		matrix = QuatToMatrix(body.m_rotation); 
	} 
} 

)"""";

ndDynamicsUpdateOpencl::ndDynamicsUpdateOpencl(ndWorld* const world, dInt32 driverNumber)
	:ndDynamicsUpdate(world)
	,m_opencl(nullptr)
{
	m_opencl = OpenclSystem::Singleton(driverNumber);
}

ndDynamicsUpdateOpencl::~ndDynamicsUpdateOpencl()
{
	if (m_opencl)
	{
		delete m_opencl;
	}
}

const char* ndDynamicsUpdateOpencl::GetStringId() const
{
	return m_opencl ? m_opencl->m_platformName : "no opencl support";
}

void ndDynamicsUpdateOpencl::BuildIsland()
{
	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	dAssert(bodyArray.GetCount() >= 1);
	if (bodyArray.GetCount() - 1)
	{
		D_TRACKTIME();
		SortJoints();
		SortIslands();
	}
}

void ndDynamicsUpdateOpencl::SortJoints()
{
	D_TRACKTIME();

	class ndSleep0 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();
				const dInt32 rows = joint->GetRowsCount();
				joint->m_rowCount = rows;

				const dInt32 equilibrium = body0->m_equilibrium & body1->m_equilibrium;
				if (!equilibrium)
				{
					body0->m_solverSleep0 = 0;
					if (body1->GetInvMass() > dFloat32(0.0f))
					{
						body1->m_solverSleep0 = 0;
					}
				}

				body0->m_bodyIsConstrained = 1;
				body0->m_resting = body0->m_resting & equilibrium;
				if (body1->GetInvMass() > dFloat32(0.0f))
				{
					body1->m_bodyIsConstrained = 1;
					body1->m_resting = body1->m_resting & equilibrium;
				}
			}
		}
	};

	class ndSleep1 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = threadIndex != (threadCount - 1) ? stride : count - start;

			dInt32 activeJointCount = 0;
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();

				const dInt32 resting = body0->m_resting & body1->m_resting;
				activeJointCount += (1 - resting);
				joint->m_resting = resting;

				const dInt32 solverSleep0 = body0->m_solverSleep0 & body1->m_solverSleep0;
				if (!solverSleep0)
				{
					body0->m_solverSleep1 = 0;
					if (body1->GetInvMass() > dFloat32(0.0f))
					{
						body1->m_solverSleep1 = 0;
					}
				}
			}
			*(((dInt32*)m_context) + threadIndex) = activeJointCount;
		}
	};

	class ndScan0 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			dInt32* const histogram = ((dInt32*)m_context) + 2 * threadIndex;
			ndConstraint** const sortBuffer = (ndConstraint**)me->GetTempBuffer();

			histogram[0] = 0;
			histogram[1] = 0;
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				sortBuffer[i + start] = joint;
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();
				const dInt32 key = body0->m_solverSleep1 & body1->m_solverSleep1;
				histogram[key]++;
			}
		}
	};

	class ndSort0 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			dInt32* const histogram = ((dInt32*)m_context) + 2 * threadIndex;
			ndConstraint** const sortBuffer = (ndConstraint**)me->GetTempBuffer();
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = sortBuffer[i + start];
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();
				const dInt32 key = body0->m_solverSleep1 & body1->m_solverSleep1;
				const dInt32 entry = histogram[key];
				jointArray[entry] = joint;
				histogram[key] = entry + 1;
			}
		}
	};

	class ndRowScan : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			dInt32* const jointCountSpans = ((dInt32*)m_context) + 128 * threadIndex;
			ndConstraint** const sortBuffer = (ndConstraint**)me->GetTempBuffer();

			memset(jointCountSpans, 0, 128 * sizeof(dInt32));
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				sortBuffer[i + start] = joint;

				dAssert((joint->GetBody0()->m_resting & joint->GetBody1()->m_resting) == joint->m_resting);
				const ndSortKey key(joint->m_resting, joint->m_rowCount);

				dAssert(key.m_value >= 0);
				dAssert(key.m_value <= 127);
				jointCountSpans[key.m_value] ++;
			}
		}
	};

	class ndSortByRows : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			dInt32* const jointCountSpans = ((dInt32*)m_context) + 128 * threadIndex;
			ndConstraint** const sortBuffer = (ndConstraint**)me->GetTempBuffer();
			ndJointBodyPairIndex* const jointBodyBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = sortBuffer[i + start];
				const ndBodyKinematic* const body0 = joint->GetBody0();
				const ndBodyKinematic* const body1 = joint->GetBody1();
				dAssert((body0->m_resting & body1->m_resting) == joint->m_resting);
				const ndSortKey key(joint->m_resting, joint->m_rowCount);
				dAssert(key.m_value >= 0);
				dAssert(key.m_value <= 127);

				const dInt32 entry = jointCountSpans[key.m_value];
				jointArray[entry] = joint;

				const dInt32 m0 = body0->m_index;
				const dInt32 m1 = body1->m_index;
				jointBodyBuffer[entry * 2 + 0].m_body = m0;
				jointBodyBuffer[entry * 2 + 0].m_joint = entry * 2 + 0;
				jointBodyBuffer[entry * 2 + 1].m_body = m1;
				jointBodyBuffer[entry * 2 + 1].m_joint = entry * 2 + 1;

				jointCountSpans[key.m_value] = entry + 1;
			}
		}
	};

	class ndJointBodySortKey
	{
		public:
		dInt32 m_lowCount;
		dInt32 m_hightCount;
	};

	class ndBodyJointJorcesScan : public ndScene::ndBaseJob
	{
		public:
		ndBodyJointJorcesScan()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			const dArray<ndJointBodyPairIndex>& bodyJointPairs = me->GetJointBodyPairIndexBuffer();
			const dInt32 count = bodyJointPairs.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			ndJointBodySortKey* const histogram = ((ndJointBodySortKey*)m_context) + threadIndex * D_MAX_BODY_RADIX_DIGIT;
			memset(histogram, 0, D_MAX_BODY_RADIX_DIGIT * sizeof(ndJointBodySortKey));
			for (dInt32 i = 0; i < blockSize; i++)
			{
				dUnsigned32 key = bodyJointPairs[i + start].m_body;
				dInt32 radix0 = key & D_MAX_BODY_RADIX_MASK;
				dInt32 radix1 = key >> D_MAX_BODY_RADIX_BIT;
				histogram[radix0].m_lowCount++;
				histogram[radix1].m_hightCount++;
			}
		}
	};

	class ndBodyJointJorcesSortLowDigit : public ndScene::ndBaseJob
	{
		public:

		ndBodyJointJorcesSortLowDigit()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			const dArray<ndJointBodyPairIndex>& bodyJointPairs = me->GetJointBodyPairIndexBuffer();
			const dInt32 count = bodyJointPairs.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			ndJointBodySortKey* const histogram = ((ndJointBodySortKey*)m_context) + threadIndex * D_MAX_BODY_RADIX_DIGIT;
			ndJointBodyPairIndex* const sortBuffer = (ndJointBodyPairIndex*)me->GetTempBuffer();
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndJointBodyPairIndex pair (bodyJointPairs[i + start]);
				dUnsigned32 key = pair.m_body & D_MAX_BODY_RADIX_MASK;
				dInt32 index = histogram[key].m_lowCount;
				sortBuffer[index] = pair;
				histogram[key].m_lowCount++;
			}
		}
	};

	class ndBodyJointJorcesSortHighDigit : public ndScene::ndBaseJob
	{
		public:
		ndBodyJointJorcesSortHighDigit()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			dArray<ndJointBodyPairIndex>& bodyJointPairs = me->GetJointBodyPairIndexBuffer();
			const dInt32 count = bodyJointPairs.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			ndJointBodySortKey* const histogram = ((ndJointBodySortKey*)m_context) + threadIndex * D_MAX_BODY_RADIX_DIGIT;
			const ndJointBodyPairIndex* const sortBuffer = (ndJointBodyPairIndex*)me->GetTempBuffer();
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndJointBodyPairIndex pair(sortBuffer[i + start]);
				dUnsigned32 key = pair.m_body >> D_MAX_BODY_RADIX_BIT;
				dInt32 index = histogram[key].m_hightCount;
				bodyJointPairs[index] = pair;
				histogram[key].m_hightCount++;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();

	for (ndSkeletonList::dNode* node = m_world->GetSkeletonList().GetFirst(); node; node = node->GetNext())
	{
		ndSkeletonContainer* const skeleton = &node->GetInfo();
		skeleton->CheckSleepState();
	}

	const ndJointList& jointList = m_world->GetJointList();
	ndConstraintArray& jointArray = scene->GetActiveContactArray();

	dInt32 index = jointArray.GetCount();
	jointArray.SetCount(index + jointList.GetCount());

	for (ndJointList::dNode* node = jointList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		if (joint->IsActive())
		{
			jointArray[index] = joint;
			index++;
		}
	}
	jointArray.SetCount(index);

	m_leftHandSide.SetCount(jointArray.GetCount() + 32);

	dInt32 histogram[D_MAX_THREADS_COUNT][2];
	dInt32 movingJoints[D_MAX_THREADS_COUNT];
	const dInt32 threadCount = scene->GetThreadCount();

	scene->SubmitJobs<ndSleep0>();
	scene->SubmitJobs<ndSleep1>(movingJoints);
	scene->SubmitJobs<ndScan0>(histogram);

	dInt32 scan[2];
	scan[0] = 0;
	scan[1] = 0;
	dInt32 movingJointCount = 0;
	for (dInt32 i = 0; i < threadCount; i++)
	{
		scan[0] += histogram[i][0];
		scan[1] += histogram[i][1];
		movingJointCount += movingJoints[i];
	}

	m_activeJointCount = scan[0];
	dAssert(m_activeJointCount <= jointArray.GetCount());
	if (!m_activeJointCount)
	{
		jointArray.SetCount(0);
		return;
	}

	dInt32 sum = 0;
	for (dInt32 i = 0; i < 2; i++)
	{
		for (dInt32 j = 0; j < threadCount; j++)
		{
			dInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
	}
	scene->SubmitJobs<ndSort0>(histogram);

	#ifdef _DEBUG
		for (dInt32 i = 0; i < (jointArray.GetCount() - 1); i++)
		{
			const ndConstraint* const joint0 = jointArray[i];
			const ndConstraint* const joint1 = jointArray[i + 1];
			const dInt32 key0 = (joint0->GetBody0()->m_solverSleep1 & joint0->GetBody1()->m_solverSleep1) ? 1 : 0;
			const dInt32 key1 = (joint1->GetBody0()->m_solverSleep1 & joint1->GetBody1()->m_solverSleep1) ? 1 : 0;
			dAssert(key0 <= key1);
		}
	#endif

	dAssert(m_activeJointCount <= jointArray.GetCount());
	jointArray.SetCount(m_activeJointCount);
	for (dInt32 i = jointArray.GetCount() - 1; i >= 0; i--)
	{
		ndConstraint* const joint = jointArray[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();

		if (body1->GetInvMass() > dFloat32(0.0f))
		{
			ndBodyKinematic* root0 = FindRootAndSplit(body0);
			ndBodyKinematic* root1 = FindRootAndSplit(body1);
			if (root0 != root1)
			{
				if (root0->m_rank > root1->m_rank)
				{
					dSwap(root0, root1);
				}
				root0->m_islandParent = root1;
				if (root0->m_rank == root1->m_rank)
				{
					root1->m_rank += 1;
					dAssert(root1->m_rank <= 6);
				}
			}

			const dInt32 sleep = body0->m_islandSleep & body1->m_islandSleep;
			if (!sleep)
			{
				dAssert(root1->m_islandParent == root1);
				root1->m_islandSleep = 0;
			}
		}
		else
		{
			if (!body0->m_islandSleep)
			{
				ndBodyKinematic* const root = FindRootAndSplit(body0);
				root->m_islandSleep = 0;
			}
		}
	}
	m_activeJointCount = movingJointCount;

	dInt32 jointRowScans[D_MAX_THREADS_COUNT][128];
	scene->SubmitJobs<ndRowScan>(jointRowScans);
	dInt32 acc = 0;
	for (dInt32 i = 0; i < 128; i++)
	{
		for (dInt32 j = 0; j < threadCount; j++)
		{
			dInt32 temp = jointRowScans[j][i];
			jointRowScans[j][i] = acc;
			acc += temp;
		}
	}

	dArray<ndJointBodyPairIndex>& jointBodyBuffer = GetJointBodyPairIndexBuffer();
	jointBodyBuffer.SetCount(jointArray.GetCount() * 2);
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);
	scene->SubmitJobs<ndSortByRows>(jointRowScans);

	ndJointBodySortKey bodyJointHistogram[D_MAX_THREADS_COUNT][D_MAX_BODY_RADIX_DIGIT];
	scene->SubmitJobs<ndBodyJointJorcesScan>(bodyJointHistogram);
	
	dInt32 lowDigitSum = 0;
	dInt32 highDigitSum = 0;
	for (dInt32 i = 0; i < D_MAX_BODY_RADIX_DIGIT; i ++)
	{
		for (dInt32 j = 0; j < threadCount; j++)
		{
			dInt32 lowDigit = bodyJointHistogram[j][i].m_lowCount;
			dInt32 highDigit = bodyJointHistogram[j][i].m_hightCount;
			bodyJointHistogram[j][i].m_lowCount = lowDigitSum;
			bodyJointHistogram[j][i].m_hightCount = highDigitSum;
			lowDigitSum += lowDigit;
			highDigitSum += highDigit;
		}
	}

	scene->SubmitJobs<ndBodyJointJorcesSortLowDigit>(bodyJointHistogram);
	scene->SubmitJobs<ndBodyJointJorcesSortHighDigit>(bodyJointHistogram);

	dArray<dInt32>& bodyJointIndex = GetJointForceIndexBuffer();
	dInt32 bodyJointIndexCount = scene->GetActiveBodyArray().GetCount() + 1;
	bodyJointIndex.SetCount(bodyJointIndexCount);
	dInt32 sizeInJacobian = (bodyJointIndexCount * sizeof(dInt32)) / sizeof(ndJacobian);
	ClearJacobianBuffer(sizeInJacobian, (ndJacobian*)&bodyJointIndex[0]);
	for (dInt32 i = (sizeInJacobian * sizeof(ndJacobian)) / sizeof(dInt32); i < bodyJointIndexCount; i++)
	{
		bodyJointIndex[i] = 0;
	}

	dInt32 rowCount = 1;
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();
		dInt32 m0 = body0->m_index;
		dInt32 m1 = body1->m_index;
		bodyJointIndex[m0] ++;
		bodyJointIndex[m1] ++;
		//dTrace(("id(%d %d) index(%d %d)\n", body0->GetId(), body1->GetId(), body0->m_index, body1->m_index));
		joint->m_rowStart = rowCount;
		rowCount += joint->m_rowCount;
	}

	dInt32 bodyJointIndexAcc = 0;
	for (dInt32 i = 0; i < bodyJointIndexCount; i++)
	{
		dInt32 count = bodyJointIndex[i];
		bodyJointIndex[i] = bodyJointIndexAcc;
		bodyJointIndexAcc += count;
	}

	m_leftHandSide.SetCount(rowCount);
	m_rightHandSide.SetCount(rowCount);

	#ifdef _DEBUG
		dAssert(m_activeJointCount <= jointArray.GetCount());
		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			dAssert(joint->m_rowStart < m_leftHandSide.GetCount());
			dAssert((joint->m_rowStart + joint->m_rowCount) <= rowCount);
		}

		for (dInt32 i = 1; i < m_activeJointCount; i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(!joint0->m_resting);
			dAssert(!joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(!(joint0->GetBody0()->m_resting & joint0->GetBody1()->m_resting));
			dAssert(!(joint1->GetBody0()->m_resting & joint1->GetBody1()->m_resting));
		}

		for (dInt32 i = m_activeJointCount + 1; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(joint0->m_resting);
			dAssert(joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(joint0->GetBody0()->m_resting & joint0->GetBody1()->m_resting);
			dAssert(joint1->GetBody0()->m_resting & joint1->GetBody1()->m_resting);
		}
	#endif
}

dInt32 ndDynamicsUpdateOpencl::CompareIslands(const ndIsland* const islandA, const ndIsland* const islandB, void* const)
{
	dUnsigned32 keyA = islandA->m_count * 2 + islandA->m_root->m_bodyIsConstrained;
	dUnsigned32 keyB = islandB->m_count * 2 + islandB->m_root->m_bodyIsConstrained;;
	if (keyA < keyB)
	{
		return 1;
	}
	else if (keyA > keyB)
	{
		return -1;
	}
	return 0;
}

void ndDynamicsUpdateOpencl::RadixSort()
{
	dInt32 elements = m_islands.GetCount();
	if (elements < 256)
	{
		dSort(&m_islands[0], elements, CompareIslands);
	}
	else
	{
		const dInt32 radixBits = 9;
		const dInt32 radix = (1 << radixBits) - 1;
		dInt32 histogram[2][1 << radixBits];

		memset(histogram, 0, sizeof(histogram));
		for (dInt32 i = 0; i < elements; i++)
		{
			dUnsigned32 key = m_islands[i].m_count * 2 + m_islands[i].m_root->m_bodyIsConstrained;
			dInt32 radix0 = radix - (key & radix);
			dInt32 radix1 = radix - (key >> radixBits);
			histogram[0][radix0]++;
			histogram[1][radix1]++;
		}

		dInt32 acc0 = 0;
		dInt32 acc1 = 0;
		for (dInt32 i = 0; i <= radix; i++)
		{
			const dInt32 n0 = histogram[0][i];
			const dInt32 n1 = histogram[1][i];
			histogram[0][i] = acc0;
			histogram[1][i] = acc1;
			acc0 += n0;
			acc1 += n1;
		}

		//ndIsland* const buffer = dAlloca(ndIsland, elements);
		ndIsland* const buffer = (ndIsland*)GetTempBuffer();

		dInt32* const scan0 = &histogram[0][0];
		for (dInt32 i = 0; i < elements; i++)
		{
			dUnsigned32 key = radix - ((m_islands[i].m_count * 2 + m_islands[i].m_root->m_bodyIsConstrained) & radix);
			const dInt32 index = scan0[key];
			buffer[index] = m_islands[i];
			scan0[key] = index + 1;
		}

		dInt32* const scan1 = &histogram[1][0];
		for (dInt32 i = 0; i < elements; i++)
		{
			dUnsigned32 key = radix - ((buffer[i].m_count * 2 + buffer[i].m_root->m_bodyIsConstrained) >> radixBits);
			const dInt32 index = scan1[key];
			m_islands[index] = buffer[i];
			scan1[key] = index + 1;
		}

		#ifdef _DEBUG
			for (dInt32 i = elements - 1; i; i--)
			{
				dUnsigned32 key0 = m_islands[i - 1].m_count * 2 + m_islands[i - 1].m_root->m_bodyIsConstrained;
				dUnsigned32 key1 = m_islands[i + 0].m_count * 2 + m_islands[i + 0].m_root->m_bodyIsConstrained;
				dAssert(key0 >= key1);
			}
		#endif
	}
}

void ndDynamicsUpdateOpencl::SortIslands()
{
	D_TRACKTIME();

	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	GetInternalForces().SetCount(bodyArray.GetCount());

	dInt32 count = 0;
	ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&GetInternalForces()[0];
	for (dInt32 i = bodyArray.GetCount() - 2; i >= 0; i--)
	{
		ndBodyKinematic* const body = bodyArray[i];
		if (!(body->m_resting & body->m_islandSleep) || body->GetAsBodyPlayerCapsule())
		{
			buffer0[count].m_body = body;
			if (body->GetInvMass() > dFloat32(0.0f))
			{
				ndBodyKinematic* root = body->m_islandParent;
				while (root != root->m_islandParent)
				{
					root = root->m_islandParent;
				}

				buffer0[count].m_root = root;
				if (root->m_rank != -1)
				{
					root->m_rank = -1;
				}
			}
			else
			{
				buffer0[count].m_root = body;
				body->m_rank = -1;
			}
			count++;
		}
	}

	m_islands.SetCount(0);
	m_bodyIslandOrder.SetCount(count);
	m_unConstrainedBodyCount = 0;
	if (count)
	{
		// sort using counting sort o(n)
		dInt32 scans[2];
		scans[0] = 0;
		scans[1] = 0;
		for (dInt32 i = 0; i < count; i++)
		{
			dInt32 j = 1 - buffer0[i].m_root->m_bodyIsConstrained;
			scans[j] ++;
		}
		scans[1] = scans[0];
		scans[0] = 0;
		ndBodyIndexPair* const buffer2 = buffer0 + count;
		for (dInt32 i = 0; i < count; i++)
		{
			const dInt32 key = 1 - buffer0[i].m_root->m_bodyIsConstrained;
			const dInt32 j = scans[key];
			buffer2[j] = buffer0[i];
			scans[key] = j + 1;
		}

		const ndBodyIndexPair* const buffer1 = buffer0 + count;
		for (dInt32 i = 0; i < count; i++)
		{
			dAssert((i == count - 1) || (buffer1[i].m_root->m_bodyIsConstrained >= buffer1[i + 1].m_root->m_bodyIsConstrained));

			m_bodyIslandOrder[i] = buffer1[i].m_body;
			if (buffer1[i].m_root->m_rank == -1)
			{
				buffer1[i].m_root->m_rank = 0;
				ndIsland island(buffer1[i].m_root);
				m_islands.PushBack(island);
			}
			buffer1[i].m_root->m_rank += 1;
		}

		dInt32 start = 0;
		dInt32 unConstrainedCount = 0;
		for (dInt32 i = 0; i < m_islands.GetCount(); i++)
		{
			ndIsland& island = m_islands[i];
			island.m_start = start;
			island.m_count = island.m_root->m_rank;
			start += island.m_count;
			unConstrainedCount += island.m_root->m_bodyIsConstrained ? 0 : 1;
		}

		m_unConstrainedBodyCount = unConstrainedCount;
		RadixSort();
	}
}

void ndDynamicsUpdateOpencl::IntegrateUnconstrainedBodies()
{
	class ndIntegrateUnconstrainedBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			dArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const dFloat32 timestep = m_timestep;
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = me->GetUnconstrainedBodyCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;
			const dInt32 base = bodyArray.GetCount() - bodyCount + start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndBodyKinematic* const body = bodyArray[base + i]->GetAsBodyKinematic();
				dAssert(body);
				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(m_timestep);
				body->IntegrateExternalForce(timestep);
			}
		}
	};

	if (m_unConstrainedBodyCount)
	{
		D_TRACKTIME();
		ndScene* const scene = m_world->GetScene();
		scene->SubmitJobs<ndIntegrateUnconstrainedBodies>();
	}
}

void ndDynamicsUpdateOpencl::InitWeights()
{
	D_TRACKTIME();
	class ndInitWeights : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			dFloat32 maxExtraPasses = dFloat32(1.0f);
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			const dInt32 stride = jointCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : jointCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const constraint = jointArray[i + start];
				ndBodyKinematic* const body0 = constraint->GetBody0();
				ndBodyKinematic* const body1 = constraint->GetBody1();

				if (body1->GetInvMass() > dFloat32(0.0f))
				{
					dScopeSpinLock lock(body1->m_lock);
					body1->m_weigh += dFloat32(1.0f);
					maxExtraPasses = dMax(body1->m_weigh, maxExtraPasses);
				}
				else if (body1->m_weigh != dFloat32(1.0f))
				{
					body1->m_weigh = dFloat32(1.0f);
				}
				dScopeSpinLock lock(body0->m_lock);
				body0->m_weigh += dFloat32(1.0f);
				dAssert(body0->GetInvMass() != dFloat32(0.0f));
				maxExtraPasses = dMax(body0->m_weigh, maxExtraPasses);
			}
			dFloat32* const extraPasses = (dFloat32*)m_context;
			extraPasses[threadIndex] = maxExtraPasses;
		}
	};

	ndScene* const scene = m_world->GetScene();
	m_invTimestep = dFloat32(1.0f) / m_timestep;
	m_invStepRK = dFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dFloat32(4.0f);

	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const dInt32 bodyCount = bodyArray.GetCount();
	GetInternalForces().SetCount(bodyCount);
	GetTempInternalForces().SetCount(bodyCount);

	dFloat32 extraPassesArray[D_MAX_THREADS_COUNT];
	memset(extraPassesArray, 0, sizeof(extraPassesArray));
	scene->SubmitJobs<ndInitWeights>(extraPassesArray);

	dFloat32 extraPasses = dFloat32(0.0f);
	const dInt32 threadCount = scene->GetThreadCount();
	for (dInt32 i = 0; i < threadCount; i++)
	{
		extraPasses = dMax(extraPasses, extraPassesArray[i]);
	}

	const dInt32 conectivity = 7;
	m_solverPasses = m_world->GetSolverIterations() + 2 * dInt32(extraPasses) / conectivity + 1;
}

void ndDynamicsUpdateOpencl::InitBodyArray()
{
	D_TRACKTIME();
	class ndInitBodyArray : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			dArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - me->GetUnconstrainedBodyCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndBodyDynamic* const body = bodyArray[i + start]->GetAsBodyDynamic();
				if (body)
				{
					dAssert(body->m_bodyIsConstrained);
					body->UpdateInvInertiaMatrix();
					body->AddDampingAcceleration(m_timestep);
					const dVector angularMomentum(body->CalculateAngularMomentum());
					body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
					body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

					body->m_accel = body->m_veloc;
					body->m_alpha = body->m_omega;
					body->m_gyroRotation = body->m_rotation;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitBodyArray>();
}

void ndDynamicsUpdateOpencl::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	dAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (dInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}

	constraintParam.m_rowsCount = 0;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	joint->JacobianDerivative(constraintParam);
	const dInt32 dof = constraintParam.m_rowsCount;
	dAssert(dof <= joint->m_rowCount);

	if (joint->GetAsContact())
	{
		ndContact* const contactJoint = joint->GetAsContact();
		contactJoint->m_isInSkeletonLoop = 0;
		ndSkeletonContainer* const skeleton0 = contactJoint->GetBody0()->GetSkeleton();
		ndSkeletonContainer* const skeleton1 = contactJoint->GetBody1()->GetSkeleton();
		if (skeleton0 && (skeleton0 == skeleton1))
		{
			if (contactJoint->IsSkeletonSelftCollision())
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
		}
		else if (contactJoint->IsSkeletonIntraCollision())
		{
			if (skeleton0 && !skeleton1)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
			else if (skeleton1 && !skeleton0)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton1->AddSelfCollisionJoint(contactJoint);
			}
		}
	}
	else
	{
		ndJointBilateralConstraint* const bilareral = joint->GetAsBilateral();
		dAssert(bilareral);
		if (!bilareral->m_isInSkeleton && (bilareral->GetSolverModel() == m_jointkinematicAttachment))
		{
			ndSkeletonContainer* const skeleton0 = bilareral->m_body0->GetSkeleton();
			ndSkeletonContainer* const skeleton1 = bilareral->m_body1->GetSkeleton();
			if (skeleton0 || skeleton1)
			{
				if (skeleton0 && !skeleton1)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton0->AddSelfCollisionJoint(bilareral);
				}
				else if (skeleton1 && !skeleton0)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton1->AddSelfCollisionJoint(bilareral);
				}
			}
		}
	}

	joint->m_rowCount = dof;
	const dInt32 baseIndex = joint->m_rowStart;
	for (dInt32 i = 0; i < dof; i++)
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);

		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = dFloat32(0.0f);
		rhs->m_diagonalRegularizer = dMax(constraintParam.m_diagonalRegularizer[i], dFloat32(1.0e-5f));

		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
		rhs->m_restitution = constraintParam.m_restitution[i];
		rhs->m_penetration = constraintParam.m_penetration[i];
		rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
		rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
		rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
		rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;

		dAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
		const dInt32 frictionIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		const dInt32 mask = frictionIndex >> 31;
		rhs->m_normalForceIndex = frictionIndex;
		rhs->m_normalForceIndexFlat = ~mask & (frictionIndex + baseIndex);
	}
}

void ndDynamicsUpdateOpencl::InitJacobianMatrix()
{
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:
		ndInitJacobianMatrix()
			:m_zero(dVector::m_zero)
		{
		}

		void BuildJacobianMatrix(ndConstraint* const joint, dInt32 jointIndex)
		{
			dAssert(joint->GetBody0());
			dAssert(joint->GetBody1());
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndBodyDynamic* const dynBody0 = body0->GetAsBodyDynamic();
			const ndBodyDynamic* const dynBody1 = body1->GetAsBodyDynamic();

			const dInt32 m0 = body0->m_index;
			const dInt32 m1 = body1->m_index;
			const dInt32 index = joint->m_rowStart;
			const dInt32 count = joint->m_rowCount;
			const dMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const dMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
			const dVector invMass0(body0->m_invMass[3]);
			const dVector invMass1(body1->m_invMass[3]);

			dVector force0(m_zero);
			dVector torque0(m_zero);
			if (dynBody0)
			{
				force0 = dynBody0->m_externalForce;
				torque0 = dynBody0->m_externalTorque;
			}

			dVector force1(m_zero);
			dVector torque1(m_zero);
			if (dynBody1)
			{
				force1 = dynBody1->m_externalForce;
				torque1 = dynBody1->m_externalTorque;
			}

			joint->m_preconditioner0 = dFloat32(1.0f);
			joint->m_preconditioner1 = dFloat32(1.0f);
			if ((invMass0.GetScalar() > dFloat32(0.0f)) && (invMass1.GetScalar() > dFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton()))
			{
				const dFloat32 mass0 = body0->GetMassMatrix().m_w;
				const dFloat32 mass1 = body1->GetMassMatrix().m_w;
				if (mass0 > (D_DIAGONAL_PRECONDITIONER * mass1))
				{
					joint->m_preconditioner0 = mass0 / (mass1 * D_DIAGONAL_PRECONDITIONER);
				}
				else if (mass1 > (D_DIAGONAL_PRECONDITIONER * mass0))
				{
					joint->m_preconditioner1 = mass1 / (mass0 * D_DIAGONAL_PRECONDITIONER);
				}
			}

			dVector forceAcc0(m_zero);
			dVector torqueAcc0(m_zero);
			dVector forceAcc1(m_zero);
			dVector torqueAcc1(m_zero);

			const dVector weigh0(body0->m_weigh * joint->m_preconditioner0);
			const dVector weigh1(body1->m_weigh * joint->m_preconditioner0);

			const dFloat32 preconditioner0 = joint->m_preconditioner0;
			const dFloat32 preconditioner1 = joint->m_preconditioner1;

			const bool isBilateral = joint->IsBilateral();
			for (dInt32 i = 0; i < count; i++)
			{
				ndLeftHandSide* const row = &m_leftHandSide[index + i];
				ndRightHandSide* const rhs = &m_rightHandSide[index + i];

				row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
				row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
				row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
				row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

				const ndJacobian& JMinvM0 = row->m_JMinv.m_jacobianM0;
				const ndJacobian& JMinvM1 = row->m_JMinv.m_jacobianM1;
				const dVector tmpAccel(
					JMinvM0.m_linear * force0 + JMinvM0.m_angular * torque0 +
					JMinvM1.m_linear * force1 + JMinvM1.m_angular * torque1);

				const dFloat32 extenalAcceleration = -tmpAccel.AddHorizontal().GetScalar();
				rhs->m_deltaAccel = extenalAcceleration;
				rhs->m_coordenateAccel += extenalAcceleration;
				dAssert(rhs->m_jointFeebackForce);
				const dFloat32 force = rhs->m_jointFeebackForce->GetInitiailGuess();

				rhs->m_force = isBilateral ? dClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
				rhs->m_maxImpact = dFloat32(0.0f);

				const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
				const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
				const dVector tmpDiag(
					weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
					weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));

				dFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
				dAssert(diag > dFloat32(0.0f));
				rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

				diag *= (dFloat32(1.0f) + rhs->m_diagonalRegularizer);
				rhs->m_invJinvMJt = dFloat32(1.0f) / diag;

				dVector f0(rhs->m_force * preconditioner0);
				dVector f1(rhs->m_force * preconditioner1);
				forceAcc0 = forceAcc0 + JtM0.m_linear * f0;
				torqueAcc0 = torqueAcc0 + JtM0.m_angular * f0;
				forceAcc1 = forceAcc1 + JtM1.m_linear * f1;
				torqueAcc1 = torqueAcc1 + JtM1.m_angular * f1;
			}

			//if (body1->GetInvMass() > dFloat32(0.0f))
			//{
			//	ndJacobian& outBody1 = m_internalForces[m1];
			//	dScopeSpinLock lock(body1->m_lock);
			//	outBody1.m_linear += forceAcc1;
			//	outBody1.m_angular += torqueAcc1;
			//}
			//ndJacobian& outBody0 = m_internalForces[m0];
			//dScopeSpinLock lock(body0->m_lock);
			//outBody0.m_linear += forceAcc0;
			//outBody0.m_angular += torqueAcc0;

			const dInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = m_internalForces[index0];
			outBody0.m_linear = forceAcc0;
			outBody0.m_angular = torqueAcc0;

			const dInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = m_internalForces[index1];
			outBody1.m_linear = forceAcc1;
			outBody1.m_angular = torqueAcc1;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			ndConstraint** const jointArray = &m_owner->GetActiveContactArray()[0];
			const dInt32 jointCount = m_owner->GetActiveContactArray().GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				me->GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint, i);
			}
		}

		dVector m_zero;
		ndJacobian* m_internalForces;
		ndLeftHandSide* m_leftHandSide;
		ndRightHandSide* m_rightHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dVector zero(dVector::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
	
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;
		
			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 startIndex = bodyIndex[i + start];
				dInt32 count = bodyIndex[i + start + 1] - startIndex;
				if (count) 
				{
					dVector force(zero);
					dVector torque(zero);
					for (dInt32 j = 0; j < count; j++)
					{
						dInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
						force += jointInternalForces[index].m_linear;
						torque += jointInternalForces[index].m_angular;
					}
					internalForces[i + start].m_linear = force;
					internalForces[i + start].m_angular = torque;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = dFloat32(1.0f);
		ClearJacobianBuffer(GetInternalForces().GetCount(), &GetInternalForces()[0]);
		scene->SubmitJobs<ndInitJacobianMatrix>();
		scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();
	}
}

void ndDynamicsUpdateOpencl::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	class ndCalculateJointsAcceleration : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timestep = me->m_timestepRK;
			joindDesc.m_invTimestep = me->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = me->m_firstPassCoef;
			dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 pairStart = joint->m_rowStart;
				joindDesc.m_rowsCount = joint->m_rowCount;
				joindDesc.m_leftHandSide = &leftHandSide[pairStart];
				joindDesc.m_rightHandSide = &rightHandSide[pairStart];
				joint->JointAccelerations(&joindDesc);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndCalculateJointsAcceleration>();
	m_firstPassCoef = dFloat32(1.0f);
}

void ndDynamicsUpdateOpencl::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			dArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();
			const dArray<ndJacobian>& internalForces = me->GetInternalForces();

			const dVector timestep4(me->m_timestepRK);
			const dVector speedFreeze2(world->m_freezeSpeed2 * dFloat32(0.1f));

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - me->GetUnconstrainedBodyCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndBodyDynamic* const body = bodyArray[i + start]->GetAsBodyDynamic();
				if (body)
				{
					dAssert(body->m_bodyIsConstrained);
					const dInt32 index = body->m_index;
					const ndJacobian& forceAndTorque = internalForces[index];
					const dVector force(body->GetForce() + forceAndTorque.m_linear);
					const dVector torque(body->GetTorque() + forceAndTorque.m_angular - body->GetGyroTorque());

					ndJacobian velocStep(body->IntegrateForceAndToque(force, torque, timestep4));

					if (!body->m_resting)
					{
						body->m_veloc += velocStep.m_linear;
						body->m_omega += velocStep.m_angular;
						body->IntegrateGyroSubstep(timestep4);
					}
					else
					{
						const dVector velocStep2(velocStep.m_linear.DotProduct(velocStep.m_linear));
						const dVector omegaStep2(velocStep.m_angular.DotProduct(velocStep.m_angular));
						const dVector test(((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2)) & dVector::m_negOne);
						const dInt32 equilibrium = test.GetSignMask() ? 0 : 1;
						body->m_resting &= equilibrium;
					}
					dAssert(body->m_veloc.m_w == dFloat32(0.0f));
					dAssert(body->m_omega.m_w == dFloat32(0.0f));
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodiesVelocity>();
}

void ndDynamicsUpdateOpencl::UpdateForceFeedback()
{
	D_TRACKTIME();
	class ndUpdateForceFeedback : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();
			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			const dFloat32 timestepRK = me->m_timestepRK;
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 rows = joint->m_rowCount;
				const dInt32 first = joint->m_rowStart;

				for (dInt32 j = 0; j < rows; j++)
				{
					const ndRightHandSide* const rhs = &rightHandSide[j + first];
					dAssert(dCheckFloat(rhs->m_force));
					rhs->m_jointFeebackForce->Push(rhs->m_force);
					rhs->m_jointFeebackForce->m_force = rhs->m_force;
					rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
				}

				if (joint->GetAsBilateral())
				{
					const dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
					dVector force0(dVector::m_zero);
					dVector force1(dVector::m_zero);
					dVector torque0(dVector::m_zero);
					dVector torque1(dVector::m_zero);

					for (dInt32 j = 0; j < rows; j++)
					{
						const ndRightHandSide* const rhs = &rightHandSide[j + first];
						const ndLeftHandSide* const lhs = &leftHandSide[j + first];
						const dVector f(rhs->m_force);
						force0 += lhs->m_Jt.m_jacobianM0.m_linear * f;
						torque0 += lhs->m_Jt.m_jacobianM0.m_angular * f;
						force1 += lhs->m_Jt.m_jacobianM1.m_linear * f;
						torque1 += lhs->m_Jt.m_jacobianM1.m_angular * f;
					}
					ndJointBilateralConstraint* const bilateral = joint->GetAsBilateral();
					bilateral->m_forceBody0 = force0;
					bilateral->m_torqueBody0 = torque0;
					bilateral->m_forceBody1 = force1;
					bilateral->m_torqueBody1 = torque1;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateForceFeedback>();
}

void ndDynamicsUpdateOpencl::IntegrateBodies()
{
	D_TRACKTIME();
	class ndIntegrateBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			dArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const dFloat32 timestep = m_timestep;
			const dVector invTime(me->m_invTimestep);

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndBodyDynamic* const dynBody = bodyArray[i + start]->GetAsBodyDynamic();

				// the initial velocity and angular velocity were stored in m_accel and dynBody->m_alpha for memory saving
				if (dynBody)
				{
					if (!dynBody->m_equilibrium)
					{
						dynBody->m_accel = invTime * (dynBody->m_veloc - dynBody->m_accel);
						dynBody->m_alpha = invTime * (dynBody->m_omega - dynBody->m_alpha);
						dynBody->IntegrateVelocity(timestep);
					}
				}
				else
				{
					ndBodyKinematic* const kinBody = bodyArray[i]->GetAsBodyKinematic();
					dAssert(kinBody);
					if (!kinBody->m_equilibrium)
					{
						kinBody->IntegrateVelocity(timestep);
					}
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodies>();
}

void ndDynamicsUpdateOpencl::DetermineSleepStates()
{
	D_TRACKTIME();
	class ndDetermineSleepStates : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			const dArray<ndIsland>& islandArray = me->m_islands;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 islandCount = islandArray.GetCount();

			for (dInt32 i = threadIndex; i < islandCount; i += threadCount)
			{
				const ndIsland& island = islandArray[i];
				me->UpdateIslandState(island);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdateOpencl::UpdateIslandState(const ndIsland& island)
{
	dFloat32 velocityDragCoeff = D_FREEZZING_VELOCITY_DRAG;

	const dInt32 count = island.m_count;
	if (count <= D_SMALL_ISLAND_COUNT)
	{
		velocityDragCoeff = dFloat32(0.9999f);
	}

	dFloat32 maxAccel = dFloat32(0.0f);
	dFloat32 maxAlpha = dFloat32(0.0f);
	dFloat32 maxSpeed = dFloat32(0.0f);
	dFloat32 maxOmega = dFloat32(0.0f);

	const dFloat32 speedFreeze = m_world->m_freezeSpeed2;
	const dFloat32 accelFreeze = m_world->m_freezeAccel2 * ((count <= D_SMALL_ISLAND_COUNT) ? dFloat32(0.01f) : dFloat32(1.0f));
	const dFloat32 acc2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;
	const dFloat32 maxAccNorm2 = (count > 4) ? acc2 : acc2 * dFloat32(0.0625f);
	const dVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, dFloat32(0.0f));

	dInt32 stackSleeping = 1;
	dInt32 sleepCounter = 10000;
	ndBodyKinematic** const bodyIslands = &m_bodyIslandOrder[island.m_start];
	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const dynBody = bodyIslands[i]->GetAsBodyDynamic();
		if (dynBody)
		{
			dAssert(dynBody->m_accel.m_w == dFloat32(0.0f));
			dAssert(dynBody->m_alpha.m_w == dFloat32(0.0f));
			dAssert(dynBody->m_veloc.m_w == dFloat32(0.0f));
			dAssert(dynBody->m_omega.m_w == dFloat32(0.0f));

			dVector accelTest((dynBody->m_accel.DotProduct(dynBody->m_accel) > maxAccNorm2) | (dynBody->m_alpha.DotProduct(dynBody->m_alpha) > maxAccNorm2));
			dynBody->m_accel = dynBody->m_accel & accelTest;
			dynBody->m_alpha = dynBody->m_alpha & accelTest;

			dUnsigned32 equilibrium = (dynBody->GetInvMass() == dFloat32(0.0f)) ? 1 : dynBody->m_autoSleep;
			const dVector isMovingMask(dynBody->m_veloc + dynBody->m_omega + dynBody->m_accel + dynBody->m_alpha);
			const dVector mask(isMovingMask.TestZero());
			const dInt32 test = mask.GetSignMask() & 7;
			if (test != 7)
			{
				const dFloat32 accel2 = dynBody->m_accel.DotProduct(dynBody->m_accel).GetScalar();
				const dFloat32 alpha2 = dynBody->m_alpha.DotProduct(dynBody->m_alpha).GetScalar();
				const dFloat32 speed2 = dynBody->m_veloc.DotProduct(dynBody->m_veloc).GetScalar();
				const dFloat32 omega2 = dynBody->m_omega.DotProduct(dynBody->m_omega).GetScalar();

				maxAccel = dMax(maxAccel, accel2);
				maxAlpha = dMax(maxAlpha, alpha2);
				maxSpeed = dMax(maxSpeed, speed2);
				maxOmega = dMax(maxOmega, omega2);
				dUnsigned32 equilibriumTest = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

				if (equilibriumTest)
				{
					const dVector veloc(dynBody->m_veloc * velocDragVect);
					const dVector omega(dynBody->m_omega * velocDragVect);
					const dVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
					const dVector omegaMask(omega.DotProduct(omega) > m_velocTol);
					dynBody->m_veloc = velocMask & veloc;
					dynBody->m_omega = omegaMask & omega;
				}

				equilibrium &= equilibriumTest;
				stackSleeping &= equilibrium;
				sleepCounter = dMin(sleepCounter, dynBody->m_sleepingCounter);
				dynBody->m_sleepingCounter++;
			}
			if (dynBody->m_equilibrium != equilibrium)
			{
				dynBody->m_equilibrium = equilibrium;
			}
		}
		else
		{
			ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
			dAssert(kinBody);
			dUnsigned32 equilibrium = (kinBody->GetInvMass() == dFloat32(0.0f)) ? 1 : (kinBody->m_autoSleep & ~kinBody->m_equilibriumOverride);
			const dVector isMovingMask(kinBody->m_veloc + kinBody->m_omega);
			const dVector mask(isMovingMask.TestZero());
			const dInt32 test = mask.GetSignMask() & 7;
			if (test != 7)
			{
				const dFloat32 speed2 = kinBody->m_veloc.DotProduct(kinBody->m_veloc).GetScalar();
				const dFloat32 omega2 = kinBody->m_omega.DotProduct(kinBody->m_omega).GetScalar();

				maxSpeed = dMax(maxSpeed, speed2);
				maxOmega = dMax(maxOmega, omega2);
				dUnsigned32 equilibriumTest = (speed2 < speedFreeze) && (omega2 < speedFreeze);

				if (equilibriumTest)
				{
					const dVector veloc(kinBody->m_veloc * velocDragVect);
					const dVector omega(kinBody->m_omega * velocDragVect);
					const dVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
					const dVector omegaMask(omega.DotProduct(omega) > m_velocTol);
					kinBody->m_veloc = velocMask & veloc;
					kinBody->m_omega = omegaMask & omega;
				}

				equilibrium &= equilibriumTest;
				stackSleeping &= equilibrium;
				sleepCounter = dMin(sleepCounter, kinBody->m_sleepingCounter);
			}
			if (kinBody->m_equilibrium != equilibrium)
			{
				kinBody->m_equilibrium = equilibrium;
			}
		}
	}

	if (stackSleeping)
	{
		for (dInt32 i = 0; i < count; i++)
		{
			// force entire island to equilibriumTest
			ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
			if (body)
			{
				body->m_accel = dVector::m_zero;
				body->m_alpha = dVector::m_zero;
				body->m_veloc = dVector::m_zero;
				body->m_omega = dVector::m_zero;
				body->m_equilibrium = (body->GetInvMass() == dFloat32(0.0f)) ? 1 : body->m_autoSleep;
			}
			else
			{
				ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
				dAssert(kinBody);
				kinBody->m_veloc = dVector::m_zero;
				kinBody->m_omega = dVector::m_zero;
				kinBody->m_equilibrium = (kinBody->GetInvMass() == dFloat32(0.0f)) ? 1 : kinBody->m_autoSleep;
			}
		}
	}
	else if ((count > 1) || bodyIslands[0]->m_bodyIsConstrained)
	{
		const bool state =
			(maxAccel > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel) ||
			(maxAlpha > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha) ||
			(maxSpeed > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc) ||
			(maxOmega > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega);

		if (state)
		{
			for (dInt32 i = 0; i < count; i++)
			{
				ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
				if (body)
				{
					body->m_sleepingCounter = 0;
				}
			}
		}
		else
		{
			if (count < D_SMALL_ISLAND_COUNT)
			{
				// delay small islandArray for about 10 seconds
				sleepCounter >>= 8;
				for (dInt32 i = 0; i < count; i++)
				{
					ndBodyKinematic* const body = bodyIslands[i];
					body->m_equilibrium = 0;
				}
			}
			dInt32 timeScaleSleepCount = dInt32(dFloat32(60.0f) * sleepCounter * m_timestep);

			dInt32 index = D_SLEEP_ENTRIES;
			for (dInt32 i = 1; i < D_SLEEP_ENTRIES; i++)
			{
				if (m_world->m_sleepTable[i].m_steps > timeScaleSleepCount)
				{
					index = i;
					break;
				}
			}
			index--;

			bool state1 =
				(maxAccel < m_world->m_sleepTable[index].m_maxAccel) &&
				(maxAlpha < m_world->m_sleepTable[index].m_maxAlpha) &&
				(maxSpeed < m_world->m_sleepTable[index].m_maxVeloc) &&
				(maxOmega < m_world->m_sleepTable[index].m_maxOmega);
			if (state1)
			{
				for (dInt32 i = 0; i < count; i++)
				{
					ndBodyKinematic* const body = bodyIslands[i];
					body->m_veloc = dVector::m_zero;
					body->m_omega = dVector::m_zero;
					body->m_equilibrium = body->m_autoSleep;
					ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
					if (dynBody)
					{
						dynBody->m_accel = dVector::m_zero;
						dynBody->m_alpha = dVector::m_zero;
						dynBody->m_sleepingCounter = 0;
					}
				}
			}
		}
	}
}

void ndDynamicsUpdateOpencl::InitSkeletons()
{
	D_TRACKTIME();

	class ndInitSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			ndSkeletonList::dNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;
			const dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;

			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);

				for (dInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitSkeletons>();
}

void ndDynamicsUpdateOpencl::UpdateSkeletons()
{
	D_TRACKTIME();
	class ndUpdateSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			ndSkeletonList::dNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dArray<ndBodyKinematic*>& activeBodies = m_owner->ndScene::GetActiveBodyArray();
			const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)&activeBodies[0];

			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->CalculateJointForce(bodyArray, internalForces);

				for (dInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateSkeletons>();
}

void ndDynamicsUpdateOpencl::CalculateJointsForce()
{
	D_TRACKTIME();
	class ndCalculateJointsForce : public ndScene::ndBaseJob
	{
		public:
		ndCalculateJointsForce()
			:m_zero(dVector::m_zero)
		{
		}

		dFloat32 JointForce(ndConstraint* const joint, dInt32 jointIndex)
		{
			dVector accNorm(m_zero);
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			dAssert(body0);
			dAssert(body1);

			const dInt32 m0 = body0->m_index;
			const dInt32 m1 = body1->m_index;
			const dInt32 rowStart = joint->m_rowStart;
			const dInt32 rowsCount = joint->m_rowCount;

			dInt32 resting = body0->m_resting & body1->m_resting;
			if (!resting)
			{
				dVector preconditioner0(joint->m_preconditioner0);
				dVector preconditioner1(joint->m_preconditioner1);

				dVector forceM0(m_internalForces[m0].m_linear * preconditioner0);
				dVector torqueM0(m_internalForces[m0].m_angular * preconditioner0);
				dVector forceM1(m_internalForces[m1].m_linear * preconditioner1);
				dVector torqueM1(m_internalForces[m1].m_angular * preconditioner1);

				preconditioner0 = preconditioner0.Scale(body0->m_weigh);
				preconditioner1 = preconditioner1.Scale(body1->m_weigh);

				for (dInt32 j = 0; j < rowsCount; j++)
				{
					ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
					const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
					const dVector force(rhs->m_force);

					dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
					a = dVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

					dAssert(rhs->m_normalForceIndexFlat >= 0);
					dVector f(force + a.Scale(rhs->m_invJinvMJt));
					const dInt32 frictionIndex = rhs->m_normalForceIndexFlat;
					const dFloat32 frictionNormal = m_rightHandSide[frictionIndex].m_force;
					const dVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
					const dVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);
					a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
					accNorm = accNorm.MulAdd(a, a);
					rhs->m_force = f.GetScalar();

					const dVector deltaForce(f - force);
					const dVector deltaForce0(deltaForce * preconditioner0);
					const dVector deltaForce1(deltaForce * preconditioner1);

					forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
					torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
					forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
					torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
				}

				const dFloat32 tol = dFloat32(0.5f);
				const dFloat32 tol2 = tol * tol;

				dVector maxAccel(accNorm);
				for (dInt32 k = 0; (k < 4) && (maxAccel.GetScalar() > tol2); k++)
				{
					maxAccel = m_zero;
					for (dInt32 j = 0; j < rowsCount; j++)
					{
						ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
						const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
						const dVector force(rhs->m_force);

						dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
						a = dVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

						dVector f(force + a.Scale(rhs->m_invJinvMJt));
						dAssert(rhs->m_normalForceIndexFlat >= 0);
						const dInt32 frictionIndex = rhs->m_normalForceIndexFlat;
						const dFloat32 frictionNormal = m_rightHandSide[frictionIndex].m_force;

						const dVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
						const dVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

						a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
						f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
						maxAccel = maxAccel.MulAdd(a, a);
						rhs->m_force = f.GetScalar();

						const dVector deltaForce(f - force);
						const dVector deltaForce0(deltaForce * preconditioner0);
						const dVector deltaForce1(deltaForce * preconditioner1);
						forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
						torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
						forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
						torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
					}
				}
			}

			dVector forceM0(m_zero);
			dVector torqueM0(m_zero);
			dVector forceM1(m_zero);
			dVector torqueM1(m_zero);

			for (dInt32 j = 0; j < rowsCount; j++)
			{
				ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
				const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];

				const dVector f(rhs->m_force);
				forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, f);
				torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, f);
				forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, f);
				torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, f);
				rhs->m_maxImpact = dMax(dAbs(f.GetScalar()), rhs->m_maxImpact);
			}

			//if (body1->GetInvMass() > dFloat32(0.0f))
			//{
			//	ndJacobian& outBody1 = m_outputForces[m1];
			//	dScopeSpinLock lock(body1->m_lock);
			//	outBody1.m_linear += forceM1;
			//	outBody1.m_angular += torqueM1;
			//}
			//
			//ndJacobian& outBody0 = m_outputForces[m0];
			//dScopeSpinLock lock(body0->m_lock);
			//outBody0.m_linear += forceM0;
			//outBody0.m_angular += torqueM0;

			const dInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = m_jointPartialForces[index0];
			outBody0.m_linear = forceM0;
			outBody0.m_angular = torqueM0;

			const dInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = m_jointPartialForces[index1];
			outBody1.m_linear = forceM1;
			outBody1.m_angular = torqueM1;

			return accNorm.GetScalar();
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetInternalForces()[0];
			m_jointPartialForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			dFloat32 accNorm = dFloat32(0.0f);
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				accNorm += JointForce(joint, i);
			}

			dFloat32* const accelNorm = (dFloat32*)m_context;
			accelNorm[threadIndex] = accNorm;
		}

		dVector m_zero;
		ndJacobian* m_jointPartialForces;
		ndRightHandSide* m_rightHandSide;
		const ndJacobian* m_internalForces;
		const ndLeftHandSide* m_leftHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dVector zero(dVector::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 startIndex = bodyIndex[i + start];
				dInt32 count = bodyIndex[i + start + 1] - startIndex;
				if (count)
				{
					dVector force(zero);
					dVector torque(zero);
					for (dInt32 j = 0; j < count; j++)
					{
						dInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
						force += jointInternalForces[index].m_linear;
						torque += jointInternalForces[index].m_angular;
					}
					internalForces[i + start].m_linear = force;
					internalForces[i + start].m_angular = torque;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 passes = m_solverPasses;
	const dInt32 threadsCount = scene->GetThreadCount();

	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);

	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++)
	{
		scene->SubmitJobs<ndCalculateJointsForce>(m_accelNorm);
		scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();

		accNorm = dFloat32(0.0f);
		for (dInt32 j = 0; j < threadsCount; j++)
		{
			accNorm = dMax(accNorm, m_accelNorm[j]);
		}
	}
}

void ndDynamicsUpdateOpencl::CalculateForces()
{
	D_TRACKTIME();
	if (m_world->GetScene()->GetActiveContactArray().GetCount())
	{
		m_firstPassCoef = dFloat32(0.0f);
		if (m_world->m_skeletonList.GetCount())
		{
			InitSkeletons();
		}

		for (dInt32 step = 0; step < 4; step++)
		{
			CalculateJointsAcceleration();
			CalculateJointsForce();
			if (m_world->m_skeletonList.GetCount())
			{
				UpdateSkeletons();
			}
			IntegrateBodiesVelocity();
		}
		UpdateForceFeedback();
	}
}

void ndDynamicsUpdateOpencl::GpuUpdate()
{
	D_TRACKTIME();
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	if (m_islands.GetCount())
	{
		IntegrateUnconstrainedBodies();
		InitWeights();
		InitBodyArray();
		InitJacobianMatrix();
		CalculateForces();
		IntegrateBodies();
		DetermineSleepStates();
	}
}
