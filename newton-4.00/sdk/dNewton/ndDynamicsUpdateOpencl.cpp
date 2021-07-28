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
#include "ndDynamicsUpdateOpencl.h"
#include "ndJointBilateralConstraint.h"

#ifdef _D_NEWTON_OPENCL
#include <CL/cl.h>


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
			CL_FALSE, 0, sizeof(T) * GetCount(), source,
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

class OpenclSystem
{
	public:
	OpenclSystem(cl_context context, cl_platform_id platform)
		:m_context(context)
		//,m_bodyArray(CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR)
		,m_bodyArray(CL_MEM_READ_ONLY)
		//,m_outBodyArray(CL_MEM_WRITE_ONLY)
		,m_bodyWorkingArray(CL_MEM_READ_WRITE)
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

		char programFile[256];
		sprintf(programFile, "%s/CL/solver/solver.cl", CL_KERNEL_PATH);
		m_solverProgram = CompileProgram(programFile);

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

	cl_program CompileProgram(char* const filename)
	{
		FILE* const fp = fopen(filename, "rb");
		if (fp)
		{
			size_t sourceSize;
			fseek(fp, 0, SEEK_END);
			sourceSize = ftell(fp);
			fseek(fp, 0, SEEK_SET);

			char* const source = dAlloca(char, sourceSize * 256);
			fread(source, 1, sourceSize, fp);
			fclose(fp);

			int errorCode;
			cl_program program = clCreateProgramWithSource(m_context, 1, (const char**)&source, &sourceSize, &errorCode);
			dAssert(errorCode == CL_SUCCESS);

			errorCode = clBuildProgram(program, 1, &m_device, "", nullptr, nullptr);
			if (errorCode == CL_BUILD_PROGRAM_FAILURE)
			{
				size_t log_size = 0;
				clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);

				char* const build_log = dAlloca(char, log_size * 256);
				clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, log_size, build_log, nullptr);
				dTrace((build_log));
			}
			dAssert(errorCode == CL_SUCCESS);

			return program;
		}
		return nullptr;
	}

	static OpenclSystem* Singleton()
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

		cl_platform_id bestPlatform = 0;
		for (cl_uint i = 0; i < numPlatforms; i++)
		{
			cl_uint numDevices = 0;
			err = clGetDeviceIDs(platforms[i], CL_DEVICE_TYPE_GPU, 0, nullptr, &numDevices);
			if (!((err != CL_SUCCESS) || (numDevices == 0)))
			{
				bestPlatform = platforms[i];
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

	// Regular OpenCL objects:
	cl_context m_context;					// hold the context handler
	cl_device_id m_device;					// hold the selected device handler
	cl_program	m_solverProgram;			// hold the program handler
	cl_command_queue m_commandQueue;		// hold the commands-queue handler

	//cl_kernel        kernel;				// hold the kernel handler
	char m_platformName[128];

	dOpenclBuffer<ndOpenclBodyProxy> m_bodyArray;
	dOpenclBuffer<ndOpenclBodyWorkingBuffer> m_bodyWorkingArray;
//	dOpenclBuffer<ndOpenclOutBodyProxy> m_outBodyArray;

	cl_kernel m_integrateUnconstrainedBodies;
};

ndDynamicsUpdateOpencl::ndDynamicsUpdateOpencl(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_opencl(nullptr)
{
	m_opencl = OpenclSystem::Singleton();
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

void ndDynamicsUpdateOpencl::Update()
{
	D_TRACKTIME();
	if (m_opencl)
	{
		//ndDynamicsUpdate::Update();
		GpuUpdate();
	}
}

void ndDynamicsUpdateOpencl::GpuUpdate()
{
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	if (m_islands.GetCount())
	{
		CopyBodyData();
		IntegrateUnconstrainedBodies();

		//InitWeights();
		//InitBodyArray();
		//InitJacobianMatrix();
		//CalculateForces();
		//IntegrateBodies();
		//DetermineSleepStates();
	}
	
	//m_opencl->m_outBodyArray.ReadData(m_opencl->m_commandQueue);

	cl_int err;
	err = clFinish(m_opencl->m_commandQueue);
	dAssert(err == CL_SUCCESS);
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

	for (dInt32 i = jointArray.GetCount() - 1; i >= 0; i--)
	{
		const ndConstraint* const joint = jointArray[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
		dAssert(body0->m_solverSleep0 <= 1);
		dAssert(body1->m_solverSleep0 <= 1);

		const dInt32 resting = body0->m_equilibrium & body1->m_equilibrium;
		if (!resting)
		{
			body0->m_solverSleep0 = 0;
			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				body1->m_solverSleep0 = 0;
			}
		}
	}

	for (dInt32 i = jointArray.GetCount() - 1; i >= 0; i--)
	{
		const ndConstraint* const joint = jointArray[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
		dAssert(body0->m_solverSleep1 <= 1);
		dAssert(body1->m_solverSleep1 <= 1);

		const dInt32 test = body0->m_solverSleep0 & body1->m_solverSleep0;
		if (!test)
		{
			body0->m_solverSleep1 = 0;
			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				body1->m_solverSleep1 = 0;
			}
		}
	}

	dInt32 currentActive = jointArray.GetCount();
	for (dInt32 i = currentActive - 1; i >= 0; i--)
	{
		ndConstraint* const joint = jointArray[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
		const dInt32 test = body0->m_solverSleep1 & body1->m_solverSleep1;
		if (!test)
		{
			const dInt32 resting = (body0->m_equilibrium & body1->m_equilibrium) ? 1 : 0;
			const dInt32 rows = joint->GetRowsCount();
			joint->m_rowCount = rows;

			body0->m_bodyIsConstrained = 1;
			body0->m_resting = body0->m_resting & resting;

			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				body1->m_bodyIsConstrained = 1;
				body1->m_resting = body1->m_resting & resting;

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
		else
		{
			currentActive--;
			jointArray[i] = jointArray[currentActive];
		}
	}

	dAssert(currentActive <= jointArray.GetCount());
	jointArray.SetCount(currentActive);
	if (!jointArray.GetCount())
	{
		m_activeJointCount = 0;
		return;
	}

	dInt32 jointCountSpans[128];
	m_leftHandSide.SetCount(jointArray.GetCount() + 32);
	ndConstraint** const sortBuffer = (ndConstraint**)&m_leftHandSide[0];
	memset(jointCountSpans, 0, sizeof(jointCountSpans));

	dInt32 activeJointCount = 0;
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		sortBuffer[i] = joint;

		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();
		const dInt32 resting = (body0->m_resting & body1->m_resting) ? 1 : 0;
		activeJointCount += (1 - resting);

		const ndSortKey key(resting, joint->m_rowCount);
		dAssert(key.m_value >= 0);
		dAssert(key.m_value < dInt32 (sizeof(jointCountSpans) / sizeof(jointCountSpans[0])));
		jointCountSpans[key.m_value] ++;
	}

	dInt32 acc = 0;
	for (dInt32 i = 0; i < dInt32 (sizeof(jointCountSpans) / sizeof(jointCountSpans[0])); i++)
	{
		const dInt32 val = jointCountSpans[i];
		jointCountSpans[i] = acc;
		acc += val;
	}

	m_activeJointCount = activeJointCount;
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = sortBuffer[i];
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();
		const dInt32 resting = (body0->m_resting & body1->m_resting) ? 1 : 0;

		const ndSortKey key(resting, joint->m_rowCount);
		dAssert(key.m_value >= 0);
		dAssert(key.m_value < dInt32 (sizeof(jointCountSpans) / sizeof(jointCountSpans[0])));

		const dInt32 entry = jointCountSpans[key.m_value];
		jointArray[entry] = joint;
		jointCountSpans[key.m_value] = entry + 1;
	}

	dInt32 rowCount = 1;
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		joint->m_rowStart = rowCount;
		rowCount += joint->m_rowCount;
	}

	m_leftHandSide.SetCount(rowCount);
	m_rightHandSide.SetCount(rowCount);
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

void ndDynamicsUpdateOpencl::SortIslands()
{
	D_TRACKTIME();

	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	m_internalForces.SetCount(bodyArray.GetCount());

	dInt32 count = 0;
	ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&m_internalForces[0];
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
		D_TRACKTIME();
		dSort(&m_islands[0], m_islands.GetCount(), CompareIslands);
	}
}

void ndDynamicsUpdateOpencl::CopyBodyData()
{
	m_opencl->m_bodyArray.SyncSize(m_opencl->m_context, m_bodyIslandOrder.GetCount());
	//m_opencl->m_outBodyArray.SyncSize(m_opencl->m_context, m_bodyIslandOrder.GetCount());
	m_opencl->m_bodyWorkingArray.SyncSize(m_opencl->m_context, m_bodyIslandOrder.GetCount());

	int xxx;
	xxx = sizeof(ndOpenclBodyProxy);
	
	for (dInt32 i = 0; i < m_bodyIslandOrder.GetCount(); i++)
	{
		ndOpenclBodyProxy& data = m_opencl->m_bodyArray[i];
		ndBodyKinematic* const body = m_bodyIslandOrder[i];

		data.m_rotation.x = body->m_rotation.m_x;
		data.m_rotation.y = body->m_rotation.m_y;
		data.m_rotation.z = body->m_rotation.m_z;
		data.m_rotation.w = body->m_rotation.m_w;
		data.m_position.x = body->m_matrix.m_posit.m_x;
		data.m_position.y = body->m_matrix.m_posit.m_y;
		data.m_position.z = body->m_matrix.m_posit.m_z;
		data.m_veloc.x = body->m_veloc.m_x;
		data.m_veloc.y = body->m_veloc.m_y;
		data.m_veloc.z = body->m_veloc.m_z;
		data.m_omega.x = body->m_omega.m_x;
		data.m_omega.y = body->m_omega.m_y;
		data.m_omega.z = body->m_omega.m_z;
		data.m_invMass.x = body->m_invMass.m_x;
		data.m_invMass.y = body->m_invMass.m_y;
		data.m_invMass.z = body->m_invMass.m_z;
		data.m_invMass.w = body->m_invMass.m_w;
	}

	m_opencl->m_bodyArray.WriteData(m_opencl->m_commandQueue);
}

void ndDynamicsUpdateOpencl::IntegrateUnconstrainedBodies()
{
	//class ndIntegrateUnconstrainedBodies : public ndScene::ndBaseJob
	//{
	//	public:
	//	virtual void Execute()
	//	{
	//		D_TRACKTIME();
	//		ndWorld* const world = m_owner->GetWorld();
	//		ndDynamicsUpdateOpencl* const me = (ndDynamicsUpdateOpencl*)world->m_solver;
	//		dArray<ndBodyKinematic*>& bodyArray = me->m_bodyIslandOrder;
	//
	//		const dInt32 threadIndex = GetThreadId();
	//		const dInt32 threadCount = m_owner->GetThreadCount();
	//		const dInt32 bodyCount = me->m_unConstrainedBodyCount;
	//		const dInt32 base = bodyArray.GetCount() - bodyCount;
	//		const dInt32 step = bodyCount / threadCount;
	//		const dInt32 start = threadIndex * step;
	//		const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;
	//		const dFloat32 timestep = m_timestep;
	//
	//		for (dInt32 i = 0; i < count; i++)
	//		{
	//			ndBodyKinematic* const body = bodyArray[base + start + i]->GetAsBodyKinematic();
	//			dAssert(body);
	//			body->UpdateInvInertiaMatrix();
	//			body->AddDampingAcceleration(m_timestep);
	//			body->IntegrateExternalForce(timestep);
	//		}
	//	}
	//};
	//
	//if (m_unConstrainedBodyCount)
	//{
	//	D_TRACKTIME();
	//	ndScene* const scene = m_world->GetScene();
	//	scene->SubmitJobs<ndIntegrateUnconstrainedBodies>();
	//}

	cl_int err;
	const cl_float timestep = m_timestep;
	const cl_int bodyCount = m_unConstrainedBodyCount;

	err = clSetKernelArg(m_opencl->m_integrateUnconstrainedBodies, 0, sizeof(cl_float), &timestep);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(m_opencl->m_integrateUnconstrainedBodies, 1, sizeof(cl_int), &bodyCount);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(m_opencl->m_integrateUnconstrainedBodies, 2, sizeof(cl_mem), &m_opencl->m_bodyArray.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	err = clSetKernelArg(m_opencl->m_integrateUnconstrainedBodies, 3, sizeof(cl_mem), &m_opencl->m_bodyWorkingArray.m_gpuBuffer);
	dAssert(err == CL_SUCCESS);

	size_t globalWorkSize = bodyCount;
	err = clEnqueueNDRangeKernel(m_opencl->m_commandQueue, m_opencl->m_integrateUnconstrainedBodies, 1, nullptr, &globalWorkSize, nullptr, 0, nullptr, nullptr);
	dAssert(err == CL_SUCCESS);

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
			dArray<ndBodyKinematic*>& bodyArray = me->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 step = bodyCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;

			const dFloat32 timestep = m_timestep;
			const dVector invTime(me->m_invTimestep);
			for (dInt32 i = 0; i < count; i++)
			{
				ndBodyDynamic* const dynBody = bodyArray[start + i]->GetAsBodyDynamic();

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
					ndBodyKinematic* const kinBody = bodyArray[start + i]->GetAsBodyKinematic();
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
#endif