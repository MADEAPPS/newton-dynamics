/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_SOLVER_H_
#define _DG_SOLVER_H_


#include "dgPhysicsStdafx.h"
#include <immintrin.h>

#define DG_SOA_WORD_GROUP_SIZE	8 

#ifdef _NEWTON_USE_DOUBLE
DG_MSC_AVX_ALIGMENT
class dgSoaFloat
{
	public:
	DG_INLINE dgSoaFloat()
	{
	}

	DG_INLINE dgSoaFloat(const dgFloat32 val)
		:m_low(_mm256_set1_pd (val))
		,m_high(_mm256_set1_pd(val))
	{
	}

	DG_INLINE dgSoaFloat(const __m256d low, const __m256d high)
		:m_low(low)
		,m_high(high)
	{
	}

	DG_INLINE dgSoaFloat(const dgSoaFloat& copy)
		:m_low(copy.m_low)
		,m_high(copy.m_high)
	{
	}

	DG_INLINE dgSoaFloat(const dgSoaFloat* const baseAddr, const dgSoaFloat& index)
		:m_low(_mm256_i64gather_pd(&baseAddr->m_f[0], index.m_lowInt, 8))
		,m_high(_mm256_i64gather_pd(&baseAddr->m_f[0], index.m_highInt, 8))
	{
	}

	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert(i < DG_SOA_WORD_GROUP_SIZE);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i < DG_SOA_WORD_GROUP_SIZE);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE dgSoaFloat operator+ (const dgSoaFloat& A) const
	{
		return dgSoaFloat (_mm256_add_pd(m_low, A.m_low), _mm256_add_pd(m_high, A.m_high));
	}

	DG_INLINE dgSoaFloat operator- (const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_sub_pd(m_low, A.m_low), _mm256_sub_pd(m_high, A.m_high));
	}

	DG_INLINE dgSoaFloat operator* (const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_mul_pd(m_low, A.m_low), _mm256_mul_pd(m_high, A.m_high));
	}

	DG_INLINE dgSoaFloat MulAdd(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		return dgSoaFloat(_mm256_fmadd_pd(A.m_low, B.m_low, m_low), _mm256_fmadd_pd(A.m_high, B.m_high, m_high));
	}

	DG_INLINE dgSoaFloat MulSub(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		return dgSoaFloat(_mm256_fnmadd_pd(A.m_low, B.m_low, m_low), _mm256_fnmadd_pd(A.m_high, B.m_high, m_high));
	}

	DG_INLINE dgSoaFloat operator> (const dgSoaFloat& A) const
	{
		return dgSoaFloat (_mm256_cmp_pd (m_low, A.m_low, _CMP_GT_OQ), _mm256_cmp_pd(m_high, A.m_high, _CMP_GT_OQ));
	}

	DG_INLINE dgSoaFloat operator< (const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_cmp_pd(m_low, A.m_low, _CMP_LT_OQ), _mm256_cmp_pd(m_high, A.m_high, _CMP_LT_OQ));
	}

	DG_INLINE dgSoaFloat operator| (const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_or_pd(m_low, A.m_low), _mm256_or_pd(m_high, A.m_high));
	}

	DG_INLINE dgSoaFloat AndNot (const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_andnot_pd(A.m_low, m_low), _mm256_andnot_pd(A.m_high, m_high));
	}

	DG_INLINE dgSoaFloat GetMin(const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_min_pd(m_low, A.m_low), _mm256_min_pd(m_high, A.m_high));
	}

	DG_INLINE dgSoaFloat GetMax(const dgSoaFloat& A) const
	{
		return dgSoaFloat(_mm256_max_pd(m_low, A.m_low), _mm256_max_pd(m_high, A.m_high));
	}

	DG_INLINE dgFloat32 AddHorizontal() const
	{
		dgSoaFloat ret;
		__m256d tmp0(_mm256_add_pd(m_low, m_high));
		__m256d tmp1(_mm256_hadd_pd(tmp0, tmp0));
		__m256d tmp2(_mm256_add_pd(tmp1, _mm256_permute2f128_pd(tmp1, tmp1, 1)));
		_mm256_storeu_pd (ret.m_f, tmp2);
		return ret.m_f[0];
	}

	static DG_INLINE void FlushRegisters()
	{
		_mm256_zeroall ();
	}

	union
	{
		struct
		{
			__m256d m_low;
			__m256d m_high;
		};
		struct
		{
			__m256i m_lowInt;
			__m256i m_highInt;
		};
		dgInt64 m_i[DG_SOA_WORD_GROUP_SIZE];
		dgFloat32 m_f[DG_SOA_WORD_GROUP_SIZE];
	};
} DG_GCC_AVX_ALIGMENT;

#else 

class dgSoaFloat
{
	public:
	DG_INLINE dgSoaFloat()
	{
	}

	DG_INLINE dgSoaFloat(const dgFloat32 val)
		:m_type(_mm256_set1_ps (val))
	{
	}

	DG_INLINE dgSoaFloat(const __m256 type)
		:m_type(type)
	{
	}

	DG_INLINE dgSoaFloat(const dgSoaFloat& copy)
		:m_type(copy.m_type)
	{
	}

	DG_INLINE dgSoaFloat(const dgSoaFloat* const baseAddr, const dgSoaFloat& index)
		:m_type(_mm256_i32gather_ps(baseAddr->m_f, index.m_typeInt, 4))
	{
	}

	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert(i < DG_SOA_WORD_GROUP_SIZE);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i < DG_SOA_WORD_GROUP_SIZE);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE dgSoaFloat operator+ (const dgSoaFloat& A) const
	{
		return _mm256_add_ps(m_type, A.m_type);
	}

	DG_INLINE dgSoaFloat operator- (const dgSoaFloat& A) const
	{
		return _mm256_sub_ps(m_type, A.m_type);
	}

	DG_INLINE dgSoaFloat operator* (const dgSoaFloat& A) const
	{
		return _mm256_mul_ps(m_type, A.m_type);
	}

	DG_INLINE dgSoaFloat MulAdd(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
	}

	DG_INLINE dgSoaFloat MulSub(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		return _mm256_fnmadd_ps(A.m_type, B.m_type, m_type);
	}

	DG_INLINE dgSoaFloat operator> (const dgSoaFloat& A) const
	{
		return _mm256_cmp_ps (m_type, A.m_type, _CMP_GT_OQ);
	}

	DG_INLINE dgSoaFloat operator< (const dgSoaFloat& A) const
	{
		return _mm256_cmp_ps (m_type, A.m_type, _CMP_LT_OQ);
	}

	DG_INLINE dgSoaFloat operator| (const dgSoaFloat& A) const
	{
		return _mm256_or_ps (m_type, A.m_type);
	}

	DG_INLINE dgSoaFloat AndNot (const dgSoaFloat& A) const
	{
		return  _mm256_andnot_ps (A.m_type, m_type);
	}

	DG_INLINE dgSoaFloat GetMin(const dgSoaFloat& A) const
	{
		return _mm256_min_ps (m_type, A.m_type);
	}

	DG_INLINE dgSoaFloat GetMax(const dgSoaFloat& A) const
	{
		return _mm256_max_ps (m_type, A.m_type);
	}

	DG_INLINE dgFloat32 AddHorizontal() const
	{
		dgSoaFloat ret;
		__m256 tmp0(_mm256_add_ps(m_type, _mm256_permute2f128_ps(m_type, m_type, 1)));
		__m256 tmp1(_mm256_hadd_ps(tmp0, tmp0));
		__m256 tmp2(_mm256_hadd_ps(tmp1, tmp1));
		_mm256_store_ps(ret.m_f, tmp2);
		return ret.m_f[0];
	}

	static DG_INLINE void FlushRegisters()
	{
		_mm256_zeroall ();
	}

	union
	{
		__m256 m_type;
		__m256i m_typeInt;
		dgInt32 m_i[DG_SOA_WORD_GROUP_SIZE];
		dgFloat32 m_f[DG_SOA_WORD_GROUP_SIZE];
	};
} DG_GCC_AVX_ALIGMENT;

#endif

DG_MSC_AVX_ALIGMENT
class dgSoaVector3
{
	public:
	dgSoaFloat m_x;
	dgSoaFloat m_y;
	dgSoaFloat m_z;
} DG_GCC_AVX_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgSoaVector6
{
	public:
	dgSoaVector3 m_linear;
	dgSoaVector3 m_angular;
} DG_GCC_AVX_ALIGMENT;

DG_MSC_AVX_ALIGMENT
class dgSoaJacobianPair
{
	public:
	dgSoaVector6 m_jacobianM0;
	dgSoaVector6 m_jacobianM1;
} DG_GCC_AVX_ALIGMENT;

DG_MSC_AVX_ALIGMENT
class dgSoaMatrixElement
{
	public:
	dgSoaJacobianPair m_Jt;
	dgSoaJacobianPair m_JMinv;

	dgSoaFloat m_force;
	dgSoaFloat m_diagDamp;
	dgSoaFloat m_invJinvMJt;
	dgSoaFloat m_coordenateAccel;
	dgSoaFloat m_normalForceIndex;
	dgSoaFloat m_lowerBoundFrictionCoefficent;
	dgSoaFloat m_upperBoundFrictionCoefficent;
} DG_GCC_AVX_ALIGMENT;




// ******************************************************************************
//
// GPU stuff start here
//
// ******************************************************************************

#define DG_GPU_WORKGROUP_SIZE		256 
#define DG_GPU_BODY_INITIAL_COUNT	4096

class dgVulkanContext
{
	public:
	dgVulkanContext()
	{
		memset(this, 0, sizeof(dgVulkanContext));
	}

	dgInt32 m_totalMemory;
	dgInt32 m_computeQueueIndex;

	VkQueue m_queue;
	VkDevice m_device;
	VkInstance m_instance;
	VkPhysicalDevice m_gpu;
	VkAllocationCallbacks m_allocators;
	VkPhysicalDeviceProperties m_gpu_props;
	VkPhysicalDeviceMemoryProperties m_memory_properties;
	
	VkPipelineCache m_pipeLineCache;

	VkPipeline m_initBodyPipeLine;
	VkShaderModule m_initBodyModule;
	VkDescriptorSetLayout m_initBodyLayout;
	VkPipelineLayout m_initBodyPipelineLayout;
};


template<class T>
class dgArrayGPU
{
	public:
	dgArrayGPU()
		:m_buffer(0)
	{
	}

	~dgArrayGPU()
	{
	}

	void Alloc(dgVulkanContext& context, dgInt32 size)
	{
		dgAssert((size % DG_GPU_WORKGROUP_SIZE) == 0);

		VkBufferCreateInfo bufferInfo;
		Clear(&bufferInfo);
		bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferInfo.size = size * sizeof(T);
		bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		bufferInfo.flags = 0;

		VkResult result = VK_SUCCESS;
		result = vkCreateBuffer(context.m_device, &bufferInfo, &context.m_allocators, &m_buffer);
		dgAssert(result == VK_SUCCESS);

		VkMemoryRequirements memReqs;
		vkGetBufferMemoryRequirements(context.m_device, m_buffer, &memReqs);
		dgAssert(memReqs.size == bufferInfo.size);

		uint32_t memoryTypeIndex;
		//VkFlags memoryProperty = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		VkFlags memoryProperty = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
		VkPhysicalDeviceMemoryProperties& memoryProperties = context.m_memory_properties;
		for (memoryTypeIndex = 0; memoryTypeIndex < memoryProperties.memoryTypeCount; ++memoryTypeIndex) {
			if (((memoryProperties.memoryTypes[memoryTypeIndex].propertyFlags & memoryProperty) == memoryProperty) &&
				((memReqs.memoryTypeBits >> memoryTypeIndex) & 1)) {
				break;
			}
		}
		dgAssert(memoryTypeIndex < memoryProperties.memoryTypeCount);

		VkMemoryAllocateInfo memInfo;
		Clear(&memInfo);
		memInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		memInfo.allocationSize = bufferInfo.size;
		memInfo.memoryTypeIndex = memoryTypeIndex;

		result = vkAllocateMemory(context.m_device, &memInfo, &context.m_allocators, &m_memory);
		dgAssert(result == VK_SUCCESS);

		result = vkBindBufferMemory(context.m_device, m_buffer, m_memory, 0);
		dgAssert(result == VK_SUCCESS);
	}

	void Free(dgVulkanContext& context)
	{
		if (m_buffer) {
			vkFreeMemory(context.m_device, m_memory, &context.m_allocators);
			vkDestroyBuffer(context.m_device, m_buffer, &context.m_allocators);
		}
	}

	T* Lock(dgVulkanContext& context, dgInt32 size)
	{
		void* ptr;
		VkResult result = VK_SUCCESS;
		result = vkMapMemory(context.m_device, m_memory, 0, size * sizeof (T), 0, &ptr);
		dgAssert(result == VK_SUCCESS);
		dgAssert((((dgUnsigned64)ptr) & 0xf) == 0);
		return (T*)ptr;
	}

	void Unlock(dgVulkanContext& context)
	{
		vkUnmapMemory(context.m_device, m_memory);
	}

	VkBuffer m_buffer; 
	VkDeviceMemory m_memory;
};

class dgGpuBody
{
	public:
	dgGpuBody()
		:m_count(0)
		,m_bufferSize(0)
	{
	}

	~dgGpuBody()
	{
		dgAssert(!m_bufferSize);
	}

	void Clear(dgVulkanContext& context)
	{
		if (m_bufferSize) {
			m_damp.Free(context);
			m_veloc.Free(context);
			m_omega.Free(context);
			m_invMass.Free(context);
			m_rotation.Free(context);
			m_position.Free(context);
		}
		m_bufferSize = 0;
	}

	void Alloc(dgVulkanContext& context, dgInt32 size)
	{
		m_damp.Alloc(context, size);
		m_veloc.Alloc(context, size);
		m_omega.Alloc(context, size);
		m_invMass.Alloc(context, size);
		m_rotation.Alloc(context, size);
		m_position.Alloc(context, size);
		m_bufferSize = size;
	}

	void Resize(dgVulkanContext& context, dgInt32 size)
	{
		m_count = size;
		dgInt32 roundSize = DG_GPU_BODY_INITIAL_COUNT * dgInt32((size + DG_GPU_BODY_INITIAL_COUNT - 1) / DG_GPU_BODY_INITIAL_COUNT);
		if (roundSize > m_bufferSize) {
			const dgInt32 bufferSize = m_bufferSize;
			Clear(context);
			while (roundSize < bufferSize) {
				roundSize *= 2;
			}
			Alloc(context, roundSize);
		}
	}

	dgInt32 m_count;
	dgInt32 m_bufferSize;
	dgArrayGPU<dgVector> m_invMass;
	dgArrayGPU<dgVector> m_veloc;
	dgArrayGPU<dgVector> m_omega;
	dgArrayGPU<dgVector> m_damp;
	dgArrayGPU<dgVector> m_rotation;
	dgArrayGPU<dgVector> m_position;
};


DG_MSC_AVX_ALIGMENT
class dgSolver: public dgParallelBodySolver
{
	public:
	dgSolver(dgWorld* const world, dgMemoryAllocator* const allocator);
	~dgSolver();
	void CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep);

	void Cleanup();
	dgVulkanContext& GetContext() {return m_context;}

	private:
	void InitWeights();
	void InitBodyArray();
	void InitSkeletons();
	void CalculateForces();
	void UpdateSkeletons();
	void InitJacobianMatrix();
	void UpdateForceFeedback();
	void CalculateJointsForce();
	void IntegrateBodiesVelocity();
	void UpdateKinematicFeedback();
	void CalculateJointsAcceleration();
	void CalculateBodiesAcceleration();
	
	void InitBodyArray(dgInt32 threadID);
	void InitSkeletons(dgInt32 threadID);
	void UpdateSkeletons(dgInt32 threadID);
	void InitJacobianMatrix(dgInt32 threadID);
	void UpdateForceFeedback(dgInt32 threadID);
	void TransposeMassMatrix(dgInt32 threadID);
	void CalculateJointsForce(dgInt32 threadID);
	void UpdateRowAcceleration(dgInt32 threadID);
	void IntegrateBodiesVelocity(dgInt32 threadID);
	void UpdateKinematicFeedback(dgInt32 threadID);
	void CalculateJointsAcceleration(dgInt32 threadID);
	void CalculateBodiesAcceleration(dgInt32 threadID);

	static void InitBodyArrayKernel(void* const context, void* const, dgInt32 threadID);
	static void InitSkeletonsKernel(void* const context, void* const, dgInt32 threadID);
	static void UpdateSkeletonsKernel(void* const context, void* const, dgInt32 threadID);
	static void InitJacobianMatrixKernel(void* const context, void* const, dgInt32 threadID);
	static void UpdateForceFeedbackKernel(void* const context, void* const, dgInt32 threadID);
	static void TransposeMassMatrixKernel(void* const context, void* const, dgInt32 threadID);
	static void CalculateJointsForceKernel(void* const context, void* const, dgInt32 threadID);
	static void UpdateRowAccelerationKernel(void* const context, void* const, dgInt32 threadID);
	static void IntegrateBodiesVelocityKernel(void* const context, void* const, dgInt32 threadID);
	static void UpdateKinematicFeedbackKernel(void* const context, void* const, dgInt32 threadID);
	static void CalculateBodiesAccelerationKernel(void* const context, void* const, dgInt32 threadID);
	static void CalculateJointsAccelerationKernel(void* const context, void* const, dgInt32 threadID);
	
	static dgInt32 CompareJointInfos(const dgJointInfo* const infoA, const dgJointInfo* const infoB, void* notUsed);
	static dgInt32 CompareBodyJointsPairs(const dgBodyJacobianPair* const pairA, const dgBodyJacobianPair* const pairB, void* notUsed);

	DG_INLINE void SortWorkGroup(dgInt32 base) const;
	DG_INLINE void TransposeRow (dgSoaMatrixElement* const row, const dgJointInfo* const jointInfoArray, dgInt32 index);
	DG_INLINE void BuildJacobianMatrix(dgJointInfo* const jointInfo, dgLeftHandSide* const leftHandSide, dgRightHandSide* const righHandSide, dgJacobian* const internalForces);
	//	DG_INLINE dgFloat32 CalculateJointForce(const dgJointInfo* const jointInfo, dgSoaMatrixElement* const massMatrix, const dgJacobian* const internalForces) const;
	dgFloat32 CalculateJointForce(const dgJointInfo* const jointInfo, dgSoaMatrixElement* const massMatrix, const dgJacobian* const internalForces) const;

	dgSoaFloat m_soaOne;
	dgSoaFloat m_soaZero;
	dgVector m_zero;
	dgVector m_negOne;
	dgVector m_unitRotation;
	dgArray<dgSoaMatrixElement> m_massMatrix;

	dgVulkanContext m_context;
	dgGpuBody m_gpuBodyArray;
} DG_GCC_AVX_ALIGMENT;

#endif
