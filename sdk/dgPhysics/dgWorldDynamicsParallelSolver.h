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

#ifndef _DG_PARALLEL_SOLVER_H_
#define _DG_PARALLEL_SOLVER_H_


#include "dgPhysicsStdafx.h"

class dgBodyInfo;
class dgJointInfo;
class dgBodyCluster;

#define DG_WORK_GROUP_BITS	3
#define DG_WORK_GROUP_SIZE	(1<<DG_WORK_GROUP_BITS) 


DG_MSC_VECTOR_ALIGMENT
class dgWorkGroupFloat
{
	public:
	DG_INLINE dgWorkGroupFloat()
	{
	}

	DG_INLINE dgWorkGroupFloat(const dgWorkGroupFloat& me)
		:m_low(me.m_low)
		,m_high(me.m_high)
	{
	}

	DG_INLINE dgWorkGroupFloat(const dgVector& low, const dgVector& high)
		:m_low(low)
		,m_high(high)
	{
	}

	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert(i < 8);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i < 8);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE dgWorkGroupFloat operator+ (const dgWorkGroupFloat& A) const
	{
		return dgWorkGroupFloat(m_low + A.m_low, m_high + A.m_high);
	}

	DG_INLINE dgWorkGroupFloat operator* (const dgWorkGroupFloat& A) const
	{
		return dgWorkGroupFloat(m_low * A.m_low, m_high * A.m_high);
	}

	DG_INLINE dgWorkGroupFloat MulAdd (const dgWorkGroupFloat& B, const dgWorkGroupFloat& C) const
	{
		return *this + B * C;
	}


	DG_INLINE static void Transpose4x8(dgWorkGroupFloat& src0, dgWorkGroupFloat& src1, dgWorkGroupFloat& src2, dgWorkGroupFloat& src3)
	{
		/*
		__m256 tmp0(_mm256_unpacklo_ps(src0.m_type, src1.m_type));
		__m256 tmp1(_mm256_unpacklo_ps(src2.m_type, src3.m_type));
		__m256 tmp2(_mm256_unpackhi_ps(src0.m_type, src1.m_type));
		__m256 tmp3(_mm256_unpackhi_ps(src2.m_type, src3.m_type));

		src0 = _mm256_shuffle_ps(tmp0, tmp1, PERMUTE_MASK(1, 0, 1, 0));
		src1 = _mm256_shuffle_ps(tmp0, tmp1, PERMUTE_MASK(3, 2, 3, 2));
		src2 = _mm256_shuffle_ps(tmp2, tmp3, PERMUTE_MASK(1, 0, 1, 0));
		src3 = _mm256_shuffle_ps(tmp2, tmp3, PERMUTE_MASK(3, 2, 3, 2));
		*/
	}



	union {
		dgFloat32 m_f[8];
		struct {
			dgVector m_low;
			dgVector m_high;
		};
	};
} DG_GCC_VECTOR_ALIGMENT;

DG_MSC_VECTOR_ALIGMENT
class dgWorkGroupVector3
{
	public:
	DG_INLINE dgWorkGroupVector3()
		:m_x()
		,m_y()
		,m_z()
	{
	}

	DG_INLINE dgWorkGroupVector3(const dgVector& v0, const dgVector& v1, const dgVector& v2, const dgVector& v3, const dgVector& v4, const dgVector& v5, const dgVector& v6, const dgVector& v7)
		:m_x()
		,m_y()
		,m_z()
	{
		dgWorkGroupFloat r0(v0, v4);
		dgWorkGroupFloat r1(v1, v5);
		dgWorkGroupFloat r2(v2, v6);
		dgWorkGroupFloat r3(v3, v7);
		dgWorkGroupFloat::Transpose4x8(r0, r1, r2, r3);
		m_x = r0;
		m_y = r1;
		m_z = r2;
	}


	dgWorkGroupFloat m_x;
	dgWorkGroupFloat m_y;
	dgWorkGroupFloat m_z;
} DG_GCC_VECTOR_ALIGMENT;


template<class T>
class dgParallelVector: public dgArray<T>
{
	public:
	dgParallelVector(dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes = DG_MEMORY_GRANULARITY)
		:dgArray<T>(allocator, aligmentInBytes)
	{
	}
};

class dgParallelSolverSyncData
{
	public:
	dgParallelSolverSyncData()
	{
		memset (this, 0, sizeof (dgParallelSolverSyncData));
	}

	dgFloat32 m_accelNorm[DG_MAX_THREADS_HIVE_COUNT];

	dgFloat32 m_timestep;
	dgFloat32 m_invTimestep;
	dgFloat32 m_invStepRK;
	dgFloat32 m_timestepRK;
	dgFloat32 m_invTimestepRK;
	dgFloat32 m_firstPassCoef;

	dgInt32 m_passes;
	dgInt32 m_rkSubSteps;
	dgInt32 m_bodyCount;
	dgInt32 m_jointCount;
	dgInt32 m_rowCount;
	dgInt32 m_atomicIndex;
	dgInt32 m_clusterCount;
	dgInt32 m_jacobianMatrixRowAtomicIndex;

	dgInt32* m_bodyLocks;  
	dgBodyInfo* m_bodyArray;
	dgJointInfo* m_jointsArray;
	dgBodyCluster* m_cluster;
	dgFloat32* m_weight;
	const dgBodyCluster* m_clusterArray;

	dgInt32 m_hasJointFeeback[DG_MAX_THREADS_HIVE_COUNT];
};

class dgParallelBodySolver
{
	public:
	dgParallelBodySolver(dgMemoryAllocator* const allocator);
	~dgParallelBodySolver();
	void CalculateJointForces(dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep);

	private:
	void Reserve (dgInt32 count);
	void InitWeights();
	void InitInvWeights();
	void InityBodyArray();
	
	static void InitWeightKernel(void* const context, void* const, dgInt32 threadID);
	static void InitInvWeightKernel(void* const context, void* const, dgInt32 threadID);
	static void InitBodyArrayKernel(void* const context, void* const, dgInt32 threadID);


	void GetVeloc(dgInt32 index);
	void GetOmega(dgInt32 index);

	dgParallelVector<dgWorkGroupFloat> m_weigh;
	dgParallelVector<dgWorkGroupFloat> m_invWeigh;
	dgParallelVector<dgWorkGroupVector3> m_veloc;
	dgParallelVector<dgWorkGroupVector3> m_omega;

	dgWorld* m_world;
	dgBodyCluster* m_cluster;
	dgBodyInfo* m_bodyArray;
	dgJointInfo* m_jointArray;
	dgInt32 m_count;
	dgInt32 m_atomicIndex;

	friend class dgWorldDynamicUpdate;
};


#endif

