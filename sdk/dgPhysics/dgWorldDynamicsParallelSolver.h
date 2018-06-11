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


#define DG_WORK_GROUP_SIZE	(2 * sizeof (dgVector)/sizeof (dgFloat32)) 
/*
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
		dgAssert(i < DG_WORK_GROUP_SIZE);
		dgAssert(i >= 0);
		return m_f[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i < DG_WORK_GROUP_SIZE);
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

	DG_INLINE dgWorkGroupFloat Reciproc () const
	{
		return dgWorkGroupFloat(m_low.Reciproc(), m_high.Reciproc());
	}

	DG_INLINE dgWorkGroupFloat operator> (const dgWorkGroupFloat& A) const
	{
		return dgWorkGroupFloat(m_low > A.m_low, m_high > A.m_high);
	}

	DG_INLINE dgWorkGroupFloat Select (const dgWorkGroupFloat& A, const dgWorkGroupFloat& B) const
	{
		return dgWorkGroupFloat((A.m_low & m_low) | B.m_low.AndNot(m_low), (A.m_high & m_high) | B.m_high.AndNot(m_high));
	}

	DG_INLINE dgWorkGroupFloat MulAdd (const dgWorkGroupFloat& B, const dgWorkGroupFloat& C) const
	{
		return *this + B * C;
	}

	DG_INLINE void Store(dgFloat32* const ptr) const
	{
		m_low.Store(ptr);
		m_high.Store(&ptr[4]);
	}

	DG_INLINE static void Transpose4x8(dgWorkGroupFloat& src0, dgWorkGroupFloat& src1, dgWorkGroupFloat& src2, dgWorkGroupFloat& src3)
	{
		dgVector::Transpose4x4(src0.m_low, src1.m_low, src2.m_low, src3.m_low, src0.m_low, src1.m_low, src2.m_low, src3.m_low);
		dgVector::Transpose4x4(src0.m_high, src1.m_high, src2.m_high, src3.m_high, src0.m_high, src1.m_high, src2.m_high, src3.m_high);
	}

	union {
		dgFloat32 m_f[DG_WORK_GROUP_SIZE];
		struct {
			dgVector m_low;
			dgVector m_high;
		};
	};

	static dgWorkGroupFloat m_one;
	static dgWorkGroupFloat m_zero;
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

	DG_INLINE dgWorkGroupVector3(const dgWorkGroupFloat& x, const dgWorkGroupFloat& y, const dgWorkGroupFloat& z)
		:m_x(x)
		,m_y(y)
		,m_z(z)
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

	DG_INLINE dgWorkGroupVector3 Scale(const dgWorkGroupFloat& a) const
	{
		return dgWorkGroupVector3(m_x * a, m_y * a, m_z * a);
	}

	DG_INLINE dgWorkGroupVector3 operator* (const dgWorkGroupVector3& a) const
	{
		return dgWorkGroupVector3(m_x * a.m_x, m_y * a.m_y, m_z * a.m_z);
	}


	DG_INLINE void Store(dgFloat32* const ptr) const
	{
		m_x.Store(&ptr[0 * DG_WORK_GROUP_SIZE]);
		m_y.Store(&ptr[1 * DG_WORK_GROUP_SIZE]);
		m_z.Store(&ptr[2 * DG_WORK_GROUP_SIZE]);
	}

	dgWorkGroupFloat m_x;
	dgWorkGroupFloat m_y;
	dgWorkGroupFloat m_z;
} DG_GCC_VECTOR_ALIGMENT;


DG_MSC_AVX_ALIGMENT
class dgWorkGroupVector6
{
	public:
	dgWorkGroupVector3 m_linear;
	dgWorkGroupVector3 m_angular;
} DG_GCC_AVX_ALIGMENT;

DG_MSC_VECTOR_ALIGMENT
class dgWorkGroupMatrix3x3
{
	public:
	DG_INLINE dgWorkGroupMatrix3x3()
	{
	}

	DG_INLINE dgWorkGroupMatrix3x3(const dgWorkGroupVector3& right, const dgWorkGroupVector3& up, const dgWorkGroupVector3& front)
		:m_right(right)
		,m_up(up)
		,m_front(front)
	{
	}

	DG_INLINE dgWorkGroupMatrix3x3(const dgMatrix& matrix0, const dgMatrix& matrix1, const dgMatrix& matrix2, const dgMatrix& matrix3, const dgMatrix& matrix4, const dgMatrix& matrix5, const dgMatrix& matrix6, const dgMatrix& matrix7)
		:m_right(matrix0[0], matrix1[0], matrix2[0], matrix3[0], matrix4[0], matrix5[0], matrix6[0], matrix7[0])
		,m_up(matrix0[1], matrix1[1], matrix2[1], matrix3[1], matrix4[1], matrix5[1], matrix6[1], matrix7[1])
		,m_front(matrix0[2], matrix1[2], matrix2[2], matrix3[2], matrix4[2], matrix5[2], matrix6[2], matrix7[2])
	{
	}

	DG_INLINE dgWorkGroupMatrix3x3 operator* (const dgWorkGroupMatrix3x3& a) const
	{
		return dgWorkGroupMatrix3x3(a.RotateVector(m_front), a.RotateVector(m_up), a.RotateVector(m_right));
	}


	DG_INLINE dgWorkGroupVector3 RotateVector(const dgWorkGroupVector3& a) const
	{
		dgWorkGroupFloat x(a.m_x * m_front.m_x + a.m_y * m_up.m_x + a.m_z * m_right.m_x);
		dgWorkGroupFloat y(a.m_x * m_front.m_y + a.m_y * m_up.m_y + a.m_z * m_right.m_y);
		dgWorkGroupFloat z(a.m_x * m_front.m_z + a.m_y * m_up.m_z + a.m_z * m_right.m_z);
		return dgWorkGroupVector3(x, y, z);
	}

	DG_INLINE dgWorkGroupVector3 UnrotateVector(const dgWorkGroupVector3& a) const
	{
		dgWorkGroupFloat x(a.m_x * m_front.m_x + a.m_y * m_front.m_y + a.m_z * m_front.m_z);
		dgWorkGroupFloat y(a.m_x * m_up.m_x + a.m_y * m_up.m_y + a.m_z * m_up.m_z);
		dgWorkGroupFloat z(a.m_x * m_right.m_x + a.m_y * m_right.m_y + a.m_z * m_right.m_z);
		return dgWorkGroupVector3(x, y, z);
	}

	DG_INLINE dgWorkGroupMatrix3x3 Transposed() const
	{
		return dgWorkGroupMatrix3x3(
			dgWorkGroupVector3(m_front.m_x, m_up.m_x, m_right.m_x),
			dgWorkGroupVector3(m_front.m_y, m_up.m_y, m_right.m_y),
			dgWorkGroupVector3(m_front.m_z, m_up.m_z, m_right.m_z));
	}


	dgWorkGroupVector3 m_front;
	dgWorkGroupVector3 m_up;
	dgWorkGroupVector3 m_right;
} DG_GCC_VECTOR_ALIGMENT;



template<class T>
class dgParallelVector: public dgArray<T>
{
	public:
	dgParallelVector(dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes = DG_MEMORY_GRANULARITY)
		:dgArray<T>(allocator, aligmentInBytes)
	{
	}

	DG_INLINE T& operator[] (dgInt32 i)
	{
		return m_array[i];
	}

	DG_INLINE const T& operator[] (dgInt32 i) const
	{
		return m_array[i];
	}
};
*/
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
	void Reserve (dgInt32 bodyCount, dgInt32 jointCount);
	void InitWeights(dgInt32 threadID);
	void InitBodyArray(dgInt32 threadID);
	void InitJacobianMatrix(dgInt32 threadID);
	void CalculateBodyForce(dgInt32 threadID);
	void CalculateJointsForce(dgInt32 threadID);
	void CalculateJointsAcceleration(dgInt32 threadID);
	
	void InitWeights();
	void InitBodyArray();
	void InitJacobianMatrix();
	void CalculateForces();
	void CalculateBodyForce();
	void CalculateJointsForce();
	void CalculateJointsAcceleration();
	void BuildJacobianMatrix(dgJointInfo* const jointInfo, dgLeftHandSide* const leftHandSide, dgRightHandSide* const righHandSide, dgJacobian* const internalForces);
	dgFloat32 CalculateJointForce(const dgJointInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, const dgJacobian* const internalForces) const;

	static void InitWeightKernel(void* const context, void* const, dgInt32 threadID);
	static void InitBodyArrayKernel(void* const context, void* const, dgInt32 threadID);
	static void InitJacobianMatrixKernel(void* const context, void* const, dgInt32 threadID);
	static void CalculateBodyForceKernel(void* const context, void* const, dgInt32 threadID);
	static void CalculateJointsForceKernel(void* const context, void* const, dgInt32 threadID);
	static void CalculateJointsAccelerationKernel(void* const context, void* const, dgInt32 threadID);
	

//	dgParallelVector<dgWorkGroupFloat> m_weigh;
//	dgParallelVector<dgWorkGroupFloat> m_invWeigh;
//	dgParallelVector<dgWorkGroupVector6> m_veloc;
//	dgParallelVector<dgWorkGroupVector6> m_veloc0;
//	dgParallelVector<dgWorkGroupFloat> m_invMass;
//	dgParallelVector<dgWorkGroupVector3> m_localInvInertia;
//	dgParallelVector<dgWorkGroupMatrix3x3> m_invInertia;

//	dgParallelVector<dgWorkGroupMatrix3x3> m_rotation;
//	dgParallelVector<dgWorkGroupVector3> m_angularDamp;
//	dgParallelVector<dgWorkGroupFloat> m_linearDamp;
//	dgParallelVector<dgWorkGroupVector6> m_internalForces;

	dgWorld* m_world;
	dgBodyCluster* m_cluster;
	dgBodyInfo* m_bodyArray;
	dgJointInfo* m_jointArray;
	dgFloat32* m_weight;
	dgFloat32* m_invWeight;
	dgFloat32 m_timestep;
	dgFloat32 m_invTimestep;
	dgFloat32 m_invStepRK;
	dgFloat32 m_timestepRK;
	dgFloat32 m_invTimestepRK;
	dgFloat32 m_firstPassCoef;
	dgFloat32 m_accelNorm[DG_MAX_THREADS_HIVE_COUNT];
//	dgInt32 m_bodyCount;
	dgInt32 m_jointCount;
	dgInt32 m_atomicIndex;
	dgInt32 m_jacobianMatrixRowAtomicIndex;
	dgInt32 m_solverPasses;
	dgInt32 m_threadCounts;

	friend class dgWorldDynamicUpdate;
};


#endif

