/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

DG_MSC_AVX_ALIGMENT
class dgSoaFloat
{
	public:
	DG_INLINE dgSoaFloat()
	{
	}

	DG_INLINE dgSoaFloat(const dgSoaFloat& me)
		:m_low(me.m_low)
		,m_high(me.m_high)
	{
	}

	DG_INLINE dgSoaFloat(const dgVector& v)
		:m_low(v)
		,m_high(v)
	{
	}

	DG_INLINE dgSoaFloat(const dgVector& low, const dgVector& high)
		:m_low(low)
		,m_high(high)
	{
	}

	DG_INLINE dgSoaFloat(const dgSoaFloat* const baseAddr, const dgSoaFloat& index)
	{
		const dgInt32* const indirectIndex = (dgInt32*)&index[0];
		const dgFloat32* const src = &(*baseAddr)[0];
		dgFloat32* const dst = &(*this)[0];
		for (dgInt32 i = 0; i < DG_SOA_WORD_GROUP_SIZE; i++) {
			dst[i] = src[indirectIndex[i]];
		}
	}

	DG_INLINE dgFloat32& operator[] (dgInt32 i)
	{
		dgAssert(i >= 0);
		dgAssert(i < DG_SOA_WORD_GROUP_SIZE);
		dgFloat32* const ptr = &m_low[0];
		return ptr[i];
	}

	DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
	{
		dgAssert(i >= 0);
		dgAssert(i < DG_SOA_WORD_GROUP_SIZE);
		const dgFloat32* const ptr = &m_low[0];
		return ptr[i];
	}

	DG_INLINE dgSoaFloat operator+ (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low + A.m_low, m_high + A.m_high);
	}

	DG_INLINE dgSoaFloat operator- (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low - A.m_low, m_high - A.m_high);
	}

	DG_INLINE dgSoaFloat operator* (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low * A.m_low, m_high * A.m_high);
	}

#ifdef _NEWTON_USE_DOUBLE
	DG_INLINE dgSoaFloat MulAdd(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		//return dgSoaFloat(m_low.MulAdd(A.m_low, B.m_low), m_high.MulAdd(A.m_high, B.m_high));
		return dgSoaFloat(dgVector(_mm_fmadd_pd(A.m_low.m_typeLow, B.m_low.m_typeLow, m_low.m_typeLow), _mm_fmadd_pd(A.m_low.m_typeHigh, B.m_low.m_typeHigh, m_low.m_typeHigh)),
			dgVector(_mm_fmadd_pd(A.m_high.m_typeLow, B.m_high.m_typeLow, m_high.m_typeLow), _mm_fmadd_pd(A.m_high.m_typeHigh, B.m_high.m_typeHigh, m_high.m_typeHigh)));
	}

	DG_INLINE dgSoaFloat MulSub(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		//return dgSoaFloat(m_low.MulSub(A.m_low, B.m_low), m_high.MulSub(A.m_high, B.m_high));
		return dgSoaFloat(dgVector (_mm_fnmadd_pd(A.m_low.m_typeLow, B.m_low.m_typeLow, m_low.m_typeLow), _mm_fnmadd_pd(A.m_low.m_typeHigh, B.m_low.m_typeHigh, m_low.m_typeHigh)),
			dgVector (_mm_fnmadd_pd(A.m_high.m_typeLow, B.m_high.m_typeLow, m_high.m_typeLow), _mm_fnmadd_pd(A.m_high.m_typeHigh, B.m_high.m_typeHigh, m_high.m_typeHigh)));
	}
#else
	DG_INLINE dgSoaFloat MulAdd(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		return dgSoaFloat(_mm_fmadd_ps (A.m_low.m_type, B.m_low.m_type, m_low.m_type), _mm_fmadd_ps(A.m_high.m_type, B.m_high.m_type, m_high.m_type));
	}

	DG_INLINE dgSoaFloat MulSub(const dgSoaFloat& A, const dgSoaFloat& B) const
	{
		return dgSoaFloat(_mm_fnmadd_ps(A.m_low.m_type, B.m_low.m_type, m_low.m_type), _mm_fnmadd_ps(A.m_high.m_type, B.m_high.m_type, m_high.m_type));
	}
#endif

	DG_INLINE dgSoaFloat operator> (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low > A.m_low, m_high > A.m_high);
	}

	DG_INLINE dgSoaFloat operator< (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low < A.m_low, m_high < A.m_high);
	}

	DG_INLINE dgSoaFloat operator| (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low | A.m_low, m_high | A.m_high);
	}

	DG_INLINE dgSoaFloat operator& (const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low & A.m_low, m_high & A.m_high);
	}

	DG_INLINE dgSoaFloat GetMin(const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low.GetMin(A.m_low), m_high.GetMin(A.m_high));
	}

	DG_INLINE dgSoaFloat GetMax(const dgSoaFloat& A) const
	{
		return dgSoaFloat(m_low.GetMax(A.m_low), m_high.GetMax(A.m_high));
	}

	DG_INLINE dgFloat32 AddHorizontal() const
	{
		return (m_low + m_high).AddHorizontal().GetScalar();
	}

	DG_INLINE dgFloat32 GetMax() const
	{
		return (m_low.GetMax(m_high)).GetMax();
	}

	static DG_INLINE void FlushRegisters()
	{
	}

	dgVector m_low;
	dgVector m_high;
}DG_GCC_AVX_ALIGMENT;


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

DG_MSC_AVX_ALIGMENT
class dgSolver: public dgParallelBodySolver
{
	public:
	dgSolver(dgWorld* const world, dgMemoryAllocator* const allocator);
	~dgSolver();
	void CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep);

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
	dgArray<dgSoaMatrixElement> m_massMatrix;
} DG_GCC_AVX_ALIGMENT;


#endif

