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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonContainer.h"

//#include "dgPhysicsStdafx.h"
//#include "dgBody.h"
//#include "dgWorld.h"
//#include "ndConstraint.h"
//#include "ndBodyKinematic.h"
//#include "ndSkeletonContainer.h"
//#include "dgWorldDynamicUpdate.h"
#include "ndJointBilateralConstraint.h"

dInt64 ndSkeletonContainer::ndNode::m_ordinalInit = 0x050403020100ll;

#if 0
class ndSkeletonContainer::ndNodePair
{
	public:
	dInt32 m_m0;
	dInt32 m_m1;
};

DG_MSC_VECTOR_ALIGNMENT
class ndSkeletonContainer::dgForcePair
{
	public:
	dgSpatialVector m_joint;
	dgSpatialVector m_body;
} DG_GCC_VECTOR_ALIGNMENT;

DG_MSC_VECTOR_ALIGNMENT 
class ndSkeletonContainer::dgMatriData
{
	public:
	dgSpatialMatrix m_jt;
	dgSpatialMatrix m_invMass;
} DG_GCC_VECTOR_ALIGNMENT;

DG_MSC_VECTOR_ALIGNMENT 
class ndSkeletonContainer::dgBodyJointMatrixDataPair
{
	public:
	dgMatriData m_body;
	dgMatriData m_joint;
} DG_GCC_VECTOR_ALIGNMENT;
#endif


ndSkeletonContainer::ndNode::ndNode()
	:m_body(nullptr)
	,m_joint(nullptr)
	,m_parent(nullptr)
	,m_child(nullptr)
	,m_sibling(nullptr)
	,m_ordinals(0)
	,m_index(0)
	,m_dof(0)
	,m_swapJacobianBodiesIndex(0)
{
}

ndSkeletonContainer::ndNode::~ndNode()
{
	m_body->SetSkeleton(nullptr);
}


#if 0
class ndSkeletonContainer::ndNode
{
	public:
	DG_INLINE void CalculateInertiaMatrix(dgSpatialMatrix* const bodyMassArray) const
	{
		dgSpatialMatrix& bodyMass = bodyMassArray[m_index];

		bodyMass = dgSpatialMatrix(dFloat32(0.0f));
		if (m_body->GetInvMass().m_w != dFloat32(0.0f)) {
			const dFloat32 mass = m_body->GetMass().m_w;
			dgMatrix inertia(m_body->CalculateInertiaMatrix());
			for (dInt32 i = 0; i < 3; i++) {
				bodyMass[i][i] = mass;
				for (dInt32 j = 0; j < 3; j++) {
					bodyMass[i + 3][j + 3] = inertia[i][j];
				}
			}
		}
	}

	DG_INLINE void GetJacobians(const dgJointInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgSpatialMatrix* const jointMassArray)
	{
		dAssert(m_parent);
		dAssert(jointInfo->m_joint == m_joint);

		dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		dgSpatialMatrix& jointMass = jointMassArray[m_index];

		const dInt32 start = jointInfo->m_pairStart;
		const dgSpatialVector zero (dgSpatialVector::m_zero);
		if (!m_swapJacobianBodiesIndex) {
			for (dInt32 i = 0; i < m_dof; i++) {
				const dInt32 k = m_sourceJacobianIndex[i];
				const dgRightHandSide* const rhs = &rightHandSide[start + k];
				const dgLeftHandSide* const row = &leftHandSide[start + k];
				jointMass[i] = zero;
				jointMass[i][i] = -rhs->m_diagDamp;
				bodyJt[i] = dgSpatialVector (row->m_Jt.m_jacobianM0.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM0.m_angular * dgVector::m_negOne);
				jointJ[i] = dgSpatialVector (row->m_Jt.m_jacobianM1.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM1.m_angular * dgVector::m_negOne);
			}
		} else {
			for (dInt32 i = 0; i < m_dof; i++) {
				const dInt32 k = m_sourceJacobianIndex[i];
				const dgRightHandSide* const rhs = &rightHandSide[start + k];
				const dgLeftHandSide* const row = &leftHandSide[start + k];
				jointMass[i] = zero;
				jointMass[i][i] = -rhs->m_diagDamp;
				bodyJt[i] = dgSpatialVector(row->m_Jt.m_jacobianM1.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM1.m_angular * dgVector::m_negOne);
				jointJ[i] = dgSpatialVector(row->m_Jt.m_jacobianM0.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM0.m_angular * dgVector::m_negOne);
			}
		}
	}

	dInt32 Factorize(const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgSpatialMatrix* const bodyMassArray, dgSpatialMatrix* const jointMassArray)
	{
		CalculateInertiaMatrix(bodyMassArray);

		m_ordinals = m_ordinalInit;
		dInt32 boundedDof = 0;
		if (m_joint) {
			dAssert (m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dAssert(jointInfo->m_joint == m_joint);

			m_dof = 0;
			dInt32 count = jointInfo->m_pairCount;
			const dInt32 first = jointInfo->m_pairStart;
			for (dInt32 i = 0; i < count; i++) {
				dInt32 k = m_sourceJacobianIndex[i];
				const dgRightHandSide* const rhs = &rightHandSide[k + first];
				if ((rhs->m_lowerBoundFrictionCoefficent <= dFloat32 (-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32 (DG_LCP_MAX_VALUE))) {
					m_dof ++;
				} else {
					dgSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[count - 1]);
					i--;
					count--;
				}
			}
			dAssert (m_dof > 0);
			dAssert (m_dof <= 6);
			boundedDof += jointInfo->m_pairCount - count;
			GetJacobians(jointInfo, leftHandSide, rightHandSide, jointMassArray);
		}

		dgSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
		const dgSpatialMatrix& bodyMass = bodyMassArray[m_index];
		if (m_body->GetInvMass().m_w != dFloat32(0.0f)) {
			for (ndNode* child = m_child; child; child = child->m_sibling) {
				CalculateBodyDiagonal(child, bodyMassArray, jointMassArray);
			}
			bodyInvMass = bodyMass.Inverse(6);
		} else {
			bodyInvMass = dgSpatialMatrix(dFloat32(0.0f));
		}

		if (m_joint) {
			dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
			dAssert(m_parent);
			for (dInt32 i = 0; i < m_dof; i++) {
				bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
			}
			CalculateJointDiagonal(bodyMassArray, jointMassArray);
			CalculateJacobianBlock();
		}
		return boundedDof;
	}

	DG_INLINE void CalculateBodyDiagonal(ndNode* const child, dgSpatialMatrix* const bodyMassArray, const dgSpatialMatrix* const jointMassArray)
	{
		dAssert(child->m_joint);
		
		dgSpatialMatrix copy (dFloat32(0.0f));
		const dInt32 dof = child->m_dof;
		const dgSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
		const dgSpatialMatrix& childDiagonal = jointMassArray[child->m_index];
		for (dInt32 i = 0; i < dof ; i++) {
			const dgSpatialVector& jacobian = jacobianMatrix[i];
			for (dInt32 j = 0; j < dof ; j++) {
				dAssert(dgAreEqual (dgFloat64(childDiagonal[i][j]), dgFloat64(childDiagonal[j][i]), dgFloat64(1.0e-5f)));
				dgFloat64 val = childDiagonal[i][j];
				copy[j] = copy[j] + jacobian.Scale(val);
			}
		}

		dgSpatialMatrix& bodyMass = bodyMassArray[m_index];
		for (dInt32 i = 0; i < dof; i++) {
			const dgSpatialVector& Jacobian = copy[i];
			const dgSpatialVector& JacobianTranspose = jacobianMatrix[i];
			for (dInt32 j = 0; j < 6; j++) {
				dgFloat64 val = -Jacobian[j];
				bodyMass[j] = bodyMass[j] + JacobianTranspose.Scale(val);
			}
		}
	}

	DG_INLINE void CalculateJointDiagonal (const dgSpatialMatrix* const bodyMassArray, dgSpatialMatrix* const jointMassArray)
	{
		const dgSpatialMatrix& bodyMass = bodyMassArray[m_index];
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;

		dgSpatialMatrix tmp;
		for (dInt32 i = 0; i < m_dof; i++) {
			tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
		}

		dgSpatialMatrix& jointMass = jointMassArray[m_index];
		for (dInt32 i = 0; i < m_dof; i++) {
			dgFloat64 a = bodyJt[i].DotProduct(tmp[i]);
			jointMass[i][i] -= a;
			for (dInt32 j = i + 1; j < m_dof; j++) {
				a = - bodyJt[i].DotProduct(tmp[j]);
				jointMass[i][j] = a;
				jointMass[j][i] = a;
			}
		}

		dgSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
		jointInvMass = jointMass.Inverse(m_dof);
	}

	DG_INLINE void CalculateJacobianBlock()
	{
		dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;

		dgSpatialMatrix copy;
		const dgSpatialVector zero (dgSpatialVector::m_zero);
		for (dInt32 i = 0; i < m_dof; i++) {
			copy[i] = jointJ[i];
			jointJ[i] = zero;
		}

		const dgSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
		for (dInt32 i = 0; i < m_dof; i++) {
			const dgSpatialVector& jacobian = copy[i];
			const dgSpatialVector& invDiagonalRow = jointInvMass[i];
			for (dInt32 j = 0; j < m_dof; j++) {
				dgFloat64 val = invDiagonalRow[j];
				jointJ[j] = jointJ[j] + jacobian.Scale(val);
			}
		}
	}

	DG_INLINE void JointJacobianTimeMassForward (dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
		}
	}

	DG_INLINE void BodyJacobianTimeMassForward(const dgForcePair& force, dgForcePair& parentForce) const 
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		for (dInt32 i = 0; i < m_dof; i++) {
			parentForce.m_body = parentForce.m_body + jointJ[i].Scale(-force.m_joint[i]);
		}
	}

	DG_INLINE void JointJacobianTimeSolutionBackward(dgForcePair& force, const dgForcePair& parentForce)
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		const dgSpatialVector& f = parentForce.m_body;
		for (dInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= f.DotProduct(jointJ[i]);
		}
	}

	DG_INLINE void BodyJacobianTimeSolutionBackward(dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dInt32 i = 0; i < m_dof; i++) {
			force.m_body = force.m_body + bodyJt[i].Scale(-force.m_joint[i]);
		}
	}

	DG_INLINE void BodyDiagInvTimeSolution(dgForcePair& force)
	{
		const dgSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
		force.m_body = bodyInvMass.VectorTimeMatrix(force.m_body);
	}

	DG_INLINE void JointDiagInvTimeSolution(dgForcePair& force)
	{
		const dgSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
		force.m_joint = jointInvMass.VectorTimeMatrix(force.m_joint, m_dof);
	}

	DG_INLINE dInt32 GetAuxiliaryRows(const dgJointInfo* const jointInfoArray, const dgRightHandSide* const rightHandSide) const
	{
		dInt32 rowCount = 0;
		if (m_joint) {
			dAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dAssert(jointInfo->m_joint == m_joint);
			dInt32 count = jointInfo->m_pairCount;
			const dInt32 first = jointInfo->m_pairStart;
			for (dInt32 i = 0; i < count; i++) {
				const dgRightHandSide* const rhs = &rightHandSide[i + first];
				if (!((rhs->m_lowerBoundFrictionCoefficent <= dFloat32 (-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32 (DG_LCP_MAX_VALUE)))) {
					rowCount++;
				}
			}
		}
		return rowCount;
	}
	
	dgBodyJointMatrixDataPair m_data;
	ndBodyKinematic* m_body;
	ndJointBilateralConstraint* m_joint;
	ndNode* m_parent;
	ndNode* m_child;
	ndNode* m_sibling;
	union
	{
		dInt64 m_ordinals;
		dgInt8 m_sourceJacobianIndex[8];
	};
	dInt16 m_index;
	dgInt8 m_dof;
	dgInt8 m_swapJacobianBodiesIndex;
	static dInt64 m_ordinalInit;
};


ndSkeletonContainer::ndNode* ndSkeletonContainer::GetParent (ndNode* const node) const
{
	return node->m_parent;
}

ndBodyKinematic* ndSkeletonContainer::GetBody(ndSkeletonContainer::ndNode* const node) const
{
	return node->m_body;
}

ndJointBilateralConstraint* ndSkeletonContainer::GetJoint(ndSkeletonContainer::ndNode* const node) const
{
	return node->m_joint;
}

ndSkeletonContainer::ndNode* ndSkeletonContainer::GetFirstChild(ndSkeletonContainer::ndNode* const parent) const
{
	return parent->m_child;
}

ndSkeletonContainer::ndNode* ndSkeletonContainer::GetNextSiblingChild(ndSkeletonContainer::ndNode* const sibling) const
{
	return sibling->m_sibling;
}

dgWorld* ndSkeletonContainer::GetWorld() const
{
	return m_world;
}

ndSkeletonContainer::ndNode* ndSkeletonContainer::FindNode(ndBodyKinematic* const body) const
{
	dInt32 stack = 1;
	ndNode* stackPool[1024];

	stackPool[0] = m_skeleton;
	while (stack) {
		stack--;
		ndNode* const node = stackPool[stack];
		if (node->m_body == body) {
			return node;
		}

		for (ndNode* ptr = node->m_child; ptr; ptr = ptr->m_sibling) {
			stackPool[stack] = ptr;
			stack++;
			dAssert(stack < dInt32(sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return nullptr;
}


void ndSkeletonContainer::RemoveLoopJoint(ndJointBilateralConstraint* const joint)
{
	for (dInt32 i = 0; i < m_loopCount; i++) {
		if (m_loopingJoints[i] == joint) {
			joint->m_isInSkeleton = false;
			m_loopCount--;
			m_loopingJoints[i] = m_loopingJoints[m_loopCount];
			break;
		}
	}
}


void ndSkeletonContainer::FactorizeMatrix(dInt32 size, dInt32 stride, dFloat32* const matrix, dFloat32* const diagDamp) const
{
	DG_TRACKTIME();
	bool isPsdMatrix = false;
	dFloat32* const backupMatrix = dgAlloca(dFloat32, size * stride);
	do {
		{
			dInt32 srcLine = 0;
			dInt32 dstLine = 0;
			for (dInt32 i = 0; i < size; i++) {
				memcpy(&backupMatrix[dstLine], &matrix[srcLine], size * sizeof(dFloat32));
				srcLine += size;
				dstLine += stride;
			}
		}
		isPsdMatrix = dgCholeskyFactorization(size, stride, matrix);
		if (!isPsdMatrix) {
			dInt32 srcLine = 0;
			dInt32 dstLine = 0;
			for (dInt32 i = 0; i < size; i++) {
				memcpy(&matrix[dstLine], &backupMatrix[srcLine], size * sizeof(dFloat32));
				diagDamp[i] *= dFloat32(4.0f);
				matrix[dstLine + i] += diagDamp[i];
				dstLine += size;
				srcLine += stride;
			}
		}
	} while (!isPsdMatrix);
}
void ndSkeletonContainer::CalculateLoopMassMatrixCoefficients(dFloat32* const diagDamp)
{
	DG_TRACKTIME();
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dInt32 index = 0; index < m_auxiliaryRowCount; index ++) {
		const dInt32 ii = m_matrixRowsIndex[primaryCount + index];
		const dgLeftHandSide* const row_i = &m_leftHandSide[ii];
		const dgRightHandSide* const rhs_i = &m_rightHandSide[ii];
		const dgJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
		const dgJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);
		const dgVector element(
			JMinvM0.m_linear * row_i->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_i->m_Jt.m_jacobianM0.m_angular +
			JMinvM1.m_linear * row_i->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_i->m_Jt.m_jacobianM1.m_angular);
		
		// I know I am doubling the matrix regularizer, but this makes the solution more robust.
		dFloat32* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * index];
		dFloat32 diagonal = element.AddHorizontal().GetScalar() + rhs_i->m_diagDamp;
		matrixRow11[index] = diagonal + rhs_i->m_diagDamp;
		//diagDamp[index] = matrixRow11[index] * (DG_PSD_DAMP_TOL * dFloat32(4.0f));
		diagDamp[index] = matrixRow11[index] * dFloat32(4.0e-3f);

		const dInt32 m0_i = m_pairs[primaryCount + index].m_m0;
		const dInt32 m1_i = m_pairs[primaryCount + index].m_m1;
		for (dInt32 j = index + 1; j < m_auxiliaryRowCount; j++) {
			const dInt32 jj = m_matrixRowsIndex[primaryCount + j];
			const dgLeftHandSide* const row_j = &m_leftHandSide[jj];

			const dInt32 k = primaryCount + j;
			dgVector acc(dgVector::m_zero);
			const dInt32 m0_j = m_pairs[k].m_m0;
			const dInt32 m1_j = m_pairs[k].m_m1;
			bool hasEffect = false;
			if (m0_i == m0_j) {
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			} else if (m0_i == m1_j) {
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}

			if (m1_i == m1_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			} else if (m1_i == m0_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}

			if (hasEffect) {
				acc = acc.AddHorizontal();
				dFloat32 offDiagValue = acc.GetScalar();
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + index] = offDiagValue;
			}
		}

		dFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * index];
		for (dInt32 j = 0; j < primaryCount; j++) {
			const dInt32 jj = m_matrixRowsIndex[j];
			const dgLeftHandSide* const row_j = &m_leftHandSide[jj];

			const dInt32 m0_j = m_pairs[j].m_m0;
			const dInt32 m1_j = m_pairs[j].m_m1;
			dgVector acc(dgVector::m_zero);
			bool hasEffect = false;
			if (m0_i == m0_j) {
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			} else if (m0_i == m1_j) {
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}

			if (m1_i == m1_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			} else if (m1_i == m0_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}

			if (hasEffect) {
				acc = acc.AddHorizontal();
				dFloat32 val = acc.GetScalar();
				matrixRow10[j] = val;
			}
		}
	}
}

void ndSkeletonContainer::ConditionMassMatrix () const
{
	DG_TRACKTIME();
	dgForcePair* const forcePair = dgAlloca(dgForcePair, m_nodeCount);
//	dgForcePair* const accelPair = dgAlloca(dgForcePair, m_nodeCount);
	const dgSpatialVector zero(dgSpatialVector::m_zero);
//	accelPair[m_nodeCount - 1].m_body = zero;
//	accelPair[m_nodeCount - 1].m_joint = zero;

	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dInt32 i = 0; i < m_auxiliaryRowCount; i ++) {
		dInt32 entry0 = 0;
		dInt32 startjoint = m_nodeCount;
		const dFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		for (dInt32 j = 0; j < m_nodeCount - 1; j++) {
			const ndNode* const node = m_nodesOrder[j];
			const dInt32 index = node->m_index;
			//accelPair[index].m_body = zero;
			//dgSpatialVector& a = accelPair[index].m_joint;
			forcePair[index].m_body = zero;
			dgSpatialVector& a = forcePair[index].m_joint;

			const int count = node->m_dof;
			for (dInt32 k = 0; k < count; k++) {
				const dFloat32 value = matrixRow10[entry0];
				a[k] = value;
				startjoint = (value == 0.0f) ? startjoint : dgMin(startjoint, index);
				entry0++;
			}
		}

		startjoint = (startjoint == m_nodeCount) ? 0 : startjoint;
		dAssert(startjoint < m_nodeCount);
		//SolveForward(forcePair, accelPair, startjoint);
		forcePair[m_nodeCount - 1].m_body = zero;
		forcePair[m_nodeCount - 1].m_joint = zero;
		SolveForward(forcePair, forcePair, startjoint);
		SolveBackward(forcePair);

		dInt32 entry1 = 0;
		dFloat32* const deltaForcePtr = &m_deltaForce[i * primaryCount];
		for (dInt32 j = 0; j < m_nodeCount - 1; j++) {
			const ndNode* const node = m_nodesOrder[j];
			const dInt32 index = node->m_index;
			const dgSpatialVector& f = forcePair[index].m_joint;
			const int count = node->m_dof;
			for (dInt32 k = 0; k < count; k++) {
				deltaForcePtr[entry1] = dFloat32(f[k]);
				entry1++;
			}
		}
	}
}

void ndSkeletonContainer::RebuildMassMatrix(const dFloat32* const diagDamp) const
{
	DG_TRACKTIME();
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	dInt16* const indexList = dgAlloca(dInt16, primaryCount);
	for (dInt32 i = 0; i < m_auxiliaryRowCount; i ++) {
		const dFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		dFloat32* const matrixRow11 = &m_massMatrix11[i * m_auxiliaryRowCount];

		dInt32 indexCount = 0;
		for (dInt32 k = 0; k < primaryCount; k++) {
			indexList[indexCount] = dInt16(k);
			indexCount += (matrixRow10[k] != dFloat32(0.0f)) ? 1 : 0;
		}

		for (dInt32 j = i; j < m_auxiliaryRowCount; j++) {
			dFloat32 offDiagonal = matrixRow11[j];
			const dFloat32* const row10 = &m_deltaForce[j * primaryCount];
			for (dInt32 k = 0; k < indexCount; k++) {
				dInt32 index = indexList[k];
				offDiagonal += matrixRow10[index] * row10[index];
			}
			matrixRow11[j] = offDiagonal;
			m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagonal;
		}

		matrixRow11[i] = dgMax(matrixRow11[i], diagDamp[i]);
	}
}

void ndSkeletonContainer::InitLoopMassMatrix(const dgJointInfo* const jointInfoArray)
{
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	dgInt8* const memoryBuffer = CalculateBufferSizeInBytes(jointInfoArray);

	m_frictionIndex = (dInt32*)memoryBuffer;
	m_matrixRowsIndex = (dInt32*)&m_frictionIndex[m_rowCount];
	m_pairs = (ndNodePair*)&m_matrixRowsIndex[m_rowCount];
	m_massMatrix11 = (dFloat32*)&m_pairs[m_rowCount];
	m_massMatrix10 = (dFloat32*)&m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
	m_deltaForce = &m_massMatrix10[m_auxiliaryRowCount * primaryCount];

	dFloat32* const diagDamp = dgAlloca(dFloat32, m_auxiliaryRowCount);
	dInt32* const boundRow = dgAlloca(dInt32, m_auxiliaryRowCount);

	m_blockSize = 0;
	dInt32 primaryIndex = 0;
	dInt32 auxiliaryIndex = 0;
	for (dInt32 i = 0; i < m_nodeCount - 1; i++) {
		const ndNode* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

		const dInt32 m0 = jointInfo->m_m0;
		const dInt32 m1 = jointInfo->m_m1;

		const dInt32 primaryDof = node->m_dof;
		const dInt32 first = jointInfo->m_pairStart;

		for (dInt32 j = 0; j < primaryDof; j++) {
			const dInt32 index = node->m_sourceJacobianIndex[j];
			m_pairs[primaryIndex].m_m0 = m0;
			m_pairs[primaryIndex].m_m1 = m1;
			m_frictionIndex[primaryIndex] = 0;
			m_matrixRowsIndex[primaryIndex] = first + index;
			primaryIndex++;
		}

		const dInt32 auxiliaryDof = jointInfo->m_pairCount - primaryDof;
		for (dInt32 j = 0; j < auxiliaryDof; j++) {
			const dInt32 index = node->m_sourceJacobianIndex[primaryDof + j];
			const dgRightHandSide* const rhs = &m_rightHandSide[first + index];

			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_frictionIndex[auxiliaryIndex + primaryCount] = 0;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + index;
			const dInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= dFloat32(-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32(DG_LCP_MAX_VALUE)) ? 1 : 0;
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}
	dAssert (m_loopRowCount == (m_auxiliaryRowCount - auxiliaryIndex));
	
	const dInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dInt32 j = 0; j < loopCount; j ++) {
		const ndConstraint* const joint = m_loopingJoints[j];
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const dInt32 m0 = jointInfo->m_m0;
		const dInt32 m1 = jointInfo->m_m1;
		const dInt32 first = jointInfo->m_pairStart;
		const dInt32 auxiliaryDof = jointInfo->m_pairCount;

		for (dInt32 i = 0; i < auxiliaryDof; i++) {
			const dgRightHandSide* const rhs = &m_rightHandSide[first + i];
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_frictionIndex[auxiliaryIndex + primaryCount] = (rhs->m_normalForceIndex < 0) ? 0 : rhs->m_normalForceIndex - i;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			const dInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= dFloat32(-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32(DG_LCP_MAX_VALUE)) ? 1 : 0;
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}

	dAssert(primaryIndex == primaryCount);
	dAssert(auxiliaryIndex == m_auxiliaryRowCount);

	for (dInt32 i = 1; i < auxiliaryIndex; i++) {
		dInt32 j = i;
		dInt32 tmpBoundRow = boundRow[j];
		ndNodePair tmpPair(m_pairs[primaryCount + j]);
		dInt32 tmpFrictionIndex = m_frictionIndex[primaryCount + j];
		dInt32 tmpMatrixRowsIndex = m_matrixRowsIndex[primaryCount + j];

		for (; j && (boundRow[j - 1] < tmpBoundRow); j--) {
			dAssert(j > 0);
			boundRow[j] = boundRow[j - 1];
			m_pairs[primaryCount + j] = m_pairs[primaryCount + j - 1];
			m_frictionIndex[primaryCount + j] = m_frictionIndex[primaryCount + j - 1];
			m_matrixRowsIndex[primaryCount + j] = m_matrixRowsIndex[primaryCount + j - 1];
		}
		boundRow[j] = tmpBoundRow;
		m_pairs[primaryCount + j] = tmpPair;
		m_frictionIndex[primaryCount + j] = tmpFrictionIndex;
		m_matrixRowsIndex[primaryCount + j] = tmpMatrixRowsIndex;
	}

	memset(m_massMatrix10, 0, primaryCount * m_auxiliaryRowCount * sizeof(dFloat32));
	memset(m_massMatrix11, 0, m_auxiliaryRowCount * m_auxiliaryRowCount * sizeof(dFloat32));

	CalculateLoopMassMatrixCoefficients(diagDamp);
	ConditionMassMatrix ();
	RebuildMassMatrix(diagDamp);

	if (m_blockSize) {
		FactorizeMatrix(m_blockSize, m_auxiliaryRowCount, m_massMatrix11, diagDamp);
		const int boundedSize = m_auxiliaryRowCount - m_blockSize;
		dFloat32* const acc = dgAlloca(dFloat32, m_auxiliaryRowCount);
		dInt32 rowStart = 0;
		for (dInt32 i = 0; i < m_blockSize; i++) {
			memset(acc, 0, boundedSize * sizeof (dFloat32));
			const dFloat32* const row = &m_massMatrix11[rowStart];
			for (dInt32 j = 0; j < i; j++) {
				const dFloat32 s = row[j];
				const dFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (dInt32 k = 0; k < boundedSize; k++) {
					acc[k] += s * x[k];
				}
			}

			dFloat32* const x = &m_massMatrix11[rowStart + m_blockSize];
			const dFloat32 den = -dFloat32(1.0f) / row[i];
			for (dInt32 j = 0; j < boundedSize; j++) {
				x[j] = (x[j] + acc[j]) * den;
			}
			rowStart += m_auxiliaryRowCount;
		}

		for (dInt32 i = m_blockSize - 1; i >= 0; i--) {
			memset(acc, 0, boundedSize * sizeof (dFloat32));
			for (dInt32 j = i + 1; j < m_blockSize; j++) {
				const dFloat32 s = m_massMatrix11[j * m_auxiliaryRowCount + i];
				const dFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (dInt32 k = 0; k < boundedSize; k++) {
					acc[k] += s * x[k];
				}
			}

			dFloat32* const x = &m_massMatrix11[i * m_auxiliaryRowCount + m_blockSize];
			const dFloat32 den = dFloat32(1.0f) / m_massMatrix11[i * m_auxiliaryRowCount + i];
			for (dInt32 j = 0; j < boundedSize; j++) {
				x[j] = (x[j] - acc[j]) * den;
			}
		}

		for (dInt32 i = 0; i < boundedSize; i++) {
			for (int j = 0; j < m_blockSize; j++) {
				acc[j] = m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize + i];
			}

			dFloat32* const arow = &m_massMatrix11[(m_blockSize + i) * m_auxiliaryRowCount + m_blockSize];
			for (int j = i; j < boundedSize; j++) {
				const dFloat32* const row1 = &m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount];
				dFloat32 elem = row1[m_blockSize + i] + dgDotProduct(m_blockSize, acc, row1);
				arow[j] = elem;
				m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount + m_blockSize + i] = elem;
			}
			arow[i] += diagDamp[m_blockSize + i];
		}
		dAssert (dgTestPSDmatrix(m_auxiliaryRowCount - m_blockSize, m_auxiliaryRowCount, &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize]));
	}
}

bool ndSkeletonContainer::SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const
{
	return true;
}

DG_INLINE void ndSkeletonContainer::SolveForward(dgForcePair* const force, const dgForcePair* const accel, dInt32 startNode) const
{
	dgSpatialVector zero (dgSpatialVector::m_zero);
	for (dInt32 i = 0; i < startNode; i++) {
		force[i].m_body = zero;
		force[i].m_joint = zero;
	}
	for (dInt32 i = startNode; i < m_nodeCount - 1; i++) {
		ndNode* const node = m_nodesOrder[i];
		dAssert(node->m_joint);
		dAssert(node->m_index == i);
		dgForcePair& f = force[i];
		const dgForcePair& a = accel[i];
		f.m_body = a.m_body;
		f.m_joint = a.m_joint;
		for (ndNode* child = node->m_child; child; child = child->m_sibling) {
			dAssert(child->m_joint);
			dAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(f);
	}

	force[m_nodeCount - 1] = accel[m_nodeCount - 1];
	for (ndNode* child = m_nodesOrder[m_nodeCount - 1]->m_child; child; child = child->m_sibling) {
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	for (dInt32 i = startNode; i < m_nodeCount - 1; i++) {
		ndNode* const node = m_nodesOrder[i];
		dgForcePair& f = force[i];
		node->BodyDiagInvTimeSolution(f);
		node->JointDiagInvTimeSolution(f);
	}
	m_nodesOrder[m_nodeCount - 1]->BodyDiagInvTimeSolution(force[m_nodeCount - 1]);
}

DG_INLINE void ndSkeletonContainer::SolveBackward(dgForcePair* const force) const
{
	for (dInt32 i = m_nodeCount - 2; i >= 0; i--) {
		ndNode* const node = m_nodesOrder[i];
		dAssert(node->m_index == i);
		dgForcePair& f = force[i];
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyJacobianTimeSolutionBackward(f);
	}
}

DG_INLINE void ndSkeletonContainer::CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const
{
	SolveForward(force, accel);
	SolveBackward(force);
}

DG_INLINE void ndSkeletonContainer::UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const dgForcePair* const force) const
{
	dgVector zero (dgVector::m_zero);
	for (dInt32 i = 0; i < (m_nodeCount - 1)  ; i ++) {
		ndNode* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		dAssert(i == node->m_index);

		const dgSpatialVector& f = force[i].m_joint;
		dAssert(jointInfo->m_joint == node->m_joint);
		const dInt32 first = jointInfo->m_pairStart;
		const dInt32 count = node->m_dof;
		for (dInt32 j = 0; j < count; j ++) {
			const dInt32 k = node->m_sourceJacobianIndex[j];
			dgRightHandSide* const rhs = &m_rightHandSide[first + k];
			const dgLeftHandSide* const row = &m_leftHandSide[first + k];

			rhs->m_force += dFloat32(f[j]);
			dgVector jointForce = dFloat32 (f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
		}

		const dInt32 m0 = jointInfo->m_m0;
		const dInt32 m1 = jointInfo->m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}
}

DG_INLINE void ndSkeletonContainer::CalculateJointAccel(dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgForcePair* const accel) const
{
	const dgSpatialVector zero (dgSpatialVector::m_zero);
	for (dInt32 i = 0; i < m_nodeCount - 1; i++) {
		ndNode* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);

		dgForcePair& a = accel[i];
		dAssert(node->m_body);
		a.m_body = zero;
		a.m_joint = zero;

		dAssert(node->m_joint);
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		dAssert(jointInfo->m_joint == node->m_joint);

		const dInt32 first = jointInfo->m_pairStart;
		const dInt32 dof = jointInfo->m_pairCount;
		const dInt32 m0 = jointInfo->m_m0;
		const dInt32 m1 = jointInfo->m_m1;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (dInt32 j = 0; j < dof; j++) {
			const dInt32 k = node->m_sourceJacobianIndex[j];
			const dgLeftHandSide* const row = &m_leftHandSide[first + k];
			const dgRightHandSide* const rhs = &m_rightHandSide[first + k];
			dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
						  row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
			a.m_joint[j] = -(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - diag.AddHorizontal().GetScalar());
		}
	}
	dAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body = zero;
	accel[m_nodeCount - 1].m_joint = zero;
}

void ndSkeletonContainer::SolveLcp(dInt32 stride, dInt32 size, const dFloat32* const matrix, const dFloat32* const x0, dFloat32* const x, const dFloat32* const b, const dFloat32* const low, const dFloat32* const high, const dInt32* const normalIndex) const
{
	D_TRACKTIME();
	if (m_world->GetCurrentPlugin()) {
		dgWorldPlugin* const plugin = m_world->GetCurrentPlugin()->GetInfo().m_plugin;
		plugin->SolveDenseLcp(stride, size, matrix, x0, x, b, low, high, normalIndex);
	} else {
#if 0
		// sequential Sidle iteration
		const dFloat32 sor = dFloat32(1.125f);
		const dFloat32 tol2 = dFloat32(0.25f);
		const dInt32 maxIterCount = 64;

		dFloat32* const invDiag1 = dgAlloca(dFloat32, size);

		int rowStart = 0;
		for (dInt32 i = 0; i < size; i++) {
			const int index = normalIndex[i];
			const dFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : 1.0f;
			const dFloat32 l = low[i] * coefficient - x0[i];
			const dFloat32 h = high[i] * coefficient - x0[i];;
			x[i] = dgClamp(dFloat32(0.0f), l, h);
			invDiag1[i] = dFloat32(1.0f) / matrix[rowStart + i];
			rowStart += stride;
		}

		dFloat32 tolerance(tol2 * dFloat32(2.0f));
		const dFloat32* const invDiag = invDiag1;

		dInt32 iterCount = 0;
		for (dInt32 k = 0; (k < maxIterCount) && (tolerance > tol2); k++) {
			iterCount++;
			dInt32 base = 0;
			tolerance = dFloat32(0.0f);
			for (dInt32 i = 0; i < size; i++) {
				const dFloat32* const row = &matrix[base];
				dFloat32 r = b[i] - dgDotProduct(size, row, x);

				const int index = normalIndex[i];
				const dFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : dFloat32(1.0f);
				const dFloat32 l = low[i] * coefficient - x0[i];
				const dFloat32 h = high[i] * coefficient - x0[i];

				dFloat32 f = x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor;
				if (f > h) {
					f = h;
				} else if (f < l) {
					f = l;
				} else {
					tolerance += r * r;
				}
				x[i] = f;
				base += stride;
			}
		}
#else
		// ready for parallelization
		const dFloat32 sor = dFloat32(1.125f);
		const dFloat32 tol2 = dFloat32(0.25f);
		const dInt32 maxIterCount = 64;

		dFloat32* const invDiag1 = dgAlloca(dFloat32, size);
		dFloat32* const residual = dgAlloca(dFloat32, size);

		dInt32 rowStart = 0;
		for (dInt32 i = 0; i < size; i++) {
			const int index = normalIndex[i];
			const dFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : 1.0f;
			const dFloat32 l = low[i] * coefficient - x0[i];
			const dFloat32 h = high[i] * coefficient - x0[i];;
			x[i] = dgClamp(dFloat32(0.0f), l, h);
			invDiag1[i] = dFloat32(1.0f) / matrix[rowStart + i];
			rowStart += stride;
		}

		dInt32 base = 0;
		for (dInt32 i = 0; i < size; i++) {
			const dFloat32* const row = &matrix[base];
			residual[i] = b[i] - dgDotProduct(size, row, x);
			base += stride;
		}

		dInt32 iterCount = 0;
		dFloat32 tolerance(tol2 * dFloat32(2.0f));
		const dFloat32* const invDiag = invDiag1;
		const dFloat32 one = dFloat32(1.0f);
		for (dInt32 k = 0; (k < maxIterCount) && (tolerance > tol2); k++) {
			base = 0;
			iterCount++;
			tolerance = dFloat32(0.0f);
			for (dInt32 i = 0; i < size; i++) {
				const dFloat32 r = residual[i];
				const int index = normalIndex[i];
				const dFloat32 coefficient = index ? x[i + index] + x0[i + index] : one;
				const dFloat32 l = low[i] * coefficient - x0[i];
				const dFloat32 h = high[i] * coefficient - x0[i];

				const dFloat32* const row = &matrix[base];
#if 0
				dFloat32 f = x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor;
				if (f > h) {
					f = h;
				} else if (f < l) {
					f = l;
				} else {
					tolerance += r * r;
				}
				const dFloat32 dx = f - x[i];
#else
				const dFloat32 f = dgClamp(x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor, l, h);
				const dFloat32 dx = f - x[i];
				const dFloat32 dr = dx * row[i];
				tolerance += dr * dr;
#endif
				x[i] = f;
				if (dgAbs (dx) > dFloat32 (1.0e-6f)) {
					for (dInt32 j = 0; j < size; j++) {
						residual[j] -= row[j] * dx;
					}
				}
				base += stride;
			}
		}
#endif
	}
}

void ndSkeletonContainer::SolveBlockLcp(dInt32 size, dInt32 blockSize, const dFloat32* const x0, dFloat32* const x, dFloat32* const b, const dFloat32* const low, const dFloat32* const high, const dInt32* const normalIndex) const
{
	if (blockSize) {
		dgSolveCholesky(blockSize, size, m_massMatrix11, x, b);
		if (blockSize != size) {

			dInt32 base = blockSize * size;
			for (dInt32 i = blockSize; i < size; i++) {
				b[i] -= dgDotProduct(blockSize, &m_massMatrix11[base], x);
				base += size;
			}

			const int boundedSize = size - blockSize;
			SolveLcp(
				size, boundedSize, &m_massMatrix11[blockSize * size + blockSize],
				&x0[blockSize], &x[blockSize], &b[blockSize], &low[blockSize], &high[blockSize], &normalIndex[blockSize]);

			for (dInt32 j = 0; j < blockSize; j++) {
				const dFloat32* const row = &m_massMatrix11[j * size + blockSize];
				dFloat32 acc = dFloat32 (0.0f);
				for (dInt32 i = 0; i < boundedSize; i++) {
					acc += x[blockSize + i] * row[i];
				}
				x[j] += acc;
			}
		}
	} else {
		SolveLcp(size, size, m_massMatrix11, x0, x, b, low, high, normalIndex);
	}
}

void ndSkeletonContainer::SolveAuxiliary(const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const dgForcePair* const accel, dgForcePair* const force) const
{
	dFloat32* const f = dgAlloca(dFloat32, m_rowCount);
	dFloat32* const u = dgAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const b = dgAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const u0 = dgAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const low = dgAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const high = dgAlloca(dFloat32, m_auxiliaryRowCount);
	dInt32* const normalIndex = dgAlloca(dInt32, m_auxiliaryRowCount);

	dInt32 primaryIndex = 0;
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (dInt32 i = 0; i < m_nodeCount - 1; i++) {
		const ndNode* const node = m_nodesOrder[i];
		const dInt32 primaryDof = node->m_dof;
		const dgSpatialVector& forceSpatial = force[i].m_joint;

		for (dInt32 j = 0; j < primaryDof; j++) {
			f[primaryIndex] = dFloat32(forceSpatial[j]);
			primaryIndex++;
		}
	}

	dAssert (primaryIndex == primaryCount);
	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const int index = m_matrixRowsIndex[primaryCount + i];
		const dgLeftHandSide* const row = &m_leftHandSide[index];
		const dgRightHandSide* const rhs = &m_rightHandSide[index];

		const int m0 = m_pairs[primaryCount + i].m_m0;
		const int m1 = m_pairs[primaryCount + i].m_m1;

		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		f[primaryCount + i] = dFloat32(0.0f);

		dgVector acc(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
					 row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
		b[i] = rhs->m_coordenateAccel - acc.AddHorizontal().GetScalar();

		normalIndex[i] = m_frictionIndex[primaryCount + i];
		u0[i] = rhs->m_force;
		low[i] = rhs->m_lowerBoundFrictionCoefficent;
		high[i] = rhs->m_upperBoundFrictionCoefficent;
	}

	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		dFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		b[i] -= dgDotProduct(primaryCount, matrixRow10, f);
	}

	SolveBlockLcp(m_auxiliaryRowCount, m_blockSize, u0, u, b, low, high, normalIndex);

	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dFloat32 s = u[i];
		f[primaryCount + i] = s;
		dgMulAdd(primaryCount, f, f, &m_deltaForce[i * primaryCount], s);
	}

	for (dInt32 i = 0; i < m_rowCount; i++) {
		dInt32 index = m_matrixRowsIndex[i];
		dgRightHandSide* const rhs = &m_rightHandSide[index];
		const dgLeftHandSide* const row = &m_leftHandSide[index];
		const dInt32 m0 = m_pairs[i].m_m0;
		const dInt32 m1 = m_pairs[i].m_m1;

		rhs->m_force += f[i];
		dgVector jointForce(f[i]);
		internalForces[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
		internalForces[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
		internalForces[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
		internalForces[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
	}
}

dgInt8* ndSkeletonContainer::CalculateBufferSizeInBytes (const dgJointInfo* const jointInfoArray)
{
	dInt32 rowCount = 0;
	dInt32 auxiliaryRowCount = 0;
	if (m_nodesOrder) {
		for (dInt32 i = 0; i < m_nodeCount - 1; i++) {
			ndNode* const node = m_nodesOrder[i];
			rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			auxiliaryRowCount += node->GetAuxiliaryRows(jointInfoArray, m_rightHandSide);
		}
	}

	dInt32 extraAuxiliaryRows = 0;
	const dInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dInt32 j = 0; j < loopCount; j++) {
		const ndConstraint* const joint = m_loopingJoints[j];
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount+= extraAuxiliaryRows;

	dInt32 size = sizeof (dInt32) * rowCount;
	size += sizeof (dInt32) * rowCount;
	size += sizeof (ndNodePair) * rowCount;
	size += sizeof(dFloat32) * auxiliaryRowCount * auxiliaryRowCount;		// matrix11[auxiliaryRowCount * auxiliaryRowCount]
	size += sizeof (dFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof (dFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size = (size + 1024) & -0x10;
	m_auxiliaryMemoryBuffer.ResizeIfNecessary((size + 1024) & -0x10);
	return &m_auxiliaryMemoryBuffer[0];
}

void ndSkeletonContainer::InitMassMatrix(const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, bool consideredCloseLoop)
{
	D_TRACKTIME();
	dInt32 rowCount = 0;
	dInt32 auxiliaryCount = 0;
	m_leftHandSide = leftHandSide;
	m_rightHandSide = rightHandSide;
	m_consideredCloseLoop = consideredCloseLoop ? 1 : 0;

	dgSpatialMatrix* const bodyMassArray = dgAlloca (dgSpatialMatrix, m_nodeCount);
	dgSpatialMatrix* const jointMassArray = dgAlloca (dgSpatialMatrix, m_nodeCount);

	if (m_nodesOrder) {
		for (dInt32 i = 0; i < m_nodeCount - 1; i++) {
			ndNode* const node = m_nodesOrder[i];
			const dgJointInfo& info = jointInfoArray[node->m_joint->m_index];
			rowCount += info.m_pairCount;
			auxiliaryCount += node->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
		}
		m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
	}
	m_rowCount = dInt16 (rowCount);
	m_auxiliaryRowCount = dInt16 (auxiliaryCount);

	dInt32 loopRowCount = 0;
	const dInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dInt32 j = 0; j < loopCount; j++) {
		const ndConstraint* const joint = m_loopingJoints[j];
		loopRowCount += jointInfoArray[joint->m_index].m_pairCount;
	}
	m_loopRowCount = dInt16 (loopRowCount);
	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;

	if (m_auxiliaryRowCount && m_consideredCloseLoop) {
		InitLoopMassMatrix(jointInfoArray);
	}
}

void ndSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces)
{
	D_TRACKTIME();
	dgForcePair* const force = dgAlloca(dgForcePair, m_nodeCount);
	dgForcePair* const accel = dgAlloca(dgForcePair, m_nodeCount);

	CalculateJointAccel(jointInfoArray, internalForces, accel);
	CalculateForce(force, accel);
	if (m_auxiliaryRowCount && m_consideredCloseLoop) {
		SolveAuxiliary (jointInfoArray, internalForces, accel, force);
	} else {
		UpdateForces(jointInfoArray, internalForces, force);
	}
}
#endif


ndSkeletonContainer::ndSkeletonContainer()
	:m_skeleton(nullptr)
	,m_nodesOrder(nullptr)
	,m_nodeList()
	,m_loopingJoints()
	//, m_pairs(nullptr)
	//, m_deltaForce(nullptr)
	//, m_massMatrix11(nullptr)
	//, m_massMatrix10(nullptr)
	//, m_rightHandSide(nullptr)
	//, m_leftHandSide(nullptr)
	//, m_frictionIndex(nullptr)
	//, m_matrixRowsIndex(nullptr)
	//, m_listNode(nullptr)
	//, m_loopingJoints(world->GetAllocator())
	//, m_auxiliaryMemoryBuffer(world->GetAllocator())
	//, m_lru(0)
	//, m_blockSize(0)
	//, m_nodeCount(1)
	,m_loopCount(0)
	,m_dynamicsLoopCount(0)
	//, m_rowCount(0)
	//, m_loopRowCount(0)
	//, m_auxiliaryRowCount(0)
	//, m_consideredCloseLoop(1)
{
}

ndSkeletonContainer::~ndSkeletonContainer()
{
	for (dInt32 i = 0; i < m_loopCount; i++) 
	{
		ndJointBilateralConstraint* const joint = m_loopingJoints[i]->GetAsBilateral();
		if (joint)
		{
			joint->m_isInSkeleton = false;
		}
	}
	
	for (dInt32 i = m_nodeList.GetCount() - 2; i >= 0; i--)
	{
		m_nodesOrder[i]->m_joint->m_isInSkeleton = false;
	}

	if (m_nodesOrder) 
	{
		dMemory::Free(m_nodesOrder);
	}

	//delete m_skeleton;
	m_nodeList.RemoveAll();
}

void ndSkeletonContainer::Init(ndBodyKinematic* const rootBody)
{
	m_skeleton = &m_nodeList.Append()->GetInfo();
	m_skeleton->m_body = rootBody;
	if (rootBody->GetInvMass() != dFloat32(0.0f))
	{
		dAssert(0);
		//rootBody->SetSkeleton(this);
		m_skeleton->m_body->SetSkeleton(this);
	}
}

ndSkeletonContainer::ndNode* ndSkeletonContainer::AddChild(ndJointBilateralConstraint* const joint, ndNode* const parent)
{
	//dAssert(m_skeleton->m_body);
	//dgMemoryAllocator* const allocator = m_world->GetAllocator();
	//ndNode* const node = new (allocator)ndNode(joint, parent);
	//m_nodeCount++;

	ndNode* const node = &m_nodeList.Append()->GetInfo();
	node->m_body = (joint->GetBody0() == parent->m_body) ? joint->GetBody1() : joint->GetBody0();
	node->m_joint = joint;
	node->m_parent = parent;
	node->m_swapJacobianBodiesIndex = (joint->GetBody0() == parent->m_body);

	dAssert(node->m_parent);
	dAssert(node->m_body->GetInvMass() != dFloat32(0.0f));
	if (node->m_parent->m_child)
	{
		node->m_sibling = node->m_parent->m_child;
	}
	node->m_parent->m_child = node;

	//m_nodeCount++;
	joint->SetSkeletonFlag(true);
	dAssert(node->m_body->GetScene()->GetWorld()->GetSentinelBody()->GetAsBodyKinematic() != node->m_body);
	node->m_body->SetSkeleton(this);
	return node;
}

void ndSkeletonContainer::SortGraph(ndNode* const root, dInt32& index)
{
	for (ndNode* node = root->m_child; node; node = node->m_sibling) 
	{
		SortGraph(node, index);
	}

	dAssert((m_nodeList.GetCount() - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_index = dInt16(index);
	index++;
	dAssert(index <= m_nodeList.GetCount());
}

void ndSkeletonContainer::Finalize(dInt32 loopJointsCount, ndJointBilateralConstraint** const loopJointArray)
{
	dAssert(m_nodeList.GetCount() >= 1);
	
	const ndBodyKinematic* const rootBody = m_skeleton->m_body;
	dAssert(((rootBody->GetInvMass() == dFloat32(0.0f)) && (m_skeleton->m_child->m_sibling == nullptr)) || (m_skeleton->m_body->GetInvMass() != dFloat32(0.0f)));
	
	m_nodesOrder = (ndNode**)dMemory::Malloc(m_nodeList.GetCount() * sizeof(ndNode*));
	
	dInt32 index = 0;
	SortGraph(m_skeleton, index);
	dAssert(index == m_nodeList.GetCount());
	
	if (loopJointsCount) 
	{
		dAssert(0);
	//	for (dInt32 i = 0; i < loopJointsCount; i++) {
	//		ndJointBilateralConstraint* const joint = loopJointArray[i];
	//		dAssert(joint->GetBody0()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
	//		dAssert(joint->GetBody1()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
	//		dAssert((FindNode((ndBodyKinematic*)joint->GetBody0()) || FindNode((ndBodyKinematic*)joint->GetBody1())));
	//		joint->m_isInSkeleton = true;
	//
	//		m_loopingJoints[m_loopCount] = joint;
	//		m_loopCount++;
	//	}
	}
}

void ndSkeletonContainer::ClearSelfCollision()
{
	m_dynamicsLoopCount = 0;
}

void ndSkeletonContainer::AddSelfCollisionJoint(ndConstraint* const joint)
{
	dScopeSpinLock lock(joint->GetBody0()->GetScene()->m_contactLock);
	if (m_loopingJoints.GetCount() < (m_loopCount + m_dynamicsLoopCount + 1)) 
	{
		m_loopingJoints.SetCount(2 * (m_loopCount + m_dynamicsLoopCount + 1));
	}
	m_loopingJoints[m_loopCount + m_dynamicsLoopCount] = joint;
	m_dynamicsLoopCount++;
}
