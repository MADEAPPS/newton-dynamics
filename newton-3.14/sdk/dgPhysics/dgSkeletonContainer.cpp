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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dgSkeletonContainer.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"



class dgSkeletonContainer::dgNodePair
{
	public:
	dgInt32 m_m0;
	dgInt32 m_m1;
};

DG_MSC_VECTOR_ALIGNMENT
class dgSkeletonContainer::dgForcePair
{
	public:
	dgSpatialVector m_joint;
	dgSpatialVector m_body;
} DG_GCC_VECTOR_ALIGNMENT;

DG_MSC_VECTOR_ALIGNMENT 
class dgSkeletonContainer::dgMatriData
{
	public:
	dgSpatialMatrix m_jt;
	dgSpatialMatrix m_invMass;
} DG_GCC_VECTOR_ALIGNMENT;

DG_MSC_VECTOR_ALIGNMENT 
class dgSkeletonContainer::dgBodyJointMatrixDataPair
{
	public:
	dgMatriData m_body;
	dgMatriData m_joint;
} DG_GCC_VECTOR_ALIGNMENT;


class dgSkeletonContainer::dgNode
{
	public:

	DG_CLASS_ALLOCATOR(allocator)
	dgNode(dgDynamicBody* const body)
		:m_body (body)
		,m_joint(NULL)
		,m_parent(NULL)
		,m_child(NULL)
		,m_sibling(NULL)
		,m_index(0)
		,m_dof(0)
		,m_swapJacobianBodiesIndex(0)
	{
	}

	dgNode (dgBilateralConstraint* const joint, dgNode* const parent)
		:m_body ((dgDynamicBody*) ((joint->GetBody0() == parent->m_body) ? joint->GetBody1() : joint->GetBody0()))
		,m_joint (joint)
		,m_parent(parent)
		,m_child(NULL)
		,m_sibling(NULL)
		,m_index(0)
		,m_dof(0)
		,m_swapJacobianBodiesIndex(joint->GetBody0() == parent->m_body)
	{
		dgAssert (m_parent);
		dgAssert (m_body->GetInvMass().m_w != dgFloat32 (0.0f));
		if (m_parent->m_child) {
			m_sibling = m_parent->m_child;
		}
		m_parent->m_child = this;
	}

	DG_INLINE ~dgNode()
	{
		m_body->SetSkeleton(NULL);

		dgNode* next;
		for (dgNode* ptr = m_child; ptr; ptr = next) {
			next = ptr->m_sibling;
			delete ptr;
		}
	}

	DG_INLINE void CalculateInertiaMatrix(dgSpatialMatrix* const bodyMassArray) const
	{
		dgSpatialMatrix& bodyMass = bodyMassArray[m_index];

		bodyMass = dgSpatialMatrix(dgFloat32(0.0f));
		if (m_body->GetInvMass().m_w != dgFloat32(0.0f)) {
			const dgFloat32 mass = m_body->GetMass().m_w;
			dgMatrix inertia(m_body->CalculateInertiaMatrix());
			for (dgInt32 i = 0; i < 3; i++) {
				bodyMass[i][i] = mass;
				for (dgInt32 j = 0; j < 3; j++) {
					bodyMass[i + 3][j + 3] = inertia[i][j];
				}
			}
		}
	}

	DG_INLINE void GetJacobians(const dgJointInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgSpatialMatrix* const jointMassArray)
	{
		dgAssert(m_parent);
		dgAssert(jointInfo->m_joint == m_joint);

		dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		dgSpatialMatrix& jointMass = jointMassArray[m_index];

		const dgInt32 start = jointInfo->m_pairStart;
		const dgSpatialVector zero (dgSpatialVector::m_zero);
		if (!m_swapJacobianBodiesIndex) {
			for (dgInt32 i = 0; i < m_dof; i++) {
				const dgInt32 k = m_sourceJacobianIndex[i];
				const dgRightHandSide* const rhs = &rightHandSide[start + k];
				const dgLeftHandSide* const row = &leftHandSide[start + k];
				jointMass[i] = zero;
				jointMass[i][i] = -rhs->m_diagDamp;
				bodyJt[i] = dgSpatialVector (row->m_Jt.m_jacobianM0.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM0.m_angular * dgVector::m_negOne);
				jointJ[i] = dgSpatialVector (row->m_Jt.m_jacobianM1.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM1.m_angular * dgVector::m_negOne);
			}
		} else {
			for (dgInt32 i = 0; i < m_dof; i++) {
				const dgInt32 k = m_sourceJacobianIndex[i];
				const dgRightHandSide* const rhs = &rightHandSide[start + k];
				const dgLeftHandSide* const row = &leftHandSide[start + k];
				jointMass[i] = zero;
				jointMass[i][i] = -rhs->m_diagDamp;
				bodyJt[i] = dgSpatialVector(row->m_Jt.m_jacobianM1.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM1.m_angular * dgVector::m_negOne);
				jointJ[i] = dgSpatialVector(row->m_Jt.m_jacobianM0.m_linear * dgVector::m_negOne, row->m_Jt.m_jacobianM0.m_angular * dgVector::m_negOne);
			}
		}
	}

	dgInt32 Factorize(const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgSpatialMatrix* const bodyMassArray, dgSpatialMatrix* const jointMassArray)
	{
		CalculateInertiaMatrix(bodyMassArray);

		m_ordinals = m_ordinalInit;
		dgInt32 boundedDof = 0;
		if (m_joint) {
			dgAssert (m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);

			m_dof = 0;
			dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			for (dgInt32 i = 0; i < count; i++) {
				dgInt32 k = m_sourceJacobianIndex[i];
				const dgRightHandSide* const rhs = &rightHandSide[k + first];
				if ((rhs->m_lowerBoundFrictionCoefficent <= dgFloat32 (-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dgFloat32 (DG_LCP_MAX_VALUE))) {
					m_dof ++;
				} else {
					dgSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[count - 1]);
					i--;
					count--;
				}
			}
			dgAssert (m_dof > 0);
			dgAssert (m_dof <= 6);
			boundedDof += jointInfo->m_pairCount - count;
			GetJacobians(jointInfo, leftHandSide, rightHandSide, jointMassArray);
		}

		dgSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
		const dgSpatialMatrix& bodyMass = bodyMassArray[m_index];
		if (m_body->GetInvMass().m_w != dgFloat32(0.0f)) {
			for (dgNode* child = m_child; child; child = child->m_sibling) {
				CalculateBodyDiagonal(child, bodyMassArray, jointMassArray);
			}
			bodyInvMass = bodyMass.Inverse(6);
		} else {
			bodyInvMass = dgSpatialMatrix(dgFloat32(0.0f));
		}

		if (m_joint) {
			dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
			dgAssert(m_parent);
			for (dgInt32 i = 0; i < m_dof; i++) {
				bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
			}
			CalculateJointDiagonal(bodyMassArray, jointMassArray);
			CalculateJacobianBlock();
		}
		return boundedDof;
	}

	DG_INLINE void CalculateBodyDiagonal(dgNode* const child, dgSpatialMatrix* const bodyMassArray, const dgSpatialMatrix* const jointMassArray)
	{
		dgAssert(child->m_joint);
		
		dgSpatialMatrix copy (dgFloat32(0.0f));
		const dgInt32 dof = child->m_dof;
		const dgSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
		const dgSpatialMatrix& childDiagonal = jointMassArray[child->m_index];
		for (dgInt32 i = 0; i < dof ; i++) {
			const dgSpatialVector& jacobian = jacobianMatrix[i];
			for (dgInt32 j = 0; j < dof ; j++) {
				dgAssert(dgAreEqual (dgFloat64(childDiagonal[i][j]), dgFloat64(childDiagonal[j][i]), dgFloat64(1.0e-5f)));
				dgFloat64 val = childDiagonal[i][j];
				copy[j] = copy[j] + jacobian.Scale(val);
			}
		}

		dgSpatialMatrix& bodyMass = bodyMassArray[m_index];
		for (dgInt32 i = 0; i < dof; i++) {
			const dgSpatialVector& Jacobian = copy[i];
			const dgSpatialVector& JacobianTranspose = jacobianMatrix[i];
			for (dgInt32 j = 0; j < 6; j++) {
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
		for (dgInt32 i = 0; i < m_dof; i++) {
			tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
		}

		dgSpatialMatrix& jointMass = jointMassArray[m_index];
		for (dgInt32 i = 0; i < m_dof; i++) {
			dgFloat64 a = bodyJt[i].DotProduct(tmp[i]);
			jointMass[i][i] -= a;
			for (dgInt32 j = i + 1; j < m_dof; j++) {
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
		for (dgInt32 i = 0; i < m_dof; i++) {
			copy[i] = jointJ[i];
			jointJ[i] = zero;
		}

		const dgSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
		for (dgInt32 i = 0; i < m_dof; i++) {
			const dgSpatialVector& jacobian = copy[i];
			const dgSpatialVector& invDiagonalRow = jointInvMass[i];
			for (dgInt32 j = 0; j < m_dof; j++) {
				dgFloat64 val = invDiagonalRow[j];
				jointJ[j] = jointJ[j] + jacobian.Scale(val);
			}
		}
	}

	DG_INLINE void JointJacobianTimeMassForward (dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
		}
	}

	DG_INLINE void BodyJacobianTimeMassForward(const dgForcePair& force, dgForcePair& parentForce) const 
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			parentForce.m_body = parentForce.m_body + jointJ[i].Scale(-force.m_joint[i]);
		}
	}

	DG_INLINE void JointJacobianTimeSolutionBackward(dgForcePair& force, const dgForcePair& parentForce)
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		const dgSpatialVector& f = parentForce.m_body;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= f.DotProduct(jointJ[i]);
		}
	}

	DG_INLINE void BodyJacobianTimeSolutionBackward(dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
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

	DG_INLINE dgInt32 GetAuxiliaryRows(const dgJointInfo* const jointInfoArray, const dgRightHandSide* const rightHandSide) const
	{
		dgInt32 rowCount = 0;
		if (m_joint) {
			dgAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			for (dgInt32 i = 0; i < count; i++) {
				const dgRightHandSide* const rhs = &rightHandSide[i + first];
				if (!((rhs->m_lowerBoundFrictionCoefficent <= dgFloat32 (-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dgFloat32 (DG_LCP_MAX_VALUE)))) {
					rowCount++;
				}
			}
		}
		return rowCount;
	}
	
	dgBodyJointMatrixDataPair m_data;
	dgDynamicBody* m_body;
	dgBilateralConstraint* m_joint;
	dgNode* m_parent;
	dgNode* m_child;
	dgNode* m_sibling;
	union
	{
		dgInt64 m_ordinals;
		dgInt8 m_sourceJacobianIndex[8];
	};
	dgInt16 m_index;
	dgInt8 m_dof;
	dgInt8 m_swapJacobianBodiesIndex;
	static dgInt64 m_ordinalInit;
};

dgInt64 dgSkeletonContainer::dgNode::m_ordinalInit = 0x050403020100ll;

dgSkeletonContainer::dgSkeletonContainer(dgWorld* const world, dgDynamicBody* const rootBody)
	:m_world(world)
	,m_skeleton(new (world->GetAllocator()) dgNode(rootBody))
	,m_nodesOrder(NULL)
	,m_pairs(NULL)
	,m_deltaForce(NULL)
	,m_massMatrix11(NULL)
	,m_massMatrix10(NULL)
	,m_rightHandSide(NULL)
	,m_leftHandSide(NULL)
	,m_frictionIndex(NULL)
	,m_matrixRowsIndex(NULL)
	,m_listNode(NULL)
	,m_loopingJoints(world->GetAllocator())
	,m_auxiliaryMemoryBuffer(world->GetAllocator())
	,m_lru(0)
	,m_blockSize(0)
	,m_nodeCount(1)
	,m_loopCount(0)
	,m_dynamicsLoopCount(0)
	,m_rowCount(0)
	,m_loopRowCount(0)
	,m_auxiliaryRowCount(0)
	,m_consideredCloseLoop(1)
{
	if (rootBody->GetInvMass().m_w != dgFloat32 (0.0f)) {
		rootBody->SetSkeleton(this);
	}
}

dgSkeletonContainer::~dgSkeletonContainer()
{
	for (dgInt32 i = 0; i < m_loopCount; i++) {
		dgConstraint* const joint = m_loopingJoints[i];
		joint->m_isInSkeleton = false;
	}

	for (dgInt32 i = 0; i < m_nodeCount - 1; i ++) {
		m_nodesOrder[i]->m_joint->m_isInSkeleton = false;
	}

	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	if (m_nodesOrder) {
		allocator->Free(m_nodesOrder);
	}

	delete m_skeleton;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::GetRoot () const
{
	return m_skeleton;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::GetParent (dgNode* const node) const
{
	return node->m_parent;
}

dgDynamicBody* dgSkeletonContainer::GetBody(dgSkeletonContainer::dgNode* const node) const
{
	return node->m_body;
}

dgBilateralConstraint* dgSkeletonContainer::GetJoint(dgSkeletonContainer::dgNode* const node) const
{
	return node->m_joint;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::GetFirstChild(dgSkeletonContainer::dgNode* const parent) const
{
	return parent->m_child;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::GetNextSiblingChild(dgSkeletonContainer::dgNode* const sibling) const
{
	return sibling->m_sibling;
}

dgWorld* dgSkeletonContainer::GetWorld() const
{
	return m_world;
}

void dgSkeletonContainer::ClearSelfCollision()
{
	m_dynamicsLoopCount = 0;
}

void dgSkeletonContainer::AddSelfCollisionJoint(dgConstraint* const joint)
{
	m_world->GlobalLock();
	m_loopingJoints[m_loopCount + m_dynamicsLoopCount] = joint;
	m_dynamicsLoopCount++;
	m_world->GlobalUnlock();
}

void dgSkeletonContainer::SortGraph(dgNode* const root, dgInt32& index)
{
	for (dgNode* node = root->m_child; node; node = node->m_sibling) {
		SortGraph(node, index);
	}

	dgAssert((m_nodeCount - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_index = dgInt16(index);
	index++;
	dgAssert(index <= m_nodeCount);
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::FindNode(dgDynamicBody* const body) const
{
	dgInt32 stack = 1;
	dgNode* stackPool[1024];

	stackPool[0] = m_skeleton;
	while (stack) {
		stack--;
		dgNode* const node = stackPool[stack];
		if (node->m_body == body) {
			return node;
		}

		for (dgNode* ptr = node->m_child; ptr; ptr = ptr->m_sibling) {
			stackPool[stack] = ptr;
			stack++;
			dgAssert(stack < dgInt32(sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return NULL;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::AddChild(dgBilateralConstraint* const joint, dgNode* const parent)
{
	dgAssert (m_skeleton->m_body);
	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	dgNode* const node = new (allocator)dgNode(joint, parent);
	m_nodeCount ++;

	joint->m_isInSkeleton = true;
	dgAssert (m_world->GetSentinelBody() != node->m_body);
	node->m_body->SetSkeleton(this);
	return node;
}

void dgSkeletonContainer::RemoveLoopJoint(dgBilateralConstraint* const joint)
{
	for (dgInt32 i = 0; i < m_loopCount; i++) {
		if (m_loopingJoints[i] == joint) {
			joint->m_isInSkeleton = false;
			m_loopCount--;
			m_loopingJoints[i] = m_loopingJoints[m_loopCount];
			break;
		}
	}
}

void dgSkeletonContainer::Finalize(dgInt32 loopJointsCount, dgBilateralConstraint** const loopJointArray)
{
	dgAssert(m_nodeCount >= 1);

	const dgDynamicBody* const rootBody = m_skeleton->m_body;
	dgAssert (((rootBody->GetInvMass().m_w == dgFloat32 (0.0f)) && (m_skeleton->m_child->m_sibling == NULL)) || (m_skeleton->m_body->GetInvMass().m_w != dgFloat32 (0.0f)));

	dgMemoryAllocator* const allocator = rootBody->GetWorld()->GetAllocator();
	m_nodesOrder = (dgNode**)allocator->Malloc(m_nodeCount * sizeof (dgNode*));

	dgInt32 index = 0;
	SortGraph(m_skeleton, index);
	dgAssert(index == m_nodeCount);

	if (loopJointsCount) {
		for (dgInt32 i = 0; i < loopJointsCount; i++) {
			dgBilateralConstraint* const joint = loopJointArray[i];
			dgAssert(joint->GetBody0()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert(joint->GetBody1()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert((FindNode((dgDynamicBody*)joint->GetBody0()) || FindNode((dgDynamicBody*)joint->GetBody1())));
			joint->m_isInSkeleton = true;

			m_loopingJoints[m_loopCount] = joint;
			m_loopCount++;
		}
	}
}

void dgSkeletonContainer::FactorizeMatrix(dgInt32 size, dgInt32 stride, dgFloat32* const matrix, dgFloat32* const diagDamp) const
{
	DG_TRACKTIME();
	//bool isPsdMatrix = false;
	//dgFloat32* const backupMatrix = dgAlloca(dgFloat32, size * stride);
	//do {
	//	{
	//		dgInt32 srcLine = 0;
	//		dgInt32 dstLine = 0;
	//		for (dgInt32 i = 0; i < size; i++) {
	//			memcpy(&backupMatrix[dstLine], &matrix[srcLine], size * sizeof(dgFloat32));
	//			srcLine += size;
	//			dstLine += stride;
	//		}
	//	}
	//	isPsdMatrix = dgCholeskyFactorization(size, stride, matrix);
	//	if (!isPsdMatrix) {
	//		dgInt32 srcLine = 0;
	//		dgInt32 dstLine = 0;
	//		for (dgInt32 i = 0; i < size; i++) {
	//			memcpy(&matrix[dstLine], &backupMatrix[srcLine], size * sizeof(dgFloat32));
	//			diagDamp[i] *= dgFloat32(4.0f);
	//			matrix[dstLine + i] += diagDamp[i];
	//			dstLine += size;
	//			srcLine += stride;
	//		}
	//	}
	//} while (!isPsdMatrix);

	bool isPsdMatrix = false;
	dgFloat32* const backupMatrix = dgAlloca(dgFloat32, size * stride);

	// save the matrix 
	dgInt32 srcLine = 0;
	dgInt32 dstLine = 0;
	for (dgInt32 i = 0; i < size; i++) {
		memcpy(&backupMatrix[dstLine], &matrix[srcLine], size * sizeof(dgFloat32));
		dstLine += size;
		srcLine += stride;
	}
	do {
		isPsdMatrix = dgCholeskyFactorization(size, stride, matrix);
		if (!isPsdMatrix) {
			srcLine = 0;
			dstLine = 0;
			for (dgInt32 i = 0; i < size; i++) {
				memcpy(&matrix[dstLine], &backupMatrix[srcLine], size * sizeof(dgFloat32));
				diagDamp[i] *= dgFloat32(4.0f);
				matrix[dstLine + i] += diagDamp[i];
				dstLine += stride;
				srcLine += size;
			}
		}
	} while (!isPsdMatrix);
}
void dgSkeletonContainer::CalculateLoopMassMatrixCoefficients(dgFloat32* const diagDamp)
{
	DG_TRACKTIME();
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dgInt32 index = 0; index < m_auxiliaryRowCount; index ++) {
		const dgInt32 ii = m_matrixRowsIndex[primaryCount + index];
		const dgLeftHandSide* const row_i = &m_leftHandSide[ii];
		const dgRightHandSide* const rhs_i = &m_rightHandSide[ii];
		const dgJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
		const dgJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);
		const dgVector element(
			JMinvM0.m_linear * row_i->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_i->m_Jt.m_jacobianM0.m_angular +
			JMinvM1.m_linear * row_i->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_i->m_Jt.m_jacobianM1.m_angular);
		
		// I know I am doubling the matrix regularizer, but this makes the solution more robust.
		dgFloat32* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * index];
		dgFloat32 diagonal = element.AddHorizontal().GetScalar() + rhs_i->m_diagDamp;
		matrixRow11[index] = diagonal + rhs_i->m_diagDamp;
		//diagDamp[index] = matrixRow11[index] * (DG_PSD_DAMP_TOL * dgFloat32(4.0f));
		diagDamp[index] = matrixRow11[index] * dgFloat32(4.0e-3f);

		const dgInt32 m0_i = m_pairs[primaryCount + index].m_m0;
		const dgInt32 m1_i = m_pairs[primaryCount + index].m_m1;
		for (dgInt32 j = index + 1; j < m_auxiliaryRowCount; j++) {
			const dgInt32 jj = m_matrixRowsIndex[primaryCount + j];
			const dgLeftHandSide* const row_j = &m_leftHandSide[jj];

			const dgInt32 k = primaryCount + j;
			dgVector acc(dgVector::m_zero);
			const dgInt32 m0_j = m_pairs[k].m_m0;
			const dgInt32 m1_j = m_pairs[k].m_m1;
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
				dgFloat32 offDiagValue = acc.GetScalar();
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + index] = offDiagValue;
			}
		}

		dgFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * index];
		for (dgInt32 j = 0; j < primaryCount; j++) {
			const dgInt32 jj = m_matrixRowsIndex[j];
			const dgLeftHandSide* const row_j = &m_leftHandSide[jj];

			const dgInt32 m0_j = m_pairs[j].m_m0;
			const dgInt32 m1_j = m_pairs[j].m_m1;
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
				dgFloat32 val = acc.GetScalar();
				matrixRow10[j] = val;
			}
		}
	}
}

void dgSkeletonContainer::ConditionMassMatrix () const
{
	DG_TRACKTIME();
	dgForcePair* const forcePair = dgAlloca(dgForcePair, m_nodeCount);
//	dgForcePair* const accelPair = dgAlloca(dgForcePair, m_nodeCount);
	const dgSpatialVector zero(dgSpatialVector::m_zero);
//	accelPair[m_nodeCount - 1].m_body = zero;
//	accelPair[m_nodeCount - 1].m_joint = zero;

	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i ++) {
		dgInt32 entry0 = 0;
		dgInt32 startjoint = m_nodeCount;
		const dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		for (dgInt32 j = 0; j < m_nodeCount - 1; j++) {
			const dgNode* const node = m_nodesOrder[j];
			const dgInt32 index = node->m_index;
			//accelPair[index].m_body = zero;
			//dgSpatialVector& a = accelPair[index].m_joint;
			forcePair[index].m_body = zero;
			dgSpatialVector& a = forcePair[index].m_joint;

			const int count = node->m_dof;
			for (dgInt32 k = 0; k < count; k++) {
				const dgFloat32 value = matrixRow10[entry0];
				a[k] = value;
				startjoint = (value == 0.0f) ? startjoint : dgMin(startjoint, index);
				entry0++;
			}
		}

		startjoint = (startjoint == m_nodeCount) ? 0 : startjoint;
		dgAssert(startjoint < m_nodeCount);
		//SolveForward(forcePair, accelPair, startjoint);
		forcePair[m_nodeCount - 1].m_body = zero;
		forcePair[m_nodeCount - 1].m_joint = zero;
		SolveForward(forcePair, forcePair, startjoint);
		SolveBackward(forcePair);

		dgInt32 entry1 = 0;
		dgFloat32* const deltaForcePtr = &m_deltaForce[i * primaryCount];
		for (dgInt32 j = 0; j < m_nodeCount - 1; j++) {
			const dgNode* const node = m_nodesOrder[j];
			const dgInt32 index = node->m_index;
			const dgSpatialVector& f = forcePair[index].m_joint;
			const int count = node->m_dof;
			for (dgInt32 k = 0; k < count; k++) {
				deltaForcePtr[entry1] = dgFloat32(f[k]);
				entry1++;
			}
		}
	}
}

void dgSkeletonContainer::RebuildMassMatrix(const dgFloat32* const diagDamp) const
{
	DG_TRACKTIME();
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	dgInt16* const indexList = dgAlloca(dgInt16, primaryCount);
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i ++) {
		const dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		dgFloat32* const matrixRow11 = &m_massMatrix11[i * m_auxiliaryRowCount];

		dgInt32 indexCount = 0;
		for (dgInt32 k = 0; k < primaryCount; k++) {
			indexList[indexCount] = dgInt16(k);
			indexCount += (matrixRow10[k] != dgFloat32(0.0f)) ? 1 : 0;
		}

		for (dgInt32 j = i; j < m_auxiliaryRowCount; j++) {
			dgFloat32 offDiagonal = matrixRow11[j];
			const dgFloat32* const row10 = &m_deltaForce[j * primaryCount];
			for (dgInt32 k = 0; k < indexCount; k++) {
				dgInt32 index = indexList[k];
				offDiagonal += matrixRow10[index] * row10[index];
			}
			matrixRow11[j] = offDiagonal;
			m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagonal;
		}

		matrixRow11[i] = dgMax(matrixRow11[i], diagDamp[i]);
	}
}

void dgSkeletonContainer::InitLoopMassMatrix(const dgJointInfo* const jointInfoArray)
{
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	dgInt8* const memoryBuffer = CalculateBufferSizeInBytes(jointInfoArray);

	m_frictionIndex = (dgInt32*)memoryBuffer;
	m_matrixRowsIndex = (dgInt32*)&m_frictionIndex[m_rowCount];
	m_pairs = (dgNodePair*)&m_matrixRowsIndex[m_rowCount];
	m_massMatrix11 = (dgFloat32*)&m_pairs[m_rowCount];
	m_massMatrix10 = (dgFloat32*)&m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
	m_deltaForce = &m_massMatrix10[m_auxiliaryRowCount * primaryCount];

	dgFloat32* const diagDamp = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgInt32* const boundRow = dgAlloca(dgInt32, m_auxiliaryRowCount);

	m_blockSize = 0;
	dgInt32 primaryIndex = 0;
	dgInt32 auxiliaryIndex = 0;
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		const dgNode* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;

		const dgInt32 primaryDof = node->m_dof;
		const dgInt32 first = jointInfo->m_pairStart;

		for (dgInt32 j = 0; j < primaryDof; j++) {
			const dgInt32 index = node->m_sourceJacobianIndex[j];
			m_pairs[primaryIndex].m_m0 = m0;
			m_pairs[primaryIndex].m_m1 = m1;
			m_frictionIndex[primaryIndex] = 0;
			m_matrixRowsIndex[primaryIndex] = first + index;
			primaryIndex++;
		}

		const dgInt32 auxiliaryDof = jointInfo->m_pairCount - primaryDof;
		for (dgInt32 j = 0; j < auxiliaryDof; j++) {
			const dgInt32 index = node->m_sourceJacobianIndex[primaryDof + j];
			const dgRightHandSide* const rhs = &m_rightHandSide[first + index];

			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_frictionIndex[auxiliaryIndex + primaryCount] = 0;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + index;
			const dgInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= dgFloat32(-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dgFloat32(DG_LCP_MAX_VALUE)) ? 1 : 0;
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}
	dgAssert (m_loopRowCount == (m_auxiliaryRowCount - auxiliaryIndex));
	
	const dgInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dgInt32 j = 0; j < loopCount; j ++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 auxiliaryDof = jointInfo->m_pairCount;

		for (dgInt32 i = 0; i < auxiliaryDof; i++) {
			const dgRightHandSide* const rhs = &m_rightHandSide[first + i];
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_frictionIndex[auxiliaryIndex + primaryCount] = (rhs->m_normalForceIndex < 0) ? 0 : rhs->m_normalForceIndex - i;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			const dgInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= dgFloat32(-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dgFloat32(DG_LCP_MAX_VALUE)) ? 1 : 0;
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}

	dgAssert(primaryIndex == primaryCount);
	dgAssert(auxiliaryIndex == m_auxiliaryRowCount);

	for (dgInt32 i = 1; i < auxiliaryIndex; i++) {
		dgInt32 j = i;
		dgInt32 tmpBoundRow = boundRow[j];
		dgNodePair tmpPair(m_pairs[primaryCount + j]);
		dgInt32 tmpFrictionIndex = m_frictionIndex[primaryCount + j];
		dgInt32 tmpMatrixRowsIndex = m_matrixRowsIndex[primaryCount + j];

		for (; j && (boundRow[j - 1] < tmpBoundRow); j--) {
			dgAssert(j > 0);
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

	memset(m_massMatrix10, 0, primaryCount * m_auxiliaryRowCount * sizeof(dgFloat32));
	memset(m_massMatrix11, 0, m_auxiliaryRowCount * m_auxiliaryRowCount * sizeof(dgFloat32));

	CalculateLoopMassMatrixCoefficients(diagDamp);
	ConditionMassMatrix ();
	RebuildMassMatrix(diagDamp);

	if (m_blockSize) {
		FactorizeMatrix(m_blockSize, m_auxiliaryRowCount, m_massMatrix11, diagDamp);
		const int boundedSize = m_auxiliaryRowCount - m_blockSize;
		dgFloat32* const acc = dgAlloca(dgFloat32, m_auxiliaryRowCount);
		dgInt32 rowStart = 0;
		for (dgInt32 i = 0; i < m_blockSize; i++) {
			memset(acc, 0, boundedSize * sizeof (dgFloat32));
			const dgFloat32* const row = &m_massMatrix11[rowStart];
			for (dgInt32 j = 0; j < i; j++) {
				const dgFloat32 s = row[j];
				const dgFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (dgInt32 k = 0; k < boundedSize; k++) {
					acc[k] += s * x[k];
				}
			}

			dgFloat32* const x = &m_massMatrix11[rowStart + m_blockSize];
			const dgFloat32 den = -dgFloat32(1.0f) / row[i];
			for (dgInt32 j = 0; j < boundedSize; j++) {
				x[j] = (x[j] + acc[j]) * den;
			}
			rowStart += m_auxiliaryRowCount;
		}

		for (dgInt32 i = m_blockSize - 1; i >= 0; i--) {
			memset(acc, 0, boundedSize * sizeof (dgFloat32));
			for (dgInt32 j = i + 1; j < m_blockSize; j++) {
				const dgFloat32 s = m_massMatrix11[j * m_auxiliaryRowCount + i];
				const dgFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (dgInt32 k = 0; k < boundedSize; k++) {
					acc[k] += s * x[k];
				}
			}

			dgFloat32* const x = &m_massMatrix11[i * m_auxiliaryRowCount + m_blockSize];
			const dgFloat32 den = dgFloat32(1.0f) / m_massMatrix11[i * m_auxiliaryRowCount + i];
			for (dgInt32 j = 0; j < boundedSize; j++) {
				x[j] = (x[j] - acc[j]) * den;
			}
		}

		for (dgInt32 i = 0; i < boundedSize; i++) {
			for (int j = 0; j < m_blockSize; j++) {
				acc[j] = m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize + i];
			}

			dgFloat32* const arow = &m_massMatrix11[(m_blockSize + i) * m_auxiliaryRowCount + m_blockSize];
			for (int j = i; j < boundedSize; j++) {
				const dgFloat32* const row1 = &m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount];
				dgFloat32 elem = row1[m_blockSize + i] + dgDotProduct(m_blockSize, acc, row1);
				arow[j] = elem;
				m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount + m_blockSize + i] = elem;
			}
			arow[i] += diagDamp[m_blockSize + i];
		}
		dgAssert (dgTestPSDmatrix(m_auxiliaryRowCount - m_blockSize, m_auxiliaryRowCount, &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize]));
	}
}

bool dgSkeletonContainer::SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const
{
	return true;
}

DG_INLINE void dgSkeletonContainer::SolveForward(dgForcePair* const force, const dgForcePair* const accel, dgInt32 startNode) const
{
	dgSpatialVector zero (dgSpatialVector::m_zero);
	for (dgInt32 i = 0; i < startNode; i++) {
		force[i].m_body = zero;
		force[i].m_joint = zero;
	}
	for (dgInt32 i = startNode; i < m_nodeCount - 1; i++) {
		dgNode* const node = m_nodesOrder[i];
		dgAssert(node->m_joint);
		dgAssert(node->m_index == i);
		dgForcePair& f = force[i];
		const dgForcePair& a = accel[i];
		f.m_body = a.m_body;
		f.m_joint = a.m_joint;
		for (dgNode* child = node->m_child; child; child = child->m_sibling) {
			dgAssert(child->m_joint);
			dgAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(f);
	}

	force[m_nodeCount - 1] = accel[m_nodeCount - 1];
	for (dgNode* child = m_nodesOrder[m_nodeCount - 1]->m_child; child; child = child->m_sibling) {
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	for (dgInt32 i = startNode; i < m_nodeCount - 1; i++) {
		dgNode* const node = m_nodesOrder[i];
		dgForcePair& f = force[i];
		node->BodyDiagInvTimeSolution(f);
		node->JointDiagInvTimeSolution(f);
	}
	m_nodesOrder[m_nodeCount - 1]->BodyDiagInvTimeSolution(force[m_nodeCount - 1]);
}

DG_INLINE void dgSkeletonContainer::SolveBackward(dgForcePair* const force) const
{
	for (dgInt32 i = m_nodeCount - 2; i >= 0; i--) {
		dgNode* const node = m_nodesOrder[i];
		dgAssert(node->m_index == i);
		dgForcePair& f = force[i];
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyJacobianTimeSolutionBackward(f);
	}
}

DG_INLINE void dgSkeletonContainer::CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const
{
	SolveForward(force, accel);
	SolveBackward(force);
}

DG_INLINE void dgSkeletonContainer::UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const dgForcePair* const force) const
{
	dgVector zero (dgVector::m_zero);
	for (dgInt32 i = 0; i < (m_nodeCount - 1)  ; i ++) {
		dgNode* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		dgAssert(i == node->m_index);

		const dgSpatialVector& f = force[i].m_joint;
		dgAssert(jointInfo->m_joint == node->m_joint);
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = node->m_dof;
		for (dgInt32 j = 0; j < count; j ++) {
			const dgInt32 k = node->m_sourceJacobianIndex[j];
			dgRightHandSide* const rhs = &m_rightHandSide[first + k];
			const dgLeftHandSide* const row = &m_leftHandSide[first + k];

			rhs->m_force += dgFloat32(f[j]);
			dgVector jointForce = dgFloat32 (f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
		}

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}
}

DG_INLINE void dgSkeletonContainer::CalculateJointAccel(dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgForcePair* const accel) const
{
	const dgSpatialVector zero (dgSpatialVector::m_zero);
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dgNode* const node = m_nodesOrder[i];
		dgAssert(i == node->m_index);

		dgForcePair& a = accel[i];
		dgAssert(node->m_body);
		a.m_body = zero;
		a.m_joint = zero;

		dgAssert(node->m_joint);
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		dgAssert(jointInfo->m_joint == node->m_joint);

		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 dof = jointInfo->m_pairCount;
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (dgInt32 j = 0; j < dof; j++) {
			const dgInt32 k = node->m_sourceJacobianIndex[j];
			const dgLeftHandSide* const row = &m_leftHandSide[first + k];
			const dgRightHandSide* const rhs = &m_rightHandSide[first + k];
			dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
						  row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
			a.m_joint[j] = -(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - diag.AddHorizontal().GetScalar());
		}
	}
	dgAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body = zero;
	accel[m_nodeCount - 1].m_joint = zero;
}

void dgSkeletonContainer::SolveLcp(dgInt32 stride, dgInt32 size, const dgFloat32* const matrix, const dgFloat32* const x0, dgFloat32* const x, const dgFloat32* const b, const dgFloat32* const low, const dgFloat32* const high, const dgInt32* const normalIndex) const
{
	D_TRACKTIME();
	if (m_world->GetCurrentPlugin()) {
		dgWorldPlugin* const plugin = m_world->GetCurrentPlugin()->GetInfo().m_plugin;
		plugin->SolveDenseLcp(stride, size, matrix, x0, x, b, low, high, normalIndex);
	} else {
#if 0
		// sequential Sidle iteration
		const dgFloat32 sor = dgFloat32(1.125f);
		const dgFloat32 tol2 = dgFloat32(0.25f);
		const dgInt32 maxIterCount = 64;

		dgFloat32* const invDiag1 = dgAlloca(dgFloat32, size);

		int rowStart = 0;
		for (dgInt32 i = 0; i < size; i++) {
			const int index = normalIndex[i];
			const dgFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : 1.0f;
			const dgFloat32 l = low[i] * coefficient - x0[i];
			const dgFloat32 h = high[i] * coefficient - x0[i];;
			x[i] = dgClamp(dgFloat32(0.0f), l, h);
			invDiag1[i] = dgFloat32(1.0f) / matrix[rowStart + i];
			rowStart += stride;
		}

		dgFloat32 tolerance(tol2 * dgFloat32(2.0f));
		const dgFloat32* const invDiag = invDiag1;

		dgInt32 iterCount = 0;
		for (dgInt32 k = 0; (k < maxIterCount) && (tolerance > tol2); k++) {
			iterCount++;
			dgInt32 base = 0;
			tolerance = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < size; i++) {
				const dgFloat32* const row = &matrix[base];
				dgFloat32 r = b[i] - dgDotProduct(size, row, x);

				const int index = normalIndex[i];
				const dgFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : dgFloat32(1.0f);
				const dgFloat32 l = low[i] * coefficient - x0[i];
				const dgFloat32 h = high[i] * coefficient - x0[i];

				dgFloat32 f = x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor;
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
		const dgFloat32 sor = dgFloat32(1.125f);
		const dgFloat32 tol2 = dgFloat32(0.25f);
		const dgInt32 maxIterCount = 64;

		dgFloat32* const invDiag1 = dgAlloca(dgFloat32, size);
		dgFloat32* const residual = dgAlloca(dgFloat32, size);

		dgInt32 rowStart = 0;
		for (dgInt32 i = 0; i < size; i++) {
			const int index = normalIndex[i];
			const dgFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : 1.0f;
			const dgFloat32 l = low[i] * coefficient - x0[i];
			const dgFloat32 h = high[i] * coefficient - x0[i];;
			x[i] = dgClamp(dgFloat32(0.0f), l, h);
			invDiag1[i] = dgFloat32(1.0f) / matrix[rowStart + i];
			rowStart += stride;
		}

		dgInt32 base = 0;
		for (dgInt32 i = 0; i < size; i++) {
			const dgFloat32* const row = &matrix[base];
			residual[i] = b[i] - dgDotProduct(size, row, x);
			base += stride;
		}

		dgInt32 iterCount = 0;
		dgFloat32 tolerance(tol2 * dgFloat32(2.0f));
		const dgFloat32* const invDiag = invDiag1;
		const dgFloat32 one = dgFloat32(1.0f);
		for (dgInt32 k = 0; (k < maxIterCount) && (tolerance > tol2); k++) {
			base = 0;
			iterCount++;
			tolerance = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < size; i++) {
				const dgFloat32 r = residual[i];
				const int index = normalIndex[i];
				const dgFloat32 coefficient = index ? x[i + index] + x0[i + index] : one;
				const dgFloat32 l = low[i] * coefficient - x0[i];
				const dgFloat32 h = high[i] * coefficient - x0[i];

				const dgFloat32* const row = &matrix[base];
#if 0
				dgFloat32 f = x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor;
				if (f > h) {
					f = h;
				} else if (f < l) {
					f = l;
				} else {
					tolerance += r * r;
				}
				const dgFloat32 dx = f - x[i];
#else
				const dgFloat32 f = dgClamp(x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor, l, h);
				const dgFloat32 dx = f - x[i];
				const dgFloat32 dr = dx * row[i];
				tolerance += dr * dr;
#endif
				x[i] = f;
				if (dgAbs (dx) > dgFloat32 (1.0e-6f)) {
					for (dgInt32 j = 0; j < size; j++) {
						residual[j] -= row[j] * dx;
					}
				}
				base += stride;
			}
		}
#endif
	}
}

void dgSkeletonContainer::SolveBlockLcp(dgInt32 size, dgInt32 blockSize, const dgFloat32* const x0, dgFloat32* const x, dgFloat32* const b, const dgFloat32* const low, const dgFloat32* const high, const dgInt32* const normalIndex) const
{
	if (blockSize) {
		dgSolveCholesky(blockSize, size, m_massMatrix11, x, b);
		if (blockSize != size) {

			dgInt32 base = blockSize * size;
			for (dgInt32 i = blockSize; i < size; i++) {
				b[i] -= dgDotProduct(blockSize, &m_massMatrix11[base], x);
				base += size;
			}

			const int boundedSize = size - blockSize;
			SolveLcp(
				size, boundedSize, &m_massMatrix11[blockSize * size + blockSize],
				&x0[blockSize], &x[blockSize], &b[blockSize], &low[blockSize], &high[blockSize], &normalIndex[blockSize]);

			for (dgInt32 j = 0; j < blockSize; j++) {
				const dgFloat32* const row = &m_massMatrix11[j * size + blockSize];
				dgFloat32 acc = dgFloat32 (0.0f);
				for (dgInt32 i = 0; i < boundedSize; i++) {
					acc += x[blockSize + i] * row[i];
				}
				x[j] += acc;
			}
		}
	} else {
		SolveLcp(size, size, m_massMatrix11, x0, x, b, low, high, normalIndex);
	}
}

void dgSkeletonContainer::SolveAuxiliary(const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const dgForcePair* const accel, dgForcePair* const force) const
{
	dgFloat32* const f = dgAlloca(dgFloat32, m_rowCount);
	dgFloat32* const u = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const b = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const u0 = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const low = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const high = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgInt32* const normalIndex = dgAlloca(dgInt32, m_auxiliaryRowCount);

	dgInt32 primaryIndex = 0;
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		const dgNode* const node = m_nodesOrder[i];
		const dgInt32 primaryDof = node->m_dof;
		const dgSpatialVector& forceSpatial = force[i].m_joint;

		for (dgInt32 j = 0; j < primaryDof; j++) {
			f[primaryIndex] = dgFloat32(forceSpatial[j]);
			primaryIndex++;
		}
	}

	dgAssert (primaryIndex == primaryCount);
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const int index = m_matrixRowsIndex[primaryCount + i];
		const dgLeftHandSide* const row = &m_leftHandSide[index];
		const dgRightHandSide* const rhs = &m_rightHandSide[index];

		const int m0 = m_pairs[primaryCount + i].m_m0;
		const int m1 = m_pairs[primaryCount + i].m_m1;

		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		f[primaryCount + i] = dgFloat32(0.0f);

		dgVector acc(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
					 row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
		b[i] = rhs->m_coordenateAccel - acc.AddHorizontal().GetScalar();

		normalIndex[i] = m_frictionIndex[primaryCount + i];
		u0[i] = rhs->m_force;
		low[i] = rhs->m_lowerBoundFrictionCoefficent;
		high[i] = rhs->m_upperBoundFrictionCoefficent;
	}

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		b[i] -= dgDotProduct(primaryCount, matrixRow10, f);
	}

	SolveBlockLcp(m_auxiliaryRowCount, m_blockSize, u0, u, b, low, high, normalIndex);

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dgFloat32 s = u[i];
		f[primaryCount + i] = s;
		dgMulAdd(primaryCount, f, f, &m_deltaForce[i * primaryCount], s);
	}

	for (dgInt32 i = 0; i < m_rowCount; i++) {
		dgInt32 index = m_matrixRowsIndex[i];
		dgRightHandSide* const rhs = &m_rightHandSide[index];
		const dgLeftHandSide* const row = &m_leftHandSide[index];
		const dgInt32 m0 = m_pairs[i].m_m0;
		const dgInt32 m1 = m_pairs[i].m_m1;

		rhs->m_force += f[i];
		dgVector jointForce(f[i]);
		internalForces[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
		internalForces[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
		internalForces[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
		internalForces[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
	}
}

dgInt8* dgSkeletonContainer::CalculateBufferSizeInBytes (const dgJointInfo* const jointInfoArray)
{
	dgInt32 rowCount = 0;
	dgInt32 auxiliaryRowCount = 0;
	if (m_nodesOrder) {
		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			dgNode* const node = m_nodesOrder[i];
			rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			auxiliaryRowCount += node->GetAuxiliaryRows(jointInfoArray, m_rightHandSide);
		}
	}

	dgInt32 extraAuxiliaryRows = 0;
	const dgInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dgInt32 j = 0; j < loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount+= extraAuxiliaryRows;

	dgInt32 size = sizeof (dgInt32) * rowCount;
	size += sizeof (dgInt32) * rowCount;
	size += sizeof (dgNodePair) * rowCount;
	size += sizeof(dgFloat32) * auxiliaryRowCount * auxiliaryRowCount;		// matrix11[auxiliaryRowCount * auxiliaryRowCount]
	size += sizeof (dgFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof (dgFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size = (size + 1024) & -0x10;
	m_auxiliaryMemoryBuffer.ResizeIfNecessary((size + 1024) & -0x10);
	return &m_auxiliaryMemoryBuffer[0];
}

void dgSkeletonContainer::InitMassMatrix(const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, bool consideredCloseLoop)
{
	D_TRACKTIME();
	dgInt32 rowCount = 0;
	dgInt32 auxiliaryCount = 0;
	m_leftHandSide = leftHandSide;
	m_rightHandSide = rightHandSide;
	m_consideredCloseLoop = consideredCloseLoop ? 1 : 0;

	dgSpatialMatrix* const bodyMassArray = dgAlloca (dgSpatialMatrix, m_nodeCount);
	dgSpatialMatrix* const jointMassArray = dgAlloca (dgSpatialMatrix, m_nodeCount);

	if (m_nodesOrder) {
		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			dgNode* const node = m_nodesOrder[i];
			const dgJointInfo& info = jointInfoArray[node->m_joint->m_index];
			rowCount += info.m_pairCount;
			auxiliaryCount += node->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
		}
		m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
	}
	m_rowCount = dgInt16 (rowCount);
	m_auxiliaryRowCount = dgInt16 (auxiliaryCount);

	dgInt32 loopRowCount = 0;
	const dgInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dgInt32 j = 0; j < loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		loopRowCount += jointInfoArray[joint->m_index].m_pairCount;
	}
	m_loopRowCount = dgInt16 (loopRowCount);
	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;

	if (m_auxiliaryRowCount && m_consideredCloseLoop) {
		InitLoopMassMatrix(jointInfoArray);
	}
}

void dgSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces)
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

