/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "dStdafxVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleSolver.h"
#include "dVehicleChassis.h"

#define D_DIAG_DAMP			 (1.0e-4f)
#define D_MAX_FRICTION_BOUND (D_COMPLEMENTARITY_MAX_FRICTION_BOUND * 0.5f)

class dMatrixData
{
	public:
	dSpatialMatrix m_jt;
	dSpatialMatrix m_mass;
	dSpatialMatrix m_invMass;
};

class dVehicleSolver::dVectorPair
{
	public:
	dSpatialVector m_body;
	dSpatialVector m_joint;
};

class dVehicleSolver::dBodyJointMatrixDataPair
{
	public:
	dMatrixData m_body;
	dMatrixData m_joint;
};

dVehicleSolver::dVehicleSolver()
	:dContainersAlloc()
	,m_vehicle(NULL)
	,m_nodesOrder(NULL)
{
}

dVehicleSolver::~dVehicleSolver()
{
	if (m_nodesOrder) {
		delete[] m_nodesOrder; 
	}
}

/*
class dgSkeletonContainer::dgNodePair
{
public:
	dgInt32 m_m0;
	dgInt32 m_m1;
};

DG_MSC_VECTOR_ALIGMENT
class dgSkeletonContainer::dVectorPair
{
public:
	dgSpatialVector m_joint;
	dgSpatialVector m_body;
} DG_GCC_VECTOR_ALIGMENT;



class dgSkeletonContainer::dgNode
{
public:

	DG_CLASS_ALLOCATOR(allocator)
		dgNode(dgDynamicBody* const body)
		:m_body(body)
		, m_joint(NULL)
		, m_parent(NULL)
		, m_child(NULL)
		, m_sibling(NULL)
		, m_index(0)
		, m_dof(0)
		, m_swapJacobianBodiesIndex(0)
	{
		}

	dgNode(dgBilateralConstraint* const joint, dgNode* const parent)
		:m_body((dgDynamicBody*)((joint->GetBody0() == parent->m_body) ? joint->GetBody1() : joint->GetBody0()))
		, m_joint(joint)
		, m_parent(parent)
		, m_child(NULL)
		, m_sibling(NULL)
		, m_index(0)
		, m_dof(0)
		, m_swapJacobianBodiesIndex(joint->GetBody0() == parent->m_body)
	{
		dgAssert(m_parent);
		dgAssert(m_body->GetInvMass().m_w != dgFloat32(0.0f));
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

	DG_INLINE void JointJacobianTimeSolutionBackward(dVectorPair& force, const dVectorPair& parentForce)
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		const dgSpatialVector& f = parentForce.m_body;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= f.DotProduct(jointJ[i]);
		}
	}

	DG_INLINE void BodyJacobianTimeSolutionBackward(dVectorPair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_body = force.m_body + bodyJt[i].Scale(-force.m_joint[i]);
		}
	}

	DG_INLINE void BodyDiagInvTimeSolution(dVectorPair& force)
	{
		const dgSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
		force.m_body = bodyInvMass.VectorTimeMatrix(force.m_body);
	}

	DG_INLINE void JointDiagInvTimeSolution(dVectorPair& force)
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
				if (!((rhs->m_lowerBoundFrictionCoefficent <= dgFloat32(-DG_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dgFloat32(DG_LCP_MAX_VALUE)))) {
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
, m_skeleton(new (world->GetAllocator()) dgNode(rootBody))
, m_nodesOrder(NULL)
, m_pairs(NULL)
, m_deltaForce(NULL)
, m_massMatrix11(NULL)
, m_massMatrix10(NULL)
, m_rightHandSide(NULL)
, m_leftHandSide(NULL)
, m_matrixRowsIndex(NULL)
, m_loopingJoints(world->GetAllocator())
, m_auxiliaryMemoryBuffer(world->GetAllocator())
, m_id(m_uniqueID)
, m_lru(0)
, m_nodeCount(1)
, m_loopCount(0)
, m_selfContactCount(0)
, m_rowCount(0)
, m_loopRowCount(0)
, m_auxiliaryRowCount(0)
{
	if (rootBody->GetInvMass().m_w != dgFloat32(0.0f)) {
		rootBody->SetSkeleton(this);
	}
	m_uniqueID++;
}

dgSkeletonContainer::~dgSkeletonContainer()
{
	for (dgInt32 i = 0; i < m_loopCount; i++) {
		dgConstraint* const joint = m_loopingJoints[i];
		joint->m_isInSkeleton = false;
	}

	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		m_nodesOrder[i]->m_joint->m_isInSkeleton = false;
	}

	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	if (m_nodesOrder) {
		allocator->Free(m_nodesOrder);
	}

	delete m_skeleton;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::GetRoot() const
{
	return m_skeleton;
}

dgSkeletonContainer::dgNode* dgSkeletonContainer::GetParent(dgNode* const node) const
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

void dgSkeletonContainer::ResetUniqueId(dgInt32 id)
{
	m_uniqueID = id;
}

void dgSkeletonContainer::ClearSelfCollision()
{
	m_selfContactCount = 0;
}

void dgSkeletonContainer::AddSelfCollisionJoint(dgContact* const contact)
{
	m_world->GlobalLock();
	m_loopingJoints[m_loopCount + m_selfContactCount] = contact;
	m_selfContactCount++;
	m_world->GlobalUnlock();
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
	dgAssert(m_skeleton->m_body);
	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	dgNode* const node = new (allocator)dgNode(joint, parent);
	m_nodeCount++;

	joint->m_isInSkeleton = true;
	dgAssert(m_world->GetSentinelBody() != node->m_body);
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


DG_INLINE void dgSkeletonContainer::CalculateLoopMassMatrixCoefficients(dgFloat32* const diagDamp)
{
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dgInt32 ii = m_matrixRowsIndex[primaryCount + i];
		const dgLeftHandSide* const row_i = &m_leftHandSide[ii];
		const dgRightHandSide* const rhs_i = &m_rightHandSide[ii];
		dgFloat32* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * i];

		dgJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
		dgJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);

		dgVector element(JMinvM0.m_linear * row_i->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_i->m_Jt.m_jacobianM0.m_angular +
			JMinvM1.m_linear * row_i->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_i->m_Jt.m_jacobianM1.m_angular);
		element = element.AddHorizontal();

		// I know I am doubling the matrix regularizer, but this makes the solution more robust.
		dgFloat32 diagonal = element.GetScalar() + rhs_i->m_diagDamp;
		matrixRow11[i] = diagonal + rhs_i->m_diagDamp;
		diagDamp[i] = matrixRow11[i] * (DG_PSD_DAMP_TOL * dgFloat32(4.0f));

		const dgInt32 m0_i = m_pairs[primaryCount + i].m_m0;
		const dgInt32 m1_i = m_pairs[primaryCount + i].m_m1;
		for (dgInt32 j = i + 1; j < m_auxiliaryRowCount; j++) {
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
			}
			else if (m0_i == m1_j) {
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}

			if (m1_i == m1_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}
			else if (m1_i == m0_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}

			if (hasEffect) {
				acc = acc.AddHorizontal();
				dgFloat32 offDiagValue = acc.GetScalar();
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagValue;
			}
		}

		dgFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * i];
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
			}
			else if (m0_i == m1_j) {
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}

			if (m1_i == m1_j) {
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}
			else if (m1_i == m0_j) {
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


void dgSkeletonContainer::InitLoopMassMatrix(const dgJointInfo* const jointInfoArray)
{
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	dgInt8* const memoryBuffer = CalculateBufferSizeInBytes(jointInfoArray);

	m_matrixRowsIndex = (dgInt32*)memoryBuffer;
	m_pairs = (dgNodePair*)&m_matrixRowsIndex[m_rowCount];
	m_massMatrix11 = (dgFloat32*)&m_pairs[m_rowCount];
	m_massMatrix10 = (dgFloat32*)&m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
	m_deltaForce = &m_massMatrix10[m_auxiliaryRowCount * primaryCount];

	dVectorPair* const forcePair = dgAlloca(dVectorPair, m_nodeCount);
	dVectorPair* const accelPair = dgAlloca(dVectorPair, m_nodeCount);
	dgFloat32* const diagDamp = dgAlloca(dgFloat32, m_auxiliaryRowCount);

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
			m_matrixRowsIndex[primaryIndex] = first + index;
			primaryIndex++;
		}

		const dgInt32 auxiliaryDof = jointInfo->m_pairCount - primaryDof;
		for (dgInt32 j = 0; j < auxiliaryDof; j++) {
			const dgInt32 index = node->m_sourceJacobianIndex[primaryDof + j];

			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + index;
			auxiliaryIndex++;
		}
	}
	dgAssert(m_loopRowCount == (m_auxiliaryRowCount - auxiliaryIndex));

	const dgInt32 loopCount = m_loopCount + m_selfContactCount;
	for (dgInt32 j = 0; j < loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 auxiliaryDof = jointInfo->m_pairCount;

		for (dgInt32 i = 0; i < auxiliaryDof; i++) {
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			auxiliaryIndex++;
		}
	}
	dgAssert(primaryIndex == primaryCount);
	dgAssert(auxiliaryIndex == m_auxiliaryRowCount);
	memset(m_massMatrix10, 0, primaryCount * m_auxiliaryRowCount * sizeof(dgFloat32));
	memset(m_massMatrix11, 0, m_auxiliaryRowCount * m_auxiliaryRowCount * sizeof(dgFloat32));

	CalculateLoopMassMatrixCoefficients(diagDamp);

	const dgSpatialVector zero(dgSpatialVector::m_zero);
	accelPair[m_nodeCount - 1].m_body = zero;
	accelPair[m_nodeCount - 1].m_joint = zero;

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		dgInt32 entry = 0;
		dgInt32 startjoint = m_nodeCount;
		const dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		for (dgInt32 j = 0; j < m_nodeCount - 1; j++) {
			const dgNode* const node = m_nodesOrder[j];
			const dgInt32 index = node->m_index;
			accelPair[index].m_body = zero;
			dgSpatialVector& a = accelPair[index].m_joint;

			const int count = node->m_dof;
			for (dgInt32 k = 0; k < count; k++) {
				const dgFloat32 value = matrixRow10[entry];
				a[k] = value;
				startjoint = (value == 0.0f) ? startjoint : dgMin(startjoint, index);
				entry++;
			}
		}

		entry = 0;
		startjoint = (startjoint == m_nodeCount) ? 0 : startjoint;
		dgAssert(startjoint < m_nodeCount);
		SolveForward(forcePair, accelPair, startjoint);
		SolveBackward(forcePair, forcePair);

		dgFloat32* const deltaForcePtr = &m_deltaForce[i * primaryCount];
		for (dgInt32 j = 0; j < m_nodeCount - 1; j++) {
			const dgNode* const node = m_nodesOrder[j];
			const dgInt32 index = node->m_index;
			const dgSpatialVector& f = forcePair[index].m_joint;
			const int count = node->m_dof;
			for (dgInt32 k = 0; k < count; k++) {
				deltaForcePtr[entry] = dgFloat32(f[k]);
				entry++;
			}
		}
	}

	dgInt16* const indexList = dgAlloca(dgInt16, primaryCount);
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		const dgFloat32* const deltaForcePtr = &m_deltaForce[i * primaryCount];
		dgFloat32* const matrixRow11 = &m_massMatrix11[i * m_auxiliaryRowCount];

		dgInt32 indexCount = 0;
		for (dgInt32 k = 0; k < primaryCount; k++) {
			indexList[indexCount] = dgInt16(k);
			indexCount += (matrixRow10[k] != dgFloat32(0.0f)) ? 1 : 0;
		}

		dgFloat32 diagonal = matrixRow11[i];
		for (dgInt32 k = 0; k < indexCount; k++) {
			dgInt32 index = indexList[k];
			diagonal += matrixRow10[index] * deltaForcePtr[index];
		}
		matrixRow11[i] = dgMax(diagonal, diagDamp[i]);

		for (dgInt32 j = i + 1; j < m_auxiliaryRowCount; j++) {
			dgFloat32 offDiagonal = matrixRow11[j];
			const dgFloat32* const row10 = &m_deltaForce[j * primaryCount];
			for (dgInt32 k = 0; k < indexCount; k++) {
				dgInt32 index = indexList[k];
				offDiagonal += matrixRow10[index] * row10[index];
			}
			matrixRow11[j] = offDiagonal;
			m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagonal;
		}
	}

	dgCholeskyApplyRegularizer(m_auxiliaryRowCount, m_massMatrix11, diagDamp);
}


bool dgSkeletonContainer::SanityCheck(const dVectorPair* const force, const dVectorPair* const accel) const
{
	return true;
}

DG_INLINE void dgSkeletonContainer::SolveBackward(dVectorPair* const force, const dVectorPair* const accel) const
{
	for (dgInt32 i = m_nodeCount - 2; i >= 0; i--) {
		dgNode* const node = m_nodesOrder[i];
		dgAssert(node->m_index == i);
		dVectorPair& f = force[i];
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyJacobianTimeSolutionBackward(f);
	}
}



DG_INLINE void dgSkeletonContainer::UpdateForces(dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const dVectorPair* const force) const
{
	dgVector zero(dgVector::m_zero);
	for (dgInt32 i = 0; i < (m_nodeCount - 1); i++) {
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
		for (dgInt32 j = 0; j < count; j++) {
			const dgInt32 k = node->m_sourceJacobianIndex[j];
			dgRightHandSide* const rhs = &m_rightHandSide[first + k];
			const dgLeftHandSide* const row = &m_leftHandSide[first + k];

			rhs->m_force += dgFloat32(f[j]);
			dgVector jointForce = dgFloat32(f[j]);
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



void dgSkeletonContainer::SolveAuxiliary(const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const dVectorPair* const accel, dVectorPair* const force) const
{
	dgFloat32* const f = dgAlloca(dgFloat32, m_rowCount);
	dgFloat32* const u = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const b = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const low = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const high = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const massMatrix11 = dgAlloca(dgFloat32, m_auxiliaryRowCount * m_auxiliaryRowCount);

	dgInt32 primaryIndex = 0;
	dgInt32 auxiliaryIndex = 0;
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		const dgNode* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		const dgInt32 first = jointInfo->m_pairStart;

		const dgInt32 primaryDof = node->m_dof;
		const dgSpatialVector& accelSpatial = accel[i].m_joint;
		const dgSpatialVector& forceSpatial = force[i].m_joint;

		for (dgInt32 j = 0; j < primaryDof; j++) {
			f[primaryIndex] = dgFloat32(forceSpatial[j]);
			primaryIndex++;
		}

		const dgInt32 auxiliaryDof = jointInfo->m_pairCount - primaryDof;
		for (dgInt32 j = 0; j < auxiliaryDof; j++) {
			const dgInt32 index = node->m_sourceJacobianIndex[primaryDof + j];
			dgRightHandSide* const rhs = &m_rightHandSide[first + index];
			f[auxiliaryIndex + primaryCount] = dgFloat32(0.0f);
			b[auxiliaryIndex] = -dgFloat32(accelSpatial[primaryDof + j]);
			dgAssert(rhs->m_force >= rhs->m_lowerBoundFrictionCoefficent * dgFloat32(2.0f));
			dgAssert(rhs->m_force <= rhs->m_upperBoundFrictionCoefficent * dgFloat32(2.0f));
			low[auxiliaryIndex] = dgClamp(rhs->m_lowerBoundFrictionCoefficent - rhs->m_force, -DG_MAX_BOUND, dgFloat32(0.0f));
			high[auxiliaryIndex] = dgClamp(rhs->m_upperBoundFrictionCoefficent - rhs->m_force, dgFloat32(0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	for (dgInt32 j = 0; j < m_loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 auxiliaryDof = jointInfo->m_pairCount;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (dgInt32 i = 0; i < auxiliaryDof; i++) {
			const dgRightHandSide* const rhs = &m_rightHandSide[first + i];
			const dgLeftHandSide* const row = &m_leftHandSide[first + i];
			f[auxiliaryIndex + primaryCount] = dgFloat32(0.0f);
			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
				row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
			b[auxiliaryIndex] = rhs->m_coordenateAccel - acc.AddHorizontal().GetScalar();

			dgAssert(rhs->m_force >= rhs->m_lowerBoundFrictionCoefficent * dgFloat32(2.0f));
			dgAssert(rhs->m_force <= rhs->m_upperBoundFrictionCoefficent * dgFloat32(2.0f));
			low[auxiliaryIndex] = dgClamp(rhs->m_lowerBoundFrictionCoefficent - rhs->m_force, -DG_MAX_BOUND, dgFloat32(0.0f));
			high[auxiliaryIndex] = dgClamp(rhs->m_upperBoundFrictionCoefficent - rhs->m_force, dgFloat32(0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	for (dgInt32 j = 0; j < m_selfContactCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[m_loopCount + j];
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 auxiliaryDof = jointInfo->m_pairCount;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 1];
		normalForce[0] = dgFloat32(1.0f);
		for (dgInt32 i = 0; i < auxiliaryDof; i++) {
			const dgLeftHandSide* const row = &m_leftHandSide[first + i];
			const dgRightHandSide* const rhs = &m_rightHandSide[first + i];

			f[auxiliaryIndex + primaryCount] = dgFloat32(0.0f);
			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
				row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
			b[auxiliaryIndex] = rhs->m_coordenateAccel - acc.AddHorizontal().GetScalar();

			dgAssert(rhs->m_normalForceIndex >= -1);
			dgAssert(rhs->m_normalForceIndex <= auxiliaryDof);
			dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;
			dgFloat32 frictionNormal = normalForce[frictionIndex];
			dgFloat32 lowerFrictionForce = frictionNormal * rhs->m_lowerBoundFrictionCoefficent;
			dgFloat32 upperFrictionForce = frictionNormal * rhs->m_upperBoundFrictionCoefficent;

			//dgAssert(rhs->m_force >= rhs->m_lowerBoundFrictionCoefficent * dgFloat32(2.0f));
			//dgAssert(rhs->m_force <= row->m_upperBoundFrictionCoefficent * dgFloat32(2.0f));
			//low[auxiliaryIndex] = dgClamp(rhs->m_lowerBoundFrictionCoefficent - rhs->m_force, -DG_MAX_BOUND, dgFloat32(0.0f));
			//high[auxiliaryIndex] = dgClamp(rhs->m_upperBoundFrictionCoefficent - rhs->m_force, dgFloat32(0.0f), DG_MAX_BOUND);
			low[auxiliaryIndex] = dgClamp(lowerFrictionForce - rhs->m_force, -DG_MAX_BOUND, dgFloat32(0.0f));
			high[auxiliaryIndex] = dgClamp(upperFrictionForce - rhs->m_force, dgFloat32(0.0f), DG_MAX_BOUND);

			normalForce[i + 1] = rhs->m_force;
			auxiliaryIndex++;
		}
	}

	memcpy(massMatrix11, m_massMatrix11, sizeof (dgFloat32)* m_auxiliaryRowCount * m_auxiliaryRowCount);
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		//u[i] = dgFloat32(0.0f);
		dgFloat32 r = dgFloat32(0.0f);
		for (dgInt32 j = 0; j < primaryCount; j++) {
			r += matrixRow10[j] * f[j];
		}
		b[i] -= r;
	}

	//dgSolveDantzigLCP(m_auxiliaryRowCount, massMatrix11, lowerTriangularMassMatrix11, u, b, low, high);
	dgSolveDantzigLCP(m_auxiliaryRowCount, massMatrix11, u, b, low, high);

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dgFloat32 s = u[i];
		f[primaryCount + i] = s;
		const dgFloat32* const deltaForce = &m_deltaForce[i * primaryCount];
		for (dgInt32 j = 0; j < primaryCount; j++) {
			f[j] += deltaForce[j] * s;
		}
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


dgInt8* dgSkeletonContainer::CalculateBufferSizeInBytes(const dgJointInfo* const jointInfoArray)
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
	const dgInt32 loopCount = m_loopCount + m_selfContactCount;
	for (dgInt32 j = 0; j < loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount += extraAuxiliaryRows;

	//	dgInt32 size = sizeof (dgLeftHandSide*) * rowCount;
	//	size += sizeof (dgRightHandSide*) * rowCount;
	dgInt32 size = sizeof (dgInt32)* rowCount;
	size += sizeof (dgNodePair)* rowCount;
	size += sizeof(dgFloat32)* auxiliaryRowCount * auxiliaryRowCount;		// matrix11[auxiliaryRowCount * auxiliaryRowCount]
	//	size += sizeof (dgFloat32) * auxiliaryRowCount * auxiliaryRowCount;		// matrixLowerTraingular [auxiliaryRowCount * auxiliaryRowCount]
	size += sizeof (dgFloat32)* auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof (dgFloat32)* auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size = (size + 1024) & -0x10;
	m_auxiliaryMemoryBuffer.ResizeIfNecessary((size + 1024) & -0x10);
	return &m_auxiliaryMemoryBuffer[0];
}


void dgSkeletonContainer::InitMassMatrix(const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide)
{
	dgInt32 rowCount = 0;
	dgInt32 auxiliaryCount = 0;
	m_leftHandSide = leftHandSide;
	m_rightHandSide = rightHandSide;

	dgSpatialMatrix* const bodyMassArray = dgAlloca(dgSpatialMatrix, m_nodeCount);
	dgSpatialMatrix* const jointMassArray = dgAlloca(dgSpatialMatrix, m_nodeCount);

	if (m_nodesOrder) {
		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			dgNode* const node = m_nodesOrder[i];
			const dgJointInfo& info = jointInfoArray[node->m_joint->m_index];
			rowCount += info.m_pairCount;
			auxiliaryCount += node->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
		}
		m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
	}
	m_rowCount = dgInt16(rowCount);
	m_auxiliaryRowCount = dgInt16(auxiliaryCount);

	dgInt32 loopRowCount = 0;
	const dgInt32 loopCount = m_loopCount + m_selfContactCount;
	for (dgInt32 j = 0; j < loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		loopRowCount += jointInfoArray[joint->m_index].m_pairCount;
	}
	m_loopRowCount = dgInt16(loopRowCount);
	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;

	if (m_auxiliaryRowCount) {
		InitLoopMassMatrix(jointInfoArray);
	}
}

void dgSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces)
{
	dVectorPair* const force = dgAlloca(dVectorPair, m_nodeCount);
	dVectorPair* const accel = dgAlloca(dVectorPair, m_nodeCount);

	CalculateJointAccel(jointInfoArray, internalForces, accel);
	CalculateForce(force, accel);
	if (m_auxiliaryRowCount) {
		SolveAuxiliary(jointInfoArray, internalForces, accel, force);
	}
	else {
		UpdateForces(jointInfoArray, internalForces, force);
	}
}
*/

int dVehicleSolver::CalculateNodeCount () const
{
	dVehicleNode* pool[256];
	int count = 0;
	int stack = 1;

	pool[0] = m_vehicle->GetVehicle();
	while (stack) {
		stack --;
		count ++;
		dVehicleNode* const root = pool[stack];
		for (dList<dVehicleNode*>::dListNode* node = root->m_children.GetFirst(); node; node = node->GetNext()) {
			pool[stack] = node->GetInfo();
			stack ++;
			dAssert (stack < sizeof (pool) / sizeof (pool[0]));
		}
	}
	return count;
}

void dVehicleSolver::SortGraph(dVehicleNode* const root, int& index)
{
	for (dList<dVehicleNode*>::dListNode* node = root->m_children.GetFirst(); node; node = node->GetNext()) {
		SortGraph(node->GetInfo(), index);
	}

	dAssert((m_nodeCount - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_solverIndex = index;
	index++;
	dAssert(index <= m_nodeCount);
}

void dVehicleSolver::Finalize(dVehicleChassis* const vehicle)
{
	m_vehicle = vehicle;
	dVehicleNode* const root = m_vehicle->GetVehicle();
	if (!root->m_children.GetCount()) {
		return ;
	}

	if (m_nodesOrder) {
		delete m_nodesOrder;
	}

	//const dgDynamicBody* const rootBody = m_skeleton->m_body;
	//dgAssert(((rootBody->GetInvMass().m_w == dgFloat32(0.0f)) && (m_skeleton->m_child->m_sibling == NULL)) || (m_skeleton->m_body->GetInvMass().m_w != dgFloat32(0.0f)));

	m_nodeCount = CalculateNodeCount ();
	m_nodesOrder = new dVehicleNode*[m_nodeCount * sizeof (dVehicleNode*)];

	int index = 0;
	SortGraph(root, index);
	dAssert(index == m_nodeCount);

	const dList<dComplementaritySolver::dBilateralJoint*>& loopJoints = m_vehicle->m_loopJoints;
	if (loopJoints.GetCount()) {
		dAssert (0);
/*
		//dgTree<dgInt32, dgDynamicBody*> filter (allocator);
		for (dgInt32 i = 0; i < loopJointsCount; i++) {
			dgBilateralConstraint* const joint = loopJointArray[i];
			dgAssert(joint->GetBody0()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert(joint->GetBody1()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert((FindNode((dgDynamicBody*)joint->GetBody0()) || FindNode((dgDynamicBody*)joint->GetBody1())));
			joint->m_isInSkeleton = true;

			//dgDynamicBody* const body0 = (dgDynamicBody*)joint->GetBody0();
			//dgDynamicBody* const body1 = (dgDynamicBody*)joint->GetBody1();
			//dgNode* const node0 = FindNode(body0);
			//dgNode* const node1 = FindNode(body1);
			//if (node0) {
			//	filter.Insert(node0->m_index, node0->m_body);
			//}
			//if (node1) {
			//	filter.Insert(node1->m_index, node1->m_body);
			//}
			//dgTree<dgInt32, dgDynamicBody*>::dgTreeNode* index0 = filter.Find(body0);
			//if (!index0) {
			//	index0 = filter.Insert(-1, body0);
			//}
			//dgTree<dgInt32, dgDynamicBody*>::dgTreeNode* index1 = filter.Find(body1);
			//if (!index1) {
			//	index1 = filter.Insert(-1, body1);
			//}
			m_loopingJoints[m_loopCount] = joint;
			m_loopCount++;
		}

		//for (dgInt32 i = 0; i < m_nodeCount; i++) {
		//	filter.Remove(m_nodesOrder[i]->m_body);
		//}
		//dgTree<dgDynamicBody*, dgInt32> bodyOrder (allocator);
		//dgTree<dgInt32, dgDynamicBody*>::Iterator iter(filter);
		//for (iter.Begin(); iter; iter ++) {
		//	bodyOrder.Insert (iter.GetKey(), iter.GetNode()->GetInfo());
		//}
*/
	}
}

void dVehicleSolver::CalculateInertiaMatrix(dVehicleNode* const node) const
{
//	dSpatialMatrix* const bodyMassArray) const

	dSpatialMatrix& bodyMass = m_data[node->m_solverIndex].m_body.m_mass;

	bodyMass = dSpatialMatrix(dFloat32(0.0f));
	dComplementaritySolver::dBodyState* const body = node->GetBody();

	dAssert (body->GetInvMass() != dFloat32(0.0f));

	const dFloat32 mass = body->GetMass();
	const dMatrix& inertia = body->GetInertia();

	for (int i = 0; i < 3; i++) {
		bodyMass[i][i] = mass;
		for (int j = 0; j < 3; j++) {
			bodyMass[i + 3][j + 3] = inertia[i][j];
		}
	}
}

void dVehicleSolver::GetJacobians(dVehicleNode* const node)
{
//const dgJointInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgSpatialMatrix* const jointMassArray

	dAssert(node->m_parent);
	dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
	//dgAssert(jointInfo->m_joint == m_joint);

	//dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	//dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	//dgSpatialMatrix& jointMass = jointMassArray[m_index];

	const int index = node->m_solverIndex;
	dSpatialMatrix& bodyJt = m_data[index].m_body.m_jt;
	dSpatialMatrix& jointJ = m_data[index].m_joint.m_jt;
	dSpatialMatrix& jointMass = m_data[index].m_joint.m_mass;
	const int start = joint->m_start;
	const dSpatialVector zero(0.0f);
	const dVector negOne (-1.0f);
	for (int i = 0; i < joint->m_dof; i++) {
		const int k = joint->m_sourceJacobianIndex[i];
		//const dgRightHandSide* const rhs = &rightHandSide[start + k];
		//const dgLeftHandSide* const row = &leftHandSide[start + k];
		const dComplementaritySolver::dJacobianColum* const rhs = &m_rightHandSide[start + k];
		const dComplementaritySolver::dJacobianPair* const row = &m_leftHandSide[start + k];

		jointMass[i] = zero;
		jointMass[i][i] = -rhs->m_diagDamp;
		bodyJt[i] = dSpatialVector(row->m_jacobian_IM0.m_linear * negOne, row->m_jacobian_IM0.m_angular * negOne);
		jointJ[i] = dSpatialVector(row->m_jacobian_IM1.m_linear * negOne, row->m_jacobian_IM1.m_angular * negOne);
	}
}

void dVehicleSolver::CalculateBodyDiagonal(dVehicleNode* const child)
{
//, dgSpatialMatrix* const bodyMassArray, const dgSpatialMatrix* const jointMassArray
	dComplementaritySolver::dBilateralJoint* const joint = child->GetJoint();
	dAssert(joint);

	dSpatialMatrix copy(dFloat(0.0f));
	const int dof = joint->m_dof;
	const int index = child->m_solverIndex;
//	const dSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
//	const dgSpatialMatrix& childDiagonal = jointMassArray[child->m_index];
	const dSpatialMatrix& jacobianMatrix = m_data[index].m_joint.m_jt;
	const dSpatialMatrix& childDiagonal = m_data[index].m_joint.m_mass;
	for (int i = 0; i < dof; i++) {
		const dSpatialVector& jacobian = jacobianMatrix[i];
		for (int j = 0; j < dof; j++) {
			//dAssert(dgAreEqual(dFloat(childDiagonal[i][j]), dgFloat64(childDiagonal[j][i]), dgFloat64(1.0e-5f)));
			dFloat val = childDiagonal[i][j];
			copy[j] = copy[j] + jacobian.Scale(val);
		}
	}

//	dSpatialMatrix& bodyMass = bodyMassArray[m_index];
	dSpatialMatrix& bodyMass = m_data[index].m_body.m_mass;
	for (int i = 0; i < dof; i++) {
		const dSpatialVector& Jacobian = copy[i];
		const dSpatialVector& JacobianTranspose = jacobianMatrix[i];
		for (int j = 0; j < 6; j++) {
			dFloat val = -Jacobian[j];
			bodyMass[j] = bodyMass[j] + JacobianTranspose.Scale(val);
		}
	}
}

void dVehicleSolver::CalculateJointDiagonal(dVehicleNode* const node)
{
//	const dgSpatialMatrix* const bodyMassArray, dgSpatialMatrix* const jointMassArray
	dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();

	const int index = node->m_solverIndex;
//	const dSpatialMatrix& bodyMass = m_bodyMassArray[nodeIndex];
//	const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	const dSpatialMatrix& bodyMass = m_data[index].m_body.m_mass;
	const dSpatialMatrix& bodyJt = m_data[index].m_body.m_jt;

	dSpatialMatrix tmp;
	for (int i = 0; i < joint->m_dof; i++) {
		tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
	}

//	dSpatialMatrix& jointMass = m_jointMassArray[nodeIndex];
	dSpatialMatrix& jointMass = m_data[index].m_joint.m_mass;
	for (int i = 0; i < joint->m_dof; i++) {
		dFloat a = bodyJt[i].DotProduct(tmp[i]);
		jointMass[i][i] -= a;
		for (int j = i + 1; j < joint->m_dof; j++) {
			a = -bodyJt[i].DotProduct(tmp[j]);
			jointMass[i][j] = a;
			jointMass[j][i] = a;
		}
	}

//	dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	dSpatialMatrix& jointInvMass = m_data[index].m_joint.m_invMass;
	jointInvMass = jointMass.Inverse(joint->m_dof);
}

void dVehicleSolver::CalculateJacobianBlock(dVehicleNode* const node)
{
	dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
	const int index = node->m_solverIndex;
//	dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	dSpatialMatrix& jointJ = m_data[index].m_joint.m_jt;

	dSpatialMatrix copy;
	const dSpatialVector zero(0.0f);
	for (int i = 0; i < joint->m_dof; i++) {
		copy[i] = jointJ[i];
		jointJ[i] = zero;
	}

//	const dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	const dSpatialMatrix& jointInvMass = m_data[index].m_joint.m_invMass;
	for (int i = 0; i < joint->m_dof; i++) {
		const dSpatialVector& jacobian = copy[i];
		const dSpatialVector& invDiagonalRow = jointInvMass[i];
		for (int j = 0; j < joint->m_dof; j++) {
			dFloat val = invDiagonalRow[j];
			jointJ[j] = jointJ[j] + jacobian.Scale(val);
		}
	}
}


int dVehicleSolver::Factorize(dVehicleNode* const node)
{
//	const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgSpatialMatrix* const bodyMassArray, dgSpatialMatrix* const jointMassArray
	CalculateInertiaMatrix(node);

	int boundedDof = 0;

	dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
	//dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
	//dAssert (joint);
	if (joint) {
		joint->m_ordinals = 0x050403020100ll;
		dAssert(node->m_parent);
		//const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
		//dgAssert(jointInfo->m_joint == m_joint);
		joint->m_dof = 0;
//		dgInt32 count = joint->m_pairCount;
//		const dgInt32 first = jointInfo->m_pairStart;
		int count = joint->m_count;
		int first = joint->m_start;
		for (int i = 0; i < count; i++) {
			int k = joint->m_sourceJacobianIndex[i];
			const dComplementaritySolver::dJacobianColum* const rhs = &m_rightHandSide[k + first];

			if ((rhs->m_jointLowFriction <= dFloat(-D_MAX_FRICTION_BOUND)) && (rhs->m_jointHighFriction >= dFloat(D_MAX_FRICTION_BOUND))) {
				joint->m_dof++;
			} else {
				dSwap(joint->m_sourceJacobianIndex[i], joint->m_sourceJacobianIndex[count - 1]);
				i--;
				count--;
			}
		}

		dAssert(joint->m_dof > 0);
		dAssert(joint->m_dof <= 6);
		boundedDof += joint->m_count - count;
		//GetJacobians(jointInfo, leftHandSide, rightHandSide, jointMassArray);
		GetJacobians(node);
	}

	const int nodeIndex = node->m_solverIndex;
	dComplementaritySolver::dBodyState* const body = node->GetBody();

	//dSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	//const dgSpatialMatrix& bodyMass = bodyMassArray[m_index];
	const dSpatialMatrix& bodyMass = m_data[nodeIndex].m_body.m_mass;
	dSpatialMatrix& bodyInvMass = m_data[nodeIndex].m_body.m_invMass;
	if (body->GetInvMass() != dFloat32(0.0f)) {
		const dList<dVehicleNode*>& children = node->GetChildren();
		//for (dgNode* child = m_child; child; child = child->m_sibling) {
		for (dList<dVehicleNode*>::dListNode* childNode = children.GetFirst(); childNode; childNode = childNode->GetNext()) {
			//CalculateBodyDiagonal(child, bodyMassArray, jointMassArray);
			CalculateBodyDiagonal(childNode->GetInfo());
		}
		bodyInvMass = bodyMass.Inverse(6);
	} else {
		bodyInvMass = dSpatialMatrix(0.0f);
	}

	if (joint) {
		//dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		dSpatialMatrix& bodyJt = m_data[nodeIndex].m_body.m_jt;
		dAssert(node->m_parent);
		for (int i = 0; i < joint->m_dof; i++) {
			bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
		}
		//CalculateJointDiagonal(bodyMassArray, jointMassArray);
		CalculateJointDiagonal(node);
		CalculateJacobianBlock(node);
	}
	return boundedDof;
}

void dVehicleSolver::InitMassMatrix()
{
//	const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide

//	dgInt32 rowCount = 0;
	
//	m_leftHandSide = leftHandSide;
//	m_rightHandSide = rightHandSide;
//	dgSpatialMatrix* const bodyMassArray = dgAlloca(dgSpatialMatrix, m_nodeCount);
//	dgSpatialMatrix* const jointMassArray = dgAlloca(dgSpatialMatrix, m_nodeCount);

	int auxiliaryCount = 0;

	dAssert (m_nodesOrder);
	for (int i = 0; i < m_nodeCount - 1; i++) {
		dVehicleNode* const node = m_nodesOrder[i];
		//const dgJointInfo& info = jointInfoArray[node->m_joint->m_index];
//		rowCount += info.m_pairCount;
//		auxiliaryCount += node->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
		auxiliaryCount += Factorize(node);
	}
	//m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
	Factorize(m_nodesOrder[m_nodeCount - 1]);
	
/*
	m_rowCount = dgInt16(rowCount);
	m_auxiliaryRowCount = dgInt16(auxiliaryCount);

	dgInt32 loopRowCount = 0;
	const dgInt32 loopCount = m_loopCount + m_selfContactCount;
	for (dgInt32 j = 0; j < loopCount; j++) {
		const dgConstraint* const joint = m_loopingJoints[j];
		loopRowCount += jointInfoArray[joint->m_index].m_pairCount;
	}
	m_loopRowCount = dgInt16(loopRowCount);
	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;

	if (m_auxiliaryRowCount) {
		InitLoopMassMatrix(jointInfoArray);
	}
*/
}

int dVehicleSolver::BuildJacobianMatrix(dFloat timestep)
{
//	int jointCount, dBilateralJoint** const jointArray, dFloat timestep, dJacobianPair* const jacobianArray, dJacobianColum* const jacobianColumnArray, int maxRowCount

	int rowCount = 0;

	dComplementaritySolver::dParamInfo constraintParams;
	constraintParams.m_timestep = timestep;
	constraintParams.m_timestepInv = 1.0f / timestep;

	// calculate Jacobian derivative for each active joint	
	//for (int j = 0; j < jointCount; j++) {
	for (int j = 0; j < m_nodeCount - 1; j++) {
		dVehicleNode* const node = m_nodesOrder[j];
		dAssert (node && node->GetJoint());
		dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();

		constraintParams.m_count = 0;
		joint->JacobianDerivative(&constraintParams);

		int dofCount = constraintParams.m_count;
		joint->m_count = dofCount;
		joint->m_start = rowCount;

		// complete the derivative matrix for this joint
		const int index = joint->m_start;
		dComplementaritySolver::dBodyState* const state0 = joint->m_state0;
		dComplementaritySolver::dBodyState* const state1 = joint->m_state1;
		const dMatrix& invInertia0 = state0->GetInvInertia();
		const dMatrix& invInertia1 = state1->GetInvInertia();
		dFloat invMass0 = state0->GetInvMass();
		dFloat invMass1 = state1->GetInvMass();
//		dFloat weight = 0.9f;
		for (int i = 0; i < dofCount; i++) {
			//dJacobianPair* const row = &jacobianArray[index];
			dComplementaritySolver::dJacobianPair& row = m_leftHandSide[index + i];
			//dJacobianColum* const col = &jacobianColumnArray[index];
			dComplementaritySolver::dJacobianColum& col = m_rightHandSide[index + i];
			//jacobianArray[rowCount] = constraintParams.m_jacobians[i];
			row = constraintParams.m_jacobians[i];

			dVector J01MinvJ01linear(row.m_jacobian_IM0.m_linear.Scale(invMass0) * row.m_jacobian_IM0.m_linear);
			dVector J10MinvJ10linear(row.m_jacobian_IM1.m_linear.Scale(invMass1) * row.m_jacobian_IM1.m_linear);
			dVector J01MinvJ01angular(invInertia0.RotateVector(row.m_jacobian_IM0.m_angular) * row.m_jacobian_IM0.m_angular);
			dVector J10MinvJ10angular(invInertia1.RotateVector(row.m_jacobian_IM1.m_angular) * row.m_jacobian_IM1.m_angular);
			dVector tmpDiag(J01MinvJ01linear + J10MinvJ10linear + J01MinvJ01angular + J10MinvJ10angular);

			//col->m_diagDamp = 1.0f;
			//col->m_coordenateAccel = constraintParams.m_jointAccel[i];
			//col->m_jointLowFriction = constraintParams.m_jointLowFriction[i];
			//col->m_jointHighFriction = constraintParams.m_jointHighFriction[i];
			//col->m_deltaAccel = extenalAcceleration;
			//col->m_coordenateAccel += extenalAcceleration;
			//col->m_force = joint->m_jointFeebackForce[i] * weight;
			//dFloat stiffness = COMPLEMENTARITY_PSD_DAMP_TOL * col->m_diagDamp;
			//dFloat diag = (tmpDiag[0] + tmpDiag[1] + tmpDiag[2]);
			//dAssert(diag > dFloat(0.0f));
			//col->m_diagDamp = diag * stiffness;
			//diag *= (dFloat(1.0f) + stiffness);
			//col->m_invDJMinvJt = dFloat(1.0f) / diag;

			col.m_diagDamp = (tmpDiag.m_x + tmpDiag.m_y + tmpDiag.m_z) * D_DIAG_DAMP;
			col.m_coordenateAccel = constraintParams.m_jointAccel[i];
			col.m_jointLowFriction = constraintParams.m_jointLowFriction[i];
			col.m_jointHighFriction = constraintParams.m_jointHighFriction[i];
		}
		rowCount += dofCount;
	}
	return rowCount;
}


void dVehicleSolver::Update(dFloat timestep)
{
	dVehicleNode* const root = m_vehicle->GetVehicle();
	if (!root->m_children.GetCount()) {
		return;
	}

//	m_bodyJt = dAlloca(dSpatialMatrix, m_nodeCount);
//	m_jointJ = dAlloca(dSpatialMatrix, m_nodeCount);
//	m_bodyInvMass = dAlloca(dSpatialMatrix, m_nodeCount);
//	m_jointInvMass = dAlloca(dSpatialMatrix, m_nodeCount);
//	m_bodyMassArray = dAlloca(dSpatialMatrix, m_nodeCount);
//	m_jointMassArray = dAlloca(dSpatialMatrix, m_nodeCount);
//int xxx = sizeof (dBodyJointMatrixDataPair);
	m_data = dAlloca(dBodyJointMatrixDataPair, m_nodeCount);
	m_leftHandSide = dAlloca(dComplementaritySolver::dJacobianPair, m_nodeCount * 6);
	m_rightHandSide = dAlloca(dComplementaritySolver::dJacobianColum, m_nodeCount * 6);

	m_vehicle->InitRigiBody(timestep);
	m_rowsCount = BuildJacobianMatrix(timestep);
	InitMassMatrix();

	CalculateJointForce();
}


void dVehicleSolver::CalculateJointAccel(dVectorPair* const accel) const
{
//dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dVectorPair* const accel

	const dSpatialVector zero(0.0f);
	for (int i = 0; i < m_nodeCount - 1; i++) {
		dVehicleNode* const node = m_nodesOrder[i];
		dAssert(i == node->m_solverIndex);

		dVectorPair& a = accel[i];
		dAssert(node->GetBody());
		dAssert(node->GetJoint());

		a.m_body = zero;
		a.m_joint = zero;
		//const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		//dgAssert(jointInfo->m_joint == node->m_joint);

		const dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
		const int first = joint->m_start;
		const int dof = joint->m_count;
		//const int m0 = jointInfo->m_m0;
		//const int m1 = jointInfo->m_m1;
//		const int m0 = node->m_solverIndex;
//		const int m1 = node->m_parent->m_solverIndex;
//		const dgJacobian& y0 = internalForces[m0];
//		const dgJacobian& y1 = internalForces[m1];
		for (int j = 0; j < dof; j++) {
			const int k = joint->m_sourceJacobianIndex[j];
			dComplementaritySolver::dJacobianColum& col = m_rightHandSide[first + k];
			//const dgLeftHandSide* const row = &m_leftHandSide[first + k];
			//const dgRightHandSide* const rhs = &m_rightHandSide[first + k];
			//dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
			//			  row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
			//a.m_joint[j] = -(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - diag.AddHorizontal().GetScalar());
			a.m_joint[j] = col.m_coordenateAccel;
		}
		
		dComplementaritySolver::dBodyState* const body = node->GetBody();
		const dVector& force = body->GetForce(); 
		const dVector& torque = body->GetTorque(); 
		for (int j = 0; j < 3; j++) {
			a.m_body[j + 0] = force[j];
			a.m_body[j + 3] = torque[j];
		}
	}

	const int n = m_nodeCount - 1;
	dAssert(n == m_nodesOrder[n]->m_solverIndex);
	dVehicleNode* const node = m_nodesOrder[n];
	dComplementaritySolver::dBodyState* const body = node->GetBody();	
	const dVector& force = body->GetForce();
	const dVector& torque = body->GetTorque();
	dVectorPair& a = accel[n];
	a.m_joint = zero;
	for (int j = 0; j < 3; j++) {
		a.m_body[j + 0] = force[j];
		a.m_body[j + 3] = torque[j];
	}
}

void dVehicleSolver::BodyJacobianTimeMassForward(dVehicleNode* const node, const dVectorPair& force, dVectorPair& parentForce) const
{
//	const dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	const dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
	const dSpatialMatrix& jointJ = m_data[node->m_solverIndex].m_joint.m_jt;
	for (int i = 0; i < joint->m_dof; i++) {
		parentForce.m_body = parentForce.m_body + jointJ[i].Scale(-force.m_joint[i]);
	}
}

void dVehicleSolver::JointJacobianTimeMassForward(dVehicleNode* const node, dVectorPair& force) const
{
//	const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	const dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
	const dSpatialMatrix& bodyJt = m_data[node->m_solverIndex].m_body.m_jt;
	for (int i = 0; i < joint->m_dof; i++) {
		force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
	}
}


void dVehicleSolver::SolveForward(dVectorPair* const force, const dVectorPair* const accel, int startNode) const
{
	dSpatialVector zero(0.0f);
	for (int i = 0; i < startNode; i++) {
		force[i].m_body = zero;
		force[i].m_joint = zero;
	}

	for (int i = startNode; i < m_nodeCount - 1; i++) {
		//dgNode* const node = m_nodesOrder[i];
		dVehicleNode* const node = m_nodesOrder[i];
		const dComplementaritySolver::dBilateralJoint* const joint = node->GetJoint();
		dAssert(joint);
		dAssert(i == node->m_solverIndex);

		dVectorPair& f = force[i];
		const dVectorPair& a = accel[i];
		f.m_body = a.m_body;
		f.m_joint = a.m_joint;

		const dList<dVehicleNode*>& children = node->GetChildren();
		//for (dgNode* child = node->m_child; child; child = child->m_sibling) {
		for (dList<dVehicleNode*>::dListNode* childNode = children.GetFirst(); childNode; childNode = childNode->GetNext()) {
			//dAssert(child->m_joint);
			dVehicleNode* const child = childNode->GetInfo();
			dAssert(child->m_parent->m_solverIndex == i);
			BodyJacobianTimeMassForward(child, force[child->m_solverIndex], f);
		}
		JointJacobianTimeMassForward(node, f);
	}

/*
	force[m_nodeCount - 1] = accel[m_nodeCount - 1];
	for (dgNode* child = m_nodesOrder[m_nodeCount - 1]->m_child; child; child = child->m_sibling) {
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	for (dgInt32 i = startNode; i < m_nodeCount - 1; i++) {
		dgNode* const node = m_nodesOrder[i];
		dVectorPair& f = force[i];
		node->BodyDiagInvTimeSolution(f);
		node->JointDiagInvTimeSolution(f);
	}
	m_nodesOrder[m_nodeCount - 1]->BodyDiagInvTimeSolution(force[m_nodeCount - 1]);
*/
}


void dVehicleSolver::CalculateForce(dVectorPair* const force, const dVectorPair* const accel) const
{
	SolveForward(force, accel, 0);
//	SolveBackward(force, accel);
}

void dVehicleSolver::CalculateJointForce()
{
//	dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces

	dVectorPair* const force = dAlloca(dVectorPair, m_nodeCount);
	dVectorPair* const accel = dAlloca(dVectorPair, m_nodeCount);
	CalculateJointAccel(accel);
	CalculateForce(force, accel);
/*
	if (m_auxiliaryRowCount) {
		SolveAuxiliary(jointInfoArray, internalForces, accel, force);
	}
	else {
		UpdateForces(jointInfoArray, internalForces, force);
	}
*/
}
