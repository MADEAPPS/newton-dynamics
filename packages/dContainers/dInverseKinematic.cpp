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

#include "dContainersStdAfx.h"
#include "dInverseKinematic.h"

/*
#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dInverseKinematicSolver.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"

class dInverseKinematicSolver::dgNodePair
{
	public:
	dgInt32 m_m0;
	dgInt32 m_m1;
};

DG_MSC_VECTOR_ALIGMENT
class dInverseKinematicSolver::dgForcePair
{
	public:
	dgSpatialVector m_joint;
	dgSpatialVector m_body;
} DG_GCC_VECTOR_ALIGMENT;

DG_MSC_VECTOR_ALIGMENT 
class dInverseKinematicSolver::dgMatriData
{
	public:
	dgSpatialMatrix m_jt;
	dgSpatialMatrix m_mass;
	dgSpatialMatrix m_invMass;
} DG_GCC_VECTOR_ALIGMENT;

DG_MSC_VECTOR_ALIGMENT 
class dInverseKinematicSolver::dgBodyJointMatrixDataPair
{
	public:
	dgMatriData m_body;
	dgMatriData m_joint;
} DG_GCC_VECTOR_ALIGMENT;

dgInt32 dInverseKinematicSolver::m_lruMarker = 1;
dgInt32 dInverseKinematicSolver::m_uniqueID = DG_SKELETON_BASE_UNIQUE_ID;
*/

class dInverseKinematicSolver::dGraph
{
	public:

	dGraph(NewtonBody* const body)
		:m_body (body)
//		,m_joint(NULL)
//		,m_parent(NULL)
//		,m_child(NULL)
//		,m_sibling(NULL)
//		,m_primaryStart(0)
//		,m_auxiliaryStart(0)
//		,m_index(0)
//		,m_dof(0)
//		,m_swapJacobianBodiesIndex(0)
	{
	}
/*
	dGraph (dgBilateralConstraint* const joint, dGraph* const parent)
		:m_body ((dgDynamicBody*) ((joint->GetBody0() == parent->m_body) ? joint->GetBody1() : joint->GetBody0()))
		,m_joint (joint)
		,m_parent(parent)
		,m_child(NULL)
		,m_sibling(NULL)
		,m_primaryStart(0)
		,m_auxiliaryStart(0)
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

	inline ~dGraph()
	{
		m_body->SetSkeleton(NULL);

		dGraph* next;
		for (dGraph* ptr = m_child; ptr; ptr = next) {
			next = ptr->m_sibling;
			delete ptr;
		}
	}

	inline void Factorize()
	{
		const dgSpatialMatrix& bodyMass = m_data.m_body.m_mass;
		dgSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass; 

		if (m_body->GetInvMass().m_w != dgFloat32(0.0f)) {
			for (dGraph* child = m_child; child; child = child->m_sibling) {
				CalculateBodyDiagonal(child);
			}
			bodyInvMass = bodyMass.Inverse(6);
		} else {
			bodyInvMass = dgSpatialMatrix (dgFloat32 (0.0f));
		}

		if (m_joint) {
			dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
			dgAssert(m_parent);
			for (dgInt32 i = 0; i < m_dof; i++) {
				bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
			}
			CalculateJointDiagonal();
			CalculateJacobianBlock();
		}
	}

	inline void CalculateInertiaMatrix()
	{
		dgSpatialMatrix& bodyMass = m_data.m_body.m_mass;

        dgFloat32 mass = m_body->GetMass().m_w;
		dgAssert(mass < dgFloat32(1.0e10f));
		dgMatrix inertia(m_body->CalculateInertiaMatrix());
		for (dgInt32 i = 0; i < 3; i++) {
			bodyMass[i][i] = mass;
			for (dgInt32 j = 0; j < 3; j++) {
				bodyMass[i + 3][j + 3] = inertia[i][j];
			}
		}
	}

	inline void GetJacobians(const dgJointInfo* const jointInfo, const dgJacobianMatrixElement* const matrixRow)
	{
		dgAssert(m_parent);
		dgAssert(jointInfo->m_joint == m_joint);

		dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		dgSpatialMatrix& jointMass = m_data.m_joint.m_mass;

		const dgInt32 start = jointInfo->m_pairStart;
		if (!m_swapJacobianBodiesIndex) {
			for (dgInt32 i = 0; i < m_dof; i++) {
				const dgInt32 k = m_sourceJacobianIndex[i];
				const dgJacobianMatrixElement* const row = &matrixRow[start + k];
				jointMass[i] = dgSpatialVector(dgFloat32(0.0f));
				jointMass[i][i] = -row->m_diagDamp;
				bodyJt[i] = dgSpatialVector (row->m_Jt.m_jacobianM0.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM0.m_angular.CompProduct4(dgVector::m_negOne));
				jointJ[i] = dgSpatialVector (row->m_Jt.m_jacobianM1.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM1.m_angular.CompProduct4(dgVector::m_negOne));
			}
		} else {
			for (dgInt32 i = 0; i < m_dof; i++) {
				const dgInt32 k = m_sourceJacobianIndex[i];
				const dgJacobianMatrixElement* const row = &matrixRow[start + k];
				jointMass[i] = dgSpatialVector(dgFloat32(0.0f));
				jointMass[i][i] = -row->m_diagDamp;
				bodyJt[i] = dgSpatialVector(row->m_Jt.m_jacobianM1.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM1.m_angular.CompProduct4(dgVector::m_negOne));
				jointJ[i] = dgSpatialVector(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM0.m_angular.CompProduct4(dgVector::m_negOne));
			}
		}
	}

	inline dgInt32 Factorize(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
	{
		dgSpatialMatrix& bodyMass = m_data.m_body.m_mass;

		bodyMass = dgSpatialMatrix(dgFloat32(0.0f));
        if (m_body->GetInvMass().m_w != dgFloat32 (0.0f)) {
			CalculateInertiaMatrix();
		}

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
				const dgJacobianMatrixElement* const row = &matrixRow[k + first];
				if ((row->m_lowerBoundFrictionCoefficent <= dgFloat32 (-DG_LCP_MAX_VALUE)) && (row->m_upperBoundFrictionCoefficent >= dgFloat32 (DG_LCP_MAX_VALUE))) {
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
			GetJacobians(jointInfo, matrixRow);
		}
		Factorize();
		return boundedDof;
	}

	inline void CalculateBodyDiagonal(dGraph* const child)
	{
		dgAssert(child->m_joint);
		
		dgSpatialMatrix copy (dgSpatialMatrix(dgFloat32(0.0f)));
		const dgInt32 dof = child->m_dof;
		const dgSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
		const dgSpatialMatrix& childDiagonal = child->m_data.m_joint.m_mass;
		for (dgInt32 i = 0; i < dof ; i++) {
			const dgSpatialVector& jacobian = jacobianMatrix[i];
			for (dgInt32 j = 0; j < dof ; j++) {
				dgAssert(dgAreEqual (dgFloat64(childDiagonal[i][j]), dgFloat64(childDiagonal[j][i]), dgFloat64(1.0e-5f)));
				dgFloat64 val = childDiagonal[i][j];
				copy[j] = copy[j] + jacobian.Scale(val);
			}
		}

		dgSpatialMatrix& bodyMass = m_data.m_body.m_mass;
		for (dgInt32 i = 0; i < dof; i++) {
			const dgSpatialVector& Jacobian = copy[i];
			const dgSpatialVector& JacobianTranspose = jacobianMatrix[i];
			for (dgInt32 j = 0; j < 6; j++) {
				dgFloat64 val = -Jacobian[j];
				bodyMass[j] = bodyMass[j] + JacobianTranspose.Scale(val);
			}
		}
	}

	inline void CalculateJointDiagonal ()
	{
		const dgSpatialMatrix& bodyMass = m_data.m_body.m_mass;
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;

		dgSpatialMatrix tmp;
		for (dgInt32 i = 0; i < m_dof; i++) {
			tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
		}

		dgSpatialMatrix& jointMass = m_data.m_joint.m_mass;
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

	inline void CalculateJacobianBlock()
	{
		dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;

		dgSpatialMatrix copy;
		for (dgInt32 i = 0; i < m_dof; i++) {
			copy[i] = jointJ[i];
			jointJ[i] = dgSpatialVector(dgFloat32(0.0f));
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

	inline void JointJacobianTimeMassForward (dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
		}
	}

	inline void BodyJacobianTimeMassForward(const dgForcePair& force, dgForcePair& parentForce) const 
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			parentForce.m_body = parentForce.m_body + jointJ[i].Scale(-force.m_joint[i]);
		}
	}

	inline void JointJacobianTimeSolutionBackward(dgForcePair& force, const dgForcePair& parentForce)
	{
		const dgSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		const dgSpatialVector& f = parentForce.m_body;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= f.DotProduct(jointJ[i]);
		}
	}

	inline void BodyJacobianTimeSolutionBackward(dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_body = force.m_body + bodyJt[i].Scale(-force.m_joint[i]);
		}
	}

	inline void BodyDiagInvTimeSolution(dgForcePair& force)
	{
		const dgSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
		force.m_body = bodyInvMass.VectorTimeMatrix(force.m_body);
	}

	inline void JointDiagInvTimeSolution(dgForcePair& force)
	{
		const dgSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
		force.m_joint = jointInvMass.VectorTimeMatrix(force.m_joint, m_dof);
	}

	inline dgInt32 GetAuxiliaryRows(const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const
	{
		dgInt32 rowCount = 0;
		if (m_joint) {
			dgAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			for (dgInt32 i = 0; i < count; i++) {
				const dgJacobianMatrixElement* const row = &matrixRow[i + first];
				if (!((row->m_lowerBoundFrictionCoefficent <= dgFloat32 (-DG_LCP_MAX_VALUE)) && (row->m_upperBoundFrictionCoefficent >= dgFloat32 (DG_LCP_MAX_VALUE)))) {
					rowCount++;
				}
			}
		}
		return rowCount;
	}
	
	dgBodyJointMatrixDataPair m_data;
	dgDynamicBody* m_body;
	dgBilateralConstraint* m_joint;
	dGraph* m_parent;
	dGraph* m_child;
	dGraph* m_sibling;
	union
	{
		dgInt64 m_ordinals;
		dgInt8 m_sourceJacobianIndex[8];
	};
	dgInt16 m_primaryStart;
	dgInt16 m_auxiliaryStart;
	dgInt16 m_index;
	dgInt8 m_dof;
	dgInt8 m_swapJacobianBodiesIndex;
	static dgInt64 m_ordinalInit;
*/

	NewtonBody* m_body;
};



#if 0
dInverseKinematicSolver::dGraph* dInverseKinematicSolver::GetRoot () const
{
	return m_skeleton;
}

dInverseKinematicSolver::dGraph* dInverseKinematicSolver::GetParent (dGraph* const node) const
{
	return node->m_parent;
}

dgDynamicBody* dInverseKinematicSolver::GetBody(dInverseKinematicSolver::dGraph* const node) const
{
	return node->m_body;
}

dgBilateralConstraint* dInverseKinematicSolver::GetParentJoint(dInverseKinematicSolver::dGraph* const node) const
{
	return node->m_joint;
}

dInverseKinematicSolver::dGraph* dInverseKinematicSolver::GetFirstChild(dInverseKinematicSolver::dGraph* const parent) const
{
	return parent->m_child;
}

dInverseKinematicSolver::dGraph* dInverseKinematicSolver::GetNextSiblingChild(dInverseKinematicSolver::dGraph* const sibling) const
{
	return sibling->m_sibling;
}

dgWorld* dInverseKinematicSolver::GetWorld() const
{
	return m_world;
}

void dInverseKinematicSolver::ResetUniqueId(dgInt32 id)
{
	m_uniqueID = id;
}

void dInverseKinematicSolver::SortGraph(dGraph* const root, dGraph* const parent, dgInt32& index)
{
	for (dGraph* node = root->m_child; node; node = node->m_sibling) {
		SortGraph(node, root, index);
	}

	dgAssert((m_nodeCount - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_index = dgInt16(index);
	index++;
	dgAssert(index <= m_nodeCount);
}

dInverseKinematicSolver::dGraph* dInverseKinematicSolver::FindNode(dgDynamicBody* const body) const
{
	dgInt32 stack = 1;
	dGraph* stackPool[1024];

	stackPool[0] = m_skeleton;
	while (stack) {
		stack--;
		dGraph* const node = stackPool[stack];
		if (node->m_body == body) {
			return node;
		}

		for (dGraph* ptr = node->m_child; ptr; ptr = ptr->m_sibling) {
			stackPool[stack] = ptr;
			stack++;
			dgAssert(stack < dgInt32(sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return NULL;
}

dInverseKinematicSolver::dGraph* dInverseKinematicSolver::AddChild(dgBilateralConstraint* const joint, dGraph* const parent)
{
	dgAssert (m_skeleton->m_body);
	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	dGraph* const node = new (allocator)dGraph(joint, parent);
	m_nodeCount ++;

	joint->m_isInSkeleton = true;
	dgAssert (m_world->GetSentinelBody() != node->m_body);
	node->m_body->SetSkeleton(this);
	return node;
}

void dInverseKinematicSolver::RemoveCyclingJoint(dgBilateralConstraint* const joint)
{
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dgCyclingJoint& entry = ptr->GetInfo();
		if (entry.m_joint == joint) {
			joint->m_isInSkeleton = false;
			m_cyclingJoints.Remove(ptr);
			break;
		}
	}

	for (dgList<dgDynamicBody*>::dgListNode* ptr = m_cyclingBodies.GetFirst(); ptr; ptr = ptr->GetNext()) {
		if ((joint->GetBody0() == ptr->GetInfo()) || (joint->GetBody1() == ptr->GetInfo())) {
			dgAssert (0);
			m_cyclingBodies.Remove(ptr);
			break;
		}
	}
}

void dInverseKinematicSolver::Finalize(dgInt32 loopJointsCount, dgBilateralConstraint** const loopJointArray)
{
	dgAssert(m_nodeCount >= 1);

	const dgDynamicBody* const rootBody = m_skeleton->m_body;
	dgAssert (((rootBody->GetInvMass().m_w == dgFloat32 (0.0f)) && (m_skeleton->m_child->m_sibling == NULL)) || (m_skeleton->m_body->GetInvMass().m_w != dgFloat32 (0.0f)));

	dgMemoryAllocator* const allocator = rootBody->GetWorld()->GetAllocator();
	m_nodesOrder = (dGraph**)allocator->Malloc(m_nodeCount * sizeof (dGraph*));

	dgInt32 index = 0;
	SortGraph(m_skeleton, NULL, index);
	dgAssert(index == m_nodeCount);

	if (loopJointsCount) {
		dgInt32 loopIndex = m_nodeCount;
		dgTree<dgInt32, dgDynamicBody*> filter (allocator);
		for (dgInt32 i = 0; i < loopJointsCount; i++) {
			dgBilateralConstraint* const joint = loopJointArray[i];
			dgDynamicBody* const body0 = (dgDynamicBody*)joint->GetBody0();
			dgDynamicBody* const body1 = (dgDynamicBody*)joint->GetBody1();

			dgAssert(body0->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert(body1->IsRTTIType(dgBody::m_dynamicBodyRTTI));

			dGraph* const node0 = FindNode(body0);
			dGraph* const node1 = FindNode(body1);
			dgAssert((node0 && !node1) || (node1 && !node0));
			joint->m_isInSkeleton = true;
		
			if (node0) {
				filter.Insert(node0->m_index, node0->m_body);
			}
			if (node1) {
				filter.Insert(node1->m_index, node1->m_body);
			}

			dgTree<dgInt32, dgDynamicBody*>::dgTreeNode* index0 = filter.Find(body0);
			if (!index0) {
				index0 = filter.Insert(loopIndex, body0);
				loopIndex ++;
			}

			dgTree<dgInt32, dgDynamicBody*>::dgTreeNode* index1 = filter.Find(body1);
			if (!index1) {
				index1 = filter.Insert(loopIndex, body1);
				loopIndex++;
			}
			dgCyclingJoint cyclicEntry(joint, index0->GetInfo(), index1->GetInfo());
			m_cyclingJoints.Append(cyclicEntry);
		}

		for (dgInt32 i = 0; i < m_nodeCount; i++) {
			filter.Remove(m_nodesOrder[i]->m_body);
		}
		dgTree<dgDynamicBody*, dgInt32> bodyOrder (allocator);
		dgTree<dgInt32, dgDynamicBody*>::Iterator iter(filter);
		for (iter.Begin(); iter; iter ++) {
			bodyOrder.Insert (iter.GetKey(), iter.GetNode()->GetInfo());
		}

		dgTree<dgDynamicBody*, dgInt32>::Iterator iter1(bodyOrder);
		for (iter1.Begin(); iter1; iter1 ++) {
			m_cyclingBodies.Append(iter1.GetNode()->GetInfo());
		}
	}
}


void dInverseKinematicSolver::InitMassMatrix(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgInt8* const memoryBuffer)
{
	dTimeTrackerEvent(__FUNCTION__);

	dgInt32 rowCount = 0;
	dgInt32 primaryStart = 0;
	dgInt32 auxiliaryStart = 0;

	if (m_nodesOrder) {
		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			dGraph* const node = m_nodesOrder[i];
			rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			node->m_auxiliaryStart = dgInt16 (auxiliaryStart);
			node->m_primaryStart = dgInt16 (primaryStart);
			auxiliaryStart += node->Factorize(jointInfoArray, matrixRow);
			primaryStart += node->m_dof;
		}
		m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, matrixRow);
	}
	m_rowCount = dgInt16 (rowCount);
	m_auxiliaryRowCount = dgInt16 (auxiliaryStart);

	dgInt32 extraAuxiliaryRows = 0;
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgConstraint* const joint = ptr->GetInfo().m_joint;
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	m_rowCount += dgInt16 (extraAuxiliaryRows);
	m_auxiliaryRowCount += dgInt16 (extraAuxiliaryRows);

	if (m_auxiliaryRowCount) {
		const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
		dgInt32 primaryIndex = 0;
		dgInt32 auxiliaryIndex = 0;

		m_rowArray = (dgJacobianMatrixElement**) memoryBuffer;
		m_pairs = (dgNodePair*) &m_rowArray[m_rowCount];
		m_massMatrix11 = (dgFloat32*)&m_pairs[m_rowCount];
		m_lowerTriangularMassMatrix11 = (dgFloat32*)&m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
		m_massMatrix10 = &m_lowerTriangularMassMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
		m_deltaForce = &m_massMatrix10[m_auxiliaryRowCount * primaryCount];

		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			const dGraph* const node = m_nodesOrder[i];
			const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
			
#ifdef DG_EXPERIMENTAL_SOLVER
			const dgInt32 m0 = node->m_swapJacobianBodiesIndex ? node->m_parent->m_index : i;
			const dgInt32 m1 = node->m_swapJacobianBodiesIndex ? i : node->m_parent->m_index;
#else 
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
#endif
			const dgInt32 primaryDof = node->m_dof;
			const dgInt32 first = jointInfo->m_pairStart;

			for (dgInt32 j = 0; j < primaryDof; j++) {
				const dgInt32 index = node->m_sourceJacobianIndex[j];
				m_rowArray[primaryIndex] = &matrixRow[first + index];
				m_pairs[primaryIndex].m_m0 = m0;
				m_pairs[primaryIndex].m_m1 = m1;
				primaryIndex++;
			}

			const dgInt32 auxiliaryDof = jointInfo->m_pairCount - primaryDof;
			for (dgInt32 j = 0; j < auxiliaryDof; j++) {
				const dgInt32 index = node->m_sourceJacobianIndex[primaryDof + j];
				dgJacobianMatrixElement* const row = &matrixRow[first + index];
				m_rowArray[auxiliaryIndex + primaryCount] = row;
				m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
				m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
				auxiliaryIndex++;
			}
		}

		for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
			const dgCyclingJoint& entry = ptr->GetInfo();
			const dgConstraint* const joint = entry.m_joint;
			const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];
			
#ifdef DG_EXPERIMENTAL_SOLVER
			const dgInt32 m0 = entry.m_m0;
			const dgInt32 m1 = entry.m_m1;
#else
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
#endif
			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 auxiliaryDof = jointInfo->m_pairCount;

			for (dgInt32 i = 0; i < auxiliaryDof; i++) {
				dgJacobianMatrixElement* const row = &matrixRow[first + i];
				m_rowArray[auxiliaryIndex + primaryCount] = row;
				m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
				m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
				auxiliaryIndex++;
			}
		}

		dgFloat32* const diagDamp = dgAlloca(dgFloat32, m_auxiliaryRowCount);
		const dgInt32 auxiliaryCount = m_rowCount - m_auxiliaryRowCount;
		for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
			const dgJacobianMatrixElement* const row_i = m_rowArray[primaryCount + i];
			dgFloat32* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * i];

			dgJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
			dgJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);

			dgVector element(JMinvM0.m_linear.CompProduct4(row_i->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row_i->m_Jt.m_jacobianM0.m_angular) +
							 JMinvM1.m_linear.CompProduct4(row_i->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row_i->m_Jt.m_jacobianM1.m_angular));
			element = element.AddHorizontal();

			// I know I am doubling the matrix regularizer, but this makes the solution more robust.
			dgFloat32 diagonal = element.GetScalar() + row_i->m_diagDamp;
			matrixRow11[i] = diagonal + row_i->m_diagDamp;
			diagDamp[i] = matrixRow11[i] * (DG_PSD_DAMP_TOL * dgFloat32 (2.0f));

			const dgInt32 m0 = m_pairs[auxiliaryCount + i].m_m0;
			const dgInt32 m1 = m_pairs[auxiliaryCount + i].m_m1;
			for (dgInt32 j = i + 1; j < m_auxiliaryRowCount; j++) {
				const dgJacobianMatrixElement* const row_j = m_rowArray[auxiliaryCount + j];

				const dgInt32 k = auxiliaryCount + j;
				dgVector acc(dgVector::m_zero);
				if (m0 == m_pairs[k].m_m0) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				} else if (m0 == m_pairs[k].m_m1) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				}

				if (m1 == m_pairs[k].m_m1) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				} else if (m1 == m_pairs[k].m_m0) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				}
				acc = acc.AddHorizontal();
				dgFloat32 offDiagValue = acc.GetScalar();
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagValue;
			}

			dgFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * i];
			for (dgInt32 j = 0; j < primaryCount; j++) {
				const dgJacobianMatrixElement* const row_j = m_rowArray[j];

				dgVector acc(dgVector::m_zero);
				if (m0 == m_pairs[j].m_m0) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				} else if (m0 == m_pairs[j].m_m1) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				}

				if (m1 == m_pairs[j].m_m1) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				} else if (m1 == m_pairs[j].m_m0) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				}
				acc = acc.AddHorizontal();
				dgFloat32 val = acc.GetScalar();
				matrixRow10[j] = val;
			}
		}

		dgForcePair* const forcePair = dgAlloca(dgForcePair, m_nodeCount);
		dgForcePair* const accelPair = dgAlloca(dgForcePair, m_nodeCount);
		accelPair[m_nodeCount - 1].m_body = dgSpatialVector(dgFloat32(0.0f));
		accelPair[m_nodeCount - 1].m_joint = dgSpatialVector(dgFloat32(0.0f));

		for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
			dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];

			dgInt32 entry = 0;
			for (dgInt32 j = 0; j < m_nodeCount - 1; j++) {
				const dGraph* const node = m_nodesOrder[j];
				const dgInt32 index = node->m_index;
				accelPair[index].m_body = dgSpatialVector(dgFloat32(0.0f));
				dgSpatialVector& a = accelPair[index].m_joint;

				const int count = node->m_dof;
				for (dgInt32 k = 0; k < count; k++) {
					a[k] = matrixRow10[entry];
					entry++;
				}
			}

			entry = 0;
			CalculateForce(forcePair, accelPair);
			dgFloat32* const deltaForcePtr = &m_deltaForce[i * primaryCount];
			for (dgInt32 j = 0; j < m_nodeCount - 1; j++) {
				const dGraph* const node = m_nodesOrder[j];
				const dgInt32 index = node->m_index;
				const dgSpatialVector& f = forcePair[index].m_joint;
				const int count = node->m_dof;
				for (dgInt32 k = 0; k < count; k++) {
					deltaForcePtr[entry] = dgFloat32 (f[k]);
					entry++;
				}
			}

			dgFloat32* const matrixRow11 = &m_massMatrix11[i * m_auxiliaryRowCount];
			dgFloat32 diagonal = matrixRow11[i];
			for (dgInt32 k = 0; k < primaryCount; k++) {
				diagonal += deltaForcePtr[k] * matrixRow10[k];
			}
			matrixRow11[i] = dgMax(diagonal, diagDamp[i]);

			for (dgInt32 j = i + 1; j < m_auxiliaryRowCount; j++) {
				dgFloat32 offDiagonal = dgFloat32(0.0f);
				const dgFloat32* const row10 = &m_massMatrix10[j * primaryCount];
				for (dgInt32 k = 0; k < primaryCount; k++) {
					offDiagonal += deltaForcePtr[k] * row10[k];
				}
				matrixRow11[j] += offDiagonal;
				m_massMatrix11[j * m_auxiliaryRowCount + i] += offDiagonal;
			}
		}

		bool isPsdMatrix = false;
		do {
			memcpy (m_lowerTriangularMassMatrix11, m_massMatrix11, sizeof (dgFloat32) * (m_auxiliaryRowCount * m_auxiliaryRowCount));
			isPsdMatrix = dgCholeskyFactorization(m_auxiliaryRowCount, m_lowerTriangularMassMatrix11);
			if (!isPsdMatrix) {
				for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
					diagDamp[i] *= dgFloat32 (2.0f);
					m_massMatrix11[i * m_auxiliaryRowCount + i] += diagDamp[i];
				}
			}
		} while (!isPsdMatrix);

		for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
			dgFloat32* const row = &m_lowerTriangularMassMatrix11[i * m_auxiliaryRowCount];
			for (dgInt32 j = i + 1; j < m_auxiliaryRowCount; j++) {
				row[j] = dgFloat32 (0.0f);
			}
		}
	}
}

bool dInverseKinematicSolver::SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const
{
	return true;
}

inline void dInverseKinematicSolver::CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const
{
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dGraph* const node = m_nodesOrder[i];
		dgAssert(node->m_joint);
		dgAssert(node->m_index == i);
		dgForcePair& f = force[i];
		const dgForcePair& a = accel[i];
		f.m_body = a.m_body;
		for (dgInt32 j = 0; j < node->m_dof; j ++) {
			f.m_joint[j] = a.m_joint[j]; 
		}
		for (dGraph* child = node->m_child; child; child = child->m_sibling) {
			dgAssert(child->m_joint);
			dgAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(f);
	}

	force[m_nodeCount - 1] = accel[m_nodeCount - 1];
	for (dGraph* child = m_nodesOrder[m_nodeCount - 1]->m_child; child; child = child->m_sibling) {
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	m_nodesOrder[m_nodeCount - 1]->BodyDiagInvTimeSolution(force[m_nodeCount - 1]);
	for (dgInt32 i = m_nodeCount - 2; i >= 0; i--) {
		dGraph* const node = m_nodesOrder[i];
		dgAssert(node->m_index == i);
		dgForcePair& f = force[i];
		node->JointDiagInvTimeSolution(f);
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyDiagInvTimeSolution(f);
		node->BodyJacobianTimeSolutionBackward(f);
	}
//	dgAssert (SanityCheck(force, accel));
}


inline void dInverseKinematicSolver::UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force) const
{
	dgVector zero (dgVector::m_zero);
	for (dgInt32 i = 0; i < (m_nodeCount - 1)  ; i ++) {
		dGraph* const node = m_nodesOrder[i];
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
			dgJacobianMatrixElement* const row = &matrixRow[first + k];

			row->m_force += dgFloat32(f[j]);
			dgVector jointForce = dgFloat32 (f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
		}

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}
}

inline void dInverseKinematicSolver::CalculateJointAccel(dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const
{
	dgSpatialVector zero(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dGraph* const node = m_nodesOrder[i];
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
			const dgJacobianMatrixElement* const row = &matrixRow[first + k];
			dgVector diag(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						  row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			a.m_joint[j] = -(row->m_coordenateAccel - row->m_force * row->m_diagDamp - (diag.AddHorizontal()).GetScalar());
		}
	}
	dgAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body = dgSpatialVector(dgFloat32(0.0f));
	accel[m_nodeCount - 1].m_joint = dgSpatialVector(dgFloat32(0.0f));
}


void dInverseKinematicSolver::SolveAuxiliary(const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force) const
{
	dTimeTrackerEvent(__FUNCTION__);
	
	dgFloat32* const f = dgAlloca(dgFloat32, m_rowCount);
	dgFloat32* const u = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const b = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const low = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const high = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const massMatrix11 = dgAlloca(dgFloat32, m_auxiliaryRowCount * m_auxiliaryRowCount);
	dgFloat32* const lowerTriangularMassMatrix11 = dgAlloca(dgFloat32, m_auxiliaryRowCount * m_auxiliaryRowCount);

	dgInt32 primaryIndex = 0;
	dgInt32 auxiliaryIndex = 0;
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) { 
		const dGraph* const node = m_nodesOrder[i];
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
			dgJacobianMatrixElement* const row = &matrixRow[first + index];
			f[auxiliaryIndex + primaryCount] = dgFloat32 (0.0f);
			b[auxiliaryIndex] = -dgFloat32(accelSpatial[primaryDof + j]);
			low[auxiliaryIndex] = dgClamp (row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dgFloat32 (0.0f));
			high[auxiliaryIndex] = dgClamp (row->m_upperBoundFrictionCoefficent - row->m_force, dgFloat32 (0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgCyclingJoint& entry = ptr->GetInfo();
		const dgConstraint* const joint = entry.m_joint;
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 auxiliaryDof = jointInfo->m_pairCount;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (dgInt32 i = 0; i < auxiliaryDof; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[first + i];
			f[auxiliaryIndex + primaryCount] = dgFloat32 (0.0f);
			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			b[auxiliaryIndex] = row->m_coordenateAccel - (acc.AddHorizontal()).GetScalar();
			low[auxiliaryIndex] = dgClamp (row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dgFloat32 (0.0f));
			high[auxiliaryIndex] = dgClamp (row->m_upperBoundFrictionCoefficent - row->m_force, dgFloat32 (0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	memcpy (massMatrix11, m_massMatrix11, sizeof (dgFloat32) * m_auxiliaryRowCount * m_auxiliaryRowCount);
	memcpy (lowerTriangularMassMatrix11, m_lowerTriangularMassMatrix11, sizeof (dgFloat32) * m_auxiliaryRowCount * m_auxiliaryRowCount);
	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i ++) {
		dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		u[i] = dgFloat32(0.0f);
		dgFloat32 r = dgFloat32(0.0f);
		for (dgInt32 j = 0; j < primaryCount; j++) {
			r += matrixRow10[j] * f[j];
		}
		b[i] -= r;
	}
	
	dgSolveDantzigLCP(m_auxiliaryRowCount, massMatrix11, lowerTriangularMassMatrix11, u, b, low, high);

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dgFloat32 s = u[i];
		f[primaryCount + i] = s;
		const dgFloat32* const deltaForce = &m_deltaForce[i * primaryCount];
		for (dgInt32 j = 0; j < primaryCount; j++) {
			f[j] += deltaForce[j] * s;
		}
	}

	for (dgInt32 i = 0; i < m_rowCount; i++) {
		dgJacobianMatrixElement* const row = m_rowArray[i];
		const dgInt32 m0 = m_pairs[i].m_m0;
		const dgInt32 m1 = m_pairs[i].m_m1;

		row->m_force += f[i];
		dgVector jointForce(f[i]);
		internalForces[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
		internalForces[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
		internalForces[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
		internalForces[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
	}
}


dgInt32 dInverseKinematicSolver::GetMemoryBufferSizeInBytes (const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const
{
	dgInt32 rowCount = 0;
	dgInt32 auxiliaryRowCount = 0;
	if (m_nodesOrder) {
		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			dGraph* const node = m_nodesOrder[i];
			rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			auxiliaryRowCount += node->GetAuxiliaryRows(jointInfoArray, matrixRow);
		}
	}

	dgInt32 extraAuxiliaryRows = 0;
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgCyclingJoint& entry = ptr->GetInfo();
		const dgConstraint* const joint = entry.m_joint;
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount+= extraAuxiliaryRows;

	dgInt32 size = sizeof (dgJacobianMatrixElement*) * rowCount;
	size += sizeof (dgNodePair) * rowCount;
	size += sizeof (dgFloat32) * auxiliaryRowCount * auxiliaryRowCount * 2;
	size += sizeof (dgFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof (dgFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	return (size + 1024) & -0x10;
}


void dInverseKinematicSolver::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow)
{
	dTimeTrackerEvent(__FUNCTION__);

	dgForcePair* const force = dgAlloca(dgForcePair, m_nodeCount);
	dgForcePair* const accel = dgAlloca(dgForcePair, m_nodeCount);

	CalculateJointAccel(jointInfoArray, internalForces, matrixRow, accel);
	CalculateForce(force, accel);
	if (m_auxiliaryRowCount) {
		SolveAuxiliary (jointInfoArray, internalForces, matrixRow, accel, force);
	} else {
		UpdateForces(jointInfoArray, internalForces, matrixRow, force);
	}
}


inline void dInverseKinematicSolver::CalculateBodyAcceleration (dgJacobian* const accel, dgFloat32 timestep) const
{
	const dgVector invTimeStep (dgFloat32 (1.0f) / timestep);
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dGraph* const node = m_nodesOrder[i];
		dgAssert(i == node->m_index);
		dgAssert(node->m_body);
		const dgDynamicBody* const body = node->m_body;
		accel[i].m_linear = (body->m_veloc - body->m_accel).CompProduct4(invTimeStep);
		accel[i].m_angular = (body->m_omega - body->m_alpha).CompProduct4(invTimeStep);
	}

	dgInt32 index = m_nodeCount;
	for (dgList<dgDynamicBody*>::dgListNode* node = m_cyclingBodies.GetFirst(); node; node = node->GetNext()) {
		const dgDynamicBody* const body = node->GetInfo();
		accel[index].m_linear = (body->m_veloc - body->m_accel).CompProduct4(invTimeStep);
		accel[index].m_angular = (body->m_omega - body->m_alpha).CompProduct4(invTimeStep);
		
		#ifdef _DEBUG
			if (body->GetInvMass().m_w == dgFloat32 (0.0f)) {
				dgAssert (accel[index].m_linear.DotProduct4(accel[index].m_linear).GetScalar() == dgFloat32 (0.0f));
				dgAssert (accel[index].m_angular.DotProduct4(accel[index].m_angular).GetScalar() == dgFloat32 (0.0f));
			}
		#endif
		index ++;
	}

#ifdef _DEBUG
	if (m_skeleton->m_body->GetInvMass().m_w == dgFloat32 (0.0f)) {
		dgAssert (accel[m_nodeCount - 1].m_linear.DotProduct4(accel[m_nodeCount - 1].m_linear).GetScalar() == dgFloat32 (0.0f));
		dgAssert (accel[m_nodeCount - 1].m_angular.DotProduct4(accel[m_nodeCount - 1].m_angular).GetScalar() == dgFloat32 (0.0f));
	}
#endif
}


inline void dInverseKinematicSolver::CalculateJointAccel(const dgJacobian* const externalAccel, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const
{
	dgSpatialVector zero (dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {

		dGraph* const node = m_nodesOrder[i];
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

		const dgInt32 m0 = node->m_swapJacobianBodiesIndex ? node->m_parent->m_index : i;
		const dgInt32 m1 = node->m_swapJacobianBodiesIndex ? i : node->m_parent->m_index;

		const dgJacobian& y0 = externalAccel[m0];
		const dgJacobian& y1 = externalAccel[m1];
		for (dgInt32 j = 0; j < dof; j++) {
			const dgInt32 k = node->m_sourceJacobianIndex[j];
			const dgJacobianMatrixElement* const row = &matrixRow[first + k];
			dgVector diag(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_Jt.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						  row->m_Jt.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_Jt.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			a.m_joint[j] = (diag.AddHorizontal()).GetScalar() - row->m_penetrationStiffness;
		}
	}
	dgAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body = dgSpatialVector(dgFloat32(0.0f));
	accel[m_nodeCount - 1].m_joint = dgSpatialVector(dgFloat32(0.0f));
}


inline void dInverseKinematicSolver::UpdateVelocity(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force, dgJacobian* const internalForces, dgFloat32 timestep) const
{
	dgVector zero(dgVector::m_zero);
	dgAssert (m_cyclingBodies.GetCount() == 0);
	//const dgInt32 bodyCount = m_cyclingBodies.GetCount() + m_nodeCount;
	const dgInt32 bodyCount = m_nodeCount;
	for (dgInt32 i = 0; i < bodyCount; i++) {
		internalForces[i].m_linear = zero; 
		internalForces[i].m_angular = zero; 
	}

	for (dgInt32 i = 0; i < (m_nodeCount - 1); i++) {
		dGraph* const node = m_nodesOrder[i];
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
			dgJacobianMatrixElement* const row = &matrixRow[first + k];

			row->m_force += dgFloat32(f[j]);
			dgVector jointForce = dgFloat32(f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
		}

		//const dgInt32 m0 = i;
		//const dgInt32 m1 = node->m_parent->m_index;
		const dgInt32 m0 = node->m_swapJacobianBodiesIndex ? node->m_parent->m_index : i;
		const dgInt32 m1 = node->m_swapJacobianBodiesIndex ? i : node->m_parent->m_index;

		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}

	dgVector timestep4 (timestep);
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dGraph* const node = m_nodesOrder[i];
		dgAssert(i == node->m_index);
		dgAssert(node->m_body);
		dgDynamicBody* const body = node->m_body;

		dgVector velocStep(internalForces[i].m_linear.Scale4 (timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(internalForces[i].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
	}

	dgInt32 index = m_nodeCount;
	for (dgList<dgDynamicBody*>::dgListNode* node = m_cyclingBodies.GetFirst(); node; node = node->GetNext()) {
		dgDynamicBody* const body = node->GetInfo();

		dgVector velocStep(internalForces[index].m_linear.Scale4(timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(internalForces[index].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
		index++;
	}
}

void dInverseKinematicSolver::SolveAuxiliaryVelocity(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force, dgJacobian* const externalAccel, dgFloat32 timestep) const
{
	dTimeTrackerEvent(__FUNCTION__);

	dgFloat32* const f = dgAlloca(dgFloat32, m_rowCount);
	dgFloat32* const u = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const b = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const low = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const high = dgAlloca(dgFloat32, m_auxiliaryRowCount);
	dgFloat32* const massMatrix11 = dgAlloca(dgFloat32, m_auxiliaryRowCount * m_auxiliaryRowCount);
	dgFloat32* const lowerTriangularMassMatrix11 = dgAlloca(dgFloat32, m_auxiliaryRowCount * m_auxiliaryRowCount);

	dgInt32 primaryIndex = 0;
	dgInt32 auxiliaryIndex = 0;
	const dgInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		const dGraph* const node = m_nodesOrder[i];
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
			dgJacobianMatrixElement* const row = &matrixRow[first + index];
			f[auxiliaryIndex + primaryCount] = dgFloat32(0.0f);
			b[auxiliaryIndex] = -dgFloat32(accelSpatial[primaryDof + j]);
			low[auxiliaryIndex] = dgClamp(row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dgFloat32(0.0f));
			high[auxiliaryIndex] = dgClamp(row->m_upperBoundFrictionCoefficent - row->m_force, dgFloat32(0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgCyclingJoint& entry = ptr->GetInfo();
		const dgConstraint* const joint = entry.m_joint;

		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];
		const dgInt32 m0 = entry.m_m0;
		const dgInt32 m1 = entry.m_m1;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 auxiliaryDof = jointInfo->m_pairCount;
		const dgJacobian& y0 = externalAccel[m0];
		const dgJacobian& y1 = externalAccel[m1];

		for (dgInt32 i = 0; i < auxiliaryDof; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[first + i];
			f[auxiliaryIndex + primaryCount] = dgFloat32(0.0f);
//			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
//						 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
//			b[auxiliaryIndex] = row->m_coordenateAccel - (acc.AddHorizontal()).GetScalar();
			dgVector diag(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_Jt.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						  row->m_Jt.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_Jt.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			b[auxiliaryIndex] = row->m_penetrationStiffness - (diag.AddHorizontal()).GetScalar();

			low[auxiliaryIndex] = dgClamp(row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dgFloat32(0.0f));
			high[auxiliaryIndex] = dgClamp(row->m_upperBoundFrictionCoefficent - row->m_force, dgFloat32(0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	memcpy(massMatrix11, m_massMatrix11, sizeof(dgFloat32) * m_auxiliaryRowCount * m_auxiliaryRowCount);
	memcpy(lowerTriangularMassMatrix11, m_lowerTriangularMassMatrix11, sizeof(dgFloat32) * m_auxiliaryRowCount * m_auxiliaryRowCount);

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		dgFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		u[i] = dgFloat32(0.0f);
		dgFloat32 r = dgFloat32(0.0f);
		for (dgInt32 j = 0; j < primaryCount; j++) {
			r += matrixRow10[j] * f[j];
		}
		b[i] -= r;
	}

	dgSolveDantzigLCP(m_auxiliaryRowCount, massMatrix11, lowerTriangularMassMatrix11, u, b, low, high);

	for (dgInt32 i = 0; i < m_auxiliaryRowCount; i++) {
		const dgFloat32 s = u[i];
		f[primaryCount + i] = s;
		const dgFloat32* const deltaForce = &m_deltaForce[i * primaryCount];
		for (dgInt32 j = 0; j < primaryCount; j++) {
			f[j] += deltaForce[j] * s;
		}
	}


	dgVector zero(dgVector::m_zero);
	const dgInt32 bodyCount = m_cyclingBodies.GetCount() + m_nodeCount;
	for (dgInt32 i = 0; i < bodyCount; i++) {
		externalAccel[i].m_linear = zero;
		externalAccel[i].m_angular = zero;
	}

	for (dgInt32 i = 0; i < m_rowCount; i++) {
		dgJacobianMatrixElement* const row = m_rowArray[i];
		const dgInt32 m0 = m_pairs[i].m_m0;
		const dgInt32 m1 = m_pairs[i].m_m1;

		row->m_force += f[i];
		dgVector jointForce(f[i]);
		externalAccel[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
		externalAccel[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
		externalAccel[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
		externalAccel[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
	}

	dgVector timestep4(timestep);
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dGraph* const node = m_nodesOrder[i];
		dgAssert(i == node->m_index);
		dgAssert(node->m_body);
		dgDynamicBody* const body = node->m_body;

		dgVector velocStep(externalAccel[i].m_linear.Scale4(timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(externalAccel[i].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
	}

	dgInt32 index = m_nodeCount;
	for (dgList<dgDynamicBody*>::dgListNode* node = m_cyclingBodies.GetFirst(); node; node = node->GetNext()) {
		dgDynamicBody* const body = node->GetInfo();

		dgVector velocStep(externalAccel[index].m_linear.Scale4(timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(externalAccel[index].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
		index++;
	}
}


void dInverseKinematicSolver::UpdateForces (dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgFloat32 timestep)
{

	dgInt32 memorySizeInBytes = GetMemoryBufferSizeInBytes(jointInfoArray, matrixRow);
	dgInt8* const memoryBuffer = (dgInt8*)dgAlloca(dgVector, memorySizeInBytes / sizeof(dgVector));
	dgForcePair* const accel = dgAlloca(dgForcePair, m_nodeCount);
	dgForcePair* const force = dgAlloca(dgForcePair, m_nodeCount);
	dgJacobian* const externalAccel = dgAlloca(dgJacobian, m_nodeCount + m_cyclingBodies.GetCount());

	dgAssert((dgInt64(accel) & 0x0f) == 0);
	dgAssert((dgInt64(force) & 0x0f) == 0);
	dgAssert((dgInt64(memoryBuffer) & 0x0f) == 0);
	dgAssert((dgInt64(externalAccel) & 0x0f) == 0);

	InitMassMatrix(jointInfoArray, matrixRow, memoryBuffer);
	CalculateBodyAcceleration (externalAccel, timestep);
	CalculateJointAccel(externalAccel, jointInfoArray, matrixRow, accel);
	CalculateForce(force, accel);

	if (m_auxiliaryRowCount) {
		SolveAuxiliaryVelocity(jointInfoArray, matrixRow, accel, force, externalAccel, timestep);
	} else {
		UpdateVelocity(jointInfoArray, matrixRow, force, externalAccel, timestep);
	}
}

#endif


dInverseKinematicSolver::dInverseKinematicSolver(NewtonBody* const rootBody)
	:dContainersAlloc()
	,m_skeleton(new dGraph(rootBody))
/*
	,m_nodesOrder(NULL)
	,m_pairs(NULL)
	,m_deltaForce(NULL)
	,m_massMatrix11(NULL)
	,m_massMatrix10(NULL)
	,m_lowerTriangularMassMatrix11(NULL)
	,m_rowArray(NULL)
	,m_cyclingBodies(rootBody->GetWorld()->GetAllocator())
	,m_cyclingJoints(rootBody->GetWorld()->GetAllocator())
	,m_id(m_uniqueID)
	,m_lru(0)
	,m_nodeCount(1)
	,m_rowCount(0)
	,m_auxiliaryRowCount(0)
*/
{
/*
	if (rootBody->GetInvMass().m_w != dgFloat32 (0.0f)) {
		rootBody->SetSkeleton(this);
	}
	m_uniqueID++;
*/
}

dInverseKinematicSolver::~dInverseKinematicSolver()
{
/*
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dgCyclingJoint& entry = ptr->GetInfo();
		entry.m_joint->m_isInSkeleton = false;
	}
	m_cyclingJoints.RemoveAll();

	for (dgInt32 i = 0; i < m_nodeCount - 1; i ++) {
		m_nodesOrder[i]->m_joint->m_isInSkeleton = false;
	}

	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	if (m_nodesOrder) {
		allocator->Free(m_nodesOrder);
	}

	delete m_skeleton;
*/
}
