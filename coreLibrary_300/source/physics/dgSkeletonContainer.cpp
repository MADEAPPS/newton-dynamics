/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////// 
////////////////////////////////////////////////////////////////////// 
dgInt32 dgSkeletonContainer::m_lruMarker = 1;
dgInt32 dgSkeletonContainer::m_uniqueID = DG_SKELETON_BASEW_UNIQUE_ID;

class dgSkeletonContainer::dgClippedNodes
{
	public:
	dgGraph* m_node;
	dgInt32 m_relIndex;
};

class dgSkeletonContainer::dgGraph
{
	public:

	DG_CLASS_ALLOCATOR(allocator)
	dgGraph (dgDynamicBody* const body, dgBilateralConstraint* const Joint, dgGraph* const parent)
		:m_body (body)
		,m_joint (Joint)
		,m_parent(parent)
		,m_child(NULL)
		,m_sibling(NULL)
		,m_index(0)
		,m_dof(0)
		,m_primaryStart(0)
		,m_auxiliaryStart(0)
	{
		if (m_parent) {
			if (m_parent->m_child) {
				m_sibling = m_parent->m_child;
			}
			m_parent->m_child = this;
		}
	}

	DG_INLINE ~dgGraph()
	{
		dgGraph* next;
		m_body->SetSkeleton(NULL);

		for (dgGraph* ptr = m_child; ptr; ptr = next) {
			next = ptr->m_sibling;
			delete ptr;
		}
	}

	DG_INLINE void Factorize(dgBodyJointMatrixDataPair* const data)
	{
		const dgSpatialMatrix& bodyMass = data[m_index].m_body.m_mass;
		dgSpatialMatrix& bodyInvMass = data[m_index].m_body.m_invMass;

		if (m_body->GetInvMass().m_w != dgFloat32(0.0f)) {
			for (dgGraph* child = m_child; child; child = child->m_sibling) {
				CalculateBodyDiagonal(data, child);
			}
			bodyMass.Inverse(bodyInvMass, 6);
		} else {
			bodyInvMass.SetZero();
		}

		if (m_joint) {
			dgSpatialMatrix& bodyJt = data[m_index].m_body.m_jt;
			dgAssert(m_parent);
			for (dgInt32 i = 0; i < m_dof; i++) {
				bodyInvMass.MultiplyNxNMatrixTimeVector(bodyJt[i], bodyJt[i]);
			}
			CalculateJointDiagonal(data);
			CalculateJacobianBlock(data);
		}
	}

	DG_INLINE void CalculateInertiaMatrix(dgBodyJointMatrixDataPair* const data)
	{
		dgSpatialMatrix& bodyMass = data[m_index].m_body.m_mass;

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

	DG_INLINE void GetJacobians(const dgJointInfo* const jointInfo, const dgJacobianMatrixElement* const matrixRow, dgBodyJointMatrixDataPair* const data)
	{
		dgAssert(m_parent);
		dgAssert(jointInfo->m_joint == m_joint);
		dgAssert(jointInfo->m_joint->GetBody0() == m_body);
		dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_body);

		dgSpatialMatrix& bodyJt = data[m_index].m_body.m_jt;
		dgSpatialMatrix& jointJ = data[m_index].m_joint.m_jt;
		dgSpatialMatrix& jointMass = data[m_index].m_joint.m_mass;

		const dgInt32 start = jointInfo->m_pairStart;
		for (dgInt32 i = 0; i < m_dof; i++) {
			const dgInt32 k = m_sourceJacobianIndex[i];
			const dgJacobianMatrixElement* const row = &matrixRow[start + k];
			jointMass[i].SetZero();
			jointMass[i][i] = -row->m_stiffness;
			bodyJt[i] = dgSpatialVector (row->m_Jt.m_jacobianM0.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM0.m_angular.CompProduct4(dgVector::m_negOne));
			jointJ[i] = dgSpatialVector (row->m_Jt.m_jacobianM1.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM1.m_angular.CompProduct4(dgVector::m_negOne));
		}
	}

	DG_INLINE void EnumerateRowsLCP(const dgJointInfo* const jointInfoArray)
	{
		dgAssert (0);
		if (m_joint) {
			m_dof = 0;
			dgAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			const dgInt32 count = jointInfo->m_pairCount;
			for (dgInt32 i = 0; i < count; i++) {
				m_sourceJacobianIndex[m_dof] = dgInt8(i);
				m_dof++;
			}
		}
	}

	DG_INLINE dgInt32 Factorize(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgBodyJointMatrixDataPair* const data)
	{
		dgAssert((dgUnsigned64(data) & 0x0f) == 0);

		dgSpatialMatrix& bodyMass = data[m_index].m_body.m_mass;

		bodyMass.SetZero();
        if (m_body->GetInvMass().m_w != dgFloat32 (0.0f)) {
			CalculateInertiaMatrix(data);
		}

		m_ordinals = m_ordinalInit;
		dgInt32 boundedDof = 0;
		if (m_joint) {
			dgAssert (m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			dgAssert(jointInfo->m_joint->GetBody0() == m_body);
			dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_body);

			m_dof = 0;
			dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			for (dgInt32 i = 0; i < count; i++) {
				dgInt32 k = m_sourceJacobianIndex[i];
				const dgJacobianMatrixElement* const row = &matrixRow[k + first];
				if (((row->m_lowerBoundFrictionCoefficent <= dgFloat32 (-DG_LCP_MAX_VALUE)) && row->m_upperBoundFrictionCoefficent >= dgFloat32 (DG_LCP_MAX_VALUE))) {
					m_dof ++;
				} else {
					dgSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[count - 1]);
					i--;
					count--;
				}
			}
			boundedDof += jointInfo->m_pairCount - count;
			GetJacobians(jointInfo, matrixRow, data);
		}
		Factorize(data);
		return boundedDof;
	}

	DG_INLINE void FactorizeLCP(const dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair& accel)
	{
dgAssert (0);
/*
		dgAssert((dgUnsigned64(&m_bodyMass) & 0x0f) == 0);

		m_bodyMass.SetZero();
		if (m_body->GetInvMass().m_w != dgFloat32(0.0f)) {
			CalculateInertiaMatrix();
		}

		if (m_joint) {
			dgAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			dgAssert(jointInfo->m_joint->GetBody0() == m_body);
			dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_body);

			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
			const dgJacobian& y0 = internalForces[m0];
			const dgJacobian& y1 = internalForces[m1];
			const dgInt32 first = jointInfo->m_pairStart;
			dgInt32 clampedValue = m_dof - 1;

			//dgSpatialVector& accel = force.m_joint;
			for (dgInt32 i = 0; i < m_dof; i++) {
				dgInt32 k = m_sourceJacobianIndex[i];
				const dgJacobianMatrixElement* const row = &matrixRow[first + k];
				
				//dgFloat32 force = row->m_force + row->m_invJMinvJt * residual.GetScalar();
				dgFloat32 force = row->m_force;
				if ((force >= row->m_lowerBoundFrictionCoefficent) && (force <= row->m_upperBoundFrictionCoefficent)) {
					dgVector residual(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
									  row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
					residual = dgVector(row->m_coordenateAccel) - residual.AddHorizontal();
					accel.m_joint[i] = -residual.GetScalar();
				} else {
					dgAssert (0);
					dgSwap (m_sourceJacobianIndex[i], m_sourceJacobianIndex[clampedValue]);
					i --;
					m_dof --;
					clampedValue --;
				}
			}
			GetJacobians(jointInfo, matrixRow);
		}

		Factorize();
*/
	}

	DG_INLINE bool ValidateForcesLCP(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair& force)
	{
dgAssert (0);
/*
		dgAssert((dgUnsigned64(&m_bodyMass) & 0x0f) == 0);

		bool valid = true;
		if (m_joint) {
			dgAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			dgAssert(jointInfo->m_joint->GetBody0() == m_body);
			dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_body);

			const dgInt32 first = jointInfo->m_pairStart;

			for (dgInt32 i = 0; (i < m_dof) && valid; i++) {
				dgInt32 k = m_sourceJacobianIndex[i];
				const dgJacobianMatrixElement* const row = &matrixRow[first + k];

				dgFloat32 f = force.m_joint[i] + row->m_force;
				valid = valid && (f >= row->m_lowerBoundFrictionCoefficent);
				valid = valid && (f <= row->m_upperBoundFrictionCoefficent);
			}
		}
		return valid;
*/
	}

	DG_INLINE void UpdateFactorizeLCP(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgForcePair& force, dgForcePair& accel)
	{
dgAssert (0);
/*
		dgAssert((dgUnsigned64(&m_bodyMass) & 0x0f) == 0);

		m_bodyMass.SetZero();
		if (m_body->GetInvMass().m_w != dgFloat32(0.0f)) {
			CalculateInertiaMatrix();
		}

		if (m_joint) {
			dgAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dgAssert(jointInfo->m_joint == m_joint);
			dgAssert(jointInfo->m_joint->GetBody0() == m_body);
			dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_body);

			const dgInt32 first = jointInfo->m_pairStart;
			dgInt32 clampedValue = m_dof - 1;

			for (dgInt32 i = 0; i < m_dof; i++) {
				dgInt32 k = m_sourceJacobianIndex[i];
				const dgJacobianMatrixElement* const row = &matrixRow[first + k];
				
				dgFloat32 f = force.m_joint[i] + row->m_force;
				if (f < row->m_lowerBoundFrictionCoefficent) {
					force.m_joint[i] = dgFloat32 (0.0f);
					dgSwap(accel.m_joint[i], accel.m_joint[clampedValue]);
					dgSwap(force.m_joint[i], force.m_joint[clampedValue]);
					dgSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[clampedValue]);
					i--;
					m_dof--;
					clampedValue--;
				} else if (f > row->m_upperBoundFrictionCoefficent) {
					force.m_joint[i] = dgFloat32 (0.0f);
					dgSwap(accel.m_joint[i], accel.m_joint[clampedValue]);
					dgSwap(force.m_joint[i], force.m_joint[clampedValue]);
					dgSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[clampedValue]);
					i--;
					m_dof--;
					clampedValue--;
				}
			}
			GetJacobians(jointInfo, matrixRow);
		}

		Factorize();
*/
	}


	DG_INLINE void CalculateBodyDiagonal(dgBodyJointMatrixDataPair* const data, dgGraph* const child)
	{
		dgAssert(child->m_joint);
		
		dgSpatialMatrix copy;
		copy.SetZero();
		const dgInt32 dof = child->m_dof;
		//const dgSpatialMatrix& jacobianMatrix = child->m_jointJ;
		//const dgSpatialMatrix& childDiagonal = child->m_jointMass;
		const dgSpatialMatrix& jacobianMatrix = data[child->m_index].m_joint.m_jt;
		const dgSpatialMatrix& childDiagonal = data[child->m_index].m_joint.m_mass;
		for (dgInt32 i = 0; i < dof ; i++) {
			const dgSpatialVector& jacobian = jacobianMatrix[i];
			for (dgInt32 j = 0; j < dof ; j++) {
				dgAssert(dgAreEqual (childDiagonal[i][j], childDiagonal[j][i], dgFloat32(1.0e-5f)));
				dgFloat32 val = childDiagonal[i][j];
				jacobian.ScaleAdd(val, copy[j], copy[j]);
			}
		}

		dgSpatialMatrix& bodyMass = data[m_index].m_body.m_mass;
		for (dgInt32 i = 0; i < dof; i++) {
			const dgSpatialVector& Jacobian = copy[i];
			const dgSpatialVector& JacobianTranspose = jacobianMatrix[i];
			for (dgInt32 j = 0; j < 6; j++) {
				dgFloat32 val = -Jacobian[j];
				JacobianTranspose.ScaleAdd(val, bodyMass[j], bodyMass[j]);
			}
		}
	}

	DG_INLINE void CalculateJointDiagonal (dgBodyJointMatrixDataPair* const data)
	{
		const dgSpatialMatrix& bodyMass = data[m_index].m_body.m_mass;
		const dgSpatialMatrix& bodyJt = data[m_index].m_body.m_jt;

		dgSpatialMatrix tmp;
		for (dgInt32 i = 0; i < m_dof; i++) {
			bodyMass.MultiplyNxNMatrixTimeVector(bodyJt[i], tmp[i]);
		}

		dgSpatialMatrix& jointMass = data[m_index].m_joint.m_mass;
		for (dgInt32 i = 0; i < m_dof; i++) {
			dgFloat32 a = bodyJt[i].DotProduct(tmp[i]);
			jointMass[i][i] -= a;
			for (dgInt32 j = i + 1; j < m_dof; j++) {
				a = - bodyJt[i].DotProduct(tmp[j]);
				jointMass[i][j] = a;
				jointMass[j][i] = a;
			}
		}
		dgSpatialMatrix& jointInvMass = data[m_index].m_joint.m_invMass;
		jointMass.Inverse(jointInvMass, m_dof);
	}

	DG_INLINE void CalculateJacobianBlock(dgBodyJointMatrixDataPair* const data)
	{
		dgSpatialMatrix& jointJ = data[m_index].m_joint.m_jt;

		dgSpatialMatrix copy;
		for (dgInt32 i = 0; i < m_dof; i++) {
			copy[i] = jointJ[i];
			jointJ[i].SetZero();
		}

		const dgSpatialMatrix& jointInvMass = data[m_index].m_joint.m_invMass;
		for (dgInt32 i = 0; i < m_dof; i++) {
			const dgSpatialVector& jacobian = copy[i];
			const dgSpatialVector& invDiagonalRow = jointInvMass[i];
			for (dgInt32 j = 0; j < m_dof; j++) {
				dgFloat32 val = invDiagonalRow[j];
				jacobian.ScaleAdd(val, jointJ[j], jointJ[j]);
			}
		}
	}

	DG_INLINE void JointJacobianTimeMassForward (const dgBodyJointMatrixDataPair* const data, dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = data[m_index].m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
		}
	}

	DG_INLINE void BodyJacobianTimeMassForward(const dgBodyJointMatrixDataPair* const data, const dgForcePair& force, dgForcePair& parentForce) const 
	{
		const dgSpatialMatrix& jointJ = data[m_index].m_joint.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			jointJ[i].ScaleAdd(-force.m_joint[i], parentForce.m_body, parentForce.m_body);
		}
	}

	DG_INLINE void JointJacobianTimeSolutionBackward(const dgBodyJointMatrixDataPair* const data, dgForcePair& force, const dgForcePair& parentForce)
	{
		const dgSpatialMatrix& jointJ = data[m_index].m_joint.m_jt;
		const dgSpatialVector& f = parentForce.m_body;
		for (dgInt32 i = 0; i < m_dof; i++) {
			force.m_joint[i] -= f.DotProduct(jointJ[i]);
		}
	}

	DG_INLINE void BodyJacobianTimeSolutionBackward(const dgBodyJointMatrixDataPair* const data, dgForcePair& force)
	{
		const dgSpatialMatrix& bodyJt = data[m_index].m_body.m_jt;
		for (dgInt32 i = 0; i < m_dof; i++) {
			bodyJt[i].ScaleAdd(-force.m_joint[i], force.m_body, force.m_body);
		}

	}

	DG_INLINE void BodyDiagInvTimeSolution(const dgBodyJointMatrixDataPair* const data, dgForcePair& force)
	{
		const dgSpatialMatrix& bodyInvMass = data[m_index].m_body.m_invMass;
		bodyInvMass.MultiplyNxNMatrixTimeVector(force.m_body, force.m_body);
	}

	DG_INLINE void JointDiagInvTimeSolution(const dgBodyJointMatrixDataPair* const data, dgForcePair& force)
	{
		const dgSpatialMatrix& jointInvMass = data[m_index].m_joint.m_invMass;
		jointInvMass.MultiplyNxNMatrixTimeVector (force.m_joint, force.m_joint, m_dof);
	}

//	dgSpatialMatrix m_bodyJt;
//	dgSpatialMatrix m_bodyMass;
//	dgSpatialMatrix m_bodyInvMass;

//	dgSpatialMatrix m_jointJ;
//	dgSpatialMatrix m_jointMass;
//	dgSpatialMatrix m_jointInvMass;

//	dgMatriData* m_bodyData;
//	dgMatriData* m_jointData;
	
	dgDynamicBody* m_body;
	dgBilateralConstraint* m_joint;
	dgGraph* m_parent;
	dgGraph* m_child;
	dgGraph* m_sibling;
	dgInt16 m_index;
	dgInt16 m_dof;
	dgInt16 m_primaryStart;
	dgInt16 m_auxiliaryStart;
	union {
		dgInt8 m_sourceJacobianIndex[8];
		dgInt64 m_ordinals;
	};

	static dgInt64 m_ordinalInit;
};

dgInt64 dgSkeletonContainer::dgGraph::m_ordinalInit = 0x050403020100ll;

dgSkeletonContainer::dgSkeletonContainer(dgWorld* const world, dgDynamicBody* const rootBody)
	:m_world(world)
	,m_skeleton(new (rootBody->GetWorld()->GetAllocator()) dgGraph(rootBody, NULL, NULL))
	,m_nodesOrder(NULL)
	,m_destructor(NULL)
	,m_id(m_uniqueID)
	,m_lru(0)
	,m_nodeCount(1)
	,m_rowCount(0)
	,m_auxiliaryRowCount(0)
{
//SetSolverMode(true);

	rootBody->SetSkeleton(this);
	m_uniqueID++;
}

dgSkeletonContainer::~dgSkeletonContainer()
{
	if (m_destructor) {
		m_destructor (this);
	}
	
	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	if (m_nodesOrder) {
		allocator->Free(m_nodesOrder);
	}
	delete m_skeleton;
}

dgSkeletonContainer::dgGraph* dgSkeletonContainer::GetRoot () const
{
	return m_skeleton;
}

dgSkeletonContainer::dgGraph* dgSkeletonContainer::GetParent (dgGraph* const node) const
{
	return node->m_parent;
}

dgBody* dgSkeletonContainer::GetBody(dgSkeletonContainer::dgGraph* const node) const
{
	return node->m_body;
}

dgBilateralConstraint* dgSkeletonContainer::GetParentJoint(dgSkeletonContainer::dgGraph* const node) const
{
	return node->m_joint;
}


dgSkeletonContainer::dgGraph* dgSkeletonContainer::GetFirstChild(dgSkeletonContainer::dgGraph* const parent) const
{
	return parent->m_child;
}

dgSkeletonContainer::dgGraph* dgSkeletonContainer::GetNextSiblingChild(dgSkeletonContainer::dgGraph* const sibling) const
{
	return sibling->m_sibling;
}


dgWorld* dgSkeletonContainer::GetWorld() const
{
	return m_world;
}

void dgSkeletonContainer::SetDestructorCallback (dgOnSkeletonContainerDestroyCallback destructor)
{
	m_destructor = destructor;
}

void dgSkeletonContainer::ResetUniqueId(dgInt32 id)
{
	m_uniqueID = id;
}

void dgSkeletonContainer::SortGraph(dgGraph* const root, dgGraph* const parent, dgInt32& index)
{
	for (dgGraph* node = root->m_child; node; node = node->m_sibling) {
		SortGraph(node, root, index);
	}

	dgAssert((m_nodeCount - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_index = dgInt16(index);
	index++;
	if (root->m_joint) {
		root->m_joint->m_hasSkeleton = true;
	}
	dgAssert(index <= m_nodeCount);
}

dgSkeletonContainer::dgGraph* dgSkeletonContainer::FindNode(dgDynamicBody* const body) const
{
	dgInt32 stack = 1;
	dgGraph* stackPool[1024];

	stackPool[0] = m_skeleton;
	while (stack) {
		stack--;
		dgGraph* const node = stackPool[stack];
		if (node->m_body == body) {
			return node;
		}

		for (dgGraph* ptr = node->m_child; ptr; ptr = ptr->m_sibling) {
			stackPool[stack] = ptr;
			stack++;
			dgAssert(stack < dgInt32(sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return NULL;
}

dgSkeletonContainer::dgGraph* dgSkeletonContainer::AddChild(dgBody* const child, dgBody* const parent)
{
	dgAssert(child);
	dgBody* const parentBody = parent ? parent : m_skeleton->m_body;
	dgAssert(parentBody);
	dgAssert(child->GetType() == dgBody::m_dynamicBody);
	dgAssert(parentBody->GetType() == dgBody::m_dynamicBody);
	return AddChild((dgDynamicBody*)child, (dgDynamicBody*)parentBody);
}

dgSkeletonContainer::dgGraph* dgSkeletonContainer::AddChild(dgDynamicBody* const child, dgDynamicBody* const parent)
{
	dgAssert (m_skeleton->m_body);
	dgBilateralConstraint* const joint = m_world->FindBilateralJoint(child, parent);
	dgAssert(joint);

	dgGraph* node = NULL;
	dgMemoryAllocator* const allocator = m_world->GetAllocator();
	if ((joint->GetBody0() == child) && (joint->GetBody1() == parent)) {
		dgGraph* const parentNode = FindNode(parent);
		dgAssert(parentNode);
		node = new (allocator)dgGraph(child, joint, parentNode);
	} else {
		dgAssert(joint->GetBody1() == child);
		dgAssert(joint->GetBody0() == parent);
		dgAssert (m_skeleton->m_body == parent);
		dgAssert (m_skeleton->m_joint == NULL);
		dgAssert (m_skeleton->m_sibling == NULL);
		m_skeleton->m_joint = joint;
		node = new (allocator) dgGraph (child, NULL, NULL);
		node->m_child = m_skeleton;
		m_skeleton->m_parent = node;
		dgSwap (m_skeleton, node);
	}

	dgAssert(node->m_joint->GetBody0() == node->m_body);
	dgAssert(node->m_joint->GetBody1() == node->m_parent->m_body);
	m_nodeCount ++;

	dgAssert (child->GetWorld()->GetSentinelBody() != child);
	child->SetSkeleton(this);
	
	return node;
}


void dgSkeletonContainer::AddJointList (dgInt32 count, dgBilateralConstraint** const array)
{
	dgTree<dgBody*, dgBody*> filter(m_world->GetAllocator());
	dgTree<dgConstraint*, dgConstraint*> jointMap(m_world->GetAllocator());

	dgInt32 stack = 0;
	dgBody* pool[1024][2];
	filter.Insert(m_skeleton->m_body, m_skeleton->m_body);
	for (dgInt32 i = 0; i < count; i++) {
		dgBilateralConstraint* const joint = array[i];
		jointMap.Insert(joint, joint);

		dgBody* const body0 = joint->GetBody0();
		dgBody* const body1 = joint->GetBody1();
		if (body1 == m_skeleton->m_body) {
			pool[stack][0] = joint->GetBody0();
			pool[stack][1] = joint->GetBody1();
			filter.Insert(pool[stack][0], pool[stack][0]);
			stack++;
		} else if (body0 == m_skeleton->m_body) {
			pool[stack][0] = joint->GetBody1();
			pool[stack][1] = joint->GetBody0();
			filter.Insert(pool[stack][0], pool[stack][0]);
			stack++;
		}
	}

	while (stack) {
		stack--;
		dgBody* const child = pool[stack][0];
		dgBody* const parent = pool[stack][1];
		AddChild(child, parent);

		for (dgConstraint* joint = child->GetFirstJoint(); joint; joint = child->GetNextJoint(joint)) {
			dgAssert(joint->IsBilateral());
			if (jointMap.Find(joint)) {
				dgBody* const body = (joint->GetBody0() != child) ? joint->GetBody0() : joint->GetBody1();
				if (!filter.Find(body)) {
					pool[stack][0] = body;
					pool[stack][1] = child;
					stack++;
					filter.Insert(body, body);
					dgAssert(stack < sizeof (pool) / (2 * sizeof (pool[0][0])));
				}
			}
		}
	}
}


void dgSkeletonContainer::Finalize()
{
	dgAssert(m_nodeCount >= 1);

	dgMemoryAllocator* const allocator = m_skeleton->m_body->GetWorld()->GetAllocator();
	m_nodesOrder = (dgGraph**)allocator->Malloc(m_nodeCount * sizeof (dgGraph*));

	dgInt32 index = 0;
	SortGraph(m_skeleton, NULL, index);
	dgAssert(index == m_nodeCount);
}


DG_INLINE void dgSkeletonContainer::InitMassMatrix(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgBodyJointMatrixDataPair* const data)
{
	dgInt32 rowCount = 0;
	dgInt32 primaryStart = 0;
	dgInt32 auxiliaryStart = 0;
	
	if (m_nodesOrder) {
		for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
			dgGraph* const node = m_nodesOrder[i];
			rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			node->m_auxiliaryStart = dgInt16 (auxiliaryStart);
			node->m_primaryStart = dgInt16 (primaryStart);
			auxiliaryStart += node->Factorize(jointInfoArray, matrixRow, data);
			primaryStart += node->m_dof;
		}
		m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, matrixRow, data);
	}
	m_rowCount = dgInt16 (rowCount);
	m_auxiliaryRowCount = dgInt16 (auxiliaryStart);
}


DG_INLINE void dgSkeletonContainer::SolveFoward (dgForcePair* const force, const dgForcePair* const accel, const dgBodyJointMatrixDataPair* const data) const
{
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dgGraph* const node = m_nodesOrder[i];
		dgAssert(node->m_joint);
		dgAssert(node->m_index == i);
		dgForcePair& f = force[i];
		const dgForcePair& a = accel[i];
		f.m_body = a.m_body;
		for (dgInt32 j = 0; j < node->m_dof; j ++) {
			f.m_joint[j] = a.m_joint[j]; 
		}
		for (dgGraph* child = node->m_child; child; child = child->m_sibling) {
			dgAssert(child->m_joint);
			dgAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(data, force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(data, f);
	}

	force[m_nodeCount - 1] = accel[m_nodeCount - 1];
	for (dgGraph* child = m_nodesOrder[m_nodeCount - 1]->m_child; child; child = child->m_sibling) {
		child->BodyJacobianTimeMassForward(data, force[child->m_index], force[child->m_parent->m_index]);
	}
}

DG_INLINE void dgSkeletonContainer::SolveBackward (dgForcePair* const force, const dgBodyJointMatrixDataPair* const data) const
{
	m_nodesOrder[m_nodeCount - 1]->BodyDiagInvTimeSolution(data, force[m_nodeCount - 1]);
	for (dgInt32 i = m_nodeCount - 2; i >= 0; i--) {
		dgGraph* const node = m_nodesOrder[i];
		dgAssert (node->m_index == i);
		dgForcePair& f = force[i];
		node->JointDiagInvTimeSolution(data, f);
		node->JointJacobianTimeSolutionBackward(data, f, force[node->m_parent->m_index]);
		node->BodyDiagInvTimeSolution(data, f);
		node->BodyJacobianTimeSolutionBackward(data, f);
	}
}

		


DG_INLINE void dgSkeletonContainer::UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force) const
{
	dgVector zero (dgVector::m_zero);
	for (dgInt32 i = 0; i < (m_nodeCount - 1)  ; i ++) {
		dgGraph* const node = m_nodesOrder[i];
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
//		const dgInt32 count = jointInfo->m_pairCount;
const dgInt32 count = node->m_dof;
		for (dgInt32 j = 0; j < count; j ++) {
			const dgInt32 k = node->m_sourceJacobianIndex[j];
			dgJacobianMatrixElement* const row = &matrixRow[first + k];

			row->m_force += f[j];
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

void dgSkeletonContainer::XXXX(dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const force)
{
	dgInt32 size = 0;
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dgGraph* const node = m_nodesOrder[i];
		dgAssert(i == node->m_index);
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		size += jointInfo->m_pairCount;
	}

	dgFloat32* const damp = dgAlloca (dgFloat32, size);
	dgFloat32* const b = dgAlloca (dgFloat32, size);
	dgFloat32* const x = dgAlloca (dgFloat32, size);
	dgFloat32* const low = dgAlloca (dgFloat32, size);
	dgFloat32* const high = dgAlloca (dgFloat32, size);

	dgFloat32* const matrix = dgAlloca (dgFloat32, size * size);
	dgJacobian* const Jt = dgAlloca (dgJacobian, size * m_nodeCount);
	dgJacobian* const JinvM = dgAlloca (dgJacobian, size * m_nodeCount);

	memset (matrix, 0, size * size * sizeof (dgFloat32));
	memset (Jt, 0, size * m_nodeCount * sizeof (dgJacobian));
	memset (JinvM, 0, size * m_nodeCount * sizeof (dgJacobian));

	dgJacobian* jtPtr = Jt; 
	dgJacobian* JinvMPtr = JinvM; 
	dgInt32 index = 0;
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dgGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (dgInt32 j = 0; j < count; j++) {
			const dgInt32 k = node->m_sourceJacobianIndex[j];
			dgJacobianMatrixElement* const row = &matrixRow[first + k];

			dgFloat32 force = row->m_force;
			dgVector residual(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
							  row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			residual = dgVector(row->m_coordenateAccel) - residual.AddHorizontal();
			b[index] = residual.GetScalar();
			x[index] = dgFloat32 (0.0f);
			damp[index] = row->m_diagDamp;
			low[index] = -DG_LCP_MAX_VALUE;
			high[index] = DG_LCP_MAX_VALUE;
			if (row->m_lowerBoundFrictionCoefficent > dgFloat32 (-1.0e9f)) {
				low[index] = row->m_lowerBoundFrictionCoefficent - force;
			}
			if (row->m_upperBoundFrictionCoefficent < dgFloat32 (1.0e9f)) {
				high[index] = row->m_upperBoundFrictionCoefficent - force;
			}

			JinvMPtr[node->m_index] = row->m_JMinv.m_jacobianM0;
			JinvMPtr[node->m_parent->m_index] = row->m_JMinv.m_jacobianM1;

			jtPtr[node->m_index] = row->m_Jt.m_jacobianM0;
			jtPtr[node->m_parent->m_index] = row->m_Jt.m_jacobianM1;

			
			index ++;
			jtPtr += m_nodeCount;
			JinvMPtr += m_nodeCount;
		}
	}

	JinvMPtr = JinvM; 
	for (dgInt32 i = 0; i < size; i++) {
		jtPtr = Jt + m_nodeCount * i; 
		dgVector acc (dgVector::m_zero);
		for (dgInt32 k = 0; k < m_nodeCount; k++) {
			acc = acc + JinvMPtr[k].m_linear.CompProduct4(jtPtr[k].m_linear) + JinvMPtr[k].m_angular.CompProduct4(jtPtr[k].m_angular);
		}
		acc = acc.AddHorizontal();
		dgFloat32 val = acc.GetScalar() + damp[i];
		matrix[i * size + i] = val;

		jtPtr += m_nodeCount; 
		for (dgInt32 j = i + 1; j < size; j++) {
			acc = dgVector::m_zero;
			for (dgInt32 k = 0; k < m_nodeCount; k++) {
				acc = acc + JinvMPtr[k].m_linear.CompProduct4(jtPtr[k].m_linear) + JinvMPtr[k].m_angular.CompProduct4(jtPtr[k].m_angular);
			}
			acc = acc.AddHorizontal();
			val = acc.GetScalar();
			matrix[i * size + j] = val;
			matrix[j * size + i] = val;
			jtPtr += m_nodeCount;
		}
		JinvMPtr += m_nodeCount;
	}

	for (dgInt32 i = m_rowCount - m_auxiliaryRowCount; i < size; i++) {
		dgFloat32* const row = &matrix[i * size];
		for (dgInt32 j = 0; j < m_rowCount - m_auxiliaryRowCount; j++) {
			dgTrace (("%5.2f ", row[j]));
		}
		dgTrace (("\n"));
	}

	dgTrace (("\n"));
	for (dgInt32 i = m_rowCount - m_auxiliaryRowCount; i < size; i++) {
		dgFloat32* const row = &matrix[i * size];
		for (dgInt32 j = m_rowCount - m_auxiliaryRowCount; j < size; j++) {
			dgTrace(("%5.2f ", row[j]));
		}
		dgTrace(("\n"));
	}

/*	
	dgSolveDantzigLCP(size, matrix, x, b, low, high);

	index = 0;
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dgGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		const dgInt32 count = jointInfo->m_pairCount;
		dgSpatialVector& f = force[i].m_joint;
		for (dgInt32 j = 0; j < count; j++) {
			f[j] = x[index];
			index ++;
		}
	}
*/
}

DG_INLINE void dgSkeletonContainer::CalculateJointAccel(dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const
{
	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		dgGraph* const node = m_nodesOrder[i];
		dgAssert(i == node->m_index);

		dgForcePair& a = accel[i];
		dgAssert(node->m_body);
		a.m_body.SetZero();
		a.m_joint.SetZero();

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
			dgJacobianMatrixElement* const row = &matrixRow[first + k];
			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			acc = dgVector(row->m_coordenateAccel) - acc.AddHorizontal();
			a.m_joint[j] = -acc.GetScalar();
		}
	}
	dgAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body.SetZero();
	accel[m_nodeCount - 1].m_joint.SetZero();
}


DG_INLINE void dgSkeletonContainer::BuildAuxiliaryMassMatrix(dgFloat32* const massMatrix01, dgFloat32* const massMatrix11, dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, const dgJacobianMatrixElement* const matrixRow) const
{
	dTimeTrackerEvent(__FUNCTION__);

	for (dgInt32 i = 0; i < m_nodeCount - 1; i++) {
		const dgGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		const dgInt32 primaryRowStart = node->m_primaryStart; 
		const dgInt32 auxiliaryRowStart = node->m_auxiliaryStart; 
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 primaryCount = node->m_dof;
		const dgInt32 auxiliaryCount = jointInfo->m_pairCount - primaryCount;

		for (dgInt32 j = 0; j < auxiliaryCount; j ++) {
			const dgInt32 index = node->m_sourceJacobianIndex[j + node->m_dof];
			const dgJacobianMatrixElement* const row = &matrixRow[first + index];
			dgJacobian JMinvM0 (row->m_JMinv.m_jacobianM0);
			dgJacobian JMinvM1 (row->m_JMinv.m_jacobianM1);

			dgFloat32* const matrixRow11 = &massMatrix11[(j + auxiliaryRowStart) * m_auxiliaryRowCount];
			dgVector acc(JMinvM0.m_linear.CompProduct4(row->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row->m_Jt.m_jacobianM0.m_angular) +
						 JMinvM1.m_linear.CompProduct4(row->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row->m_Jt.m_jacobianM1.m_linear));
			acc = acc.AddHorizontal();
			matrixRow11[j + auxiliaryRowStart] = acc.GetScalar() + row->m_diagDamp;
			for (dgInt32 k = j + 1; k < auxiliaryCount; k++) {
				const dgInt32 index = node->m_sourceJacobianIndex[k + primaryCount];
				const dgJacobianMatrixElement* const rowB = &matrixRow[first + index];
				dgVector acc(JMinvM0.m_linear.CompProduct4(rowB->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(rowB->m_Jt.m_jacobianM0.m_angular) +
							 JMinvM1.m_linear.CompProduct4(rowB->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(rowB->m_Jt.m_jacobianM1.m_linear));
				dgFloat32 a = acc.AddHorizontal().GetScalar();
				matrixRow11[k + auxiliaryRowStart] = a;
				massMatrix11[(k + auxiliaryRowStart) * m_auxiliaryRowCount + j + auxiliaryRowStart] = a;
			}

			dgFloat32* const matrixRow01 = &massMatrix01[(j + auxiliaryRowStart) * (m_rowCount - m_auxiliaryRowCount)];
			for (dgInt32 k = 0; k < primaryCount; k++) {
				const dgInt32 index = node->m_sourceJacobianIndex[k];
				const dgJacobianMatrixElement* const rowB = &matrixRow[first + index];
				dgVector acc(JMinvM0.m_linear.CompProduct4(rowB->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(rowB->m_Jt.m_jacobianM0.m_angular) +
							 JMinvM1.m_linear.CompProduct4(rowB->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(rowB->m_Jt.m_jacobianM1.m_linear));
				dgFloat32 a = acc.AddHorizontal().GetScalar();
				matrixRow01[k + primaryRowStart] = a;
			}

			const dgGraph* const parentNode = node->m_parent;
			if (parentNode->m_joint) {
				const dgJointInfo* const parentJointInfo = &jointInfoArray[parentNode->m_joint->m_index];
				const dgInt32 parentFirst = parentJointInfo->m_pairStart;
				const dgInt32 parentPrimaryCount = parentNode->m_dof;
				const dgInt32 parentAuxiliaryCount = parentJointInfo->m_pairCount - parentPrimaryCount;

				const dgInt32 parentAuxiliaryRowStart = parentNode->m_auxiliaryStart;
				for (dgInt32 k = 0; k < parentAuxiliaryCount; k ++) {
					const dgInt32 index = parentNode->m_sourceJacobianIndex[k + parentPrimaryCount];
					const dgJacobianMatrixElement* const parentRow = &matrixRow[parentFirst + index];
					dgVector acc(JMinvM1.m_linear.CompProduct4(parentRow->m_Jt.m_jacobianM0.m_linear) + JMinvM1.m_angular.CompProduct4(parentRow->m_Jt.m_jacobianM0.m_linear));
					dgFloat32 a = acc.AddHorizontal().GetScalar();
					matrixRow11[k + parentAuxiliaryRowStart] = a;
					massMatrix11[(k + parentAuxiliaryRowStart) * m_auxiliaryRowCount + j + auxiliaryRowStart] = a;
				}

				const dgInt32 parentPrimaryRowStart = parentNode->m_primaryStart;
				for (dgInt32 k = 0; k < parentPrimaryCount; k++) {
					const dgInt32 index = parentNode->m_sourceJacobianIndex[k];
					const dgJacobianMatrixElement* const parentRow = &matrixRow[parentFirst + index];
					dgVector acc(JMinvM1.m_linear.CompProduct4(parentRow->m_Jt.m_jacobianM0.m_linear) + JMinvM1.m_angular.CompProduct4(parentRow->m_Jt.m_jacobianM0.m_linear));
					dgFloat32 a = acc.AddHorizontal().GetScalar();
					matrixRow01[k + parentPrimaryRowStart] = a;
				}
			}

			for (const dgGraph* childNode = node->m_child; childNode; childNode = childNode->m_sibling) {
				const dgJointInfo* const childJointInfo = &jointInfoArray[childNode->m_joint->m_index];
				const dgInt32 childFirst = childJointInfo->m_pairStart;
				const dgInt32 childPrimaryCount = childNode->m_dof;

				const dgInt32 childPrimaryRowStart = childNode->m_primaryStart;
				for (dgInt32 k = 0; k < childPrimaryCount; k++) {
					const dgInt32 index = parentNode->m_sourceJacobianIndex[k];
					const dgJacobianMatrixElement* const childRow = &matrixRow[childFirst + index];
					dgVector acc(JMinvM0.m_linear.CompProduct4(childRow->m_Jt.m_jacobianM1.m_linear) + JMinvM0.m_angular.CompProduct4(childRow->m_Jt.m_jacobianM1.m_linear));
					dgFloat32 a = acc.AddHorizontal().GetScalar();
					matrixRow01[k + childPrimaryRowStart] = a;
				}
			}
		}
	}
}

void dgSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow)
{
	dTimeTrackerEvent(__FUNCTION__);

{
	const int xxx = 4;

	dgFloat32 x[xxx];
	dgFloat32 b[xxx];
	dgFloat32 low[xxx];
	dgFloat32 high[xxx];
	dgFloat32 a[xxx][xxx];

	memset(a, 0, sizeof (a));
	for (int i = 0; i < xxx; i++) {
		b[i] = 18.0f;
		x[i] = 1.8f;
		low[i] = -DG_LCP_MAX_VALUE;
		high[i] = DG_LCP_MAX_VALUE;
		a[i][i] = 3.0f;
		for (int j = i + 1; j < xxx; j++) {
			a[i][j] = 2.0f;
			a[j][i] = 2.0f;
		}
	}
	high[2] = 10.5f;
	high[3] = 10.2f;

	//	dgSolveDantzigLCP(xxx, &a[0][0], x, b, low, high);
	dgSolveBlockDantzigLCP(xxx, &a[0][0], x, b, low, high);
	low[4] = -10.0f;
}


{
	const int xxx = 6;

	dgFloat32 x[xxx];
	dgFloat32 b[xxx];
	dgFloat32 low[xxx];
	dgFloat32 high[xxx];
	dgFloat32 a[xxx][xxx];

	memset(a, 0, sizeof (a));
	for (int i = 0; i < xxx; i++) {
		b[i] = 0.0f;
		x[i] = 0.0f;
		low[i] = -DG_LCP_MAX_VALUE;
		high[i] = DG_LCP_MAX_VALUE;
		a[i][i] = 2.0f;
	}
	for (int i = 0; i < xxx - 1; i++) {
		a[i][i + 1] = -1.0f;
		a[i + 1][i] = -1.0f;
	}
	b[xxx - 1] = 14.0f;
	low[3] = -10.0f;
	low[4] = -10.0f;
	low[5] = -10.0f;

	//dgSolveDantzigLCP(xxx, &a[0][0], x, b, low, high);
	dgSolveBlockDantzigLCP(xxx, &a[0][0], x, b, low, high);
	low[4] = -10.0f;
}


	dgForcePair* const force = dgAlloca(dgForcePair, m_nodeCount);
	dgBodyJointMatrixDataPair* const data = dgAlloca(dgBodyJointMatrixDataPair, m_nodeCount);

	InitMassMatrix(jointInfoArray, matrixRow, data);
	CalculateJointAccel(jointInfoArray, internalForces, matrixRow, force);
	SolveFoward(force, force, data);
	SolveBackward(force, data);
	if (m_auxiliaryRowCount) {
		dgFloat32* const massMatrix11 = dgAlloca (dgFloat32, m_auxiliaryRowCount * m_auxiliaryRowCount);
		dgFloat32* const massMatrix01 = dgAlloca (dgFloat32, (m_rowCount - m_auxiliaryRowCount) * m_auxiliaryRowCount);
		memset (massMatrix11, 0, m_auxiliaryRowCount * m_auxiliaryRowCount * sizeof (dgFloat32));
		memset (massMatrix01, 0, m_auxiliaryRowCount * (m_rowCount - m_auxiliaryRowCount) * m_auxiliaryRowCount * sizeof (dgFloat32));


dgForcePair* const f = dgAlloca(dgForcePair, m_nodeCount);
XXXX(jointInfoArray, internalForces, matrixRow, f);

		BuildAuxiliaryMassMatrix(massMatrix11, massMatrix01, jointInfoArray, internalForces, matrixRow);

	}
	UpdateForces(jointInfoArray, internalForces, matrixRow, force);
}



