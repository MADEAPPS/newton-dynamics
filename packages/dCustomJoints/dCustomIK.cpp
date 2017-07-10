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

#include "dCustomJointLibraryStdAfx.h"
#include "dCustomIK.h"


/*
#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dCustomIKSolver.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"

class dCustomIKSolver::dgNodePair
{
	public:
	int m_m0;
	int m_m1;
};

DG_MSC_VECTOR_ALIGMENT
class dCustomIKSolver::dgForcePair
{
	public:
	dSpatialVector m_joint;
	dSpatialVector m_body;
} DG_GCC_VECTOR_ALIGMENT;




int dCustomIKSolver::m_lruMarker = 1;
int dCustomIKSolver::m_uniqueID = DG_SKELETON_BASE_UNIQUE_ID;
*/

/*
class dCustomIKSolver::dGraph
{
	public:

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
		dAssert (m_parent);
		dAssert (m_body->GetInvMass().m_w != dFloat (0.0f));
		if (m_parent->m_child) {
			m_sibling = m_parent->m_child;
		}
		m_parent->m_child = this;
	}

	inline void JointJacobianTimeMassForward (dgForcePair& force)
	{
		const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (int i = 0; i < m_dof; i++) {
			force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
		}
	}

	inline void BodyJacobianTimeMassForward(const dgForcePair& force, dgForcePair& parentForce) const 
	{
		const dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		for (int i = 0; i < m_dof; i++) {
			parentForce.m_body = parentForce.m_body + jointJ[i].Scale(-force.m_joint[i]);
		}
	}

	inline void JointJacobianTimeSolutionBackward(dgForcePair& force, const dgForcePair& parentForce)
	{
		const dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
		const dSpatialVector& f = parentForce.m_body;
		for (int i = 0; i < m_dof; i++) {
			force.m_joint[i] -= f.DotProduct(jointJ[i]);
		}
	}

	inline void BodyJacobianTimeSolutionBackward(dgForcePair& force)
	{
		const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		for (int i = 0; i < m_dof; i++) {
			force.m_body = force.m_body + bodyJt[i].Scale(-force.m_joint[i]);
		}
	}

	inline void BodyDiagInvTimeSolution(dgForcePair& force)
	{
		const dSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
		force.m_body = bodyInvMass.VectorTimeMatrix(force.m_body);
	}

	inline void JointDiagInvTimeSolution(dgForcePair& force)
	{
		const dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
		force.m_joint = jointInvMass.VectorTimeMatrix(force.m_joint, m_dof);
	}

	inline int GetAuxiliaryRows(const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const
	{
		int rowCount = 0;
		if (m_joint) {
			dAssert(m_parent);
			const dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
			dAssert(jointInfo->m_joint == m_joint);
			int count = jointInfo->m_pairCount;
			const int first = jointInfo->m_pairStart;
			for (int i = 0; i < count; i++) {
				const dgJacobianMatrixElement* const row = &matrixRow[i + first];
				if (!((row->m_lowerBoundFrictionCoefficent <= dFloat (-DG_LCP_MAX_VALUE)) && (row->m_upperBoundFrictionCoefficent >= dFloat (DG_LCP_MAX_VALUE)))) {
					rowCount++;
				}
			}
		}
		return rowCount;
	}
	
	dgBodyJointMatrixDataPair m_data;
	union
	{
		dgInt64 m_ordinals;
		dgInt8 m_sourceJacobianIndex[8];
	};
	short m_primaryStart;
	short m_auxiliaryStart;
	short m_index;
	dgInt8 m_dof;
	dgInt8 m_swapJacobianBodiesIndex;
};
*/


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

void dInverseKinematicSolver::ResetUniqueId(int id)
{
	m_uniqueID = id;
}


dInverseKinematicSolver::dGraph* dInverseKinematicSolver::FindNode(dgDynamicBody* const body) const
{
	int stack = 1;
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
			dAssert(stack < int(sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return NULL;
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
			dAssert (0);
			m_cyclingBodies.Remove(ptr);
			break;
		}
	}
}




bool dInverseKinematicSolver::SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const
{
	return true;
}

inline void dInverseKinematicSolver::CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const
{
	for (int i = 0; i < m_nodeCount - 1; i++) {
		dGraph* const node = m_nodesOrder[i];
		dAssert(node->m_joint);
		dAssert(node->m_index == i);
		dgForcePair& f = force[i];
		const dgForcePair& a = accel[i];
		f.m_body = a.m_body;
		for (int j = 0; j < node->m_dof; j ++) {
			f.m_joint[j] = a.m_joint[j]; 
		}
		for (dGraph* child = node->m_child; child; child = child->m_sibling) {
			dAssert(child->m_joint);
			dAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(f);
	}

	force[m_nodeCount - 1] = accel[m_nodeCount - 1];
	for (dGraph* child = m_nodesOrder[m_nodeCount - 1]->m_child; child; child = child->m_sibling) {
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	m_nodesOrder[m_nodeCount - 1]->BodyDiagInvTimeSolution(force[m_nodeCount - 1]);
	for (int i = m_nodeCount - 2; i >= 0; i--) {
		dGraph* const node = m_nodesOrder[i];
		dAssert(node->m_index == i);
		dgForcePair& f = force[i];
		node->JointDiagInvTimeSolution(f);
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyDiagInvTimeSolution(f);
		node->BodyJacobianTimeSolutionBackward(f);
	}
//	dAssert (SanityCheck(force, accel));
}


inline void dInverseKinematicSolver::UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force) const
{
	dgVector zero (dgVector::m_zero);
	for (int i = 0; i < (m_nodeCount - 1)  ; i ++) {
		dGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		dAssert(i == node->m_index);

		const dSpatialVector& f = force[i].m_joint;
		dAssert(jointInfo->m_joint == node->m_joint);
		const int first = jointInfo->m_pairStart;
		const int count = node->m_dof;
		for (int j = 0; j < count; j ++) {
			const int k = node->m_sourceJacobianIndex[j];
			dgJacobianMatrixElement* const row = &matrixRow[first + k];

			row->m_force += dFloat(f[j]);
			dgVector jointForce = dFloat (f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
		}

		const int m0 = jointInfo->m_m0;
		const int m1 = jointInfo->m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}
}

inline void dInverseKinematicSolver::CalculateJointAccel(dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const
{
	dSpatialVector zero(dFloat(0.0f));
	for (int i = 0; i < m_nodeCount - 1; i++) {
		dGraph* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);

		dgForcePair& a = accel[i];
		dAssert(node->m_body);
		a.m_body = zero;
		a.m_joint = zero;

		dAssert(node->m_joint);
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		dAssert(jointInfo->m_joint == node->m_joint);

		const int first = jointInfo->m_pairStart;
		const int dof = jointInfo->m_pairCount;
		const int m0 = jointInfo->m_m0;
		const int m1 = jointInfo->m_m1;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (int j = 0; j < dof; j++) {
			const int k = node->m_sourceJacobianIndex[j];
			const dgJacobianMatrixElement* const row = &matrixRow[first + k];
			dgVector diag(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						  row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			a.m_joint[j] = -(row->m_coordenateAccel - row->m_force * row->m_diagDamp - (diag.AddHorizontal()).GetScalar());
		}
	}
	dAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body = dSpatialVector(dFloat(0.0f));
	accel[m_nodeCount - 1].m_joint = dSpatialVector(dFloat(0.0f));
}


void dInverseKinematicSolver::SolveAuxiliary(const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force) const
{
	dTimeTrackerEvent(__FUNCTION__);
	
	dFloat* const f = dgAlloca(dFloat, m_rowCount);
	dFloat* const u = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const b = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const low = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const high = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const massMatrix11 = dgAlloca(dFloat, m_auxiliaryRowCount * m_auxiliaryRowCount);
	dFloat* const lowerTriangularMassMatrix11 = dgAlloca(dFloat, m_auxiliaryRowCount * m_auxiliaryRowCount);

	int primaryIndex = 0;
	int auxiliaryIndex = 0;
	const int primaryCount = m_rowCount - m_auxiliaryRowCount;
	
	for (int i = 0; i < m_nodeCount - 1; i++) { 
		const dGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		const int first = jointInfo->m_pairStart;

		const int primaryDof = node->m_dof;
		const dSpatialVector& accelSpatial = accel[i].m_joint;
		const dSpatialVector& forceSpatial = force[i].m_joint;

		for (int j = 0; j < primaryDof; j++) {
			f[primaryIndex] = dFloat(forceSpatial[j]);
			primaryIndex++;
		}

		const int auxiliaryDof = jointInfo->m_pairCount - primaryDof;
		for (int j = 0; j < auxiliaryDof; j++) {
			const int index = node->m_sourceJacobianIndex[primaryDof + j];
			dgJacobianMatrixElement* const row = &matrixRow[first + index];
			f[auxiliaryIndex + primaryCount] = dFloat (0.0f);
			b[auxiliaryIndex] = -dFloat(accelSpatial[primaryDof + j]);
			low[auxiliaryIndex] = dgClamp (row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dFloat (0.0f));
			high[auxiliaryIndex] = dgClamp (row->m_upperBoundFrictionCoefficent - row->m_force, dFloat (0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgCyclingJoint& entry = ptr->GetInfo();
		const dgConstraint* const joint = entry.m_joint;
		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];

		const int m0 = jointInfo->m_m0;
		const int m1 = jointInfo->m_m1;
		const int first = jointInfo->m_pairStart;
		const int auxiliaryDof = jointInfo->m_pairCount;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		for (int i = 0; i < auxiliaryDof; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[first + i];
			f[auxiliaryIndex + primaryCount] = dFloat (0.0f);
			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			b[auxiliaryIndex] = row->m_coordenateAccel - (acc.AddHorizontal()).GetScalar();
			low[auxiliaryIndex] = dgClamp (row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dFloat (0.0f));
			high[auxiliaryIndex] = dgClamp (row->m_upperBoundFrictionCoefficent - row->m_force, dFloat (0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	memcpy (massMatrix11, m_massMatrix11, sizeof (dFloat) * m_auxiliaryRowCount * m_auxiliaryRowCount);
	memcpy (lowerTriangularMassMatrix11, m_lowerTriangularMassMatrix11, sizeof (dFloat) * m_auxiliaryRowCount * m_auxiliaryRowCount);
	for (int i = 0; i < m_auxiliaryRowCount; i ++) {
		dFloat* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		u[i] = dFloat(0.0f);
		dFloat r = dFloat(0.0f);
		for (int j = 0; j < primaryCount; j++) {
			r += matrixRow10[j] * f[j];
		}
		b[i] -= r;
	}
	
	dgSolveDantzigLCP(m_auxiliaryRowCount, massMatrix11, lowerTriangularMassMatrix11, u, b, low, high);

	for (int i = 0; i < m_auxiliaryRowCount; i++) {
		const dFloat s = u[i];
		f[primaryCount + i] = s;
		const dFloat* const deltaForce = &m_deltaForce[i * primaryCount];
		for (int j = 0; j < primaryCount; j++) {
			f[j] += deltaForce[j] * s;
		}
	}

	for (int i = 0; i < m_rowCount; i++) {
		dgJacobianMatrixElement* const row = m_rowArray[i];
		const int m0 = m_pairs[i].m_m0;
		const int m1 = m_pairs[i].m_m1;

		row->m_force += f[i];
		dgVector jointForce(f[i]);
		internalForces[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
		internalForces[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
		internalForces[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
		internalForces[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
	}
}


int dInverseKinematicSolver::GetMemoryBufferSizeInBytes (const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const
{
	int rowCount = 0;
	int auxiliaryRowCount = 0;
	if (m_nodesOrder) {
		for (int i = 0; i < m_nodeCount - 1; i++) {
			dGraph* const node = m_nodesOrder[i];
			rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			auxiliaryRowCount += node->GetAuxiliaryRows(jointInfoArray, matrixRow);
		}
	}

	int extraAuxiliaryRows = 0;
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgCyclingJoint& entry = ptr->GetInfo();
		const dgConstraint* const joint = entry.m_joint;
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount+= extraAuxiliaryRows;

	int size = sizeof (dgJacobianMatrixElement*) * rowCount;
	size += sizeof (dgNodePair) * rowCount;
	size += sizeof (dFloat) * auxiliaryRowCount * auxiliaryRowCount * 2;
	size += sizeof (dFloat) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof (dFloat) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
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


inline void dInverseKinematicSolver::CalculateBodyAcceleration (dgJacobian* const accel, dFloat timestep) const
{
	const dgVector invTimeStep (dFloat (1.0f) / timestep);
	for (int i = 0; i < m_nodeCount; i++) {
		dGraph* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);
		dAssert(node->m_body);
		const dgDynamicBody* const body = node->m_body;
		accel[i].m_linear = (body->m_veloc - body->m_accel).CompProduct4(invTimeStep);
		accel[i].m_angular = (body->m_omega - body->m_alpha).CompProduct4(invTimeStep);
	}

	int index = m_nodeCount;
	for (dgList<dgDynamicBody*>::dgListNode* node = m_cyclingBodies.GetFirst(); node; node = node->GetNext()) {
		const dgDynamicBody* const body = node->GetInfo();
		accel[index].m_linear = (body->m_veloc - body->m_accel).CompProduct4(invTimeStep);
		accel[index].m_angular = (body->m_omega - body->m_alpha).CompProduct4(invTimeStep);
		
		#ifdef _DEBUG
			if (body->GetInvMass().m_w == dFloat (0.0f)) {
				dAssert (accel[index].m_linear.DotProduct4(accel[index].m_linear).GetScalar() == dFloat (0.0f));
				dAssert (accel[index].m_angular.DotProduct4(accel[index].m_angular).GetScalar() == dFloat (0.0f));
			}
		#endif
		index ++;
	}

#ifdef _DEBUG
	if (m_skeleton->m_body->GetInvMass().m_w == dFloat (0.0f)) {
		dAssert (accel[m_nodeCount - 1].m_linear.DotProduct4(accel[m_nodeCount - 1].m_linear).GetScalar() == dFloat (0.0f));
		dAssert (accel[m_nodeCount - 1].m_angular.DotProduct4(accel[m_nodeCount - 1].m_angular).GetScalar() == dFloat (0.0f));
	}
#endif
}


inline void dCustomIKSolver::CalculateJointAccel(const dgJacobian* const externalAccel, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const
{
	dSpatialVector zero (dFloat(0.0f));
	for (int i = 0; i < m_nodeCount - 1; i++) {

		dGraph* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);

		dgForcePair& a = accel[i];
		dAssert(node->m_body);
		a.m_body = zero;
		a.m_joint = zero;

		dAssert(node->m_joint);
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		dAssert(jointInfo->m_joint == node->m_joint);

		const int first = jointInfo->m_pairStart;
		const int dof = jointInfo->m_pairCount;

		const int m0 = node->m_swapJacobianBodiesIndex ? node->m_parent->m_index : i;
		const int m1 = node->m_swapJacobianBodiesIndex ? i : node->m_parent->m_index;

		const dgJacobian& y0 = externalAccel[m0];
		const dgJacobian& y1 = externalAccel[m1];
		for (int j = 0; j < dof; j++) {
			const int k = node->m_sourceJacobianIndex[j];
			const dgJacobianMatrixElement* const row = &matrixRow[first + k];
			dgVector diag(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_Jt.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						  row->m_Jt.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_Jt.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			a.m_joint[j] = (diag.AddHorizontal()).GetScalar() - row->m_penetrationStiffness;
		}
	}
	dAssert((m_nodeCount - 1) == m_nodesOrder[m_nodeCount - 1]->m_index);
	accel[m_nodeCount - 1].m_body = dSpatialVector(dFloat(0.0f));
	accel[m_nodeCount - 1].m_joint = dSpatialVector(dFloat(0.0f));
}


inline void dCustomIKSolver::UpdateVelocity(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force, dgJacobian* const internalForces, dFloat timestep) const
{
	dgVector zero(dgVector::m_zero);
	dAssert (m_cyclingBodies.GetCount() == 0);
	//const int bodyCount = m_cyclingBodies.GetCount() + m_nodeCount;
	const int bodyCount = m_nodeCount;
	for (int i = 0; i < bodyCount; i++) {
		internalForces[i].m_linear = zero; 
		internalForces[i].m_angular = zero; 
	}

	for (int i = 0; i < (m_nodeCount - 1); i++) {
		dGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		dAssert(i == node->m_index);

		const dSpatialVector& f = force[i].m_joint;
		dAssert(jointInfo->m_joint == node->m_joint);
		const int first = jointInfo->m_pairStart;
		const int count = node->m_dof;
		for (int j = 0; j < count; j++) {
			const int k = node->m_sourceJacobianIndex[j];
			dgJacobianMatrixElement* const row = &matrixRow[first + k];

			row->m_force += dFloat(f[j]);
			dgVector jointForce = dFloat(f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
		}

		//const int m0 = i;
		//const int m1 = node->m_parent->m_index;
		const int m0 = node->m_swapJacobianBodiesIndex ? node->m_parent->m_index : i;
		const int m1 = node->m_swapJacobianBodiesIndex ? i : node->m_parent->m_index;

		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}

	dgVector timestep4 (timestep);
	for (int i = 0; i < m_nodeCount; i++) {
		dGraph* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);
		dAssert(node->m_body);
		dgDynamicBody* const body = node->m_body;

		dgVector velocStep(internalForces[i].m_linear.Scale4 (timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(internalForces[i].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
	}

	int index = m_nodeCount;
	for (dgList<dgDynamicBody*>::dgListNode* node = m_cyclingBodies.GetFirst(); node; node = node->GetNext()) {
		dgDynamicBody* const body = node->GetInfo();

		dgVector velocStep(internalForces[index].m_linear.Scale4(timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(internalForces[index].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
		index++;
	}
}

void dCustomIKSolver::SolveAuxiliaryVelocity(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force, dgJacobian* const externalAccel, dFloat timestep) const
{
	dTimeTrackerEvent(__FUNCTION__);

	dFloat* const f = dgAlloca(dFloat, m_rowCount);
	dFloat* const u = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const b = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const low = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const high = dgAlloca(dFloat, m_auxiliaryRowCount);
	dFloat* const massMatrix11 = dgAlloca(dFloat, m_auxiliaryRowCount * m_auxiliaryRowCount);
	dFloat* const lowerTriangularMassMatrix11 = dgAlloca(dFloat, m_auxiliaryRowCount * m_auxiliaryRowCount);

	int primaryIndex = 0;
	int auxiliaryIndex = 0;
	const int primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (int i = 0; i < m_nodeCount - 1; i++) {
		const dGraph* const node = m_nodesOrder[i];
		const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];
		const int first = jointInfo->m_pairStart;

		const int primaryDof = node->m_dof;
		const dSpatialVector& accelSpatial = accel[i].m_joint;
		const dSpatialVector& forceSpatial = force[i].m_joint;

		for (int j = 0; j < primaryDof; j++) {
			f[primaryIndex] = dFloat(forceSpatial[j]);
			primaryIndex++;
		}

		const int auxiliaryDof = jointInfo->m_pairCount - primaryDof;
		for (int j = 0; j < auxiliaryDof; j++) {
			const int index = node->m_sourceJacobianIndex[primaryDof + j];
			dgJacobianMatrixElement* const row = &matrixRow[first + index];
			f[auxiliaryIndex + primaryCount] = dFloat(0.0f);
			b[auxiliaryIndex] = -dFloat(accelSpatial[primaryDof + j]);
			low[auxiliaryIndex] = dgClamp(row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dFloat(0.0f));
			high[auxiliaryIndex] = dgClamp(row->m_upperBoundFrictionCoefficent - row->m_force, dFloat(0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgCyclingJoint& entry = ptr->GetInfo();
		const dgConstraint* const joint = entry.m_joint;

		const dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];
		const int m0 = entry.m_m0;
		const int m1 = entry.m_m1;
		const int first = jointInfo->m_pairStart;
		const int auxiliaryDof = jointInfo->m_pairCount;
		const dgJacobian& y0 = externalAccel[m0];
		const dgJacobian& y1 = externalAccel[m1];

		for (int i = 0; i < auxiliaryDof; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[first + i];
			f[auxiliaryIndex + primaryCount] = dFloat(0.0f);
//			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
//						 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
//			b[auxiliaryIndex] = row->m_coordenateAccel - (acc.AddHorizontal()).GetScalar();
			dgVector diag(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_Jt.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						  row->m_Jt.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_Jt.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			b[auxiliaryIndex] = row->m_penetrationStiffness - (diag.AddHorizontal()).GetScalar();

			low[auxiliaryIndex] = dgClamp(row->m_lowerBoundFrictionCoefficent - row->m_force, -DG_MAX_BOUND, dFloat(0.0f));
			high[auxiliaryIndex] = dgClamp(row->m_upperBoundFrictionCoefficent - row->m_force, dFloat(0.0f), DG_MAX_BOUND);
			auxiliaryIndex++;
		}
	}

	memcpy(massMatrix11, m_massMatrix11, sizeof(dFloat) * m_auxiliaryRowCount * m_auxiliaryRowCount);
	memcpy(lowerTriangularMassMatrix11, m_lowerTriangularMassMatrix11, sizeof(dFloat) * m_auxiliaryRowCount * m_auxiliaryRowCount);

	for (int i = 0; i < m_auxiliaryRowCount; i++) {
		dFloat* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		u[i] = dFloat(0.0f);
		dFloat r = dFloat(0.0f);
		for (int j = 0; j < primaryCount; j++) {
			r += matrixRow10[j] * f[j];
		}
		b[i] -= r;
	}

	dgSolveDantzigLCP(m_auxiliaryRowCount, massMatrix11, lowerTriangularMassMatrix11, u, b, low, high);

	for (int i = 0; i < m_auxiliaryRowCount; i++) {
		const dFloat s = u[i];
		f[primaryCount + i] = s;
		const dFloat* const deltaForce = &m_deltaForce[i * primaryCount];
		for (int j = 0; j < primaryCount; j++) {
			f[j] += deltaForce[j] * s;
		}
	}


	dgVector zero(dgVector::m_zero);
	const int bodyCount = m_cyclingBodies.GetCount() + m_nodeCount;
	for (int i = 0; i < bodyCount; i++) {
		externalAccel[i].m_linear = zero;
		externalAccel[i].m_angular = zero;
	}

	for (int i = 0; i < m_rowCount; i++) {
		dgJacobianMatrixElement* const row = m_rowArray[i];
		const int m0 = m_pairs[i].m_m0;
		const int m1 = m_pairs[i].m_m1;

		row->m_force += f[i];
		dgVector jointForce(f[i]);
		externalAccel[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(jointForce);
		externalAccel[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(jointForce);
		externalAccel[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(jointForce);
		externalAccel[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(jointForce);
	}

	dgVector timestep4(timestep);
	for (int i = 0; i < m_nodeCount; i++) {
		dGraph* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);
		dAssert(node->m_body);
		dgDynamicBody* const body = node->m_body;

		dgVector velocStep(externalAccel[i].m_linear.Scale4(timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(externalAccel[i].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
	}

	int index = m_nodeCount;
	for (dgList<dgDynamicBody*>::dgListNode* node = m_cyclingBodies.GetFirst(); node; node = node->GetNext()) {
		dgDynamicBody* const body = node->GetInfo();

		dgVector velocStep(externalAccel[index].m_linear.Scale4(timestep * body->m_invMass.m_w));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(externalAccel[index].m_angular)).CompProduct4(timestep4));

		body->m_veloc += velocStep;
		body->m_omega += omegaStep;
		index++;
	}
}
#endif


#define D_LCP_MAX_VALUE dFloat (1.0e10f)

long long dCustomIKSolver::dJoint::m_ordinalInit = 0x050403020100ll;


class dCustomIKSolver::dJointInfo
{
	public:
	short m_pairCount;
	short m_pairStart;
};

class dCustomIKSolver::dMatriData
{
	public:
	dSpatialMatrix m_jt;
	dSpatialMatrix m_mass;
	dSpatialMatrix m_invMass;
};

class dCustomIKSolver::dBodyJointMatrixDataPair
{
	public:
	dMatriData m_body;
	dMatriData m_joint;
};


dCustomIKSolver::dJoint::dJoint(NewtonBody* const body)
	:dContainersAlloc()
	,m_body(body)
	,m_parent(NULL)
	,m_child(NULL)
	,m_sibling(NULL)
	,m_customJoint(NULL)
	,m_dof(0)
	,m_index(0)
	,m_primaryStart(0)
	,m_auxiliaryStart(0)
	,m_ordinals(0)
{
}


dCustomIKSolver::dJoint::dJoint(dCustomJoint* const child, dJoint* const parent)
	:dContainersAlloc()
	,m_body(child->GetBody0())
	,m_parent(parent)	
	,m_child(NULL)
	,m_sibling(NULL)
	,m_customJoint(child)
	,m_dof(0)
	,m_index(0)
	,m_primaryStart(0)
	,m_auxiliaryStart(0)
	,m_ordinals(0)
{
	dAssert(m_parent);
	dAssert(m_customJoint->GetBody1() == parent->m_body);
	dAssert(GetBodyInvMass() != dFloat(0.0f));

	if (m_parent->m_child) {
		m_sibling = m_parent->m_child;
	}
	m_parent->m_child = this;
}


dCustomIKSolver::dJoint::~dJoint()
{
	for (dJoint* ptr = m_child, *next = NULL; ptr; ptr = next) {
		next = ptr->m_sibling; 
		delete ptr;
	}
}


inline void dCustomIKSolver::dJoint::CalculateInertiaMatrix(dBodyJointMatrixDataPair* const massMatrix)
{
	//dSpatialMatrix& bodyMass = m_data.m_body.m_mass;
	dSpatialMatrix& bodyMass = massMatrix[m_index].m_body.m_mass;

	//dFloat mass = m_body->GetMass().m_w;
	dFloat mass = GetBodyMass();
	dAssert(mass < dFloat(1.0e10f));
	//dMatrix inertia(m_body->CalculateInertiaMatrix());
	dMatrix inertia(GetInertiaMatrix());
	for (int i = 0; i < 3; i++) {
		bodyMass[i][i] = mass;
		for (int j = 0; j < 3; j++) {
			bodyMass[i + 3][j + 3] = inertia[i][j];
		}
	}
}


//inline void dCustomIKSolver::dJoint::GetJacobians(const dgJointInfo* const jointInfo, const dgJacobianMatrixElement* const matrixRow)
inline void dCustomIKSolver::dJoint::GetJacobians(const dJointInfo* const jointInfo, const NewtonIKJacobianMatrixElement* const jacobianMatrix, dBodyJointMatrixDataPair* const massMatrix)
{
	dAssert(m_parent);
	
	//dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	//dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	//dSpatialMatrix& jointMass = m_data.m_joint.m_mass;
	dSpatialMatrix& bodyJt = massMatrix[m_index].m_body.m_jt;
	dSpatialMatrix& jointJ = massMatrix[m_index].m_joint.m_jt;
	dSpatialMatrix& jointMass = massMatrix[m_index].m_joint.m_mass;

	const int start = jointInfo->m_pairStart;
	for (int i = 0; i < m_dof; i++) {
		const int k = m_sourceJacobianIndex[i];
		const NewtonIKJacobianMatrixElement* const row = &jacobianMatrix[start + k];
		jointMass[i] = dSpatialVector(0.0f);
		jointMass[i][i] = -row->m_diagDamp;
		//bodyJt[i] = dSpatialVector(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM0.m_angular.CompProduct4(dgVector::m_negOne));
		//jointJ[i] = dSpatialVector(row->m_Jt.m_jacobianM1.m_linear.CompProduct4(dgVector::m_negOne), row->m_Jt.m_jacobianM1.m_angular.CompProduct4(dgVector::m_negOne));
		bodyJt[i] = dSpatialVector(row->m_Jt01).Scale (-1.0f);
		jointJ[i] = dSpatialVector(row->m_Jt10).Scale (-1.0f);
	}
}


inline void dCustomIKSolver::dJoint::CalculateJacobianBlock(dBodyJointMatrixDataPair* const massMatrix)
{
	//dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	//const dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	dSpatialMatrix& jointJ = massMatrix[m_index].m_joint.m_jt;
	const dSpatialMatrix& jointInvMass = massMatrix[m_index].m_joint.m_invMass;

	dSpatialMatrix copy;
	for (int i = 0; i < m_dof; i++) {
		copy[i] = jointJ[i];
		jointJ[i] = dSpatialVector(0.0f);
	}
	
	for (int i = 0; i < m_dof; i++) {
		const dSpatialVector& jacobian = copy[i];
		const dSpatialVector& invDiagonalRow = jointInvMass[i];
		for (int j = 0; j < m_dof; j++) {
			dFloat val = invDiagonalRow[j];
			jointJ[j] = jointJ[j] + jacobian.Scale(val);
		}
	}
}

inline void dCustomIKSolver::dJoint::CalculateJointDiagonal(dBodyJointMatrixDataPair* const massMatrix)
{
	//const dSpatialMatrix& bodyMass = m_data.m_body.m_mass;
	//const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	//dSpatialMatrix& jointMass = m_data.m_joint.m_mass;
	//dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	const dSpatialMatrix& bodyMass = massMatrix[m_index].m_body.m_mass;
	const dSpatialMatrix& bodyJt = massMatrix[m_index].m_body.m_jt;
	dSpatialMatrix& jointMass = massMatrix[m_index].m_joint.m_mass;
	dSpatialMatrix& jointInvMass = massMatrix[m_index].m_joint.m_invMass;

	dSpatialMatrix tmp;
	for (int i = 0; i < m_dof; i++) {
		tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
	}
	
	for (int i = 0; i < m_dof; i++) {
		dFloat a = bodyJt[i].DotProduct(tmp[i]);
		jointMass[i][i] -= a;
		for (int j = i + 1; j < m_dof; j++) {
			a = -bodyJt[i].DotProduct(tmp[j]);
			jointMass[i][j] = a;
			jointMass[j][i] = a;
		}
	}
	jointInvMass = jointMass.Inverse(m_dof);
}

inline void dCustomIKSolver::dJoint::CalculateBodyDiagonal(dBodyJointMatrixDataPair* const massMatrix, dJoint* const child)
{
	dAssert(child->m_customJoint);

	//dSpatialMatrix& bodyMass = m_data.m_body.m_mass;
	//const dSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
	//const dSpatialMatrix& childDiagonal = child->m_data.m_joint.m_mass;
	dSpatialMatrix& bodyMass = massMatrix[m_index].m_body.m_mass;
	const dSpatialMatrix& jacobianMatrix = massMatrix[child->m_index].m_joint.m_jt;
	const dSpatialMatrix& childDiagonal = massMatrix[child->m_index].m_joint.m_mass;
	dSpatialMatrix copy(dSpatialMatrix(0.0f));

	const int dof = child->m_dof;
	for (int i = 0; i < dof; i++) {
		const dSpatialVector& jacobian = jacobianMatrix[i];
		for (int j = 0; j < dof; j++) {
			//dAssert(dgAreEqual(dgFloat64(childDiagonal[i][j]), dgFloat64(childDiagonal[j][i]), dgFloat64(1.0e-5f)));
			dFloat val = childDiagonal[i][j];
			copy[j] = copy[j] + jacobian.Scale(val);
		}
	}
	
	for (int i = 0; i < dof; i++) {
		const dSpatialVector& Jacobian = copy[i];
		const dSpatialVector& JacobianTranspose = jacobianMatrix[i];
		for (int j = 0; j < 6; j++) {
			dFloat val = -Jacobian[j];
			bodyMass[j] = bodyMass[j] + JacobianTranspose.Scale(val);
		}
	}
}


//inline int Factorize (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
inline int dCustomIKSolver::dJoint::Factorize (const dJointInfo* const jointInfoArray, const NewtonIKJacobianMatrixElement* const jacobianMatrix, dBodyJointMatrixDataPair* const massMatrix)
{
	//dSpatialMatrix& bodyMass = m_data.m_body.m_mass;
	//dSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	//dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	dSpatialMatrix& bodyMass = massMatrix[m_index].m_body.m_mass;
	dSpatialMatrix& bodyInvMass = massMatrix[m_index].m_body.m_invMass;
	dSpatialMatrix& bodyJt = massMatrix[m_index].m_body.m_jt;

	bodyMass = dSpatialMatrix(0.0f);
//	dAssert (m_jacobian->GetBodyInvMass(m_body) != 0.0f);
	CalculateInertiaMatrix(massMatrix);

	int boundedDof = 0;
	m_ordinals = m_ordinalInit;
	if (m_customJoint) {
		dAssert(m_parent);

		//const dJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
		const dJointInfo* const jointInfo = &jointInfoArray[m_index];


		m_dof = 0;
		int count = jointInfo->m_pairCount;
		const int first = jointInfo->m_pairStart;

		for (int i = 0; i < count; i++) {
			int k = m_sourceJacobianIndex[i];
			const NewtonIKJacobianMatrixElement* const row = &jacobianMatrix[k + first];
			if ((row->m_lowerBoundFrictionCoefficent <= dFloat(-D_LCP_MAX_VALUE)) && (row->m_upperBoundFrictionCoefficent >= dFloat(D_LCP_MAX_VALUE))) {
				m_dof++;
			} else {
				dSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[count - 1]);
				i--;
				count--;
			}
		}
		dAssert(m_dof > 0);
		dAssert(m_dof <= 6);
		boundedDof += jointInfo->m_pairCount - count;
		GetJacobians(jointInfo, jacobianMatrix, massMatrix);
	}

	dAssert (GetBodyInvMass() != dFloat(0.0f));
//	if (m_body->GetInvMass().m_w != dFloat(0.0f)) {
		for (dJoint* child = m_child; child; child = child->m_sibling) {
			CalculateBodyDiagonal(massMatrix, child);
		}
		bodyInvMass = bodyMass.Inverse(6);
//	} else {
//		bodyInvMass = dSpatialMatrix(dFloat(0.0f));
//	}

	if (m_customJoint) {
		dAssert(m_parent);
		for (int i = 0; i < m_dof; i++) {
			bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
		}
		CalculateJointDiagonal(massMatrix);
		CalculateJacobianBlock(massMatrix);
	}

	return boundedDof;
}





dCustomIKSolver::dCustomIKSolver(NewtonBody* const rootBody)
	:dContainersAlloc()
	,m_skeleton(new dJoint(rootBody))
	,m_nodesOrder(NULL)
	,m_rowCount(0)
	,m_nodeCount(1)
	,m_auxiliaryRowCount(0)
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
*/
{
}

dCustomIKSolver::~dCustomIKSolver()
{
/*
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dgCyclingJoint& entry = ptr->GetInfo();
		entry.m_joint->m_isInSkeleton = false;
	}
	m_cyclingJoints.RemoveAll();

	for (int i = 0; i < m_nodeCount - 1; i ++) {
		m_nodesOrder[i]->m_joint->m_isInSkeleton = false;
	}
*/
	
	if (m_nodesOrder) {
		dContainersAlloc::Free(m_nodesOrder);
	}

	if (m_skeleton) {
		delete m_skeleton;
	}
}

dCustomIKSolver::dJoint* dCustomIKSolver::GetRoot () const
{
	return m_skeleton;
}

dCustomIKSolver::dJoint* dCustomIKSolver::AddChild (dCustomJoint* const child, dJoint* const parentBone)
{
	dAssert(parentBone);
	m_nodeCount++;
	return new dJoint(child, parentBone);
}




void dCustomIKSolver::UpdateJointAngles (dFloat timestep)
{
//	int memorySizeInBytes = GetMemoryBufferSizeInBytes(jointInfoArray, matrixRow);
//	dgInt8* const memoryBuffer = (dgInt8*)dgAlloca(dgVector, memorySizeInBytes / sizeof(dgVector));
//	dgForcePair* const accel = dgAlloca(dgForcePair, m_nodeCount);
//	dgForcePair* const force = dgAlloca(dgForcePair, m_nodeCount);
//	dgJacobian* const externalAccel = dgAlloca(dgJacobian, m_nodeCount + m_cyclingBodies.GetCount());
//	dAssert((dgInt64(accel) & 0x0f) == 0);
//	dAssert((dgInt64(force) & 0x0f) == 0);
//	dAssert((dgInt64(memoryBuffer) & 0x0f) == 0);
//	dAssert((dgInt64(externalAccel) & 0x0f) == 0);

	dJointInfo* const jointInfo = dAlloca(dJointInfo, m_nodeCount);
	dBodyJointMatrixDataPair* const massMatrix = dAlloca(dBodyJointMatrixDataPair, m_nodeCount);
	NewtonIKJacobianMatrixElement* const jacobianMatrix = dAlloca(NewtonIKJacobianMatrixElement, m_nodeCount * 6);

	//InitMassMatrix(jointInfoArray, matrixRow, memoryBuffer);
	InitMassMatrix(timestep, massMatrix, jointInfo, jacobianMatrix);
/*
	CalculateBodyAcceleration(externalAccel, timestep);
	CalculateJointAccel(externalAccel, jointInfoArray, matrixRow, accel);
	CalculateForce(force, accel);

	if (m_auxiliaryRowCount) {
		SolveAuxiliaryVelocity(jointInfoArray, matrixRow, accel, force, externalAccel, timestep);
	}
	else {
		UpdateVelocity(jointInfoArray, matrixRow, force, externalAccel, timestep);
	}
*/
}


void dCustomIKSolver::SortGraph(dJoint* const root, int& index)
{
	for (dJoint* node = root->m_child; node; node = node->m_sibling) {
		SortGraph(node, index);
	}

	dAssert((m_nodeCount - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_index = short(index);
	index++;
	dAssert(index <= m_nodeCount);
}


//void dCustomIKSolver::Finalize(int loopJointsCount, dgBilateralConstraint** const loopJointArray)
void dCustomIKSolver::Finalize(int loopJointsCount)
{
	dAssert(m_nodeCount >= 1);

//	const dgDynamicBody* const rootBody = m_skeleton->m_body;
//	dAssert(((rootBody->GetInvMass().m_w == dFloat(0.0f)) && (m_skeleton->m_child->m_sibling == NULL)) || (m_skeleton->m_body->GetInvMass().m_w != dFloat(0.0f)));
//	dgMemoryAllocator* const allocator = rootBody->GetWorld()->GetAllocator();

	int index = 0;
	m_nodesOrder = (dJoint**) dContainersAlloc::Alloc(m_nodeCount * sizeof(dJoint*));
	SortGraph(m_skeleton, index);
	dAssert(index == m_nodeCount);

	if (loopJointsCount) {
		dAssert (0);
/*
		int loopIndex = m_nodeCount;
		dgTree<int, dgDynamicBody*> filter(allocator);
		for (int i = 0; i < loopJointsCount; i++) {
			dgBilateralConstraint* const joint = loopJointArray[i];
			dgDynamicBody* const body0 = (dgDynamicBody*)joint->GetBody0();
			dgDynamicBody* const body1 = (dgDynamicBody*)joint->GetBody1();

			dAssert(body0->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dAssert(body1->IsRTTIType(dgBody::m_dynamicBodyRTTI));

			dGraph* const node0 = FindNode(body0);
			dGraph* const node1 = FindNode(body1);
			dAssert((node0 && !node1) || (node1 && !node0));
			joint->m_isInSkeleton = true;

			if (node0) {
				filter.Insert(node0->m_index, node0->m_body);
			}
			if (node1) {
				filter.Insert(node1->m_index, node1->m_body);
			}

			dgTree<int, dgDynamicBody*>::dgTreeNode* index0 = filter.Find(body0);
			if (!index0) {
				index0 = filter.Insert(loopIndex, body0);
				loopIndex++;
			}

			dgTree<int, dgDynamicBody*>::dgTreeNode* index1 = filter.Find(body1);
			if (!index1) {
				index1 = filter.Insert(loopIndex, body1);
				loopIndex++;
			}
			dgCyclingJoint cyclicEntry(joint, index0->GetInfo(), index1->GetInfo());
			m_cyclingJoints.Append(cyclicEntry);
		}

		for (int i = 0; i < m_nodeCount; i++) {
			filter.Remove(m_nodesOrder[i]->m_body);
		}
		dgTree<dgDynamicBody*, int> bodyOrder(allocator);
		dgTree<int, dgDynamicBody*>::Iterator iter(filter);
		for (iter.Begin(); iter; iter++) {
			bodyOrder.Insert(iter.GetKey(), iter.GetNode()->GetInfo());
		}

		dgTree<dgDynamicBody*, int>::Iterator iter1(bodyOrder);
		for (iter1.Begin(); iter1; iter1++) {
			m_cyclingBodies.Append(iter1.GetNode()->GetInfo());
		}
*/
	}
}



//void dCustomIKSolver::InitMassMatrix(const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgInt8* const memoryBuffer)
void dCustomIKSolver::InitMassMatrix(dFloat timestep, dBodyJointMatrixDataPair* const massMatrix, dJointInfo* const jointInfo, NewtonIKJacobianMatrixElement* const jacobianMatrix)
{
//	dTimeTrackerEvent(__FUNCTION__);

	int rowCount = 0;
	int primaryStart = 0;
	int auxiliaryStart = 0;

	if (m_nodesOrder) {
		for (int i = 0; i < m_nodeCount - 1; i++) {
			dJoint* const node = m_nodesOrder[i];
			//rowCount += jointInfoArray[node->m_joint->m_index].m_pairCount;
			//rowCount += node->m_customJoint->GetDOF();

			dJointInfo& info = jointInfo[i];
			NewtonIKJacobianMatrixElement& jointJacobianMatrix = jacobianMatrix[rowCount];
			info.m_pairCount = short(node->m_customJoint->GetIKJacobians(timestep, &jointJacobianMatrix));
			info.m_pairStart = short(rowCount);
			rowCount += info.m_pairCount;

			node->m_primaryStart = short(primaryStart);
			node->m_auxiliaryStart = short(auxiliaryStart);
			//auxiliaryStart += node->Factorize(jointInfoArray, matrixRow);
			auxiliaryStart += node->Factorize(jointInfo, jacobianMatrix, massMatrix);
			primaryStart += node->m_dof;
		}
		//m_nodesOrder[m_nodeCount - 1]->Factorize(jointInfoArray, matrixRow);
		m_nodesOrder[m_nodeCount-1]->Factorize(jointInfo, jacobianMatrix, massMatrix);
	}

	m_rowCount = short(rowCount);
	m_auxiliaryRowCount = short(auxiliaryStart);
	int extraAuxiliaryRows = 0;

dAssert(0);
/*
	for (dgList<dgCyclingJoint>::dgListNode* ptr = m_cyclingJoints.GetFirst(); ptr; ptr = ptr->GetNext()) {
		const dgConstraint* const joint = ptr->GetInfo().m_joint;
		extraAuxiliaryRows += jointInfoArray[joint->m_index].m_pairCount;
	}
	m_rowCount += short(extraAuxiliaryRows);
	m_auxiliaryRowCount += short(extraAuxiliaryRows);

	if (m_auxiliaryRowCount) {
		const int primaryCount = m_rowCount - m_auxiliaryRowCount;
		int primaryIndex = 0;
		int auxiliaryIndex = 0;

		m_rowArray = (dgJacobianMatrixElement**)memoryBuffer;
		m_pairs = (dgNodePair*)&m_rowArray[m_rowCount];
		m_massMatrix11 = (dFloat*)&m_pairs[m_rowCount];
		m_lowerTriangularMassMatrix11 = (dFloat*)&m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
		m_massMatrix10 = &m_lowerTriangularMassMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
		m_deltaForce = &m_massMatrix10[m_auxiliaryRowCount * primaryCount];

		for (int i = 0; i < m_nodeCount - 1; i++) {
			const dGraph* const node = m_nodesOrder[i];
			const dgJointInfo* const jointInfo = &jointInfoArray[node->m_joint->m_index];

#ifdef DG_EXPERIMENTAL_SOLVER
			const int m0 = node->m_swapJacobianBodiesIndex ? node->m_parent->m_index : i;
			const int m1 = node->m_swapJacobianBodiesIndex ? i : node->m_parent->m_index;
#else 
			const int m0 = jointInfo->m_m0;
			const int m1 = jointInfo->m_m1;
#endif
			const int primaryDof = node->m_dof;
			const int first = jointInfo->m_pairStart;

			for (int j = 0; j < primaryDof; j++) {
				const int index = node->m_sourceJacobianIndex[j];
				m_rowArray[primaryIndex] = &matrixRow[first + index];
				m_pairs[primaryIndex].m_m0 = m0;
				m_pairs[primaryIndex].m_m1 = m1;
				primaryIndex++;
			}

			const int auxiliaryDof = jointInfo->m_pairCount - primaryDof;
			for (int j = 0; j < auxiliaryDof; j++) {
				const int index = node->m_sourceJacobianIndex[primaryDof + j];
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
			const int m0 = entry.m_m0;
			const int m1 = entry.m_m1;
#else
			const int m0 = jointInfo->m_m0;
			const int m1 = jointInfo->m_m1;
#endif
			const int first = jointInfo->m_pairStart;
			const int auxiliaryDof = jointInfo->m_pairCount;

			for (int i = 0; i < auxiliaryDof; i++) {
				dgJacobianMatrixElement* const row = &matrixRow[first + i];
				m_rowArray[auxiliaryIndex + primaryCount] = row;
				m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
				m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
				auxiliaryIndex++;
			}
		}

		dFloat* const diagDamp = dgAlloca(dFloat, m_auxiliaryRowCount);
		const int auxiliaryCount = m_rowCount - m_auxiliaryRowCount;
		for (int i = 0; i < m_auxiliaryRowCount; i++) {
			const dgJacobianMatrixElement* const row_i = m_rowArray[primaryCount + i];
			dFloat* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * i];

			dgJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
			dgJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);

			dgVector element(JMinvM0.m_linear.CompProduct4(row_i->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row_i->m_Jt.m_jacobianM0.m_angular) +
				JMinvM1.m_linear.CompProduct4(row_i->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row_i->m_Jt.m_jacobianM1.m_angular));
			element = element.AddHorizontal();

			// I know I am doubling the matrix regularizer, but this makes the solution more robust.
			dFloat diagonal = element.GetScalar() + row_i->m_diagDamp;
			matrixRow11[i] = diagonal + row_i->m_diagDamp;
			diagDamp[i] = matrixRow11[i] * (DG_PSD_DAMP_TOL * dFloat(2.0f));

			const int m0 = m_pairs[auxiliaryCount + i].m_m0;
			const int m1 = m_pairs[auxiliaryCount + i].m_m1;
			for (int j = i + 1; j < m_auxiliaryRowCount; j++) {
				const dgJacobianMatrixElement* const row_j = m_rowArray[auxiliaryCount + j];

				const int k = auxiliaryCount + j;
				dgVector acc(dgVector::m_zero);
				if (m0 == m_pairs[k].m_m0) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				}
				else if (m0 == m_pairs[k].m_m1) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				}

				if (m1 == m_pairs[k].m_m1) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				}
				else if (m1 == m_pairs[k].m_m0) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				}
				acc = acc.AddHorizontal();
				dFloat offDiagValue = acc.GetScalar();
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagValue;
			}

			dFloat* const matrixRow10 = &m_massMatrix10[primaryCount * i];
			for (int j = 0; j < primaryCount; j++) {
				const dgJacobianMatrixElement* const row_j = m_rowArray[j];

				dgVector acc(dgVector::m_zero);
				if (m0 == m_pairs[j].m_m0) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				}
				else if (m0 == m_pairs[j].m_m1) {
					acc += JMinvM0.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM0.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				}

				if (m1 == m_pairs[j].m_m1) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM1.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM1.m_angular);
				}
				else if (m1 == m_pairs[j].m_m0) {
					acc += JMinvM1.m_linear.CompProduct4(row_j->m_Jt.m_jacobianM0.m_linear) + JMinvM1.m_angular.CompProduct4(row_j->m_Jt.m_jacobianM0.m_angular);
				}
				acc = acc.AddHorizontal();
				dFloat val = acc.GetScalar();
				matrixRow10[j] = val;
			}
		}

		dgForcePair* const forcePair = dgAlloca(dgForcePair, m_nodeCount);
		dgForcePair* const accelPair = dgAlloca(dgForcePair, m_nodeCount);
		accelPair[m_nodeCount - 1].m_body = dSpatialVector(dFloat(0.0f));
		accelPair[m_nodeCount - 1].m_joint = dSpatialVector(dFloat(0.0f));

		for (int i = 0; i < m_auxiliaryRowCount; i++) {
			dFloat* const matrixRow10 = &m_massMatrix10[i * primaryCount];

			int entry = 0;
			for (int j = 0; j < m_nodeCount - 1; j++) {
				const dGraph* const node = m_nodesOrder[j];
				const int index = node->m_index;
				accelPair[index].m_body = dSpatialVector(dFloat(0.0f));
				dSpatialVector& a = accelPair[index].m_joint;

				const int count = node->m_dof;
				for (int k = 0; k < count; k++) {
					a[k] = matrixRow10[entry];
					entry++;
				}
			}

			entry = 0;
			CalculateForce(forcePair, accelPair);
			dFloat* const deltaForcePtr = &m_deltaForce[i * primaryCount];
			for (int j = 0; j < m_nodeCount - 1; j++) {
				const dGraph* const node = m_nodesOrder[j];
				const int index = node->m_index;
				const dSpatialVector& f = forcePair[index].m_joint;
				const int count = node->m_dof;
				for (int k = 0; k < count; k++) {
					deltaForcePtr[entry] = dFloat(f[k]);
					entry++;
				}
			}

			dFloat* const matrixRow11 = &m_massMatrix11[i * m_auxiliaryRowCount];
			dFloat diagonal = matrixRow11[i];
			for (int k = 0; k < primaryCount; k++) {
				diagonal += deltaForcePtr[k] * matrixRow10[k];
			}
			matrixRow11[i] = dgMax(diagonal, diagDamp[i]);

			for (int j = i + 1; j < m_auxiliaryRowCount; j++) {
				dFloat offDiagonal = dFloat(0.0f);
				const dFloat* const row10 = &m_massMatrix10[j * primaryCount];
				for (int k = 0; k < primaryCount; k++) {
					offDiagonal += deltaForcePtr[k] * row10[k];
				}
				matrixRow11[j] += offDiagonal;
				m_massMatrix11[j * m_auxiliaryRowCount + i] += offDiagonal;
			}
		}

		bool isPsdMatrix = false;
		do {
			memcpy(m_lowerTriangularMassMatrix11, m_massMatrix11, sizeof(dFloat) * (m_auxiliaryRowCount * m_auxiliaryRowCount));
			isPsdMatrix = dgCholeskyFactorization(m_auxiliaryRowCount, m_lowerTriangularMassMatrix11);
			if (!isPsdMatrix) {
				for (int i = 0; i < m_auxiliaryRowCount; i++) {
					diagDamp[i] *= dFloat(2.0f);
					m_massMatrix11[i * m_auxiliaryRowCount + i] += diagDamp[i];
				}
			}
		} while (!isPsdMatrix);

		for (int i = 0; i < m_auxiliaryRowCount; i++) {
			dFloat* const row = &m_lowerTriangularMassMatrix11[i * m_auxiliaryRowCount];
			for (int j = i + 1; j < m_auxiliaryRowCount; j++) {
				row[j] = dFloat(0.0f);
			}
		}
	}
*/
}

