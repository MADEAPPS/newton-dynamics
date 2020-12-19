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
#include "ndJointBilateralConstraint.h"


#if 0
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
#endif

dInt64 ndSkeletonContainer::ndNode::m_ordinalInit = 0x050403020100ll;

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

D_INLINE dInt32 ndSkeletonContainer::ndNode::GetAuxiliaryRows(const ndRightHandSide* const rightHandSide) const
{
	dInt32 rowCount = 0;
	if (m_joint) 
	{
		dAssert(m_parent);
		const dInt32 count = m_joint->m_rowCount;
		const dInt32 first = m_joint->m_rowStart;
		for (dInt32 i = 0; i < count; i++) 
		{
			const ndRightHandSide* const rhs = &rightHandSide[i + first];
			if (!((rhs->m_lowerBoundFrictionCoefficent <= dFloat32(-D_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32(D_LCP_MAX_VALUE)))) 
			{
				rowCount++;
			}
		}
	}
	return rowCount;
}

D_INLINE void ndSkeletonContainer::ndNode::CalculateInertiaMatrix(dSpatialMatrix* const bodyMassArray) const
{
	dSpatialMatrix& bodyMass = bodyMassArray[m_index];
	
	bodyMass = dSpatialMatrix(dFloat32(0.0f));
	if (m_body->GetInvMass() != dFloat32(0.0f)) 
	{
		const dFloat32 mass = m_body->GetMassMatrix().m_w;
		dMatrix inertia (m_body->CalculateInertiaMatrix());
		for (dInt32 i = 0; i < 3; i++) 
		{
			bodyMass[i][i] = mass;
			for (dInt32 j = 0; j < 3; j++) 
			{
				bodyMass[i + 3][j + 3] = inertia[i][j];
			}
		}
	}
}

D_INLINE void ndSkeletonContainer::ndNode::GetJacobians(const ndLeftHandSide* const leftHandSide, const ndRightHandSide* const rightHandSide, dSpatialMatrix* const jointMassArray)
{
	dAssert(m_parent);

	dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	dSpatialMatrix& jointMass = jointMassArray[m_index];

	const dInt32 start = m_joint->m_rowStart;
	const dSpatialVector zero(dSpatialVector::m_zero);
	if (!m_swapJacobianBodiesIndex) 
	{
		for (dInt32 i = 0; i < m_dof; i++) 
		{
			const dInt32 k = m_sourceJacobianIndex[i];
			const ndRightHandSide* const rhs = &rightHandSide[start + k];
			const ndLeftHandSide* const row = &leftHandSide[start + k];
			jointMass[i] = zero;
			jointMass[i][i] = -rhs->m_diagDamp;
			bodyJt[i] = dSpatialVector(row->m_Jt.m_jacobianM0.m_linear * dVector::m_negOne, row->m_Jt.m_jacobianM0.m_angular * dVector::m_negOne);
			jointJ[i] = dSpatialVector(row->m_Jt.m_jacobianM1.m_linear * dVector::m_negOne, row->m_Jt.m_jacobianM1.m_angular * dVector::m_negOne);
		}
	}
	else 
	{
		for (dInt32 i = 0; i < m_dof; i++) 
		{
			const dInt32 k = m_sourceJacobianIndex[i];
			const ndRightHandSide* const rhs = &rightHandSide[start + k];
			const ndLeftHandSide* const row = &leftHandSide[start + k];
			jointMass[i] = zero;
			jointMass[i][i] = -rhs->m_diagDamp;
			bodyJt[i] = dSpatialVector(row->m_Jt.m_jacobianM1.m_linear * dVector::m_negOne, row->m_Jt.m_jacobianM1.m_angular * dVector::m_negOne);
			jointJ[i] = dSpatialVector(row->m_Jt.m_jacobianM0.m_linear * dVector::m_negOne, row->m_Jt.m_jacobianM0.m_angular * dVector::m_negOne);
		}
	}
}

D_INLINE void ndSkeletonContainer::ndNode::CalculateBodyDiagonal(ndNode* const child, dSpatialMatrix* const bodyMassArray, const dSpatialMatrix* const jointMassArray)
{
	dAssert(child->m_joint);

	dSpatialMatrix copy(dFloat32(0.0f));
	const dInt32 dof = child->m_dof;
	const dSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
	const dSpatialMatrix& childDiagonal = jointMassArray[child->m_index];
	for (dInt32 i = 0; i < dof; i++) 
	{
		const dSpatialVector& jacobian = jacobianMatrix[i];
		for (dInt32 j = 0; j < dof; j++) 
		{
			dAssert(dAreEqual(childDiagonal[i][j], childDiagonal[j][i], dFloat64(1.0e-5f)));
			dFloat64 val = childDiagonal[i][j];
			copy[j] = copy[j] + jacobian.Scale(val);
		}
	}

	dSpatialMatrix& bodyMass = bodyMassArray[m_index];
	for (dInt32 i = 0; i < dof; i++) 
	{
		const dSpatialVector& Jacobian = copy[i];
		const dSpatialVector& JacobianTranspose = jacobianMatrix[i];
		for (dInt32 j = 0; j < 6; j++) 
		{
			dFloat64 val = -Jacobian[j];
			bodyMass[j] = bodyMass[j] + JacobianTranspose.Scale(val);
		}
	}
}

D_INLINE void ndSkeletonContainer::ndNode::CalculateJointDiagonal(const dSpatialMatrix* const bodyMassArray, dSpatialMatrix* const jointMassArray)
{
	const dSpatialMatrix& bodyMass = bodyMassArray[m_index];
	const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;

	dSpatialMatrix tmp;
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
	}

	dSpatialMatrix& jointMass = jointMassArray[m_index];
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		dFloat64 a = bodyJt[i].DotProduct(tmp[i]);
		jointMass[i][i] -= a;
		for (dInt32 j = i + 1; j < m_dof; j++) 
		{
			a = -bodyJt[i].DotProduct(tmp[j]);
			jointMass[i][j] = a;
			jointMass[j][i] = a;
		}
	}

	dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	jointInvMass = jointMass.Inverse(m_dof);
}

D_INLINE void ndSkeletonContainer::ndNode::CalculateJacobianBlock()
{
	dSpatialMatrix& jointJ = m_data.m_joint.m_jt;

	dSpatialMatrix copy;
	const dSpatialVector zero(dSpatialVector::m_zero);
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		copy[i] = jointJ[i];
		jointJ[i] = zero;
	}

	const dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		const dSpatialVector& jacobian = copy[i];
		const dSpatialVector& invDiagonalRow = jointInvMass[i];
		for (dInt32 j = 0; j < m_dof; j++) 
		{
			dFloat64 val = invDiagonalRow[j];
			jointJ[j] = jointJ[j] + jacobian.Scale(val);
		}
	}
}

dInt32 ndSkeletonContainer::ndNode::Factorize(const ndLeftHandSide* const leftHandSide, const ndRightHandSide* const rightHandSide, dSpatialMatrix* const bodyMassArray, dSpatialMatrix* const jointMassArray)
{
	CalculateInertiaMatrix(bodyMassArray);

	dInt32 boundedDof = 0;
	m_ordinals = m_ordinalInit;

	if (m_joint) 
	{
		m_dof = 0;
		dAssert(m_parent);
		dInt32 count = m_joint->m_rowCount;
		const dInt32 first = m_joint->m_rowStart;
		for (dInt32 i = 0; i < count; i++) 
		{
			dInt32 k = m_sourceJacobianIndex[i];
			const ndRightHandSide* const rhs = &rightHandSide[k + first];
			if ((rhs->m_lowerBoundFrictionCoefficent <= dFloat32(-D_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32(D_LCP_MAX_VALUE))) 
			{
				m_dof++;
			}
			else 
			{
				dSwap(m_sourceJacobianIndex[i], m_sourceJacobianIndex[count - 1]);
				i--;
				count--;
			}
		}
		dAssert(m_dof > 0);
		dAssert(m_dof <= 6);
		//boundedDof += jointInfo->m_pairCount - count;
		boundedDof += m_joint->m_rowCount - count;
		GetJacobians(leftHandSide, rightHandSide, jointMassArray);
	}
	
	dSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	const dSpatialMatrix& bodyMass = bodyMassArray[m_index];
	if (m_body->GetInvMass() != dFloat32(0.0f)) 
	{
		for (ndNode* child = m_child; child; child = child->m_sibling) 
		{
			CalculateBodyDiagonal(child, bodyMassArray, jointMassArray);
		}
		bodyInvMass = bodyMass.Inverse(6);
	}
	else 
	{
		bodyInvMass = dSpatialMatrix(dFloat32(0.0f));
	}
	
	if (m_joint) 
	{
		dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
		dAssert(m_parent);
		for (dInt32 i = 0; i < m_dof; i++) 
		{
			bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
		}
		CalculateJointDiagonal(bodyMassArray, jointMassArray);
		CalculateJacobianBlock();
	}
	return boundedDof;
}

D_INLINE void ndSkeletonContainer::ndNode::BodyJacobianTimeSolutionBackward(ndForcePair& force) const
{
	const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		force.m_body = force.m_body + bodyJt[i].Scale(-force.m_joint[i]);
	}
}

D_INLINE void ndSkeletonContainer::ndNode::JointJacobianTimeSolutionBackward(ndForcePair& force, const ndForcePair& parentForce) const
{
	const dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	const dSpatialVector& f = parentForce.m_body;
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		force.m_joint[i] -= f.DotProduct(jointJ[i]);
	}
}

D_INLINE void ndSkeletonContainer::ndNode::JointDiagInvTimeSolution(ndForcePair& force)
{
	const dSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	force.m_joint = jointInvMass.VectorTimeMatrix(force.m_joint, m_dof);
}

D_INLINE void ndSkeletonContainer::ndNode::BodyDiagInvTimeSolution(ndForcePair& force)
{
	const dSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	force.m_body = bodyInvMass.VectorTimeMatrix(force.m_body);
}

D_INLINE void ndSkeletonContainer::ndNode::JointJacobianTimeMassForward(ndForcePair& force)
{
	const dSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
	}
}

D_INLINE void ndSkeletonContainer::ndNode::BodyJacobianTimeMassForward(const ndForcePair& force, ndForcePair& parentForce) const
{
	const dSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	for (dInt32 i = 0; i < m_dof; i++) 
	{
		parentForce.m_body = parentForce.m_body + jointJ[i].Scale(-force.m_joint[i]);
	}
}


ndSkeletonContainer::ndSkeletonContainer()
	:m_skeleton(nullptr)
	,m_nodesOrder(nullptr)
	,m_rightHandSide(nullptr)
	,m_leftHandSide(nullptr)
	,m_pairs(nullptr)
	,m_frictionIndex(nullptr)
	,m_matrixRowsIndex(nullptr)
	,m_massMatrix11(nullptr)
	,m_massMatrix10(nullptr)
	,m_deltaForce(nullptr)
	,m_nodeList()
	,m_loopingJoints(32)
	,m_auxiliaryMemoryBuffer(1024 * 8)
	,m_blockSize(0)
	,m_rowCount(0)
	,m_loopRowCount(0)
	,m_auxiliaryRowCount(0)
	,m_loopCount(0)
	,m_dynamicsLoopCount(0)
	,m_consideredCloseLoop(1)
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

	m_nodeList.RemoveAll();
}

void ndSkeletonContainer::Init(ndBodyKinematic* const rootBody)
{
	m_skeleton = &m_nodeList.Append()->GetInfo();
	m_skeleton->m_body = rootBody;
	if (rootBody->GetInvMass() != dFloat32(0.0f))
	{
		//rootBody->SetSkeleton(this);
		m_skeleton->m_body->SetSkeleton(this);
	}
}

ndSkeletonContainer::ndNode* ndSkeletonContainer::AddChild(ndJointBilateralConstraint* const joint, ndNode* const parent)
{
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
	//dAssert(((m_skeleton->m_body->GetInvMass() == dFloat32(0.0f)) && (m_skeleton->m_child->m_sibling == nullptr)) || (m_skeleton->m_body->GetInvMass() != dFloat32(0.0f)));
	
	m_nodesOrder = (ndNode**)dMemory::Malloc(m_nodeList.GetCount() * sizeof(ndNode*));
	
	dInt32 index = 0;
	SortGraph(m_skeleton, index);
	dAssert(index == m_nodeList.GetCount());
	
	if (loopJointsCount) 
	{
		for (dInt32 i = 0; i < loopJointsCount; i++) 
		{
			ndJointBilateralConstraint* const joint = loopJointArray[i];
			//dAssert(joint->GetBody0()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			//dAssert(joint->GetBody1()->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			//dAssert((FindNode((ndBodyKinematic*)joint->GetBody0()) || FindNode((ndBodyKinematic*)joint->GetBody1())));
			joint->m_isInSkeleton = true;
			//m_loopingJoints[m_loopCount] = joint;
			m_loopingJoints.PushBack(joint);
			m_loopCount++;
		}
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

void ndSkeletonContainer::InitMassMatrix(const ndLeftHandSide* const leftHandSide, ndRightHandSide* const rightHandSide, bool consideredCloseLoop)
{
	D_TRACKTIME();
	dInt32 rowCount = 0;
	dInt32 auxiliaryCount = 0;
	m_leftHandSide = leftHandSide;
	m_rightHandSide = rightHandSide;
	m_consideredCloseLoop = consideredCloseLoop ? 1 : 0;
	
	const dInt32 nodeCount = m_nodeList.GetCount();
	dSpatialMatrix* const bodyMassArray = dAlloca(dSpatialMatrix, nodeCount);
	dSpatialMatrix* const jointMassArray = dAlloca(dSpatialMatrix, nodeCount);
	if (m_nodesOrder) 
	{
		for (dInt32 i = 0; i < nodeCount - 1; i++) 
		{
			ndNode* const node = m_nodesOrder[i];
			rowCount += node->m_joint->m_rowCount;
			auxiliaryCount += node->Factorize(leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
		}
		m_nodesOrder[nodeCount - 1]->Factorize(leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
	}

	m_rowCount = dInt16(rowCount);
	m_auxiliaryRowCount = dInt16(auxiliaryCount);
	
	dInt32 loopRowCount = 0;
	const dInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dInt32 j = 0; j < loopCount; j++) 
	{
		const ndConstraint* const joint = m_loopingJoints[j];
		loopRowCount += joint->m_rowCount;
	}

	m_loopRowCount = dInt16(loopRowCount);
	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;
	
	if (m_auxiliaryRowCount && m_consideredCloseLoop) 
	{
		InitLoopMassMatrix();
	}
}

void ndSkeletonContainer::CalculateBufferSizeInBytes()
{
	dInt32 rowCount = 0;
	dInt32 auxiliaryRowCount = 0;
	if (m_nodesOrder) 
	{
		const dInt32 nodeCount = m_nodeList.GetCount() - 1;
		for (dInt32 i = 0; i < nodeCount; i++) 
		{
			ndNode* const node = m_nodesOrder[i];
			rowCount += node->m_joint->m_rowCount;
			auxiliaryRowCount += node->GetAuxiliaryRows(m_rightHandSide);
		}
	}

	dInt32 extraAuxiliaryRows = 0;
	const dInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dInt32 j = 0; j < loopCount; j++) 
	{
		const ndConstraint* const joint = m_loopingJoints[j];
		extraAuxiliaryRows += joint->m_rowCount;
	}
	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount += extraAuxiliaryRows;

	dInt32 size = sizeof(dInt32) * rowCount;
	size += sizeof(dInt32) * rowCount;
	size += sizeof(ndNodePair) * rowCount;
	size += sizeof(dFloat32) * auxiliaryRowCount * auxiliaryRowCount;
	size += sizeof(dFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof(dFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size = (size + 1024) & -0x10;
	m_auxiliaryMemoryBuffer.SetCount((size + 1024) & -0x10);
}

void ndSkeletonContainer::CalculateLoopMassMatrixCoefficients(dFloat32* const diagDamp)
{
	D_TRACKTIME();
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dInt32 index = 0; index < m_auxiliaryRowCount; index++) 
	{
		const dInt32 ii = m_matrixRowsIndex[primaryCount + index];
		const ndLeftHandSide* const row_i = &m_leftHandSide[ii];
		const ndRightHandSide* const rhs_i = &m_rightHandSide[ii];
		const ndJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
		const ndJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);
		const dVector element(
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
		for (dInt32 j = index + 1; j < m_auxiliaryRowCount; j++) 
		{
			const dInt32 jj = m_matrixRowsIndex[primaryCount + j];
			const ndLeftHandSide* const row_j = &m_leftHandSide[jj];

			const dInt32 k = primaryCount + j;
			dVector acc(dVector::m_zero);
			const dInt32 m0_j = m_pairs[k].m_m0;
			const dInt32 m1_j = m_pairs[k].m_m1;
			bool hasEffect = false;
			if (m0_i == m0_j) 
			{
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}
			else if (m0_i == m1_j) 
			{
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}

			if (m1_i == m1_j) 
			{
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}
			else if (m1_i == m0_j) 
			{
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}

			if (hasEffect) 
			{
				acc = acc.AddHorizontal();
				dFloat32 offDiagValue = acc.GetScalar();
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + index] = offDiagValue;
			}
		}

		dFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * index];
		for (dInt32 j = 0; j < primaryCount; j++) 
		{
			const dInt32 jj = m_matrixRowsIndex[j];
			const ndLeftHandSide* const row_j = &m_leftHandSide[jj];

			const dInt32 m0_j = m_pairs[j].m_m0;
			const dInt32 m1_j = m_pairs[j].m_m1;
			dVector acc(dVector::m_zero);
			bool hasEffect = false;
			if (m0_i == m0_j) 
			{
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}
			else if (m0_i == m1_j) 
			{
				hasEffect = true;
				acc += JMinvM0.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}

			if (m1_i == m1_j) 
			{
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular;
			}
			else if (m1_i == m0_j) 
			{
				hasEffect = true;
				acc += JMinvM1.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM0.m_angular;
			}

			if (hasEffect) 
			{
				acc = acc.AddHorizontal();
				dFloat32 val = acc.GetScalar();
				matrixRow10[j] = val;
			}
		}
	}
}

D_INLINE void ndSkeletonContainer::SolveForward(ndForcePair* const force, const ndForcePair* const accel, dInt32 startNode) const
{
	dSpatialVector zero(dSpatialVector::m_zero);
	for (dInt32 i = 0; i < startNode; i++) 
	{
		force[i].m_body = zero;
		force[i].m_joint = zero;
	}

	const dInt32 nodeCount = m_nodeList.GetCount();
	for (dInt32 i = startNode; i < nodeCount - 1; i++) 
	{
		ndNode* const node = m_nodesOrder[i];
		dAssert(node->m_joint);
		dAssert(node->m_index == i);
		ndForcePair& f = force[i];
		const ndForcePair& a = accel[i];
		f.m_body = a.m_body;
		f.m_joint = a.m_joint;
		for (ndNode* child = node->m_child; child; child = child->m_sibling) 
		{
			dAssert(child->m_joint);
			dAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(f);
	}

	force[nodeCount - 1] = accel[nodeCount - 1];
	for (ndNode* child = m_nodesOrder[nodeCount - 1]->m_child; child; child = child->m_sibling) 
	{
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	for (dInt32 i = startNode; i < nodeCount - 1; i++) 
	{
		ndNode* const node = m_nodesOrder[i];
		ndForcePair& f = force[i];
		node->BodyDiagInvTimeSolution(f);
		node->JointDiagInvTimeSolution(f);
	}
	m_nodesOrder[nodeCount - 1]->BodyDiagInvTimeSolution(force[nodeCount - 1]);
}

D_INLINE void ndSkeletonContainer::SolveBackward(ndForcePair* const force) const
{
	const dInt32 nodeCount = m_nodeList.GetCount();
	for (dInt32 i = nodeCount - 2; i >= 0; i--) 
	{
		ndNode* const node = m_nodesOrder[i];
		dAssert(node->m_index == i);
		ndForcePair& f = force[i];
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyJacobianTimeSolutionBackward(f);
	}
}

void ndSkeletonContainer::ConditionMassMatrix() const
{
	D_TRACKTIME();
	const dInt32 nodeCount = m_nodeList.GetCount();
	ndForcePair* const forcePair = dAlloca(ndForcePair, nodeCount);
	const dSpatialVector zero(dSpatialVector::m_zero);

	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) 
	{
		dInt32 entry0 = 0;
		dInt32 startjoint = nodeCount;
		const dFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		for (dInt32 j = 0; j < nodeCount - 1; j++) 
		{
			const ndNode* const node = m_nodesOrder[j];
			const dInt32 index = node->m_index;
			forcePair[index].m_body = zero;
			dSpatialVector& a = forcePair[index].m_joint;

			const int count = node->m_dof;
			for (dInt32 k = 0; k < count; k++) 
			{
				const dFloat32 value = matrixRow10[entry0];
				a[k] = value;
				startjoint = (value == 0.0f) ? startjoint : dMin(startjoint, index);
				entry0++;
			}
		}

		startjoint = (startjoint == nodeCount) ? 0 : startjoint;
		dAssert(startjoint < nodeCount);
		forcePair[nodeCount - 1].m_body = zero;
		forcePair[nodeCount - 1].m_joint = zero;
		SolveForward(forcePair, forcePair, startjoint);
		SolveBackward(forcePair);

		dInt32 entry1 = 0;
		dFloat32* const deltaForcePtr = &m_deltaForce[i * primaryCount];
		for (dInt32 j = 0; j < nodeCount - 1; j++) 
		{
			const ndNode* const node = m_nodesOrder[j];
			const dInt32 index = node->m_index;
			const dSpatialVector& f = forcePair[index].m_joint;
			const int count = node->m_dof;
			for (dInt32 k = 0; k < count; k++) 
			{
				deltaForcePtr[entry1] = dFloat32(f[k]);
				entry1++;
			}
		}
	}
}

void ndSkeletonContainer::RebuildMassMatrix(const dFloat32* const diagDamp) const
{
	D_TRACKTIME();
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
	dInt16* const indexList = dAlloca(dInt16, primaryCount);
	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) 
	{
		const dFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		dFloat32* const matrixRow11 = &m_massMatrix11[i * m_auxiliaryRowCount];

		dInt32 indexCount = 0;
		for (dInt32 k = 0; k < primaryCount; k++) 
		{
			indexList[indexCount] = dInt16(k);
			indexCount += (matrixRow10[k] != dFloat32(0.0f)) ? 1 : 0;
		}

		for (dInt32 j = i; j < m_auxiliaryRowCount; j++) 
		{
			dFloat32 offDiagonal = matrixRow11[j];
			const dFloat32* const row10 = &m_deltaForce[j * primaryCount];
			for (dInt32 k = 0; k < indexCount; k++) 
			{
				dInt32 index = indexList[k];
				offDiagonal += matrixRow10[index] * row10[index];
			}
			matrixRow11[j] = offDiagonal;
			m_massMatrix11[j * m_auxiliaryRowCount + i] = offDiagonal;
		}

		matrixRow11[i] = dMax(matrixRow11[i], diagDamp[i]);
	}
}

void ndSkeletonContainer::FactorizeMatrix(dInt32 size, dInt32 stride, dFloat32* const matrix, dFloat32* const diagDamp) const
{
	D_TRACKTIME();
	bool isPsdMatrix = false;
	dFloat32* const backupMatrix = dAlloca(dFloat32, size * stride);
	do 
	{
		{
			dInt32 srcLine = 0;
			dInt32 dstLine = 0;
			for (dInt32 i = 0; i < size; i++) {
				memcpy(&backupMatrix[dstLine], &matrix[srcLine], size * sizeof(dFloat32));
				srcLine += size;
				dstLine += stride;
			}
		}
		isPsdMatrix = dCholeskyFactorization(size, stride, matrix);
		if (!isPsdMatrix) 
		{
			dInt32 srcLine = 0;
			dInt32 dstLine = 0;
			for (dInt32 i = 0; i < size; i++) 
			{
				memcpy(&matrix[dstLine], &backupMatrix[srcLine], size * sizeof(dFloat32));
				diagDamp[i] *= dFloat32(4.0f);
				matrix[dstLine + i] += diagDamp[i];
				dstLine += size;
				srcLine += stride;
			}
		}
	} while (!isPsdMatrix);
}

void ndSkeletonContainer::InitLoopMassMatrix()
{
	CalculateBufferSizeInBytes();
	dInt8* const memoryBuffer = &m_auxiliaryMemoryBuffer[0];
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	m_frictionIndex = (dInt32*)memoryBuffer;
	m_matrixRowsIndex = (dInt32*)&m_frictionIndex[m_rowCount];
	m_pairs = (ndNodePair*)&m_matrixRowsIndex[m_rowCount];
	m_massMatrix11 = (dFloat32*)&m_pairs[m_rowCount];
	m_massMatrix10 = (dFloat32*)&m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount];
	m_deltaForce = &m_massMatrix10[m_auxiliaryRowCount * primaryCount];
	
	dInt32* const boundRow = dAlloca(dInt32, m_auxiliaryRowCount);

	m_blockSize = 0;
	dInt32 primaryIndex = 0;
	dInt32 auxiliaryIndex = 0;
	const dInt32 nodeCount = m_nodeList.GetCount() - 1;
	for (dInt32 i = 0; i < nodeCount; i++)
	{
		const ndNode* const node = m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;
		const dInt32 m0 = joint->GetBody0()->m_index;
		const dInt32 m1 = joint->GetBody1()->m_index;
		const dInt32 primaryDof = node->m_dof;
		const dInt32 first = joint->m_rowStart;
		for (dInt32 j = 0; j < primaryDof; j++) 
		{
			const dInt32 index = node->m_sourceJacobianIndex[j];
			m_pairs[primaryIndex].m_m0 = m0;
			m_pairs[primaryIndex].m_m1 = m1;
			m_frictionIndex[primaryIndex] = 0;
			m_matrixRowsIndex[primaryIndex] = first + index;
			primaryIndex++;
		}

		const dInt32 auxiliaryDof = joint->m_rowCount - primaryDof;
		for (dInt32 j = 0; j < auxiliaryDof; j++) 
		{
			const dInt32 index = node->m_sourceJacobianIndex[primaryDof + j];
			const ndRightHandSide* const rhs = &m_rightHandSide[first + index];

			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_frictionIndex[auxiliaryIndex + primaryCount] = 0;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + index;
			const dInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= dFloat32(-D_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32(D_LCP_MAX_VALUE)) ? 1 : 0;
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}
	dAssert(m_loopRowCount == (m_auxiliaryRowCount - auxiliaryIndex));

	const dInt32 loopCount = m_loopCount + m_dynamicsLoopCount;
	for (dInt32 j = 0; j < loopCount; j++) 
	{
		const ndConstraint* const joint = m_loopingJoints[j];
		const dInt32 m0 = joint->GetBody0()->m_index;
		const dInt32 m1 = joint->GetBody1()->m_index;

		const dInt32 first = joint->m_rowStart;
		const dInt32 auxiliaryDof = joint->m_rowCount;
		for (dInt32 i = 0; i < auxiliaryDof; i++) 
		{
			const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_frictionIndex[auxiliaryIndex + primaryCount] = (rhs->m_normalForceIndex < 0) ? 0 : rhs->m_normalForceIndex - i;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			const dInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= dFloat32(-D_LCP_MAX_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= dFloat32(D_LCP_MAX_VALUE)) ? 1 : 0;
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}

	dAssert(primaryIndex == primaryCount);
	dAssert(auxiliaryIndex == m_auxiliaryRowCount);

	for (dInt32 i = 1; i < auxiliaryIndex; i++) 
	{
		dInt32 j = i;
		dInt32 tmpBoundRow = boundRow[j];
		ndNodePair tmpPair(m_pairs[primaryCount + j]);
		dInt32 tmpFrictionIndex = m_frictionIndex[primaryCount + j];
		dInt32 tmpMatrixRowsIndex = m_matrixRowsIndex[primaryCount + j];

		for (; j && (boundRow[j - 1] < tmpBoundRow); j--) 
		{
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

	dFloat32* const diagDamp = dAlloca(dFloat32, m_auxiliaryRowCount);
	memset(m_massMatrix10, 0, primaryCount * m_auxiliaryRowCount * sizeof(dFloat32));
	memset(m_massMatrix11, 0, m_auxiliaryRowCount * m_auxiliaryRowCount * sizeof(dFloat32));

	CalculateLoopMassMatrixCoefficients(diagDamp);
	ConditionMassMatrix();
	RebuildMassMatrix(diagDamp);

	if (m_blockSize) 
	{
		FactorizeMatrix(m_blockSize, m_auxiliaryRowCount, m_massMatrix11, diagDamp);
		const int boundedSize = m_auxiliaryRowCount - m_blockSize;
		dFloat32* const acc = dAlloca(dFloat32, m_auxiliaryRowCount);
		dInt32 rowStart = 0;

		for (dInt32 i = 0; i < m_blockSize; i++) 
		{
			memset(acc, 0, boundedSize * sizeof(dFloat32));
			const dFloat32* const row = &m_massMatrix11[rowStart];
			for (dInt32 j = 0; j < i; j++) 
			{
				const dFloat32 s = row[j];
				const dFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (dInt32 k = 0; k < boundedSize; k++) 
				{
					acc[k] += s * x[k];
				}
			}

			dFloat32* const x = &m_massMatrix11[rowStart + m_blockSize];
			const dFloat32 den = -dFloat32(1.0f) / row[i];
			for (dInt32 j = 0; j < boundedSize; j++) 
			{
				x[j] = (x[j] + acc[j]) * den;
			}
			rowStart += m_auxiliaryRowCount;
		}

		for (dInt32 i = m_blockSize - 1; i >= 0; i--) 
		{
			memset(acc, 0, boundedSize * sizeof(dFloat32));
			for (dInt32 j = i + 1; j < m_blockSize; j++) 
			{
				const dFloat32 s = m_massMatrix11[j * m_auxiliaryRowCount + i];
				const dFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (dInt32 k = 0; k < boundedSize; k++) 
				{
					acc[k] += s * x[k];
				}
			}

			dFloat32* const x = &m_massMatrix11[i * m_auxiliaryRowCount + m_blockSize];
			const dFloat32 den = dFloat32(1.0f) / m_massMatrix11[i * m_auxiliaryRowCount + i];
			for (dInt32 j = 0; j < boundedSize; j++) 
			{
				x[j] = (x[j] - acc[j]) * den;
			}
		}

		for (dInt32 i = 0; i < boundedSize; i++) 
		{
			for (int j = 0; j < m_blockSize; j++) 
			{
				acc[j] = m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize + i];
			}

			dFloat32* const arow = &m_massMatrix11[(m_blockSize + i) * m_auxiliaryRowCount + m_blockSize];
			for (int j = i; j < boundedSize; j++) 
			{
				const dFloat32* const row1 = &m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount];
				dFloat32 elem = row1[m_blockSize + i] + dDotProduct(m_blockSize, acc, row1);
				arow[j] = elem;
				m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount + m_blockSize + i] = elem;
			}
			arow[i] += diagDamp[m_blockSize + i];
		}
		dAssert(dTestPSDmatrix(m_auxiliaryRowCount - m_blockSize, m_auxiliaryRowCount, &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize]));
	}
}

D_INLINE void ndSkeletonContainer::CalculateJointAccel(const ndJacobian* const internalForces, ndForcePair* const accel) const
{
	const dSpatialVector zero(dSpatialVector::m_zero);
	const dInt32 nodeCount = m_nodeList.GetCount();
	for (dInt32 i = 0; i < nodeCount - 1; i++) 
	{
		ndNode* const node = m_nodesOrder[i];
		dAssert(i == node->m_index);

		ndForcePair& a = accel[i];
		dAssert(node->m_body);
		a.m_body = zero;
		a.m_joint = zero;

		dAssert(node->m_joint);
		ndJointBilateralConstraint* const joint = node->m_joint;

		const dInt32 first = joint->m_rowStart;
		const dInt32 dof = joint->m_rowCount;
		const dInt32 m0 = joint->GetBody0()->m_index;
		const dInt32 m1 = joint->GetBody1()->m_index;
		const ndJacobian& y0 = internalForces[m0];
		const ndJacobian& y1 = internalForces[m1];

		for (dInt32 j = 0; j < dof; j++) 
		{
			const dInt32 k = node->m_sourceJacobianIndex[j];
			const ndLeftHandSide* const row = &m_leftHandSide[first + k];
			const ndRightHandSide* const rhs = &m_rightHandSide[first + k];
			dVector diag(
				row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
				row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
			a.m_joint[j] = -(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - diag.AddHorizontal().GetScalar());
		}
	}
	dAssert((nodeCount - 1) == m_nodesOrder[nodeCount - 1]->m_index);
	accel[nodeCount - 1].m_body = zero;
	accel[nodeCount - 1].m_joint = zero;
}

D_INLINE void ndSkeletonContainer::CalculateForce(ndForcePair* const force, const ndForcePair* const accel) const
{
	SolveForward(force, accel, 0);
	SolveBackward(force);
}

D_INLINE void ndSkeletonContainer::UpdateForces(ndJacobian* const internalForces, const ndForcePair* const force) const
{
	dVector zero(dVector::m_zero);
	const dInt32 nodeCount = m_nodeList.GetCount();
	for (dInt32 i = 0; i < (nodeCount - 1); i++) 
	{
		ndNode* const node = m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;

		ndJacobian y0;
		ndJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		dAssert(i == node->m_index);

		const dSpatialVector& f = force[i].m_joint;
		const dInt32 first = joint->m_rowStart;
		const dInt32 count = node->m_dof;
		for (dInt32 j = 0; j < count; j++) 
		{
			const dInt32 k = node->m_sourceJacobianIndex[j];
			ndRightHandSide* const rhs = &m_rightHandSide[first + k];
			const ndLeftHandSide* const row = &m_leftHandSide[first + k];

			rhs->m_force += dFloat32(f[j]);
			dVector jointForce = dFloat32(f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
		}

		const dInt32 m0 = joint->GetBody0()->m_index;
		const dInt32 m1 = joint->GetBody1()->m_index;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}
}

void ndSkeletonContainer::SolveLcp(dInt32 stride, dInt32 size, const dFloat32* const matrix, const dFloat32* const x0, dFloat32* const x, const dFloat32* const b, const dFloat32* const low, const dFloat32* const high, const dInt32* const normalIndex) const
{
	D_TRACKTIME();
	//if (m_world->GetCurrentPlugin()) 
	if (0)
	{
		dAssert(0);
		//dgWorldPlugin* const plugin = m_world->GetCurrentPlugin()->GetInfo().m_plugin;
		//plugin->SolveDenseLcp(stride, size, matrix, x0, x, b, low, high, normalIndex);
	}
	else 
	{
#if 0
		// sequential Sidle iteration
		const dFloat32 sor = dFloat32(1.125f);
		const dFloat32 tol2 = dFloat32(0.25f);
		const dInt32 maxIterCount = 64;

		dFloat32* const invDiag1 = dAlloca(dFloat32, size);

		int rowStart = 0;
		for (dInt32 i = 0; i < size; i++) 
		{
			const int index = normalIndex[i];
			const dFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : 1.0f;
			const dFloat32 l = low[i] * coefficient - x0[i];
			const dFloat32 h = high[i] * coefficient - x0[i];;
			x[i] = dClamp(dFloat32(0.0f), l, h);
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
			for (dInt32 i = 0; i < size; i++) 
			{
				const dFloat32* const row = &matrix[base];
				dFloat32 r = b[i] - dDotProduct(size, row, x);

				const int index = normalIndex[i];
				const dFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : dFloat32(1.0f);
				const dFloat32 l = low[i] * coefficient - x0[i];
				const dFloat32 h = high[i] * coefficient - x0[i];

				dFloat32 f = x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor;
				if (f > h) 
				{
					f = h;
				}
				else if (f < l) 
				{
					f = l;
				}
				else 
				{
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

		dFloat32* const invDiag1 = dAlloca(dFloat32, size);
		dFloat32* const residual = dAlloca(dFloat32, size);

		dInt32 rowStart = 0;
		for (dInt32 i = 0; i < size; i++) 
		{
			const int index = normalIndex[i];
			const dFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : 1.0f;
			const dFloat32 l = low[i] * coefficient - x0[i];
			const dFloat32 h = high[i] * coefficient - x0[i];
			x[i] = dClamp(dFloat32(0.0f), l, h);
			invDiag1[i] = dFloat32(1.0f) / matrix[rowStart + i];
			rowStart += stride;
		}

		dInt32 base = 0;
		for (dInt32 i = 0; i < size; i++) 
		{
			const dFloat32* const row = &matrix[base];
			residual[i] = b[i] - dDotProduct(size, row, x);
			base += stride;
		}

		dInt32 iterCount = 0;
		dFloat32 tolerance(tol2 * dFloat32(2.0f));
		const dFloat32* const invDiag = invDiag1;
		const dFloat32 one = dFloat32(1.0f);
		for (dInt32 k = 0; (k < maxIterCount) && (tolerance > tol2); k++) 
		{
			base = 0;
			iterCount++;
			tolerance = dFloat32(0.0f);
			for (dInt32 i = 0; i < size; i++) 
			{
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
				}
				else if (f < l) {
					f = l;
				}
				else {
					tolerance += r * r;
				}
				const dFloat32 dx = f - x[i];
#else
				const dFloat32 f = dClamp(x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor, l, h);
				const dFloat32 dx = f - x[i];
				const dFloat32 dr = dx * row[i];
				tolerance += dr * dr;
#endif
				x[i] = f;
				if (dAbs(dx) > dFloat32(1.0e-6f)) 
				{
					for (dInt32 j = 0; j < size; j++) 
					{
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
	if (blockSize) 
	{
		dSolveCholesky(blockSize, size, m_massMatrix11, x, b);
		if (blockSize != size) 
		{

			dInt32 base = blockSize * size;
			for (dInt32 i = blockSize; i < size; i++) 
			{
				b[i] -= dDotProduct(blockSize, &m_massMatrix11[base], x);
				base += size;
			}

			const int boundedSize = size - blockSize;
			SolveLcp(
				size, boundedSize, &m_massMatrix11[blockSize * size + blockSize],
				&x0[blockSize], &x[blockSize], &b[blockSize], &low[blockSize], &high[blockSize], &normalIndex[blockSize]);

			for (dInt32 j = 0; j < blockSize; j++) 
			{
				const dFloat32* const row = &m_massMatrix11[j * size + blockSize];
				dFloat32 acc = dFloat32(0.0f);
				for (dInt32 i = 0; i < boundedSize; i++) 
				{
					acc += x[blockSize + i] * row[i];
				}
				x[j] += acc;
			}
		}
	}
	else 
	{
		SolveLcp(size, size, m_massMatrix11, x0, x, b, low, high, normalIndex);
	}
}

void ndSkeletonContainer::SolveAuxiliary(ndJacobian* const internalForces, const ndForcePair* const accel, ndForcePair* const force) const
{
	dFloat32* const f = dAlloca(dFloat32, m_rowCount);
	dFloat32* const u = dAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const b = dAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const u0 = dAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const low = dAlloca(dFloat32, m_auxiliaryRowCount);
	dFloat32* const high = dAlloca(dFloat32, m_auxiliaryRowCount);
	dInt32* const normalIndex = dAlloca(dInt32, m_auxiliaryRowCount);

	dInt32 primaryIndex = 0;
	const dInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	const dInt32 nodeCount = m_nodeList.GetCount();
	for (dInt32 i = 0; i < nodeCount - 1; i++) 
	{
		const ndNode* const node = m_nodesOrder[i];
		const dInt32 primaryDof = node->m_dof;
		const dSpatialVector& forceSpatial = force[i].m_joint;

		for (dInt32 j = 0; j < primaryDof; j++) 
		{
			f[primaryIndex] = dFloat32(forceSpatial[j]);
			primaryIndex++;
		}
	}

	dAssert(primaryIndex == primaryCount);
	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) 
	{
		const int index = m_matrixRowsIndex[primaryCount + i];
		const ndLeftHandSide* const row = &m_leftHandSide[index];
		const ndRightHandSide* const rhs = &m_rightHandSide[index];

		const int m0 = m_pairs[primaryCount + i].m_m0;
		const int m1 = m_pairs[primaryCount + i].m_m1;

		const ndJacobian& y0 = internalForces[m0];
		const ndJacobian& y1 = internalForces[m1];

		f[primaryCount + i] = dFloat32(0.0f);

		dVector acc(
			row->m_JMinv.m_jacobianM0.m_linear * y0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * y0.m_angular +
			row->m_JMinv.m_jacobianM1.m_linear * y1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * y1.m_angular);
		b[i] = rhs->m_coordenateAccel - acc.AddHorizontal().GetScalar();

		normalIndex[i] = m_frictionIndex[primaryCount + i];
		u0[i] = rhs->m_force;
		low[i] = rhs->m_lowerBoundFrictionCoefficent;
		high[i] = rhs->m_upperBoundFrictionCoefficent;
	}

	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) 
	{
		dFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		b[i] -= dDotProduct(primaryCount, matrixRow10, f);
	}

	SolveBlockLcp(m_auxiliaryRowCount, m_blockSize, u0, u, b, low, high, normalIndex);

	for (dInt32 i = 0; i < m_auxiliaryRowCount; i++) 
	{
		const dFloat32 s = u[i];
		f[primaryCount + i] = s;
		dMulAdd(primaryCount, f, f, &m_deltaForce[i * primaryCount], s);
	}

	for (dInt32 i = 0; i < m_rowCount; i++) 
	{
		dInt32 index = m_matrixRowsIndex[i];
		ndRightHandSide* const rhs = &m_rightHandSide[index];
		const ndLeftHandSide* const row = &m_leftHandSide[index];
		const dInt32 m0 = m_pairs[i].m_m0;
		const dInt32 m1 = m_pairs[i].m_m1;

		rhs->m_force += f[i];
		dVector jointForce(f[i]);
		internalForces[m0].m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
		internalForces[m0].m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
		internalForces[m1].m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
		internalForces[m1].m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
	}
}

void ndSkeletonContainer::CalculateJointForce(const ndBodyKinematic** const bodyArray, ndJacobian* const internalForces)
{
	D_TRACKTIME();
	const dInt32 nodeCount = m_nodeList.GetCount();
	ndForcePair* const force = dAlloca(ndForcePair, nodeCount);
	ndForcePair* const accel = dAlloca(ndForcePair, nodeCount);
	
	CalculateJointAccel(internalForces, accel);
	CalculateForce(force, accel);
	if (m_auxiliaryRowCount && m_consideredCloseLoop) 
	{
		SolveAuxiliary(internalForces, accel, force);
	}
	else 
	{
		UpdateForces(internalForces, force);
	}
}
