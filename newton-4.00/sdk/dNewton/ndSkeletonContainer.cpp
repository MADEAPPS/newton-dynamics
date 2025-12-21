/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndSort.h"
#include "ndWorld.h"
#include "ndContact.h"
#include "ndIkSolver.h"
#include "ndBodyDynamic.h"
#include "ndDynamicsUpdate.h"
#include "ndSkeletonContainer.h"
#include "ndJointBilateralConstraint.h"

#define D_MAX_OPEN_LOOP_DOF				6
#define D_MAX_SKELETON_LCP_VALUE		(D_LCP_MAX_VALUE * ndFloat32 (0.25f))
#define D_MAX_POSIT_ERROR_VIOLATION2	(ndFloat32(0.125f) * ndFloat32(0.125f))

ndSkeletonContainer::ndNode::ndNode()
	:m_body(nullptr)
	,m_joint(nullptr)
	,m_parent(nullptr)
	,m_child(nullptr)
	,m_sibling(nullptr)
	,m_index(0)
	,m_factorizeLayer(0)
	,m_ordinal()
	,m_dof(0)
	,m_swapJacobianBodiesIndex(0)
{
}

ndSkeletonContainer::ndNode::~ndNode()
{
	m_body->SetSkeleton(nullptr);
}

ndInt32 ndSkeletonContainer::ndNode::GetAuxiliaryRows(const ndRightHandSide* const rightHandSide) const
{
	ndInt32 rowCount = 0;
	if (m_joint) 
	{
		ndAssert(m_parent);
		const ndInt32 count = m_joint->m_rowCount;
		const ndInt32 first = m_joint->m_rowStart;
		for (ndInt32 i = 0; i < count; ++i) 
		{
			const ndRightHandSide* const rhs = &rightHandSide[i + first];
			if (!((rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)))) 
			{
				rowCount++;
			}
			ndAssert(rhs->SanityCheck());
		}
	}
	return rowCount;
}

void ndSkeletonContainer::ndNode::CalculateInertiaMatrix(ndSpatialMatrix* const bodyMassArray) const
{
	ndSpatialMatrix& bodyMass = bodyMassArray[m_index];
	
	bodyMass = ndSpatialMatrix::m_zero;
	if (m_body->GetInvMass() != ndFloat32(0.0f)) 
	{
		const ndFloat32 mass = m_body->GetMassMatrix().m_w;
		ndMatrix inertia (m_body->CalculateInertiaMatrix());
		for (ndInt32 i = 0; i < 3; ++i) 
		{
			bodyMass[i][i] = mass;
			for (ndInt32 j = 0; j < 3; ++j)  
			{
				bodyMass[i + 3][j + 3] = inertia[i][j];
			}
		}
	}
}

void ndSkeletonContainer::ndNode::GetJacobians(const ndLeftHandSide* const leftHandSide, const ndRightHandSide* const rightHandSide, ndSpatialMatrix* const jointMassArray)
{
	ndAssert(m_parent);

	ndSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	ndSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	ndSpatialMatrix& jointMass = jointMassArray[m_index];

	const ndVector negOne(ndVector::m_negOne);
	const ndInt32 start = m_joint->m_rowStart;
	const ndSpatialVector zero(ndSpatialVector::m_zero);
	if (!m_swapJacobianBodiesIndex) 
	{
		for (ndInt32 i = 0; i < m_dof; ++i) 
		{
			const ndInt32 k = m_ordinal.m_sourceJacobianIndex[i];
			const ndRightHandSide* const rhs = &rightHandSide[start + k];
			const ndLeftHandSide* const row = &leftHandSide[start + k];
			jointMass[i] = zero;
			jointMass[i][i] = -rhs->m_diagDamp;

			bodyJt[i] = ndSpatialVector(row->m_Jt.m_jacobianM0.m_linear * negOne, row->m_Jt.m_jacobianM0.m_angular * negOne);
			jointJ[i] = ndSpatialVector(row->m_Jt.m_jacobianM1.m_linear * negOne, row->m_Jt.m_jacobianM1.m_angular * negOne);
			for (ndInt32 j = 0; j < 4; ++j) 
			{
				bodyJt[i][3 + j] = bodyJt[i][4 + j];
				jointJ[i][3 + j] = jointJ[i][4 + j];
			}
		}
	}
	else 
	{
		for (ndInt32 i = 0; i < m_dof; ++i) 
		{
			const ndInt32 k = m_ordinal.m_sourceJacobianIndex[i];
			const ndRightHandSide* const rhs = &rightHandSide[start + k];
			const ndLeftHandSide* const row = &leftHandSide[start + k];
			jointMass[i] = zero;
			jointMass[i][i] = -rhs->m_diagDamp;

			jointJ[i] = ndSpatialVector(row->m_Jt.m_jacobianM0.m_linear * negOne, row->m_Jt.m_jacobianM0.m_angular * negOne);
			bodyJt[i] = ndSpatialVector(row->m_Jt.m_jacobianM1.m_linear * negOne, row->m_Jt.m_jacobianM1.m_angular * negOne);
			for (ndInt32 j = 0; j < 4; ++j) 
			{
				bodyJt[i][3 + j] = bodyJt[i][4 + j];
				jointJ[i][3 + j] = jointJ[i][4 + j];
			}
		}
	}
}

void ndSkeletonContainer::ndNode::CalculateBodyDiagonal(ndNode* const child, ndSpatialMatrix* const bodyMassArray, const ndSpatialMatrix* const jointMassArray)
{
	ndAssert(child->m_joint);

	ndSpatialMatrix copy(ndFloat32(0.0f));
	const ndInt32 dof = child->m_dof;
	const ndSpatialMatrix& jacobianMatrix = child->m_data.m_joint.m_jt;
	const ndSpatialMatrix& childDiagonal = jointMassArray[child->m_index];
	for (ndInt32 i = 0; i < dof; ++i) 
	{
		const ndSpatialVector& jacobian = jacobianMatrix[i];
		for (ndInt32 j = 0; j < dof; ++j)  
		{
			ndAssert(ndAreEqual(childDiagonal[i][j], childDiagonal[j][i], ndFloat64(1.0e-5f)));
			ndFloat64 val = childDiagonal[i][j];
			copy[j] = copy[j] + jacobian.Scale(val);
		}
	}

	ndSpatialMatrix& bodyMass = bodyMassArray[m_index];
	for (ndInt32 i = 0; i < dof; ++i) 
	{
		const ndSpatialVector& Jacobian = copy[i];
		const ndSpatialVector& JacobianTranspose = jacobianMatrix[i];
		for (ndInt32 j = 0; j < 6; ++j)  
		{
			ndFloat64 val = -Jacobian[j];
			bodyMass[j] = bodyMass[j] + JacobianTranspose.Scale(val);
		}
	}
}

void ndSkeletonContainer::ndNode::CalculateJointDiagonal(const ndSpatialMatrix* const bodyMassArray, ndSpatialMatrix* const jointMassArray)
{
	const ndSpatialMatrix& bodyMass = bodyMassArray[m_index];
	const ndSpatialMatrix& bodyJt = m_data.m_body.m_jt;

	ndSpatialMatrix tmp;
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		tmp[i] = bodyMass.VectorTimeMatrix(bodyJt[i]);
	}

	ndSpatialMatrix& jointMass = jointMassArray[m_index];
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		ndFloat64 a = bodyJt[i].DotProduct(tmp[i]);
		jointMass[i][i] -= a;
		for (ndInt32 j = i + 1; j < m_dof; ++j)  
		{
			a = -bodyJt[i].DotProduct(tmp[j]);
			jointMass[i][j] = a;
			jointMass[j][i] = a;
		}
	}

	// can't use CholeskyFactorization because neither bodyMass or jointMass
	// are guarantee to be PSD
	ndSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	jointInvMass = jointMass.Inverse(m_dof);
}

void ndSkeletonContainer::ndNode::CalculateJacobianBlock()
{
	ndSpatialMatrix& jointJ = m_data.m_joint.m_jt;

	ndSpatialMatrix copy;
	const ndSpatialVector zero(ndSpatialVector::m_zero);
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		copy[i] = jointJ[i];
		jointJ[i] = zero;
	}

	const ndSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		const ndSpatialVector& jacobian = copy[i];
		const ndSpatialVector& invDiagonalRow = jointInvMass[i];
		for (ndInt32 j = 0; j < m_dof; ++j)  
		{
			ndFloat64 val = invDiagonalRow[j];
			jointJ[j] = jointJ[j] + jacobian.Scale(val);
		}
	}
}

ndInt32 ndSkeletonContainer::ndNode::FactorizeChild(const ndLeftHandSide* const leftHandSide, const ndRightHandSide* const rightHandSide, ndSpatialMatrix* const bodyMassArray, ndSpatialMatrix* const jointMassArray)
{
	CalculateInertiaMatrix(bodyMassArray);

	m_dof = 0;
	ndAssert(m_joint);
	ndAssert(m_parent);

	m_ordinal = ndOrdinal();
	ndInt32 count = m_joint->m_rowCount;
	const ndInt32 first = m_joint->m_rowStart;
	for (ndInt32 i = 0; i < count; ++i) 
	{
		ndInt32 k = m_ordinal.m_sourceJacobianIndex[i];
		const ndRightHandSide* const rhs = &rightHandSide[k + first];
		ndAssert(rhs->SanityCheck());

		if ((rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE))) 
		{
			m_dof++;
		}
		else 
		{
			ndSwap(m_ordinal.m_sourceJacobianIndex[i], m_ordinal.m_sourceJacobianIndex[count - 1]);
			i--;
			count--;
		}
	}
	m_dof = ndMin(m_dof, ndInt8(D_MAX_OPEN_LOOP_DOF));
	ndAssert(m_dof >= 0);

	ndInt32 boundedDof = m_joint->m_rowCount - m_dof;
	GetJacobians(leftHandSide, rightHandSide, jointMassArray);
	
	ndSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	const ndSpatialMatrix& bodyMass = bodyMassArray[m_index];

	ndAssert(m_body->GetInvMass() != ndFloat32(0.0f));
	for (ndNode* child = m_child; child; child = child->m_sibling) 
	{
		CalculateBodyDiagonal(child, bodyMassArray, jointMassArray);
	}

	// can't use CholeskyFactorization because neither bodyMass or jointMass
	// are guarantee to be PSD, using gaussian pivoting inverse
	bodyInvMass = bodyMass.Inverse(6);
	ndSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	ndAssert(m_parent);
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		bodyJt[i] = bodyInvMass.VectorTimeMatrix(bodyJt[i]);
	}
	CalculateJointDiagonal(bodyMassArray, jointMassArray);
	CalculateJacobianBlock();

	return boundedDof;
}

ndInt32 ndSkeletonContainer::ndNode::FactorizeRoot(ndSpatialMatrix* const bodyMassArray, ndSpatialMatrix* const jointMassArray)
{
	CalculateInertiaMatrix(bodyMassArray);

	ndInt32 boundedDof = 0;
	m_ordinal = ndOrdinal();

	ndAssert(!m_joint);

	ndSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	const ndSpatialMatrix& bodyMass = bodyMassArray[m_index];
	if (m_body->GetInvMass() != ndFloat32(0.0f))
	{
		for (ndNode* child = m_child; child; child = child->m_sibling)
		{
			CalculateBodyDiagonal(child, bodyMassArray, jointMassArray);
		}

		// can't use CholeskyFactorization because neither bodyMass or jointMass
		// are guarantee to be PSD
		bodyInvMass = bodyMass.Inverse(6);
	}
	else
	{
		bodyInvMass = ndSpatialMatrix(ndFloat32(0.0f));
	}
	return boundedDof;
}

void ndSkeletonContainer::ndNode::BodyJacobianTimeSolutionBackward(ndForcePair& force) const
{
	const ndSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		force.m_body = force.m_body + bodyJt[i].Scale(-force.m_joint[i]);
	}
}

void ndSkeletonContainer::ndNode::JointJacobianTimeSolutionBackward(ndForcePair& force, const ndForcePair& parentForce) const
{
	const ndSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	const ndSpatialVector& f = parentForce.m_body;
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		force.m_joint[i] -= f.DotProduct(jointJ[i]);
	}
}

void ndSkeletonContainer::ndNode::JointDiagInvTimeSolution(ndForcePair& force)
{
	const ndSpatialMatrix& jointInvMass = m_data.m_joint.m_invMass;
	force.m_joint = jointInvMass.VectorTimeMatrix(force.m_joint, m_dof);
}

void ndSkeletonContainer::ndNode::BodyDiagInvTimeSolution(ndForcePair& force)
{
	const ndSpatialMatrix& bodyInvMass = m_data.m_body.m_invMass;
	force.m_body = bodyInvMass.VectorTimeMatrix(force.m_body);
}

void ndSkeletonContainer::ndNode::JointJacobianTimeMassForward(ndForcePair& force)
{
	const ndSpatialMatrix& bodyJt = m_data.m_body.m_jt;
	for (ndInt32 i = 0; i < m_dof; ++i) 
	{
		force.m_joint[i] -= bodyJt[i].DotProduct(force.m_body);
	}
}

void ndSkeletonContainer::ndNode::BodyJacobianTimeMassForward(const ndForcePair& force, ndForcePair& parentForce) const
{
	const ndSpatialMatrix& jointJ = m_data.m_joint.m_jt;
	for (ndInt32 i = 0; i < m_dof; ++i) 
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
	,m_diagonalPreconditioner(nullptr)
	,m_precondinonedMassMatrix11(nullptr)
	,m_nodeList()
	,m_transientLoopingContacts()
	,m_transientLoopingJoints()
	,m_permanentLoopingJoints()
	,m_auxiliaryMemoryBuffer()
	,m_lock()
	,m_id(0)
	,m_blockSize(0)
	,m_rowCount(0)
	,m_loopRowCount(0)
	,m_auxiliaryRowCount(0)
	,m_isResting(0)
	,m_maxPositError2(ndFloat32 (0.0f))
	,m_auxiliaryJointViolationsRowCount(0)
{
	m_auxiliaryMemoryBuffer.SetCount(1024 * 8);
	m_auxiliaryMemoryBuffer.SetCount(0);
}

ndSkeletonContainer::~ndSkeletonContainer()
{
	for (ndInt32 i = ndInt32 (m_permanentLoopingJoints.GetCount() - 1); i >=0; --i)
	{
		ndJointBilateralConstraint* const joint = m_permanentLoopingJoints[i];
		joint->m_isInSkeleton = false;
	}
	
	for (ndInt32 i = m_nodeList.GetCount() - 2; i >= 0; i--)
	{
		m_nodesOrder[i]->m_joint->m_isInSkeleton = false;
	}

	if (m_nodesOrder) 
	{
		ndMemory::Free(m_nodesOrder);
	}

	m_nodeList.RemoveAll();
}

ndInt32 ndSkeletonContainer::GetId() const
{
	return m_id;
}

ndSkeletonContainer::ndNode* ndSkeletonContainer::GetRoot() const
{
	return m_skeleton;
}

const ndSkeletonContainer::ndNodeList& ndSkeletonContainer::GetNodeList() const
{
	return m_nodeList;
}

void ndSkeletonContainer::Clear()
{
	for (ndInt32 i = ndInt32(m_permanentLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		ndJointBilateralConstraint* const joint = m_permanentLoopingJoints[i];
		joint->m_isInSkeleton = false;
	}
	m_permanentLoopingJoints.SetCount(0);
	m_transientLoopingJoints.SetCount(0);
	m_transientLoopingContacts.SetCount(0);
}

void ndSkeletonContainer::Init(ndBodyKinematic* const rootBody, ndInt32 id)
{
	m_id = id;
	m_skeleton = &m_nodeList.Append()->GetInfo();
	m_skeleton->m_body = rootBody;
	if (rootBody->GetInvMass() != ndFloat32(0.0f))
	{
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

	ndAssert(node->m_parent);
#ifdef _DEBUG
	if (node->m_body->GetInvMass() == ndFloat32(0.0f))
	{
		ndTrace(("%s (%d %f) (%d %f)\n", joint->ClassName(), 
			joint->GetBody0()->GetId(), joint->GetBody0()->GetInvMass(), 
			joint->GetBody1()->GetId(), joint->GetBody1()->GetInvMass()));
	}
#endif
	ndAssert(node->m_body->GetInvMass() != ndFloat32(0.0f));

	if (node->m_parent->m_child)
	{
		node->m_sibling = node->m_parent->m_child;
	}
	node->m_parent->m_child = node;

	joint->SetSkeletonFlag(true);
	ndAssert(node->m_body->GetScene()->GetWorld()->GetSentinelBody()->GetAsBodyKinematic() != node->m_body);
	node->m_body->SetSkeleton(this);
	return node;
}

void ndSkeletonContainer::Finalize(ndInt32 loopJointsCount, ndJointBilateralConstraint** const loopJointArray)
{
	ndAssert(m_nodeList.GetCount() >= 1);
	m_nodesOrder = (ndNode**)ndMemory::Malloc(2 * m_nodeList.GetCount() * sizeof(ndNode*));
	m_nodesFactorizationOrder = &m_nodesOrder[m_nodeList.GetCount()];

	ndInt32 index = m_nodeList.GetCount();
	ndFixSizeArray<ndNode*, 1024> stack;

	ndNode** tmpArray = ndAlloca(ndNode*, m_nodeList.GetCount());

	stack.PushBack(m_skeleton);
	while (stack.GetCount())
	{
		index--;
		ndAssert(index >= 0);
		ndNode* const node = stack.Pop();
		m_nodesOrder[index] = node;
		tmpArray[index] = node;
		node->m_index = index;
		node->m_factorizeLayer = 0;

		for (ndNode* childNode = node->m_child; childNode; childNode = childNode->m_sibling)
		{
			stack.PushBack(childNode);
		}
	}

	ndInt32 maxLayer = 0;
	bool layerPending = true;
	while (layerPending)
	{
		layerPending = false;
		for (ndInt32 i = m_nodeList.GetCount()-2; i >= 0; --i)
		{
			ndNode* const node = tmpArray[i];
			if (node->m_child && (node->m_factorizeLayer == node->m_child->m_factorizeLayer))
			{
				layerPending = true;
				node->m_factorizeLayer = node->m_child->m_factorizeLayer + 1;
				maxLayer = ndMax(maxLayer, node->m_factorizeLayer);
			}
		}
	}

	class ndEvaluateKey
	{
		public:
		ndEvaluateKey(void* const)
		{
		}

		ndInt32 GetKey(const ndNode* const node) const
		{
			return node->m_factorizeLayer;
		}
	};

	ndAssert(maxLayer < 128);
	m_factorizeSpans.SetCount(128);
	ndCountingSort<ndNode*, ndEvaluateKey, 7>(tmpArray, m_nodesFactorizationOrder, m_nodeList.GetCount() - 1, &m_factorizeSpans[0], &maxLayer);
	m_factorizeSpans.SetCount(maxLayer + 2);
	ndAssert(ndInt32(m_factorizeSpans[m_factorizeSpans.GetCount() - 1]) == ndInt32(m_nodeList.GetCount() - 1));

	for (ndInt32 i = 0; i < loopJointsCount; ++i) 
	{
		ndJointBilateralConstraint* const joint = loopJointArray[i];
		joint->m_isInSkeleton = true;
		m_permanentLoopingJoints.PushBack(joint);
	}
}

void ndSkeletonContainer::AddCloseLoopJoint(ndConstraint* const joint)
{
	ndScopeSpinLock lock(m_lock);
	ndContact* const contact = joint->GetAsContact();
	if (contact)
	{
		m_transientLoopingContacts.PushBack(contact);
	}
	else
	{
		m_transientLoopingJoints.PushBack(joint->GetAsBilateral());
	}
}

void ndSkeletonContainer::ClearCloseLoopJoints()
{
	ndScopeSpinLock lock(m_lock);
	for (ndInt32 i = ndInt32(m_transientLoopingContacts.GetCount() - 1); i >= 0; --i)
	{
		ndContact* const contact = m_transientLoopingContacts[i];
		ndAssert(contact->GetAsContact());
		contact->m_skeletonExtraContact = 0;
		contact->GetBody0()->m_skeletonExtraContact = 0;
		contact->GetBody1()->m_skeletonExtraContact = 0;
	}

	m_transientLoopingJoints.SetCount(0);
	m_transientLoopingContacts.SetCount(0);
}

void ndSkeletonContainer::CheckSleepState()
{
	ndUnsigned8 equilibrium = 1;
	for (ndInt32 i = m_nodeList.GetCount() - 1; i >= 0; --i)
	{
		ndNode* const node = m_nodesOrder[i];
		ndAssert(node->m_body);
		equilibrium &= node->m_body->m_equilibrium;
	}

	if (!equilibrium)
	{
		for (ndInt32 i = m_nodeList.GetCount() - 1; i >= 0; --i)
		{
			ndNode* const node = m_nodesOrder[i];
			if (node->m_body->GetInvMass() > ndFloat32(0.0f))
			{
				node->m_body->m_equilibrium = 0;
			}
		}
	}
	m_isResting = equilibrium;
}

ndInt32 ndSkeletonContainer::FindBoneIndex(const ndBodyKinematic* const body) const
{
	if (body->GetInvMass() == ndFloat32(0.0f))
	{
		return (m_nodesOrder[m_nodeList.GetCount() - 1]->m_body == body) ? m_nodeList.GetCount() - 1 : -1;
	}
	else
	{
		for (ndInt32 i = m_nodeList.GetCount() - 1; i >= 0; --i)
		{
			const ndNode* const node = m_nodesOrder[i];
			if (node->m_body == body)
			{
				return i;
			}
		}
	}
	return -1;
}

void ndSkeletonContainer::CalculateBufferSizeInBytes()
{
	ndInt32 rowCount = 0;
	ndInt32 auxiliaryRowCount = 0;
	if (m_nodesOrder) 
	{
		const ndInt32 nodeCount = m_nodeList.GetCount() - 1;
		for (ndInt32 i = 0; i < nodeCount; ++i) 
		{
			ndNode* const node = m_nodesOrder[i];
			rowCount += node->m_joint->m_rowCount;
			auxiliaryRowCount += node->GetAuxiliaryRows(m_rightHandSide);
		}
	}

	ndInt32 extraAuxiliaryRows = 0;
	for (ndInt32 i = ndInt32(m_permanentLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_permanentLoopingJoints[i];
		extraAuxiliaryRows += joint->m_rowCount;
	}
	for (ndInt32 i = ndInt32(m_transientLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_transientLoopingJoints[i];
		extraAuxiliaryRows += joint->m_rowCount;
	}
	for (ndInt32 i = ndInt32(m_transientLoopingContacts.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_transientLoopingContacts[i];
		extraAuxiliaryRows += joint->m_rowCount;
	}

	rowCount += extraAuxiliaryRows;
	auxiliaryRowCount += extraAuxiliaryRows;

	ndInt32 size = ndInt32(sizeof(ndInt32) * rowCount);
	size += sizeof(ndInt32) * rowCount;
	size += sizeof(ndNodePair) * rowCount;
	size += sizeof(ndNodePair) * auxiliaryRowCount;

	size += sizeof(ndInt32) * rowCount * 2;
	size += sizeof(ndBodyForceIndexPair) * rowCount * 2;

	size += sizeof(ndFloat32) * auxiliaryRowCount * auxiliaryRowCount;
	size += sizeof(ndFloat32) * auxiliaryRowCount * auxiliaryRowCount;
	size += sizeof(ndFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);
	size += sizeof(ndFloat32) * auxiliaryRowCount * (rowCount - auxiliaryRowCount);

	size = (size + 32 * 16) & -32;
	m_auxiliaryMemoryBuffer.SetCount(size + 1024);
}

void ndSkeletonContainer::CalculateLoopMassMatrixCoefficients(ndThreadPool* const threadPool, ndFloat32* const diagDamp)
{
	D_TRACKTIME();
	auto CalculateLoopMassMatrixCoefficients = ndMakeObject::ndFunction([this, diagDamp](ndInt32 groupId, ndInt32)
	{
		const ndInt32 index = groupId;
		const ndVector8 zero(ndVector8::m_zero);

		const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
		const ndInt32 ii = m_matrixRowsIndex[primaryCount + index];
		const ndLeftHandSide* const row_i = &m_leftHandSide[ii];
		const ndRightHandSide* const rhs_i = &m_rightHandSide[ii];
		
		const ndVector8 JMinvM0(row_i->m_JMinv.m_jacobianM0.m_linear, row_i->m_JMinv.m_jacobianM0.m_angular);
		const ndVector8 JMinvM1(row_i->m_JMinv.m_jacobianM1.m_linear, row_i->m_JMinv.m_jacobianM1.m_angular);
		const ndVector8 element(JMinvM0 * (ndVector8&)row_i->m_Jt.m_jacobianM0 + JMinvM1 * (ndVector8&)row_i->m_Jt.m_jacobianM1);
		
		// I know I am doubling the matrix regularizer, but this makes the solution more robust.
		ndFloat32* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * index];
		ndFloat32 diagonal = element.AddHorizontal() + rhs_i->m_diagDamp;
		matrixRow11[index] = diagonal + rhs_i->m_diagDamp;
		diagDamp[index] = matrixRow11[index] * ndFloat32(4.0e-3f);
		
		const ndInt32 m0_i = m_pairs[primaryCount + index].m_m0;
		const ndInt32 m1_i = m_pairs[primaryCount + index].m_m1;
		for (ndInt32 j = index + 1; j < m_auxiliaryRowCount; ++j)
		{
			const ndInt32 jj = m_matrixRowsIndex[primaryCount + j];
			const ndLeftHandSide* const row_j = &m_leftHandSide[jj];
		
			const ndInt32 k = primaryCount + j;
			const ndInt32 m0_j = m_pairs[k].m_m0;
			const ndInt32 m1_j = m_pairs[k].m_m1;
		
			bool hasEffect = false;
			ndVector8 acc(zero);
			if (m0_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}
			else if (m0_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}
		
			if (m1_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}
			else if (m1_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}
		
			if (hasEffect)
			{
				ndFloat32 offDiagValue = acc.AddHorizontal();
				ndAssert(matrixRow11[j] == ndFloat32 (0.0f));
				matrixRow11[j] = offDiagValue;
				m_massMatrix11[j * m_auxiliaryRowCount + index] = offDiagValue;
			}
		}
		
		ndFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * index];
		for (ndInt32 j = 0; j < primaryCount; ++j)
		{
			const ndInt32 jj = m_matrixRowsIndex[j];
			const ndLeftHandSide* const row_j = &m_leftHandSide[jj];
		
			const ndInt32 m0_j = m_pairs[j].m_m0;
			const ndInt32 m1_j = m_pairs[j].m_m1;
		
			bool hasEffect = false;
		
			ndVector8 acc(zero);
			if (m0_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}
			else if (m0_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}
		
			if (m1_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}
			else if (m1_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}
		
			if (hasEffect)
			{
				ndFloat32 val = acc.AddHorizontal();
				ndAssert(matrixRow10[j]== ndFloat32 (0.0f));
				matrixRow10[j] = val;
			}
		}
	});

	if (threadPool)
	{
		threadPool->ParallelExecute(CalculateLoopMassMatrixCoefficients, m_auxiliaryRowCount, 2);
	}
	else
	{
		for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
		{
			CalculateLoopMassMatrixCoefficients(index, 0);
		}
	}
}

void ndSkeletonContainer::SolveForward(ndForcePair* const force, const ndForcePair* const accel, ndInt32 startNode) const
{
	const ndSpatialVector zero(ndSpatialVector::m_zero);
	for (ndInt32 i = 0; i < startNode; ++i) 
	{
		force[i].m_body = zero;
		force[i].m_joint = zero;
	}

	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = startNode; i < nodeCount - 1; ++i) 
	{
		ndNode* const node = m_nodesOrder[i];
		ndAssert(node->m_joint);
		ndAssert(node->m_index == i);
		ndForcePair& f = force[i];
		const ndForcePair& a = accel[i];
		f.m_body = a.m_body;
		f.m_joint = a.m_joint;
		for (ndNode* child = node->m_child; child; child = child->m_sibling) 
		{
			ndAssert(child->m_joint);
			ndAssert(child->m_parent->m_index == i);
			child->BodyJacobianTimeMassForward(force[child->m_index], f);
		}
		node->JointJacobianTimeMassForward(f);
	}

	force[nodeCount - 1] = accel[nodeCount - 1];
	for (ndNode* child = m_nodesOrder[nodeCount - 1]->m_child; child; child = child->m_sibling) 
	{
		child->BodyJacobianTimeMassForward(force[child->m_index], force[child->m_parent->m_index]);
	}

	for (ndInt32 i = startNode; i < nodeCount - 1; ++i) 
	{
		ndNode* const node = m_nodesOrder[i];
		ndForcePair& f = force[i];
		node->BodyDiagInvTimeSolution(f);
		node->JointDiagInvTimeSolution(f);
	}
	m_nodesOrder[nodeCount - 1]->BodyDiagInvTimeSolution(force[nodeCount - 1]);
}

void ndSkeletonContainer::SolveBackward(ndForcePair* const force) const
{
	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = nodeCount - 2; i >= 0; i--) 
	{
		ndNode* const node = m_nodesOrder[i];
		ndAssert(node->m_index == i);
		ndForcePair& f = force[i];
		node->JointJacobianTimeSolutionBackward(f, force[node->m_parent->m_index]);
		node->BodyJacobianTimeSolutionBackward(f);
	}
}

void ndSkeletonContainer::ConditionMassMatrix(ndThreadPool* const threadPool) const
{
	D_TRACKTIME();
	auto ConditionMassMatrix = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		ndInt32 entry0 = 0;
		const ndInt32 nodeCount = m_nodeList.GetCount();
		const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

		const ndSpatialVector zero(ndSpatialVector::m_zero);
		ndForcePair* const forcePair = ndAlloca(ndForcePair, nodeCount);

		ndInt32 startjoint = nodeCount;
		const ndFloat32* const matrixRow10 = &m_massMatrix10[groupId * primaryCount];
		for (ndInt32 j = 0; j < nodeCount - 1; ++j)
		{
			const ndNode* const node = m_nodesOrder[j];
			const ndInt32 index = node->m_index;
			forcePair[index].m_body = zero;
			ndSpatialVector& a = forcePair[index].m_joint;

			const ndInt32 count = node->m_dof;
			for (ndInt32 k = 0; k < count; ++k)
			{
				const ndFloat32 value = matrixRow10[entry0];
				a[k] = value;
				startjoint = (value == 0.0f) ? startjoint : ndMin(startjoint, index);
				entry0++;
			}
		}

		startjoint = (startjoint == nodeCount) ? 0 : startjoint;
		ndAssert(startjoint < nodeCount);
		forcePair[nodeCount - 1].m_body = zero;
		forcePair[nodeCount - 1].m_joint = zero;
		SolveForward(forcePair, forcePair, startjoint);
		SolveBackward(forcePair);

		ndInt32 entry1 = 0;
		ndFloat32* const deltaForcePtr = &m_deltaForce[groupId * primaryCount];
		for (ndInt32 j = 0; j < nodeCount - 1; ++j)
		{
			const ndNode* const node = m_nodesOrder[j];
			const ndInt32 index = node->m_index;
			const ndSpatialVector& f = forcePair[index].m_joint;
			const ndInt32 count = node->m_dof;
			for (ndInt32 k = 0; k < count; ++k)
			{
				deltaForcePtr[entry1] = ndFloat32(f[k]);
				entry1++;
			}
		}
	});

	if (threadPool)
	{
		threadPool->ParallelExecute(ConditionMassMatrix, m_auxiliaryRowCount, 1);
	}
	else
	{
		for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
		{
			ConditionMassMatrix(index, 0);
		}
	}
}

void ndSkeletonContainer::RebuildMassMatrix(ndThreadPool* const threadPool, const ndFloat32* const diagDamp) const
{
	D_TRACKTIME();
	auto RebuildMassMatrix = ndMakeObject::ndFunction([this, diagDamp](ndInt32 groupId, ndInt32)
	{
		const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
		const ndFloat32* const matrixRow10 = &m_massMatrix10[groupId * primaryCount];
		ndFloat32* const matrixRow11 = &m_massMatrix11[groupId * m_auxiliaryRowCount];

		ndInt32 indexCount = 0;
		ndInt16* const indexList = ndAlloca(ndInt16, primaryCount);
		for (ndInt32 k = 0; k < primaryCount; ++k)
		{
			indexList[indexCount] = ndInt16(k);
			indexCount += (matrixRow10[k] != ndFloat32(0.0f)) ? 1 : 0;
		}

		for (ndInt32 j = groupId; j < m_auxiliaryRowCount; ++j)
		{
			ndFloat32 offDiagonal = matrixRow11[j];
			const ndFloat32* const row10 = &m_deltaForce[j * primaryCount];
			for (ndInt32 k = 0; k < indexCount; ++k)
			{
				ndInt32 index = indexList[k];
				offDiagonal += matrixRow10[index] * row10[index];
			}
			matrixRow11[j] = offDiagonal;
			m_massMatrix11[j * m_auxiliaryRowCount + groupId] = offDiagonal;
		}

		matrixRow11[groupId] = ndMax(matrixRow11[groupId], diagDamp[groupId]);
	});

	if (threadPool)
	{
		threadPool->ParallelExecute(RebuildMassMatrix, m_auxiliaryRowCount, 1);
	}
	else
	{
		for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
		{
			RebuildMassMatrix(index, 0);
		}
	}
}

void ndSkeletonContainer::FactorizeMatrix(ndInt32 size, ndInt32 stride, ndFloat32* const matrix, ndFloat32* const diagDamp) const
{
	D_TRACKTIME();
	// save the matrix 
	ndInt32 srcLine = 0;
	ndInt32 dstLine = 0;
	ndFloat32* const backupMatrix = ndAlloca(ndFloat32, size * stride);
	for (ndInt32 i = 0; i < size; ++i) 
	{
		ndMemCpy(&backupMatrix[dstLine], &matrix[srcLine], size);
		dstLine += size;
		srcLine += stride;
	}

	while (!ndCholeskyFactorization(size, stride, matrix))
	{
		srcLine = 0;
		dstLine = 0;
		for (ndInt32 i = 0; i < size; ++i)
		{
			ndMemCpy(&matrix[dstLine], &backupMatrix[srcLine], size);
			diagDamp[i] *= ndFloat32(4.0f);
			matrix[dstLine + i] += diagDamp[i];
			dstLine += stride;
			srcLine += size;
		}
	}
}

void ndSkeletonContainer::CalculateJointAccel(ndThreadPool* const threadPool, const ndJacobian* const internalForces, ndForcePair* const accel) const
{
	const ndSpatialVector zero(ndSpatialVector::m_zero);

	const ndInt32 nodeCount = m_nodeList.GetCount();
	const ndVector8* const internalForcesArray = (ndVector8*)internalForces;

	auto CalculateJointAccel = ndMakeObject::ndFunction([this, internalForcesArray, accel, &zero](ndInt32 groupId, ndInt32)
	{
		ndNode* const node = m_nodesOrder[groupId];
		ndAssert(groupId == node->m_index);

		ndForcePair& a = accel[groupId];
		ndAssert(node->m_body);
		a.m_body = zero;
		a.m_joint = zero;

		ndAssert(node->m_joint);
		ndJointBilateralConstraint* const joint = node->m_joint;

		const ndInt32 first = joint->m_rowStart;
		const ndInt32 dof = joint->m_rowCount;
		const ndInt32 m0 = joint->GetBody0()->m_index;
		const ndInt32 m1 = joint->GetBody1()->m_index;

		const ndVector8& y0 = internalForcesArray[m0];
		const ndVector8& y1 = internalForcesArray[m1];

		for (ndInt32 j = 0; j < dof; ++j)
		{
			const ndInt32 k = node->m_ordinal.m_sourceJacobianIndex[j];
			const ndLeftHandSide* const row = &m_leftHandSide[first + k];
			const ndRightHandSide* const rhs = &m_rightHandSide[first + k];
			const ndVector8 diag((ndVector8&)row->m_JMinv.m_jacobianM0 * y0 + (ndVector8&)row->m_JMinv.m_jacobianM1 * y1);
			a.m_joint[j] = -(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - diag.AddHorizontal());
		}
	});

	if (threadPool)
	{
		threadPool->ParallelExecute(CalculateJointAccel, nodeCount - 1, 2);
	}
	else
	{
		for (ndInt32 index = 0; index < nodeCount - 1; ++index)
		{
			CalculateJointAccel(index, 0);
		}
	}

	ndAssert((nodeCount - 1) == m_nodesOrder[nodeCount - 1]->m_index);
	accel[nodeCount - 1].m_body = zero;
	accel[nodeCount - 1].m_joint = zero;
}

void ndSkeletonContainer::CalculateForce(ndForcePair* const force, const ndForcePair* const accel) const
{
	SolveForward(force, accel, 0);
	SolveBackward(force);
}

void ndSkeletonContainer::UpdateForces(ndJacobian* const internalForces, const ndForcePair* const force) const
{
	const ndVector zero(ndVector::m_zero);
	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = 0; i < (nodeCount - 1); ++i) 
	{
		ndNode* const node = m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;

		ndJacobian y0;
		ndJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		ndAssert(i == node->m_index);

		const ndSpatialVector& f = force[i].m_joint;
		const ndInt32 first = joint->m_rowStart;
		const ndInt32 count = node->m_dof;
		for (ndInt32 j = 0; j < count; ++j)  
		{
			const ndInt32 k = node->m_ordinal.m_sourceJacobianIndex[j];
			const ndLeftHandSide* const row = &m_leftHandSide[first + k];

			ndVector jointForce = ndFloat32(f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
		}

		const ndInt32 m0 = joint->GetBody0()->m_index;
		const ndInt32 m1 = joint->GetBody1()->m_index;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}
}

void ndSkeletonContainer::CalculateBodyImpulses(ndJacobian* const bodyImpulse, const ndForcePair* const jointImpulse) const
{
	const ndVector zero(ndVector::m_zero);
	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = 0; i < nodeCount; ++i)
	{
		bodyImpulse[i].m_linear = zero;
		bodyImpulse[i].m_angular = zero;
	}

	for (ndInt32 i = 0; i < (nodeCount - 1); ++i)
	{
		ndNode* const node = m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;

		ndJacobian y0;
		ndJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		ndAssert(i == node->m_index);

		const ndSpatialVector& f = jointImpulse[i].m_joint;
		const ndInt32 first = joint->m_rowStart;
		const ndInt32 count = node->m_dof;
		for (ndInt32 j = 0; j < count; ++j)
		{
			const ndInt32 k = node->m_ordinal.m_sourceJacobianIndex[j];
			const ndLeftHandSide* const row = &m_leftHandSide[first + k];

			ndVector jointForce = ndFloat32(f[j]);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear * jointForce;
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular * jointForce;
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear * jointForce;
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular * jointForce;
		}

		//const ndInt32 m0 = joint->GetBody0()->m_index;
		//const ndInt32 m1 = joint->GetBody1()->m_index;
		const ndInt32 m0 = node->m_index;
		const ndInt32 m1 = node->m_parent->m_index;

		bodyImpulse[m0].m_linear += y0.m_linear;
		bodyImpulse[m0].m_angular += y0.m_angular;
		bodyImpulse[m1].m_linear += y1.m_linear;
		bodyImpulse[m1].m_angular += y1.m_angular;
	}
}

void ndSkeletonContainer::CalculateExtraBodyImpulses(ndJacobian* const bodyImpulse, const ndForcePair* const jointVeloc, const ndForcePair* const jointImpulse) const
{
	ndTrace(("TO DO, nothing for now\n"));
}

void ndSkeletonContainer::SolveLcp(ndInt32 stride, ndInt32 size, ndFloat32* const x, const ndFloat32* const b, const ndFloat32* const low, const ndFloat32* const high, const ndInt32* const normalIndex, ndFloat32 accelTol) const
{
	D_TRACKTIME();
	// better chance for auto vectorization. 
	const ndFloat32 tol2 = accelTol * accelTol;
	const ndFloat32* const matrix = &m_precondinonedMassMatrix11[0];
	ndAssert(ndTestPSDmatrix(size, stride, matrix));

	ndFloat32* const bScaled = ndAlloca(ndFloat32, stride);

	for (ndInt32 i = 0; i < size; ++i)
	{
		const ndInt32 index = normalIndex[i] + i;
		x[i] /= m_diagonalPreconditioner[i];
		bScaled[i] = b[i] * m_diagonalPreconditioner[i];

		const ndFloat32 coefficient = x[index];
	
		const ndFloat32 l = low[i] * coefficient;
		const ndFloat32 h = high[i] * coefficient;
	
		x[i] = ndClamp(x[i], l, h);
	}

	const ndInt32 maxIterCount = 64;
	ndFloat32 error2 = tol2 * ndFloat32(2.0f);
	//const ndFloat32 sor = ndFloat32(1.0f);
	const ndFloat32 sor = ndFloat32(1.125f);
	for (ndInt32 m = maxIterCount; (m >= 0) && (error2 > tol2); --m)
	{
		ndInt32 rowBase = 0;
		error2 = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < size; ++i)
		{
			const ndFloat32* const row = &matrix[rowBase];
			ndFloat32 r = bScaled[i];
			for (ndInt32 j = 0; j < size; ++j)
			{
				r -= row[j] * x[j];
			}
	
			const ndInt32 index = normalIndex[i] + i;
			const ndFloat32 coefficient = x[index];
			const ndFloat32 l = low[i] * coefficient;
			const ndFloat32 h = high[i] * coefficient;
			const ndFloat32 x0 = x[i];
			const ndFloat32 x1 = x0 + r;
			const ndFloat32 x2 = x0 + (x1 - x0) * sor;
			const ndFloat32 f = ndClamp(x2, l, h);
			ndAssert(ndCheckFloat(f));
	
			const ndFloat32 dx = f - x0;
			const ndFloat32 dr = dx * row[i];
			error2 += dr * dr;
			x[i] = f;
	
			rowBase += stride;
		}
	}
																															
	for (ndInt32 i = 0; i < size; ++i)
	{
		x[i] *= m_diagonalPreconditioner[i];
	}
}



void ndSkeletonContainer::RegularizeLcp() const
{
	ndInt32 size = m_auxiliaryRowCount - m_blockSize;
	ndFloat32* const matrix = &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize];
	if (!ndTestPSDmatrix(size, m_auxiliaryRowCount, matrix))
	{
		ndFloat32* const regulatiser = ndAlloca(ndFloat32, size);
		ndMemSet(regulatiser, ndFloat32(1.01f), size);
		ndInt32 step = m_auxiliaryRowCount + 1;
		//ndInt32 passes = 0;
		ndFloat32 reg = ndFloat32 (1.125f);
		do 
		{
			ndFloat32* base = &m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount - 1];
			for (ndInt32 i = size - 1; i >= 0; --i)
			{
				*base = *base * reg;
				base -= step;
			}
			reg *= ndFloat32(1.125f);
		} while (!ndTestPSDmatrix(size, m_auxiliaryRowCount, matrix));
	}
}

void ndSkeletonContainer::SolveBlockLcp(ndInt32 size, ndInt32 blockSize, ndFloat32* const x, ndFloat32* const b, const ndFloat32* const low, const ndFloat32* const high, const ndInt32* const normalIndex, ndFloat32 accelTol) const
{
	if (blockSize) 
	{
		ndSolveCholesky(blockSize, size, m_massMatrix11, x, b);
		if (blockSize != size) 
		{
			ndInt32 base = blockSize * size;
			for (ndInt32 i = blockSize; i < size; ++i) 
			{
				b[i] -= ndDotProduct(blockSize, &m_massMatrix11[base], x);
				base += size;
			}

			const ndInt32 boundedSize = size - blockSize;
			SolveLcp(size, boundedSize, &x[blockSize], &b[blockSize], &low[blockSize], &high[blockSize], &normalIndex[blockSize], accelTol);

			for (ndInt32 j = 0; j < blockSize; ++j)  
			{
				const ndFloat32* const row = &m_massMatrix11[j * size + blockSize];
				ndFloat32 acc = ndFloat32(0.0f);
				for (ndInt32 i = 0; i < boundedSize; ++i) 
				{
					acc += x[blockSize + i] * row[i];
				}
				x[j] += acc;
			}
		}
	}
	else 
	{
		SolveLcp(size, size, x, b, low, high, normalIndex, accelTol);
	}
}

void ndSkeletonContainer::AddExtraContacts()
{
	ndFixSizeArray<ndBodyKinematic*, 1024> queue;
	for (ndInt32 i = ndInt32(m_transientLoopingContacts.GetCount() - 1); i >= 0; --i)
	{
		ndContact* const contact = m_transientLoopingContacts[i];
		ndAssert(contact->IsActive());
		ndAssert(contact->m_skeletonExtraContact == 0);
		contact->m_skeletonExtraContact = 1;
		ndBodyKinematic* const collidingBody = (contact->GetBody0()->GetSkeleton() == this) ? contact->GetBody1() : contact->GetBody0();
		if (collidingBody->GetSkeleton() != this)
		{
			if (!collidingBody->m_skeletonExtraContact && (collidingBody->GetInvMass() != ndFloat32(0.0f)))
			{
				queue.PushBack(collidingBody);
				collidingBody->m_equilibrium = 0;
				collidingBody->m_skeletonExtraContact = 1;
			}
		}
	}

	ndInt32 numberOfLayers = 5;
	for (ndInt32 j = numberOfLayers - 1; j >= 0; --j)
	{
		ndFixSizeArray<ndBodyKinematic*, 1024> layer;
		for (ndInt32 i = queue.GetCount() - 1; i >= 0; --i)
		{
			layer.PushBack(queue.Pop());
		}
		for (ndInt32 i = layer.GetCount() - 1; i >= 0; --i)
		{
			ndBodyKinematic* const body = layer.Pop();
			ndAssert(!body->m_equilibrium);
			ndBodyKinematic::ndContactMap::Iterator it(body->GetContactMap());
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = *it;
				if (contact->IsActive() && !contact->m_skeletonExtraContact)
				{
					contact->m_skeletonExtraContact = 1;
					AddCloseLoopJoint(contact);
					ndBodyKinematic* const child = (contact->GetBody0() == body) ? contact->GetBody1() : contact->GetBody0();
					if (child->GetInvMass() > ndFloat32(0.0f))
					{
						if (!child->m_equilibrium)
						{
							queue.PushBack(child);
							ndAssert(!body->m_equilibrium);
							child->m_skeletonExtraContact = 1;
						}
						child->m_equilibrium = 0;
					}
				}
			}
		}
	}

	for (ndInt32 i = ndInt32(m_transientLoopingContacts.GetCount() - 1); i >= 0; --i)
	{
		ndContact* const contact = m_transientLoopingContacts[i];
		ndAssert(contact->GetAsContact());
		contact->m_skeletonExtraContact = 0;
		contact->GetBody0()->m_skeletonExtraContact = 0;
		contact->GetBody1()->m_skeletonExtraContact = 0;
	}
}

void ndSkeletonContainer::CalculateJointAccelImmediate(ndForcePair* const accel) const
{
	const ndSpatialVector zero(ndSpatialVector::m_zero);
	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = 0; i < nodeCount - 1; ++i)
	{
		ndNode* const node = m_nodesOrder[i];
		ndAssert(i == node->m_index);

		ndForcePair& a = accel[i];
		ndAssert(node->m_body);
		ndBodyKinematic* const body = node->m_body;
		
		const ndVector& force = body->m_accel;
		const ndVector& torque = body->m_alpha;

		a.m_body.m_f[0] = force.m_x;
		a.m_body.m_f[1] = force.m_y;
		a.m_body.m_f[2] = force.m_z;
		a.m_body.m_f[3] = torque.m_x;
		a.m_body.m_f[4] = torque.m_y;
		a.m_body.m_f[5] = torque.m_z;
		a.m_body.m_f[6] = ndFloat64(0.0f);
		a.m_body.m_f[7] = ndFloat64(0.0f);

		ndAssert(node->m_joint);
		ndJointBilateralConstraint* const joint = node->m_joint;

		const ndInt32 first = joint->m_rowStart;
		const ndInt32 dof = joint->m_rowCount;

		a.m_joint = zero;
		for (ndInt32 j = 0; j < dof; ++j)
		{
			const ndInt32 k = node->m_ordinal.m_sourceJacobianIndex[j];
			const ndRightHandSide* const rhs = &m_rightHandSide[first + k];
			a.m_joint[j] = -rhs->m_coordenateAccel;
		}
	}

	ndAssert((nodeCount - 1) == m_nodesOrder[nodeCount - 1]->m_index);
	ndForcePair& a = accel[nodeCount - 1];
	a.m_joint = zero;

	ndNode* const node = m_nodesOrder[nodeCount - 1];
	ndAssert(node->m_body);
	ndBodyKinematic* const body = node->m_body;
	const ndVector& force = body->m_accel;
	const ndVector& torque = body->m_alpha;

	a.m_body.m_f[0] = force.m_x;
	a.m_body.m_f[1] = force.m_y;
	a.m_body.m_f[2] = force.m_z;
	a.m_body.m_f[3] = torque.m_x;
	a.m_body.m_f[4] = torque.m_y;
	a.m_body.m_f[5] = torque.m_z;
	a.m_body.m_f[6] = ndFloat64(0.0f);
	a.m_body.m_f[7] = ndFloat64(0.0f);
}

void ndSkeletonContainer::UpdateForcesImmediate(const ndForcePair* const force) const
{
	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = 0; i < nodeCount; ++i)
	{
		ndNode* const node = m_nodesOrder[i];
		ndBodyKinematic* const body = node->m_body;
		ndAssert(body);
		const ndSpatialVector& spatialAccel = force[i].m_body;
		body->m_accel = ndVector (ndFloat32(spatialAccel.m_f[0]), ndFloat32(spatialAccel.m_f[1]), ndFloat32(spatialAccel.m_f[2]), ndFloat32(0.0f));
		body->m_alpha = ndVector(ndFloat32(spatialAccel.m_f[3]), ndFloat32(spatialAccel.m_f[4]), ndFloat32(spatialAccel.m_f[5]), ndFloat32(0.0f));
	}
}

void ndSkeletonContainer::SolveAuxiliaryImmediate(ndArray<ndBodyKinematic*>& bodyArray, ndForcePair* const force) const
{
	ndFloat32* const f = ndAlloca(ndFloat32, m_rowCount);
	ndFloat32* const b = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const low = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const high = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const u = ndAlloca(ndFloat32, m_auxiliaryRowCount + 1);

	ndInt32 primaryIndex = 0;
	const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	const ndVector zero(ndVector::m_zero);
	const ndInt32 nodeCount = m_nodeList.GetCount();
	for (ndInt32 i = 0; i < nodeCount - 1; ++i)
	{
		const ndNode* const node = m_nodesOrder[i];
		const ndInt32 primaryDof = node->m_dof;
		const ndSpatialVector& forceSpatial = force[i].m_joint;

		for (ndInt32 j = 0; j < primaryDof; ++j)
		{
			f[primaryIndex] = ndFloat32(forceSpatial[j]);
			primaryIndex++;
		}
	}

	ndAssert(primaryIndex == primaryCount);
	for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
	{
		const ndInt32 index = m_matrixRowsIndex[primaryCount + i];
		const ndLeftHandSide* const row = &m_leftHandSide[index];
		const ndRightHandSide* const rhs = &m_rightHandSide[index];

		const ndInt32 m0 = m_pairs[primaryCount + i].m_m0;
		const ndInt32 m1 = m_pairs[primaryCount + i].m_m1;

		const ndBodyKinematic* const body0 = bodyArray[m0];
		const ndBodyKinematic* const body1 = bodyArray[m1];

		const ndVector& force0 = body0->m_accel;
		const ndVector& torque0 = body0->m_alpha;
		const ndVector& force1 = body1->m_accel;
		const ndVector& torque1 = body1->m_alpha;
		ndAssert((primaryCount + i) < m_rowCount);

		const ndVector acc(
			row->m_JMinv.m_jacobianM0.m_linear * force0 + row->m_JMinv.m_jacobianM0.m_angular * torque0 +
			row->m_JMinv.m_jacobianM1.m_linear * force1 + row->m_JMinv.m_jacobianM1.m_angular * torque1);
		b[i] = rhs->m_coordenateAccel - acc.AddHorizontal().GetScalar();

		ndFloat32* const matrixRow10 = &m_massMatrix10[i * primaryCount];
		b[i] -= ndDotProduct(primaryCount, matrixRow10, f);

		u[i] = rhs->m_force;
		low[i] = rhs->m_lowerBoundFrictionCoefficent;
		high[i] = rhs->m_upperBoundFrictionCoefficent;
		ndAssert(rhs->SanityCheck());
	}

	u[m_auxiliaryRowCount] = ndFloat32(1.0f);
	const ndInt32* const normalIndex = &m_frictionIndex[primaryCount];

	SolveBlockLcp(m_auxiliaryRowCount, m_blockSize, u, b, low, high, normalIndex, ndFloat32 (0.1f));
	for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
	{
		const ndFloat32 s = u[i];
		f[primaryCount + i] = s;
		ndScaleAdd(primaryCount, f, &m_deltaForce[i * primaryCount], s);
	}

	for (ndInt32 i = 0; i < m_rowCount; ++i)
	{
		ndConstraint* const joint = (ndConstraint*)m_pairs[i].m_joint;

		ndInt32 index = m_matrixRowsIndex[i];
		const ndLeftHandSide* const row = &m_leftHandSide[index];
		const ndVector jointForce(f[i]);

		const ndVector force0(row->m_Jt.m_jacobianM0.m_linear * jointForce);
		const ndVector torque0(row->m_Jt.m_jacobianM0.m_angular * jointForce);
		const ndVector force1(row->m_Jt.m_jacobianM1.m_linear * jointForce);
		const ndVector torque1(row->m_Jt.m_jacobianM1.m_angular * jointForce);
		
		const ndInt32 m0 = (joint->GetBody0()->GetInvMass() > ndFloat32(0.0f)) ? joint->GetBody0()->m_index : 0;
		const ndInt32 m1 = (joint->GetBody1()->GetInvMass() > ndFloat32(0.0f)) ? joint->GetBody1()->m_index : 0;
		ndBodyKinematic* const body0 = bodyArray[m0];
		ndBodyKinematic* const body1 = bodyArray[m1];

		body0->m_accel += force0;
		body0->m_alpha += torque0;
		body1->m_accel += force1;
		body1->m_alpha += torque1;
	}

	for (ndInt32 i = ndInt32(bodyArray.GetCount()) - 1; i >= 0; --i)
	{
		ndBodyKinematic* const body = bodyArray[i];
		const ndMatrix& invInertia = body->GetInvInertiaMatrix();
		body->m_alpha = invInertia.RotateVector(body->m_alpha);
		body->m_accel = body->m_accel.Scale (body->GetInvMass());
	}
}

void ndSkeletonContainer::SolveImmediate(ndIkSolver& solverInfo)
{
	D_TRACKTIME();
	const ndInt32 nodeCount = m_nodeList.GetCount();
	ndForcePair* const x = ndAlloca(ndForcePair, nodeCount);
	ndForcePair* const b = ndAlloca(ndForcePair, nodeCount);

	CalculateJointAccelImmediate(b);
	CalculateForce(x, b);
	if (m_auxiliaryRowCount)
	{
		SolveAuxiliaryImmediate(solverInfo.m_bodies, x);
	}
	else
	{
		UpdateForcesImmediate(x);
	}
}

void ndSkeletonContainer::InitLoopMassMatrix(ndThreadPool* const threadPool)
{
	CalculateBufferSizeInBytes();
	ndInt8* const memoryBuffer = &m_auxiliaryMemoryBuffer[0];
	const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	#define ndAlignedPtr(type, ptr) (type*)((ndUnsigned64(ptr) + 31) & -0x20)

	m_frictionIndex = ndAlignedPtr(ndInt32, memoryBuffer);
	m_matrixRowsIndex = ndAlignedPtr(ndInt32, &m_frictionIndex[m_rowCount]);

	m_bodyForceRemap0.m_index = ndAlignedPtr(ndBodyForceIndexPair, &m_matrixRowsIndex[m_rowCount]);
	m_bodyForceRemap0.m_indexSpan = ndAlignedPtr(ndInt32, &m_bodyForceRemap0.m_index[m_rowCount]);
	m_bodyForceRemap1.m_index = ndAlignedPtr(ndBodyForceIndexPair, &m_bodyForceRemap0.m_indexSpan[m_rowCount]);
	m_bodyForceRemap1.m_indexSpan = ndAlignedPtr(ndInt32, &m_bodyForceRemap1.m_index[m_rowCount]);

	m_pairs = ndAlignedPtr(ndNodePair, &m_bodyForceRemap1.m_indexSpan[m_rowCount]);
	m_diagonalPreconditioner = ndAlignedPtr(ndFloat32, &m_pairs[m_rowCount]);
	m_precondinonedMassMatrix11 = ndAlignedPtr(ndFloat32, &m_diagonalPreconditioner[m_rowCount]);

	m_massMatrix11 = ndAlignedPtr(ndFloat32, &m_precondinonedMassMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount]);
	m_massMatrix10 = ndAlignedPtr(ndFloat32, &m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount]);
	m_deltaForce = ndAlignedPtr(ndFloat32, &m_massMatrix10[m_auxiliaryRowCount * primaryCount]);

	ndInt32* const boundRow = ndAlloca(ndInt32, m_auxiliaryRowCount);

	m_blockSize = 0;
	ndInt32 primaryIndex = 0;
	ndInt32 auxiliaryIndex = 0;
	{
		const ndInt32 nodeCount = m_nodeList.GetCount() - 1;
		for (ndInt32 i = 0; i < nodeCount; ++i)
		{
			const ndNode* const node = m_nodesOrder[i];
			ndJointBilateralConstraint* const joint = node->m_joint;
			const ndInt32 m0 = joint->GetBody0()->m_index;
			const ndInt32 m1 = joint->GetBody1()->m_index;
			const ndInt32 primaryDof = node->m_dof;
			const ndInt32 first = joint->m_rowStart;
			for (ndInt32 j = 0; j < primaryDof; ++j)
			{
				const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[j];
				m_pairs[primaryIndex].m_m0 = m0;
				m_pairs[primaryIndex].m_m1 = m1;
				m_pairs[primaryIndex].m_joint = joint;
				m_frictionIndex[primaryIndex] = 0;
				m_matrixRowsIndex[primaryIndex] = first + index;
				primaryIndex++;
			}

			const ndInt32 auxiliaryDof = joint->m_rowCount - primaryDof;
			for (ndInt32 j = 0; j < auxiliaryDof; ++j)
			{
				const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[primaryDof + j];
				const ndRightHandSide* const rhs = &m_rightHandSide[first + index];

				m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
				m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
				m_pairs[auxiliaryIndex + primaryCount].m_joint = joint;
				m_frictionIndex[auxiliaryIndex + primaryCount] = m_auxiliaryRowCount - auxiliaryIndex;
				m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + index;
				const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
				ndAssert(joint->IsBilateral());
				ndAssert(rhs->SanityCheck());
				boundRow[auxiliaryIndex] = boundIndex;
				m_blockSize += boundIndex;
				auxiliaryIndex++;
			}
		}
	}
	ndAssert(m_loopRowCount == (m_auxiliaryRowCount - auxiliaryIndex));
	
	for (ndInt32 j = ndInt32(m_permanentLoopingJoints.GetCount() - 1); j >= 0; --j)
	{
		const ndJointBilateralConstraint* const joint = m_permanentLoopingJoints[j];
		const ndInt32 m0 = joint->GetBody0()->m_index;
		const ndInt32 m1 = joint->GetBody1()->m_index;

		const ndInt32 first = joint->m_rowStart;
		const ndInt32 auxiliaryDof = joint->m_rowCount;
		for (ndInt32 i = 0; i < auxiliaryDof; ++i)
		{
			const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_pairs[auxiliaryIndex + primaryCount].m_joint = joint;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			m_frictionIndex[auxiliaryIndex + primaryCount] = (rhs->m_normalForceIndex < 0) ? m_auxiliaryRowCount - auxiliaryIndex : rhs->m_normalForceIndex - i;
			const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
			ndAssert(rhs->SanityCheck());
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}

	for (ndInt32 j = ndInt32(m_transientLoopingJoints.GetCount() - 1); j >= 0; --j)
	{
		const ndJointBilateralConstraint* const joint = m_transientLoopingJoints[j];
		const ndInt32 m0 = joint->GetBody0()->m_index;
		const ndInt32 m1 = joint->GetBody1()->m_index;

		const ndInt32 first = joint->m_rowStart;
		const ndInt32 auxiliaryDof = joint->m_rowCount;
		for (ndInt32 i = 0; i < auxiliaryDof; ++i)
		{
			const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_pairs[auxiliaryIndex + primaryCount].m_joint = joint;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			m_frictionIndex[auxiliaryIndex + primaryCount] = (rhs->m_normalForceIndex < 0) ? m_auxiliaryRowCount - auxiliaryIndex : rhs->m_normalForceIndex - i;
			ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
			ndAssert(rhs->SanityCheck());
			boundRow[auxiliaryIndex] = boundIndex;
			m_blockSize += boundIndex;
			auxiliaryIndex++;
		}
	}

	ndInt32 contactRowsStart = auxiliaryIndex;
	for (ndInt32 j = ndInt32(m_transientLoopingContacts.GetCount() - 1); j >= 0; --j)
	{
		const ndContact* const joint = m_transientLoopingContacts[j];
		const ndInt32 m0 = joint->GetBody0()->m_index;
		const ndInt32 m1 = joint->GetBody1()->m_index;

		const ndInt32 first = joint->m_rowStart;
		const ndInt32 auxiliaryDof = joint->m_rowCount;
		for (ndInt32 i = 0; i < auxiliaryDof; ++i)
		{
			const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
			m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
			m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
			m_pairs[auxiliaryIndex + primaryCount].m_joint = joint;
			m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + i;
			m_frictionIndex[auxiliaryIndex + primaryCount] = (rhs->m_normalForceIndex < 0) ? m_auxiliaryRowCount - auxiliaryIndex : rhs->m_normalForceIndex - i;
			ndAssert(rhs->SanityCheck());
			ndAssert((rhs->m_lowerBoundFrictionCoefficent > ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) || (rhs->m_upperBoundFrictionCoefficent < ndFloat32(D_MAX_SKELETON_LCP_VALUE)) && ((ndConstraint*)joint)->GetAsContact());
			boundRow[auxiliaryIndex] = 0;
			auxiliaryIndex++;
		}
	}

	ndAssert(primaryIndex == primaryCount);
	ndAssert(auxiliaryIndex == m_auxiliaryRowCount);
	ndAssert(m_frictionIndex[primaryCount] == m_auxiliaryRowCount);

	for (ndInt32 i = 1; i < contactRowsStart; ++i)
	{
		ndInt32 tmpBoundRow = boundRow[i];
		ndNodePair tmpPair(m_pairs[primaryCount + i]);
		ndInt32 tmpMatrixRowsIndex = m_matrixRowsIndex[primaryCount + i];
		ndAssert((m_frictionIndex[primaryCount + i] + i) == m_auxiliaryRowCount);

		ndInt32 j = i;
		for (; j && (boundRow[j - 1] < tmpBoundRow); --j)
		{
			ndAssert(j > 0);
			ndAssert(m_frictionIndex[primaryCount + j - 1] > 0);

			boundRow[j] = boundRow[j - 1];
			m_pairs[primaryCount + j] = m_pairs[primaryCount + j - 1];
			m_matrixRowsIndex[primaryCount + j] = m_matrixRowsIndex[primaryCount + j - 1];
		}
		boundRow[j] = tmpBoundRow;
		m_pairs[primaryCount + j] = tmpPair;
		m_matrixRowsIndex[primaryCount + j] = tmpMatrixRowsIndex;
	}

	ndFloat32* const diagDamp = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndMemSet(m_massMatrix10, ndFloat32(0.0f), primaryCount * m_auxiliaryRowCount);
	ndMemSet(m_massMatrix11, ndFloat32(0.0f), m_auxiliaryRowCount * m_auxiliaryRowCount);

	CalculateLoopMassMatrixCoefficients(threadPool, diagDamp);
	ConditionMassMatrix(threadPool);
	RebuildMassMatrix(threadPool, diagDamp);

	if (m_blockSize)
	{
		FactorizeMatrix(m_blockSize, m_auxiliaryRowCount, m_massMatrix11, diagDamp);

		ndInt32 rowStart = 0;
		const ndInt32 boundedSize = m_auxiliaryRowCount - m_blockSize;
		ndFloat32* const acc = ndAlloca(ndFloat32, m_auxiliaryRowCount);

		for (ndInt32 i = 0; i < m_blockSize; ++i)
		{
			ndMemSet(acc, ndFloat32(0.0f), boundedSize);
			const ndFloat32* const row = &m_massMatrix11[rowStart];
			for (ndInt32 j = 0; j < i; ++j)
			{
				const ndFloat32 s = row[j];
				const ndFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (ndInt32 k = 0; k < boundedSize; ++k)
				{
					acc[k] += s * x[k];
				}
			}

			ndFloat32* const x = &m_massMatrix11[rowStart + m_blockSize];
			const ndFloat32 den = -ndFloat32(1.0f) / row[i];
			for (ndInt32 j = 0; j < boundedSize; ++j)
			{
				x[j] = (x[j] + acc[j]) * den;
			}
			rowStart += m_auxiliaryRowCount;
		}

		for (ndInt32 i = m_blockSize - 1; i >= 0; i--)
		{
			ndMemSet(acc, ndFloat32(0.0f), boundedSize);
			for (ndInt32 j = i + 1; j < m_blockSize; ++j)
			{
				const ndFloat32 s = m_massMatrix11[j * m_auxiliaryRowCount + i];
				const ndFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (ndInt32 k = 0; k < boundedSize; ++k)
				{
					acc[k] += s * x[k];
				}
			}

			ndFloat32* const x = &m_massMatrix11[i * m_auxiliaryRowCount + m_blockSize];
			const ndFloat32 den = ndFloat32(1.0f) / m_massMatrix11[i * m_auxiliaryRowCount + i];
			for (ndInt32 j = 0; j < boundedSize; ++j)
			{
				x[j] = (x[j] - acc[j]) * den;
			}
		}

		auto InitMassMatrixBoundedBlock = ndMakeObject::ndFunction([this, boundedSize, diagDamp](ndInt32 groupId, ndInt32)
		{
			ndFixSizeArray<ndFloat32, 1024> acc(m_blockSize);
			ndAssert(m_blockSize <= acc.GetCapacity());
			for (ndInt32 j = 0; j < m_blockSize; ++j)
			{
				acc[j] = m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize + groupId];
			}
				
			ndFloat32* const arow = &m_massMatrix11[(m_blockSize + groupId) * m_auxiliaryRowCount + m_blockSize];
			for (ndInt32 j = groupId + 1; j < boundedSize; ++j)
			{
				const ndFloat32* const row1 = &m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount];
				ndFloat32 elem = row1[m_blockSize + groupId] + ndDotProduct(m_blockSize, &acc[0], row1);
				arow[j] = elem;
				m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount + m_blockSize + groupId] = elem;
			}
			const ndFloat32* const row1 = &m_massMatrix11[(m_blockSize + groupId) * m_auxiliaryRowCount];
			ndFloat32 elem = row1[m_blockSize + groupId] + ndDotProduct(m_blockSize, &acc[0], row1);
			arow[groupId] = elem + diagDamp[m_blockSize + groupId];
		});

		if (threadPool)
		{
			threadPool->ParallelExecute(InitMassMatrixBoundedBlock, boundedSize, 1);
		}
		else
		{
			for (ndInt32 index = 0; index < boundedSize; ++index)
			{
				InitMassMatrixBoundedBlock(index, 0);
			}
		}
		ndAssert(!boundedSize || ndTestPSDmatrix(m_auxiliaryRowCount - m_blockSize, m_auxiliaryRowCount, &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize]));
	}

	if (m_transientLoopingContacts.GetCount() || m_transientLoopingJoints.GetCount())
	{
		RegularizeLcp();
	}

	auto SortIndexArray = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		class CompareKey
		{
			public:
			CompareKey(void* const)
			{
			}

			ndInt32 Compare(const ndBodyForceIndexPair& elementA, const ndBodyForceIndexPair& elementB) const
			{
				ndInt32 indexA = (elementA.m_bodyIndex << 16) + elementA.m_forceIndex;
				ndInt32 indexB = (elementB.m_bodyIndex << 16) + elementB.m_forceIndex;
				if (indexA < indexB)
				{
					return -1;
				}
				else if (indexA > indexB)
				{
					return 1;
				}
				return 0;
			}
		};
		ndBodyForcePtr& bodyForceRemap = groupId ? m_bodyForceRemap1 : m_bodyForceRemap0;
		ndSort<ndBodyForceIndexPair, CompareKey>(bodyForceRemap.m_index, m_rowCount, nullptr);

		ndInt32 spanIndex = 0;
		for (ndInt32 i = 0; i < m_rowCount; ++i)
		{
			bodyForceRemap.m_indexSpan[spanIndex] = i;
			ndInt32 test = bodyForceRemap.m_index[i].m_bodyIndex;
			for (++i; (i < m_rowCount) && (bodyForceRemap.m_index[i].m_bodyIndex == test); ++i);
			i--;
			spanIndex++;
		}
		ndAssert(spanIndex < m_rowCount);
		bodyForceRemap.m_indexSpan[spanIndex] = ndInt16(m_rowCount);
		bodyForceRemap.m_spansCount = spanIndex;
	});

	for (ndInt32 i = 0; i < m_rowCount; ++i)
	{
		const ndInt32 m0 = m_pairs[i].m_m0;
		const ndInt32 m1 = m_pairs[i].m_m1;
		m_bodyForceRemap0.m_index[i].m_bodyIndex = m0;
		m_bodyForceRemap0.m_index[i].m_forceIndex = i;
		m_bodyForceRemap1.m_index[i].m_bodyIndex = m1;
		m_bodyForceRemap1.m_index[i].m_forceIndex = i;
	}

	if (threadPool)
	{
		threadPool->ParallelExecute(SortIndexArray, 2, 1);
	}
	else
	{
		SortIndexArray(0, 0);
		SortIndexArray(1, 0);
	}
}

void ndSkeletonContainer::SolveAuxiliary(ndThreadPool* const threadPool, ndJacobian* const internalForces, const ndForcePair* const, ndForcePair* const force) const
{
	ndFloat32* const f = ndAlloca(ndFloat32, m_rowCount);
	ndFloat32* const b = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const low = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const high = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const u = ndAlloca(ndFloat32, m_auxiliaryRowCount + 1);

	ndInt32 primaryIndex = 0;
	const ndInt32 nodeCount = m_nodeList.GetCount();
	const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (ndInt32 i = 0; i < nodeCount - 1; ++i)
	{
		const ndNode* const node = m_nodesOrder[i];
		const ndInt32 primaryDof = node->m_dof;
		const ndSpatialVector& forceSpatial = force[i].m_joint;

		for (ndInt32 j = 0; j < primaryDof; ++j)
		{
			f[primaryIndex] = ndFloat32(forceSpatial[j]);
			primaryIndex++;
		}
	}

	ndAssert(primaryIndex == primaryCount);
	
	ndVector8* const internalForcesArray = (ndVector8*)internalForces;
	auto SolveAuxiliary_1 = ndMakeObject::ndFunction([this, primaryCount, u, f, b, low, high, internalForcesArray](ndInt32 groupId, ndInt32)
	{
		const ndInt32 index = m_matrixRowsIndex[primaryCount + groupId];
		const ndLeftHandSide* const row = &m_leftHandSide[index];
		const ndRightHandSide* const rhs = &m_rightHandSide[index];

		const ndInt32 m0 = m_pairs[primaryCount + groupId].m_m0;
		const ndInt32 m1 = m_pairs[primaryCount + groupId].m_m1;

		const ndVector8& y0 = internalForcesArray[m0];
		const ndVector8& y1 = internalForcesArray[m1];

		const ndVector8 acc((ndVector8&)row->m_JMinv.m_jacobianM0 * y0 + (ndVector8&)row->m_JMinv.m_jacobianM1 * y1);
		b[groupId] = rhs->m_coordenateAccel - acc.AddHorizontal();

		const ndFloat32* const matrixRow10 = &m_massMatrix10[groupId * primaryCount];
		b[groupId] -= ndDotProduct(primaryCount, matrixRow10, f);

		u[groupId] = rhs->m_force;
		low[groupId] = rhs->m_lowerBoundFrictionCoefficent;
		high[groupId] = rhs->m_upperBoundFrictionCoefficent;
		ndAssert(rhs->SanityCheck());
	});

	//for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
	if (threadPool)
	{
		threadPool->ParallelExecute(SolveAuxiliary_1, m_auxiliaryRowCount, 1);
	}
	else
	{
		for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
		{
			SolveAuxiliary_1(index, 0);
		}
	}

	const ndInt32* const normalIndex = &m_frictionIndex[primaryCount];
	u[m_auxiliaryRowCount] = ndFloat32(1.0f);
	SolveBlockLcp(m_auxiliaryRowCount, m_blockSize, u, b, low, high, normalIndex, ndFloat32(0.5f));

	for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
	{
		const ndFloat32 s = u[i];
		f[primaryCount + i] = s;
	}

	const ndInt32 threadCount = threadPool ? threadPool->GetThreadCount() : 1;
	const ndInt32 span = (primaryCount / threadCount + 31) & -32;
	auto SolveAuxiliary_2 = ndMakeObject::ndFunction([this, primaryCount, u, f, span](ndInt32 groupId, ndInt32)
	{
		ndInt32 start = groupId * span;
		ndInt32 size = ((start + span) < primaryCount) ? span : primaryCount - start;

		ndFloat32* const forceAcc = &f[start];
		ndFloat32* const deltaForce = &m_deltaForce[start];
		for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
		{
			const ndFloat32 s = u[i];
			ndScaleAdd(size, forceAcc, &deltaForce[i * primaryCount], s);
		}
	});

	if (threadPool)
	{
		threadPool->ParallelExecute(SolveAuxiliary_2, threadCount, 1);
	}
	else
	{
		SolveAuxiliary_2(0, 0);
	}

	//for (ndInt32 i = 0; i < m_rowCount; ++i)
	//{
	//	ndInt32 index = m_matrixRowsIndex[i];
	//	const ndLeftHandSide* const row = &m_leftHandSide[index];
	//
	//	const ndVector8 jointForce(f[i]);
	//	const ndInt32 m0 = m_pairs[i].m_m0;
	//	const ndInt32 m1 = m_pairs[i].m_m1;
	//	internalForcesArray[m0] = internalForcesArray[m0].MulAdd((ndVector8&)row->m_Jt.m_jacobianM0, jointForce);
	//	internalForcesArray[m1] = internalForcesArray[m1].MulAdd((ndVector8&)row->m_Jt.m_jacobianM1, jointForce);
	//}

	if (threadPool)
	{
		ndLeftHandSide* leftHandSide;
		ndBodyForcePtr bodyForceRemap;
		auto AddForces = ndMakeObject::ndFunction([this, f, internalForcesArray, &leftHandSide, &bodyForceRemap](ndInt32 groupId, ndInt32)
		{
			const ndInt32 start = bodyForceRemap.m_indexSpan[groupId];
			const ndInt32 count = bodyForceRemap.m_indexSpan[groupId + 1] - start;

			const ndInt32 m = bodyForceRemap.m_index[start].m_bodyIndex;
			ndVector8 force(internalForcesArray[m]);
			for (ndInt32 j = 0; j < count; ++j)
			{
				const ndInt32 i = bodyForceRemap.m_index[j + start].m_forceIndex;
				const ndVector8 jointForce(f[i]);
				const ndInt32 index = m_matrixRowsIndex[i];
				const ndLeftHandSide* const row = &leftHandSide[index];
				force = force.MulAdd((ndVector8&)row->m_Jt.m_jacobianM0, jointForce);
			}
			internalForcesArray[m] = force;
		});

		bodyForceRemap = m_bodyForceRemap0;
		leftHandSide = (ndLeftHandSide*)m_leftHandSide;
		threadPool->ParallelExecute(AddForces, bodyForceRemap.m_spansCount, 4);

		bodyForceRemap = m_bodyForceRemap1;
		leftHandSide = (ndLeftHandSide*)(((ndJacobian*)m_leftHandSide) + 1);
		threadPool->ParallelExecute(AddForces, bodyForceRemap.m_spansCount, 4);
	}
	else
	{
		auto AddForces = ndMakeObject::ndFunction([this, f, internalForcesArray](ndInt32 groupId, ndInt32)
		{
			const ndBodyForcePtr bodyForceRemap(groupId ? m_bodyForceRemap1 : m_bodyForceRemap0);
			//const ndLeftHandSide* const leftHandSide = groupId ? (ndLeftHandSide*)&m_leftHandSide->m_Jt.m_jacobianM1 : m_leftHandSide;
			const ndLeftHandSide* const leftHandSide = groupId ? (ndLeftHandSide*) (((ndJacobian*)m_leftHandSide)+1): m_leftHandSide;

			for (ndInt32 k = 0; k < bodyForceRemap.m_spansCount; ++k)
			{
				const ndInt32 start = bodyForceRemap.m_indexSpan[k];
				const ndInt32 count = bodyForceRemap.m_indexSpan[k + 1] - start;

				const ndInt32 m = bodyForceRemap.m_index[start].m_bodyIndex;
				ndVector8 force(internalForcesArray[m]);
				for (ndInt32 j = 0; j < count; ++j)
				{
					const ndInt32 i = bodyForceRemap.m_index[j + start].m_forceIndex;
					const ndVector8 jointForce(f[i]);
					const ndInt32 index = m_matrixRowsIndex[i];
					const ndLeftHandSide* const row = &leftHandSide[index];
					force = force.MulAdd((ndVector8&)row->m_Jt.m_jacobianM0, jointForce);
				}
				internalForcesArray[m] = force;
			}
		});
		AddForces(0, 0);
		AddForces(1, 0);
	}
}

void ndSkeletonContainer::InitMassMatrix(
	ndThreadPool* const threadPool,
	const ndLeftHandSide* const leftHandSide, 
	ndRightHandSide* const rightHandSide)
{
	D_TRACKTIME();
	if (m_isResting)
	{
		return;
	}

	ndInt32 rowCount = 0;
	ndInt32 auxiliaryCount = 0;
	m_leftHandSide = leftHandSide;
	m_rightHandSide = rightHandSide;

	const ndInt32 nodeCount = m_nodeList.GetCount();
	ndSpatialMatrix* const bodyMassArray = ndAlloca(ndSpatialMatrix, nodeCount);
	ndSpatialMatrix* const jointMassArray = ndAlloca(ndSpatialMatrix, nodeCount);
	if (m_nodesOrder)
	{
		if (threadPool)
		{
			ndFixSizeArray<ndInt32, D_MAX_THREADS_COUNT> rows(D_MAX_THREADS_COUNT);
			ndFixSizeArray<ndInt32, D_MAX_THREADS_COUNT> auxiliaries(D_MAX_THREADS_COUNT);

			const ndInt32 threadCount = threadPool->GetThreadCount();
			ndMemSet(&rows[0], ndInt32(0), threadCount);
			ndMemSet(&auxiliaries[0], ndInt32(0), threadCount);

			for (ndInt32 i = 0; i < m_factorizeSpans.GetCount() - 1; ++i)
			{
				const ndInt32 start = ndInt32(m_factorizeSpans[i]);
				ndInt32 count = ndInt32(m_factorizeSpans[i + 1]) - start;

				auto Factorize = ndMakeObject::ndFunction([this, start, bodyMassArray, jointMassArray, leftHandSide, rightHandSide, &rows, &auxiliaries](ndInt32 groupId, ndInt32 threadIndex)
				{
					ndNode* const node = m_nodesFactorizationOrder[start + groupId];
					rows[threadIndex] += node->m_joint->m_rowCount;
					auxiliaries[threadIndex] += node->FactorizeChild(leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
				});

				threadPool->ParallelExecute(Factorize, count);
			}

			for (ndInt32 i = threadCount - 1; i >= 0; --i)
			{
				rowCount += rows[i];
				auxiliaryCount += auxiliaries[i];
			}
		}
		else
		{
			for (ndInt32 i = 0; i < nodeCount - 1; ++i)
			{
				ndNode* const node = m_nodesOrder[i];
				rowCount += node->m_joint->m_rowCount;
				auxiliaryCount += node->FactorizeChild(leftHandSide, rightHandSide, bodyMassArray, jointMassArray);
			}
		}
		m_nodesOrder[nodeCount - 1]->FactorizeRoot(bodyMassArray, jointMassArray);
	}

	m_rowCount = rowCount;
	m_auxiliaryRowCount = auxiliaryCount;

	ndInt32 loopRowCount = 0;
	for (ndInt32 i = ndInt32(m_permanentLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_permanentLoopingJoints[i];
		loopRowCount += joint->m_rowCount;
	}
	for (ndInt32 i = ndInt32(m_transientLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_transientLoopingJoints[i];
		loopRowCount += joint->m_rowCount;
	}
	for (ndInt32 i = ndInt32(m_transientLoopingContacts.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_transientLoopingContacts[i];
		loopRowCount += joint->m_rowCount;
	}

	m_loopRowCount = loopRowCount;

	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;
	if (m_auxiliaryRowCount)
	{
		InitLoopMassMatrix(threadPool);

		const ndInt32 stride = m_auxiliaryRowCount;
		const ndInt32 size = m_auxiliaryRowCount - m_blockSize;
		const ndFloat32* const matrix = &m_massMatrix11[m_blockSize * stride + m_blockSize];

		for (ndInt32 i = 0; i < size; ++i)
		{
			ndFloat32 diagSqrt = ndSqrt(matrix[i * stride + i]);
			m_diagonalPreconditioner[i] = ndFloat32(1.0f) / diagSqrt;
		}

		ndFloat32* const preconditionMatrix = &m_precondinonedMassMatrix11[0];
		auto Precondition = ndMakeObject::ndFunction([this, stride, size, matrix, preconditionMatrix](ndInt32 groupId, ndInt32)
		{
			const ndFloat32* const srcRow = &matrix[groupId * stride];
			ndFloat32* const dstRow = &preconditionMatrix[groupId * stride];
			ndFloat32 diagonal = m_diagonalPreconditioner[groupId];
			for (ndInt32 j = 0; j < size; ++j)
			{
				dstRow[j] = diagonal * srcRow[j] * m_diagonalPreconditioner[j];
			}
		});

		if (threadPool)
		{
			threadPool->ParallelExecute(Precondition, size, 2);
		}
		else
		{
			for (ndInt32 i = 0; i < size; ++i)
			{
				Precondition(i, 0);
			}
		}
	}
}

void ndSkeletonContainer::CalculateReactionForces(ndThreadPool* const threadPool, ndJacobian* const internalForces)
{
	if (m_isResting)
	{
		return;
	}

	D_TRACKTIME();
	const ndInt32 nodeCount = m_nodeList.GetCount();
	ndForcePair* const force = ndAlloca(ndForcePair, nodeCount);
	ndForcePair* const accel = ndAlloca(ndForcePair, nodeCount);

	CalculateJointAccel(threadPool, internalForces, accel);
	CalculateForce(force, accel);
	if (m_auxiliaryRowCount)
	{
		SolveAuxiliary(threadPool, internalForces, accel, force);
	}
	else
	{
		UpdateForces(internalForces, force);
	}
}

void ndSkeletonContainer::CalculatePositionImpulse(ndFloat32 timestep, ndForcePair* const veloc)
{
	const ndSpatialVector zero(ndSpatialVector::m_zero);

	m_auxiliaryJointViolationsRowCount = 0;
	ndFloat32 invTimeStep = ndFloat32(0.9f) / timestep;
	const ndInt32 nodeCount = m_nodeList.GetCount();
	
	auto CalculateJointVeloc = [this, invTimeStep, veloc, &zero](ndInt32 groupId, ndInt32)
	{
		ndNode* const node = m_nodesOrder[groupId];
		ndAssert(node->m_parent);
		ndAssert(groupId == node->m_index);

		ndForcePair& v = veloc[groupId];
		ndAssert(node->m_body);
		v.m_body = zero;
		v.m_joint = zero;

		ndAssert(node->m_joint);
		ndJointBilateralConstraint* const joint = node->m_joint;
		ndAssert(joint->IsBilateral());

		const ndInt32 dof = joint->m_rowCount;
		const ndInt32 first = joint->m_rowStart;

		ndFloat32 maxPenetration2 = ndFloat32(0.0f);
		for (ndInt32 j = 0; j < dof; ++j)
		{
			const ndInt32 k = node->m_ordinal.m_sourceJacobianIndex[j];
			const ndRightHandSide* const rhs = &m_rightHandSide[first + k];
			maxPenetration2 = ndMax(maxPenetration2, rhs->m_errorViolation * rhs->m_errorViolation);
			if (maxPenetration2 > D_MAX_POSIT_ERROR_VIOLATION2)
			{
				maxPenetration2 *= 1;
			}
			ndFloat32 relSpeed = invTimeStep * rhs->m_errorViolation;
			v.m_joint[j] = -relSpeed;
		}

		const ndInt32 m0 = node->m_index;
		const ndInt32 m1 = node->m_parent->m_index;

		for (ndInt32 i = node->m_dof; i < dof; ++i)
		{
			//for (ndInt32 i = 0; i < nodeCount; ++i)
			//{
				//const ndNode* const node = m_nodesOrder[i];
				//ndJointBilateralConstraint* const joint = node->m_joint;
				//const ndInt32 m0 = joint->GetBody0()->m_index;
				//const ndInt32 m1 = joint->GetBody1()->m_index;
				//const ndInt32 primaryDof = node->m_dof;
				//const ndInt32 first = joint->m_rowStart;
				//for (ndInt32 j = 0; j < primaryDof; ++j)
				//{
				//	const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[j];
				//	m_pairs[primaryIndex].m_m0 = m0;
				//	m_pairs[primaryIndex].m_m1 = m1;
				//	m_pairs[primaryIndex].m_joint = joint;
				//	m_frictionIndex[primaryIndex] = 0;
				//	m_matrixRowsIndex[primaryIndex] = first + index;
				//	primaryIndex++;
				//}
				//
				//const ndInt32 auxiliaryDof = joint->m_rowCount - primaryDof;
				//for (ndInt32 j = 0; j < auxiliaryDof; ++j)
				//{
				//	const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[primaryDof + j];
				//	const ndRightHandSide* const rhs = &m_rightHandSide[first + index];
				//
				//	m_pairs[auxiliaryIndex + primaryCount].m_m0 = m0;
				//	m_pairs[auxiliaryIndex + primaryCount].m_m1 = m1;
				//	m_pairs[auxiliaryIndex + primaryCount].m_joint = joint;
				//	m_frictionIndex[auxiliaryIndex + primaryCount] = m_auxiliaryRowCount - auxiliaryIndex;
				//	m_matrixRowsIndex[auxiliaryIndex + primaryCount] = first + index;
				//	const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
				//	ndAssert(joint->IsBilateral());
				//	ndAssert(rhs->SanityCheck());
				//	boundRow[auxiliaryIndex] = boundIndex;
				//	m_blockSize += boundIndex;
				//	auxiliaryIndex++;
				//}
			//}

			ndAssert(0);
			const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[i];
			//const ndRightHandSide* const rhs = &m_rightHandSide[first + index];
			m_pairs[m_auxiliaryJointViolationsRowCount].m_m0 = m0;
			m_pairs[m_auxiliaryJointViolationsRowCount].m_m1 = m1;
			m_pairs[m_auxiliaryJointViolationsRowCount].m_joint = joint;
			m_frictionIndex[m_auxiliaryJointViolationsRowCount] = 0;
			m_matrixRowsIndex[m_auxiliaryJointViolationsRowCount] = first + index;
			//const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
			//boundRow[auxiliaryIndex] = boundIndex;
			//m_blockSize += boundIndex;
			m_auxiliaryJointViolationsRowCount++;
		}

		return maxPenetration2;
	};

	m_maxPositError2 = ndFloat32(0.0f);
	for (ndInt32 index = 0; index < (nodeCount - 1); ++index)
	{
		ndFloat32 penetration2 = CalculateJointVeloc(index, 0);
		m_maxPositError2 = ndMax(m_maxPositError2, penetration2);
	}

	ndAssert((nodeCount - 1) == m_nodesOrder[nodeCount - 1]->m_index);
	veloc[nodeCount - 1].m_body = zero;
	veloc[nodeCount - 1].m_joint = zero;
}

void ndSkeletonContainer::ResolveJointsPostSolverViolations(ndFloat32 timestep)
{
	if (m_isResting)
	{
		return;
	}

	const ndInt32 nodeCount = m_nodeList.GetCount();
	ndForcePair* const jointVeloc = ndAlloca(ndForcePair, nodeCount);
	ndForcePair* const jointImpulse = ndAlloca(ndForcePair, nodeCount);
	ndJacobian* const bodyImpulse = ndAlloca(ndJacobian, nodeCount + 1);

	// apply two impulse passes.
	// the first one to cancel all position violations, 
	// and the second to cancel all velocity violations.
	
	// apply possition violation pass.
	CalculatePositionImpulse(timestep, jointVeloc);
	if (m_maxPositError2 > D_MAX_POSIT_ERROR_VIOLATION2)
	{
		CalculateForce(jointImpulse, jointVeloc);
		if (m_auxiliaryJointViolationsRowCount)
		{
			//SolveAuxiliary(threadPool, internalForces, accel, force);
			CalculateExtraBodyImpulses(bodyImpulse, jointVeloc, jointImpulse);
		}
		else
		{
			CalculateBodyImpulses(bodyImpulse, jointImpulse);
		}

		for (ndInt32 i = 0; i < (nodeCount - 1); ++i)
		{
			ndNode* const node = m_nodesOrder[i];
			ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();;
			ndAssert(body);
			ndAssert(body->GetInvMass() > ndFloat32(0.0f));

			const ndVector savedOmega(body->GetOmega());
			const ndVector savedVeloc(body->GetVelocity());

			const ndJacobian& impulse = bodyImpulse[i];
			const ndVector veloc(impulse.m_linear.Scale(body->GetInvMass()));
			const ndVector omega(body->m_invWorldInertiaMatrix.RotateVector(impulse.m_angular));

			body->SetOmega(omega);
			body->SetVelocity(veloc);

			body->IntegrateVelocity(timestep);

			body->SetOmega(savedOmega);
			body->SetVelocity(savedVeloc);
		}
	}
}