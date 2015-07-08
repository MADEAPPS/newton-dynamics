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


#define DG_SKELETON_STACK_SIZE		512


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////// 
dgInt32 dgSkeletonContainer::m_uniqueID = DG_SKELETON_BASEW_UNIQUE_ID;


class dgSkeletonContainer::dgSkeletonGraph
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	class dgSpacialVector
	{
		public:
		dgVector m_v[2];

		DG_INLINE dgFloat32& operator[] (dgInt32 i)
		{
			dgAssert(i < 8);
			dgAssert(i >= 0);
			dgFloat32* const ptr = &m_v[0].m_x;
			return ptr[i];
		}

		DG_INLINE const dgFloat32& operator[] (dgInt32 i) const
		{
			dgAssert(i < 8);
			dgAssert(i >= 0);
			const dgFloat32* const ptr = &m_v[0].m_x;
			return ptr[i];
		}

		DG_INLINE void SetZero()
		{
			m_v[0] = dgVector::m_zero;
			m_v[1] = dgVector::m_zero;
		}

		DG_INLINE dgFloat32 DotProduct(const dgSpacialVector& v) const
		{
			return (m_v[0].DotProduct4(v.m_v[0]) + m_v[1].DotProduct4(v.m_v[1])).GetScalar();
		}

		DG_INLINE void Scale(const dgVector& s, dgSpacialVector& dst) const
		{
			dst.m_v[0] = m_v[0].CompProduct4(s);
			dst.m_v[1] = m_v[1].CompProduct4(s);
		}

		DG_INLINE void ScaleAdd(const dgVector& s, const dgSpacialVector& b, dgSpacialVector& dst) const
		{
			dst.m_v[0] = b.m_v[0] + m_v[0].CompProduct4(s);
			dst.m_v[1] = b.m_v[1] + m_v[1].CompProduct4(s);
		}
	};

	class dgSpacialMatrix
	{
		public:
		DG_INLINE dgSpacialVector& operator[] (dgInt32 i)
		{
			dgAssert(i < 6);
			dgAssert(i >= 0);
			return m_rows[i];
		}

		DG_INLINE const dgSpacialVector& operator[] (dgInt32 i) const
		{
			dgAssert(i < 6);
			dgAssert(i >= 0);
			return m_rows[i];
		}

		DG_INLINE void SetZero()
		{
			for (dgInt32 i = 0; i < 6; i++) {
				m_rows[i].SetZero();
			}
		}

		DG_INLINE void SetIdentity(dgInt32 rows)
		{
			for (dgInt32 i = 0; i < rows; i++) {
				m_rows[i].m_v[0] = dgVector::m_zero;
				m_rows[i].m_v[1] = dgVector::m_zero;
				m_rows[i][i] = dgFloat32(1.0f);
			}
		}

		DG_INLINE void MultiplyMatrix6x6TimeJacobianTransposed(const dgSpacialVector& jacobian, dgSpacialVector& out) const
		{
			for (dgInt32 i = 0; i < 6; i++) {
				out[i] = m_rows[i].DotProduct(jacobian);
			}
			out[6] = dgFloat32(0.0f);
			out[7] = dgFloat32(0.0f);
		}

		DG_INLINE void MultiplyMatrixNxNTimeJacobianTransposed(const dgSpacialVector& jacobian, dgSpacialVector& out, dgInt32 dof) const
		{
			for (dgInt32 i = 0; i < dof; i++) {
				out[i] = m_rows[i].DotProduct(jacobian);
			}
			for (dgInt32 i = dof; i < 6; i++) {
				out[i] = dgFloat32 (0.0f);
			}
		}

		DG_INLINE void Inverse(const dgSpacialMatrix& src, dgInt32 rows)
		{
			dgSpacialMatrix copy;
			for (dgInt32 i = 0; i < rows; i++) {
				copy[i] = src[i];
			}
			SetIdentity(rows);

			for (dgInt32 i = 0; i < rows; i++) {
				dgFloat32 val = copy.m_rows[i][i];
				dgAssert(dgAbsf(val) > dgFloat32(1.0e-12f));
				dgVector den(dgFloat32(1.0f) / val);

				m_rows[i].Scale(den, m_rows[i]);
				copy[i].Scale(den, copy[i]);

				for (dgInt32 j = 0; j < rows; j++) {
					if (j != i) {
						dgVector pivot(-copy[j][i]);
						m_rows[i].ScaleAdd(pivot, m_rows[j], m_rows[j]);
						copy[i].ScaleAdd(pivot, copy[j], copy[j]);
					}
				}
			}
			dgAssert (CheckPSD(rows));
		}

		DG_INLINE bool CheckPSD(dgInt32 rows)
		{
			return true;
		}

		dgSpacialVector m_rows[6];
	};

	class dgData
	{
		public:
		dgSpacialMatrix m_diagonal;
		dgSpacialMatrix m_invDiagonal;
		dgSpacialMatrix m_offDiagonal;
		dgSpacialVector m_force;
	};


	dgSkeletonGraph(dgMemoryAllocator* const allocator, dgSkeletonGraph* const parent)
		:m_parent(parent)
		,m_children(allocator)
		,m_data(NULL)
		,m_index(0)
	{
		if (m_parent) {
			m_parent->m_children.Append(this);
		}
	}

	~dgSkeletonGraph()
	{
		for (dgList<dgSkeletonGraph*>::dgListNode* ptr = m_children.GetFirst(); ptr; ptr = ptr->GetNext()) {
			delete ptr->GetInfo();
		}
	}

	virtual dgBody* GetBody() const 
	{
		return NULL;
	}

	virtual dgBilateralConstraint* GetJoint() const
	{
		return NULL;
	}

	virtual void SetPriority(dgUnsigned32 priority) const
	{
	}

	virtual void Init (dgData* const buffer, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow) 
	{
		m_data = buffer;
		m_data->m_diagonal.SetZero();
		m_data->m_offDiagonal.SetZero();
	}

	virtual void CalculateDiagonal(dgSkeletonGraph* const child, dgJointInfo* const jointInfoArray)
	{
		dgAssert (0);
	}

	virtual void CalculateDiagonalInverse()
	{
		dgAssert(0);
	}

	virtual void CalculateOffDiagonalBlock()
	{
		dgAssert (0);
	}

	virtual void NegJtTimeSolutionForward (dgSkeletonGraph* const dst)
	{
		dgAssert(0);
	}

	virtual void NegJtTimeSolutionBackward()
	{
		dgAssert(0);
	}

	virtual void DiagInvTimeSolution ()
	{
		dgAssert(0);
	}

	dgSkeletonGraph* m_parent;
	dgList<dgSkeletonGraph*> m_children;
	dgData* m_data;
	dgInt16 m_index;
};


class dgSkeletonContainer::dgSkeletonJointGraph: public dgSkeletonGraph
{
	public:
	dgSkeletonJointGraph(dgMemoryAllocator* const allocator, dgBilateralConstraint* const Joint, dgSkeletonGraph* const parent)
		:dgSkeletonGraph(allocator, parent)
		, m_joint(Joint)
		, m_jacobianDof(0)
	{
	}

	virtual dgBilateralConstraint* GetJoint() const
	{
		return m_joint;
	}

	virtual void SetPriority(dgUnsigned32 priority) const
	{
		m_joint->m_priority = priority;
	}

	virtual void Init(dgData* const buffer, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
	{
		dgAssert(m_parent);

		dgSkeletonGraph::Init(buffer, jointInfoArray, matrixRow);
		dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
		dgAssert(jointInfo->m_joint == m_joint);
		dgAssert(jointInfo->m_joint->GetBody1() == m_parent->GetBody());

		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			dgInt32 index = jointInfo->m_pairStart + i;
			const dgJacobianMatrixElement* const row = &matrixRow[index];
			m_data->m_diagonal[i][i] = row->m_diagDamp;
			for (dgInt32 j = 0; j < 3; j++) {
				m_data->m_offDiagonal[i][j + 0] = row->m_Jt.m_jacobianM1.m_linear[j];
				m_data->m_offDiagonal[i][j + 3] = row->m_Jt.m_jacobianM1.m_angular[j];
			}
		}
	}

	virtual void CalculateDiagonal(dgSkeletonGraph* const child, dgJointInfo* const jointInfoArray)
	{
		dgAssert(child->GetBody());
		dgSpacialMatrix tmp;
		const dgSpacialMatrix& childDiagonal = child->m_data->m_diagonal;
		const dgSpacialMatrix& jacobianTransposed = child->m_data->m_offDiagonal;
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			childDiagonal.MultiplyMatrix6x6TimeJacobianTransposed(jacobianTransposed[i], tmp[i]);
		}

		dgSpacialMatrix& diagonal = m_data->m_diagonal;
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			diagonal[i][i] -= jacobianTransposed[i].DotProduct(tmp[i]);
			//for (dgInt32 j = 0; j < i; j++) {
			for (dgInt32 j = i + 1; j < m_jacobianDof; j++) {
				dgFloat32 a = jacobianTransposed[i].DotProduct(tmp[j]);
				diagonal[i][j] -= a;
				diagonal[j][i] -= a;
			}
		}
		dgAssert (diagonal.CheckPSD(m_jacobianDof));
	}

	virtual void CalculateDiagonalInverse()
	{
		m_data->m_invDiagonal.Inverse(m_data->m_diagonal, m_jacobianDof);
	}

	virtual void CalculateOffDiagonalBlock()
	{
		dgSpacialMatrix copy;
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			copy[i] = m_data->m_offDiagonal[i];
		}
		m_data->m_offDiagonal.SetZero();
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			const dgSpacialVector& jacobian = copy[i];
			const dgSpacialVector& invDiagonalRow = m_data->m_invDiagonal[i];
			for (dgInt32 j = 0; j < m_jacobianDof; j++) {
				dgVector val(invDiagonalRow[j]);
				jacobian.ScaleAdd(val, m_data->m_offDiagonal[j], m_data->m_offDiagonal[j]);
			}
		}
	}

	virtual void NegJtTimeSolutionForward(dgSkeletonGraph* const dst)
	{
		dgAssert(dst->GetBody());

		const dgInt32 dof = m_jacobianDof;
		const dgSpacialVector& force = m_data->m_force;
		const dgSpacialMatrix& Jt = m_data->m_offDiagonal;
		dgSpacialVector& outForce = dst->m_data->m_force;
		for (dgInt32 i = 0; i < dof; i++) {
			Jt[i].ScaleAdd (dgVector (-force[i]), outForce, outForce);
		}
	}

	virtual void NegJtTimeSolutionBackward()
	{
		const dgInt32 dof = m_jacobianDof;
		const dgSpacialVector& force = m_parent->m_data->m_force;
		const dgSpacialMatrix& Jt = m_data->m_offDiagonal;
		dgSpacialVector& outForce = m_data->m_force;
		for (dgInt32 i = 0; i < dof; i++) {
			outForce[i] -= force.DotProduct(Jt[i]);
		}
	}


	virtual void DiagInvTimeSolution()
	{
		dgSpacialVector tmp(m_data->m_force);
		for (dgInt32 i = m_jacobianDof; i < 6; i ++) {
			tmp[i] = dgFloat32 (0.0f);
		}
		m_data->m_invDiagonal.MultiplyMatrixNxNTimeJacobianTransposed(tmp, m_data->m_force, m_jacobianDof);
	}


	dgBilateralConstraint* m_joint;
	dgInt16 m_jacobianDof;
};


class dgSkeletonContainer::dgSkeletonBodyGraph: public dgSkeletonGraph
{
	public:
	dgSkeletonBodyGraph (dgMemoryAllocator* const allocator, dgDynamicBody* const child, dgSkeletonGraph* const parent)
		:dgSkeletonGraph(allocator, parent)
		,m_body(child)
	{
	}

	virtual dgBody* GetBody() const
	{
		return m_body;
	}

	virtual void Init (dgData* const buffer, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
	{
		dgSkeletonGraph::Init(buffer, jointInfoArray, matrixRow);

		dgFloat32 mass = m_body->GetMass().m_w;
		dgAssert (mass < dgFloat32 (1.0e10f));
		dgMatrix inertia(m_body->CalculateInertiaMatrix());

		for (dgInt32 i = 0; i < 3; i++) {
			m_data->m_diagonal[i][i] = mass;
			for (dgInt32 j = 0; j < 3; j++) {
				m_data->m_diagonal[i + 3][j + 3] = inertia[i][j];
			}
		}
		dgAssert (m_data->m_diagonal.CheckPSD(6));

		if (m_parent) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_parent;
			dgJointInfo* const jointInfo = &jointInfoArray[jointNode->m_joint->m_index];
			dgAssert(jointNode->GetJoint());
			dgAssert(jointInfo->m_joint == jointNode->GetJoint());
			dgAssert(jointInfo->m_joint->GetBody0() == m_body);
			dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_parent->GetBody());

			const dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			dgInt32 dofCount = 0;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dofCount += ((row->m_lowerBoundFrictionCoefficent < dgFloat32(-1.0e10f)) && (row->m_upperBoundFrictionCoefficent > dgFloat32(1.0e10f))) ? 1 : 0;
			}
			jointNode->m_jacobianDof = dgInt16 (dofCount);

			for (dgInt32 i = 0; i < dofCount; i++) {
				dgInt32 index = jointInfo->m_pairStart + i;
				for (dgInt32 j = 0; j < 3; j++) {
					m_data->m_offDiagonal[i][j + 0] = matrixRow[index].m_Jt.m_jacobianM0.m_linear[j];
					m_data->m_offDiagonal[i][j + 3] = matrixRow[index].m_Jt.m_jacobianM0.m_angular[j];
				}
			}
		}
	}

	virtual void CalculateDiagonalInverse()
	{
		m_data->m_invDiagonal.Inverse(m_data->m_diagonal, 6);
	}


	virtual void CalculateDiagonal(dgSkeletonGraph* const childNode, dgJointInfo* const jointInfoArray)
	{
		dgSpacialMatrix tmp;
		dgSkeletonJointGraph* const child = (dgSkeletonJointGraph*) childNode;
		dgAssert (child->GetJoint());
		const dgSpacialMatrix& childDiagonal = child->m_data->m_diagonal;

		dgSpacialMatrix copy;
		copy.SetZero();

		const dgSpacialMatrix& jacobianMatrix = child->m_data->m_offDiagonal;
		for (dgInt32 i = 0; i < child->m_jacobianDof; i++) {
			const dgSpacialVector& jacobian = jacobianMatrix[i];
			for (dgInt32 j = 0; j < child->m_jacobianDof; j++) {
				dgAssert(dgAbsf(childDiagonal[i][j] - childDiagonal[j][i]) < dgFloat32(1.0e-5f));
				dgVector val(childDiagonal[i][j]);
				jacobian.ScaleAdd(val, copy[j], copy[j]);
			}
		}

		dgSpacialMatrix& diagonal = m_data->m_diagonal;
		for (dgInt32 i = 0; i < child->m_jacobianDof; i++) {
			const dgSpacialVector& Jacobian = copy[i];
			const dgSpacialVector& JacobianTranspose = jacobianMatrix[i];
			for (dgInt32 j = 0; j < 6; j++) {
				dgFloat32 val(-Jacobian[j]);
				JacobianTranspose.ScaleAdd(val, diagonal[j], diagonal[j]);
			}
		}
		dgAssert (diagonal.CheckPSD(6));
	}

	virtual void CalculateOffDiagonalBlock()
	{
		for (dgInt32 i = 0; i < 6; i++) {
			dgSpacialVector	tmp (m_data->m_offDiagonal[i]);
			m_data->m_invDiagonal.MultiplyMatrix6x6TimeJacobianTransposed(tmp, m_data->m_offDiagonal[i]);
		}
	}

	virtual void NegJtTimeSolutionForward(dgSkeletonGraph* const dst)
	{
		dgAssert (dst->GetJoint());
		dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*) dst;

		const dgInt32 dof = jointNode->m_jacobianDof;
		const dgSpacialVector& force = m_data->m_force;
		const dgSpacialMatrix& Jt = m_data->m_offDiagonal;
		dgSpacialVector& outForce = jointNode->m_data->m_force;
		for (dgInt32 i = 0; i < dof; i ++) {
			outForce[i] -= Jt[i].DotProduct(force);
		}
	}

	virtual void NegJtTimeSolutionBackward()
	{
		dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_parent;
		dgAssert (jointNode->GetJoint());

		const dgInt32 dof = jointNode->m_jacobianDof;
		const dgSpacialVector& force = jointNode->m_data->m_force;
		const dgSpacialMatrix& Jt = m_data->m_offDiagonal;
		dgSpacialVector& outForce = m_data->m_force;
		for (dgInt32 i = 0; i < dof; i++) {
			Jt[i].ScaleAdd(dgVector(-force[i]), outForce, outForce);
		}
	}

	virtual void DiagInvTimeSolution()
	{
		dgSpacialVector tmp (m_data->m_force);
		m_data->m_invDiagonal.MultiplyMatrix6x6TimeJacobianTransposed (tmp, m_data->m_force);
	}

	dgDynamicBody* m_body;
};

dgSkeletonContainer::dgSkeletonContainer(dgWorld* const world, dgDynamicBody* const rootBody)
	:m_world(world)
	,m_skeleton(new (rootBody->GetWorld()->GetAllocator()) dgSkeletonBodyGraph(rootBody->GetWorld()->GetAllocator(), rootBody, NULL))
	,m_nodesOrder(NULL)
	,m_id(m_uniqueID)
	,m_nodeCount(1)
{
	m_uniqueID++;
}

dgSkeletonContainer::~dgSkeletonContainer()
{
	dgMemoryAllocator* const allocator = m_skeleton->m_body->GetWorld()->GetAllocator();
	if (m_nodesOrder) {
		allocator->Free(m_nodesOrder);
	}
	delete m_skeleton;
}

void dgSkeletonContainer::ResetUniqueId(dgInt32 id)
{
	m_uniqueID = id;
}

dgInt32 dgSkeletonContainer::GetBufferSize() const
{
	dgInt32 blocksize = sizeof(dgSkeletonGraph::dgData);
	return blocksize * m_nodeCount;
}

void dgSkeletonContainer::SortGraph(dgSkeletonGraph* const root, dgSkeletonGraph* const parent, dgInt32& index)
{
	for (dgList<dgSkeletonGraph*>::dgListNode* node = root->m_children.GetFirst(); node; node = node->GetNext()) {
		SortGraph(node->GetInfo(), root, index);
	}

	root->SetPriority((m_id << DG_SKELETON_BIT_SHIFT_KEY) + index);
	dgAssert((m_nodeCount - index - 1) >= 0);
	m_nodesOrder[index] = root;
	root->m_index = dgInt16(index);
	index++;
	dgAssert(index <= m_nodeCount);
}

dgSkeletonContainer::dgSkeletonGraph* dgSkeletonContainer::FindNode(dgDynamicBody* const body) const
{
	dgInt32 stack = 1;
	dgSkeletonGraph* stackPool[DG_SKELETON_STACK_SIZE];

	stackPool[0] = m_skeleton;
	while (stack) {
		stack--;
		dgSkeletonGraph* const node = stackPool[stack];
		if (node->GetBody() == body) {
			return node;
		}

		for (dgList<dgSkeletonGraph*>::dgListNode* ptr = node->m_children.GetFirst(); ptr; ptr = ptr->GetNext()) {
			stackPool[stack] = ptr->GetInfo();
			stack++;
			dgAssert(stack < dgInt32(sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return NULL;
}


void dgSkeletonContainer::AddChild(dgBody* const child, dgBody* const parent)
{
	dgAssert(child);
	dgBody* const parentBody = parent ? parent : m_skeleton->GetBody();
	dgAssert(parentBody);
	dgAssert(child->GetType() == dgBody::m_dynamicBody);
	dgAssert(parentBody->GetType() == dgBody::m_dynamicBody);
	AddChild((dgDynamicBody*)child, (dgDynamicBody*)parentBody);
}

void dgSkeletonContainer::AddChild(dgDynamicBody* const child, dgDynamicBody* const parent)
{
	dgSkeletonGraph* const parentNode = FindNode(parent);
	dgAssert(parentNode);
	dgAssert (m_skeleton->GetBody());
	dgWorld* const world = m_skeleton->GetBody()->GetWorld();
	dgMemoryAllocator* const allocator = world->GetAllocator();
	dgBilateralConstraint* const joint = world->FindBilateralJoint(child, parent);
	dgAssert(joint);

	dgAssert(joint->GetBody0() == child);
	dgAssert(joint->GetBody1() == parent);
	dgSkeletonJointGraph* const jointParent = new (allocator)dgSkeletonJointGraph(allocator, joint, parentNode);
	new (allocator)dgSkeletonBodyGraph(allocator, child, jointParent);
	m_nodeCount += 2;
}


void dgSkeletonContainer::Finalize()
{
	dgAssert(m_nodeCount >= 1);

	dgMemoryAllocator* const allocator = m_skeleton->m_body->GetWorld()->GetAllocator();
	m_nodesOrder = (dgSkeletonGraph**)allocator->Malloc(2 * m_nodeCount * sizeof (dgSkeletonGraph*));

	dgInt32 index = 0;
	SortGraph(m_skeleton, NULL, index);
	dgAssert(index == m_nodeCount);
}


void dgSkeletonContainer::InitMassMatrix (char* const buffer, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
{
	dgSkeletonGraph::dgData* const ptr = (dgSkeletonGraph::dgData*) buffer;
	dgAssert( (dgUnsigned64(buffer) & 0x0f) == 0);
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_nodesOrder[i];
		node->Init(&ptr[i], jointInfoArray, matrixRow);
	}

	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_nodesOrder[i];
		for (dgList<dgSkeletonGraph*>::dgListNode* child = node->m_children.GetFirst(); child; child = child->GetNext()) {
			node->CalculateDiagonal(child->GetInfo(), jointInfoArray);
		}

		node->CalculateDiagonalInverse();
		if (node->m_parent) {
			node->CalculateOffDiagonalBlock();
		}
	}
}


dgFloat32 dgSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgInt32 index = 0;
	dgFloat32 retAccel = dgFloat32(0.0f);
	const dgWorldDynamicUpdate& dynamicsUpdate = *m_world;
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		if (m_nodesOrder[i]->GetJoint()) {
			dgSkeletonJointGraph* const skeletonNode = (dgSkeletonJointGraph*)m_nodesOrder[i];
			const dgJointInfo* const jointInfo = &jointInfoArray[index];
			index ++;
			dgAssert(jointInfo->m_joint == skeletonNode->m_joint);
			const dgInt32 count = skeletonNode->m_jacobianDof;
			if (count < jointInfo->m_pairCount) {
				dgJointInfo info(*jointInfo);
				info.m_pairStart += count;
				info.m_pairCount = jointInfo->m_pairCount - dgInt16(count);
				dgFloat32 accel = dynamicsUpdate.CalculateJointForce(&info, bodyArray, internalForces, matrixRow);
				retAccel = (accel > retAccel) ? accel : retAccel;
			}
		}
	}

	index = 0;
	dgFloat32 accNorm = dgFloat32(0.0f);
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		if (m_nodesOrder[i]->GetJoint()) {
			dgSkeletonJointGraph* const skeletonNode = (dgSkeletonJointGraph*)m_nodesOrder[i];
			const dgJointInfo* const jointInfo = &jointInfoArray[index];
			index++;
			dgAssert(jointInfo->m_joint == skeletonNode->m_joint);
			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 count = skeletonNode->m_jacobianDof;
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
			const dgJacobian& y0 = internalForces[m0];
			const dgJacobian& y1 = internalForces[m1];
			
			dgSkeletonGraph::dgSpacialVector& accel = skeletonNode->m_data->m_force;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
								row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
				acc = dgVector(row->m_coordenateAccel) - acc.AddHorizontal();
				accel[j] = - acc.GetScalar();
				accNorm += acc.Abs().GetScalar();
			}
		} else {
			m_nodesOrder[i]->m_data->m_force.SetZero();
		}
	}
	retAccel = dgMax (accNorm, retAccel);


	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_nodesOrder[i];
		for (dgList<dgSkeletonGraph*>::dgListNode* childNode = node->m_children.GetFirst(); childNode; childNode = childNode->GetNext()) {
			dgSkeletonGraph* const child = childNode->GetInfo();
			child->NegJtTimeSolutionForward(node);
		}
	}

	for (dgInt32 i = m_nodeCount - 1; i >= 0; i--) {
		dgSkeletonGraph* const node = m_nodesOrder[i];
		node->DiagInvTimeSolution();
		if (node->m_parent) {
			node->NegJtTimeSolutionBackward();
		}
	}

	index = 0;
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		if (m_nodesOrder[i]->GetJoint()) {
			dgSkeletonJointGraph* const skeletonNode = (dgSkeletonJointGraph*)m_nodesOrder[i];
			const dgJointInfo* const jointInfo = &jointInfoArray[index];
			index++;

			dgJacobian y0;
			dgJacobian y1;
			y0.m_linear = dgVector::m_zero;
			y0.m_angular = dgVector::m_zero;
			y1.m_linear = dgVector::m_zero;
			y1.m_angular = dgVector::m_zero;

			dgAssert(jointInfo->m_joint == skeletonNode->m_joint);
			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 count = skeletonNode->m_jacobianDof;
			dgSkeletonGraph::dgSpacialVector& force = skeletonNode->m_data->m_force;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgVector val(force[j]);
				dgAssert(dgCheckFloat(force[j]));
				row->m_force += force[j];
				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(val);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(val);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(val);
			}
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
			
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		}
	}

	return retAccel;
}
