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

		dgVector m_v[2];
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

	enum dgType 
	{
		m_isBody,
		m_isJoint
	};

	dgSkeletonGraph (dgSkeletonGraph* const parent)
		:m_parent(parent)
		,m_child(NULL)
		,m_sibling(NULL)
		,m_index(0)
	{
		if (m_parent) {
			if (m_parent->m_child) {
				m_sibling = m_parent->m_child;
			}
			m_parent->m_child = this;
		}
	}

	DG_INLINE virtual ~dgSkeletonGraph()
	{
		dgSkeletonGraph* next;
		for (dgSkeletonGraph* ptr = m_child; ptr; ptr = next) {
			next = ptr->m_sibling;
			delete ptr;
		}
	}

	DG_INLINE dgType GetType () const 
	{
		return m_type;
	}

	DG_INLINE virtual dgBody* GetBody() const 
	{
		return NULL;
	}

	DG_INLINE virtual dgBilateralConstraint* GetJoint() const
	{
		return NULL;
	}

	DG_INLINE virtual void SetPriority(dgUnsigned32 priority) const
	{
	}

	dgData m_data;
	dgSkeletonGraph* m_parent;
	dgSkeletonGraph* m_child;
	dgSkeletonGraph* m_sibling;
	dgInt16 m_index;
	dgType m_type;
};


class dgSkeletonContainer::dgSkeletonJointGraph: public dgSkeletonGraph
{
	public:
	dgSkeletonJointGraph(dgBilateralConstraint* const Joint, dgSkeletonGraph* const parent)
		:dgSkeletonGraph(parent)
		, m_joint(Joint)
		, m_jacobianDof(0)
	{
		m_type = m_isJoint;
	}

	DG_INLINE virtual dgBilateralConstraint* GetJoint() const
	{
		return m_joint;
	}

	DG_INLINE virtual void SetPriority(dgUnsigned32 priority) const
	{
		m_joint->m_priority = priority;
	}

	DG_INLINE void Init(dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
	{
		dgAssert(m_parent);

		m_data.m_diagonal.SetZero();
		m_data.m_offDiagonal.SetZero();

		dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];
		dgAssert(jointInfo->m_joint == m_joint);
		dgAssert(jointInfo->m_joint->GetBody1() == m_parent->GetBody());

		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			dgInt32 index = jointInfo->m_pairStart + i;
			const dgJacobianMatrixElement* const row = &matrixRow[index];
			m_data.m_diagonal[i][i] = row->m_diagDamp;
			for (dgInt32 j = 0; j < 3; j++) {
				m_data.m_offDiagonal[i][j + 0] = row->m_Jt.m_jacobianM1.m_linear[j];
				m_data.m_offDiagonal[i][j + 3] = row->m_Jt.m_jacobianM1.m_angular[j];
			}
		}
	}

	DG_INLINE void CalculateDiagonal(dgSkeletonGraph* const child, dgJointInfo* const jointInfoArray)
	{
		dgAssert(child->GetBody());
		dgSpacialMatrix tmp;
		const dgSpacialMatrix& childDiagonal = child->m_data.m_diagonal;
		const dgSpacialMatrix& jacobianTransposed = child->m_data.m_offDiagonal;
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			childDiagonal.MultiplyMatrix6x6TimeJacobianTransposed(jacobianTransposed[i], tmp[i]);
		}

		dgSpacialMatrix& diagonal = m_data.m_diagonal;
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

	DG_INLINE void CalculateDiagonalInverse()
	{
		m_data.m_invDiagonal.Inverse(m_data.m_diagonal, m_jacobianDof);
	}

	DG_INLINE void CalculateOffDiagonalBlock()
	{
		dgSpacialMatrix copy;
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			copy[i] = m_data.m_offDiagonal[i];
		}
		m_data.m_offDiagonal.SetZero();
		for (dgInt32 i = 0; i < m_jacobianDof; i++) {
			const dgSpacialVector& jacobian = copy[i];
			const dgSpacialVector& invDiagonalRow = m_data.m_invDiagonal[i];
			for (dgInt32 j = 0; j < m_jacobianDof; j++) {
				dgVector val(invDiagonalRow[j]);
				jacobian.ScaleAdd(val, m_data.m_offDiagonal[j], m_data.m_offDiagonal[j]);
			}
		}
	}

	DG_INLINE void NegJtTimeSolutionForward(dgSkeletonGraph* const dst)
	{
		dgAssert(dst->GetBody());

		const dgInt32 dof = m_jacobianDof;
		const dgSpacialVector& force = m_data.m_force;
		const dgSpacialMatrix& Jt = m_data.m_offDiagonal;
		dgSpacialVector& outForce = dst->m_data.m_force;
		for (dgInt32 i = 0; i < dof; i++) {
			Jt[i].ScaleAdd (dgVector (-force[i]), outForce, outForce);
		}
	}

	DG_INLINE void NegJtTimeSolutionBackward()
	{
		const dgInt32 dof = m_jacobianDof;
		const dgSpacialVector& force = m_parent->m_data.m_force;
		const dgSpacialMatrix& Jt = m_data.m_offDiagonal;
		dgSpacialVector& outForce = m_data.m_force;
		for (dgInt32 i = 0; i < dof; i++) {
			outForce[i] -= force.DotProduct(Jt[i]);
		}
	}

	DG_INLINE void DiagInvTimeSolution()
	{
		dgSpacialVector tmp(m_data.m_force);
		for (dgInt32 i = m_jacobianDof; i < 6; i ++) {
			tmp[i] = dgFloat32 (0.0f);
		}
		m_data.m_invDiagonal.MultiplyMatrixNxNTimeJacobianTransposed(tmp, m_data.m_force, m_jacobianDof);
	}

	dgBilateralConstraint* m_joint;
	dgInt16 m_jacobianDof;
};


class dgSkeletonContainer::dgSkeletonBodyGraph: public dgSkeletonGraph
{
	public:
	dgSkeletonBodyGraph (dgDynamicBody* const child, dgSkeletonGraph* const parent)
		:dgSkeletonGraph(parent)
		,m_body(child)
	{
		m_type = m_isBody;
	}

	virtual dgBody* GetBody() const
	{
		return m_body;
	}

	DG_INLINE void Init (dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
	{
		m_data.m_diagonal.SetZero();
		m_data.m_offDiagonal.SetZero();

		dgFloat32 mass = m_body->GetMass().m_w;
		dgAssert (mass < dgFloat32 (1.0e10f));
		dgMatrix inertia(m_body->CalculateInertiaMatrix());

		for (dgInt32 i = 0; i < 3; i++) {
			m_data.m_diagonal[i][i] = mass;
			for (dgInt32 j = 0; j < 3; j++) {
				m_data.m_diagonal[i + 3][j + 3] = inertia[i][j];
			}
		}
		dgAssert (m_data.m_diagonal.CheckPSD(6));

		if (m_parent) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_parent;
			dgJointInfo* const jointInfo = &jointInfoArray[jointNode->m_joint->m_index];
			dgAssert(jointNode->GetJoint());
			dgAssert(jointInfo->m_joint == jointNode->GetJoint());
			dgAssert(jointInfo->m_joint->GetBody0() == m_body);
			dgAssert(jointInfo->m_joint->GetBody1() == m_parent->m_parent->GetBody());

			dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgInt32 index = ((row->m_lowerBoundFrictionCoefficent < dgFloat32(-1.0e10f)) && (row->m_upperBoundFrictionCoefficent > dgFloat32(1.0e10f))) ? 0 : 1;
				if (index) {
					count --;
					if (j != count) {
						dgSwap(matrixRow[j + first], matrixRow[count + first]);
					}
					j --;
				}
			}
			jointNode->m_jacobianDof = dgInt16 (count);
			for (dgInt32 i = 0; i < count; i++) {
				dgInt32 index = jointInfo->m_pairStart + i;
				for (dgInt32 j = 0; j < 3; j++) {
					m_data.m_offDiagonal[i][j + 0] = matrixRow[index].m_Jt.m_jacobianM0.m_linear[j];
					m_data.m_offDiagonal[i][j + 3] = matrixRow[index].m_Jt.m_jacobianM0.m_angular[j];
				}
			}
		}
	}

	DG_INLINE void CalculateDiagonalInverse()
	{
		m_data.m_invDiagonal.Inverse(m_data.m_diagonal, 6);
	}


	virtual void CalculateDiagonal(dgSkeletonGraph* const childNode, dgJointInfo* const jointInfoArray)
	{
		dgSpacialMatrix tmp;
		dgSkeletonJointGraph* const child = (dgSkeletonJointGraph*) childNode;
		dgAssert (child->GetJoint());
		const dgSpacialMatrix& childDiagonal = child->m_data.m_diagonal;

		dgSpacialMatrix copy;
		copy.SetZero();

		const dgSpacialMatrix& jacobianMatrix = child->m_data.m_offDiagonal;
		for (dgInt32 i = 0; i < child->m_jacobianDof; i++) {
			const dgSpacialVector& jacobian = jacobianMatrix[i];
			for (dgInt32 j = 0; j < child->m_jacobianDof; j++) {
				dgAssert(dgAbsf(childDiagonal[i][j] - childDiagonal[j][i]) < dgFloat32(1.0e-5f));
				dgVector val(childDiagonal[i][j]);
				jacobian.ScaleAdd(val, copy[j], copy[j]);
			}
		}

		dgSpacialMatrix& diagonal = m_data.m_diagonal;
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

	DG_INLINE void CalculateOffDiagonalBlock()
	{
		for (dgInt32 i = 0; i < 6; i++) {
			dgSpacialVector	tmp (m_data.m_offDiagonal[i]);
			m_data.m_invDiagonal.MultiplyMatrix6x6TimeJacobianTransposed(tmp, m_data.m_offDiagonal[i]);
		}
	}

	DG_INLINE void NegJtTimeSolutionForward(dgSkeletonGraph* const dst)
	{
		dgAssert (dst->GetJoint());
		dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*) dst;

		const dgInt32 dof = jointNode->m_jacobianDof;
		const dgSpacialVector& force = m_data.m_force;
		const dgSpacialMatrix& Jt = m_data.m_offDiagonal;
		dgSpacialVector& outForce = jointNode->m_data.m_force;
		for (dgInt32 i = 0; i < dof; i ++) {
			outForce[i] -= Jt[i].DotProduct(force);
		}
	}

	DG_INLINE void NegJtTimeSolutionBackward()
	{
		dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_parent;
		dgAssert (jointNode->GetJoint());

		const dgInt32 dof = jointNode->m_jacobianDof;
		const dgSpacialVector& force = jointNode->m_data.m_force;
		const dgSpacialMatrix& Jt = m_data.m_offDiagonal;
		dgSpacialVector& outForce = m_data.m_force;
		for (dgInt32 i = 0; i < dof; i++) {
			Jt[i].ScaleAdd(dgVector(-force[i]), outForce, outForce);
		}
	}

	DG_INLINE void DiagInvTimeSolution()
	{
		dgSpacialVector tmp (m_data.m_force);
		m_data.m_invDiagonal.MultiplyMatrix6x6TimeJacobianTransposed (tmp, m_data.m_force);
	}

	dgDynamicBody* m_body;
};

dgSkeletonContainer::dgSkeletonContainer(dgWorld* const world, dgDynamicBody* const rootBody)
	:m_world(world)
	,m_skeleton(new (rootBody->GetWorld()->GetAllocator()) dgSkeletonBodyGraph(rootBody, NULL))
	,m_nodesOrder(NULL)
	,m_destructor(NULL)
	,m_id(m_uniqueID)
	,m_nodeCount(1)
{
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

void dgSkeletonContainer::SortGraph(dgSkeletonGraph* const root, dgSkeletonGraph* const parent, dgInt32& index)
{
	for (dgSkeletonGraph* node = root->m_child; node; node = node->m_sibling) {
		SortGraph(node, root, index);
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

		for (dgSkeletonGraph* ptr = node->m_child; ptr; ptr = ptr->m_sibling) {
			stackPool[stack] = ptr;
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
	dgAssert (m_skeleton->GetBody());
	dgWorld* const world = m_skeleton->GetBody()->GetWorld();
	dgMemoryAllocator* const allocator = world->GetAllocator();
	dgBilateralConstraint* const joint = world->FindBilateralJoint(child, parent);
	dgAssert(joint);

	if ((joint->GetBody0() == child) && (joint->GetBody1() == parent)) {
		dgSkeletonGraph* const parentNode = FindNode(parent);
		dgAssert(parentNode);
		dgSkeletonJointGraph* const jointNode = new (allocator)dgSkeletonJointGraph(joint, parentNode);
		new (allocator)dgSkeletonBodyGraph(child, jointNode);
	} else {
		dgAssert(joint->GetBody1() == child);
		dgAssert(joint->GetBody0() == parent);
		dgAssert (m_skeleton->m_body == parent);
		dgSkeletonBodyGraph* const newRoot = new (allocator) dgSkeletonBodyGraph (child, NULL);
		dgSkeletonJointGraph* const jointNode = new (allocator)dgSkeletonJointGraph(joint, newRoot);

		m_skeleton->m_parent = jointNode;
		jointNode->m_child = m_skeleton;
		m_skeleton = newRoot;
	}
	m_nodeCount += 2;
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
	m_nodesOrder = (dgSkeletonGraph**)allocator->Malloc(2 * m_nodeCount * sizeof (dgSkeletonGraph*));

	dgInt32 index = 0;
	SortGraph(m_skeleton, NULL, index);
	dgAssert(index == m_nodeCount);
}


void dgSkeletonContainer::InitMassMatrix (dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
{
	for (dgInt32 i = 0; i < m_nodeCount; i += 2) {
		dgSkeletonBodyGraph* const bodyNode = (dgSkeletonBodyGraph*) m_nodesOrder[i];
		dgAssert (bodyNode->GetBody());
		dgAssert( (dgUnsigned64(&bodyNode->m_data) & 0x0f) == 0);
		bodyNode->Init(jointInfoArray, matrixRow);

		if (bodyNode->m_parent) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_nodesOrder[i + 1];
			dgAssert(jointNode->GetJoint());
			dgAssert((dgUnsigned64(&jointNode->m_data) & 0x0f) == 0);
			jointNode->Init(jointInfoArray, matrixRow);
		}
	}

	for (dgInt32 i = 0; i < m_nodeCount; i += 2) {
		dgSkeletonBodyGraph* const bodyNode = (dgSkeletonBodyGraph*) m_nodesOrder[i];
		dgAssert (bodyNode->GetBody());
		for (dgSkeletonGraph* child = bodyNode->m_child; child; child = child->m_sibling) {
			bodyNode->CalculateDiagonal(child, jointInfoArray);
		}
		bodyNode->CalculateDiagonalInverse();
		if (bodyNode->m_parent) {
			bodyNode->CalculateOffDiagonalBlock();
		}

		if (bodyNode->m_parent) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_nodesOrder[i + 1];
			dgAssert(jointNode->GetJoint());
			for (dgSkeletonGraph* child = jointNode->m_child; child; child = child->m_sibling) {
				jointNode->CalculateDiagonal(child, jointInfoArray);
			}

			jointNode->CalculateDiagonalInverse();
			if (jointNode->m_parent) {
				jointNode->CalculateOffDiagonalBlock();
			}
		}
	}
}


dgFloat32 dgSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgFloat32 retAccel = dgFloat32(0.0f);
	const dgWorldDynamicUpdate& dynamicsUpdate = *m_world;
	for (dgInt32 i = 1; i < m_nodeCount; i += 2) {
		dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_nodesOrder[i];
		dgAssert(jointNode->GetJoint());
		const dgJointInfo* const jointInfo = &jointInfoArray[i>>1];
		dgAssert(jointInfo->m_joint == jointNode->m_joint);
		const dgInt32 count = jointNode->m_jacobianDof;
		if (count < jointInfo->m_pairCount) {
			dgJointInfo info(*jointInfo);
			info.m_pairStart += count;
			info.m_pairCount = jointInfo->m_pairCount - dgInt16(count);
			dgFloat32 accel = dynamicsUpdate.CalculateJointForce(&info, bodyArray, internalForces, matrixRow);
			retAccel = (accel > retAccel) ? accel : retAccel;
		}
	}


	dgFloat32 accNorm = dgFloat32(0.0f);
	for (dgInt32 i = 0; i < m_nodeCount; i += 2 ) {
		dgSkeletonBodyGraph* const bodyNode = (dgSkeletonBodyGraph*) m_nodesOrder[i];
		dgAssert (bodyNode->GetBody());
		bodyNode->m_data.m_force.SetZero();
		if (bodyNode->m_parent) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_nodesOrder[i + 1];
			const dgJointInfo* const jointInfo = &jointInfoArray[i>>1];
			dgAssert(jointInfo->m_joint == jointNode->m_joint);
			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 count = jointNode->m_jacobianDof;
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
			const dgJacobian& y0 = internalForces[m0];
			const dgJacobian& y1 = internalForces[m1];
			
			dgSkeletonGraph::dgSpacialVector& accel = jointNode->m_data.m_force;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
							 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
				acc = dgVector(row->m_coordenateAccel) - acc.AddHorizontal();
				accel[j] = - acc.GetScalar();
				accNorm += acc.Abs().GetScalar();
			}
		}
	}
	retAccel = dgMax (accNorm, retAccel);

	for (dgInt32 i = 0; i < m_nodeCount; i += 2 ) {
		dgSkeletonBodyGraph* const bodyNode = (dgSkeletonBodyGraph*)m_nodesOrder[i];
		dgAssert(bodyNode->GetBody());
		for (dgSkeletonGraph* childNode = bodyNode->m_child; childNode; childNode = childNode->m_sibling) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)childNode;
			dgAssert(jointNode->GetJoint());
			jointNode->NegJtTimeSolutionForward(bodyNode);
		}
		if (bodyNode->m_parent) {
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_nodesOrder[i + 1];
			dgAssert(jointNode->GetJoint());
			for (dgSkeletonGraph* childNode = jointNode->m_child; childNode; childNode = childNode->m_sibling) {
				dgSkeletonBodyGraph* const bodyNode = (dgSkeletonBodyGraph*)childNode;
				dgAssert(bodyNode->GetBody());
				bodyNode->NegJtTimeSolutionForward(jointNode);
			}
		}
	}

	for (dgInt32 i = m_nodeCount - 1; i >= 0; i -- ) {
		if (m_nodesOrder[i]->GetType() == dgSkeletonGraph::m_isBody) {
			dgSkeletonBodyGraph* const bodyNode = (dgSkeletonBodyGraph*) m_nodesOrder[i];
			bodyNode->DiagInvTimeSolution();
			if (bodyNode->m_parent) {
				bodyNode->NegJtTimeSolutionBackward();
			}
		} else {
			dgAssert (m_nodesOrder[i]->GetType() == dgSkeletonGraph::m_isJoint);
			dgAssert (m_nodesOrder[i]->m_parent);
			dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*) m_nodesOrder[i];
			jointNode->DiagInvTimeSolution();
			jointNode->NegJtTimeSolutionBackward();
		}
	}


	for (dgInt32 i = 1; i < m_nodeCount; i += 2) {
		dgSkeletonJointGraph* const jointNode = (dgSkeletonJointGraph*)m_nodesOrder[i];
		dgAssert(jointNode->GetJoint());
		const dgJointInfo* const jointInfo = &jointInfoArray[i>>1];

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = dgVector::m_zero;
		y0.m_angular = dgVector::m_zero;
		y1.m_linear = dgVector::m_zero;
		y1.m_angular = dgVector::m_zero;

		dgAssert(jointInfo->m_joint == jointNode->m_joint);
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = jointNode->m_jacobianDof;
		dgSkeletonGraph::dgSpacialVector& force = jointNode->m_data.m_force;
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

	return retAccel;
}
