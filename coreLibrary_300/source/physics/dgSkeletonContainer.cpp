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
dgInt32 dgSkeletonContainer::m_uniqueID = 10;

class dgSkeletonContainer::dgSolverData
{
	public: 
	dgJacobian m_intemidiateForce;
};

class dgSkeletonContainer::dgSolverJointData
{
	struct dgInfo
	{
		dgFloat32 m_force;
		dgFloat32 m_deltaForce;
		dgFloat32 m_accel;
		dgFloat32 m_deltaAccel;
	};
	public: 
	dgInfo m_data[6];
};

class dgSkeletonContainer::dgSkeletonGraph
{
	public:
	DG_CLASS_ALLOCATOR(allocator)
/*
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

		DG_INLINE dgFloat32 DotProduct (const dgSpacialVector& v) const
		{
			return (m_v[0].DotProduct4(v.m_v[0]) + m_v[1].DotProduct4(v.m_v[1])).GetScalar();
		}

		DG_INLINE void Scale (const dgVector& s, dgSpacialVector& dst) const
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

		DG_INLINE void SetZero ()
		{
			memset (m_rows, 0, sizeof(m_rows));
		}

		DG_INLINE void SetIdentity (dgInt32 rows)
		{
			for (dgInt32 i = 0; i < rows; i ++) {
				m_rows[i].m_v[0] = dgVector::m_zero;
				m_rows[i].m_v[1] = dgVector::m_zero;
				m_rows[i][i] = dgFloat32 (1.0f);
			}
		}

		DG_INLINE void Inverse (const dgSpacialMatrix& src, dgInt32 rows)
		{
			dgSpacialMatrix copy;
			for (dgInt32 i = 0; i < rows; i ++) {
				copy[i] = src[i];
			}
			SetIdentity(rows);

			for (dgInt32 i = 0; i < rows; i++) {
				dgFloat32 val = copy.m_rows[i][i];
				dgAssert(dgAbsf(val) > dgFloat32(1.0e-12f));
				dgVector den(dgFloat32(1.0f) / val);

				m_rows[i].Scale(den, m_rows[i]);
				copy[i].Scale (den, copy[i]);

				for (dgInt32 j = 0; j < rows; j++) {
					if (j != i) {
						dgVector pivot(-copy[j][i]);
						m_rows[i].ScaleAdd (pivot, m_rows[j], m_rows[j]);
						copy[i].ScaleAdd (pivot, copy[j], copy[j]);
					}
				}
			}
		}

		DG_INLINE void MultiplyMatrix6x6TimeJacobianTransposed (dgSpacialVector& jacobian) const
		{
			dgSpacialVector tmp (jacobian);
			dgAssert (tmp[6] == dgFloat32 (0.0f));
			dgAssert (tmp[7] == dgFloat32 (0.0f));
			for (dgInt32 i = 0; i < 6; i ++) {
				jacobian[i] = m_rows[i].DotProduct(tmp);
			}
		}

		DG_INLINE void Multiply (const dgSpacialMatrix& B, dgSpacialMatrix& dst) const
		{
			const dgSpacialMatrix& A = *this;
			for (dgInt32 i = 0; i < 6; i ++) {
				for (dgInt32 j = 0; j < 6; j ++) {
					dgFloat32 acc = dgFloat32 (0.0f);
					for (dgInt32 k = 0; k < 6; k ++) {
						dgFloat32 a = A[i][k];
						dgFloat32 b = B[k][j];
						acc += a * b;
					}
					dst[i][j] = acc;
				}
			}
		}

		void Trace (dgInt32 n) const
		{
			dgTrace(("\n"));
			for (dgInt32 i = 0; i < n; i ++) {
				for (dgInt32 j = 0; j < 6; j ++) {
					dgTrace(("%f, ", m_rows[i][j]));
				}
				dgTrace(("\n"));
			}
		}

		dgSpacialVector m_rows[6];
	};

	class dgData
	{
		public:
		dgSpacialMatrix m_diagonal;
		dgSpacialMatrix m_invDiagonal;
		dgSpacialMatrix m_offDiagonal;
	};
*/

	dgSkeletonGraph(dgMemoryAllocator* const allocator, dgDynamicBody* const root)
		:m_parent(NULL)
		,m_body(root)
		,m_joint(NULL)
		,m_data(NULL)
		,m_children(allocator)
		,m_m0(0)
		,m_index(0)
		,m_bodyM0(0)
		,m_bodyM1(0)
		,m_jointDOF(0)
		//,m_diaginalDof(0)
		//,m_jacobialDof(0)
	{
	}

	dgSkeletonGraph(dgMemoryAllocator* const allocator, dgDynamicBody* const child, dgSkeletonGraph* const parent)
		:m_parent(parent)
		,m_body(child)
		,m_joint(NULL)
		,m_data(NULL)
		,m_children(allocator)
		,m_m0(0)
		,m_index(0)
		,m_bodyM0(0)
		,m_bodyM1(0)
		,m_jointDOF(0)
		//,m_diaginalDof(0)
		//,m_jacobialDof(0)
	{
		m_parent->m_children.Append(this);
	}

	dgSkeletonGraph(dgMemoryAllocator* const allocator, dgBilateralConstraint* const Joint, dgSkeletonGraph* const parent)
		:m_parent(parent)
		,m_body(NULL)
		,m_joint(Joint)
		,m_data(NULL)
		,m_children(allocator)
		,m_m0(0)
		,m_index(0)
		,m_bodyM0(0)
		,m_bodyM1(0)
		,m_jointDOF(0)
		//,m_diaginalDof(0)
		//,m_jacobialDof(0)
	{
		m_parent->m_children.Append(this);
	}


	~dgSkeletonGraph()
	{
		for (dgList<dgSkeletonGraph*>::dgListNode* ptr = m_children.GetFirst(); ptr; ptr = ptr->GetNext()) {
			delete ptr->GetInfo();
		}
	}

	virtual void SetPriority(dgUnsigned32 priority) const
	{
		if (m_joint) {
			m_joint->m_priority = priority;
		}
	}

/*
	void Init (dgData* const buffer, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow) 
	{
		m_data = buffer;

		m_data->m_diagonal.SetZero();
		m_data->m_offDiagonal.SetZero();
		if (m_body) {
			m_diaginalDof = 6;
			dgFloat32 mass = m_body->GetMass().m_w;
			dgMatrix inertia (m_body->CalculateInertiaMatrix());
			
			for (dgInt32 i = 0; i < 3; i++) {
				m_data->m_diagonal[i][i] = mass;
				for (dgInt32 j = 0; j < 3; j++) {
					m_data->m_diagonal[i + 3][j + 3] = inertia[i][j];
				}
			}

			if (m_parent) {
				dgBilateralConstraint* const joint = m_parent->m_joint;
				dgAssert (joint);
				dgAssert (joint->GetBody0() == m_body);
				dgAssert (joint->GetBody1() == m_parent->m_parent->m_body);
				dgJointInfo* const jointInfo = &jointInfoArray[joint->m_index];
				dgAssert (jointInfo->m_joint == joint);
				m_jacobialDof = jointInfo->m_pairCount;

				for (dgInt32 i = 0; i < m_jacobialDof; i ++) {
					dgInt32 index = jointInfo->m_pairStart + i;
					for (dgInt32 j = 0; j < 3; j ++) {
						m_data->m_offDiagonal[i][j + 0] = matrixRow[index].m_JMinv.m_jacobianM0.m_linear[j];
						m_data->m_offDiagonal[i][j + 3] = matrixRow[index].m_JMinv.m_jacobianM0.m_angular[j];
					}
				}
			}
		} else {
			dgJointInfo* const jointInfo = &jointInfoArray[m_joint->m_index];

			dgAssert (m_parent);
			dgAssert (jointInfo->m_joint == m_joint);
			dgAssert (jointInfo->m_joint->GetBody1() == m_parent->m_body);
			m_diaginalDof = jointInfo->m_pairCount;
			m_jacobialDof = jointInfo->m_pairCount;
		
			for (dgInt32 i = 0; i < m_diaginalDof; i++) {
				dgInt32 index = jointInfo->m_pairStart + i;
				for (dgInt32 j = 0; j < 3; j ++) {
					m_data->m_offDiagonal[i][j + 0] = matrixRow[index].m_JMinv.m_jacobianM1.m_linear[j];
					m_data->m_offDiagonal[i][j + 3] = matrixRow[index].m_JMinv.m_jacobianM1.m_angular[j];
				}
			}
		}
	}

	void CalculateOffDiagonalBlock ()
	{
		if (m_body) {
			for (dgInt32 i = 0; i < m_jacobialDof; i ++) {
				m_data->m_invDiagonal.MultiplyMatrix6x6TimeJacobianTransposed (m_data->m_offDiagonal[i]);
			}
		} else {
			dgSpacialMatrix copy;
			for (dgInt32 i = 0; i < m_jacobialDof; i ++) {
				copy[i] = m_data->m_offDiagonal[i];
			}
			m_data->m_offDiagonal.SetZero();
			for (dgInt32 i = 0; i < m_jacobialDof; i ++) {
				const dgSpacialVector& jacobian = copy[i];
				const dgSpacialVector& invDiagonalRow = m_data->m_invDiagonal[i];
				for (dgInt32 j = 0; j < m_jacobialDof; j ++) {
					dgVector val (invDiagonalRow[j]);
					jacobian.ScaleAdd(val, m_data->m_offDiagonal[j], m_data->m_offDiagonal[j]);
				}
			}
		}
	}

	virtual void CalculateDiagonal(dgSkeletonGraph* const child, dgJointInfo* const jointInfoArray)
	{
		dgSpacialMatrix tmp;
		const dgSpacialMatrix& childDiagonal = child->m_data->m_diagonal;
		if (m_body) {
			dgSpacialMatrix copy;
			copy.SetZero();

			const dgSpacialMatrix& jacobianMatrix = child->m_data->m_offDiagonal;
			for (dgInt32 i = 0; i < child->m_jacobialDof; i++) {
				const dgSpacialVector& jacobian = jacobianMatrix[i];
				for (dgInt32 j = 0; j < child->m_jacobialDof; j++) {
					dgAssert (dgAbsf (childDiagonal[i][j] - childDiagonal[j][i]) < dgFloat32 (1.0e-5f));
					dgVector val(childDiagonal[i][j]);
					jacobian.ScaleAdd (val, copy[j], copy[j]);
				}
			}

//childDiagonal.Trace(child->m_jacobialDof);
//copy.Trace (child->m_jacobialDof);
//jacobianMatrix.Trace (child->m_jacobialDof);

			dgAssert (m_diaginalDof == 6);
			dgSpacialMatrix& diagonal = m_data->m_diagonal;
			for (dgInt32 i = 0; i < child->m_jacobialDof; i++) {
				const dgSpacialVector& Jacobian = copy[i];
				const dgSpacialVector& JacobianTranspose = jacobianMatrix[i];
				for (dgInt32 j = 0; j < 6; j ++) {
					dgFloat32 val (-Jacobian[j]);
					JacobianTranspose.ScaleAdd (val, diagonal[j], diagonal[j]);
				}
			}
//diagonal.Trace(6);

			#ifdef _DEBUG
 			for (dgInt32 i = 0; i < 6; i++) {
				for (dgInt32 k = 0; k < i; k++) {
					dgFloat32 a = diagonal[i][k];
					dgFloat32 b = diagonal[k][i];
					dgAssert(dgAbsf(a - b) < dgFloat32(1.0e-5f));
				}
			}
			#endif
		} else {
			dgAssert (child->m_body);
			const dgSpacialMatrix& jacobianTransposed = child->m_data->m_offDiagonal;
			for (dgInt32 i = 0; i < m_jacobialDof; i++) {
				tmp[i] = jacobianTransposed[i];
				childDiagonal.MultiplyMatrix6x6TimeJacobianTransposed(tmp[i]);
			}

			dgSpacialMatrix& diagonal = m_data->m_diagonal;
			for (dgInt32 i = 0; i < m_jacobialDof; i++) {
				diagonal[i][i] -= jacobianTransposed[i].DotProduct(tmp[i]);
				for (dgInt32 j = 0; j < i; j ++) {
					dgFloat32 a = jacobianTransposed[i].DotProduct(tmp[j]);
					diagonal[i][j] -= a;
					diagonal[j][i] -= a;
				}
			}
		}
	}

	virtual void CalculateDiagonalInverse ()
	{
		m_data->m_invDiagonal.Inverse (m_data->m_diagonal, m_diaginalDof);
	}
*/
	dgSkeletonGraph* m_parent;
	dgDynamicBody* m_body;
	dgBilateralConstraint* m_joint;
	//dgData* m_data;
	dgSolverJointData* m_data;
	dgList<dgSkeletonGraph*> m_children;
	dgInt32 m_m0;
	dgInt16 m_index;
	dgInt16 m_bodyM0;
	dgInt16 m_bodyM1;
	dgInt16 m_jointDOF; 
//	dgInt16 m_diaginalDof;
//	dgInt16 m_jacobialDof;
};


dgSkeletonContainer::dgSkeletonContainer (dgWorld* const world, dgDynamicBody* const rootBody)
	:m_world(world)
	,m_solverData(NULL)
	,m_skeleton(new (rootBody->GetWorld()->GetAllocator()) dgSkeletonGraph (rootBody->GetWorld()->GetAllocator(), rootBody))
	,m_jointArray(NULL)
	,m_bottomTopOrder(NULL)
//	,m_topBottomOrder(NULL)
	,m_id(m_uniqueID)
	,m_nodeCount(1)
{
	m_uniqueID ++;
}

dgSkeletonContainer::~dgSkeletonContainer ()
{
	dgMemoryAllocator* const allocator = m_skeleton->m_body->GetWorld()->GetAllocator();
	if (m_jointArray) {
		allocator->Free(m_jointArray);
	}
	delete m_skeleton;
}

void dgSkeletonContainer::ResetUniqueId(dgInt32 id)
{
	m_uniqueID = 10;
}

dgSkeletonContainer::dgSkeletonGraph* dgSkeletonContainer::FindNode (dgDynamicBody* const body) const
{
	dgInt32 stack = 1;
	dgSkeletonGraph* stackPool[DG_SKELETON_STACK_SIZE];

	stackPool[0] = m_skeleton;
	while (stack) {
		stack --;
		dgSkeletonGraph* const node = stackPool[stack];
		if (node->m_body == body) {
			return node;
		}

		for (dgList<dgSkeletonGraph*>::dgListNode* ptr = node->m_children.GetFirst(); ptr; ptr = ptr->GetNext()) {
			stackPool[stack] = ptr->GetInfo();
			stack ++;
			dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
		}
	}
	return NULL;
}

void dgSkeletonContainer::AddChild (dgBody* const child, dgBody* const parent)
{
	dgAssert (child);
	dgBody* const parent1 = parent ? parent : m_skeleton->m_body;
	dgAssert (child->GetType() == dgBody::m_dynamicBody);
	dgAssert (parent1->GetType() == dgBody::m_dynamicBody);
	AddChild ((dgDynamicBody*) child, (dgDynamicBody*) parent1);
}

void dgSkeletonContainer::AddChild (dgDynamicBody* const child, dgDynamicBody* const parent)
{
	dgSkeletonGraph* const parentNode = FindNode (parent);
	dgAssert (parentNode);
	dgWorld* const world = m_skeleton->m_body->GetWorld();
	dgMemoryAllocator* const allocator = world->GetAllocator();
	dgBilateralConstraint* const joint = world->FindBilateralJoint (child, parent);
	dgAssert (joint);

	dgAssert (joint->GetBody0() == child);
	dgAssert (joint->GetBody1() == parent);
	dgSkeletonGraph* const massParent = new (allocator) dgSkeletonGraph (allocator, joint, parentNode);
	new (allocator) dgSkeletonGraph (allocator, child, massParent);
	m_nodeCount += 2;
}


dgInt32 dgSkeletonContainer::GetBufferSize () const
{
//	dgInt32 blocksize = sizeof(dgSkeletonGraph::dgData);
//	return blocksize * m_nodeCount;
	dgInt32 bodyCount = (m_nodeCount + 1) / 2;
	dgInt32 jointCount = (m_nodeCount - 1) / 2;
	dgInt32 bodysize = sizeof(dgSolverData);
	dgInt32 jointsize = sizeof(dgSolverJointData);
	return bodysize * bodyCount + jointCount * jointsize;
}

void dgSkeletonContainer::SortGraph (dgSkeletonGraph* const root, dgSkeletonGraph* const parent, dgInt32& index)
{
	for (dgList<dgSkeletonGraph*>::dgListNode* node = root->m_children.GetFirst(); node; node = node->GetNext()) {
		SortGraph (node->GetInfo(), root, index);
	}

	root->SetPriority((m_id << DG_SKELETON_BIT_SHIFT_KEY) + index);
	dgAssert ((m_nodeCount - index - 1) >= 0);
	//m_topBottomOrder[m_nodeCount - index - 1] = root;
	m_bottomTopOrder[index] = root;
	root->m_index = dgInt16 (index);
	index ++;
	dgAssert (index <= m_nodeCount);
}

void dgSkeletonContainer::Finalize ()
{
	dgAssert (m_nodeCount >= 1);

	dgInt32 jointCount = (m_nodeCount - 1) / 2;
	dgMemoryAllocator* const allocator = m_skeleton->m_body->GetWorld()->GetAllocator();
	m_jointArray = (dgSkeletonGraph**) allocator->Malloc ((2 * m_nodeCount + jointCount) * sizeof (void*));
	m_bodyArray = &m_jointArray[jointCount];
	m_bottomTopOrder = &m_bodyArray[(m_nodeCount + 1) / 2];
//	m_topBottomOrder = &m_bottomTopOrder[m_nodeCount];
	
	dgInt32 index = 0;
	SortGraph (m_skeleton, NULL, index);
	dgAssert (index == m_nodeCount);

	dgInt32 bodyIndex = 0;
	dgInt32 jointIndex = 0;
	for (dgInt32 i = 0; i < m_nodeCount; i ++) {
		dgSkeletonGraph* const node = m_bottomTopOrder[i];
		if (node->m_body) {
			m_bodyArray[bodyIndex] = node;
			bodyIndex ++;
			if (node->m_parent) {
				dgSkeletonGraph* const jointNode = node->m_parent;
				dgAssert(jointNode->m_joint->GetBody0() == node->m_body);
				dgAssert(jointNode->m_joint->GetBody1() == jointNode->m_parent->m_body);
				jointNode->m_bodyM0 = (node->m_index + 1) / 2;
				jointNode->m_bodyM1 = (jointNode->m_parent->m_index + 1) / 2;
			}
		} else {
			m_jointArray[jointIndex] = node;
			jointIndex++;
		}
	}
}




bool dgSkeletonContainer::Sanity() const
{
/*
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_bottomTopOrder[i];
		dgInt32 n = node->m_diaginalDof;

		dgSkeletonGraph::dgSpacialMatrix identity;
		node->m_data->m_diagonal.Multiply(node->m_data->m_invDiagonal, identity);

		for (dgInt32 j = 0; j < n; j++) {
			for (dgInt32 k = 0; k < n; k++) {
				if (dgAbsf (node->m_data->m_diagonal[j][k] - node->m_data->m_diagonal[k][j]) > dgFloat32 (1.0e-5f)) {
					return false;
				}
				dgTrace (("%f, ", node->m_data->m_diagonal[j][k]));
			}

			dgTrace (("    "));
			for (dgInt32 k = 0; k < n; k++) {
				dgTrace (("%f, ", node->m_data->m_invDiagonal[j][k]));
			}

			dgTrace (("    "));
			for (dgInt32 k = 0; k < n; k++) {
				dgTrace(("%f, ", identity[j][k]));
			}
			dgTrace (("\n"));

		}
		dgTrace (("\n"));
	}
*/
	return true;
}


void dgSkeletonContainer::InitMassMatrix (char* const buffer, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow)
{

/*
	dgSkeletonGraph::dgData* const ptr = (dgSkeletonGraph::dgData*) buffer;
	dgAssert( (dgUnsigned64(buffer) & 0x0f) == 0);
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_bottomTopOrder[i];
		node->Init(&ptr[i], jointInfoArray, matrixRow);
	}

	dgAssert (Sanity ());
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_bottomTopOrder[i];
		for (dgList<dgSkeletonGraph*>::dgListNode* child = node->m_children.GetFirst(); child; child = child->GetNext()) {
			node->CalculateDiagonal(child->GetInfo(), jointInfoArray);
			dgAssert (Sanity ());
		}
		node->CalculateDiagonalInverse();
		dgAssert (Sanity ());
		if (node->m_parent) {
			node->CalculateOffDiagonalBlock();
			dgAssert (Sanity ());
		}
	}
*/

	dgSolverData* const ptr = (dgSolverData*) buffer;
	dgAssert((dgUnsigned64(buffer) & 0x0f) == 0);
	m_solverData = ptr;

	const dgInt32 bodyCount = (m_nodeCount + 1) / 2;
	dgSolverJointData* jointData = (dgSolverJointData*) (&m_solverData[bodyCount]);

	dgInt32 bodyIndex = 0;
	dgInt32 jointIndex = 0;
	for (dgInt32 i = 0; i < m_nodeCount; i++) {
		dgSkeletonGraph* const node = m_bottomTopOrder[i];
		if (node->m_body) {
			if (node->m_parent) {
				const dgSkeletonGraph* const jointNode = node->m_parent;
				const dgJointInfo* const jointInfo = &jointInfoArray[bodyIndex];
				dgAssert(jointInfo->m_joint == m_jointArray[bodyIndex]->m_joint);
				dgAssert(jointNode->m_joint->GetBody0() == node->m_body);
				dgAssert(jointNode->m_joint->GetBody1() == jointNode->m_parent->m_body);
				node->m_m0 = jointInfo->m_m0;
				jointNode->m_parent->m_m0 = jointInfo->m_m1;
				bodyIndex ++;
			}
		} else {
			node->m_data = jointData;
			const dgJointInfo* const jointInfo = &jointInfoArray[jointIndex];
			dgAssert(jointInfo->m_joint == node->m_joint);
			jointIndex ++;
			const dgInt32 count = jointInfo->m_pairCount;
			const dgInt32 first = jointInfo->m_pairStart;
			dgInt32 dofCount = 0;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dofCount += ((row->m_lowerBoundFrictionCoefficent < dgFloat32 (-1.0e10f)) && (row->m_upperBoundFrictionCoefficent > dgFloat32 (1.0e10f))) ? 1 : 0;
			}
			node->m_jointDOF = dgInt16 (dofCount);
			jointData ++;
		}
	}
}



dgFloat32 dgSkeletonContainer::CalculateJointForce(dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgJacobian* const scratchData = &m_solverData->m_intemidiateForce;
	const dgInt32 jointCount = (m_nodeCount - 1) / 2;

	const dgWorldDynamicUpdate& dynamicsUpdate = *m_world;

	dgFloat32 retAccel = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < jointCount; i++) {
		const dgJointInfo* const jointInfo = &jointInfoArray[i];
		dgSkeletonGraph* const skeletonNode = m_jointArray[i];
		dgAssert(jointInfo->m_joint == skeletonNode->m_joint);
		const dgInt32 count = skeletonNode->m_jointDOF;
		if (count < jointInfo->m_pairCount) {
			dgJointInfo info (jointInfoArray[i]);
			info.m_pairStart += count;
			info.m_pairCount = jointInfo->m_pairCount - dgInt16 (count);
			dgFloat32 accel = dynamicsUpdate.CalculateJointForce(&info, bodyArray, internalForces, matrixRow);
			retAccel = (accel > retAccel) ? accel : retAccel;
		}
	}

	const dgInt32 bodyCount = (m_nodeCount + 1) / 2;
	for (dgInt32 i = 0; i < bodyCount; i++) {
		const dgSkeletonGraph* const skeletonNode = m_bodyArray[i];
		scratchData[i] = internalForces[skeletonNode->m_m0];
	}

	

	dgFloat32 accNorm = dgFloat32(0.0f);
	dgFloat64 akNum = dgFloat32(0.0f);
	for (dgInt32 i = 0; i < jointCount; i++) {
		const dgJointInfo* const jointInfo = &jointInfoArray[i];
		dgSkeletonGraph* const skeletonNode = m_jointArray[i];
		dgAssert(jointInfo->m_joint == skeletonNode->m_joint);
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = skeletonNode->m_jointDOF;
		const dgInt32 m0 = skeletonNode->m_bodyM0;
		const dgInt32 m1 = skeletonNode->m_bodyM1;
		const dgJacobian& y0 = scratchData[m0];
		const dgJacobian& y1 = scratchData[m1];

		dgSolverJointData* const data = skeletonNode->m_data;
		for (dgInt32 j = 0; j < count; j++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
						 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));
			acc = dgVector(row->m_coordenateAccel) - acc.AddHorizontal();
			data->m_data[j].m_force = dgFloat32(0.0f); 
			data->m_data[j].m_accel = acc.GetScalar();
			data->m_data[j].m_deltaForce = data->m_data[j].m_accel * row->m_invDJMinvJt;
			akNum += data->m_data[j].m_accel * data->m_data[j].m_deltaForce;
			accNorm += acc.Abs().GetScalar();
		}
	}

	retAccel = dgMax (accNorm, retAccel);;
	const dgFloat32 maxAccel = DG_SOLVER_MAX_ERROR;
	if (accNorm > maxAccel) {
		const dgInt32 maxPasses = 32;
		for (dgInt32 passes = 0; (passes < maxPasses) && (accNorm > maxAccel); passes++) {
			for (dgInt32 i = 0; i < bodyCount; i++) {
				scratchData[i].m_linear = dgVector::m_zero;
				scratchData[i].m_angular = dgVector::m_zero;
			}

			for (dgInt32 i = 0; i < jointCount; i++) {
				dgJacobian y0;
				dgJacobian y1;
				y0.m_linear = dgVector::m_zero;
				y0.m_angular = dgVector::m_zero;
				y1.m_linear = dgVector::m_zero;
				y1.m_angular = dgVector::m_zero;

				const dgJointInfo* const jointInfo = &jointInfoArray[i];
				dgSkeletonGraph* const skeletonNode = m_jointArray[i];
				const dgInt32 first = jointInfo->m_pairStart;
				const dgInt32 count = skeletonNode->m_jointDOF;
				dgSolverJointData* const data = skeletonNode->m_data;

				for (dgInt32 j = 0; j < count; j++) {
					dgJacobianMatrixElement* const row = &matrixRow[j + first];
					dgVector val(data->m_data[j].m_deltaForce);
					dgAssert(dgCheckFloat(data->m_data[j].m_deltaForce));
					y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
					y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(val);
					y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(val);
					y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(val);
				}

				const dgInt32 m0 = skeletonNode->m_bodyM0;
				const dgInt32 m1 = skeletonNode->m_bodyM1;
				scratchData[m0].m_linear += y0.m_linear;
				scratchData[m0].m_angular += y0.m_angular;
				scratchData[m1].m_linear += y1.m_linear;
				scratchData[m1].m_angular += y1.m_angular;
			}

			dgFloat64 akDen = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < jointCount; i++) {
				const dgJointInfo* const jointInfo = &jointInfoArray[i];
				dgSkeletonGraph* const skeletonNode = m_jointArray[i];
				const dgInt32 first = jointInfo->m_pairStart;
				const dgInt32 count = skeletonNode->m_jointDOF;
				const dgInt32 m0 = skeletonNode->m_bodyM0;
				const dgInt32 m1 = skeletonNode->m_bodyM1;

				const dgJacobian& y0 = scratchData[m0];
				const dgJacobian& y1 = scratchData[m1];
				dgSolverJointData* const data = skeletonNode->m_data;
				for (dgInt32 j = 0; j < count; j++) {
					dgJacobianMatrixElement* const row = &matrixRow[j + first];
					dgVector acc(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(y0.m_angular) +
								 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(y1.m_angular));

					data->m_data[j].m_deltaAccel = acc.AddHorizontal().GetScalar();
					akDen += data->m_data[j].m_deltaAccel * data->m_data[j].m_deltaForce;
				}
			}

			dgFloat32 ak = dgFloat32(akNum / akDen);
			dgVector accelMag(dgVector::m_zero);
			for (dgInt32 i = 0; i < jointCount; i++) {
				//dgJointInfo* const jointInfo = &jointInfoArray[i];
				dgSkeletonGraph* const skeletonNode = m_jointArray[i];
				const dgInt32 count = skeletonNode->m_jointDOF;
				dgSolverJointData* const data = skeletonNode->m_data;
				for (dgInt32 j = 0; j < count; j++) {
					data->m_data[j].m_force += ak * data->m_data[j].m_deltaForce;
					data->m_data[j].m_accel -= ak * data->m_data[j].m_deltaAccel;
					accelMag += dgVector(data->m_data[j].m_accel).Abs();
				}
			}

			accNorm = accelMag.GetScalar();
			if (accNorm > maxAccel) {
				akDen = akNum;
				akNum = dgFloat32(0.0f);
				for (dgInt32 i = 0; i < jointCount; i++) {
					const dgJointInfo* const jointInfo = &jointInfoArray[i];
					dgSkeletonGraph* const skeletonNode = m_jointArray[i];
					const dgInt32 first = jointInfo->m_pairStart;
					const dgInt32 count = skeletonNode->m_jointDOF;
					dgSolverJointData* const data = skeletonNode->m_data;
					for (dgInt32 j = 0; j < count; j++) {
						dgJacobianMatrixElement* const row = &matrixRow[j + first];
						data->m_data[j].m_deltaAccel = data->m_data[j].m_accel * row->m_invDJMinvJt;;
						akNum += data->m_data[j].m_accel * data->m_data[j].m_deltaAccel;
					}
				}

				ak = dgFloat32 (akNum / akDen);
				for (dgInt32 i = 0; i < jointCount; i++) {
					//const dgJointInfo* const jointInfo = &jointInfoArray[i];
					dgSkeletonGraph* const skeletonNode = m_jointArray[i];
					const dgInt32 count = skeletonNode->m_jointDOF;
					dgSolverJointData* const data = skeletonNode->m_data;
					for (dgInt32 j = 0; j < count; j++) {
						data->m_data[j].m_deltaForce = data->m_data[j].m_deltaAccel + ak * data->m_data[j].m_deltaForce;
					}
				}
			}
		}

		for (dgInt32 i = 0; i < bodyCount; i++) {
			scratchData[i].m_linear = dgVector::m_zero;
			scratchData[i].m_angular = dgVector::m_zero;
		}

		for (dgInt32 i = 0; i < jointCount; i++) {
			dgJacobian y0;
			dgJacobian y1;
			y0.m_linear = dgVector::m_zero;
			y0.m_angular = dgVector::m_zero;
			y1.m_linear = dgVector::m_zero;
			y1.m_angular = dgVector::m_zero;

			const dgJointInfo* const jointInfo = &jointInfoArray[i];
			dgSkeletonGraph* const skeletonNode = m_jointArray[i];
			dgAssert(jointInfo->m_joint == m_jointArray[i]->m_joint);
			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 count = skeletonNode->m_jointDOF;
			dgSolverJointData* const data = skeletonNode->m_data;
			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgVector val(data->m_data[j].m_force);
				dgAssert(dgCheckFloat(data->m_data[j].m_force));
				row->m_force += data->m_data[j].m_force;
				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(val);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(val);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(val);
			}
			const dgInt32 m0 = skeletonNode->m_bodyM0;
			const dgInt32 m1 = skeletonNode->m_bodyM1;
			scratchData[m0].m_linear += y0.m_linear;
			scratchData[m0].m_angular += y0.m_angular;
			scratchData[m1].m_linear += y1.m_linear;
			scratchData[m1].m_angular += y1.m_angular;
		}

		for (dgInt32 i = 0; i < bodyCount; i++) {
			const dgSkeletonGraph* const skeletonNode = m_bodyArray[i];
			internalForces[skeletonNode->m_m0].m_linear += scratchData[i].m_linear;
			internalForces[skeletonNode->m_m0].m_angular += scratchData[i].m_angular;
		}
	}

	return retAccel;
}


