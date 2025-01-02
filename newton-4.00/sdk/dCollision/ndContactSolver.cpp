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
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndShape.h"
#include "ndContact.h"
#include "ndShapePoint.h"
#include "ndShapeConvex.h"
#include "ndShapeCompound.h"
#include "ndBodyKinematic.h"
#include "ndContactSolver.h"
#include "ndPolygonMeshDesc.h"
#include "ndShapeStatic_bvh.h"
#include "ndShapeStaticMesh.h"
#include "ndShapeHeightfield.h"
#include "ndShapeConvexPolygon.h"
#include "ndShapeStaticProceduralMesh.h"

ndVector ndContactSolver::m_pruneUpDir(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
ndVector ndContactSolver::m_pruneSupportX(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));

ndVector ndContactSolver::m_hullDirs[] =
{
	ndVector(ndFloat32(0.577350f), ndFloat32(-0.577350f), ndFloat32(0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-0.577350f), ndFloat32(-0.577350f), ndFloat32(-0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.577350f), ndFloat32(-0.577350f), ndFloat32(-0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-0.577350f), ndFloat32(0.577350f), ndFloat32(0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.577350f), ndFloat32(0.577350f), ndFloat32(-0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-0.577350f), ndFloat32(0.577350f), ndFloat32(-0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-0.577350f), ndFloat32(-0.577350f), ndFloat32(0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.577350f), ndFloat32(0.577350f), ndFloat32(0.577350f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.000000f), ndFloat32(-1.000000f), ndFloat32(0.000000f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.000000f), ndFloat32(1.000000f), ndFloat32(0.000000f), ndFloat32(0.0f)),
	ndVector(ndFloat32(1.000000f), ndFloat32(0.000000f), ndFloat32(0.000000f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.000000f), ndFloat32(0.000000f), ndFloat32(0.000000f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.000000f), ndFloat32(0.000000f), ndFloat32(1.000000f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.000000f), ndFloat32(0.000000f), ndFloat32(-1.000000f), ndFloat32(0.0f)),
};

ndInt32 ndContactSolver::m_rayCastSimplex[4][4] =
{
	{ 0, 1, 2, 3 },
	{ 0, 2, 3, 1 },
	{ 2, 1, 3, 0 },
	{ 1, 0, 3, 2 },
};

D_MSV_NEWTON_ALIGN_32
class ndContactSolver::ndBoxBoxDistance2
{
	public:
	ndBoxBoxDistance2(const ndMatrix& matrix0, const ndMatrix& matrix1)
		:m_matrix0(matrix0)
		,m_matrix1(matrix1)
	{
		m_localMatrix0 = m_matrix1 * m_matrix0.OrthoInverse();
		m_localMatrix1 = m_localMatrix0.OrthoInverse();

		ndInt32 index = 0;
		for (ndInt32 i = 0; i < 3; ++i)
		{
			m_matrixAbs0[i] = m_matrix0[i].Abs();
			m_matrixAbs1[i] = m_matrix1[i].Abs();
			m_localMatrixAbs0[i] = m_localMatrix0[i].Abs();
			m_localMatrixAbs1[i] = m_localMatrix1[i].Abs();
			for (ndInt32 j = 0; j < 3; ++j)
			{
				const ndVector axis(m_matrix0[i].CrossProduct(m_matrix1[j]));
				if (axis.DotProduct(axis).GetScalar() > ndFloat32(1.0e-5f))
				{
					m_crossAxis[index] = (axis.Normalize()).Abs();
					index++;
				}
			}
		}

		for (ndInt32 i = index; i < ndInt32(sizeof(m_crossAxis) / sizeof(m_crossAxis[0])); ++i)
		{
			m_crossAxis[i] = m_crossAxis[0];
		}

		m_matrixAbs0.m_posit = ndVector::m_wOne;
		m_matrixAbs1.m_posit = ndVector::m_wOne;
		m_localMatrixAbs0.m_posit = ndVector::m_wOne;
		m_localMatrixAbs1.m_posit = ndVector::m_wOne;

		ndVector tmp;
		ndVector::Transpose4x4(
			m_crossAxis[0], m_crossAxis[1], m_crossAxis[2], tmp,
			m_crossAxis[0], m_crossAxis[1], m_crossAxis[2], m_crossAxis[3]);
		ndVector::Transpose4x4(
			m_crossAxis[3], m_crossAxis[4], m_crossAxis[5], tmp,
			m_crossAxis[4], m_crossAxis[5], m_crossAxis[6], m_crossAxis[7]);
		ndVector::Transpose4x4(
			m_crossAxis[6], m_crossAxis[7], m_crossAxis[8], tmp,
			m_crossAxis[8], m_crossAxis[8], m_crossAxis[8], m_crossAxis[8]);
	}

	ndFloat32 CalculateBox1Distance2(const ndVector& origin0, const ndVector& size0, const ndVector& origin1, const ndVector& size1) const
	{
		const ndVector localOrigin1(m_localMatrix0.TransformVector(origin1));
		const ndVector localSize1(m_localMatrixAbs0.m_front.Scale(size1.m_x) + m_localMatrixAbs0.m_up.Scale(size1.m_y) + m_localMatrixAbs0.m_right.Scale(size1.m_z));

		const ndVector minLocalP1(localOrigin1 - localSize1);
		const ndVector maxLocalP1(localOrigin1 + localSize1);

		const ndVector minLocalP0(origin0 - size0);
		const ndVector maxLocalP0(origin0 + size0);

		const ndVector minBox(minLocalP1 - maxLocalP0);
		const ndVector maxBox(maxLocalP1 - minLocalP0);

		ndFloat32 dist2 = ndBoxDistanceToOrigin2(minBox, maxBox);
		return dist2;
	}

	ndFloat32 CalculateBox0Distance2(const ndVector& origin0, const ndVector& size0, const ndVector& origin1, const ndVector& size1) const
	{
		const ndVector localOrigin0(m_localMatrix1.TransformVector(origin0));
		const ndVector localSize0(m_localMatrixAbs1.m_front.Scale(size0.m_x) + m_localMatrixAbs1.m_up.Scale(size0.m_y) + m_localMatrixAbs1.m_right.Scale(size0.m_z));

		const ndVector minLocalP0(localOrigin0 - localSize0);
		const ndVector maxLocalP0(localOrigin0 + localSize0);

		const ndVector minLocalP1(origin1 - size1);
		const ndVector maxLocalP1(origin1 + size1);

		const ndVector minBox(minLocalP1 - maxLocalP0);
		const ndVector maxBox(maxLocalP1 - minLocalP0);

		ndFloat32 dist2 = ndBoxDistanceToOrigin2(minBox, maxBox);
		return dist2;
	}

	ndFloat32 CalculateDistance2(const ndVector& localOrigin0, const ndVector& size0, const ndVector& localOrigin1, const ndVector& size1) const
	{
		ndFloat32 separatingDistance2 = CalculateBox0Distance2(localOrigin0, size0, localOrigin1, size1); 
		if (separatingDistance2 == ndFloat32(0.0f))
		{
			separatingDistance2 = CalculateBox1Distance2(localOrigin0, size0, localOrigin1, size1);
			if (separatingDistance2 == ndFloat32(0.0f))
			{
				const ndVector origin0(m_matrix0.TransformVector(localOrigin0));
				const ndVector span0(m_matrixAbs0[0].Scale(size0[0]) + m_matrixAbs0[1].Scale(size0[1]) + m_matrixAbs0[2].Scale(size0[2]));
				const ndVector minBox0(origin0 - span0);
				const ndVector maxBox0(origin0 + span0);
				
				const ndVector origin1(m_matrix1.TransformVector(localOrigin1));
				const ndVector span1(m_matrixAbs1[0].Scale(size1[0]) + m_matrixAbs1[1].Scale(size1[1]) +	m_matrixAbs1[2].Scale(size1[2]));
				const ndVector minBox1(origin1 - span1);
				const ndVector maxBox1(origin1 + span1);
				
				const ndVector minBox0_x(minBox0.m_x);
				const ndVector minBox0_y(minBox0.m_y);
				const ndVector minBox0_z(minBox0.m_z);
				const ndVector maxBox0_x(maxBox0.m_x);
				const ndVector maxBox0_y(maxBox0.m_y);
				const ndVector maxBox0_z(maxBox0.m_z);
				
				const ndVector minBox1_x(minBox1.m_x);
				const ndVector minBox1_y(minBox1.m_y);
				const ndVector minBox1_z(minBox1.m_z);
				const ndVector maxBox1_x(maxBox1.m_x);
				const ndVector maxBox1_y(maxBox1.m_y);
				const ndVector maxBox1_z(maxBox1.m_z);
				
				ndInt32 i = 0;
				do
				{
					const ndVector minProject0(m_crossAxis[i + 0] * minBox0_x + m_crossAxis[i + 1] * minBox0_y + m_crossAxis[i + 2] * minBox0_z);
					const ndVector maxProject0(m_crossAxis[i + 0] * maxBox0_x + m_crossAxis[i + 1] * maxBox0_y + m_crossAxis[i + 2] * maxBox0_z);
					const ndVector minProject00(minProject0.GetMin(maxProject0));
					const ndVector maxProject00(minProject0.GetMax(maxProject0));
				
					const ndVector minProject1(m_crossAxis[i + 0] * minBox1_x + m_crossAxis[i + 1] * minBox1_y + m_crossAxis[i + 2] * minBox1_z);
					const ndVector maxProject1(m_crossAxis[i + 0] * maxBox1_x + m_crossAxis[i + 1] * maxBox1_y + m_crossAxis[i + 2] * maxBox1_z);
					const ndVector minProject11(minProject1.GetMin(maxProject1));
					const ndVector maxProject11(minProject1.GetMax(maxProject1));
				
					const ndVector box0 (maxProject11 - minProject00);
					const ndVector box1 (minProject11 - maxProject00);
				
					const ndVector mask((box0 * box1) > ndVector::m_zero);
					const ndVector dist(box0.Abs().GetMin(box1.Abs()) & mask);

					const ndVector maxVal(dist.GetMax());
					separatingDistance2 = maxVal.GetScalar();
					i += 3;
				} while ((i < 9) && (separatingDistance2 == ndFloat32 (0.0f)));
				separatingDistance2 = separatingDistance2 * separatingDistance2;
			}
		}

		return separatingDistance2;
	}

	ndMatrix m_matrix0;
	ndMatrix m_matrix1;
	ndMatrix m_matrixAbs0;
	ndMatrix m_matrixAbs1;

	ndMatrix m_localMatrix0;
	ndMatrix m_localMatrix1;
	ndMatrix m_localMatrixAbs0;
	ndMatrix m_localMatrixAbs1;

	ndVector m_crossAxis[9];
} D_GCC_NEWTON_ALIGN_32;

class ndStackBvhStackEntry
{
	public:
	void PushStackEntry(
		ndContactSolver::ndBoxBoxDistance2& data,
		ndInt32& stack,
		ndStackBvhStackEntry* const stackPool,
		const ndShapeCompound::ndNodeBase* const compoundNode,
		ndShapeStatic_bvh* const bvhTreeCollision,
		ndInt32 treeNodeType,
		const ndAabbPolygonSoup::ndNode* const treeNode)
	{
		if (stack < ((2 * D_COMPOUND_STACK_DEPTH) - 4))
		{
			ndVector bvhp0;
			ndVector bvhp1;
			bvhTreeCollision->GetNodeAabb(treeNode, bvhp0, bvhp1);
			const ndVector bvhSize((bvhp1 - bvhp0) * ndVector::m_half);
			const ndVector bvhOrigin((bvhp1 + bvhp0) * ndVector::m_half);

			ndInt32 j = stack;
			ndFloat32 dist2 = data.CalculateDistance2(compoundNode->m_origin, compoundNode->m_size, bvhOrigin, bvhSize);
			for (; j && (dist2 > stackPool[j - 1].m_dist2); --j)
			{
				stackPool[j] = stackPool[j - 1];
			}
			stackPool[j].m_treeNodeIsLeaf = treeNodeType;
			stackPool[j].m_compoundNode = compoundNode;
			stackPool[j].m_collisionTreeNode = treeNode;
			stackPool[j].m_dist2 = dist2;
			stack++;
			ndAssert(stack < 2 * D_COMPOUND_STACK_DEPTH);
		}
	}

	const ndShapeCompound::ndNodeBase* m_compoundNode;
	const ndAabbPolygonSoup::ndNode* m_collisionTreeNode;
	ndFloat32 m_dist2;
	ndInt32 m_treeNodeIsLeaf;
};

class ndStackEntry
{
	public:
	void PushStackEntry(
		ndContactSolver::ndBoxBoxDistance2& data,
		ndInt32& stack,
		ndStackEntry* const stackPool,
		const ndShapeCompound::ndNodeBase* const node0,
		const ndShapeCompound::ndNodeBase* const node1)
	{
		if (stack < ((2 * D_COMPOUND_STACK_DEPTH) - 4))
		{
			ndAssert(node0);
			ndAssert(node1);

			ndInt32 j = stack;
			ndFloat32 subDist2 = data.CalculateDistance2(node0->m_origin, node0->m_size, node1->m_origin, node1->m_size);
			for (; j && (subDist2 > stackPool[j - 1].m_dist2); --j)
			{
				stackPool[j] = stackPool[j - 1];
			}
			stackPool[j].m_node0 = node0;
			stackPool[j].m_node1 = node1;
			stackPool[j].m_dist2 = subDist2;
			stack++;
			ndAssert(stack < 2 * D_COMPOUND_STACK_DEPTH);
		}
	}

	ndFloat32 CalculateHeighfieldDist2(const ndContactSolver::ndBoxBoxDistance2& data, const ndShapeCompound::ndNodeBase* const compoundNode, ndShapeInstance* const heightfieldInstance)
	{
		const ndVector scale(heightfieldInstance->GetScale());
		const ndVector invScale(heightfieldInstance->GetInvScale());
		const ndVector size(invScale * data.m_localMatrixAbs1.RotateVector(compoundNode->m_size));
		const ndVector origin(invScale * data.m_localMatrix1.TransformVector(compoundNode->m_origin));
		const ndVector p0(origin - size);
		const ndVector p1(origin + size);

		ndVector boxP0;
		ndVector boxP1;
		ndShapeHeightfield* const shape = heightfieldInstance->GetShape()->GetAsShapeHeightfield();
		shape->GetLocalAabb(p0, p1, boxP0, boxP1);
		const ndVector boxSize((boxP1 - boxP0) * ndVector::m_half * scale);
		const ndVector boxOrigin((boxP1 + boxP0) * ndVector::m_half * scale);

		ndFloat32 dist2 = data.CalculateDistance2(compoundNode->m_origin, compoundNode->m_size, boxOrigin, boxSize);
		return dist2;
	}

	ndFloat32 CalculateProceduralDist2(const ndContactSolver::ndBoxBoxDistance2& data, const ndShapeCompound::ndNodeBase* const compoundNode, ndShapeInstance* const proceduralInstance)
	{
		const ndVector scale(proceduralInstance->GetScale());
		const ndVector invScale(proceduralInstance->GetInvScale());
		const ndVector size(invScale * data.m_localMatrixAbs1.RotateVector(compoundNode->m_size));
		const ndVector origin(invScale * data.m_localMatrix1.TransformVector(compoundNode->m_origin));
		ndFloat32 dist2 = data.CalculateDistance2(compoundNode->m_origin, compoundNode->m_size, origin, size);
		return dist2;
	}
	
	const ndShapeCompound::ndNodeBase* m_node0;
	const ndShapeCompound::ndNodeBase* m_node1;
	ndFloat32 m_dist2;
};

ndContactSolver::ndContactSolver()
	:ndDownHeap<ndMinkFace*, ndFloat32>(m_heapBuffer, sizeof(m_heapBuffer))
	,m_instance0(nullptr)
	,m_instance1(nullptr)
	,m_separatingVector(ndContact::m_initialSeparatingVector)
	,m_contact(nullptr)
	,m_freeFace(nullptr)
	,m_notification(nullptr)
	,m_contactBuffer(nullptr)
	,m_timestep(ndFloat32 (0.0f))
	,m_skinMargin(ndFloat32(0.0f))
	,m_separationDistance(ndFloat32(0.0f))
	,m_threadId(0)
	,m_maxCount(D_MAX_CONTATCS)
	,m_faceIndex(0)
	,m_vertexIndex(0)
	,m_pruneContacts(1)
	,m_intersectionTestOnly(0)
{
}

ndContactSolver::ndContactSolver(ndShapeInstance* const instance, ndContactNotify* const notification, ndFloat32 timestep, ndInt32 threadIndex)
	:ndDownHeap<ndMinkFace*, ndFloat32>(m_heapBuffer, sizeof (m_heapBuffer))
	,m_instance0(*instance, (ndShape*)instance->GetShape())
	,m_instance1(*instance, (ndShape*)instance->GetShape())
	,m_separatingVector(ndContact::m_initialSeparatingVector)
	,m_contact(nullptr)
	,m_freeFace(nullptr)
	,m_notification(notification)
	,m_contactBuffer(nullptr)
	,m_timestep(timestep)
	,m_skinMargin(ndFloat32(0.0f))
	,m_separationDistance(ndFloat32(0.0f))
	,m_threadId(threadIndex)
	,m_maxCount(D_MAX_CONTATCS)
	,m_faceIndex(0)
	,m_vertexIndex(0)
	,m_pruneContacts(1)
	,m_intersectionTestOnly(0)
{
}

ndContactSolver::ndContactSolver(ndContact* const contact, ndContactNotify* const notification, ndFloat32 timestep, ndInt32 threadIndex)
	:ndDownHeap<ndMinkFace*, ndFloat32>(m_heapBuffer, sizeof(m_heapBuffer))
	,m_instance0(contact->GetBody0()->GetCollisionShape(), contact->GetBody0()->GetCollisionShape().GetShape())
	,m_instance1(contact->GetBody1()->GetCollisionShape(), contact->GetBody1()->GetCollisionShape().GetShape())
	,m_closestPoint0(ndVector::m_zero)
	,m_closestPoint1(ndVector::m_zero)
	,m_separatingVector(ndContact::m_initialSeparatingVector)
	,m_contact(contact)
	,m_freeFace(nullptr)
	,m_notification(notification)
	,m_contactBuffer(nullptr)
	,m_timestep(timestep)
	,m_skinMargin(ndFloat32(0.0f))
	,m_separationDistance(ndFloat32(0.0f))
	,m_threadId(threadIndex)
	,m_maxCount(D_MAX_CONTATCS)
	,m_faceIndex(0)
	,m_vertexIndex(0)
	,m_pruneContacts(1)
	,m_intersectionTestOnly(0)
{
}

ndContactSolver::ndContactSolver(const ndContactSolver& src, const ndShapeInstance& instance0, const ndShapeInstance& instance1)
	:ndDownHeap<ndMinkFace*, ndFloat32>(m_heapBuffer, sizeof(m_heapBuffer))
	,m_instance0(instance0, ((ndShapeInstance*)&instance0)->GetShape())
	,m_instance1(instance1, ((ndShapeInstance*)&instance1)->GetShape())
	,m_closestPoint0(ndVector::m_zero)
	,m_closestPoint1(ndVector::m_zero)
	,m_separatingVector(src.m_separatingVector)
	,m_contact(src.m_contact)
	,m_freeFace(nullptr)
	,m_notification(src.m_notification)
	,m_contactBuffer(src.m_contactBuffer)
	,m_timestep(src.m_timestep)
	,m_skinMargin(src.m_skinMargin)
	,m_separationDistance(src.m_separationDistance)
	,m_threadId(src.m_threadId)
	,m_maxCount(D_MAX_CONTATCS)
	,m_faceIndex(0)
	,m_vertexIndex(0)
	,m_pruneContacts(src.m_pruneContacts)
	,m_intersectionTestOnly(src.m_intersectionTestOnly)
{
}

void ndContactSolver::TranslateSimplex(const ndVector& step)
{
	m_instance1.m_globalMatrix.m_posit -= step;
	for (ndInt32 i = 0; i < m_vertexIndex; ++i) 
	{
		m_hullSum[i] -= step;
		m_hullDiff[i] += step;
	}
}

void ndContactSolver::SupportVertex(const ndVector& dir0, ndInt32 vertexIndex)
{
	ndAssert(dir0.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(dir0.DotProduct(dir0).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
	ndVector dir1(dir0 * ndVector::m_negOne);
	
	const ndMatrix& matrix0 = m_instance0.m_globalMatrix;
	const ndMatrix& matrix1 = m_instance1.m_globalMatrix;
	ndVector p(matrix0.TransformVector(m_instance0.SupportVertexSpecial(matrix0.UnrotateVector (dir0))) & ndVector::m_triplexMask);
	ndVector q(matrix1.TransformVector(m_instance1.SupportVertexSpecial(matrix1.UnrotateVector (dir1))) & ndVector::m_triplexMask);
	m_hullDiff[vertexIndex] = p - q;
	m_hullSum[vertexIndex] = p + q;
}

ndBigVector ndContactSolver::ReduceLine(ndInt32& indexOut)
{
	const ndBigVector p0(m_hullDiff[0]);
	const ndBigVector p1(m_hullDiff[1]);
	const ndBigVector dp(p1 - p0);
	ndBigVector v;

	const ndFloat64 mag2 = dp.DotProduct(dp).GetScalar();
	ndAssert(mag2 > ndFloat64(0.0f));
	if (mag2 < ndFloat32(1.0e-24f)) 
	{
		v = p0;
		indexOut = 1;
	}
	else 
	{
		const ndFloat64 alpha0 = -p0.DotProduct(dp).GetScalar();
		if (alpha0 > mag2) 
		{
			v = p1;
			indexOut = 1;
			m_hullSum[0] = m_hullSum[1];
			m_hullDiff[0] = m_hullDiff[1];
		}
		else if (alpha0 < ndFloat64(0.0f)) 
		{
			v = p0;
			indexOut = 1;
		}
		else 
		{
			v = p0 + dp.Scale(alpha0 / mag2);
		}
	}
	return v;
}

ndBigVector ndContactSolver::ReduceTriangle(ndInt32& indexOut)
{
	const ndBigVector p0(m_hullDiff[0]);
	const ndBigVector p1(m_hullDiff[1]);
	const ndBigVector p2(m_hullDiff[2]);
	const ndBigVector e10(p1 - p0);
	const ndBigVector e20(p2 - p0);
	const ndFloat64 a00 = e10.DotProduct(e10).GetScalar();
	const ndFloat64 a11 = e20.DotProduct(e20).GetScalar();
	const ndFloat64 a01 = e10.DotProduct(e20).GetScalar();

	const ndFloat64 det = a00 * a11 - a01 * a01;
	ndAssert(det >= ndFloat32(0.0f));
	if (ndAbs(det) > ndFloat32(1.0e-16f)) 
	{
		const ndFloat64 b0 = -e10.DotProduct(p0).GetScalar();
		const ndFloat64 b1 = -e20.DotProduct(p0).GetScalar();

		const ndFloat64 u2 = b1 * a00 - a01 * b0;
		const ndFloat64 u1 = b0 * a11 - a01 * b1;

		if (u2 < ndFloat32(0.0f)) 
		{
			// this looks funny but it is correct
		}
		else if (u1 < ndFloat32(0.0f)) 
		{
			m_hullSum[1] = m_hullSum[2];
			m_hullDiff[1] = m_hullDiff[2];
		}
		else if ((u1 + u2) > det) 
		{
			m_hullSum[0] = m_hullSum[2];
			m_hullDiff[0] = m_hullDiff[2];
		}
		else 
		{
			return p0 + (e10.Scale(u1) + e20.Scale(u2)).Scale(ndFloat64(1.0f) / det);
		}
		indexOut = 2;
		return ReduceLine(indexOut);
	}
	else
	{
		ndInt32 count = 3;
		for (ndInt32 i = 2; i > 0; i--)
		{
			for (ndInt32 j = i - 1; j >= 0; --j)
			{
				ndVector dist(m_hullDiff[i] - m_hullDiff[j]);
				ndFloat32 mag2 = dist.DotProduct(dist).GetScalar();
				if (mag2 < ndFloat32(1.0e-12f))
				{
					count--;
					m_hullSum[j] = m_hullSum[count];
					m_hullDiff[j] = m_hullDiff[count];
					break;
				}
			}
		}
		if (count == 2)
		{
			return ReduceLine(indexOut);
		}
		else if (count == 1)
		{
			indexOut = 1;
			return m_hullDiff[0];
		}
	}
	// this is a degenerated triangle. this should never happens
	ndAssert(0);
	return ndBigVector::m_zero;
}

ndBigVector ndContactSolver::ReduceTetrahedrum(ndInt32& indexOut)
{
	const ndBigVector p0(m_hullDiff[0]);
	const ndBigVector p1(m_hullDiff[1]);
	const ndBigVector p2(m_hullDiff[2]);
	const ndBigVector p3(m_hullDiff[3]);
	const ndBigVector e10(p1 - p0);
	const ndBigVector e20(p2 - p0);
	const ndBigVector e30(p3 - p0);

	const ndFloat64 d0 = sqrt(e10.DotProduct(e10).GetScalar());
	if (d0 > ndFloat64(0.0f)) 
	{
		const ndFloat64 invd0 = ndFloat64(1.0f) / d0;
		const ndFloat64 l10 = e20.DotProduct(e10).GetScalar() * invd0;
		const ndFloat64 l20 = e30.DotProduct(e10).GetScalar() * invd0;
		const ndFloat64 desc11 = e20.DotProduct(e20).GetScalar() - l10 * l10;
		if (desc11 > ndFloat64(0.0f)) 
		{
			const ndFloat64 d1 = sqrt(desc11);
			const ndFloat64 invd1 = ndFloat64(1.0f) / d1;
			const ndFloat64 l21 = (e30.DotProduct(e20).GetScalar() - l20 * l10) * invd1;
			const ndFloat64 desc22 = e30.DotProduct(e30).GetScalar() - l20 * l20 - l21 * l21;
			if (desc22 > ndFloat64(0.0f)) 
			{
				const ndFloat64 d2 = sqrt(desc22);
				const ndFloat64 invd2 = ndFloat64(1.0f) / d2;
				const ndFloat64 b0 = -e10.DotProduct(p0).GetScalar();
				const ndFloat64 b1 = -e20.DotProduct(p0).GetScalar();
				const ndFloat64 b2 = -e30.DotProduct(p0).GetScalar();

				ndFloat64 u1 = b0 * invd0;
				ndFloat64 u2 = (b1 - l10 * u1) * invd1;
				ndFloat64 u3 = (b2 - l20 * u1 - l21 * u2) * invd2 * invd2;
				u2 = (u2 - l21 * u3) * invd1;
				u1 = (u1 - l10 * u2 - l20 * u3) * invd0;
				if (u3 < ndFloat64(0.0f)) 
				{
					// this looks funny but it is correct
				}
				else if (u2 < ndFloat64(0.0f)) 
				{
					m_hullSum[2] = m_hullSum[3];
					m_hullDiff[2] = m_hullDiff[3];
				}
				else if (u1 < ndFloat64(0.0f)) 
				{
					m_hullSum[1] = m_hullSum[3];
					m_hullDiff[1] = m_hullDiff[3];
				}
				else if (u1 + u2 + u3 > ndFloat64(1.0f)) 
				{
					m_hullSum[0] = m_hullSum[3];
					m_hullDiff[0] = m_hullDiff[3];
				}
				else 
				{
					return ndBigVector::m_zero;
				}
				indexOut = 3;
				return ReduceTriangle(indexOut);
			}
		}
	}
	// this is a degenerated tetra. this should never happens.
	// it seems this does happens about once per several millions calls, 
	// I will assume is acceptable. No fall back needed
	return ndBigVector::m_zero;
}

ndInt32 ndContactSolver::CalculateClosestSimplex()
{
	ndBigVector v(ndBigVector::m_zero);
	ndInt32 index = 1;
	if (m_vertexIndex <= 0) 
	{
		SupportVertex (m_separatingVector, 0);
						
		v = m_hullDiff[0];
	} 
	else 
	{
		switch (m_vertexIndex) 
		{
			case 1:
			{
				v = m_hullDiff[0];
				break;
			}
	
			case 2:
			{
				v = ReduceLine (m_vertexIndex);
				break;
			}
	
			case 3:
			{
				v = ReduceTriangle (m_vertexIndex);
				break;
			}
	
			case 4:
			{
				v = ReduceTetrahedrum (m_vertexIndex);
				break;
			}
		}
		index = m_vertexIndex;
	}
	
	ndVector bestNormal (m_separatingVector);
	
	ndInt32 iter = 0;
	ndInt32 cycling = 0;
	ndFloat64 minDist = ndFloat32 (1.0e20f);
	ndFloat64 bestNormalDist = ndFloat32 (1.0e20f);
	do 
	{
		ndFloat64 dist = v.DotProduct(v).GetScalar();
		if (dist < ndFloat32 (1.0e-9f)) 
		{
			// very deep penetration, resolve with generic Minkowski solver
			return -index; 
		}
	
		if (dist < minDist) 
		{
			minDist = dist;
			cycling = -1;
		}
		cycling ++;
		if (cycling > 4) 
		{
			return -index;
		}
	
		const ndVector dir (v.Scale (-ndRsqrt(dist)));
		ndAssert (dir.m_w == ndFloat32 (0.0f));
		SupportVertex (dir, index);
	
		const ndBigVector w (m_hullDiff[index]);
		const ndVector wv (w - v);
		ndAssert (wv.m_w == ndFloat32 (0.0f));
		const ndFloat64 dist1 = dir.DotProduct(wv).GetScalar();
		if (dist1 < ndFloat64 (1.0e-3f)) 
		{
			m_separatingVector = dir;
			return index;
		}
	
		if (dist1 < bestNormalDist) 
		{
			bestNormal = dir;
			bestNormalDist = dist1;
		}
	
		index ++;
		switch (index) 
		{
			case 2:
			{
				v = ReduceLine (index);
				break;
			}
	
			case 3:
			{
				v = ReduceTriangle (index);
				break;
			}
	
			case 4:
			{
				v = ReduceTetrahedrum (index);
				break;
			}
		}
	
		iter ++;
	} while (iter < D_CONNICS_CONTATS_ITERATIONS); 

	m_separatingVector = bestNormal;
	return (index < 4) ? index : -4;
}

void ndContactSolver::CalculateContactFromFeacture(ndInt32 featureType)
{
	ndVector d;
	ndVector s;
	switch (featureType)
	{
		case 3:
		{
			const ndBigVector p0(m_hullDiff[0]);
			const ndBigVector p1(m_hullDiff[1]);
			const ndBigVector p2(m_hullDiff[2]);
			const ndBigVector e10(p1 - p0);
			const ndBigVector e20(p2 - p0);
			const ndFloat64 a00 = e10.DotProduct(e10).GetScalar();
			const ndFloat64 a11 = e20.DotProduct(e20).GetScalar();
			const ndFloat64 a01 = e10.DotProduct(e20).GetScalar();

			const ndFloat64 det = a00 * a11 - a01 * a01;
			//ndAssert(det >= ndFloat32(0.0f));
			// check against machine precision 
			if (ndAbs(det) > ndFloat32(1.0e-16f))
			{
				const ndFloat64 b0 = -e10.DotProduct(p0).GetScalar();
				const ndFloat64 b1 = -e20.DotProduct(p0).GetScalar();

				const ndFloat64 u2 = b1 * a00 - a01 * b0;
				const ndFloat64 u1 = b0 * a11 - a01 * b1;

				if (u2 < ndFloat32(0.0f))
				{
					// this looks funny but it is correct
				}
				else if (u1 < ndFloat32(0.0f))
				{
					m_hullSum[1] = m_hullSum[2];
					m_hullDiff[1] = m_hullDiff[2];
				}
				else if ((u1 + u2) > det)
				{
					m_hullSum[0] = m_hullSum[2];
					m_hullDiff[0] = m_hullDiff[2];
				}
				else
				{
					const ndBigVector invDet(ndFloat64(1.0f) / det);
					const ndBigVector q0(m_hullSum[0]);
					const ndBigVector q1(m_hullSum[1]);
					const ndBigVector q2(m_hullSum[2]);
					const ndBigVector q10(q1 - q0);
					const ndBigVector q20(q2 - q0);

					d = ndVector(p0 + (e10.Scale(u1) + e20.Scale(u2)) * invDet);
					s = ndVector(q0 + (q10.Scale(u1) + q20.Scale(u2)) * invDet);
					break;
				}
			}
			else
			{
				// find extreme and reduce line
				ndAssert ((a00 > ndFloat32(0.0f)) || (a11 > ndFloat32(0.0f)));
				const ndBigVector dir ((a00 > a11) ? e10 : e20);
				ndInt32 maxIndex = 0;
				ndInt32 minIndex = 0;
				ndFloat64 maxVal = ndFloat32(-1.0e20f);
				ndFloat64 minVal = ndFloat32( 1.0e20f);
				for (ndInt32 i = 0; i < 3; ++i)
				{
					ndFloat64 val = dir.DotProduct(m_hullDiff[i]).GetScalar();
					if (val > maxVal) 
					{
						maxIndex = i;
						maxVal = val;
					}
					if (val < minVal)
					{
						minIndex = i;
						minVal = val;
					}
				}
				const ndBigVector mindiff(m_hullDiff[minIndex]);
				const ndBigVector minSum(m_hullSum[minIndex]);
				const ndBigVector maxdiff(m_hullDiff[maxIndex]);
				const ndBigVector maxSum(m_hullSum[maxIndex]);

				m_hullDiff[0] = mindiff;
				m_hullSum[0] = minSum;
				m_hullDiff[1] = maxdiff;
				m_hullSum[1] = maxSum;
			}
		}

		case 2:
		{
			const ndBigVector p0(m_hullDiff[0]);
			const ndBigVector p1(m_hullDiff[1]);
			const ndBigVector dp(p1 - p0);

			const ndFloat64 mag2 = dp.DotProduct(dp).GetScalar();
			ndAssert(mag2 > ndFloat64(0.0f));
			if (mag2 < ndFloat32(1.0e-24f))
			{
				s = m_hullSum[0];
				d = m_hullDiff[0];
			}
			else
			{
				const ndFloat64 alpha0 = -p0.DotProduct(dp).GetScalar();
				if (alpha0 > mag2)
				{
					s = m_hullSum[1];
					d = m_hullDiff[1];
				}
				else if (alpha0 < ndFloat64(0.0f))
				{
					s = m_hullSum[0];
					d = m_hullDiff[0];
				}
				else
				{
					const ndBigVector scale(alpha0 / mag2);
					const ndBigVector q0(m_hullSum[0]);
					const ndBigVector q1(m_hullSum[1]);
					const ndBigVector dq(q1 - q0);
					d = ndVector(p0 + dp * scale);
					s = ndVector(q0 + dq * scale);
				}
			}
			break;
		}

		case 1:
		default:
		{
			s = m_hullSum[0];
			d = m_hullDiff[0];
			break;
		}
	}

	m_closestPoint0 = ndVector::m_half * (s + d);
	m_closestPoint1 = ndVector::m_half * (s - d);
	ndAssert(m_separatingVector.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(m_separatingVector.DotProduct(m_separatingVector).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
}

ndInt32 ndContactSolver::ConvexPolygonToLineIntersection(const ndVector& normal, ndInt32 count1, ndVector* const shape1, ndInt32 count2, ndVector* const shape2, ndVector* const contactOut, ndVector* const mem) const
{
	ndInt32 count = 0;
	ndVector* output = mem;

	ndAssert(count1 >= 3);
	ndAssert(count2 <= 2);
	ndAssert(normal.m_w == ndFloat32(0.0f));

	ndVector* ptr = nullptr;
	// face line intersection
	if (count2 == 2) 
	{
		ptr = (ndVector*)&shape2[0];
		ndInt32 i0 = count1 - 1;
		for (ndInt32 i1 = 0; i1 < count1; ++i1) 
		{
			const ndVector testPoint(shape1[i0]);
			const ndVector n(normal.CrossProduct(shape1[i1] - testPoint));
			
			ndAssert(n.m_w == ndFloat32(0.0f));
			ndAssert(n.DotProduct(n).GetScalar() > ndFloat32(0.0f));

			ndFloat32 test0 = n.DotProduct(ptr[0] - testPoint).GetScalar();
			ndFloat32 test1 = n.DotProduct(ptr[1] - testPoint).GetScalar();

			if (test0 >= ndFloat32(0.0f)) 
			{
				if (test1 >= ndFloat32(0.0f)) 
				{
					output[count + 0] = ptr[0];
					output[count + 1] = ptr[1];
					count += 2;
				}
				else 
				{
					ndVector dp(ptr[1] - ptr[0]);
					ndAssert(dp.m_w == ndFloat32(0.0f));
					ndFloat32 den = n.DotProduct(dp).GetScalar();
					if (ndAbs(den) < ndFloat32 (1.0e-10f)) 
					{
						den = ndFloat32(1.0e-10f);
					}
					output[count + 0] = ptr[0];
					ndAssert(dp.m_w == ndFloat32(0.0f));
					output[count + 1] = ptr[0] - dp.Scale(test0 / den);
					count += 2;
				}
			}
			else if (test1 >= ndFloat32(0.0f)) 
			{
				ndVector dp(ptr[1] - ptr[0]);
				ndAssert(dp.m_w == ndFloat32(0.0f));
				ndFloat32 den = n.DotProduct(dp).GetScalar();
				if (ndAbs(den) < ndFloat32(1.0e-10f))
				{
					den = ndFloat32(1.0e-10f);
				}
				ndAssert(dp.m_w == ndFloat32(0.0f));
				output[count] = ptr[0] - dp.Scale(test0 / den);
				count++;
				output[count] = ptr[1];
				count++;
			}
			else 
			{
				return 0;
			}

			count2 = count;
			ptr = output;
			output = &output[count];
			count = 0;
			i0 = i1;
		}
	}
	else if (count2 == 1) 
	{
		const ndVector& p = shape2[0];
		ndInt32 i0 = count1 - 1;
		for (ndInt32 i1 = 0; i1 < count1; ++i1) 
		{
			ndVector n(normal.CrossProduct(shape1[i1] - shape1[i0]));
			ndAssert(n.m_w == ndFloat32(0.0f));
			ndAssert(n.DotProduct(n).GetScalar() > ndFloat32(0.0f));
			ndFloat32 test0 = n.DotProduct(p - shape1[i0]).GetScalar();
			if (test0 < ndFloat32(-1.e-3f)) 
			{
				return 0;
			}
			i0 = i1;
		}
		ptr = output;
		output[count] = p;
		count++;
	}
	else 
	{
		count2 = 0;
	}

	for (ndInt32 i0 = 0; i0 < count2; ++i0) 
	{
		contactOut[i0] = ptr[i0];
	}
	return count2;
}

ndInt32 ndContactSolver::ConvexPolygonsIntersection(const ndVector& normal, ndInt32 count0, ndVector* const shape0, ndInt32 count1, ndVector* const shape1, ndVector* const contactOut, ndInt32 maxContacts) const
{
	ndInt32 count = 0;
	if (count1 <= 2) 
	{
		count = ConvexPolygonToLineIntersection(normal.Scale(ndFloat32(-1.0f)), count0, shape0, count1, shape1, contactOut, &contactOut[count0 + count1 + maxContacts]);
	}
	else if (count0 <= 2) 
	{
		count = ConvexPolygonToLineIntersection(normal, count1, shape1, count0, shape0, contactOut, &contactOut[count0 + count1 + maxContacts]);
	}
	else 
	{
		ndAssert(count0 >= 3);
		ndAssert(count1 >= 3);

		dgPerimenterEdge subdivision[128];
		ndAssert((2 * (count0 + count1)) < ndInt32(sizeof(subdivision) / sizeof(subdivision[0])));

		for (ndInt32 i0 = 1; i0 < count1; ++i0) 
		{
			subdivision[i0].m_vertex = &shape1[i0];
			subdivision[i0].m_prev = &subdivision[i0 - 1];
			subdivision[i0].m_next = &subdivision[i0 + 1];
		}
		subdivision[0].m_vertex = &shape1[0];
		subdivision[0].m_prev = &subdivision[count1 - 1];
		subdivision[0].m_next = &subdivision[1];

		subdivision[count1 - 1].m_next = &subdivision[0];

		dgPerimenterEdge* edgeClipped[2];
		ndVector* output = &contactOut[count0 + count1 + maxContacts];

		edgeClipped[0] = nullptr;
		edgeClipped[1] = nullptr;
		ndInt32 j0 = 0;
		ndInt32 edgeIndex = count1;
		dgPerimenterEdge* poly = &subdivision[0];
		for (ndInt32 j1 = count0 - 1; j1 >= 0; --j1) 
		{
			const ndVector testPoint(shape0[j0]);
			const ndVector n(normal.CrossProduct(shape0[j1] - testPoint));
			ndAssert(n.m_w == 0.0f);
			j0 = j1;
			count = 0;
			dgPerimenterEdge* tmp = poly;
			ndInt32 isInside = 0;
			ndFloat32 test0 = n.DotProduct(*tmp->m_vertex - testPoint).GetScalar();
			do 
			{
				ndFloat32 test1 = n.DotProduct(*tmp->m_next->m_vertex - testPoint).GetScalar();

				if (test0 >= ndFloat32(0.0f)) 
				{
					isInside |= 1;
					if (test1 < ndFloat32(0.0f)) 
					{
						const ndVector& p0 = *tmp->m_vertex;
						const ndVector& p1 = *tmp->m_next->m_vertex;
						ndVector dp(p1 - p0);
						ndAssert(dp.m_w == 0.0f);
						ndFloat32 den = n.DotProduct(dp).GetScalar();
						if (ndAbs(den) < ndFloat32(1.0e-24f)) 
						{
							den = (den >= ndFloat32(0.0f)) ? ndFloat32(1.0e-24f) : ndFloat32(-1.0e-24f);
						}

						den = test0 / den;
						if (den >= ndFloat32(0.0f)) 
						{
							den = ndFloat32(0.0f);
						}
						else if (den <= -1.0f) 
						{
							den = ndFloat32(-1.0f);
						}
						ndAssert(dp.m_w == ndFloat32(0.0f));
						output[0] = p0 - dp.Scale(den);
						edgeClipped[0] = tmp;
						count++;
					}
				}
				else if (test1 >= ndFloat32(0.0f)) 
				{
					const ndVector& p0 = *tmp->m_vertex;
					const ndVector& p1 = *tmp->m_next->m_vertex;
					isInside |= 1;
					ndVector dp(p1 - p0);
					ndAssert(dp.m_w == 0.0f);
					ndFloat32 den = n.DotProduct(dp).GetScalar();
					if (ndAbs(den) < ndFloat32(1.0e-24f)) 
					{
						den = (den >= ndFloat32(0.0f)) ? ndFloat32(1.0e-24f) : ndFloat32(-1.0e-24f);
					}
					den = test0 / den;
					if (den >= ndFloat32(0.0f)) 
					{
						den = ndFloat32(0.0f);
					}
					else if (den <= -1.0f) 
					{
						den = ndFloat32(-1.0f);
					}
					ndAssert(dp.m_w == ndFloat32(0.0f));
					output[1] = p0 - dp.Scale(den);
					edgeClipped[1] = tmp;
					count++;
				}

				test0 = test1;
				tmp = tmp->m_next;
			} while (tmp != poly && (count < 2));

			if (!isInside) 
			{
				return 0;
			}

			if (count == 2) 
			{
				dgPerimenterEdge* const newEdge = &subdivision[edgeIndex];
				newEdge->m_next = edgeClipped[1];
				newEdge->m_prev = edgeClipped[0];
				edgeClipped[0]->m_next = newEdge;
				edgeClipped[1]->m_prev = newEdge;

				newEdge->m_vertex = &output[0];
				edgeClipped[1]->m_vertex = &output[1];
				poly = newEdge;

				output += 2;
				edgeIndex++;
				//ndAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
				ndAssert(edgeIndex < ndInt32(sizeof(subdivision) / sizeof(subdivision[0])));
			}
		}

		ndAssert(poly);
		count = 0;
		dgPerimenterEdge* intersection = poly;
		do 
		{
			contactOut[count] = *intersection->m_vertex;
			count++;
			intersection = intersection->m_next;
		} while (intersection != poly);
	}
	return count;
}

ndInt32 ndContactSolver::PruneSupport(ndInt32 count, const ndVector& dir, const ndVector* const points) const
{
	ndInt32 index = 0;
	ndFloat32 maxVal = ndFloat32(-1.0e20f);
	for (ndInt32 i = 0; i < count; ++i) 
	{
		ndFloat32 dist = dir.DotProduct(points[i]).GetScalar();
		if (dist > maxVal) 
		{
			index = i;
			maxVal = dist;
		}
	}
	return index;
}

ndInt32 ndContactSolver::Prune2dContacts(const ndMatrix& matrix, ndInt32 count, ndContactPoint* const contactArray, ndInt32 maxCount) const
{
	if (count <= 3)
	{
		return count;
	}

	class ndConvexFaceNode
	{
		public:
		ndVector m_point2d;
		ndConvexFaceNode* m_next;
		ndConvexFaceNode* m_prev;
		ndInt32 m_mask;
	};
	
	ndFixSizeArray<ndContactPoint, 32> buffer;
	ndFixSizeArray<ndVector, D_MAX_CONTATCS> array;
	ndFixSizeArray<ndConvexFaceNode, D_MAX_CONTATCS> convexHull;
	const ndVector xyMask(ndVector::m_xMask | ndVector::m_yMask);
	ndUpHeap<ndConvexFaceNode*, ndFloat32> sortHeap(&array[0], D_MAX_CONTATCS * array.GetCapacity());
	
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndVector p(matrix.UntransformVector(contactArray[i].m_point) & xyMask);
		p.m_w = ndFloat32(i);
		array.PushBack(p);
	}
	ndInt32 hullCount = ndConvexHull2d(&array[0], array.GetCount());
	if (hullCount <= 3)
	{
		for (ndInt32 i = hullCount - 1; i >= 0; --i)
		{
			ndInt32 index = ndInt32 (array[i].m_w);
			buffer.PushBack(contactArray[index]);
		}

		for (ndInt32 i = hullCount - 1; i >= 0; --i)
		{
			contactArray[i] = buffer[i];
		}
		return hullCount;
	}

	ndInt32 last = hullCount - 1;
	convexHull.SetCount(hullCount + 1);
	for (ndInt32 i = 0; i < hullCount; ++i)
	{
		convexHull[i].m_point2d = array[i];
		convexHull[i].m_next = &convexHull[i + 1];
		convexHull[i].m_prev = &convexHull[last];
		convexHull[i].m_mask = 0;
		last = i;
	}
	convexHull[last].m_next = &convexHull[0];
	
	ndFloat32 totalArea = ndFloat32(0.0f);
	ndVector areaEdge0(array[1] - array[0]);
	for (ndInt32 i = 2; i < hullCount; ++i)
	{
		const ndVector areaEdge1(array[i] - array[0]);
		ndFloat32 area = areaEdge0.m_y * areaEdge1.m_x - areaEdge0.m_x * areaEdge1.m_y;
		totalArea += area;
		areaEdge0 = areaEdge1;
	}

	ndAssert(totalArea >= ndFloat32(0.0f));
	bool hasLinearCombination = true;
	ndConvexFaceNode* hullPoint = &convexHull[0];
	while (hasLinearCombination)
	{
		sortHeap.Flush();
		hasLinearCombination = false;
		ndConvexFaceNode* ptr = hullPoint;
		ndVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do
		{
			const ndVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			ndFloat32 area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (sortHeap.Value() * ndFloat32(16.0f) < totalArea))
		{
			ndConvexFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask)
			{
				if (hullPoint == corner)
				{
					hullPoint = corner->m_prev;
				}
				hullCount--;
				hasLinearCombination = true;
				corner->m_prev->m_mask = 0;
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}

	while (hullCount > maxCount)
	{
		sortHeap.Flush();
		ndConvexFaceNode* ptr = hullPoint;
		ndVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do
		{
			ndVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			ndFloat32 area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (hullCount > maxCount))
		{
			ndConvexFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask)
			{
				if (hullPoint == corner)
				{
					hullPoint = corner->m_prev;
				}
				hullCount--;
				ndAssert(hullCount >= 1);
				hasLinearCombination = true;
				corner->m_prev->m_mask = 0;
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}

	//hullCount = 0;
	ndConvexFaceNode* ptr = hullPoint;
	do
	{
		ndInt32 index = ndInt32(ptr->m_point2d.m_w);
		//buffer[hullCount] = contactArray[index];
		buffer.PushBack(contactArray[index]);
		//hullCount++;
		ptr = ptr->m_next;
	} while (ptr != hullPoint);

	//for (ndInt32 i = hullCount-1; i >= 0; --i)
	for (ndInt32 i = buffer.GetCount() - 1; i >= 0; --i)
	{
		contactArray[i] = buffer[i];
	}
	return buffer.GetCount();
}

ndInt32 ndContactSolver::Prune3dContacts(const ndMatrix& matrix, ndInt32 count, ndContactPoint* const contactArray, ndInt32 maxCount) const
{
#if 0
	ndFixSizeArray<ndVector, D_MAX_CONTATCS> array;
	ndFloat32 max_x = ndFloat32(1.0e20f);
	ndInt32 maxIndex = 0;
	array.SetCount(count);
	for (ndInt32 i = 0; i < count; ++i) 
	{
		array[i] = matrix.UntransformVector(contactArray[i].m_point);
		array[i].m_w = ndFloat32(i);
		if (array[i].m_x < max_x) 
		{
			maxIndex = i;
			max_x = array[i].m_x;
		}
	}
	dSwap(array[0], array[maxIndex]);

	for (ndInt32 i = 2; i < count; ++i) 
	{
		ndInt32 j = i;
		ndVector tmp(array[i]);
		for (; array[j - 1].m_x > tmp.m_x; --j) 
		{
			dAssert(j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

	ndFloat32 window = ndFloat32(2.5e-3f);
	do 
	{
		ndInt32 packContacts = 0;
		window *= ndFloat32(2.0f);
		const ndFloat32 window2 = window * window;

		ndUnsigned8 mask[D_MAX_CONTATCS];
		memset(mask, 0, count * sizeof(ndUnsigned8));
		for (ndInt32 i = 0; i < count; ++i) 
		{
			if (!mask[i]) 
			{
				const ndFloat32 val = array[i].m_x + window;
				for (ndInt32 j = i + 1; (j < count) && (array[j].m_x < val); ++j) 
				{
					if (!mask[j]) 
					{
						ndVector dp((array[j] - array[i]) & ndVector::m_triplexMask);
						dAssert(dp.m_w == ndFloat32(0.0f));
						ndFloat32 dist2 = dp.DotProduct(dp).GetScalar();
						if (dist2 < window2) 
						{
							mask[j] = 1;
							packContacts = 1;
						}
					}
				}
			}
		}

		if (packContacts) 
		{
			ndInt32 j = 0;
			for (ndInt32 i = 0; i < count; ++i) 
			{
				if (!mask[i]) 
				{
					array[j] = array[i];
					j++;
				}
			}
			count = j;
		}
	} while (count > maxCount);

	//ndContactPoint tmpContact[16];
	ndFixSizeArray<ndContactPoint, D_MAX_CONTATCS> tmpContact;
	tmpContact.SetCount(count);
	for (ndInt32 i = 0; i < count; ++i) 
	{
		ndInt32 index = ndInt32(array[i].m_w);
		tmpContact[i] = contactArray[index];
	}

	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		contactArray[i] = tmpContact[i];
	}

	return count;
#else

	class ndCluster
	{
		public:
		ndVector m_sum;
		ndVector m_sum2;
		ndInt32 m_start;
		ndInt32 m_count;
	};

	ndCluster cluster;
	cluster.m_start = 0;
	cluster.m_count = count;
	cluster.m_sum = ndVector::m_zero;
	cluster.m_sum2 = ndVector::m_zero;

	ndFixSizeArray<ndVector, D_MAX_CONTATCS> array;
	array.SetCount(count);
	for (ndInt32 i = 0; i < count; ++i)
	{
		const ndVector p(matrix.UntransformVector(contactArray[i].m_point));
		array[i] = p;
		array[i].m_w = ndFloat32(i);
		cluster.m_sum += p;
		cluster.m_sum2 += p * p;
	}
	
	ndInt32 baseCount = 0;
	const ndInt32 clusterSize = 8;
	if (cluster.m_count > clusterSize)
	{
		ndCluster spliteStack[128];
		spliteStack[0] = cluster;

		baseCount = 0;
		ndInt32 stack = 1;
		while (stack)
		{
			stack--;
			cluster = spliteStack[stack];

			const ndVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count));
			const ndVector variance2(cluster.m_sum2.Scale(ndFloat32(1.0f) / (ndFloat32)cluster.m_count) - origin * origin);
			ndFloat32 maxVariance2 = ndMax(ndMax(variance2.m_x, variance2.m_y), variance2.m_z);

			if ((cluster.m_count <= clusterSize) || (stack >  (ndInt32(sizeof(spliteStack) / sizeof(spliteStack[0])) - 4)) || (maxVariance2 < ndFloat32(1.e-4f)))
			{
				ndAssert(baseCount <= cluster.m_start);
				const ndFloat32 window = ndFloat32(2.5e-3f);
				ndFloat32 window2 = window * window;
				for (ndInt32 i = 0; i < cluster.m_count - 1; ++i)
				{
					const ndVector test(array[cluster.m_start + i]);
					for (ndInt32 j = i + 1; j < cluster.m_count; ++j)
					{
						const ndVector error(test - array[cluster.m_start + j]);
						ndFloat32 dist2 = error.DotProduct(error).GetScalar();
						if (dist2 < window2)
						{
							array[cluster.m_start + j] = array[cluster.m_start + cluster.m_count - 1];
							cluster.m_count--;
							i--;
							break;
						}
					}
				}

				for (ndInt32 i = 0; i < cluster.m_count; ++i)
				{
					array[baseCount] = array[cluster.m_start + i];
					baseCount++;
				}
			}
			else
			{
				ndInt32 firstSortAxis = 0;
				if ((variance2.m_y >= variance2.m_x) && (variance2.m_y >= variance2.m_z))
				{
					firstSortAxis = 1;
				}
				else if ((variance2.m_z >= variance2.m_x) && (variance2.m_z >= variance2.m_y))
				{
					firstSortAxis = 2;
				}
				ndFloat32 axisVal = origin[firstSortAxis];

				ndInt32 i0 = 0;
				ndInt32 i1 = cluster.m_count - 1;

				const ndInt32 start = cluster.m_start;
				while (i0 < i1)
				{
					while ((array[start + i0][firstSortAxis] <= axisVal) && (i0 < i1))
					{
						++i0;
					};

					while ((array[start + i1][firstSortAxis] > axisVal) && (i0 < i1))
					{
						--i1;
					}
					
					ndAssert(i0 <= i1);
					if (i0 < i1)
					{
						ndSwap(array[start + i0], array[start + i1]);
						++i0;
						--i1;
					}
				}

				while ((array[start + i0][firstSortAxis] <= axisVal) && (i0 < cluster.m_count))
				{
					++i0;
				};

				#ifdef _DEBUG
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndAssert(array[start + i][firstSortAxis] <= axisVal);
				}

				for (ndInt32 i = i0; i < cluster.m_count; ++i)
				{
					ndAssert(array[start + i][firstSortAxis] > axisVal);
				}
				#endif

				ndVector xc(ndVector::m_zero);
				ndVector x2c(ndVector::m_zero);
				for (ndInt32 i = 0; i < i0; ++i)
				{
					const ndVector x(array[start + i]);
					xc += x;
					x2c += x * x;
				}

				ndCluster cluster_i1(cluster);
				cluster_i1.m_start = start + i0;
				cluster_i1.m_count = cluster.m_count - i0;
				cluster_i1.m_sum -= xc;
				cluster_i1.m_sum2 -= x2c;
				spliteStack[stack] = cluster_i1;
				stack++;

				ndCluster cluster_i0(cluster);
				cluster_i0.m_start = start;
				cluster_i0.m_count = i0;
				cluster_i0.m_sum = xc;
				cluster_i0.m_sum2 = x2c;
				spliteStack[stack] = cluster_i0;
				stack++;
			}
		}
	}
	else
	{
		ndAssert(cluster.m_start == 0);
		const ndFloat32 window = ndFloat32(2.5e-3f);
		ndFloat32 window2 = window * window;
		for (ndInt32 i = 0; i < cluster.m_count - 1; ++i)
		{
			const ndVector test(array[i]);
			for (ndInt32 j = i + 1; j < cluster.m_count; ++j)
			{
				const ndVector error(test - array[j]);
				ndFloat32 dist2 = error.DotProduct(error).GetScalar();
				if (dist2 < window2)
				{
					array[j] = array[cluster.m_count - 1];
					cluster.m_count--;
					i--;
					break;
				}
			}
		}
		baseCount = cluster.m_count;
	}

	count = baseCount;
	if (baseCount > maxCount)
	{
		ndInt32 maxIndex = 0;
		ndFloat32 min_x = ndFloat32(1.0e20f);
		for (ndInt32 i = 0; i < count; ++i)
		{
			if (array[i].m_x < min_x)
			{
				maxIndex = i;
				min_x = array[i].m_x;
			}
		}
		ndSwap(array[0], array[maxIndex]);

		for (ndInt32 i = 2; i < count; ++i)
		{
			ndInt32 j = i;
			ndVector tmp(array[i]);
			for (; array[j - 1].m_x > tmp.m_x; --j)
			{
				ndAssert(j > 0);
				array[j] = array[j - 1];
			}
			array[j] = tmp;
		}

		ndFloat32 window = ndFloat32(2.5e-3f);
		do
		{
			ndInt32 packContacts = 0;
			window *= ndFloat32(2.0f);
			const ndFloat32 window2 = window * window;

			ndUnsigned8 mask[D_MAX_CONTATCS];
			memset(mask, 0, count * sizeof(ndUnsigned8));
			for (ndInt32 i = 0; i < count; ++i)
			{
				if (!mask[i])
				{
					const ndFloat32 val = array[i].m_x + window;
					for (ndInt32 j = i + 1; (j < count) && (array[j].m_x < val); ++j)
					{
						if (!mask[j])
						{
							ndVector dp((array[j] - array[i]) & ndVector::m_triplexMask);
							ndAssert(dp.m_w == ndFloat32(0.0f));
							ndFloat32 dist2 = dp.DotProduct(dp).GetScalar();
							if (dist2 < window2)
							{
								mask[j] = 1;
								packContacts = 1;
							}
						}
					}
				}
			}

			if (packContacts)
			{
				ndInt32 j = 0;
				for (ndInt32 i = 0; i < count; ++i)
				{
					if (!mask[i])
					{
						array[j] = array[i];
						j++;
					}
				}
				count = j;
			}
		} while (count > maxCount);
	}

	ndFixSizeArray<ndContactPoint, D_MAX_CONTATCS> tmpContact;
	tmpContact.SetCount(count);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndInt32 index = ndInt32(array[i].m_w);
		tmpContact[i] = contactArray[index];
	}

	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		contactArray[i] = tmpContact[i];
	}
	return count;
#endif
}

ndInt32 ndContactSolver::PruneContacts(ndInt32 count, ndInt32 maxCount) const
{
	ndVector origin(ndVector::m_zero);
	ndContactPoint* const contactArray = m_contactBuffer;

	for (ndInt32 i = 0; i < count; ++i) 
	{
		origin += contactArray[i].m_point;
	}
	ndVector scale(ndFloat32(1.0f) / (ndFloat32)count);
	origin = origin * scale;
	origin.m_w = ndFloat32(1.0f);

	ndMatrix covariance(ndGetZeroMatrix());
	for (ndInt32 i = 0; i < count; ++i) 
	{
		ndVector p((contactArray[i].m_point - origin) & ndVector::m_triplexMask);
		ndAssert(p.m_w == ndFloat32(0.0f));
		ndMatrix matrix(ndCovarianceMatrix(p, p));
		covariance.m_front += matrix.m_front;
		covariance.m_up += matrix.m_up;
		covariance.m_right += matrix.m_right;
	}

	for (ndInt32 i = 0; i < 3; ++i) 
	{
		if (ndAbs(covariance[i][i]) < ndFloat32(1.0e-8f)) 
		{
			for (ndInt32 j = 0; j < 3; ++j) 
			{
				covariance[i][j] = ndFloat32(0.0f);
				covariance[j][i] = ndFloat32(0.0f);
			}
			covariance[i][i] = ndFloat32(1.0e-8f);
		}
	}

	ndVector eigen(covariance.EigenVectors() & ndVector::m_triplexMask);
	ndAssert(eigen.DotProduct(eigen).GetScalar() > ndFloat32(0.0f));
	eigen = eigen.Normalize();
	covariance.m_posit = origin;
	if (eigen[1] < eigen[2]) 
	{
		ndSwap(eigen[1], eigen[2]);
		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndSwap(covariance[i][1], covariance[i][2]);
		}
	}
	if (eigen[0] < eigen[1]) 
	{
		ndSwap(eigen[0], eigen[1]);
		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndSwap(covariance[i][0], covariance[i][1]);
		}
	}
	if (eigen[1] < eigen[2]) 
	{
		ndSwap(eigen[1], eigen[2]);
		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndSwap(covariance[i][1], covariance[i][2]);
		}
	}

	const ndFloat32 eigenValueError = ndFloat32(1.0e-4f);
	if (eigen[2] > eigenValueError) 
	{
		// 3d convex Hull
		return Prune3dContacts(covariance, count, contactArray, maxCount);
	}
	else if (eigen[1] > eigenValueError) 
	{
		// is a 2d or 1d convex hull
		return Prune2dContacts(covariance, count, contactArray, maxCount);
	}
	else if (eigen[0] > eigenValueError) 
	{
		// is a line or a point convex hull
		if (count > 2) 
		{
			ndFloat32 maxValue = ndFloat32(-1.0e10f);
			ndFloat32 minValue = ndFloat32(-1.0e10f);
			ndInt32 j0 = 0;
			ndInt32 j1 = 0;
			for (ndInt32 i = 0; i < count; ++i) 
			{
				ndFloat32 dist = contactArray[i].m_point.DotProduct(covariance.m_front).GetScalar();
				if (dist > maxValue) 
				{
					j0 = i;
					maxValue = dist;
				}
				if (-dist > minValue) 
				{
					j1 = i;
					minValue = -dist;
				}
			}
			ndContactPoint c0(contactArray[j0]);
			ndContactPoint c1(contactArray[j1]);
			contactArray[0] = c0;
			contactArray[1] = c1;
			count = 2;
		}
		if (count == 2)
		{
			ndVector segment(contactArray[1].m_point - contactArray[0].m_point);
			ndFloat32 dist2 = segment.DotProduct(segment).GetScalar();
			if (dist2 < ndFloat32(5.0e-3f * 5.0e-3f))
			{
				count = 1;
			}
		}
		//ndAssert(count <= 1);
		return count;
	}

	return 1;
}

ndFloat32 ndContactSolver::RayCast(const ndVector& localP0, const ndVector& localP1, ndContactPoint& contactOut)
{
	ndVector point(localP0);
	ndVector point0(localP0);
	ndVector normal(ndVector::m_zero);
	ndVector p0p1(localP0 - localP1);

	// avoid NaN as a result of a division by zero
	if ((p0p1.TestZero().GetSignMask() & 7) == 7) 
	{
		return ndFloat32(1.2f);
	}

	ndFloat32 param = ndFloat32(0.0f);

	ndInt32 index = 0;
	memset(m_hullSum, 0, 4 * sizeof(m_hullSum[0]));
	const ndShapeConvex* const shape = m_instance0.GetShape()->GetAsShapeConvex();

	ndVector dir1(p0p1.Normalize());
	m_hullDiff[0] = shape->SupportVertex(dir1) - point;
	ndBigVector v(m_hullDiff[0]);
	index = 1;
	do 
	{
		ndInt32 iter = 0;
		ndInt32 cycling = 0;
		ndFloat64 minDist = ndFloat32(1.0e20f);

		do 
		{
			ndAssert(v.m_w == ndFloat32(0.0f));
			const ndFloat64 distance = v.DotProduct(v).GetScalar();
			if (distance < ndFloat32(1.0e-9f)) 
			{
				index = -1;
				break;
			}

			if (distance < minDist) 
			{
				minDist = distance;
				cycling = -1;
			}
			cycling++;
			if (cycling > 4) 
			{
				index = -1;
				break;
			}

			ndVector dir(v.Scale(-ndRsqrt(ndFloat32(distance))));
			ndAssert(dir.m_w == ndFloat32(0.0f));
			m_hullDiff[index] = shape->SupportVertex(dir) - point;
			const ndBigVector w(m_hullDiff[index]);
			const ndVector wv(w - v);
			ndAssert(wv.m_w == ndFloat32(0.0f));
			const ndFloat32 distance1 = dir.DotProduct(wv).GetScalar();
			if (distance1 < ndFloat64(1.0e-3f)) 
			{
				normal = dir;
				break;
			}

			index++;
			switch (index)
			{
				case 2:
				{
					v = ReduceLine(index);
					break;
				}

				case 3:
				{
					v = ReduceTriangle(index);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum(index);
					break;
				}
			}

			iter++;
		} while (iter < D_CONNICS_CONTATS_ITERATIONS);

		ndAssert(index);
		if (index > 0) 
		{
			ndVector q(v + point);
			ndFloat32 den = normal.DotProduct(p0p1).GetScalar();
			if (ndAbs(den) < ndFloat32(1.0e-12f)) 
			{
				den = ndSign(den) * ndFloat32(1.0e-12f);
			}
			ndAssert(normal.m_w == ndFloat32(0.0f));
			ndFloat32 t1 = normal.DotProduct(localP0 - q).GetScalar() / den;

			if (t1 < param) 
			{
				index = -1;
				t1 = ndFloat32(0.0f);
			}
			else if (t1 > ndFloat32 (1.0f)) 
			{
				index = -1;
				t1 = ndFloat32(1.0f);
			}
			param = t1;

			point = localP0 - p0p1.Scale(param);
			ndVector step(point0 - point);
			point0 = point;
			for (ndInt32 i = 0; i < index; ++i) 
			{
				m_hullDiff[i] += step;
			}

			switch (index)
			{
				case 1:
				{
					v = m_hullDiff[0];
					break;
				}

				case 2:
				{
					v = ReduceLine(index);
					break;
				}

				case 3:
				{
					v = ReduceTriangle(index);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum(index);
					break;
				}
			}
		}
	} while (index >= 0);

	if ((param > ndFloat32(0.0f)) && (param < ndFloat32(1.0f)))
	{
		contactOut.m_normal = normal;
	}
	else 
	{
		param = ndFloat32(1.2f);
	}
	return param;
}

inline ndMinkFace* ndContactSolver::NewFace()
{
	ndMinkFace* face = (ndMinkFace*)m_freeFace;
	if (m_freeFace) 
	{
		m_freeFace = m_freeFace->m_next;
	}
	else 
	{
		face = &m_facePool[m_faceIndex];
		m_faceIndex++;
		if (m_faceIndex >= D_CONVEX_MINK_MAX_FACES) 
		{
			return nullptr;
		}
	}

#ifdef _DEBUG
	memset(face, 0, sizeof(ndMinkFace));
#endif
	return face;
}

inline ndMinkFace* ndContactSolver::AddFace(ndInt32 v0, ndInt32 v1, ndInt32 v2)
{
	ndMinkFace* const face = NewFace();
	face->m_mark = 0;
	face->m_vertex[0] = ndInt16(v0);
	face->m_vertex[1] = ndInt16(v1);
	face->m_vertex[2] = ndInt16(v2);
	return face;
}

inline void ndContactSolver::PushFace(ndMinkFace* const face)
{
	ndInt32 i0 = face->m_vertex[0];
	ndInt32 i1 = face->m_vertex[1];
	ndInt32 i2 = face->m_vertex[2];

	ndPlane plane(m_hullDiff[i0], m_hullDiff[i1], m_hullDiff[i2]);
	ndFloat32 mag2 = plane.DotProduct(plane & ndVector::m_triplexMask).GetScalar();
	face->m_alive = 1;
	if (mag2 > ndFloat32(1.0e-16f)) 
	{
		face->m_plane = plane.Scale(ndRsqrt(mag2));
		ndMinkFace* face1 = face;
		Push(face1, face->m_plane.m_w);
	}
	else 
	{
		face->m_plane = ndPlane(ndVector::m_zero);
	}
}

inline void ndContactSolver::DeleteFace(ndMinkFace* const face)
{
	dgFaceFreeList* const freeFace = (dgFaceFreeList*)face;
	freeFace->m_next = m_freeFace;
	m_freeFace = freeFace;
}

ndInt32 ndContactSolver::CalculateIntersectingPlane(ndInt32 count)
{
	ndAssert(count >= 1);
	if (count == 1) 
	{
		SupportVertex(m_contact->m_initialSeparatingVector.Scale(ndFloat32(-1.0f)), 1);
		ndVector err(m_hullDiff[1] - m_hullDiff[0]);
		ndAssert(err.m_w == ndFloat32(0.0f));
		if (err.DotProduct(err).GetScalar() < ndFloat32(1.0e-8f)) 
		{
			return -1;
		}
		count = 2;
	}

	if (count == 2) 
	{
		ndVector e0(m_hullDiff[1] - m_hullDiff[0]);
		ndAssert(e0.m_w == ndFloat32(0.0f));
		ndAssert(e0.DotProduct(e0).GetScalar() > ndFloat32(0.0f));
		//ndMatrix matrix(e0.Scale(ndRsqrt(e0.DotProduct(e0).GetScalar())));
		ndMatrix matrix(ndGramSchmidtMatrix(e0));
		ndMatrix rotation(ndPitchMatrix(ndFloat32(45.0f * ndDegreeToRad)));
		ndFloat32 maxArea = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < 8; ++i) 
		{
			SupportVertex(matrix[1], 3);
			ndVector e1(m_hullDiff[3] - m_hullDiff[0]);
			ndAssert(e1.m_w == ndFloat32(0.0f));
			ndVector area(e0.CrossProduct(e1));
			ndFloat32 area2 = area.DotProduct(area).GetScalar();
			if (area2 > maxArea) 
			{
				m_hullSum[2] = m_hullSum[3];
				m_hullDiff[2] = m_hullDiff[3];
				maxArea = area2;
			}
			matrix = rotation * matrix;
		}
		if (ndAbs(maxArea) < ndFloat32(1e-15f)) 
		{
			return -1;
		}
		ndAssert(maxArea > ndFloat32(0.0f));
		count++;
	}

	ndFloat32 volume = ndFloat32(0.0f);
	if (count == 3) 
	{
		ndVector e10(m_hullDiff[1] - m_hullDiff[0]);
		ndVector e20(m_hullDiff[2] - m_hullDiff[0]);
		ndVector normal(e10.CrossProduct(e20));
		ndAssert(normal.m_w == ndFloat32(0.0f));
		ndFloat32 mag2 = normal.DotProduct(normal).GetScalar();
		ndAssert(mag2 > ndFloat32(0.0f));
		normal = normal.Scale(ndRsqrt(mag2));
		SupportVertex(normal, 3);
		volume = normal.DotProduct(m_hullDiff[3] - m_hullDiff[0]).GetScalar();
		if (ndAbs(volume) < ndFloat32(1.0e-10f)) 
		{
			normal = normal.Scale(ndFloat32(-1.0f));
			SupportVertex(normal, 3);
			volume = -normal.DotProduct(m_hullDiff[3] - m_hullDiff[0]).GetScalar();
			if (ndAbs(volume) < ndFloat32(1.0e-10f)) 
			{
				volume = ndFloat32(0.0f);
			}
		}
		count = 4;
	}
	else if (count == 4) 
	{
		ndVector e0(m_hullDiff[1] - m_hullDiff[0]);
		ndVector e1(m_hullDiff[2] - m_hullDiff[0]);
		ndVector e2(m_hullDiff[3] - m_hullDiff[0]);
		ndVector n(e1.CrossProduct(e2));
		ndAssert(n.m_w == ndFloat32(0.0f));
		volume = e0.DotProduct(n).GetScalar();
	}

	ndAssert(count == 4);
	if (volume > ndFloat32(0.0f)) 
	{
		ndSwap(m_hullSum[1], m_hullSum[0]);
		ndSwap(m_hullDiff[1], m_hullDiff[0]);
	}

	if (ndAbs(volume) < ndFloat32(1e-15f)) 
	{
		// this volume is unrealizable, let us build  a different tetrahedron using the method of core 200
		ndVector e1;
		ndVector e2;
		ndVector e3;
		ndVector normal(ndFloat32(0.0f));

		const ndInt32 nCount = ndInt32(sizeof(m_hullDirs) / sizeof(m_hullDirs[0]));
		const ndFloat32 D_CALCULATE_SEPARATING_PLANE_ERROR = ndFloat32(1.0f / 1024.0f);

		ndFloat32 error2 = ndFloat32(0.0f);
		SupportVertex(m_hullDirs[0], 0);

		ndInt32 i = 1;
		for (; i < nCount; ++i) 
		{
			SupportVertex(m_hullDirs[i], 1);
			e1 = m_hullDiff[1] - m_hullDiff[0];
			ndAssert(e1.m_w == ndFloat32(0.0f));
			error2 = e1.DotProduct(e1).GetScalar();
			if (error2 > D_CALCULATE_SEPARATING_PLANE_ERROR) 
			{
				break;
			}
		}

		for (i++; i < nCount; ++i) 
		{
			SupportVertex(m_hullDirs[i], 2);
			e2 = m_hullDiff[2] - m_hullDiff[0];
			normal = e1.CrossProduct(e2);
			ndAssert(normal.m_w == ndFloat32(0.0f));
			error2 = normal.DotProduct(normal).GetScalar();
			if (error2 > D_CALCULATE_SEPARATING_PLANE_ERROR) 
			{
				break;
			}
		}

		error2 = ndFloat32(0.0f);
		for (i++; i < nCount; ++i) 
		{
			SupportVertex(m_hullDirs[i], 3);
			e3 = m_hullDiff[3] - m_hullDiff[0];
			ndAssert(normal.m_w == ndFloat32(0.0f));
			error2 = normal.DotProduct(e3).GetScalar();
			if (ndAbs(error2) > D_CALCULATE_SEPARATING_PLANE_ERROR) 
			{
				break;
			}
		}

		if (i >= nCount) 
		{
			return -1;
		}

		if (error2 > ndFloat32(0.0f)) 
		{
			ndSwap(m_hullSum[1], m_hullSum[2]);
			ndSwap(m_hullDiff[1], m_hullDiff[2]);
		}

		#ifdef _DEBUG
		{
			ndVector f0(m_hullDiff[1] - m_hullDiff[0]);
			ndVector f1(m_hullDiff[2] - m_hullDiff[0]);
			ndVector f2(m_hullDiff[3] - m_hullDiff[0]);
			ndVector n(f1.CrossProduct(f2));
			ndAssert(n.m_w == ndFloat32(0.0f));
			ndFloat32 volume1 = f0.DotProduct(n).GetScalar();
			ndAssert(volume1 < ndFloat32(0.0f));
		}
		#endif
	}

	// clear the face cache!!
	Flush();
	m_faceIndex = 0;
	m_vertexIndex = 4;
	m_freeFace = nullptr;

	ndMinkFace* const f0 = AddFace(0, 1, 2);
	ndMinkFace* const f1 = AddFace(0, 2, 3);
	ndMinkFace* const f2 = AddFace(2, 1, 3);
	ndMinkFace* const f3 = AddFace(1, 0, 3);

	f0->m_twin[0] = f3;
	f0->m_twin[1] = f2;
	f0->m_twin[2] = f1;

	f1->m_twin[0] = f0;
	f1->m_twin[1] = f2;
	f1->m_twin[2] = f3;

	f2->m_twin[0] = f0;
	f2->m_twin[1] = f3;
	f2->m_twin[2] = f1;

	f3->m_twin[0] = f0;
	f3->m_twin[1] = f1;
	f3->m_twin[2] = f2;

	PushFace(f0);
	PushFace(f1);
	PushFace(f2);
	PushFace(f3);

	ndInt32 cycling = 0;
	ndInt32 iterCount = 0;
	ndFloat32 cyclingMem[4];
	cyclingMem[0] = ndFloat32(1.0e10f);
	cyclingMem[1] = ndFloat32(1.0e10f);
	cyclingMem[2] = ndFloat32(1.0e10f);
	cyclingMem[3] = ndFloat32(1.0e10f);

	const ndFloat32 resolutionScale = ndFloat32(0.125f);
	const ndFloat32 minTolerance = D_PENETRATION_TOL;

	while (GetCount()) 
	{
		ndMinkFace* const faceNode = (*this)[0];
		Pop();

		if (faceNode->m_alive) 
		{
			SupportVertex(faceNode->m_plane & ndVector::m_triplexMask, m_vertexIndex);
			const ndVector& p = m_hullDiff[m_vertexIndex];
			ndFloat32 dist = faceNode->m_plane.Evalue(p);
			ndFloat32 distTolerance = ndMax(ndAbs(faceNode->m_plane.m_w) * resolutionScale, minTolerance);

			if (dist < distTolerance) 
			{
				ndVector sum[3];
				ndVector diff[3];
				m_separatingVector = faceNode->m_plane & ndVector::m_triplexMask;
				for (ndInt32 i = 0; i < 3; ++i) 
				{
					ndInt32 j = faceNode->m_vertex[i];
					sum[i] = m_hullSum[j];
					diff[i] = m_hullDiff[j];
				}
				for (ndInt32 i = 0; i < 3; ++i) 
				{
					m_hullSum[i] = sum[i];
					m_hullDiff[i] = diff[i];
				}
				return 3;
			}

			iterCount++;
			bool isCycling = false;
			cyclingMem[cycling] = dist;
			if (iterCount > 10) 
			{
				ndInt32 cyclingIndex = cycling;
				for (ndInt32 i = 0; i < 3; ++i) 
				{
					ndInt32 cyclingIndex0 = (cyclingIndex - 1) & 3;
					if (((cyclingMem[cyclingIndex0] - cyclingMem[cyclingIndex]) < ndFloat32(-1.0e-5f))) 
					{
						isCycling = true;
						cyclingMem[0] = ndFloat32(1.0e10f);
						cyclingMem[1] = ndFloat32(1.0e10f);
						cyclingMem[2] = ndFloat32(1.0e10f);
						cyclingMem[3] = ndFloat32(1.0e10f);
						break;
					}
					cyclingIndex = cyclingIndex0;
				}
			}
			cycling = (cycling + 1) & 3;

			if (!isCycling) 
			{
				m_faceStack[0] = faceNode;
				ndInt32 stackIndex = 1;
				ndInt32 deletedCount = 0;

				while (stackIndex) 
				{
					stackIndex--;
					ndMinkFace* const face = m_faceStack[stackIndex];

					if (!face->m_mark && (face->m_plane.Evalue(p) > ndFloat32(0.0f))) 
					{
						#ifdef _DEBUG
						for (ndInt32 i = 0; i < deletedCount; ++i) 
						{
							ndAssert(m_deletedFaceList[i] != face);
						}
						#endif

						m_deletedFaceList[deletedCount] = face;
						deletedCount++;
						ndAssert(deletedCount < ndInt32 (sizeof(m_deletedFaceList) / sizeof(m_deletedFaceList[0])));
						face->m_mark = 1;

						for (ndInt32 i = 0; i < 3; ++i) 
						{
							ndMinkFace* const twinFace = face->m_twin[i];
							if (twinFace && !twinFace->m_mark) 
							{
								m_faceStack[stackIndex] = twinFace;
								stackIndex++;
								ndAssert(stackIndex < ndInt32 (sizeof(m_faceStack) / sizeof(m_faceStack[0])));
							}
						}
					}
				}

				//ndAssert (SanityCheck());
				ndInt32 newCount = 0;
				for (ndInt32 i = 0; i < deletedCount; ++i) 
				{
					ndMinkFace* const face = m_deletedFaceList[i];
					face->m_alive = 0;
					ndAssert(face->m_mark == 1);
					ndInt32 j0 = 2;
					for (ndInt32 j1 = 0; j1 < 3; j1++) 
					{
						ndMinkFace* const twinFace = face->m_twin[j0];
						if (twinFace && !twinFace->m_mark) 
						{
							ndMinkFace* const newFace = NewFace();
							if (newFace) 
							{
								newFace->m_mark = 0;
								newFace->m_vertex[0] = ndInt16(m_vertexIndex);
								newFace->m_vertex[1] = ndInt16(face->m_vertex[j0]);
								newFace->m_vertex[2] = ndInt16(face->m_vertex[j1]);
								PushFace(newFace);

								newFace->m_twin[1] = twinFace;
								ndInt32 index = (twinFace->m_twin[0] == face) ? 0 : ((twinFace->m_twin[1] == face) ? 1 : 2);
								twinFace->m_twin[index] = newFace;

								m_coneFaceList[newCount] = newFace;
								newCount++;
								ndAssert(newCount < ndInt32 (sizeof(m_coneFaceList) / sizeof(m_coneFaceList[0])));
							}
							else 
							{
								// this is very rare but is does happend with some degenerated faces.
								return -1;
							}
						}
						j0 = j1;
					}
				}

				ndInt32 i0 = newCount - 1;
				for (ndInt32 i1 = 0; i1 < newCount; ++i1) 
				{
					ndMinkFace* const faceA = m_coneFaceList[i0];
					ndAssert(faceA->m_mark == 0);

					ndInt32 j0 = newCount - 1;
					for (ndInt32 j1 = 0; j1 < newCount; j1++) 
					{
						if (i0 != j0) 
						{
							ndMinkFace* const faceB = m_coneFaceList[j0];
							ndAssert(faceB->m_mark == 0);
							if (faceA->m_vertex[2] == faceB->m_vertex[1]) 
							{
								faceA->m_twin[2] = faceB;
								faceB->m_twin[0] = faceA;
								break;
							}
						}
						j0 = j1;
					}
					i0 = i1;
				}

				m_vertexIndex++;
				ndAssert(m_vertexIndex < ndInt32 (sizeof(m_hullDiff) / sizeof(m_hullDiff[0])));
				//ndAssert(SanityCheck());
			}
		}
		else 
		{
			DeleteFace(faceNode);
		}
	}
	return -1;
}

bool ndContactSolver::CalculateClosestPoints()
{
	ndInt32 simplexPointCount = CalculateClosestSimplex();
	if (simplexPointCount < 0)
	{
		simplexPointCount = CalculateIntersectingPlane(-simplexPointCount);
	}

	if (simplexPointCount > 0)
	{
		ndAssert((simplexPointCount > 0) && (simplexPointCount <= 3));
		CalculateContactFromFeacture(simplexPointCount);

		const ndMatrix& matrix0 = m_instance0.m_globalMatrix;
		const ndMatrix& matrix1 = m_instance1.m_globalMatrix;
		m_closestPoint0 = matrix0.TransformVector(m_instance0.SupportVertexSpecialProjectPoint(matrix0.UntransformVector(m_closestPoint0), matrix0.UnrotateVector(m_separatingVector)));
		m_closestPoint1 = matrix1.TransformVector(m_instance1.SupportVertexSpecialProjectPoint(matrix1.UntransformVector(m_closestPoint1), matrix1.UnrotateVector(m_separatingVector * ndVector::m_negOne)));
		m_vertexIndex = simplexPointCount;
	}
	return simplexPointCount >= 0;
}

//*************************************************************
// calculate proper separation distance for discrete collision.
//*************************************************************
ndInt32 ndContactSolver::CalculateContactsDiscrete()
{
	ndInt32 count = 0;
	if (m_instance0.GetShape()->GetAsShapeCompound() || m_instance1.GetShape()->GetAsShapeCompound())
	{
		count = CompoundContactsDiscrete();
	}
	else if (m_instance0.GetShape()->GetAsShapeConvex())
	{
		count = ConvexContactsDiscrete();
	}
	else
	{
		//ndTrace(("Fix compound contact for pair: %s %s\n", m_instance0.GetShape()->ClassName(), m_instance1.GetShape()->ClassName()));
		//ndAssert(0);
	}

	m_contact->m_timeOfImpact = m_timestep;
	m_contact->m_separatingVector = m_separatingVector;
	ndAssert(!count || (m_separationDistance < ndFloat32(100.0f)));
	m_contact->m_separationDistance = m_separationDistance;
	return count;
}

ndInt32 ndContactSolver::ConvexContactsDiscrete()
{
	const ndVector origin0(m_instance0.m_globalMatrix.m_posit);
	const ndVector origin1(m_instance1.m_globalMatrix.m_posit);
	m_instance0.m_globalMatrix.m_posit = ndVector::m_wOne;
	m_instance1.m_globalMatrix.m_posit -= (origin0 & ndVector::m_triplexMask);

	// handle rare case of two shapes located exactly at the same origin
	const ndVector error(m_instance1.m_globalMatrix.m_posit - m_instance0.m_globalMatrix.m_posit);
	if (error.DotProduct(error).GetScalar() < ndFloat32(1.0e-6f))
	{
		m_instance1.m_globalMatrix.m_posit.m_y += ndFloat32(1.0e-3f);
	}

	ndInt32 count = 0;
	if (m_instance1.GetShape()->GetAsShapeConvex())
	{
		ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
		count = ConvexToConvexContactsDiscrete();
	}
	else
	{
		if (m_instance1.GetShape()->GetAsShapeStaticMesh())
		{
			count = ConvexToStaticMeshContactsDiscrete();
		}
		else
		{
			//ndAssert(0);
			count = 0;
		}
	}

	if (m_pruneContacts)
	{
		switch (count)
		{
			case 0:
			case 1:
				break;

			case 2:
			{
				ndVector length(m_contactBuffer[1].m_point - m_contactBuffer[0].m_point);
				ndFloat32 mag2 = length.DotProduct(length & ndVector::m_triplexMask).GetScalar();
				if (mag2 < ndFloat32(2.0e-3f) * ndFloat32(2.0e-3f))
				{
					m_contactBuffer[0].m_point = ndVector::m_half * (m_contactBuffer[1].m_point + m_contactBuffer[0].m_point);
					count = 1;
				}
				break;
			}

			case 3:
				// could write special case for triangles, but for now, just call prune
				//break;

			default:
				count = PruneContacts(count, 16);
		}
	}

	const ndVector offset(origin0 & ndVector::m_triplexMask);
	m_closestPoint0 += offset;
	m_closestPoint1 += offset;

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (ndInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_point += offset;
		}
	}

	m_instance0.m_globalMatrix.m_posit = origin0;
	m_instance1.m_globalMatrix.m_posit = origin1;
	return count;
}

ndInt32 ndContactSolver::ConvexToConvexContactsDiscrete()
{
	ndAssert(m_instance0.GetConvexVertexCount() && m_instance1.GetConvexVertexCount());
	ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(m_instance1.GetShape()->GetAsShapeConvex());
	ndAssert(!m_instance0.GetShape()->GetAsShapeNull());
	ndAssert(!m_instance1.GetShape()->GetAsShapeNull());

	ndInt32 count = 0;
	bool colliding = CalculateClosestPoints();
	ndFloat32 penetration = m_separatingVector.DotProduct(m_closestPoint1 - m_closestPoint0).GetScalar() - m_skinMargin - D_PENETRATION_TOL;
	m_separationDistance = penetration;
	if (m_intersectionTestOnly)
	{
		count = (penetration <= ndFloat32(0.0f)) ? 1 : 0;
	}
	else if (colliding)
	{
		if (penetration <= ndFloat32(1.0e-5f))
		{
			if (ndInt8 (m_instance0.GetCollisionMode()) & ndInt8(m_instance1.GetCollisionMode()))
			{
				count = CalculateContacts(m_closestPoint0, m_closestPoint1, m_separatingVector * ndVector::m_negOne);
				// skip convex shape polygon because they could have a skirt
				ndShapeConvexPolygon* const convexPolygon = m_instance1.GetShape()->GetAsShapeAsConvexPolygon();
				if (!(count || convexPolygon))
				{
					// poly line failed probably because of rounding error
					// but we know the shapes are colliding
					// just return the closest points as contacts
					m_buffer[0] = ndVector::m_half * (m_closestPoint0 + m_closestPoint1);
					count = 1;
				}
				else if (convexPolygon && count)
				{
					// the contact point most be very close to the polygon
					ndFixSizeArray<ndBigVector, 32> poly;
					ndAssert(convexPolygon->m_count < poly.GetCapacity());
					for (ndInt32 i = 0; i < convexPolygon->m_count; ++i)
					{
						poly.PushBack(convexPolygon->m_localPoly[i]);
					}

					for (ndInt32 i = count - 1; i >= 0; --i)
					{
						//ndBigVector point(m_buffer[i]);
						ndBigVector point(m_buffer[i] - convexPolygon->m_normal * (m_buffer[i] - poly[0]).DotProduct(convexPolygon->m_normal));
						ndBigVector pointInPoly(ndPointToPolygonDistance(point, &poly[0], convexPolygon->m_count));

						const ndBigVector error(point - pointInPoly);
						ndFloat64 dist2 = error.DotProduct(error & ndBigVector::m_triplexMask).GetScalar();

						if (dist2 > ndFloat64 (5.0e-4f))
						{
							count--;
							m_buffer[i] = m_buffer[count];
						}
					}
				}
			}
		}

		count = ndMin(m_maxCount, count);
		ndContactPoint* const contactOut = m_contactBuffer;

		ndBodyKinematic* const body0 = m_contact->GetBody0();
		ndBodyKinematic* const body1 = m_contact->GetBody1();
		ndShapeInstance* const instance0 = &body0->GetCollisionShape();
		ndShapeInstance* const instance1 = &body1->GetCollisionShape();

		ndVector normal(m_separatingVector * ndVector::m_negOne);
		for (ndInt32 i = count - 1; i >= 0; --i)
		{
			contactOut[i].m_point = m_buffer[i];
			contactOut[i].m_normal = normal;
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
			contactOut[i].m_penetration = -penetration;
		}
	}

	ndAssert(m_separationDistance < ndFloat32(1.0e12f));
	return count;
}

ndInt32 ndContactSolver::CompoundContactsDiscrete()
{
	if (!m_instance1.GetShape()->GetAsShapeCompound())
	{
		ndAssert(m_instance0.GetShape()->GetAsShapeCompound());
		if (m_instance1.GetShape()->GetAsShapeConvex())
		{
			return CompoundToConvexContactsDiscrete();
		}
		else if (m_instance1.GetShape()->GetAsShapeStaticBVH())
		{
			return CompoundToShapeStaticBvhContactsDiscrete();
		}
		else if (m_instance1.GetShape()->GetAsShapeHeightfield())
		{
			return CompoundToStaticHeightfieldContactsDiscrete();
		}
		else if (m_instance1.GetShape()->GetAsShapeStaticProceduralMesh())
		{
			return CompoundToStaticProceduralMesh();
		}
		else
		{
			ndTrace(("Fix compound contact for pair: %s %s\n", m_instance0.GetShape()->ClassName(), m_instance1.GetShape()->ClassName()));
			ndAssert(0);
		}
	}
	else if (!m_instance0.GetShape()->GetAsShapeCompound())
	{
		ndAssert(m_instance1.GetShape()->GetAsShapeCompound());
		if (m_instance0.GetShape()->GetAsShapeConvex())
		{
			return ConvexToCompoundContactsDiscrete();
		}
		else
		{
			ndTrace(("Fix compound contact for pair: %s %s\n", m_instance0.GetShape()->ClassName(), m_instance1.GetShape()->ClassName()));
			ndAssert(0);
		}
	}
	else
	{
		ndAssert(m_instance0.GetShape()->GetAsShapeCompound() && m_instance1.GetShape()->GetAsShapeCompound());
		return CompoundToCompoundContactsDiscrete();
	}
	return 0;
}

ndInt32 ndContactSolver::ConvexToCompoundContactsDiscrete()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const convexBody = contactJoint->GetBody0();
	ndBodyKinematic* const compoundBody = contactJoint->GetBody1();
	ndShapeInstance* const convexInstance = &convexBody->GetCollisionShape();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();
	ndShapeCompound* const compoundShape = m_instance1.GetShape()->GetAsShapeCompound();
	ndAssert(compoundShape);

	ndVector size;
	ndVector origin;
	convexInstance->CalculateObb(origin, size);
	const ndMatrix& matrix0 = convexInstance->GetGlobalMatrix();
	const ndMatrix& matrix1 = compoundInstance->GetGlobalMatrix();
	ndBoxBoxDistance2 data(matrix0, matrix1);

	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	ndFloat32 stackDistance[D_SCENE_MAX_STACK_DEPTH];
	const ndShapeCompound::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	stackPool[0] = compoundShape->m_root;
	stackDistance[0] = data.CalculateDistance2(origin, size, compoundShape->m_root->m_origin, compoundShape->m_root->m_size);
	ndFloat32 closestDist = (stackDistance[0] > ndFloat32(0.0f)) ? stackDistance[0] : ndFloat32(1.0e10f);

	while (stack)
	{
		stack--;
		
		ndFloat32 dist2 = stackDistance[stack];
		if (dist2 > ndFloat32(0.0f))
		{
			closestDist = ndMin (closestDist, dist2);
			break;
		}

		const ndShapeCompound::ndNodeBase* const node = stackPool[stack];
		ndAssert(node);

		if (node->m_type == ndShapeCompound::m_leaf)
		{
			ndShapeInstance* const subShape = node->GetShape();
			if (subShape->GetCollisionMode())
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, convexInstance, subShape);
				if (processContacts)
				{
					ndShapeInstance childInstance(*subShape, subShape->GetShape());
					childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * matrix1;

					ndContactSolver contactSolver(*this, m_instance0, childInstance);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexContactsDiscrete();
					ndFloat32 dist = ndMax(contactSolver.m_separationDistance, ndFloat32(0.0f));
					closestDist = ndMin(closestDist, dist * dist);
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance0 = subShape;
						}
						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (stack < (ndInt32(sizeof(stackPool) / sizeof(stackPool[0])) - 2))
		{
			ndAssert(node->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const left = node->m_left;
				ndAssert(left);
				ndFloat32 subDist2 = data.CalculateDistance2(origin, size, left->m_origin, left->m_size);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = left;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}

			{
				const ndShapeCompound::ndNodeBase* const right = node->m_right;
				ndAssert(right);
				ndFloat32 subDist2 = data.CalculateDistance2(origin, size, right->m_origin, right->m_size);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = right;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}

	ndAssert(closestDist < ndFloat32(1.0e6f));
	m_separationDistance = ndSqrt (closestDist);
	return contactCount;
}

ndInt32 ndContactSolver::CompoundToConvexContactsDiscrete()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const convexBody = contactJoint->GetBody1();
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndShapeInstance* const convexInstance = &convexBody->GetCollisionShape();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();

	ndVector size;
	ndVector origin;
	convexInstance->CalculateObb(origin, size);
	const ndMatrix& matrix0 = compoundInstance->GetGlobalMatrix();
	const ndMatrix& matrix1 = convexInstance->GetGlobalMatrix();
	ndBoxBoxDistance2 data(matrix0, matrix1);

	ndShapeCompound* const compoundShape = m_instance0.GetShape()->GetAsShapeCompound();
	ndAssert(compoundShape);

	ndFloat32 stackDistance[D_SCENE_MAX_STACK_DEPTH];
	const ndShapeCompound::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	stackPool[0] = compoundShape->m_root;
	stackDistance[0] = data.CalculateDistance2(compoundShape->m_root->m_origin, compoundShape->m_root->m_size, origin, size);
	ndFloat32 closestDist = (stackDistance[0] > ndFloat32(0.0f)) ? stackDistance[0] : ndFloat32(1.0e10f);

	while (stack)
	{
		stack--;

		ndFloat32 dist2 = stackDistance[stack];
		if (dist2 > ndFloat32(0.0f))
		{
			closestDist = ndMin(closestDist, dist2);
			break;
		}
		const ndShapeCompound::ndNodeBase* const node = stackPool[stack];
		ndAssert(node);

		if (node->m_type == ndShapeCompound::m_leaf)
		{
			ndShapeInstance* const subShape = node->GetShape();
			if (subShape->GetCollisionMode())
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, convexInstance);
				if (processContacts)
				{
					ndShapeInstance childInstance(*subShape, subShape->GetShape());
					childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * matrix0;

					ndContactSolver contactSolver(*this, childInstance, m_instance1);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexContactsDiscrete();
					ndFloat32 dist = ndMax(contactSolver.m_separationDistance, ndFloat32(0.0f));
					closestDist = ndMin(closestDist, dist * dist);
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance0 = subShape;
						}
						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (stack < (ndInt32(sizeof(stackPool) / sizeof(stackPool[0])) - 2))
		{
			ndAssert(node->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const left = node->m_left;
				ndAssert(left);
				ndFloat32 subDist2 = data.CalculateDistance2(left->m_origin, left->m_size, origin, size);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = left;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}

			{
				const ndShapeCompound::ndNodeBase* const right = node->m_right;
				ndAssert(right);
				ndFloat32 subDist2 = data.CalculateDistance2(right->m_origin, right->m_size, origin, size);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = right;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}

	ndAssert(closestDist < ndFloat32(1.0e6f));
	m_separationDistance = ndSqrt(closestDist);
	return contactCount;
}

ndInt32 ndContactSolver::CompoundToCompoundContactsDiscrete()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody0 = contactJoint->GetBody0();
	ndBodyKinematic* const compoundBody1 = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance0 = &compoundBody0->GetCollisionShape();
	ndShapeInstance* const compoundInstance1 = &compoundBody1->GetCollisionShape();

	const ndMatrix& matrix0 = compoundInstance0->GetGlobalMatrix();
	const ndMatrix& matrix1 = compoundInstance1->GetGlobalMatrix();
	ndBoxBoxDistance2 data(matrix0, matrix1);

	ndShapeCompound* const compoundShape0 = m_instance0.GetShape()->GetAsShapeCompound();
	ndShapeCompound* const compoundShape1 = m_instance1.GetShape()->GetAsShapeCompound();
	ndAssert(compoundShape0);
	ndAssert(compoundShape1);

	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	ndStackEntry stackPool[2 * D_COMPOUND_STACK_DEPTH];

	stackPool[0].m_node0 = compoundShape0->m_root;
	stackPool[0].m_node1 = compoundShape1->m_root;
	stackPool[0].m_dist2 = data.CalculateDistance2(compoundShape0->m_root->m_origin, compoundShape0->m_root->m_size, compoundShape1->m_root->m_origin, compoundShape1->m_root->m_size);

	ndFloat32 closestDist = (stackPool[0].m_dist2 > ndFloat32(0.0f)) ? stackPool[0].m_dist2 : ndFloat32(1.0e10f);

	ndStackEntry callback;
	while (stack)
	{
		stack--;
		ndFloat32 dist2 = stackPool[stack].m_dist2;
		if (dist2 > ndFloat32(0.0f))
		{
			closestDist = ndMin(closestDist, dist2);
			break;
		}

		const ndShapeCompound::ndNodeBase* const node0 = stackPool[stack].m_node0;
		const ndShapeCompound::ndNodeBase* const node1 = stackPool[stack].m_node1;
		ndAssert(node0 && node1);

		if ((node0->m_type == ndShapeCompound::m_leaf) && (node1->m_type == ndShapeCompound::m_leaf))
		{
			ndShapeInstance* const subShape0 = node0->GetShape();
			ndShapeInstance* const subShape1 = node1->GetShape();

			if (ndInt8(subShape0->GetCollisionMode()) & ndInt8(subShape1->GetCollisionMode()))
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape0, subShape1);
				if (processContacts)
				{
					ndShapeInstance childInstance0(*subShape0, subShape0->GetShape());
					ndShapeInstance childInstance1(*subShape1, subShape1->GetShape());
					childInstance0.m_globalMatrix = childInstance0.GetLocalMatrix() * matrix0;
					childInstance1.m_globalMatrix = childInstance1.GetLocalMatrix() * matrix1;

					ndContactSolver contactSolver(*this, childInstance0, childInstance1);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexContactsDiscrete();
					ndFloat32 dist = ndMax(contactSolver.m_separationDistance, ndFloat32(0.0f));
					closestDist = ndMin(closestDist, dist * dist);
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance0 = subShape0;
							contacts[contactCount + i].m_shapeInstance1 = subShape1;
						}
						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (node0->m_type == ndShapeCompound::m_leaf)
		{
			ndAssert(node1->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1->m_left;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}

			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1->m_right;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}
		}
		else if (node1->m_type == ndShapeCompound::m_leaf)
		{
			ndAssert(node0->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0->m_left;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}

			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0->m_right;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}
		}
		else
		{
			ndAssert(node0->m_type == ndShapeCompound::m_node);
			ndAssert(node1->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0->m_left;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1->m_left;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}

			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0->m_left;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1->m_right;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}

			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0->m_right;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1->m_left;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}

			{
				const ndShapeCompound::ndNodeBase* const subNode0 = node0->m_right;
				const ndShapeCompound::ndNodeBase* const subNode1 = node1->m_right;
				callback.PushStackEntry(data, stack, stackPool, subNode0, subNode1);
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}

	ndAssert(closestDist < ndFloat32(1.0e6f));
	m_separationDistance = ndSqrt(closestDist);
	return contactCount;
}

ndInt32 ndContactSolver::CompoundToShapeStaticBvhContactsDiscrete()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndBodyKinematic* const bvhTreeBody = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();
	ndShapeInstance* const bvhTreeInstance = &bvhTreeBody->GetCollisionShape();
	ndShapeStatic_bvh* const bvhTreeCollision = m_instance1.GetShape()->GetAsShapeStaticBVH();
	ndShapeCompound* const compoundShape = m_instance0.GetShape()->GetAsShapeCompound();

	ndAssert(compoundShape);
	ndAssert(bvhTreeCollision);

	const ndMatrix& treeMatrix = bvhTreeInstance->GetGlobalMatrix();
	const ndMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	ndBoxBoxDistance2 data(compoundMatrix, treeMatrix);

	ndVector bvhp0;
	ndVector bvhp1;
	bvhTreeCollision->GetNodeAabb(bvhTreeCollision->GetRootNode(), bvhp0, bvhp1);
	const ndVector bvhSize((bvhp1 - bvhp0) * ndVector::m_half);
	const ndVector bvhOrigin((bvhp1 + bvhp0) * ndVector::m_half);
	const ndVector treeScale(bvhTreeInstance->GetScale());

	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	ndStackBvhStackEntry stackPool[2 * D_COMPOUND_STACK_DEPTH];

	stackPool[0].m_treeNodeIsLeaf = 0;
	stackPool[0].m_compoundNode = compoundShape->m_root;
	stackPool[0].m_collisionTreeNode = bvhTreeCollision->GetRootNode();
	stackPool[0].m_dist2 = data.CalculateDistance2(compoundShape->m_root->m_origin, compoundShape->m_root->m_size, bvhOrigin, bvhSize);

	ndStackBvhStackEntry callback;
	ndFloat32 closestDist = (stackPool[0].m_dist2 > ndFloat32(0.0f)) ? stackPool[0].m_dist2 : ndFloat32(1.0e10f);
	while (stack)
	{
		stack--;

		ndFloat32 dist2 = stackPool[stack].m_dist2;
		if (dist2 > ndFloat32(0.0f))
		{
			closestDist = ndMin(closestDist, dist2);
			break;
		}

		const ndShapeCompound::ndNodeBase* const compoundNode = stackPool[stack].m_compoundNode;
		const ndAabbPolygonSoup::ndNode* const collisionTreeNode = stackPool[stack].m_collisionTreeNode;
		const ndInt32 treeNodeIsLeaf = stackPool[stack].m_treeNodeIsLeaf;

		ndAssert(compoundNode && collisionTreeNode);

		if (treeNodeIsLeaf && (compoundNode->m_type == ndShapeCompound::m_leaf))
		{
			ndShapeInstance* const subShape = compoundNode->GetShape();
			if (subShape->GetCollisionMode())
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, bvhTreeInstance);
				if (processContacts)
				{
					ndShapeInstance childInstance(*subShape, subShape->GetShape());
					childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;

					ndContactSolver contactSolver(*this, childInstance, m_instance1);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexToSaticStaticBvhContactsNodeDescrete(collisionTreeNode);
					ndFloat32 dist = ndMax(contactSolver.m_separationDistance, ndFloat32(0.0f));
					closestDist = ndMin(closestDist, dist * dist);
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance0 = subShape;
						}
						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (compoundNode->m_type == ndShapeCompound::m_leaf)
		{
			ndAssert(!treeNodeIsLeaf);
			const ndAabbPolygonSoup::ndNode* const backNode = bvhTreeCollision->GetBackNode(collisionTreeNode);
			const ndAabbPolygonSoup::ndNode* const frontNode = bvhTreeCollision->GetFrontNode(collisionTreeNode);

			if (backNode && frontNode)
			{
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, backNode);
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, frontNode);
			}
			else if (backNode && !frontNode)
			{
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, backNode);
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 1, collisionTreeNode);
			}
			else if (!backNode && frontNode)
			{
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, frontNode);
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 1, collisionTreeNode);
			}
			else
			{
				callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 1, collisionTreeNode);
			}
		}
		else if (treeNodeIsLeaf)
		{
			ndAssert(compoundNode->m_type == ndShapeCompound::m_node);
			callback.PushStackEntry(data, stack, stackPool, compoundNode->m_left, bvhTreeCollision, 1, collisionTreeNode);
			callback.PushStackEntry(data, stack, stackPool, compoundNode->m_right, bvhTreeCollision, 1, collisionTreeNode);
		}
		else
		{
			ndAssert(compoundNode->m_type == ndShapeCompound::m_node);
			ndAssert(!treeNodeIsLeaf);

			ndVector p0;
			ndVector p1;
			bvhTreeCollision->GetNodeAabb(collisionTreeNode, p0, p1);
			p0 = p0 * treeScale;
			p1 = p1 * treeScale;
			ndVector size((p1 - p0) * ndVector::m_half);
			ndFloat32 area = size.DotProduct(size.ShiftTripleRight()).GetScalar();

			if (area > compoundNode->m_area)
			{
				const ndAabbPolygonSoup::ndNode* const backNode = bvhTreeCollision->GetBackNode(collisionTreeNode);
				const ndAabbPolygonSoup::ndNode* const frontNode = bvhTreeCollision->GetFrontNode(collisionTreeNode);
				if (backNode && frontNode)
				{
					callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, backNode);
					callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, frontNode);
				}
				else if (backNode && !frontNode)
				{
					callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, backNode);
					callback.PushStackEntry(data, stack, stackPool, compoundNode->m_left, bvhTreeCollision, 1, collisionTreeNode);
					callback.PushStackEntry(data, stack, stackPool, compoundNode->m_right, bvhTreeCollision, 1, collisionTreeNode);
				}
				else if (!backNode && frontNode)
				{
					callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 0, frontNode);
					callback.PushStackEntry(data, stack, stackPool, compoundNode->m_left, bvhTreeCollision, 1, collisionTreeNode);
					callback.PushStackEntry(data, stack, stackPool, compoundNode->m_right, bvhTreeCollision, 1, collisionTreeNode);
				}
				else
				{
					callback.PushStackEntry(data, stack, stackPool, compoundNode, bvhTreeCollision, 1, collisionTreeNode);
				}
			}
			else
			{
				ndAssert(!treeNodeIsLeaf);
				ndAssert(compoundNode->m_left);
				ndAssert(compoundNode->m_right);
				callback.PushStackEntry(data, stack, stackPool, compoundNode->m_left, bvhTreeCollision, 0, collisionTreeNode);
				callback.PushStackEntry(data, stack, stackPool, compoundNode->m_right, bvhTreeCollision, 0, collisionTreeNode);
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}

	ndAssert(closestDist < ndFloat32(1000.0f));
	m_separationDistance = closestDist;
	return contactCount;
}

ndInt32 ndContactSolver::CompoundToStaticHeightfieldContactsDiscrete()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndBodyKinematic* const heightfieldBody = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();
	ndShapeInstance* const heightfieldInstance = &heightfieldBody->GetCollisionShape();
	ndShapeCompound* const compoundShape = compoundInstance->GetShape()->GetAsShapeCompound();

	ndShapeCompound::ndNodeBase nodeProxi;
	nodeProxi.m_left = nullptr;
	nodeProxi.m_right = nullptr;
	const ndVector heighFieldScale(heightfieldInstance->GetScale());
	const ndVector heighFieldInvScale(heightfieldInstance->GetInvScale());
	
	const ndMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	const ndMatrix& heightfieldMatrix = heightfieldInstance->GetGlobalMatrix();
	ndBoxBoxDistance2 data(compoundMatrix, heightfieldMatrix);

	ndFloat32 stackDistance[D_SCENE_MAX_STACK_DEPTH];
	const ndShapeCompound::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	ndStackEntry callback;
	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	stackPool[0] = compoundShape->m_root;
	stackDistance[0] = callback.CalculateHeighfieldDist2(data, compoundShape->m_root, heightfieldInstance);
	ndFloat32 closestDist = (stackDistance[0] > ndFloat32(0.0f)) ? stackDistance[0] : ndFloat32(1.0e10f);

	while (stack)
	{
		stack--;

		ndFloat32 dist2 = stackDistance[stack];
		if (dist2 > ndFloat32(0.0f))
		{
			closestDist = ndMin(closestDist, dist2);
			break;
		}

		const ndShapeCompound::ndNodeBase* const node = stackPool[stack];
		ndAssert(node);

		if (node->m_type == ndShapeCompound::m_leaf)
		{
			ndShapeInstance* const subShape = node->GetShape();
			if (subShape->GetCollisionMode())
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, heightfieldInstance);
				if (processContacts)
				{
					ndShapeInstance childInstance(*subShape, subShape->GetShape());
					childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;

					ndContactSolver contactSolver(*this, childInstance, m_instance1);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexContactsDiscrete();
					//closestDist = ndMin(closestDist, contactSolver.m_separationDistance);
					ndFloat32 dist = ndMax(contactSolver.m_separationDistance, ndFloat32(0.0f));
					closestDist = ndMin(closestDist, dist * dist);
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance0 = subShape;
						}
						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (stack < (ndInt32(sizeof(stackPool) / sizeof(stackPool[0]))- 2))
		{
			ndAssert(node->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const left = node->m_left;
				ndAssert(left);
				ndFloat32 subDist2 = callback.CalculateHeighfieldDist2(data, left, heightfieldInstance);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = left;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}

			{
				const ndShapeCompound::ndNodeBase* const right = node->m_right;
				ndAssert(right);
				ndFloat32 subDist2 = callback.CalculateHeighfieldDist2(data, right, heightfieldInstance);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = right;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	ndAssert(closestDist < ndFloat32(1.0e6f));
	m_separationDistance = ndSqrt(closestDist);
	return contactCount;
}

ndInt32 ndContactSolver::CompoundToStaticProceduralMesh()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndBodyKinematic* const ProceduralBody = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();
	ndShapeInstance* const ProceduralInstance = &ProceduralBody->GetCollisionShape();
	ndShapeCompound* const compoundShape = compoundInstance->GetShape()->GetAsShapeCompound();

	ndShapeCompound::ndNodeBase nodeProxi;
	nodeProxi.m_left = nullptr;
	nodeProxi.m_right = nullptr;
	const ndVector ProceduralScale(ProceduralInstance->GetScale());
	const ndVector ProceduralInvScale(ProceduralInstance->GetInvScale());

	const ndMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	const ndMatrix& ProceduralMatrix = ProceduralInstance->GetGlobalMatrix();
	ndBoxBoxDistance2 data(compoundMatrix, ProceduralMatrix);

	ndFloat32 stackDistance[D_SCENE_MAX_STACK_DEPTH];
	const ndShapeCompound::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	ndStackEntry callback;
	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	stackPool[0] = compoundShape->m_root;
	stackDistance[0] = callback.CalculateProceduralDist2(data, compoundShape->m_root, ProceduralInstance);
	ndFloat32 closestDist = (stackDistance[0] > ndFloat32(0.0f)) ? stackDistance[0] : ndFloat32(1.0e10f);

	while (stack)
	{
		stack--;

		ndFloat32 dist2 = stackDistance[stack];
		if (dist2 > ndFloat32(0.0f))
		{
			closestDist = ndMin(closestDist, dist2);
			break;
		}

		const ndShapeCompound::ndNodeBase* const node = stackPool[stack];
		ndAssert(node);

		if (node->m_type == ndShapeCompound::m_leaf)
		{
			ndShapeInstance* const subShape = node->GetShape();
			if (subShape->GetCollisionMode())
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, ProceduralInstance);
				if (processContacts)
				{
					ndShapeInstance childInstance(*subShape, subShape->GetShape());
					childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;

					ndContactSolver contactSolver(*this, childInstance, m_instance1);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexContactsDiscrete();
					ndFloat32 dist = ndMax(contactSolver.m_separationDistance, ndFloat32(0.0f));
					closestDist = ndMin(closestDist, dist * dist);
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance0 = subShape;
						}
						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (stack < (ndInt32(sizeof(stackPool) / sizeof(stackPool[0])) - 2))
		{
			ndAssert(node->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const left = node->m_left;
				ndAssert(left);
				ndFloat32 subDist2 = callback.CalculateProceduralDist2(data, left, ProceduralInstance);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = left;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}

			{
				const ndShapeCompound::ndNodeBase* const right = node->m_right;
				ndAssert(right);
				ndFloat32 subDist2 = callback.CalculateProceduralDist2(data, right, ProceduralInstance);
				ndInt32 j = stack;
				for (; j && (subDist2 > stackDistance[j - 1]); --j)
				{
					stackPool[j] = stackPool[j - 1];
					stackDistance[j] = stackDistance[j - 1];
				}
				stackPool[j] = right;
				stackDistance[j] = subDist2;
				stack++;
				ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	ndAssert(closestDist < ndFloat32(1.0e6f));
	m_separationDistance = ndSqrt(closestDist);
	return contactCount;
}

//*************************************************************
// calculate proper separation distance for continue collision.
//*************************************************************
ndInt32 ndContactSolver::CalculateContactsContinue()
{
	ndInt32 count = 0;
	if (m_instance0.GetShape()->GetAsShapeCompound() || m_instance1.GetShape()->GetAsShapeCompound())
	{
		count = CompoundContactsContinue();
	}
	else if (m_instance0.GetShape()->GetAsShapeConvex())
	{
		count = ConvexContactsContinue();
	}
	else
	{
		//ndTrace(("!!!!!Fix compound contact\n"));
		ndAssert(0);
	}

	m_contact->m_timeOfImpact = m_timestep;
	m_contact->m_separatingVector = m_separatingVector;
	m_contact->m_separationDistance = m_separationDistance;
	return count;
}

ndInt32 ndContactSolver::CompoundContactsContinue()
{
	if (!m_instance1.GetShape()->GetAsShapeCompound())
	{
		ndAssert(0);
		//	ndAssert(m_instance0.GetShape()->GetAsShapeCompound());
		//	if (m_instance1.GetShape()->GetAsShapeConvex())
		//	{
		//		return CompoundToConvexContactsDiscrete();
		//	}
		//	else if (m_instance1.GetShape()->GetAsShapeStaticBVH())
		//	{
		//		return CompoundToShapeStaticBvhContactsDiscrete();
		//	}
		//	else if (m_instance1.GetShape()->GetAsShapeHeightfield())
		//	{
		//		return CompoundToStaticHeightfieldContactsDiscrete();
		//	}
		//	else
		//	{
		//		ndAssert(0);
		//	}
	}
	else if (!m_instance0.GetShape()->GetAsShapeCompound())
	{
		ndAssert(m_instance1.GetShape()->GetAsShapeCompound());
		if (m_instance0.GetShape()->GetAsShapeConvex())
		{
			return ConvexToCompoundContactsContinue();
		}
		else
		{
			ndAssert(0);
		}
	}
	else
	{
		ndAssert(0);
		//	ndAssert(m_instance0.GetShape()->GetAsShapeCompound() && m_instance1.GetShape()->GetAsShapeCompound());
		//	return CompoundToCompoundContactsDiscrete();
	}
	return 0;
}

ndInt32 ndContactSolver::ConvexContactsContinue()
{
	const ndVector origin0(m_instance0.m_globalMatrix.m_posit);
	const ndVector origin1(m_instance1.m_globalMatrix.m_posit);
	m_instance0.m_globalMatrix.m_posit = ndVector::m_wOne;
	m_instance1.m_globalMatrix.m_posit -= (origin0 & ndVector::m_triplexMask);

	// handle rare case of two shapes located exactly at the same origin
	const ndVector error(m_instance1.m_globalMatrix.m_posit - m_instance0.m_globalMatrix.m_posit);
	if (error.DotProduct(error).GetScalar() < ndFloat32(1.0e-6f))
	{
		m_instance1.m_globalMatrix.m_posit.m_y += ndFloat32(1.0e-3f);
	}

	ndInt32 count = 0;
	if (m_instance1.GetShape()->GetAsShapeConvex())
	{
		ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
		count = ConvexToConvexContactsContinue();
	}
	else
	{
		ndShapeStaticMesh* const meshShape = m_instance1.GetShape()->GetAsShapeStaticMesh();
		if (meshShape)
		{
			count = ConvexToStaticMeshContactsContinue();
		}
		else
		{
			ndAssert(0);
			count = 0;
		}
	}

	if (m_pruneContacts && (count > 1))
	{
		count = PruneContacts(count, 16);
	}

	const ndVector offset(origin0 & ndVector::m_triplexMask);
	m_closestPoint0 += offset;
	m_closestPoint1 += offset;

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (ndInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_point += offset;
		}
	}

	m_instance0.m_globalMatrix.m_posit = origin0;
	m_instance1.m_globalMatrix.m_posit = origin1;
	return count;
}

ndInt32 ndContactSolver::ConvexToConvexContactsContinue()
{
	ndAssert(m_instance0.GetConvexVertexCount() && m_instance1.GetConvexVertexCount());
	ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(m_instance1.GetShape()->GetAsShapeConvex());
	ndAssert(!m_instance0.GetShape()->GetAsShapeNull());
	ndAssert(!m_instance1.GetShape()->GetAsShapeNull());

	const ndVector savedPosition1(m_instance1.m_globalMatrix.m_posit);
	const ndBodyKinematic* const body0 = m_contact->m_body0;
	const ndBodyKinematic* const body1 = m_contact->m_body1;
	const ndVector relVeloc(body0->GetVelocity() - body1->GetVelocity());
	ndVector closestPoint1;

	ndInt32 iter = 0;
	ndInt32 count = 0;
	ndFloat32 tacc = ndFloat32(0.0f);
	ndFloat32 timestep = m_timestep;
	do
	{
		bool state = CalculateClosestPoints();
		if (!state)
		{
			break;
		}
		ndAssert(m_separatingVector.m_w == ndFloat32(0.0f));
		ndFloat32 den = m_separatingVector.DotProduct(relVeloc).GetScalar();
		if (den <= ndFloat32(1.0e-6f))
		{
			// bodies are residing from each other, even if they are touching 
			// they are not considered to be colliding because the motion will 
			// move them apart get the closet point and the normal at contact point
			m_timestep = ndFloat32(1.0e10f);
			m_separatingVector = m_separatingVector * ndVector::m_negOne;
			break;
		}

		ndFloat32 num = m_separatingVector.DotProduct(m_closestPoint1 - m_closestPoint0).GetScalar() - m_skinMargin;
		if ((num <= ndFloat32(1.0e-5f)) && (tacc <= timestep))
		{
			// bodies collide at time tacc, but we do not set it yet
			ndVector step(relVeloc.Scale(tacc));
			m_timestep = tacc;
			closestPoint1 = m_closestPoint1 + step;
			m_separatingVector = m_separatingVector * ndVector::m_negOne;
			ndFloat32 penetration = ndMax(num * ndFloat32(-1.0f) + D_PENETRATION_TOL, ndFloat32(0.0f));
			if (m_contactBuffer && !m_intersectionTestOnly)
			{
				if (ndInt8(m_instance0.GetCollisionMode()) & ndInt8(m_instance1.GetCollisionMode()))
				{
					count = CalculateContacts(m_closestPoint0, m_closestPoint1, m_separatingVector);
					if (count)
					{
						count = ndMin(m_maxCount, count);
						ndContactPoint* const contactOut = m_contactBuffer;

						for (ndInt32 i = 0; i < count; ++i)
						{
							contactOut[i].m_point = m_hullDiff[i] + step;
							contactOut[i].m_normal = m_separatingVector;
							contactOut[i].m_penetration = penetration;
						}
					}
				}
			}
			break;
		}

		ndAssert(den > ndFloat32(0.0f));
		ndFloat32 dt = num / den;
		if ((tacc + dt) >= timestep)
		{
			// object do not collide on this timestep
			m_timestep = tacc + dt;
			closestPoint1 = m_closestPoint1;
			m_separatingVector = m_separatingVector * ndVector::m_negOne;
			break;
		}

		tacc += dt;
		ndVector step(relVeloc.Scale(dt));
		TranslateSimplex(step);

		iter++;
	} while (iter < D_SEPARATION_PLANES_ITERATIONS);

	m_closestPoint1 = closestPoint1;
	m_instance1.m_globalMatrix.m_posit = savedPosition1;
	m_separationDistance = m_separatingVector.DotProduct(m_closestPoint0 - m_closestPoint1).GetScalar();
	return count;
}

ndInt32 ndContactSolver::ConvexToCompoundContactsContinue()
{
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const convexBody = contactJoint->GetBody0();
	ndBodyKinematic* const compoundBody = contactJoint->GetBody1();
	ndShapeInstance* const convexInstance = &convexBody->GetCollisionShape();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();

	const ndMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	const ndMatrix matrix(convexInstance->GetGlobalMatrix() * compoundMatrix.OrthoInverse());

	ndShapeCompound* const compoundShape = m_instance1.GetShape()->GetAsShapeCompound();
	ndAssert(compoundShape);

	ndVector boxP0;
	ndVector boxP1;
	convexInstance->CalculateAabb(matrix, boxP0, boxP1);
	const ndVector relVeloc(matrix.UnrotateVector(convexBody->GetVelocity() - compoundBody->GetVelocity()));
	ndFastRay ray(ndVector::m_zero, relVeloc);

	const ndVector rootMinBox(compoundShape->m_root->m_p0 - boxP1);
	const ndVector rootMaxBox(compoundShape->m_root->m_p1 - boxP0);

	ndVector closestPoint0(ndVector::m_zero);
	ndVector closestPoint1(ndVector::m_zero);
	ndVector separatingVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));

	ndFloat32 impactTime[D_SCENE_MAX_STACK_DEPTH];
	const ndShapeCompound::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	ndInt32 stack = 1;
	ndInt32 contactCount = 0;
	ndFloat32 minTimeStep = m_timestep;

	stackPool[0] = compoundShape->m_root;
	impactTime[0] = ray.BoxIntersect(rootMinBox, rootMaxBox);
	while (stack)
	{
		stack--;
		ndFloat32 dist = impactTime[stack];
		if (dist > ndFloat32 (1.0f))
		{
			break;
		}

		ndAssert(stackPool[stack]);
		const ndShapeCompound::ndNodeBase* const node = stackPool[stack];
		if (node->m_type == ndShapeCompound::m_leaf)
		{
			ndShapeInstance* const subShape = node->GetShape();
			if (subShape->GetCollisionMode())
			{
				bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, convexInstance, subShape);
				if (processContacts)
				{
					ndShapeInstance childInstance(*subShape, subShape->GetShape());
					childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;

					ndContactSolver contactSolver(*this, m_instance0, childInstance);
					contactSolver.m_pruneContacts = 0;
					contactSolver.m_timestep = minTimeStep;
					contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
					contactSolver.m_contactBuffer += contactCount;

					ndInt32 count = contactSolver.ConvexContactsContinue();
					if (!m_intersectionTestOnly)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							contacts[contactCount + i].m_shapeInstance1 = subShape;
						}

						ndFloat32 error = contactSolver.m_timestep - minTimeStep;
						if (error < ndFloat32(-1.0e-3f))
						{
							if (contactCount)
							{
								for (ndInt32 i = 0; i < count; ++i)
								{
									contacts[i] = contacts[contactCount + i];
								}
								contactCount = 0;
							}
						}

						if (contactSolver.m_timestep < minTimeStep)
						{
							minTimeStep = contactSolver.m_timestep;
							closestPoint0 = contactSolver.m_closestPoint0;
							closestPoint1 = contactSolver.m_closestPoint1;
							separatingVector = contactSolver.m_separatingVector;
						}

						contactCount += count;
						if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
						{
							contactCount = PruneContacts(contactCount, 16);
						}
					}
				}
			}
		}
		else if (stack < (ndInt32(sizeof(stackPool) / sizeof(stackPool[0]))- 2))
		{
			ndAssert(node->m_type == ndShapeCompound::m_node);
			{
				const ndShapeCompound::ndNodeBase* const left = node->m_left;
				ndAssert(left);
				const ndVector minBox(left->m_p0 - boxP1);
				const ndVector maxBox(left->m_p1 - boxP0);
				ndFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
				if (dist1 <= ndFloat32 (1.0f))
				{
					ndInt32 j = stack;
					for (; j && (dist1 > impactTime[j - 1]); --j)
					{
						stackPool[j] = stackPool[j - 1];
						impactTime[j] = impactTime[j - 1];
					}
					stackPool[j] = left;
					impactTime[j] = dist1;
					stack++;
					ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
				}
			}

			{
				const ndShapeCompound::ndNodeBase* const right = node->m_right;
				ndAssert(right);
				const ndVector minBox(right->m_p0 - boxP1);
				const ndVector maxBox = right->m_p1 - boxP0;
				ndFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
				if (dist1 <= ndFloat32(1.0f))
				{
					ndInt32 j = stack;
					for (; j && (dist1 > impactTime[j - 1]); --j)
					{
						stackPool[j] = stackPool[j - 1];
						impactTime[j] = impactTime[j - 1];
					}
					stackPool[j] = right;
					impactTime[j] = dist1;
					stack++;
					ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0])));
				}
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}

	if (minTimeStep < m_timestep)
	{
		m_timestep = minTimeStep;
		m_closestPoint0 = closestPoint0;
		m_closestPoint1 = closestPoint1;
		m_separatingVector = separatingVector;
		m_separationDistance = m_separatingVector.DotProduct(m_closestPoint0 - m_closestPoint1).GetScalar();
	}
	else
	{
		m_separationDistance = ndSqrt(relVeloc.DotProduct(relVeloc).GetScalar()) * minTimeStep;
	}
	return contactCount;
}

ndInt32 ndContactSolver::CalculatePolySoupToHullContactsContinue(ndPolygonMeshDesc& data)
{
	ndShapeConvexPolygon polygon;
	ndShapeInstance polySoupInstance(m_instance1);

	ndAssert(data.m_staticMeshQuery->m_faceIndexCount.GetCount());
	m_instance1.m_shape->Release();
	m_instance1.m_shape = polygon.AddRef();
	m_instance1.SetScale(ndVector::m_one);
	m_instance1.m_localMatrix = ndGetIdentityMatrix();
	m_instance1.m_globalMatrix = ndGetIdentityMatrix();

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = ndInt32(data.m_vertexStrideInBytes / sizeof(ndFloat32));

	ndInt32 count = 0;
	ndInt32 maxContacts = m_maxCount;
	ndInt32 countleft = maxContacts;
	ndInt32 maxReduceLimit = maxContacts - 16;

	const ndVector& polygonInstanceScale = polySoupInstance.GetScale();
	const ndMatrix& polySoupGlobalMatrix = polySoupInstance.GetGlobalMatrix();
	const ndMatrix& polySoupGlobalalignmentMatrix = polySoupInstance.GetAlignmentMatrix();

	ndMatrix polySoupScaledMatrix(
		polySoupGlobalalignmentMatrix[0] * polygonInstanceScale,
		polySoupGlobalalignmentMatrix[1] * polygonInstanceScale,
		polySoupGlobalalignmentMatrix[2] * polygonInstanceScale,
		polySoupGlobalalignmentMatrix[3]);
	polySoupScaledMatrix = polySoupScaledMatrix * polySoupGlobalMatrix;

	const ndInt32 stride = polygon.m_stride;
	const ndFloat32* const vertex = polygon.m_vertex;
	ndAssert(m_instance1.m_scaleType == ndShapeInstance::m_unit);

	ndContactPoint* const contactOut = m_contactBuffer;

	ndPolygonMeshDesc::ndStaticMeshFaceQuery& query = *data.m_staticMeshQuery;
	ndInt32* const indexArray = (ndInt32*)&query.m_faceVertexIndex[0];
	data.SortFaceArray();

	ndVector separatingVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector closestPoint0(ndVector::m_zero);
	ndVector closestPoint1(ndVector::m_zero);
	ndFloat32 minTimeStep = m_timestep;
	ndFloat32 savedTimestep = m_timestep;
	ndFloat32 epsilon = ndFloat32(-1.0e-3f) * m_timestep;

	for (ndInt32 i = 0; (i < query.m_faceIndexCount.GetCount()) && (m_timestep >= (query.m_hitDistance[i] * savedTimestep)); ++i)
	{
		ndInt32 address = query.m_faceIndexStart[i];
		const ndInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = query.m_faceIndexCount[i];
		polygon.m_paddedCount = polygon.m_count;
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray(localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId(localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize(localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex(localIndexArray, polygon.m_count);
		polygon.m_normal = polygon.CalculateGlobalNormal(&polySoupInstance, ndVector(&vertex[polygon.m_faceNormalIndex * stride]) & ndVector::m_triplexMask);
		ndAssert(polygon.m_normal.m_w == ndFloat32(0.0f));
		for (ndInt32 j = 0; j < polygon.m_count; ++j)
		{
			polygon.m_localPoly[j] = polySoupScaledMatrix.TransformVector(ndVector(&vertex[localIndexArray[j] * stride]) & ndVector::m_triplexMask);
		}

		m_vertexIndex = 0;
		m_maxCount = countleft;
		m_contactBuffer = &contactOut[count];
		ndInt32 count1 = polygon.CalculateContactToConvexHullContinue(&polySoupInstance, *this);

		if (count1 > 0)
		{
			ndFloat32 error = m_timestep - minTimeStep;
			if (error < epsilon)
			{
				count = 0;
				countleft = maxContacts;
				for (ndInt32 j = 0; j < count1; ++j)
				{
					contactOut[j] = m_contactBuffer[j];
				}
			}
			count += count1;
			countleft -= count1;
			ndAssert(countleft >= 0);
			if (count >= maxReduceLimit)
			{
				m_contactBuffer = contactOut;
				count = PruneContacts(count, 16);
				countleft = maxContacts - count;
				ndAssert(countleft >= 0);
			}
		}

		if (m_timestep < minTimeStep)
		{
			minTimeStep = m_timestep;
			closestPoint0 = m_closestPoint0;
			closestPoint1 = m_closestPoint1;
			separatingVector = m_separatingVector;
		}
	}

	m_contactBuffer = contactOut;
	m_instance1 = polySoupInstance;
	if (minTimeStep < savedTimestep)
	{
		m_timestep = minTimeStep;
		m_closestPoint0 = closestPoint0;
		m_closestPoint1 = closestPoint1;
		m_separatingVector = separatingVector;
		m_separationDistance = m_separatingVector.DotProduct(m_closestPoint0 - m_closestPoint1).GetScalar();
	}

	return count;
}

ndInt32 ndContactSolver::ConvexToStaticMeshContactsContinue()
{
	ndAssert(m_instance0.GetConvexVertexCount());
	ndAssert(!m_instance0.GetShape()->GetAsShapeNull());
	ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(m_instance1.GetShape()->GetAsShapeStaticMesh());

	ndInt32 count = 0;
	//ndPolygonMeshLocalDesc data(*this, true);
	ndPolygonMeshDesc data(*this, true);

	ndVector relVeloc(m_contact->m_body0->GetVelocity() - m_contact->m_body1->GetVelocity());
	ndFloat32 baseLinearSpeed = ndSqrt(relVeloc.DotProduct(relVeloc).GetScalar());
	if (baseLinearSpeed > ndFloat32(1.0e-6f))
	{
		//const ndFloat32 minRadius = m_instance0.GetBoxMinRadius();
		//const ndFloat32 maxRadius = m_instance0.GetBoxMaxRadius();
		//ndFloat32 maxAngularSpeed = ndSqrt(hullOmega.DotProduct(hullOmega).GetScalar());
		//ndFloat32 maxAngularSpeed = ndFloat32 (0.0f);
		//ndFloat32 angularSpeedBound = maxAngularSpeed * (maxRadius - minRadius);
		//ndFloat32 upperBoundSpeed = baseLinearSpeed + dgSqrt(angularSpeedBound);
		ndFloat32 upperBoundSpeed = baseLinearSpeed;
		ndVector upperBoundVeloc(relVeloc.Scale(m_timestep * upperBoundSpeed / baseLinearSpeed));
		data.SetDistanceTravel(upperBoundVeloc);
	}

	ndShapeStaticMesh* const polysoup = m_instance1.GetShape()->GetAsShapeStaticMesh();
	polysoup->GetCollidingFaces(&data);

	//if (data.m_faceCount)
	if (data.m_staticMeshQuery->m_faceIndexCount.GetCount())
	{
		m_separationDistance = ndSqrt(relVeloc.DotProduct(relVeloc).GetScalar()) * m_timestep;
		count = CalculatePolySoupToHullContactsContinue(data);
	}

	ndBodyKinematic* const body0 = m_contact->GetBody0();
	ndBodyKinematic* const body1 = m_contact->GetBody1();
	ndShapeInstance* const instance0 = &body0->GetCollisionShape();
	ndShapeInstance* const instance1 = &body1->GetCollisionShape();

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (ndInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
		}
	}

	return count;
}

void ndContactSolver::CalculateContacts(
	const ndShapeInstance* const instanceA, const ndMatrix& matrixA, const ndVector& velocA,
	const ndShapeInstance* const instanceB, const ndMatrix& matrixB, const ndVector& velocB,
	ndFixSizeArray<ndContactPoint, 16>& contactOut, ndContactNotify* const notification)
{
	ndContact contact;
	ndBodyKinematic bodyA;
	ndBodyKinematic bodyB;
	ndContactPoint contactBuffer[D_MAX_CONTATCS];

	ndShape* const shapeA = (ndShape*)(instanceA->GetShape());
	ndShape* const shapeB = (ndShape*)(instanceB->GetShape());

	m_instance0.SetShape(shapeA);
	m_instance0.SetLocalMatrix(instanceA->GetLocalMatrix());
	//Setting the global matrix before setting the collision shape of the body
	m_instance0.SetGlobalMatrix(m_instance0.GetLocalMatrix() * matrixA);

	m_instance1.SetShape(shapeB);
	m_instance1.SetLocalMatrix(instanceB->GetLocalMatrix());
	// Setting the global matrix before setting the collision shape of the body
	m_instance1.SetGlobalMatrix(m_instance1.GetLocalMatrix() * matrixB);

	bodyA.SetCollisionShape(m_instance0);
	bodyB.SetCollisionShape(m_instance1);

	bodyA.SetMatrix(matrixA);
	bodyB.SetMatrix(matrixB);
	bodyA.SetVelocity(velocA);
	bodyB.SetVelocity(velocB);

	if (!shapeA->GetAsShapeStaticMesh())
	{
		bodyA.SetMassMatrix(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f));
	}
	if (!shapeB->GetAsShapeStaticMesh())
	{
		bodyB.SetMassMatrix(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f));
	}
	contact.SetBodies(&bodyA, &bodyB);

	m_closestPoint0 = ndVector::m_zero;
	m_closestPoint1 = ndVector::m_zero;
	m_separatingVector = ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	m_contact = &contact;
	m_freeFace = nullptr;
	m_notification = notification;
	m_contactBuffer = contactBuffer;
	m_timestep = ndFloat32(1.0f);
	m_skinMargin = ndFloat32(0.0f);
	m_separationDistance = ndFloat32(1.0e10f);
	m_maxCount = D_MAX_CONTATCS;
	m_vertexIndex = 0;
	m_pruneContacts = 1;
	m_intersectionTestOnly = 0;

	const ndInt32 count = ndMin(CalculateContactsDiscrete(), contactOut.GetCapacity());
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndContactPoint& contactPoint = contactBuffer[i];
		contactPoint.m_body0 = nullptr;
		contactPoint.m_body1 = nullptr;
		contactPoint.m_shapeInstance0 = instanceA;
		contactPoint.m_shapeInstance1 = instanceB;
		contactOut.PushBack(contactPoint);
	}
}

ndInt32 ndContactSolver::CalculatePolySoupToHullContactsDescrete(ndPolygonMeshDesc& data)
{
	ndShapeConvexPolygon polygon;
	ndShapeInstance polySoupInstance(m_instance1);
	ndPolygonMeshDesc::ndStaticMeshFaceQuery& query = *data.m_staticMeshQuery;

	ndAssert(query.m_faceIndexCount.GetCount());
	m_instance1.m_shape = &polygon;
	m_instance1.SetScale(ndVector::m_one);
	m_instance1.m_localMatrix = ndGetIdentityMatrix();
	m_instance1.m_globalMatrix = ndGetIdentityMatrix();

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = ndInt32(data.m_vertexStrideInBytes / sizeof(ndFloat32));

	ndInt32 count = 0;
	ndInt32 maxContacts = m_maxCount;
	ndInt32 countleft = maxContacts;
	ndInt32 maxReduceLimit = maxContacts - 16;

	const ndVector& polygonInstanceScale = polySoupInstance.GetScale();
	const ndMatrix& polySoupGlobalMatrix = polySoupInstance.GetGlobalMatrix();
	const ndMatrix& polySoupGlobalalignmentMatrix = polySoupInstance.GetAlignmentMatrix();

	ndMatrix polySoupScaledMatrix(
		polySoupGlobalalignmentMatrix[0] * polygonInstanceScale,
		polySoupGlobalalignmentMatrix[1] * polygonInstanceScale,
		polySoupGlobalalignmentMatrix[2] * polygonInstanceScale,
		polySoupGlobalalignmentMatrix[3]);
	polySoupScaledMatrix = polySoupScaledMatrix * polySoupGlobalMatrix;

	ndAssert(m_contact);
	ndVector separatingVector(m_instance0.m_globalMatrix.m_up);

	const ndInt32 stride = polygon.m_stride;
	const ndFloat32* const vertex = polygon.m_vertex;
	ndAssert(m_instance1.m_scaleType == ndShapeInstance::m_unit);
	ndFloat32 closestDist = ndFloat32(1.0e10f);
	ndContactPoint* const contactOut = m_contactBuffer;
	ndContact* const contactJoint = m_contact;
	const ndInt32* const indexArray = &query.m_faceVertexIndex[0];

	data.SortFaceArray();
	for (ndInt32 i = ndInt32(query.m_faceIndexCount.GetCount()) - 1; (i >= 0) && (count < 32); --i)
	{
		ndInt32 address = query.m_faceIndexStart[i];
		const ndInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = query.m_faceIndexCount[i];
		polygon.m_paddedCount = polygon.m_count;
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray(localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId(localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize(localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex(localIndexArray, polygon.m_count);
		polygon.m_normal = polygon.CalculateGlobalNormal(&polySoupInstance, ndVector(&vertex[polygon.m_faceNormalIndex * stride]) & ndVector::m_triplexMask);
		ndAssert(polygon.m_normal.m_w == ndFloat32(0.0f));
		for (ndInt32 j = 0; j < polygon.m_count; ++j)
		{
			polygon.m_localPoly[j] = polySoupScaledMatrix.TransformVector(ndVector(&vertex[localIndexArray[j] * stride]) & ndVector::m_triplexMask);
		}

		contactJoint->m_separatingVector = separatingVector;
		m_maxCount = countleft;
		m_vertexIndex = 0;
		m_contactBuffer = &contactOut[count];

		ndInt32 count1 = polygon.CalculateContactToConvexHullDescrete(&polySoupInstance, *this);
		closestDist = ndMin(closestDist, m_separationDistance);

		if (count1 > 0)
		{
			if (!m_intersectionTestOnly)
			{
				count += count1;
				countleft -= count1;
				ndAssert(countleft >= 0);
				if (count >= maxReduceLimit)
				{
					ndAssert(0);
					//count = PruneContacts(count, contactOut, ndFloat32(1.0e-2f), 16);
					//countleft = maxContacts - count;
					//ndAssert(countleft >= 0);
					//proxy.m_maxContacts = countleft;
				}
			}
			else
			{
				count = 1;
				break;
			}
		}
	}

	m_contactBuffer = contactOut;
	ndAssert(!count || (closestDist < ndFloat32(1000.0f)));
	m_separationDistance = closestDist;
	m_instance1.m_shape = polySoupInstance.m_shape;
	m_instance1 = polySoupInstance;

	return count;
}

ndInt32 ndContactSolver::ConvexToStaticMeshContactsDiscrete()
{
	D_TRACKTIME();
	ndAssert(m_instance0.GetConvexVertexCount());
	ndAssert(!m_instance0.GetShape()->GetAsShapeNull());
	ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(m_instance1.GetShape()->GetAsShapeStaticMesh());

	ndInt32 count = 0;
	ndPolygonMeshDesc data(*this, false);
	ndShapeStaticMesh* const polysoup = m_instance1.GetShape()->GetAsShapeStaticMesh();
	ndAssert(polysoup);
	polysoup->GetCollidingFaces(&data);
	if (data.m_staticMeshQuery->m_faceIndexCount.GetCount())
	{
		count = CalculatePolySoupToHullContactsDescrete(data);
	}

	ndBodyKinematic* const body0 = m_contact->GetBody0();
	ndBodyKinematic* const body1 = m_contact->GetBody1();
	ndShapeInstance* const instance0 = &body0->GetCollisionShape();
	ndShapeInstance* const instance1 = &body1->GetCollisionShape();

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (ndInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
		}
	}

	ndAssert(!count || (m_separationDistance < ndFloat32(1.0e6f)));
	return count;
}

ndInt32 ndContactSolver::ConvexToSaticStaticBvhContactsNodeDescrete(const ndAabbPolygonSoup::ndNode* const node)
{
	ndVector origin0(m_instance0.m_globalMatrix.m_posit);
	ndVector origin1(m_instance1.m_globalMatrix.m_posit);
	m_instance0.m_globalMatrix.m_posit = ndVector::m_wOne;
	m_instance1.m_globalMatrix.m_posit -= (origin0 & ndVector::m_triplexMask);

	ndAssert(m_instance1.GetShape()->GetAsShapeStaticBVH());

	ndShapeStatic_bvh* const polysoup = m_instance1.GetShape()->GetAsShapeStaticBVH();
	ndAssert(polysoup);
	ndPolygonMeshDesc data(*this, false);
	data.m_vertex = polysoup->GetLocalVertexPool();
	data.m_vertexStrideInBytes = polysoup->GetStrideInBytes();
	polysoup->ForThisSector(node, data, data.m_boxDistanceTravelInMeshSpace, data.m_maxT, polysoup->GetPolygon, &data);

	ndInt32 count = 0;
	if (data.m_staticMeshQuery->m_faceIndexCount.GetCount())
	{
		count = CalculatePolySoupToHullContactsDescrete(data);
	}

	ndBodyKinematic* const body0 = m_contact->GetBody0();
	ndBodyKinematic* const body1 = m_contact->GetBody1();
	ndShapeInstance* const instance0 = &body0->GetCollisionShape();
	ndShapeInstance* const instance1 = &body1->GetCollisionShape();

	//if (!m_intersectionTestOnly)
	//{
	//	ndContactPoint* const contactOut = m_contactBuffer;
	//	for (dInt32 i = count - 1; i >= 0; i--)
	//	{
	//		contactOut[i].m_body0 = body0;
	//		contactOut[i].m_body1 = body1;
	//		contactOut[i].m_shapeInstance0 = instance0;
	//		contactOut[i].m_shapeInstance1 = instance1;
	//	}
	//}

	ndVector offset = (origin0 & ndVector::m_triplexMask);
	m_closestPoint0 += offset;
	m_closestPoint1 += offset;

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (ndInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_point += offset;
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
		}
	}

	m_instance0.m_globalMatrix.m_posit = origin0;
	m_instance1.m_globalMatrix.m_posit = origin1;
	return count;
}

ndInt32 ndContactSolver::CalculateContacts(const ndVector& point0, const ndVector& point1, const ndVector& normal)
{
	ndAssert(m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(m_instance1.GetShape()->GetAsShapeConvex());

	ndInt32 count = 0;
	ndInt32 count1 = 0;
	const ndInt32 baseCount = 16;
	ndVector* const contactsOut = &m_buffer[0];
	ndVector* const shape1 = &contactsOut[baseCount];
	ndAssert(normal.m_w == ndFloat32(0.0f));

	ndVector origin(m_instance1.GetShape()->GetAsShapeAsConvexPolygon() ? point1 : (point0 + point1).Scale(ndFloat32(0.5f)));
	const ndMatrix& matrix1 = m_instance1.m_globalMatrix;
	ndVector pointOnInstance1(matrix1.UntransformVector(origin));
	ndVector normalOnInstance1(matrix1.UnrotateVector(normal));
	ndFloat32 dist = (normal.DotProduct(point0 - point1)).GetScalar();

	if (dist < (D_PENETRATION_TOL * ndFloat32(-0.5f)))
	{
		count1 = m_instance1.CalculatePlaneIntersection(normalOnInstance1, pointOnInstance1, shape1);
	}
	if (!count1)
	{
		ndVector step(normal.Scale(D_PENETRATION_TOL * ndFloat32(2.0f)));
		ndVector alternatePoint(point1);
		for (ndInt32 i = 0; (i < 3) && !count1; ++i)
		{
			alternatePoint -= step;
			ndVector alternatePointOnInstance1(matrix1.UntransformVector(alternatePoint));
			count1 = m_instance1.CalculatePlaneIntersection(normalOnInstance1, alternatePointOnInstance1, shape1);
		}
		step = matrix1.UnrotateVector(normal * ((alternatePoint - origin).DotProduct(normal)));
		for (ndInt32 i = 0; i < count1; ++i)
		{
			shape1[i] -= step;
		}
	}

	if (count1)
	{
		for (ndInt32 i = 0; i < count1; ++i)
		{
			shape1[i] = matrix1.TransformVector(shape1[i]);
		}

		ndInt32 count0 = 0;
		ndVector* const shape0 = &contactsOut[baseCount + count1];

		const ndMatrix& matrix0 = m_instance0.m_globalMatrix;
		ndVector pointOnInstance0(matrix0.UntransformVector(origin));
		ndVector normalOnInstance0(matrix0.UnrotateVector(normal.Scale(ndFloat32(-1.0f))));

		if (dist < (D_PENETRATION_TOL * ndFloat32(-0.5f)))
		{
			count0 = m_instance0.CalculatePlaneIntersection(normalOnInstance0, pointOnInstance0, shape0);
		}
		if (!count0)
		{
			ndVector step(normal.Scale(D_PENETRATION_TOL * ndFloat32(2.0f)));
			ndVector alternatePoint(point0);
			for (ndInt32 i = 0; (i < 3) && !count0; ++i)
			{
				alternatePoint += step;
				ndVector alternatePointOnInstance0(matrix0.UntransformVector(alternatePoint));
				count0 = m_instance0.CalculatePlaneIntersection(normalOnInstance0, alternatePointOnInstance0, shape0);
			}
			//ndTrace (("If this is a frequent event, this routine should return the translation distance as the contact point\n"))
			//ndAssert(count0);
			step = matrix0.UnrotateVector(normal * ((alternatePoint - origin).DotProduct(normal)));
			for (ndInt32 i = 0; i < count0; ++i)
			{
				shape0[i] -= step;
			}
		}

		if (count0)
		{
			for (ndInt32 i = 0; i < count0; ++i)
			{
				shape0[i] = matrix0.TransformVector(shape0[i]);
			}

			if (count0 == 1)
			{
				count = 1;
				contactsOut[0] = shape0[0];
			}
			else if (count1 == 1)
			{
				count = 1;
				contactsOut[0] = shape1[0];
				if (m_instance1.GetShape()->GetAsShapeAsConvexPolygon())
				{
					ndInt32 i0 = count0 - 1;
					const ndVector p (shape1[0]);
					for (ndInt32 i1 = 0; i1 < count0; i1++)
					{
						const ndVector e(shape0[i1] - shape0[i0]);
						const ndVector n(normal.CrossProduct(e));
						ndAssert(n.m_w == ndFloat32(0.0f));
						ndAssert(n.DotProduct(n).GetScalar() > ndFloat32(0.0f));

						ndFloat32 test = n.DotProduct(p - shape0[i0]).GetScalar();
						if (test < ndFloat32(-1.e-3f))
						{
							count = 0;
							break;
						}
						i0 = i1;
					}
				}
			}
			else if ((count1 == 2) && (count0 == 2))
			{
				ndVector p0(shape1[0]);
				ndVector p1(shape1[1]);
				const ndVector& q0 = shape0[0];
				const ndVector& q1 = shape0[1];
				ndVector p10(p1 - p0);
				ndVector q10(q1 - q0);
				ndAssert(p10.m_w == ndFloat32(0.0f));
				ndAssert(q10.m_w == ndFloat32(0.0f));
				p10 = p10.Scale(ndRsqrt(p10.DotProduct(p10).GetScalar() + ndFloat32(1.0e-8f)));
				q10 = q10.Scale(ndRsqrt(q10.DotProduct(q10).GetScalar() + ndFloat32(1.0e-8f)));
				ndFloat32 dot = q10.DotProduct(p10).GetScalar();
				if (ndAbs(dot) > ndFloat32(0.998f))
				{
					// segment are collinear, the contact points is the overlapping segment
					ndFloat32 pl0 = p0.DotProduct(p10).GetScalar();
					ndFloat32 pl1 = p1.DotProduct(p10).GetScalar();
					ndFloat32 ql0 = q0.DotProduct(p10).GetScalar();
					ndFloat32 ql1 = q1.DotProduct(p10).GetScalar();
					if (pl0 > pl1)
					{
						ndSwap(pl0, pl1);
						ndSwap(p0, p1);
						p10 = p10.Scale(ndFloat32(-1.0f));
					}
					if (ql0 > ql1)
					{
						ndSwap(ql0, ql1);
					}
					if (!((ql0 > pl1) && (ql1 < pl0)))
					{
						ndFloat32 clip0 = (ql0 > pl0) ? ql0 : pl0;
						ndFloat32 clip1 = (ql1 < pl1) ? ql1 : pl1;

						count = 2;
						contactsOut[0] = p0 + p10.Scale(clip0 - pl0);
						contactsOut[1] = p0 + p10.Scale(clip1 - pl0);
					}
				}
				else
				{
					// only on contact at the closest distance segment
					count = 1;
					const ndFastRay ray(p0, p1);
					const ndRay intesect(ray.RayDistance(q0, q1));
					contactsOut[0] = (intesect.m_p0 + intesect.m_p1).Scale(ndFloat32(0.5f));
				}
			}
			else
			{
				ndAssert((count1 >= 2) && (count0 >= 2));
				count = ConvexPolygonsIntersection(normal, count0, shape0, count1, shape1, contactsOut, baseCount);
			}
		}
	}
	return count;
}