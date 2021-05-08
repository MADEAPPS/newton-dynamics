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
#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndCollisionStdafx.h"
#include "ndShapeCompoundConvex.h"


ndShapeCompoundConvex::ndNodeBase::ndNodeBase(ndShapeInstance* const instance)
	:m_type(m_leaf)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_shape(new ndShapeInstance(*instance))
	,m_myNode(nullptr)
{
	CalculateAABB();
}

ndShapeCompoundConvex::ndNodeBase::ndNodeBase(ndNodeBase* const left, ndNodeBase* const right)
	:m_type(m_node)
	,m_left(left)
	,m_right(right)
	,m_parent(nullptr)
	,m_shape(nullptr)
	,m_myNode(nullptr)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	dVector p0(left->m_p0.GetMin(right->m_p0));
	dVector p1(left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}

void ndShapeCompoundConvex::ndNodeBase::SetBox(const dVector& p0, const dVector& p1)
{
	m_p0 = p0;
	m_p1 = p1;
	dAssert(m_p0.m_w == dFloat32(0.0f));
	dAssert(m_p1.m_w == dFloat32(0.0f));
	m_size = dVector::m_half * (m_p1 - m_p0);
	m_origin = dVector::m_half * (m_p1 + m_p0);
	m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
}

void ndShapeCompoundConvex::ndNodeBase::CalculateAABB()
{
	dVector p0;
	dVector p1;
	m_shape->CalculateAABB(m_shape->GetLocalMatrix(), p0, p1);
	SetBox(p0, p1);
}

ndShapeCompoundConvex::ndTreeArray::ndTreeArray()
	:dTree<ndNodeBase*, dInt32, dContainersFreeListAlloc<ndNodeBase*>>()
{
}

void ndShapeCompoundConvex::ndTreeArray::AddNode(ndNodeBase* const node, dInt32 index, const ndShapeInstance* const parent)
{
	ndTreeArray::dTreeNode* const myNode = Insert(node, index);
	node->m_myNode = myNode;
	node->m_shape->m_parent = parent;
	node->m_shape->m_subCollisionHandle = myNode;
}

ndShapeCompoundConvex::ndShapeCompoundConvex()
	:ndShape(m_compoundConvex)
	,m_array()
	,m_treeEntropy(dFloat32(0.0f))
	,m_boxMinRadius(dFloat32(0.0f))
	,m_boxMaxRadius(dFloat32(0.0f))
	,m_root(nullptr)
	,m_myInstance(nullptr)
	,m_idIndex(0)
{
}

ndShapeCompoundConvex::ndShapeCompoundConvex(const nd::TiXmlNode* const xmlNode)
	:ndShape(m_compoundConvex)
	,m_array()
	,m_treeEntropy(dFloat32 (0.0f))
	,m_boxMinRadius(dFloat32(0.0f))
	,m_boxMaxRadius(dFloat32(0.0f))
	,m_root(nullptr)
	,m_myInstance(nullptr)
	,m_idIndex(0)
{
	dAssert(0);
	xmlGetInt(xmlNode, "xxxx");
}

ndShapeCompoundConvex::~ndShapeCompoundConvex()
{
	dAssert(0);
}

/*
void ndShapeCompoundConvex::CalcAABB(const dMatrix& matrix, dVector &p0, dVector &p1) const
{
	dAssert(0);
	dVector origin(matrix.TransformVector(m_boxOrigin));
	dVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & dVector::m_triplexMask;
	p1 = (origin + size) & dVector::m_triplexMask;
}


//dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dFloat32* const vertex, const dInt32* const index, dInt32 indexCount, dInt32 stride, const dPlane& localPlane, dVector* const contactsOut) const
dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dFloat32* const, const dInt32* const, dInt32, dInt32, const dPlane&, dVector* const) const
{
	dAssert(0);
	return 0;
	//dInt32 count = 0;
	//dInt32 j = index[indexCount - 1] * stride;
	//dVector p0(&vertex[j]);
	//p0 = p0 & dVector::m_triplexMask;
	//dFloat32 side0 = localPlane.Evalue(p0);
	//for (dInt32 i = 0; i < indexCount; i++) {
	//	j = index[i] * stride;
	//	dVector p1(&vertex[j]);
	//	p1 = p1 & dVector::m_triplexMask;
	//	dFloat32 side1 = localPlane.Evalue(p1);
	//
	//	if (side0 < dFloat32(0.0f)) {
	//		if (side1 >= dFloat32(0.0f)) {
	//			dVector dp(p1 - p0);
	//			dAssert(dp.m_w == dFloat32(0.0f));
	//			dFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//			dAssert(dgAbs(t) >= dFloat32(0.0f));
	//			if (dgAbs(t) < dFloat32(1.0e-8f)) {
	//				t = dgSign(t) * dFloat32(1.0e-8f);
	//			}
	//			dAssert(0);
	//			contactsOut[count] = p0 - dp.Scale(side0 / t);
	//			count++;
	//
	//		}
	//	}
	//	else if (side1 <= dFloat32(0.0f)) {
	//		dVector dp(p1 - p0);
	//		dAssert(dp.m_w == dFloat32(0.0f));
	//		dFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//		dAssert(dgAbs(t) >= dFloat32(0.0f));
	//		if (dgAbs(t) < dFloat32(1.0e-8f)) {
	//			t = dgSign(t) * dFloat32(1.0e-8f);
	//		}
	//		dAssert(0);
	//		contactsOut[count] = p0 - dp.Scale(side0 / t);
	//		count++;
	//	}
	//
	//	side0 = side1;
	//	p0 = p1;
	//}
	//
	//return count;
}
*/

ndShapeInfo ndShapeCompoundConvex::GetShapeInfo() const
{
	ndShapeInfo info(ndShape::GetShapeInfo());

	dAssert(0);
	return info;
}

//void ndShapeCompoundConvex::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
void ndShapeCompoundConvex::DebugShape(const dMatrix&, ndShapeDebugCallback&) const
{
	dAssert(0);
}

dFloat32 ndShapeCompoundConvex::GetVolume() const
{
	dAssert(0);
	return dFloat32(0.0f);
}

dFloat32 ndShapeCompoundConvex::GetBoxMinRadius() const
{
	dAssert(0);
	return dFloat32(0.0f);
}

dFloat32 ndShapeCompoundConvex::GetBoxMaxRadius() const
{
	dAssert(0);
	return dFloat32(0.0f);
}

//void ndShapeCompoundConvex::CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
void ndShapeCompoundConvex::CalcAABB(const dMatrix&, dVector&, dVector&) const
{
	dAssert(0);
}

dVector ndShapeCompoundConvex::SupportVertex(const dVector&, dInt32* const) const
{
	dAssert(0);
	return dVector::m_zero;
}

//dVector ndShapeCompoundConvex::SupportVertexSpecialProjectPoint(const dVector& point, const dVector&) const
dVector ndShapeCompoundConvex::SupportVertexSpecialProjectPoint(const dVector&, const dVector&) const
{ 
	dAssert(0);
	return dVector::m_zero;
}

dVector ndShapeCompoundConvex::SupportVertexSpecial(const dVector& dir, dFloat32, dInt32* const vertexIndex) const
{
	dAssert(0);
	return SupportVertex(dir, vertexIndex);
}

dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dVector&, const dVector&, dVector* const) const
{
	dAssert(0);
	return 0;
}

dVector ndShapeCompoundConvex::CalculateVolumeIntegral(const dMatrix&, const dVector&, const ndShapeInstance&) const
{
	dAssert(0);
	return dVector::m_zero;
}

//dFloat32 ndShapeCompoundConvex::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
dFloat32 ndShapeCompoundConvex::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	dAssert(0);
	return 0;
}

void ndShapeCompoundConvex::BeginAddRemove()
{
	dAssert(m_myInstance);
}

void ndShapeCompoundConvex::ImproveNodeFitness(ndNodeBase* const node) const
{
	dAssert(node->m_left);
	dAssert(node->m_right);

	if (node->m_parent) 
	{
		if (node->m_parent->m_left == node) 
		{
			dFloat32 cost0 = node->m_area;

			dVector cost1P0;
			dVector cost1P1;
			dFloat32 cost1 = CalculateSurfaceArea(node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dVector cost2P0;
			dVector cost2P1;
			dFloat32 cost2 = CalculateSurfaceArea(node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			dAssert(node->m_parent->m_p0.m_w == dFloat32(0.0f));
			dAssert(node->m_parent->m_p1.m_w == dFloat32(0.0f));

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_left = node->m_right;
				node->m_right = parent;
				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;
				parent->m_area = cost1;
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;

			}
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_left = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;
				parent->m_area = cost2;
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;
			}
		}
		else 
		{
			dFloat32 cost0 = node->m_area;

			dVector cost1P0;
			dVector cost1P1;
			dFloat32 cost1 = CalculateSurfaceArea(node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dVector cost2P0;
			dVector cost2P1;
			dFloat32 cost2 = CalculateSurfaceArea(node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_right = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;
				parent->m_area = cost1;
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;

			}
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_right = node->m_right;
				node->m_right = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;
				parent->m_area = cost2;
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;
			}
		}
	}
	else 
{
		// in the future I can handle this but it is too much work for little payoff
	}
}

dFloat64 ndShapeCompoundConvex::CalculateEntropy(dInt32 count, ndNodeBase** array)
{
	dFloat64 cost0 = dFloat32(1.0e20f);
	dFloat64 cost1 = cost0;
	do {
		cost1 = cost0;
		//for (dgList<ndNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
		for (dInt32 i = 0; i < count; i ++) 
		{
			ndNodeBase* const node = array[i];
			ImproveNodeFitness(node);
		}

		cost0 = dFloat32(0.0f);
		//for (dgList<ndNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
		for (dInt32 i = 0; i < count; i++)
		{
			//ndNodeBase* const node = listNode->GetInfo();
			ndNodeBase* const node = array[i];
			cost0 += node->m_area;
		}
	} while (cost0 < (cost1 * dFloat32(0.9999f)));
	return cost0;
}

void ndShapeCompoundConvex::EndAddRemove()
{
	if (m_root) 
	{
		//dgScopeSpinLock lock(&m_criticalSectionLock);

		ndTreeArray::Iterator iter(m_array);
		for (iter.Begin(); iter; iter++) 
		{
			ndNodeBase* const node = iter.GetNode()->GetInfo();
			node->CalculateAABB();
		}
		
		//dList<ndNodeBase*> list;
		//dList<ndNodeBase*> stack;


		dInt32 stack = 1;
		dInt32 listCount = 0;
		ndNodeBase** list = dAlloca(ndNodeBase*, m_array.GetCount() * 2 + 10);
		ndNodeBase* stackBuffer[1024];

		stackBuffer[0] = m_root;
		while (stack) 
		{
			stack--;
			ndNodeBase* const node = stackBuffer[stack];
		
			if (node->m_type == m_node) 
			{
				list[listCount] = node;
				listCount++;
				//stack.Append(node->m_right);
				stackBuffer[stack] = node->m_right;
				stack++;
				dAssert(stack < sizeof(stackBuffer) / sizeof(stackBuffer[0]));
				//stack.Append(node->m_left);
				stackBuffer[stack] = node->m_left;
				stack++;
				dAssert(stack < sizeof(stackBuffer) / sizeof(stackBuffer[0]));
			}
		}
		
		if (listCount) 
		{
			dFloat64 cost = CalculateEntropy(listCount, list);
			if ((cost > m_treeEntropy * dFloat32(2.0f)) || (cost < m_treeEntropy * dFloat32(0.5f))) 
			{
				dInt32 count = listCount * 2 + 12;
				dInt32 leafNodesCount = 0;
				//dgStack<ndNodeBase*> leafArray(count);
		//		for (dgList<ndNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
				for (dInt32 i = 0; i < listCount; i++)
				{ 
		//			ndNodeBase* const node = listNode->GetInfo();
					ndNodeBase* const node = list[i];
					if (node->m_left->m_type == m_leaf) 
					{
						dAssert(0);
		//				leafArray[leafNodesCount] = node->m_left;
		//				leafNodesCount++;
					}
					if (node->m_right->m_type == m_leaf) 
					{
						dAssert(0);
		//				leafArray[leafNodesCount] = node->m_right;
		//				leafNodesCount++;
					}
				}
		
				dAssert(0);
		//		dgList<ndNodeBase*>::dgListNode* nodePtr = list.GetFirst();
		//		dgSortIndirect(&leafArray[0], leafNodesCount, CompareNodes);
		//		m_root = BuildTopDownBig(&leafArray[0], 0, leafNodesCount - 1, &nodePtr);
				m_treeEntropy = CalculateEntropy(listCount, list);
			}
			while (m_root->m_parent) 
			{
				m_root = m_root->m_parent;
			}
		}
		else 
		{
			m_treeEntropy = dFloat32(2.0f);
		}
		
		dAssert(m_root->m_size.m_w == dFloat32(0.0f));
		m_boxMinRadius = dMin(m_root->m_size.m_x, m_root->m_size.m_y, m_root->m_size.m_z);
		m_boxMaxRadius = dSqrt(m_root->m_size.DotProduct(m_root->m_size).GetScalar());
		
		m_boxSize = m_root->m_size;
		m_boxOrigin = m_root->m_origin;
		MassProperties();
	}
}

ndShapeCompoundConvex::ndTreeArray::dTreeNode* ndShapeCompoundConvex::AddCollision(ndShapeInstance* const shape)
{
	dAssert(m_myInstance);
	ndNodeBase* const newNode = new ndNodeBase(shape);
	m_array.AddNode(newNode, m_idIndex, m_myInstance);

	m_idIndex++;
	
	if (!m_root) 
	{
		m_root = newNode;
	}
	else 
	{
		dVector p0;
		dVector p1;
		ndNodeBase* sibling = m_root;
		dFloat32 surfaceArea = CalculateSurfaceArea(newNode, sibling, p0, p1);
		while (sibling->m_left && sibling->m_right) 
		{
			if (surfaceArea > sibling->m_area) 
			{
				break;
			}
	
			sibling->SetBox(p0, p1);
	
			dVector leftP0;
			dVector leftP1;
			dFloat32 leftSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_left, leftP0, leftP1);
	
			dVector rightP0;
			dVector rightP1;
			dFloat32 rightSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_right, rightP0, rightP1);
	
			if (leftSurfaceArea < rightSurfaceArea) 
			{
				sibling = sibling->m_left;
				p0 = leftP0;
				p1 = leftP1;
				surfaceArea = leftSurfaceArea;
			}
			else 
			{
				sibling = sibling->m_right;
				p0 = rightP0;
				p1 = rightP1;
				surfaceArea = rightSurfaceArea;
			}
		}
	
		if (!sibling->m_parent) 
		{
			m_root = new ndNodeBase(sibling, newNode);
		}
		else 
		{
			ndNodeBase* const parent = sibling->m_parent;
			if (parent->m_left == sibling) 
			{
				ndNodeBase* const node = new ndNodeBase(sibling, newNode);
				parent->m_left = node;
				node->m_parent = parent;
			}
			else 
			{
				dAssert(parent->m_right == sibling);
				ndNodeBase* const node = new ndNodeBase(sibling, newNode);
				parent->m_right = node;
				node->m_parent = parent;
			}
		}
	}
	
	return newNode->m_myNode;
}