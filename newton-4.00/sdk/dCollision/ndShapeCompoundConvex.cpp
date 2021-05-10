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
#include "ndContact.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndCollisionStdafx.h"
#include "ndShapeCompoundConvex.h"

class ndShapeCompoundConvex::ndNodeBase
{
	public:
	ndNodeBase()
		:m_type(m_node)
		,m_left(nullptr)
		,m_right(nullptr)
		,m_parent(nullptr)
		,m_shape(nullptr)
		,m_myNode(nullptr)
	{
	}

	ndNodeBase(const ndNodeBase& copyFrom)
		:m_p0(copyFrom.m_p0)
		,m_p1(copyFrom.m_p1)
		,m_size(copyFrom.m_size)
		,m_origin(copyFrom.m_origin)
		,m_area(copyFrom.m_area)
		,m_type(copyFrom.m_type)
		,m_left(nullptr)
		,m_right(nullptr)
		,m_parent(nullptr)
		,m_shape(nullptr)
		,m_myNode(nullptr)
	{
		dAssert(!copyFrom.m_shape);
	}

	ndNodeBase(ndShapeInstance* const instance)
		:m_type(m_leaf)
		,m_left(nullptr)
		,m_right(nullptr)
		,m_parent(nullptr)
		,m_shape(new ndShapeInstance(*instance))
		,m_myNode(nullptr)
	{
		CalculateAABB();
	}

	ndNodeBase(ndNodeBase* const left, ndNodeBase* const right)
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

	~ndNodeBase()
	{
		if (m_shape) 
		{
			//m_shape->Release();
			delete m_shape;
		}
		if (m_left) 
		{
			delete m_left;
		}
		if (m_right) 
		{
			delete m_right;
		}
	}

	//void Sanity(int level = 0)
	//{
	//	char space[256];
	//	for (int i = 0; i < level; i++)
	//	{
	//		space[i] = ' ';
	//	}
	//	space[level] = 0;
	//
	//	dTrace(("%s%d\n", space, xxxxxx_));
	//
	//	if (m_left)
	//	{
	//		m_left->Sanity(level + 1);
	//	}
	//
	//	if (m_right)
	//	{
	//		m_right->Sanity(level + 1);
	//	}
	//}

	void CalculateAABB()
	{
		dVector p0;
		dVector p1;
		m_shape->CalculateAABB(m_shape->GetLocalMatrix(), p0, p1);
		SetBox(p0, p1);
	}

	void SetBox(const dVector& p0, const dVector& p1)
	{
		m_p0 = p0;
		m_p1 = p1;
		dAssert(m_p0.m_w == dFloat32(0.0f));
		dAssert(m_p1.m_w == dFloat32(0.0f));
		m_size = dVector::m_half * (m_p1 - m_p0);
		m_origin = dVector::m_half * (m_p1 + m_p0);
		m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
	}

	//bool BoxTest(const dgOOBBTestData& data) const;
	//bool BoxTest(const dgOOBBTestData& data, const ndNodeBase* const otherNode) const;
	//dFloat32 RayBoxDistance(const dgOOBBTestData& data, const dgFastRayTest& myRay, const dgFastRayTest& otherRay, const ndNodeBase* const otherNode) const;
	
	ndShapeInstance* GetShape() const
	{
		return m_shape;
	}
	
	//DG_INLINE dInt32 BoxIntersect(const dgFastRayTest& ray, const dVector& boxP0, const dVector& boxP1) const
	//{
	//	dVector minBox(m_p0 - boxP1);
	//	dVector maxBox(m_p1 - boxP0);
	//	return ray.BoxTest(minBox, maxBox);
	//}

	dVector m_p0;
	dVector m_p1;
	dVector m_size;
	dVector m_origin;
	dFloat32 m_area;
	dInt32 m_type;
	ndNodeBase* m_left;
	ndNodeBase* m_right;
	ndNodeBase* m_parent;
	ndShapeInstance* m_shape;
	ndTreeArray::dTreeNode* m_myNode;
};


class ndShapeCompoundConvex::ndSpliteInfo
{
	public:
	ndSpliteInfo(ndNodeBase** const boxArray, dInt32 boxCount)
	{
		dVector minP(dFloat32(1.0e15f));
		dVector maxP(-dFloat32(1.0e15f));

		if (boxCount == 2)
		{
			m_axis = 1;
			for (dInt32 i = 0; i < boxCount; i++)
			{
				ndNodeBase* const node = boxArray[i];
				dAssert(node->m_type == m_leaf);
				minP = minP.GetMin(node->m_p0);
				maxP = maxP.GetMax(node->m_p1);
			}
		}
		else
		{
			dVector median(dVector::m_zero);
			dVector varian(dVector::m_zero);

			for (dInt32 i = 0; i < boxCount; i++)
			{
				ndNodeBase* const node = boxArray[i];
				dAssert(node->m_type == m_leaf);
				minP = minP.GetMin(node->m_p0);
				maxP = maxP.GetMax(node->m_p1);
				dVector p(dVector::m_half * (node->m_p0 + node->m_p1));
				median += p;
				varian += p * p;
			}

			varian = varian.Scale(dFloat32(boxCount)) - median * median;

			dInt32 index = 0;
			dFloat32 maxVarian = dFloat32(-1.0e10f);
			for (dInt32 i = 0; i < 3; i++)
			{
				if (varian[i] > maxVarian)
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			dVector center = median.Scale(dFloat32(1.0f) / dFloat32(boxCount));

			dFloat32 test = center[index];

			dInt32 i0 = 0;
			dInt32 i1 = boxCount - 1;
			do
			{
				for (; i0 <= i1; i0++)
				{
					ndNodeBase* const node = boxArray[i0];
					dFloat32 val = (node->m_p0[index] + node->m_p1[index]) * dFloat32(0.5f);
					if (val > test)
					{
						break;
					}
				}

				for (; i1 >= i0; i1--)
				{
					ndNodeBase* const node = boxArray[i1];
					dFloat32 val = (node->m_p0[index] + node->m_p1[index]) * dFloat32(0.5f);
					if (val < test)
					{
						break;
					}
				}

				if (i0 < i1)
				{
					dSwap(boxArray[i0], boxArray[i1]);
					i0++;
					i1--;
				}

			} while (i0 <= i1);

			if (i0 > 0)
			{
				i0--;
			}
			if ((i0 + 1) >= boxCount)
			{
				i0 = boxCount - 2;
			}

			m_axis = i0 + 1;
		}

		dAssert(maxP.m_x - minP.m_x >= dFloat32(0.0f));
		dAssert(maxP.m_y - minP.m_y >= dFloat32(0.0f));
		dAssert(maxP.m_z - minP.m_z >= dFloat32(0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	dInt32 m_axis;
	dVector m_p0;
	dVector m_p1;
};

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

ndShapeCompoundConvex::ndShapeCompoundConvex(const ndShapeCompoundConvex& source, const ndShapeInstance* const myInstance)
	:ndShape(source)
	,m_array()
	,m_treeEntropy(dFloat32(0.0f))
	,m_boxMinRadius(dFloat32(0.0f))
	,m_boxMaxRadius(dFloat32(0.0f))
	,m_root(nullptr)
	,m_myInstance(myInstance)
	,m_idIndex(0)
{
	ndTreeArray::Iterator iter(source.m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const shape = node->GetShape();
		ndNodeBase* const newNode = new ndNodeBase(shape);
		m_array.AddNode(newNode, iter.GetNode()->GetKey(), m_myInstance);
	}

	if (source.m_root) 
	{
		ndNodeBase* pool[D_COMPOUND_STACK_DEPTH];
		ndNodeBase* parents[D_COMPOUND_STACK_DEPTH];
		pool[0] = source.m_root;
		parents[0] = nullptr;
		dInt32 stack = 1;
		while (stack) 
		{
			stack--;
			ndNodeBase* const sourceNode = pool[stack];
	
			ndNodeBase* parent = nullptr;
			if (sourceNode->m_type == m_node) 
			{
				parent = new ndNodeBase(*sourceNode);
				if (!sourceNode->m_parent) 
				{
					m_root = parent;
				}
				else 
				{
					parent->m_parent = parents[stack];
					if (parent->m_parent) 
					{
						if (sourceNode->m_parent->m_left == sourceNode) 
						{
							parent->m_parent->m_left = parent;
						}
						else 
						{
							dAssert(sourceNode->m_parent->m_right == sourceNode);
							parent->m_parent->m_right = parent;
						}
					}
				}
			}
			else 
			{
				//ndNodeBase* const node = m_array.Find (sourceNode->m_shape)->GetInfo();
				ndNodeBase* const node = m_array.Find(sourceNode->m_myNode->GetKey())->GetInfo();
				dAssert(node);
				node->m_parent = parents[stack];
				if (node->m_parent) 
				{
					if (sourceNode->m_parent->m_left == sourceNode) 
					{
						node->m_parent->m_left = node;
					}
					else 
					{
						dAssert(sourceNode->m_parent->m_right == sourceNode);
						node->m_parent->m_right = node;
					}
				}
				else 
				{
					m_root = node;
				}
			}
	
			if (sourceNode->m_left) 
			{
				parents[stack] = parent;
				pool[stack] = sourceNode->m_left;
				stack++;
				dAssert(stack < D_COMPOUND_STACK_DEPTH);
			}
	
			if (sourceNode->m_right) 
			{
				parents[stack] = parent;
				pool[stack] = sourceNode->m_right;
				stack++;
				dAssert(stack < D_COMPOUND_STACK_DEPTH);
			}
		}
	}
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
	if (m_root) 
	{
		delete m_root;
	}
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

void ndShapeCompoundConvex::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		collision->DebugShape(matrix, debugCallback);
	}
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

void ndShapeCompoundConvex::CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	if (m_root) 
	{
		const dVector origin(matrix.TransformVector(m_root->m_origin));
		const dVector size(matrix.m_front.Abs().Scale(m_root->m_size.m_x) + matrix.m_up.Abs().Scale(m_root->m_size.m_y) + matrix.m_right.Abs().Scale(m_root->m_size.m_z));
		p0 = (origin - size) & dVector::m_triplexMask;
		p1 = (origin + size) & dVector::m_triplexMask;
	}
	else 
{
		p0 = dVector::m_zero;
		p1 = dVector::m_zero;
	}
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

dFloat32 ndShapeCompoundConvex::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	if (!m_root) 
	{
		return dFloat32 (1.2f);
	}

	dFloat32 distance[D_COMPOUND_STACK_DEPTH];
	const ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

//	dFloat32 maxParam = maxT;
	dFastRayTest ray (localP0, localP1);

	dInt32 stack = 1;
	stackPool[0] = m_root;
	distance[0] = ray.BoxIntersect(m_root->m_p0, m_root->m_p1);
	while (stack) 
	{
		stack --;
		dFloat32 dist = distance[stack];

		if (dist > maxT) 
		{
			break;
		} 
		else 
		{
			const ndNodeBase* const me = stackPool[stack];
			dAssert (me);
			if (me->m_type == m_leaf) 
			{
				ndContactPoint tmpContactOut;
				ndShapeInstance* const shape = me->GetShape();
				dVector p0 (shape->GetLocalMatrix().UntransformVector (localP0));
				dVector p1 (shape->GetLocalMatrix().UntransformVector (localP1));
				dAssert(0);
				//dFloat32 param = shape->RayCast (p0, p1, maxT, tmpContactOut, preFilter, body, userData);
				//dFloat32 param = shape->RayCast(callback, p0, p1, maxT, body, &tmpContactOut);
				//if (param < maxT) 
				//{
				//	maxT = param;
				//	contactOut.m_normal = shape->GetLocalMatrix().RotateVector (tmpContactOut.m_normal);
				//	contactOut.m_shapeId0 = tmpContactOut.m_shapeId0;
				//	contactOut.m_shapeId1 = tmpContactOut.m_shapeId0;
				//	contactOut.m_collision0 = tmpContactOut.m_collision0;
				//	contactOut.m_collision1 = tmpContactOut.m_collision1;
				//}

			} 
			else 
			{
				dAssert(0);
				//dAssert (me->m_type == m_node);
				//const ndNodeBase* const left = me->m_left;
				//dAssert (left);
				//dFloat32 dist1 = ray.BoxIntersect(left->m_p0, left->m_p1);
				//if (dist1 < maxT) {
				//	dInt32 j = stack;
				//	for ( ; j && (dist1 > distance[j - 1]); j --) {
				//		stackPool[j] = stackPool[j - 1];
				//		distance[j] = distance[j - 1];
				//	}
				//	stackPool[j] = left;
				//	distance[j] = dist1;
				//	stack++;
				//	dgAssert (stack < dInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				//}
				//
				//const ndNodeBase* const right = me->m_right;
				//dgAssert (right);
				//dist1 = ray.BoxIntersect(right->m_p0, right->m_p1);
				//if (dist1 < maxT) {
				//	dInt32 j = stack;
				//	for ( ; j && (dist1 > distance[j - 1]); j --) {
				//		stackPool[j] = stackPool[j - 1];
				//		distance[j] = distance[j - 1];
				//	}
				//	stackPool[j] = right;
				//	distance[j] = dist1;
				//	stack++;
				//	dgAssert (stack < dInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				//}
			}
		}
	}
	return maxT;
}

void ndShapeCompoundConvex::BeginAddRemove()
{
	dAssert(m_myInstance);
}

dFloat32 ndShapeCompoundConvex::CalculateSurfaceArea(ndNodeBase* const node0, ndNodeBase* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_p0.GetMin(node1->m_p0);
	maxBox = node0->m_p1.GetMax(node1->m_p1);
	dVector side0(dVector::m_half * (maxBox - minBox));
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
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

dInt32 ndShapeCompoundConvex::CompareNodes(const ndNodeBase* const nodeA, const ndNodeBase* const nodeB, void*)
{
	dFloat32 areaA = nodeA->m_area;
	dFloat32 areaB = nodeB->m_area;
	if (areaA < areaB) 
	{
		return -1;
	}
	if (areaA > areaB) 
	{
		return 1;
	}
	return 0;
}

ndShapeCompoundConvex::ndNodeBase* ndShapeCompoundConvex::BuildTopDown(ndNodeBase** const leafArray, dInt32 firstBox, dInt32 lastBox, ndNodeBase** rootNodesMemory, dInt32& rootIndex)
{
	dAssert(lastBox >= 0);
	dAssert(firstBox >= 0);
	
	if (lastBox == firstBox) 
	{
		ndNodeBase* const node = leafArray[firstBox];
		return node;
	}
	
	ndSpliteInfo info(&leafArray[firstBox], lastBox - firstBox + 1);
		
	ndNodeBase* const parent = rootNodesMemory[rootIndex];
	rootIndex++;
	parent->m_parent = nullptr;
		
	parent->SetBox(info.m_p0, info.m_p1);
	parent->m_right = BuildTopDown(leafArray, firstBox + info.m_axis, lastBox, rootNodesMemory, rootIndex);
	parent->m_right->m_parent = parent;
		
	parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + info.m_axis - 1, rootNodesMemory, rootIndex);
	parent->m_left->m_parent = parent;
	return parent;
}

ndShapeCompoundConvex::ndNodeBase* ndShapeCompoundConvex::BuildTopDownBig(ndNodeBase** const leafArray, dInt32 firstBox, dInt32 lastBox, ndNodeBase** rootNodesMemory, dInt32& rootIndex)
{
	if (lastBox == firstBox) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, rootNodesMemory, rootIndex);
	}

	dInt32 midPoint = -1;
	const dFloat32 scale = dFloat32(10.0f);
	const dFloat32 scale2 = dFloat32(3.0f) * scale * scale;
	const dInt32 count = lastBox - firstBox;
	for (dInt32 i = 0; i < count; i++) 
	{
		const ndNodeBase* const node0 = leafArray[firstBox + i];
		const ndNodeBase* const node1 = leafArray[firstBox + i + 1];
		if (node1->m_area > (scale2 * node0->m_area)) 
		{
			midPoint = i;
			break;
		}
	}

	if (midPoint == -1) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, rootNodesMemory, rootIndex);
	}

	ndNodeBase* const parent = rootNodesMemory[rootIndex];
	rootIndex++;
	parent->m_parent = nullptr;
	
	dVector minP(dFloat32(1.0e15f));
	dVector maxP(-dFloat32(1.0e15f));
	for (dInt32 i = 0; i <= count; i++) 
	{
		const ndNodeBase* const node = leafArray[firstBox + i];
		dAssert(node->m_shape);
		minP = minP.GetMin(node->m_p0);
		maxP = maxP.GetMax(node->m_p1);
	}
	
	parent->SetBox(minP, maxP);
	parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + midPoint, rootNodesMemory, rootIndex);
	parent->m_left->m_parent = parent;
	
	parent->m_right = BuildTopDownBig(leafArray, firstBox + midPoint + 1, lastBox, rootNodesMemory, rootIndex);
	parent->m_right->m_parent = parent;
	return parent;
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

		dInt32 stack = 1;
		dInt32 nodeCount = 0;
		ndNodeBase** nodeArray = dAlloca(ndNodeBase*, m_array.GetCount() + 10);
		ndNodeBase* stackBuffer[D_COMPOUND_STACK_DEPTH];

		stackBuffer[0] = m_root;
		while (stack) 
		{
			stack--;
			ndNodeBase* const node = stackBuffer[stack];
		
			if (node->m_type == m_node) 
			{
				nodeArray[nodeCount] = node;
				nodeCount++;
				dAssert(nodeCount <= m_array.GetCount());

				stackBuffer[stack] = node->m_right;
				stack++;
				dAssert(stack < sizeof(stackBuffer) / sizeof(stackBuffer[0]));

				stackBuffer[stack] = node->m_left;
				stack++;
				dAssert(stack < sizeof(stackBuffer) / sizeof(stackBuffer[0]));
			}
		}
		
		if (nodeCount)
		{
			dFloat64 cost = CalculateEntropy(nodeCount, nodeArray);
			if ((cost > m_treeEntropy * dFloat32(2.0f)) || (cost < m_treeEntropy * dFloat32(0.5f))) 
			{
				dInt32 leafNodesCount = 0;
				ndNodeBase** leafArray = dAlloca(ndNodeBase*, nodeCount + 12);
				for (dInt32 i = 0; i < nodeCount; i++)
				{ 
					ndNodeBase* const node = nodeArray[i];
					if (node->m_left->m_type == m_leaf) 
					{
						leafArray[leafNodesCount] = node->m_left;
						leafNodesCount++;
						dAssert(leafNodesCount <= (nodeCount + 1));
					}
					if (node->m_right->m_type == m_leaf) 
					{
						leafArray[leafNodesCount] = node->m_right;
						leafNodesCount++;
						dAssert(leafNodesCount <= (nodeCount + 1));
					}
				}
		
				dSortIndirect(&leafArray[0], leafNodesCount, CompareNodes);

				dInt32 rootIndex = 0;
				m_root = BuildTopDownBig(&leafArray[0], 0, leafNodesCount - 1, nodeArray, rootIndex);

				//m_root->Sanity();
				m_treeEntropy = CalculateEntropy(nodeCount, nodeArray);
			}
			while (m_root->m_parent) 
			{
				m_root = m_root->m_parent;
			}

			//m_root->Sanity();
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

void ndShapeCompoundConvex::MassProperties()
{
#ifdef _DEBUG
	//	dVector origin_ (dVector::m_zero);
	//	dVector inertia_ (dVector::m_zero);
	//	dVector crossInertia_ (dVector::m_zero);
	//	dgPolyhedraMassProperties localData;
	//	DebugCollision (dgGetIdentityMatrix(), CalculateInertia, &localData);
	//	dFloat32 volume_ = localData.MassProperties (origin_, inertia_, crossInertia_);
	//	dAssert (volume_ > dFloat32 (0.0f));
	//	dFloat32 invVolume_ = dFloat32 (1.0f)/volume_;
	//	m_centerOfMass = origin_.Scale (invVolume_);
	//	m_centerOfMass.m_w = volume_;
	//	m_inertia = inertia_.Scale (invVolume_);
	//	m_crossInertia = crossInertia_.Scale(invVolume_);
#endif

	dFloat32 volume = dFloat32(0.0f);
	dVector origin(dVector::m_zero);
	dVector inertiaII(dVector::m_zero);
	dVector inertiaIJ(dVector::m_zero);
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		dMatrix shapeInertia(collision->CalculateInertia());
		dFloat32 shapeVolume = collision->GetVolume();

		volume += shapeVolume;
		origin += shapeInertia.m_posit.Scale(shapeVolume);
		inertiaII += dVector(shapeInertia[0][0], shapeInertia[1][1], shapeInertia[2][2], dFloat32(0.0f)).Scale(shapeVolume);
		inertiaIJ += dVector(shapeInertia[1][2], shapeInertia[0][2], shapeInertia[0][1], dFloat32(0.0f)).Scale(shapeVolume);
	}
	if (volume > dFloat32(0.0f)) 
	{
		dFloat32 invVolume = dFloat32(1.0f) / volume;
		m_inertia = inertiaII.Scale(invVolume);
		m_crossInertia = inertiaIJ.Scale(invVolume);
		m_centerOfMass = origin.Scale(invVolume);
		m_centerOfMass.m_w = volume;
	}

	ndShape::MassProperties();
}

void ndShapeCompoundConvex::ApplyScale(const dVector& scale)
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const collision = node->GetShape();
		collision->SetGlobalScale(scale);
	}
	m_treeEntropy = dFloat32(0.0f);
	EndAddRemove();
}