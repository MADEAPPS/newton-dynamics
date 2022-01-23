/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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
#include "ndContact.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"
#include "ndShapeCompound.h"

#define D_MAX_MIN_VOLUME	ndFloat32 (1.0e-3f)
D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeCompound)

ndShapeCompound::ndNodeBase::~ndNodeBase()
{
	if (m_shapeInstance) 
	{
		delete m_shapeInstance;
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


ndShapeCompound::ndNodeBase::ndNodeBase()
	:ndClassAlloc()
	, m_type(m_node)
	, m_left(nullptr)
	, m_right(nullptr)
	, m_parent(nullptr)
	, m_myNode(nullptr)
	, m_shapeInstance(nullptr)
{
}

ndShapeCompound::ndNodeBase::ndNodeBase(const ndNodeBase& copyFrom)
	:ndClassAlloc()
	, m_p0(copyFrom.m_p0)
	, m_p1(copyFrom.m_p1)
	, m_size(copyFrom.m_size)
	, m_origin(copyFrom.m_origin)
	, m_area(copyFrom.m_area)
	, m_type(copyFrom.m_type)
	, m_left(nullptr)
	, m_right(nullptr)
	, m_parent(nullptr)
	, m_myNode(nullptr)
	, m_shapeInstance(nullptr)
{
	dAssert(!copyFrom.m_shapeInstance);
}

ndShapeCompound::ndNodeBase::ndNodeBase(ndShapeInstance* const instance)
	:ndClassAlloc()
	, m_type(m_leaf)
	, m_left(nullptr)
	, m_right(nullptr)
	, m_parent(nullptr)
	, m_myNode(nullptr)
	, m_shapeInstance(new ndShapeInstance(*instance))
{
	CalculateAABB();
}

ndShapeCompound::ndNodeBase::ndNodeBase(ndNodeBase* const left, ndNodeBase* const right)
	:ndClassAlloc()
	, m_type(m_node)
	, m_left(left)
	, m_right(right)
	, m_parent(nullptr)
	, m_myNode(nullptr)
	, m_shapeInstance(nullptr)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	ndVector p0(left->m_p0.GetMin(right->m_p0));
	ndVector p1(left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}

ndShapeInstance* ndShapeCompound::ndNodeBase::GetShape() const
{
	return m_shapeInstance;
}

inline void ndShapeCompound::ndNodeBase::CalculateAABB()
{
	ndVector p0;
	ndVector p1;
	m_shapeInstance->CalculateAabb(m_shapeInstance->GetLocalMatrix(), p0, p1);
	SetBox(p0, p1);
}

inline void ndShapeCompound::ndNodeBase::SetBox(const ndVector& p0, const ndVector& p1)
{
	m_p0 = p0;
	m_p1 = p1;
	dAssert(m_p0.m_w == ndFloat32(0.0f));
	dAssert(m_p1.m_w == ndFloat32(0.0f));
	m_size = ndVector::m_half * (m_p1 - m_p0);
	m_origin = ndVector::m_half * (m_p1 + m_p0);
	m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
}

const ndShapeCompound::ndTreeArray& ndShapeCompound::GetTree() const
{
	return m_array;
}

class ndShapeCompound::ndSpliteInfo
{
	public:
	ndSpliteInfo(ndNodeBase** const boxArray, ndInt32 boxCount)
	{
		ndVector minP(ndFloat32(1.0e15f));
		ndVector maxP(-ndFloat32(1.0e15f));

		if (boxCount == 2)
		{
			m_axis = 1;
			for (ndInt32 i = 0; i < boxCount; i++)
			{
				ndNodeBase* const node = boxArray[i];
				dAssert(node->m_type == m_leaf);
				minP = minP.GetMin(node->m_p0);
				maxP = maxP.GetMax(node->m_p1);
			}
		}
		else
		{
			ndVector median(ndVector::m_zero);
			ndVector varian(ndVector::m_zero);

			for (ndInt32 i = 0; i < boxCount; i++)
			{
				ndNodeBase* const node = boxArray[i];
				dAssert(node->m_type == m_leaf);
				minP = minP.GetMin(node->m_p0);
				maxP = maxP.GetMax(node->m_p1);
				ndVector p(ndVector::m_half * (node->m_p0 + node->m_p1));
				median += p;
				varian += p * p;
			}

			varian = varian.Scale(ndFloat32(boxCount)) - median * median;

			ndInt32 index = 0;
			ndFloat32 maxVarian = ndFloat32(-1.0e10f);
			for (ndInt32 i = 0; i < 3; i++)
			{
				if (varian[i] > maxVarian)
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			ndVector center = median.Scale(ndFloat32(1.0f) / ndFloat32(boxCount));

			ndFloat32 test = center[index];

			ndInt32 i0 = 0;
			ndInt32 i1 = boxCount - 1;
			do
			{
				for (; i0 <= i1; i0++)
				{
					ndNodeBase* const node = boxArray[i0];
					ndFloat32 val = (node->m_p0[index] + node->m_p1[index]) * ndFloat32(0.5f);
					if (val > test)
					{
						break;
					}
				}

				for (; i1 >= i0; i1--)
				{
					ndNodeBase* const node = boxArray[i1];
					ndFloat32 val = (node->m_p0[index] + node->m_p1[index]) * ndFloat32(0.5f);
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

		dAssert(maxP.m_x - minP.m_x >= ndFloat32(0.0f));
		dAssert(maxP.m_y - minP.m_y >= ndFloat32(0.0f));
		dAssert(maxP.m_z - minP.m_z >= ndFloat32(0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	ndInt32 m_axis;
	ndVector m_p0;
	ndVector m_p1;
};

ndShapeCompound::ndTreeArray::ndTreeArray()
	:ndTree<ndNodeBase*, ndInt32, ndContainersFreeListAlloc<ndNodeBase*>>()
{
}

void ndShapeCompound::ndTreeArray::AddNode(ndNodeBase* const node, ndInt32 index, const ndShapeInstance* const parent)
{
	ndTreeArray::ndNode* const myNode = Insert(node, index);
	node->m_myNode = myNode;
	node->m_shapeInstance->m_parent = parent;
	node->m_shapeInstance->m_subCollisionHandle = myNode;
}

ndShapeCompound::ndShapeCompound()
	:ndShape(m_compound)
	,m_array()
	,m_treeEntropy(ndFloat32(0.0f))
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
	,m_root(nullptr)
	,m_myInstance(nullptr)
	,m_idIndex(0)
{
}

ndShapeCompound::ndShapeCompound(const ndShapeCompound& source, const ndShapeInstance* const myInstance)
	:ndShape(source)
	,m_array()
	,m_treeEntropy(ndFloat32(0.0f))
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
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
		ndInt32 stack = 1;
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

ndShapeCompound::ndShapeCompound(const ndLoadSaveBase::ndLoadDescriptor&)
	:ndShape(m_compound)
	,m_array()
	,m_treeEntropy(ndFloat32 (0.0f))
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
	,m_root(nullptr)
	,m_myInstance(nullptr)
	,m_idIndex(0)
{
	//do nothing here;
	//sub shapes will be load in a post process pass,
	//when all sub shape are converted to instanaced.
}

ndShapeCompound::~ndShapeCompound()
{
	if (m_root) 
	{
		delete m_root;
	}
}

/*
void ndShapeCompound::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	dAssert(0);
	ndVector origin(matrix.TransformVector(m_boxOrigin));
	ndVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & ndVector::m_triplexMask;
	p1 = (origin + size) & ndVector::m_triplexMask;
}


//ndInt32 ndShapeCompound::CalculatePlaneIntersection(const ndFloat32* const vertex, const ndInt32* const index, ndInt32 indexCount, ndInt32 stride, const dPlane& localPlane, ndVector* const contactsOut) const
ndInt32 ndShapeCompound::CalculatePlaneIntersection(const ndFloat32* const, const ndInt32* const, ndInt32, ndInt32, const dPlane&, ndVector* const) const
{
	dAssert(0);
	return 0;
	//ndInt32 count = 0;
	//ndInt32 j = index[indexCount - 1] * stride;
	//ndVector p0(&vertex[j]);
	//p0 = p0 & ndVector::m_triplexMask;
	//ndFloat32 side0 = localPlane.Evalue(p0);
	//for (ndInt32 i = 0; i < indexCount; i++) {
	//	j = index[i] * stride;
	//	ndVector p1(&vertex[j]);
	//	p1 = p1 & ndVector::m_triplexMask;
	//	ndFloat32 side1 = localPlane.Evalue(p1);
	//
	//	if (side0 < ndFloat32(0.0f)) {
	//		if (side1 >= ndFloat32(0.0f)) {
	//			ndVector dp(p1 - p0);
	//			dAssert(dp.m_w == ndFloat32(0.0f));
	//			ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//			dAssert(dgAbs(t) >= ndFloat32(0.0f));
	//			if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//				t = dgSign(t) * ndFloat32(1.0e-8f);
	//			}
	//			dAssert(0);
	//			contactsOut[count] = p0 - dp.Scale(side0 / t);
	//			count++;
	//
	//		}
	//	}
	//	else if (side1 <= ndFloat32(0.0f)) {
	//		ndVector dp(p1 - p0);
	//		dAssert(dp.m_w == ndFloat32(0.0f));
	//		ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//		dAssert(dgAbs(t) >= ndFloat32(0.0f));
	//		if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//			t = dgSign(t) * ndFloat32(1.0e-8f);
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

ndShapeInfo ndShapeCompound::GetShapeInfo() const
{
	ndShapeInfo info(ndShape::GetShapeInfo());
	info.m_compound.m_noUsed = 0;
	return info;
}

void ndShapeCompound::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		collision->DebugShape(matrix, debugCallback);
	}
}

ndFloat32 ndShapeCompound::GetVolume() const
{
	dAssert(0);
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeCompound::GetBoxMinRadius() const
{
	return m_boxMinRadius;
}

ndFloat32 ndShapeCompound::GetBoxMaxRadius() const
{
	return m_boxMaxRadius;
}

void ndShapeCompound::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	if (m_root) 
	{
		const ndVector origin(matrix.TransformVector(m_root->m_origin));
		const ndVector size(matrix.m_front.Abs().Scale(m_root->m_size.m_x) + matrix.m_up.Abs().Scale(m_root->m_size.m_y) + matrix.m_right.Abs().Scale(m_root->m_size.m_z));
		p0 = (origin - size) & ndVector::m_triplexMask;
		p1 = (origin + size) & ndVector::m_triplexMask;
	}
	else 
	{
		p0 = ndVector::m_zero;
		p1 = ndVector::m_zero;
	}
}

ndVector ndShapeCompound::SupportVertex(const ndVector&, ndInt32* const) const
{
	dAssert(0);
	return ndVector::m_zero;
}

//ndVector ndShapeCompound::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
ndVector ndShapeCompound::SupportVertexSpecialProjectPoint(const ndVector&, const ndVector&) const
{ 
	dAssert(0);
	return ndVector::m_zero;
}

ndVector ndShapeCompound::SupportVertexSpecial(const ndVector& dir, ndFloat32, ndInt32* const vertexIndex) const
{
	dAssert(0);
	return SupportVertex(dir, vertexIndex);
}

ndInt32 ndShapeCompound::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	dAssert(0);
	return 0;
}

ndVector ndShapeCompound::CalculateVolumeIntegral(const ndMatrix&, const ndVector&, const ndShapeInstance&) const
{
	dAssert(0);
	return ndVector::m_zero;
}

ndFloat32 ndShapeCompound::RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	if (!m_root) 
	{
		return ndFloat32 (1.2f);
	}

	ndFloat32 distance[D_COMPOUND_STACK_DEPTH];
	const ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

//	ndFloat32 maxParam = maxT;
	ndFastRay ray (localP0, localP1);

	ndInt32 stack = 1;
	stackPool[0] = m_root;
	distance[0] = ray.BoxIntersect(m_root->m_p0, m_root->m_p1);
	while (stack) 
	{
		stack --;
		ndFloat32 dist = distance[stack];

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
				const ndVector p0 (shape->GetLocalMatrix().UntransformVector (localP0) & ndVector::m_triplexMask);
				const ndVector p1 (shape->GetLocalMatrix().UntransformVector (localP1) & ndVector::m_triplexMask);
				//ndFloat32 param = shape->RayCast (p0, p1, maxT, tmpContactOut, preFilter, body, userData);
				ndFloat32 param = shape->RayCast(callback, p0, p1, body, tmpContactOut);
				if (param < maxT) 
				{
					maxT = param;
					contactOut.m_normal = shape->GetLocalMatrix().RotateVector (tmpContactOut.m_normal);
					contactOut.m_shapeId0 = tmpContactOut.m_shapeId0;
					contactOut.m_shapeId1 = tmpContactOut.m_shapeId0;
					contactOut.m_shapeInstance0 = tmpContactOut.m_shapeInstance0;
					contactOut.m_shapeInstance1 = tmpContactOut.m_shapeInstance1;
				}
			} 
			else 
			{
				dAssert (me->m_type == m_node);
				const ndNodeBase* const left = me->m_left;
				dAssert (left);
				ndFloat32 dist1 = ray.BoxIntersect(left->m_p0, left->m_p1);
				if (dist1 < maxT) 
				{
					ndInt32 j = stack;
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist1;
					stack++;
					dAssert (stack < ndInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}
				
				const ndNodeBase* const right = me->m_right;
				dAssert (right);
				dist1 = ray.BoxIntersect(right->m_p0, right->m_p1);
				if (dist1 < maxT) 
				{
					ndInt32 j = stack;
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist1;
					stack++;
					dAssert (stack < ndInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}
			}
		}
	}
	return maxT;
}

void ndShapeCompound::BeginAddRemove()
{
	dAssert(m_myInstance);
}

ndFloat32 ndShapeCompound::CalculateSurfaceArea(ndNodeBase* const node0, ndNodeBase* const node1, ndVector& minBox, ndVector& maxBox) const
{
	minBox = node0->m_p0.GetMin(node1->m_p0);
	maxBox = node0->m_p1.GetMax(node1->m_p1);
	ndVector side0(ndVector::m_half * (maxBox - minBox));
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

void ndShapeCompound::ImproveNodeFitness(ndNodeBase* const node) const
{
	dAssert(node->m_left);
	dAssert(node->m_right);

	if (node->m_parent) 
	{
		if (node->m_parent->m_left == node) 
		{
			ndFloat32 cost0 = node->m_area;

			ndVector cost1P0;
			ndVector cost1P1;
			ndFloat32 cost1 = CalculateSurfaceArea(node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			ndVector cost2P0;
			ndVector cost2P1;
			ndFloat32 cost2 = CalculateSurfaceArea(node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			dAssert(node->m_parent->m_p0.m_w == ndFloat32(0.0f));
			dAssert(node->m_parent->m_p1.m_w == ndFloat32(0.0f));

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
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;

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
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;
			}
		}
		else 
		{
			ndFloat32 cost0 = node->m_area;

			ndVector cost1P0;
			ndVector cost1P1;
			ndFloat32 cost1 = CalculateSurfaceArea(node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			ndVector cost2P0;
			ndVector cost2P1;
			ndFloat32 cost2 = CalculateSurfaceArea(node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

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
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;

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
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;
			}
		}
	}
}

ndFloat64 ndShapeCompound::CalculateEntropy(ndInt32 count, ndNodeBase** array)
{
	ndFloat64 cost0 = ndFloat32(1.0e20f);
	ndFloat64 cost1 = cost0;
	do {
		cost1 = cost0;
		//for (dgList<ndNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
		for (ndInt32 i = 0; i < count; i ++) 
		{
			ndNodeBase* const node = array[i];
			ImproveNodeFitness(node);
		}

		cost0 = ndFloat32(0.0f);
		//for (dgList<ndNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
		for (ndInt32 i = 0; i < count; i++)
		{
			//ndNodeBase* const node = listNode->GetInfo();
			ndNodeBase* const node = array[i];
			cost0 += node->m_area;
		}
	} while (cost0 < (cost1 * ndFloat32(0.9999f)));
	return cost0;
}


ndShapeCompound::ndNodeBase* ndShapeCompound::BuildTopDown(ndNodeBase** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBase** rootNodesMemory, ndInt32& rootIndex)
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

ndShapeCompound::ndNodeBase* ndShapeCompound::BuildTopDownBig(ndNodeBase** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBase** rootNodesMemory, ndInt32& rootIndex)
{
	if (lastBox == firstBox) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, rootNodesMemory, rootIndex);
	}

	ndInt32 midPoint = -1;
	const ndFloat32 scale = ndFloat32(10.0f);
	const ndFloat32 scale2 = ndFloat32(3.0f) * scale * scale;
	const ndInt32 count = lastBox - firstBox;
	for (ndInt32 i = 0; i < count; i++) 
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
	
	ndVector minP(ndFloat32(1.0e15f));
	ndVector maxP(-ndFloat32(1.0e15f));
	for (ndInt32 i = 0; i <= count; i++) 
	{
		const ndNodeBase* const node = leafArray[firstBox + i];
		dAssert(node->m_shapeInstance);
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

void ndShapeCompound::EndAddRemove()
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

		ndInt32 stack = 1;
		ndInt32 nodeCount = 0;
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
				dAssert(stack < ndInt32 (sizeof(stackBuffer) / sizeof(stackBuffer[0])));

				stackBuffer[stack] = node->m_left;
				stack++;
				dAssert(stack < ndInt32(sizeof(stackBuffer) / sizeof(stackBuffer[0])));
			}
		}
		
		if (nodeCount)
		{
			ndFloat64 cost = CalculateEntropy(nodeCount, nodeArray);
			if ((cost > m_treeEntropy * ndFloat32(2.0f)) || (cost < m_treeEntropy * ndFloat32(0.5f))) 
			{
				ndInt32 leafNodesCount = 0;
				ndNodeBase** leafArray = dAlloca(ndNodeBase*, nodeCount + 12);
				for (ndInt32 i = 0; i < nodeCount; i++)
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
		
				class CompareNodes
				{
					public:
					ndInt32 Compare(const ndNodeBase* const elementA, const ndNodeBase* const elementB, void* const) const
					{
						ndFloat32 areaA = elementA->m_area;
						ndFloat32 areaB = elementB->m_area;
						if (areaA < areaB)
						{
							return 1;
						}
						if (areaA > areaB)
						{
							return -1;
						}
						return 0;
					}
				};
				ndSort<ndNodeBase*, CompareNodes>(leafArray, leafNodesCount);

				ndInt32 rootIndex = 0;
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
			m_treeEntropy = ndFloat32(2.0f);
		}
		
		dAssert(m_root->m_size.m_w == ndFloat32(0.0f));
		m_boxMinRadius = dMin(dMin(m_root->m_size.m_x, m_root->m_size.m_y), m_root->m_size.m_z);
		m_boxMaxRadius = ndSqrt(m_root->m_size.DotProduct(m_root->m_size).GetScalar());
		
		m_boxSize = m_root->m_size;
		m_boxOrigin = m_root->m_origin;
		MassProperties();
	}
}

ndShapeCompound::ndTreeArray::ndNode* ndShapeCompound::AddCollision(ndShapeInstance* const subInstance)
{
	dAssert(m_myInstance);
	ndNodeBase* const newNode = new ndNodeBase(subInstance);
	m_array.AddNode(newNode, m_idIndex, m_myInstance);

	m_idIndex++;
	
	if (!m_root) 
	{
		m_root = newNode;
	}
	else 
	{
		ndVector p0;
		ndVector p1;
		ndNodeBase* sibling = m_root;
		ndFloat32 surfaceArea = CalculateSurfaceArea(newNode, sibling, p0, p1);
		while (sibling->m_left && sibling->m_right) 
		{
			if (surfaceArea > sibling->m_area) 
			{
				break;
			}
	
			sibling->SetBox(p0, p1);
	
			ndVector leftP0;
			ndVector leftP1;
			ndFloat32 leftSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_left, leftP0, leftP1);
	
			ndVector rightP0;
			ndVector rightP1;
			ndFloat32 rightSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_right, rightP0, rightP1);
	
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

void ndShapeCompound::MassProperties()
{
#ifdef _DEBUG
	//	ndVector origin_ (ndVector::m_zero);
	//	ndVector inertia_ (ndVector::m_zero);
	//	ndVector crossInertia_ (ndVector::m_zero);
	//	dgPolyhedraMassProperties localData;
	//	DebugCollision (dgGetIdentityMatrix(), CalculateInertia, &localData);
	//	ndFloat32 volume_ = localData.MassProperties (origin_, inertia_, crossInertia_);
	//	dAssert (volume_ > ndFloat32 (0.0f));
	//	ndFloat32 invVolume_ = ndFloat32 (1.0f)/volume_;
	//	m_centerOfMass = origin_.Scale (invVolume_);
	//	m_centerOfMass.m_w = volume_;
	//	m_inertia = inertia_.Scale (invVolume_);
	//	m_crossInertia = crossInertia_.Scale(invVolume_);
#endif

	ndFloat32 volume = ndFloat32(0.0f);
	ndVector origin(ndVector::m_zero);
	ndVector inertiaII(ndVector::m_zero);
	ndVector inertiaIJ(ndVector::m_zero);
	ndTreeArray::Iterator iter(m_array);
	bool hasVolume = true;
	for (iter.Begin(); iter; iter++) 
	{
		ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		ndMatrix shapeInertia(collision->CalculateInertia());
		ndFloat32 shapeVolume = collision->GetVolume();

		hasVolume = hasVolume && (collision->GetShape()->GetAsShapeStaticMesh() != nullptr);
		volume += shapeVolume;
		origin += shapeInertia.m_posit.Scale(shapeVolume);
		inertiaII += ndVector(shapeInertia[0][0], shapeInertia[1][1], shapeInertia[2][2], ndFloat32(0.0f)).Scale(shapeVolume);
		inertiaIJ += ndVector(shapeInertia[1][2], shapeInertia[0][2], shapeInertia[0][1], ndFloat32(0.0f)).Scale(shapeVolume);
	}

	m_inertia = ndVector::m_zero;
	m_crossInertia = ndVector::m_zero;
	m_centerOfMass = ndVector::m_zero;
	if (hasVolume && (volume > ndFloat32(0.0f)))
	{
		ndFloat32 invVolume = ndFloat32(1.0f) / volume;
		m_inertia = inertiaII.Scale(invVolume);
		m_crossInertia = inertiaIJ.Scale(invVolume);
		m_centerOfMass = origin.Scale(invVolume);
		m_centerOfMass.m_w = volume;
		ndShape::MassProperties();
	}
}

void ndShapeCompound::SetSubShapeOwner(ndBodyKinematic* const body)
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++)
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const collision = node->GetShape();
		collision->m_ownerBody = body;
	}
}

void ndShapeCompound::ApplyScale(const ndVector& scale)
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const collision = node->GetShape();
		collision->SetGlobalScale(scale);
	}
	m_treeEntropy = ndFloat32(0.0f);
	EndAddRemove();
}

ndFloat32 ndShapeCompound::CalculateMassProperties(const ndMatrix& offset, ndVector& inertia, ndVector& crossInertia, ndVector& centerOfMass) const
{
	class ndCalculateMassProperties: public ndShapeDebugNotify
	{
		public:

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const)
		{
			m_localData.AddInertiaAndCrossFace(vertexCount, faceArray);
		}

		ndPolyhedraMassProperties m_localData;
	};

	ndCalculateMassProperties massPropretiesCalculator;
	DebugShape(offset, massPropretiesCalculator);
	return massPropretiesCalculator.m_localData.MassProperties(centerOfMass, inertia, crossInertia);
}

ndMatrix ndShapeCompound::CalculateInertiaAndCenterOfMass(const ndMatrix& alignMatrix, const ndVector& localScale, const ndMatrix& matrix) const
{
	ndVector inertiaII;
	ndVector crossInertia;
	ndVector centerOfMass;
	ndMatrix scaledMatrix(matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale(localScale.m_x);
	scaledMatrix[1] = scaledMatrix[1].Scale(localScale.m_y);
	scaledMatrix[2] = scaledMatrix[2].Scale(localScale.m_z);
	scaledMatrix = alignMatrix * scaledMatrix;

	ndFloat32 volume = CalculateMassProperties(scaledMatrix, inertiaII, crossInertia, centerOfMass);
	if (volume < D_MAX_MIN_VOLUME)
	{
		volume = D_MAX_MIN_VOLUME;
	}

	ndFloat32 invVolume = ndFloat32(1.0f) / volume;
	centerOfMass = centerOfMass.Scale(invVolume);
	inertiaII = inertiaII.Scale(invVolume);
	crossInertia = crossInertia.Scale(invVolume);
	ndMatrix inertia(dGetIdentityMatrix());
	inertia[0][0] = inertiaII[0];
	inertia[1][1] = inertiaII[1];
	inertia[2][2] = inertiaII[2];
	inertia[0][1] = crossInertia[2];
	inertia[1][0] = crossInertia[2];
	inertia[0][2] = crossInertia[1];
	inertia[2][0] = crossInertia[1];
	inertia[1][2] = crossInertia[0];
	inertia[2][1] = crossInertia[0];
	inertia[3] = centerOfMass;
	return inertia;
}

void ndShapeCompound::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShape::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	nd::TiXmlElement* const subShapedNode = new nd::TiXmlElement("ndCompoundsSubShaped");
	childNode->LinkEndChild(subShapedNode);
	ndLoadSaveBase::ndSaveDescriptor subShapeDesc(desc);
	subShapeDesc.m_rootNode = subShapedNode;
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++)
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const instance = node->GetShape();
		subShapeDesc.m_shapeNodeHash = subShapeDesc.m_shapeMap->Find(instance->GetShape())->GetInfo();
		instance->Save(subShapeDesc);
	}
}