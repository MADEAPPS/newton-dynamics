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
#include "dgWorld.h"
#include "dgCollisionBVH.h"
#include "dgCollisionConvex.h"
#include "dgCollisionCompound.h"
#include "dgCollisionInstance.h"
#include "dgCollisionUserMesh.h"
#include "dgCollisionHeightField.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_MAX_MIN_VOLUME				dgFloat32 (1.0e-3f)


dgVector dgCollisionCompound::m_padding (dgFloat32 (1.0e-3f)); 

class dgCollisionCompound::dgHeapNodePair
{
	public:
	dgNodeBase* m_nodeA;
	dgNodeBase* m_nodeB;
};


dgCollisionCompound::dgTreeArray::dgTreeArray (dgMemoryAllocator* const allocator)
	:dgTree<dgNodeBase*, dgInt32>(allocator)
{
}

void dgCollisionCompound::dgTreeArray::AddNode (dgNodeBase* const node, dgInt32 index, const dgCollisionInstance* const parent) 
{
	dgTreeNode* const myNode = dgTree<dgNodeBase*, dgInt32>::Insert(node, index);
	node->m_myNode = myNode;
	node->m_shape->m_parent = parent;
	node->m_shape->m_subCollisionHandle = myNode;
}


dgCollisionCompound::dgOOBBTestData::dgOOBBTestData (const dgMatrix& matrix)
	:m_matrix (matrix)
{
	m_absMatrix[0] = m_matrix[0].Abs();
	m_absMatrix[1] = m_matrix[1].Abs();
	m_absMatrix[2] = m_matrix[2].Abs();
	m_absMatrix[3] = dgVector::m_wOne;

	dgInt32 index = 0;
	for (dgInt32 i = 0; i < 3; i ++) {
		dgVector dir(dgFloat32 (0.0f));
		dir[i] = dgFloat32 (1.0f);
		for (dgInt32 j = 0; j < 3; j ++) {
			dgVector axis (dir * m_matrix[j]);
			m_crossAxis[index] = axis;
			m_crossAxisAbs[index] = axis.Abs();
			m_crossAxisDotAbs[index] = matrix.UnrotateVector (axis).Abs();
			index ++;
		}
	}

	dgVector tmp;
	dgVector::Transpose4x4 (m_crossAxis[0], m_crossAxis[1], m_crossAxis[2], m_crossAxis[3], m_crossAxis[0], m_crossAxis[1], m_crossAxis[2], m_crossAxis[3]);
	dgVector::Transpose4x4 (m_crossAxis[3], m_crossAxis[4], m_crossAxis[5], m_crossAxis[6], m_crossAxis[4], m_crossAxis[5], m_crossAxis[6], m_crossAxis[7]);
	dgVector::Transpose4x4 (m_crossAxis[6], m_crossAxis[7], m_crossAxis[8], tmp,			m_crossAxis[8], m_crossAxis[8], m_crossAxis[8], m_crossAxis[8]);

	dgVector::Transpose4x4 (m_crossAxisAbs[0], m_crossAxisAbs[1], m_crossAxisAbs[2], m_crossAxisAbs[3], m_crossAxisAbs[0], m_crossAxisAbs[1], m_crossAxisAbs[2], m_crossAxisAbs[3]);
	dgVector::Transpose4x4 (m_crossAxisAbs[3], m_crossAxisAbs[4], m_crossAxisAbs[5], m_crossAxisAbs[6], m_crossAxisAbs[4], m_crossAxisAbs[5], m_crossAxisAbs[6], m_crossAxisAbs[7]);
	dgVector::Transpose4x4 (m_crossAxisAbs[6], m_crossAxisAbs[7], m_crossAxisAbs[8], tmp,			    m_crossAxisAbs[8], m_crossAxisAbs[8], m_crossAxisAbs[8], m_crossAxisAbs[8]);

	dgVector::Transpose4x4 (m_crossAxisDotAbs[0], m_crossAxisDotAbs[1], m_crossAxisDotAbs[2], m_crossAxisDotAbs[3], m_crossAxisDotAbs[0], m_crossAxisDotAbs[1], m_crossAxisDotAbs[2], m_crossAxisDotAbs[3]);
	dgVector::Transpose4x4 (m_crossAxisDotAbs[3], m_crossAxisDotAbs[4], m_crossAxisDotAbs[5], m_crossAxisDotAbs[6], m_crossAxisDotAbs[4], m_crossAxisDotAbs[5], m_crossAxisDotAbs[6], m_crossAxisDotAbs[7]);
	dgVector::Transpose4x4 (m_crossAxisDotAbs[6], m_crossAxisDotAbs[7], m_crossAxisDotAbs[8], tmp,				    m_crossAxisDotAbs[8], m_crossAxisDotAbs[8], m_crossAxisDotAbs[8], m_crossAxisDotAbs[8]);
}


dgCollisionCompound::dgOOBBTestData::dgOOBBTestData (const dgMatrix& matrix, const dgVector& localOrigin, const dgVector& localSize)
	:m_matrix (matrix)
	,m_origin(localOrigin)
	,m_size(localSize)
	,m_localP0 (localOrigin - localSize)
	,m_localP1 (localOrigin + localSize)
{
	m_absMatrix[0] = m_matrix[0].Abs();
	m_absMatrix[1] = m_matrix[1].Abs();
	m_absMatrix[2] = m_matrix[2].Abs();
	m_absMatrix[3] = dgVector::m_wOne;

	dgInt32 index = 0;
	for (dgInt32 i = 0; i < 3; i ++) {
		dgVector dir(dgFloat32 (0.0f));
		dir[i] = dgFloat32 (1.0f);
		for (dgInt32 j = 0; j < 3; j ++) {
			m_crossAxis[index] = dir * m_matrix[j];
			index ++;
		}
	}

	dgVector size (m_absMatrix.RotateVector(m_size));
	dgVector origin (m_matrix.TransformVector (m_origin));
	m_aabbP0 = origin - size;
	m_aabbP1 = origin + size;

	index = 0;
	dgVector extends[9];
	for (dgInt32 i = 0; i < 3; i ++) {
		for (dgInt32 j = 0; j < 3; j ++) {
			const dgVector& axis = m_crossAxis[index];
			dgAssert (axis.m_w == dgFloat32 (0.0f));
			dgVector tmp (m_matrix.UnrotateVector(axis));
			dgVector d (m_size.DotProduct4(tmp.Abs()) + m_padding);
			dgVector c (origin.DotProduct4(axis));
			extends[index] = ((c - d).PackLow (c + d));
			m_crossAxisAbs[index] = axis.Abs();
			index ++;
		}
	}

	dgVector tmp;
	dgVector::Transpose4x4 (m_crossAxis[0], m_crossAxis[1], m_crossAxis[2], m_crossAxis[3], m_crossAxis[0], m_crossAxis[1], m_crossAxis[2], m_crossAxis[3]);
	dgVector::Transpose4x4 (m_crossAxis[3], m_crossAxis[4], m_crossAxis[5], m_crossAxis[6], m_crossAxis[4], m_crossAxis[5], m_crossAxis[6], m_crossAxis[7]);
	dgVector::Transpose4x4 (m_crossAxis[6], m_crossAxis[7], m_crossAxis[8], tmp,				m_crossAxis[8], m_crossAxis[8], m_crossAxis[8], m_crossAxis[8]);

	dgVector::Transpose4x4 (m_crossAxisAbs[0], m_crossAxisAbs[1], m_crossAxisAbs[2], m_crossAxisAbs[3], m_crossAxisAbs[0], m_crossAxisAbs[1], m_crossAxisAbs[2], m_crossAxisAbs[3]);
	dgVector::Transpose4x4 (m_crossAxisAbs[3], m_crossAxisAbs[4], m_crossAxisAbs[5], m_crossAxisAbs[6], m_crossAxisAbs[4], m_crossAxisAbs[5], m_crossAxisAbs[6], m_crossAxisAbs[7]);
	dgVector::Transpose4x4 (m_crossAxisAbs[6], m_crossAxisAbs[7], m_crossAxisAbs[8], tmp,				    m_crossAxisAbs[8], m_crossAxisAbs[8], m_crossAxisAbs[8], m_crossAxisAbs[8]);

	dgVector::Transpose4x4 (m_extendsMinX[0], m_extendsMaxX[0], tmp, tmp, extends[0], extends[1], extends[2], extends[3]);
	dgVector::Transpose4x4 (m_extendsMinX[1], m_extendsMaxX[1], tmp, tmp, extends[4], extends[5], extends[6], extends[7]);
	dgVector::Transpose4x4 (m_extendsMinX[2], m_extendsMaxX[2], tmp, tmp, extends[8], extends[8], extends[8], extends[8]);
}


dgCollisionCompound::dgNodeBase::dgNodeBase () 
	:m_left(NULL) 
	,m_right(NULL)
	,m_parent(NULL)
	,m_shape(NULL)
	,m_myNode(NULL)
{
}

dgCollisionCompound::dgNodeBase::dgNodeBase (const dgNodeBase& copyFrom)
	:m_p0(copyFrom.m_p0)
	,m_p1(copyFrom.m_p1)
	,m_size(copyFrom.m_size)
	,m_origin(copyFrom.m_origin)
	,m_area(copyFrom.m_area)
	,m_type(copyFrom.m_type)
	,m_left(NULL)
	,m_right(NULL)
	,m_parent(NULL)
	,m_shape(copyFrom.m_shape)
	,m_myNode(NULL)
{
	dgAssert (!copyFrom.m_shape);
}


dgCollisionCompound::dgNodeBase::dgNodeBase (dgCollisionInstance* const instance)
	:m_type(m_leaf)
	,m_left(NULL) 
	,m_right(NULL)
	,m_parent(NULL)
	,m_shape(new (instance->GetAllocator()) dgCollisionInstance (*instance))
	,m_myNode(NULL)
{	
	CalculateAABB();
}


dgCollisionCompound::dgNodeBase::dgNodeBase (dgNodeBase* const left, dgNodeBase* const right)
	:m_type(m_node)
	,m_left(left)
	,m_right(right)
	,m_parent(NULL)
	,m_shape(NULL)
	,m_myNode(NULL)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	dgVector p0 (left->m_p0.GetMin(right->m_p0));
	dgVector p1 (left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}

dgCollisionCompound::dgNodeBase::~dgNodeBase()
{
	if (m_shape) {
		m_shape->Release ();
	}
	if (m_left) {
		delete m_left;
	}
	if (m_right) {
		delete m_right;
	}
}


void dgCollisionCompound::dgNodeBase::SetBox (const dgVector& p0, const dgVector& p1)
{
	m_p0 = p0;
	m_p1 = p1;
	dgAssert (m_p0.m_w == dgFloat32 (0.0f));
	dgAssert (m_p1.m_w == dgFloat32 (0.0f));
	m_size = (m_p1 - m_p0).CompProduct4 (dgVector::m_half);
	m_origin = (m_p1 + m_p0).CompProduct4 (dgVector::m_half);
	m_area = m_size.DotProduct4(m_size.ShiftTripleRight()).m_x;
}

void dgCollisionCompound::dgNodeBase::CalculateAABB()
{
	dgVector p0;
	dgVector p1;
	m_shape->CalcAABB(m_shape->GetLocalMatrix (), p0, p1);
	SetBox (p0, p1);
}

bool dgCollisionCompound::dgNodeBase::BoxTest (const dgOOBBTestData& data) const
{
	if (dgOverlapTest (data.m_aabbP0, data.m_aabbP1, m_p0, m_p1)) {
		dgVector origin (data.m_matrix.UntransformVector(m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);
		if (dgOverlapTest (p0, p1, data.m_localP0, data.m_localP1)) {
			dgVector size_x (m_size.m_x);
			dgVector size_y (m_size.m_y);
			dgVector size_z (m_size.m_z);

			dgVector origin_x (m_origin.m_x);
			dgVector origin_y (m_origin.m_y);
			dgVector origin_z (m_origin.m_z);

			bool ret = true;
			for (dgInt32 i = 0; (i < 3) && ret; i ++) {
				const dgInt32 j = i * 3;
				dgVector c (origin_x.CompProduct4(data.m_crossAxis[j + 0]) + origin_y.CompProduct4(data.m_crossAxis[j + 1]) + origin_z.CompProduct4(data.m_crossAxis[j + 2]));
				dgVector d (size_x.CompProduct4(data.m_crossAxisAbs[j + 0]) + size_y.CompProduct4(data.m_crossAxisAbs[j + 1]) + size_z.CompProduct4(data.m_crossAxisAbs[j + 2]) + m_padding); 
				dgVector x0 (c - d);
				dgVector x1 (c + d);
				dgVector box0 (x0 - data.m_extendsMaxX[i]);
				dgVector box1 (x1 - data.m_extendsMinX[i]);
				dgVector test (box0.CompProduct4((box1)));
				ret = (test.GetSignMask() & 0x0f) == 0x0f;
			}
			return ret;
		}
	}
	return false;
}


bool dgCollisionCompound::dgNodeBase::BoxTest (const dgOOBBTestData& data, const dgNodeBase* const otherNode) const
{
	dgVector otherOrigin (data.m_matrix.TransformVector(otherNode->m_origin));
	dgVector otherSize (data.m_absMatrix.RotateVector(otherNode->m_size));
	dgVector otherP0 ((otherOrigin - otherSize) & dgVector::m_triplexMask);
	dgVector otherP1 ((otherOrigin + otherSize) & dgVector::m_triplexMask);
	if (dgOverlapTest (m_p0, m_p1, otherP0, otherP1)) {
		dgVector origin (data.m_matrix.UntransformVector(m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);
		if (dgOverlapTest (p0, p1, otherNode->m_p0, otherNode->m_p1)) {

			dgVector size0_x (m_size.m_x);
			dgVector size0_y (m_size.m_y);
			dgVector size0_z (m_size.m_z);

			dgVector origin0_x (m_origin.m_x);
			dgVector origin0_y (m_origin.m_y);
			dgVector origin0_z (m_origin.m_z);

			dgVector size1_x (otherNode->m_size.m_x);
			dgVector size1_y (otherNode->m_size.m_y);
			dgVector size1_z (otherNode->m_size.m_z);

			dgVector origin1_x (otherOrigin.m_x);
			dgVector origin1_y (otherOrigin.m_y);
			dgVector origin1_z (otherOrigin.m_z);

			bool ret = true;
			for (dgInt32 j = 0; (j < 9) && ret; j += 3) {
				dgVector c0 (origin0_x.CompProduct4(data.m_crossAxis[j + 0]) + origin0_y.CompProduct4(data.m_crossAxis[j + 1]) + origin0_z.CompProduct4(data.m_crossAxis[j + 2]));
				dgVector d0 (size0_x.CompProduct4(data.m_crossAxisAbs[j + 0]) + size0_y.CompProduct4(data.m_crossAxisAbs[j + 1]) + size0_z.CompProduct4(data.m_crossAxisAbs[j + 2]) + m_padding); 
				dgVector x0 (c0 - d0);
				dgVector x1 (c0 + d0);

				dgVector c1 (origin1_x.CompProduct4(data.m_crossAxis[j + 0]) + origin1_y.CompProduct4(data.m_crossAxis[j + 1]) + origin1_z.CompProduct4(data.m_crossAxis[j + 2]));
				dgVector d1 (size1_x.CompProduct4(data.m_crossAxisDotAbs[j + 0]) + size1_y.CompProduct4(data.m_crossAxisDotAbs[j + 1]) + size1_z.CompProduct4(data.m_crossAxisDotAbs[j + 2]) + m_padding); 
				dgVector z0 (c1 - d1);
				dgVector z1 (c1 + d1);

				dgVector box0 (x0 - z1);
				dgVector box1 (x1 - z0);
				dgVector test (box0.CompProduct4((box1)));
				ret = (test.GetSignMask() & 0x0f) == 0x0f;
			}
			return ret;
		}
	}
	return false;
}

dgFloat32 dgCollisionCompound::dgNodeBase::RayBoxDistance (const dgOOBBTestData& data, const dgFastRayTest& myRay, const dgFastRayTest& otherRay, const dgNodeBase* const otherNode) const
{
	dgVector otherOrigin (data.m_matrix.TransformVector(otherNode->m_origin));
	dgVector otherSize (data.m_absMatrix.RotateVector(otherNode->m_size));
	dgVector otherP0 ((otherOrigin - otherSize) & dgVector::m_triplexMask);
	dgVector otherP1 ((otherOrigin + otherSize) & dgVector::m_triplexMask);
	dgVector minBox (m_p0 - otherP1);
	dgVector maxBox (m_p1 - otherP0);
	dgFloat32 dist = myRay.BoxIntersect (minBox, maxBox);
	if (dist <= 1.0f) {
		dgVector origin (data.m_matrix.UntransformVector(m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);
		dgVector minBox1 (p0 - otherNode->m_p1);
		dgVector maxBox1 (p1 - otherNode->m_p0);
		dgFloat32 dist1 = otherRay.BoxIntersect (minBox1, maxBox1);
		dist = dgMax(dist, dist1);
	}

	return dist;
}

class dgCollisionCompound::dgSpliteInfo
{
	public:
	dgSpliteInfo (dgNodeBase** const boxArray, dgInt32 boxCount)
	{
		dgVector minP ( dgFloat32 (1.0e15f)); 
		dgVector maxP (-dgFloat32 (1.0e15f)); 

		if (boxCount == 2) {
			m_axis = 1;
			for (dgInt32 i = 0; i < boxCount; i ++) {
				dgNodeBase* const node = boxArray[i];
				dgAssert (node->m_type == m_leaf);
				minP = minP.GetMin (node->m_p0); 
				maxP = maxP.GetMax (node->m_p1); 
			}
		} else {
			dgVector median (dgFloat32 (0.0f));
			dgVector varian (dgFloat32 (0.0f));

			for (dgInt32 i = 0; i < boxCount; i ++) {
				dgNodeBase* const node = boxArray[i];
				dgAssert (node->m_type == m_leaf);
				minP = minP.GetMin (node->m_p0); 
				maxP = maxP.GetMax (node->m_p1); 
				dgVector p ((node->m_p0 + node->m_p1).CompProduct4(dgVector::m_half));
				median += p;
				varian += p.CompProduct4(p);
			}

			varian = varian.Scale4 (dgFloat32 (boxCount)) - median.CompProduct4(median);

			dgInt32 index = 0;
			dgFloat32 maxVarian = dgFloat32 (-1.0e10f);
			for (dgInt32 i = 0; i < 3; i ++) {
				if (varian[i] > maxVarian) {
					index = i;
					maxVarian = varian[i];
				}
			}

			dgVector center = median.Scale4 (dgFloat32 (1.0f) / dgFloat32 (boxCount));

			dgFloat32 test = center[index];

			dgInt32 i0 = 0;
			dgInt32 i1 = boxCount - 1;
			do {    
				for (; i0 <= i1; i0 ++) {
					dgNodeBase* const node = boxArray[i0];
					dgFloat32 val = (node->m_p0[index] + node->m_p1[index]) * dgFloat32 (0.5f);
					if (val > test) {
						break;
					}
				}

				for (; i1 >= i0; i1 --) {
					dgNodeBase* const node = boxArray[i1];
					dgFloat32 val = (node->m_p0[index] + node->m_p1[index]) * dgFloat32 (0.5f);
					if (val < test) {
						break;
					}
				}

				if (i0 < i1)	{
					dgSwap(boxArray[i0], boxArray[i1]);
					i0++; 
					i1--;
				}

			} while (i0 <= i1);

			if (i0 > 0){
				i0 --;
			}
			if ((i0 + 1) >= boxCount) {
				i0 = boxCount - 2;
			}

			m_axis = i0 + 1;
		}

		dgAssert (maxP.m_x - minP.m_x >= dgFloat32 (0.0f));
		dgAssert (maxP.m_y - minP.m_y >= dgFloat32 (0.0f));
		dgAssert (maxP.m_z - minP.m_z >= dgFloat32 (0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	dgInt32 m_axis;
	dgVector m_p0;
	dgVector m_p1;
};




dgCollisionCompound::dgCollisionCompound(dgWorld* const world)
	:dgCollision (world->GetAllocator(), 0, m_compoundCollision) 
	,m_boxMinRadius (dgFloat32 (0.0f))
	,m_boxMaxRadius (dgFloat32 (0.0f))
	,m_treeEntropy (dgFloat32 (0.0f))
	,m_world(world)
	,m_root(NULL)
	,m_myInstance(NULL)
	,m_criticalSectionLock()
	,m_array (world->GetAllocator())
	,m_idIndex(0)
{
	m_rtti |= dgCollisionCompound_RTTI;
}

dgCollisionCompound::dgCollisionCompound (const dgCollisionCompound& source, const dgCollisionInstance* const myInstance)
	:dgCollision (source) 
	,m_boxMinRadius(source.m_boxMinRadius)
	,m_boxMaxRadius(source.m_boxMaxRadius)
	,m_treeEntropy(source.m_treeEntropy)
	,m_world (source.m_world)	
	,m_root(NULL)
	,m_myInstance(myInstance)
	,m_criticalSectionLock()
	,m_array (source.GetAllocator())
	,m_idIndex(source.m_idIndex)
{
	m_rtti |= dgCollisionCompound_RTTI;

	dgTreeArray::Iterator iter (source.m_array);
	for (iter.Begin(); iter; iter ++) {
		dgNodeBase* const node = iter.GetNode()->GetInfo();
		dgNodeBase* const newNode = new (m_allocator) dgNodeBase (node->GetShape());
		m_array.AddNode(newNode, iter.GetNode()->GetKey(), m_myInstance);
	}

	if (source.m_root) {
		dgNodeBase* pool[DG_COMPOUND_STACK_DEPTH];
		dgNodeBase* parents[DG_COMPOUND_STACK_DEPTH];
		pool[0] = source.m_root;
		parents[0] = NULL;
		dgInt32 stack = 1;
		while (stack) {
			stack --;
			dgNodeBase* const sourceNode = pool [stack];

			dgNodeBase* parent = NULL;
			if (sourceNode->m_type == m_node) {
				parent = new (m_allocator) dgNodeBase (*sourceNode);
				if (!sourceNode->m_parent) {
					m_root = parent;
				} else {
					parent->m_parent = parents[stack];
					if (parent->m_parent) {
						if (sourceNode->m_parent->m_left == sourceNode) {
							parent->m_parent->m_left = parent;
						} else {
							dgAssert (sourceNode->m_parent->m_right == sourceNode);
							parent->m_parent->m_right = parent;
						}
					}
				}
			} else {
				//dgNodeBase* const node = m_array.Find (sourceNode->m_shape)->GetInfo();
				dgNodeBase* const node = m_array.Find (sourceNode->m_myNode->GetKey())->GetInfo();
				dgAssert (node);
				node->m_parent = parents[stack];
				if (node->m_parent) {
					if (sourceNode->m_parent->m_left == sourceNode) {
						node->m_parent->m_left = node;
					} else {
						dgAssert (sourceNode->m_parent->m_right == sourceNode);
						node->m_parent->m_right = node;
					}
				} else {
					m_root = node;
				}
			}

			if (sourceNode->m_left) {
				parents[stack] = parent;
				pool[stack] = sourceNode->m_left;
				stack ++;
				dgAssert (stack < DG_COMPOUND_STACK_DEPTH);
			}

			if (sourceNode->m_right) {
				parents[stack] = parent;
				pool[stack] = sourceNode->m_right;
				stack ++;
				dgAssert (stack < DG_COMPOUND_STACK_DEPTH);
			}
		}
	}
}

dgCollisionCompound::dgCollisionCompound (dgWorld* const world, dgDeserialize deserialization, void* const userData, const dgCollisionInstance* const myInstance)
	:dgCollision (world, deserialization, userData)
	,m_boxMinRadius (dgFloat32 (0.0f))
	,m_boxMaxRadius (dgFloat32 (0.0f))
	,m_treeEntropy (dgFloat32 (0.0f))
	,m_world(world)
	,m_root(NULL)
	,m_myInstance(myInstance)
	,m_criticalSectionLock()
	,m_array (world->GetAllocator())
	,m_idIndex(0)
{
	dgAssert (m_rtti | dgCollisionCompound_RTTI);

	dgInt32 count;
	deserialization (userData, &count, sizeof (count));
	BeginAddRemove ();
	for (dgInt32 i = 0; i < count; i ++) {
		dgCollisionInstance* const collision = new  (world->GetAllocator()) dgCollisionInstance (world, deserialization, userData);
		AddCollision (collision); 
		collision->Release();
	}
	EndAddRemove();
}


dgCollisionCompound::~dgCollisionCompound()
{
	if (m_root) {
		delete m_root;
	}
}

void dgCollisionCompound::SetParent (const dgCollisionInstance* const instance)
{
	m_myInstance = instance;
}

void dgCollisionCompound::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}

void dgCollisionCompound::GetAABB (dgVector& boxMin, dgVector& boxMax) const
{
	if (m_root) {
		boxMin = m_root->m_p0;
		boxMax = m_root->m_p1;
	} else {
		boxMin = dgVector (dgFloat32 (0.0f));
		boxMax = dgVector (dgFloat32 (0.0f));
	}
}


dgInt32 dgCollisionCompound::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}



void dgCollisionCompound::CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	if (m_root) {
		dgVector origin (matrix.TransformVector(m_root->m_origin));
		dgVector size (matrix.m_front.Abs().Scale4(m_root->m_size.m_x) + matrix.m_up.Abs().Scale4(m_root->m_size.m_y) + matrix.m_right.Abs().Scale4(m_root->m_size.m_z));

		size -= dgCollisionInstance::m_padding;
		p0 = (origin - size) & dgVector::m_triplexMask;
		p1 = (origin + size) & dgVector::m_triplexMask;
	} else {
		p0 = dgVector (dgFloat32 (0.0f));
		p1 = dgVector (dgFloat32 (0.0f));
	}
}



dgInt32 dgCollisionCompound::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	return 0;
}


void dgCollisionCompound::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTreeArray::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgCollisionInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		collision->DebugCollision (matrix, callback, userData);
	}
}


dgFloat32 dgCollisionCompound::RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	if (!m_root) {
		return dgFloat32 (1.2f);
	}

	dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

//	dgFloat32 maxParam = maxT;
	dgFastRayTest ray (localP0, localP1);

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	distance[0] = ray.BoxIntersect(m_root->m_p0, m_root->m_p1);
	while (stack) {
		stack --;
		dgFloat32 dist = distance[stack];

		if (dist > maxT) {
			break;
		} else {
			const dgNodeBase* const me = stackPool[stack];
			dgAssert (me);
			if (me->m_type == m_leaf) {
				dgContactPoint tmpContactOut;
				dgCollisionInstance* const shape = me->GetShape();
				dgVector p0 (shape->GetLocalMatrix().UntransformVector (localP0));
				dgVector p1 (shape->GetLocalMatrix().UntransformVector (localP1));
				dgFloat32 param = shape->RayCast (p0, p1, maxT, tmpContactOut, preFilter, body, userData);
				if (param < maxT) {
					maxT = param;
					contactOut.m_normal = shape->GetLocalMatrix().RotateVector (tmpContactOut.m_normal);
					contactOut.m_shapeId0 = tmpContactOut.m_shapeId0;
					contactOut.m_shapeId1 = tmpContactOut.m_shapeId0;
					contactOut.m_collision0 = tmpContactOut.m_collision0;
					contactOut.m_collision1 = tmpContactOut.m_collision1;
				}

			} else {
				dgAssert (me->m_type == m_node);
				const dgNodeBase* const left = me->m_left;
				dgAssert (left);
				dgFloat32 dist = ray.BoxIntersect(left->m_p0, left->m_p1);
				if (dist < maxT) {
					dgInt32 j = stack;
					for ( ; j && (dist > distance[j - 1]); j --) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}

				const dgNodeBase* const right = me->m_right;
				dgAssert (right);
				dist = ray.BoxIntersect(right->m_p0, right->m_p1);
				if (dist < maxT) {
					dgInt32 j = stack;
					for ( ; j && (dist > distance[j - 1]); j --) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}
			}
		}
	}
	return maxT;
}



dgFloat32 dgCollisionCompound::GetVolume () const
{
	dgFloat32 volume = dgFloat32 (0.0f);
	dgTreeArray::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgCollisionConvex* const collision = (dgCollisionConvex*)iter.GetNode()->GetInfo()->GetShape()->GetChildShape();
		volume += collision->GetVolume();
    }
	return volume;
}

dgFloat32 dgCollisionCompound::GetBoxMinRadius () const
{
	return m_boxMinRadius;
}

dgFloat32 dgCollisionCompound::GetBoxMaxRadius () const
{
	return m_boxMaxRadius;
}



dgVector dgCollisionCompound::CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dgVector& plane, const dgCollisionInstance& parentScale) const
{
	dgVector totalVolume (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgTreeArray::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		const dgCollisionInstance* const childInstance = iter.GetNode()->GetInfo()->GetShape();
		dgCollisionConvex* const collision = (dgCollisionConvex*)childInstance->GetChildShape();
		dgMatrix matrix (childInstance->m_localMatrix * globalMatrix);
		dgVector vol (collision->CalculateVolumeIntegral (matrix, plane, *childInstance));
		totalVolume.m_x += vol.m_x * vol.m_w;
		totalVolume.m_y += vol.m_y * vol.m_w;
		totalVolume.m_z += vol.m_z * vol.m_w;
		totalVolume.m_w	+= vol.m_w;
    }

	dgFloat32 scale = dgFloat32 (0.0f);
	if (m_root) {
		scale = dgFloat32 (1.0f) / (totalVolume.m_w + dgFloat32 (1.0e-6f));
	}
	totalVolume.m_x *= scale;
	totalVolume.m_y *= scale;
	totalVolume.m_z *= scale;

	return totalVolume;
}

dgFloat32 dgCollisionCompound::CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const
{
	dgPolyhedraMassProperties localData;
	DebugCollision (offset, CalculateInertia, &localData);
	return localData.MassProperties (centerOfMass, inertia, crossInertia);
}


dgMatrix dgCollisionCompound::CalculateInertiaAndCenterOfMass (const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const
{
	dgVector inertiaII;
	dgVector crossInertia;
	dgVector centerOfMass;
	dgMatrix scaledMatrix(matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale3(localScale.m_x);
	scaledMatrix[1] = scaledMatrix[1].Scale3(localScale.m_y);
	scaledMatrix[2] = scaledMatrix[2].Scale3(localScale.m_z);
	scaledMatrix = m_alignMatrix * scaledMatrix;

	dgFloat32 volume = CalculateMassProperties (scaledMatrix, inertiaII, crossInertia, centerOfMass);
	if (volume < DG_MAX_MIN_VOLUME) {
		volume = DG_MAX_MIN_VOLUME;
	}

	dgFloat32 invVolume = dgFloat32 (1.0f) / volume;
	centerOfMass = centerOfMass.Scale3(invVolume);
	inertiaII = inertiaII.Scale3 (invVolume);
	crossInertia = crossInertia.Scale3 (invVolume);
	dgMatrix inertia (dgGetIdentityMatrix());
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


void dgCollisionCompound::CalculateInertia (void* userData, int indexCount, const dgFloat32* const faceVertex, int faceId)
{
	dgPolyhedraMassProperties& localData = *((dgPolyhedraMassProperties*) userData);
	localData.AddInertiaAndCrossFace(indexCount, faceVertex);
}


void dgCollisionCompound::MassProperties ()
{
#ifdef _DEBUG
//	dgVector origin_ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
//	dgVector inertia_ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
//	dgVector crossInertia_ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
//	dgPolyhedraMassProperties localData;
//	DebugCollision (dgGetIdentityMatrix(), CalculateInertia, &localData);
//	dgFloat32 volume_ = localData.MassProperties (origin_, inertia_, crossInertia_);
//	dgAssert (volume_ > dgFloat32 (0.0f));
//	dgFloat32 invVolume_ = dgFloat32 (1.0f)/volume_;
//	m_centerOfMass = origin_.Scale3 (invVolume_);
//	m_centerOfMass.m_w = volume_;
//	m_inertia = inertia_.Scale3 (invVolume_);
//	m_crossInertia = crossInertia_.Scale3(invVolume_);
#endif


	dgFloat32 volume = dgFloat32 (0.0f);
	dgVector origin (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector inertiaII (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector inertiaIJ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgTreeArray::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgCollisionInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		dgMatrix shapeInertia (collision->CalculateInertia());
		dgFloat32 shapeVolume = collision->GetVolume();

		volume += shapeVolume;
		origin += shapeInertia.m_posit.Scale3(shapeVolume);
		inertiaII += dgVector (shapeInertia[0][0], shapeInertia[0][0], shapeInertia[0][0], dgFloat32 (0.0f)).Scale3 (shapeVolume);
		inertiaIJ += dgVector (shapeInertia[1][2], shapeInertia[0][2], shapeInertia[0][1], dgFloat32 (0.0f)).Scale3 (shapeVolume);
	}
	if (volume > dgFloat32 (0.0f)) { 
		dgFloat32 invVolume = dgFloat32 (1.0f)/volume;
		m_inertia = inertiaII.Scale3 (invVolume);
		m_crossInertia = inertiaIJ.Scale3 (invVolume);
		m_centerOfMass = origin.Scale3 (invVolume);
		m_centerOfMass.m_w = volume;
	}

	dgCollision::MassProperties ();
}

void dgCollisionCompound::ApplyScale (const dgVector& scale)
{
	dgTreeArray::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgNodeBase* const node = iter.GetNode()->GetInfo();
		dgCollisionInstance* const collision = node->GetShape();
		collision->SetGlobalScale (scale);
	}
	m_treeEntropy = dgFloat32 (0.0f);
	EndAddRemove ();
}


void dgCollisionCompound::BeginAddRemove ()
{
}


dgCollisionCompound::dgNodeBase* dgCollisionCompound::BuildTopDown (dgNodeBase** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgList<dgNodeBase*>::dgListNode** const nextNode)
{
	dgAssert (firstBox >= 0);
	dgAssert (lastBox >= 0);

	if (lastBox == firstBox) {
		return leafArray[firstBox];
	} else {
		dgSpliteInfo info (&leafArray[firstBox], lastBox - firstBox + 1);

		dgNodeBase* const parent = (*nextNode)->GetInfo();
		parent->m_parent = NULL;
		*nextNode = (*nextNode)->GetNext();

		parent->SetBox (info.m_p0, info.m_p1);
		parent->m_right = BuildTopDown (leafArray, firstBox + info.m_axis, lastBox, nextNode);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDown (leafArray, firstBox, firstBox + info.m_axis - 1, nextNode);
		parent->m_left->m_parent = parent;
		return parent;
	}
}

dgFloat64 dgCollisionCompound::CalculateEntropy (dgList<dgNodeBase*>& list)
{
	dgFloat64 cost0 = dgFloat32 (1.0e20f);
	dgFloat64 cost1 = cost0;
	do {
		cost1 = cost0;
		for (dgList<dgNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
			dgNodeBase* const node = listNode->GetInfo();
			ImproveNodeFitness (node);
		}

		cost0 = dgFloat32 (0.0f);
		for (dgList<dgNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
			dgNodeBase* const node = listNode->GetInfo();
			cost0 += node->m_area;
		}
	} while (cost0 < (cost1 * dgFloat32 (0.9999f)));
	return cost0;
}

void dgCollisionCompound::EndAddRemove (bool flushCache)
{
	if (m_root) {
		dgWorld* const world = m_world;
		dgThreadHiveScopeLock lock (world, &m_criticalSectionLock);

		dgTreeArray::Iterator iter (m_array);
		for (iter.Begin(); iter; iter ++) {
			dgNodeBase* const node = iter.GetNode()->GetInfo();
			node->CalculateAABB();
		}

		dgList<dgNodeBase*> list (GetAllocator());
		dgList<dgNodeBase*> stack (GetAllocator());
		stack.Append(m_root);
		while (stack.GetCount()) {
			dgList<dgNodeBase*>::dgListNode* const stackNode = stack.GetLast();
			dgNodeBase* const node = stackNode->GetInfo();
			stack.Remove(stackNode);

			//if (node->m_type == m_node) {
			//	list.Append(node);
			//}

			if (node->m_type == m_node) {
				list.Append(node);
				stack.Append(node->m_right);
				stack.Append(node->m_left);
			} 
		}

		if (list.GetCount()) {
			dgFloat64 cost = CalculateEntropy (list);
			if ((cost > m_treeEntropy * dgFloat32 (2.0f)) || (cost < m_treeEntropy * dgFloat32 (0.5f))) {
				dgInt32 count = list.GetCount() * 2 + 12;
				dgInt32 leafNodesCount = 0;
				dgStack<dgNodeBase*> leafArray(count);
				for (dgList<dgNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
					dgNodeBase* const node = listNode->GetInfo();
					if (node->m_left->m_type == m_leaf) {
						leafArray[leafNodesCount] = node->m_left;
						leafNodesCount ++;
					}
					if (node->m_right->m_type == m_leaf) {
						leafArray[leafNodesCount] = node->m_right;
						leafNodesCount ++;
					}
				}

				dgList<dgNodeBase*>::dgListNode* nodePtr = list.GetFirst();
				m_root = BuildTopDown (&leafArray[0], 0, leafNodesCount - 1, &nodePtr);
				m_treeEntropy = CalculateEntropy (list);
			}
			while (m_root->m_parent) {
				m_root = m_root->m_parent;
			}
		} else {
			m_treeEntropy = dgFloat32 (2.0f);
		}

		m_boxMinRadius = dgMin(m_root->m_size.m_x, m_root->m_size.m_y, m_root->m_size.m_z);
		m_boxMaxRadius = dgSqrt (m_root->m_size % m_root->m_size);

		m_boxSize = m_root->m_size;
		m_boxOrigin = m_root->m_origin;
		MassProperties ();

		if (flushCache) {
			m_world->FlushCache ();
		}
	}
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::AddCollision (dgCollisionInstance* const shape)
{
	dgNodeBase* const newNode = new (m_allocator) dgNodeBase (shape);
	m_array.AddNode(newNode, m_idIndex, m_myInstance);

	m_idIndex ++;

	if (!m_root) {
		m_root = newNode;
	} else {
		dgVector p0;
		dgVector p1;		
		dgNodeBase* sibling = m_root;
		dgFloat32 surfaceArea = CalculateSurfaceArea (newNode, sibling, p0, p1);
		while(sibling->m_left && sibling->m_right) {

			if (surfaceArea > sibling->m_area) {
				break;
			} 

			sibling->SetBox (p0, p1);

			dgVector leftP0;
			dgVector leftP1;		
			dgFloat32 leftSurfaceArea = CalculateSurfaceArea (newNode, sibling->m_left, leftP0, leftP1);

			dgVector rightP0;
			dgVector rightP1;		
			dgFloat32 rightSurfaceArea = CalculateSurfaceArea (newNode, sibling->m_right, rightP0, rightP1);

			if (leftSurfaceArea < rightSurfaceArea) {
				sibling = sibling->m_left;
				p0 = leftP0;
				p1 = leftP1;
				surfaceArea = leftSurfaceArea;
			} else {
				sibling = sibling->m_right;
				p0 = rightP0;
				p1 = rightP1;
				surfaceArea = rightSurfaceArea;
			}
		} 

		if (!sibling->m_parent) {
			m_root = new (m_world->GetAllocator()) dgNodeBase (sibling, newNode);
		} else {
			dgNodeBase* const parent = sibling->m_parent;
			if (parent->m_left == sibling) {
				dgNodeBase* const node = new (m_world->GetAllocator()) dgNodeBase (sibling, newNode);
				parent->m_left = node;
				node->m_parent = parent;
			} else {
				dgAssert (parent->m_right == sibling); 
				dgNodeBase* const node = new (m_world->GetAllocator()) dgNodeBase (sibling, newNode);
				parent->m_right = node;
				node->m_parent = parent;
			}
		}
	}

	return newNode->m_myNode;
}


void dgCollisionCompound::RemoveCollision (dgTreeArray::dgTreeNode* const node)
{
	if (node) {
		dgCollisionInstance* const instance = node->GetInfo()->GetShape();
		instance->AddRef();
		RemoveCollision (node->GetInfo());
		instance->Release();
		m_array.Remove(node);
	}
}

void dgCollisionCompound::SetCollisionMatrix (dgTreeArray::dgTreeNode* const node, const dgMatrix& matrix)
{
	if (node) {
		dgWorld* const world = m_world;
		dgNodeBase* const baseNode = node->GetInfo();
		dgCollisionInstance* const instance = baseNode->GetShape();
		instance->SetLocalMatrix(matrix);

		dgVector p0;
		dgVector p1;
		instance->CalcAABB(instance->GetLocalMatrix (), p0, p1);
		{
			dgThreadHiveScopeLock lock (world, &m_criticalSectionLock);
			baseNode->SetBox (p0, p1);
		}

		for (dgNodeBase* parent = baseNode->m_parent; parent; parent = parent->m_parent) {
			dgVector minBox;
			dgVector maxBox;
			CalculateSurfaceArea (parent->m_left, parent->m_right, minBox, maxBox);
			if (dgBoxInclusionTest (minBox, maxBox, parent->m_p0, parent->m_p1)) {
				break;
			}
			
			dgThreadHiveScopeLock lock (world, &m_criticalSectionLock);
			parent->SetBox (minBox, maxBox);
		}
	}
}


void dgCollisionCompound::RemoveCollision (dgNodeBase* const treeNode)
{
	if (!treeNode->m_parent) {
		delete (m_root);
		m_root = NULL;
	} else if (!treeNode->m_parent->m_parent) {
		dgNodeBase* const root = m_root;
		if (treeNode->m_parent->m_left == treeNode) {
			m_root = treeNode->m_parent->m_right;
			treeNode->m_parent->m_right = NULL;
		} else {
			dgAssert (treeNode->m_parent->m_right == treeNode);
			m_root = treeNode->m_parent->m_left;
			treeNode->m_parent->m_left= NULL;
		}
		m_root->m_parent = NULL;
		delete (root);

	} else {
		dgNodeBase* const root = treeNode->m_parent->m_parent;
		if (treeNode->m_parent == root->m_left) {
			if (treeNode->m_parent->m_right == treeNode) {
				root->m_left = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = NULL;
			} else {
				dgAssert (treeNode->m_parent->m_left == treeNode);
				root->m_left = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = NULL;
			}
			root->m_left->m_parent = root;
		} else {
			if (treeNode->m_parent->m_right == treeNode) {
				root->m_right = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = NULL;
			} else {
				dgAssert (treeNode->m_parent->m_left == treeNode);
				root->m_right = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = NULL;
			}
			root->m_right->m_parent = root;
		}
		delete (treeNode->m_parent);
	}
}

dgInt32 dgCollisionCompound::GetNodeIndex(dgTreeArray::dgTreeNode* const node) const
{
	return node->GetKey();
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::FindNodeByIndex (dgInt32 index) const
{
	return m_array.Find (index);
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::GetFirstNode () const
{
	dgTreeArray::Iterator iter (m_array);
	iter.Begin();

	dgTreeArray::dgTreeNode* node = NULL;
	if (iter) {
		node = iter.GetNode();
	}
	return node; 
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::GetNextNode (dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* const node) const
{
	dgTreeArray::Iterator iter (m_array);
	iter.Set (node);
	iter ++;
	dgTreeArray::dgTreeNode* nextNode = NULL;
	if (iter) {
		nextNode = iter.GetNode();
	}
	return nextNode;
}

dgCollisionInstance* dgCollisionCompound::GetCollisionFromNode (dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* const node) const
{
	dgAssert (node->GetInfo());
	dgAssert (node->GetInfo()->GetShape());
	return node->GetInfo()->GetShape();
}


dgVector dgCollisionCompound::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgFloat32 aabbProjection[DG_COMPOUND_STACK_DEPTH];
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	aabbProjection[0] = dgFloat32 (1.0e10f);

	dgFloat32 maxProj = dgFloat32 (-1.0e20f); 
	//dgVector searchDir (m_offset.UnrotateVector(dir));

	dgInt32 ix = (dir[0] > dgFloat32 (0.0f)) ? 1 : 0;
	dgInt32 iy = (dir[1] > dgFloat32 (0.0f)) ? 1 : 0;
	dgInt32 iz = (dir[2] > dgFloat32 (0.0f)) ? 1 : 0;
	dgVector supportVertex (dgFloat32 (0.0f), dgFloat32 (0.0f),  dgFloat32 (0.0f),  dgFloat32 (0.0f));   

	while (stack) {

		stack--;
		dgFloat32 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) {
			const dgNodeBase* const me = stackPool[stack];

			if (me->m_type == m_leaf) {
				dgInt32 index; 
				dgCollisionInstance* const subShape = me->GetShape();
				const dgMatrix& matrix = subShape->GetLocalMatrix(); 
				dgVector newDir (matrix.UnrotateVector(dir)); 
				dgVector vertex (matrix.TransformVector (subShape->SupportVertex(newDir, &index)));		
				dgFloat32 dist = dir % vertex;
				if (dist > maxProj) {
					maxProj = dist;
					supportVertex = vertex;		
				}

			} else {
				const dgNodeBase* const left = me->m_left;
				const dgNodeBase* const right = me->m_right;

				const dgVector* const box0 = &left->m_p0;
				dgVector p0 (box0[ix].m_x, box0[iy].m_y, box0[iz].m_z, dgFloat32 (0.0f));

				const dgVector* const box1 = &right->m_p0;
				dgVector p1 (box1[ix].m_x, box1[iy].m_y, box1[iz].m_z, dgFloat32 (0.0f));

				dgFloat32 dist0 = p0 % dir;
				dgFloat32 dist1 = p1 % dir;
				if (dist0 > dist1) {
					stackPool[stack] = right;
					aabbProjection[stack] = dist1;
					stack ++;

					stackPool[stack] = left;
					aabbProjection[stack] = dist0;
					stack ++;
				} else {
					stackPool[stack] = left;
					aabbProjection[stack] = dist0;
					stack ++;

					stackPool[stack] = right;
					aabbProjection[stack] = dist1;
					stack ++;
				} 
			}
		}
	}

//	return m_offset.TransformVector (supportVertex);
	return supportVertex;
}


void dgCollisionCompound::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollision::GetCollisionInfo(info);

	info->m_compoundCollision.m_chidrenCount = m_array.GetCount();
	info->m_collisionType = m_compoundCollision;
}

void dgCollisionCompound::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	
	dgInt32 count = m_array.GetCount();
	callback (userData, &count, sizeof (count));
	dgTreeArray::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgCollisionInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		collision->Serialize(callback, userData);
	}
}



dgFloat32 dgCollisionCompound::CalculateSurfaceArea (dgNodeBase* const node0, dgNodeBase* const node1, dgVector& minBox, dgVector& maxBox) const
{
	minBox = node0->m_p0.GetMin(node1->m_p0);
	maxBox = node0->m_p1.GetMax(node1->m_p1);
	dgVector side0 ((maxBox - minBox).CompProduct4 (dgVector::m_half));
	return side0.DotProduct4(side0.ShiftTripleRight()).m_x;
}


void dgCollisionCompound::ImproveNodeFitness (dgNodeBase* const node) const
{
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (node->m_parent)	{
		if (node->m_parent->m_left == node) {
			dgFloat32 cost0 = node->m_area;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));
			}
		} else {
			dgFloat32 cost0 = node->m_area;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {

				dgNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));
			}
		}
	} else {
		// in the future I can handle this but it is too much work for little payoff
	}
}



dgInt32 dgCollisionCompound::CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgInt32 contactCount = 0;
	if (m_root) {
		dgAssert (IsType (dgCollision::dgCollisionCompound_RTTI));
		dgContact* const constraint = pair->m_contact;
		dgBody* const body1 = constraint->GetBody1();

		if (proxy.m_continueCollision) {
			if (body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
				contactCount = CalculateContactsToSingleContinue (pair, proxy);
			} else if (body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
				contactCount = CalculateContactsToCompoundContinue (pair, proxy);
			} else if (body1->m_collision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
				contactCount = CalculateContactsToCollisionTreeContinue (pair, proxy);
			} else if (body1->m_collision->IsType (dgCollision::dgCollisionHeightField_RTTI)) {
				dgAssert (0);
//				contactCount = CalculateContactsToHeightField (pair, proxy);
			} else {
				dgAssert (0);
				dgAssert (body1->m_collision->IsType (dgCollision::dgCollisionUserMesh_RTTI));
//				contactCount = CalculateContactsUserDefinedCollision (pair, proxy);
			}

		} else {
			if (body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
				contactCount = CalculateContactsToSingle (pair, proxy);
			} else if (body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
				contactCount = CalculateContactsToCompound (pair, proxy);
			} else if (body1->m_collision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
				contactCount = CalculateContactsToCollisionTree (pair, proxy);
			} else if (body1->m_collision->IsType (dgCollision::dgCollisionHeightField_RTTI)) {
				contactCount = CalculateContactsToHeightField (pair, proxy);
			} else {
				dgAssert (body1->m_collision->IsType (dgCollision::dgCollisionUserMesh_RTTI));
				contactCount = CalculateContactsUserDefinedCollision (pair, proxy);
			}
		}
	}
	pair->m_contactCount = contactCount;
	return contactCount;
}


//dgInt32 dgCollisionCompound::ClosestDistance (dgBody* const compoundBody, dgTriplex& contactA, dgBody* const bodyB, dgTriplex& contactB, dgTriplex& normalAB) const
dgInt32 dgCollisionCompound::ClosestDistance (dgCollisionParamProxy& proxy) const
{
	int count = 0;
	if (m_root) {
		if (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
			count = ClosestDistanceToConvex (proxy);
		} else if (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
			count = ClosestDistanceToCompound (proxy);
		} else if (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
			dgAssert(0);
		} else {
			dgAssert(0);
		}
	}

	return count;
}


dgInt32 dgCollisionCompound::ClosestDistanceToConvex (dgCollisionParamProxy& proxy) const
{
	dgInt32 retFlag = 0;

	dgCollisionInstance* const compoundInstance = proxy.m_referenceCollision;
	dgCollisionInstance* const otherInstance = proxy.m_floatingCollision;

	const dgMatrix myMatrix = compoundInstance->GetGlobalMatrix();
	dgMatrix matrix (otherInstance->GetGlobalMatrix() * myMatrix.Inverse());

	dgVector p0;
	dgVector p1;
	otherInstance->CalcAABB(matrix, p0, p1);

	dgUnsigned8 pool[64 * (sizeof (dgNodeBase*) + sizeof (dgFloat32))];
	dgUpHeap<dgNodeBase*, dgFloat32> heap (pool, sizeof (pool));
	
	dgNodeBase* node = m_root;
	dgVector boxP0 (p0 - m_root->m_p1);
	dgVector boxP1 (p1 - m_root->m_p0);
	heap.Push(node, dgBoxDistanceToOrigin2 (boxP0, boxP1));

	dgContactPoint contact0;
	dgContactPoint contact1;
	dgFloat32 minDist2 = dgFloat32 (1.0e10f);
	while (heap.GetCount() && (heap.Value() <= minDist2)) {
		const dgNodeBase* const node = heap[0];
		heap.Pop();
		if (node->m_type == m_leaf) {
			dgCollisionInstance* const subShape = node->GetShape();
			dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());

			childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
			proxy.m_referenceCollision = &childInstance; 
			dgInt32 flag = m_world->ClosestPoint (proxy);
			if (flag) {
				retFlag = 1;
				dgFloat32 dist2 = proxy.m_contactJoint->m_closestDistance * proxy.m_contactJoint->m_closestDistance;
				if (dist2 < minDist2) {
					minDist2 = dist2;
					contact0 = proxy.m_contacts[0];
					contact1 = proxy.m_contacts[1];
				}
			} else {
				dgAssert (0);
				break;
			}

		} else {
			dgNodeBase* left = node->m_left;
			dgNodeBase* right = node->m_right;

			dgVector leftBoxP0 (p0 - left->m_p1);
			dgVector leftBoxP1 (p1 - left->m_p0);
			heap.Push(left, dgBoxDistanceToOrigin2 (leftBoxP0, leftBoxP1));

			dgVector rightBoxP0 (p0 - right->m_p1);
			dgVector rightBoxP1 (p1 - right->m_p0);
			heap.Push(right, dgBoxDistanceToOrigin2 (rightBoxP0, rightBoxP1));
		}
	}

	if (retFlag) {
		proxy.m_contacts[0] = contact0;
		proxy.m_contacts[1] = contact1;
		proxy.m_contactJoint->m_closestDistance = dgSqrt (minDist2);
	}
	return retFlag;
}

DG_INLINE void dgCollisionCompound::PushNode (const dgMatrix& matrix, dgUpHeap<dgHeapNodePair, dgFloat32>& heap, dgNodeBase* const myNode, dgNodeBase* const otherNode) const
{
	dgVector p0;
	dgVector p1;
	dgHeapNodePair pair; 
	pair.m_nodeA = myNode;
	pair.m_nodeB = otherNode;
	matrix.TransformBBox (otherNode->m_p0, otherNode->m_p1, p0, p1);			
	dgVector boxP0 (p0 - myNode->m_p1);
	dgVector boxP1 (p1 - myNode->m_p0);	
	heap.Push(pair, dgBoxDistanceToOrigin2(boxP0, boxP1));
}

dgInt32 dgCollisionCompound::ClosestDistanceToCompound (dgCollisionParamProxy& proxy) const
{
	dgInt32 retFlag = 0;

	dgUnsigned8 pool[128 * (sizeof (dgHeapNodePair) + sizeof (dgFloat32))];
	dgUpHeap<dgHeapNodePair, dgFloat32> heap (pool, sizeof (pool));

	dgCollisionInstance* const compoundInstance = proxy.m_referenceCollision;
	dgCollisionInstance* const otherCompoundInstance = proxy.m_floatingCollision;

	const dgMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	const dgMatrix& otherMatrix = otherCompoundInstance->GetGlobalMatrix();
	dgMatrix matrix (otherMatrix * myMatrix.Inverse());

	const dgCollisionCompound* const otherShape = (dgCollisionCompound*)otherCompoundInstance->GetChildShape();
	PushNode (matrix, heap, m_root, otherShape->m_root);

	dgContactPoint contact0;
	dgContactPoint contact1;
	dgFloat32 minDist2 = dgFloat32 (1.0e10f);
	while (heap.GetCount() && (heap.Value() <= minDist2)) {

		dgHeapNodePair pair = heap[0];
		heap.Pop();

		if ((pair.m_nodeA->m_type == m_leaf) && (pair.m_nodeB->m_type == m_leaf)) {
			dgCollisionInstance* const mySubShape = pair.m_nodeA->GetShape();
			dgCollisionInstance myChildInstance (*mySubShape, mySubShape->GetChildShape());
			myChildInstance.m_globalMatrix = myChildInstance.GetLocalMatrix() * myMatrix;
			proxy.m_referenceCollision = &myChildInstance; 

			dgCollisionInstance* const otherSubShape = pair.m_nodeB->GetShape();
			dgCollisionInstance otherChildInstance (*otherSubShape, otherSubShape->GetChildShape());
			otherChildInstance.m_globalMatrix = otherChildInstance.GetLocalMatrix() * otherMatrix;
			proxy.m_floatingCollision = &otherChildInstance; 

			dgInt32 flag = m_world->ClosestPoint (proxy);
			if (flag) {
				retFlag = 1;
				dgFloat32 dist2 = proxy.m_contactJoint->m_closestDistance * proxy.m_contactJoint->m_closestDistance;
				if (dist2 < minDist2) {
					minDist2 = dist2;
					contact0 = proxy.m_contacts[0];
					contact1 = proxy.m_contacts[1];
				}
			} else {
				dgAssert (0);
				break;
			}
		} else if (pair.m_nodeA->m_type == m_leaf) {
			PushNode (matrix, heap, pair.m_nodeA, pair.m_nodeB->m_left);
			PushNode (matrix, heap, pair.m_nodeA, pair.m_nodeB->m_right);

		} else if (pair.m_nodeB->m_type == m_leaf) {
			PushNode (matrix, heap, pair.m_nodeA->m_left, pair.m_nodeB);
			PushNode (matrix, heap, pair.m_nodeA->m_right, pair.m_nodeB);

		} else {
			PushNode (matrix, heap, pair.m_nodeA->m_left, pair.m_nodeB->m_left);
			PushNode (matrix, heap, pair.m_nodeA->m_left, pair.m_nodeB->m_right);
			PushNode (matrix, heap, pair.m_nodeA->m_right, pair.m_nodeB->m_left);
			PushNode (matrix, heap, pair.m_nodeA->m_right, pair.m_nodeB->m_right);
		}
	}

	if (retFlag) {
		proxy.m_contacts[0] = contact0;
		proxy.m_contacts[1] = contact1;
		proxy.m_contactJoint->m_closestDistance = dgSqrt (minDist2);
	}
	return retFlag;
}



dgInt32 dgCollisionCompound::CalculateContactsToCompound (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContactPoint* const contacts = proxy.m_contacts;
	const dgNodeBase* stackPool[4 * DG_COMPOUND_STACK_DEPTH][2];

	dgInt32 contactCount = 0;
	dgContact* const constraint = pair->m_contact;
	dgBody* const myBody = constraint->GetBody0();
	dgBody* const otherBody = constraint->GetBody1();

	dgCollisionInstance* const myCompoundInstance = myBody->m_collision;
	dgCollisionInstance* const otherCompoundInstance = otherBody->m_collision;

	dgAssert (myCompoundInstance->GetChildShape() == this);
	dgAssert (otherCompoundInstance->IsType (dgCollision::dgCollisionCompound_RTTI));
	dgCollisionCompound* const otherCompound = (dgCollisionCompound*)otherCompoundInstance->GetChildShape();

	proxy.m_referenceBody = myBody;
	proxy.m_floatingBody = otherBody;

	const dgMatrix& myMatrix = myCompoundInstance->GetGlobalMatrix();
	const dgMatrix& otherMatrix = otherCompoundInstance->GetGlobalMatrix();
	dgOOBBTestData data (otherMatrix * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0][0] = m_root;
	stackPool[0][1] = otherCompound->m_root;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgAssert (contacts);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack][0];
		const dgNodeBase* const other = stackPool[stack][1];

		dgAssert (me && other);

		if (me->BoxTest (data, other)) {

			if ((me->m_type == m_leaf) && (other->m_type == m_leaf)) {
				bool processContacts = true;
				if (material->m_compoundAABBOverlap) {
					processContacts = material->m_compoundAABBOverlap (*material, myBody, me->m_myNode, otherBody, other->m_myNode, proxy.m_threadIndex);
				}
				if (processContacts) {
					if (me->GetShape()->GetCollisionMode() & other->GetShape()->GetCollisionMode()) {
						const dgCollisionInstance* const subShape = me->GetShape();
						const dgCollisionInstance* const otherSubShape = other->GetShape();

						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						dgCollisionInstance otherChildInstance (*otherSubShape, otherSubShape->GetChildShape());
						otherChildInstance.m_globalMatrix = otherChildInstance.GetLocalMatrix() * otherMatrix;
						proxy.m_floatingCollision = &otherChildInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						if (!proxy.m_intersectionTestOnly) {
							for (dgInt32 i = 0; i < count; i ++) {
								dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
								dgAssert (contacts[contactCount + i].m_collision1 == &otherChildInstance);
								contacts[contactCount + i].m_collision0 = subShape;
								contacts[contactCount + i].m_collision1 = otherSubShape;
							}
							contactCount += count;
							if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
								contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
							}
						} else if (count == -1) {
							contactCount = -1;
							break;
						}
						//childInstance.SetUserData(NULL);
						//otherChildInstance.SetUserData(NULL);
						proxy.m_referenceCollision = NULL;
						proxy.m_floatingCollision = NULL; 
					}
				}

			} else if (me->m_type == m_leaf) {
				dgAssert (other->m_type == m_node);

				stackPool[stack][0] = me;
				stackPool[stack][1] = other->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me;
				stackPool[stack][1] = other->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));


			} else if (other->m_type == m_leaf) {
				dgAssert (me->m_type == m_node);

				stackPool[stack][0] = me->m_left;
				stackPool[stack][1] = other;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_right;
				stackPool[stack][1] = other;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));
			} else {
				dgAssert (me->m_type == m_node);
				dgAssert (other->m_type == m_node);

				stackPool[stack][0] = me->m_left;
				stackPool[stack][1] = other->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_left;
				stackPool[stack][1] = other->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_right;
				stackPool[stack][1] = other->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_right;
				stackPool[stack][1] = other->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;
	return contactCount;
}




dgInt32 dgCollisionCompound::CalculateContactsToHeightField (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContactPoint* const contacts = proxy.m_contacts;

	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgInt32 contactCount = 0;
	dgContact* const constraint = pair->m_contact;
	dgBody* const myBody = constraint->GetBody0();
	dgBody* const terrainBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = myBody->m_collision;
	dgCollisionInstance* const terrainInstance = terrainBody->m_collision;

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (terrainInstance->IsType (dgCollision::dgCollisionHeightField_RTTI));
	dgCollisionHeightField* const terrainCollision = (dgCollisionHeightField*)terrainInstance->GetChildShape();

	proxy.m_referenceBody = myBody;
	proxy.m_floatingBody = terrainBody;

	proxy.m_floatingCollision = terrainInstance;
//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);
	const dgMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	dgOOBBTestData data (terrainInstance->GetGlobalMatrix() * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0] = m_root;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgAssert (contacts);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack];

		dgVector origin (data.m_matrix.UntransformVector(me->m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(me->m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);
		terrainCollision->GetLocalAABB (p0, p1, nodeProxi.m_p0, nodeProxi.m_p1);
		//nodeProxi.m_size = (nodeProxi.m_p1 - nodeProxi.m_p0).Scale3 (dgFloat32 (0.5f));
		//nodeProxi.m_origin = (nodeProxi.m_p1 + nodeProxi.m_p0).Scale3 (dgFloat32 (0.5f));
		nodeProxi.m_size = (nodeProxi.m_p1 - nodeProxi.m_p0).CompProduct4(dgVector::m_half);
		nodeProxi.m_origin = (nodeProxi.m_p1 + nodeProxi.m_p0).CompProduct4(dgVector::m_half);

		if (me->BoxTest (data, &nodeProxi)) {
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me->m_myNode, terrainBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = 0;
						count += m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						if (!proxy.m_intersectionTestOnly) {
							for (dgInt32 i = 0; i < count; i ++) {
								dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
								contacts[contactCount + i].m_collision0 = subShape;
							}
							contactCount += count;

							if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
								contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
							}
						} else if (count == -1) {
							contactCount = -1;
							break;
						}
						//childInstance.SetUserData(NULL);
						proxy.m_referenceCollision = NULL;
					}
				}

			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack] = me->m_left;
				stack++;

				stackPool[stack] = me->m_right;
				stack++;
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;	
	return contactCount;
}


dgInt32 dgCollisionCompound::CalculateContactsUserDefinedCollision (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContactPoint* const contacts = proxy.m_contacts;

	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgInt32 contactCount = 0;
	dgContact* const constraint = pair->m_contact;
	dgBody* const myBody = constraint->GetBody0();
	dgBody* const userBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = myBody->m_collision;
	dgCollisionInstance* const userMeshInstance = userBody->m_collision;

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (userMeshInstance->IsType (dgCollision::dgCollisionUserMesh_RTTI));
	dgCollisionUserMesh* const userMeshCollision = (dgCollisionUserMesh*)userMeshInstance->GetChildShape();

	proxy.m_referenceBody = myBody;
	proxy.m_floatingBody = userBody;

	proxy.m_floatingCollision = userMeshInstance;
//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);
	const dgMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	dgOOBBTestData data (userMeshInstance->GetGlobalMatrix() * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0] = m_root;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgAssert (contacts);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack];

		dgVector origin (data.m_matrix.UntransformVector(me->m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(me->m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);

		if (userMeshCollision->AABBOvelapTest (p0, p1)) {
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me->m_myNode, userBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = 0;
						count += m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						if (!proxy.m_intersectionTestOnly) {
							for (dgInt32 i = 0; i < count; i ++) {
								dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
								contacts[contactCount + i].m_collision0 = subShape;
							}
							contactCount += count;

							if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
								contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
							}
						} else if (count == -1) {
							contactCount = -1;
							break;
						}
						//childInstance.SetUserData(NULL);
						proxy.m_referenceCollision = NULL;
					}
				}

			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack] = me->m_left;
				stack++;

				stackPool[stack] = me->m_right;
				stack++;
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;	
	return contactCount;
}


dgInt32 dgCollisionCompound::CalculateContactsToSingle (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContactPoint* const contacts = proxy.m_contacts;
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgContact* const constraint = pair->m_contact;

	dgBody* const compoundBody = constraint->GetBody0();
	dgBody* const otherBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = compoundBody->m_collision;
	dgCollisionInstance* const otherInstance = otherBody->m_collision;

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (otherInstance->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	proxy.m_referenceBody = compoundBody;

	proxy.m_floatingBody = otherBody;
	proxy.m_floatingCollision = otherBody->m_collision;

	dgInt32 contactCount = 0;
//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * compoundBody->m_matrix);
	const dgMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	dgMatrix matrix (otherBody->m_collision->GetGlobalMatrix() * myMatrix.Inverse());

	dgVector size;
	dgVector origin;
	otherInstance->CalcObb (origin, size);
	dgOOBBTestData data (matrix, origin, size);

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgAssert (contacts);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);

	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack];
		dgAssert (me);

		if (me->BoxTest (data)) {
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, compoundBody, me->m_myNode, otherBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);
						if (!proxy.m_intersectionTestOnly) {
							for (dgInt32 i = 0; i < count; i ++) {
								dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
								contacts[contactCount + i].m_collision0 = subShape;
							}
							contactCount += count;
							if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
								contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
							}
						} else if (count == -1) {
							contactCount = -1;
							break;
						}
						//childInstance.SetUserData(NULL);
						proxy.m_referenceCollision = NULL;
					}
				}

			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack] = me->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack] = me->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;
	return contactCount;
}



dgInt32 dgCollisionCompound::CalculateContactsToCollisionTree (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContactPoint* const contacts = proxy.m_contacts;

	dgNodePairs stackPool[4 * DG_COMPOUND_STACK_DEPTH];

	dgInt32 contactCount = 0;

	dgContact* const constraint = pair->m_contact;
	dgBody* const myBody = constraint->GetBody0();
	dgBody* const treeBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = myBody->m_collision;
	dgCollisionInstance* const treeCollisionInstance = treeBody->m_collision;

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (treeCollisionInstance->IsType (dgCollision::dgCollisionBVH_RTTI));
	dgCollisionBVH* const treeCollision = (dgCollisionBVH*)treeCollisionInstance->GetChildShape();

	proxy.m_referenceBody = myBody;
	proxy.m_floatingBody = treeBody;
	proxy.m_floatingCollision = treeCollisionInstance;

//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);
	const dgMatrix& myMatrix (compoundInstance->GetGlobalMatrix());
	dgOOBBTestData data (treeCollisionInstance->GetGlobalMatrix() * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0].m_myNode = m_root;
	stackPool[0].m_treeNode = treeCollision->GetRootNode();
	stackPool[0].m_treeNodeIsLeaf = 0;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;

	const dgContactMaterial* const material = constraint->GetMaterial();
	const dgVector& treeScale = treeCollisionInstance->GetScale();
	
	dgAssert (contacts);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {

		stack --;
		const dgNodePairs* const stackEntry = &stackPool[stack];

		dgNodeBase* const me = stackEntry->m_myNode;
		const void* const other = stackEntry->m_treeNode;
		dgInt32 treeNodeIsLeaf = stackEntry->m_treeNodeIsLeaf;

		dgAssert (me && other);
		dgVector p0;
		dgVector p1;

		treeCollision->GetNodeAABB(other, p0, p1);
		nodeProxi.m_p0 = p0.CompProduct4(treeScale);
		nodeProxi.m_p1 = p1.CompProduct4(treeScale);

		p0 = nodeProxi.m_p0.CompProduct4(dgVector::m_half);
		p1 = nodeProxi.m_p1.CompProduct4(dgVector::m_half);
		nodeProxi.m_size = p1 - p0;
		nodeProxi.m_origin = p1 + p0;
		nodeProxi.m_area = nodeProxi.m_size.ShiftTripleRight().DotProduct4(nodeProxi.m_size).GetScalar();

		if (me->BoxTest (data, &nodeProxi)) {
			if ((me->m_type == m_leaf) && treeNodeIsLeaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me->m_myNode, treeBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						if (!proxy.m_intersectionTestOnly) {
							for (dgInt32 i = 0; i < count; i ++) {
								dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
								contacts[contactCount + i].m_collision0 = subShape;
							}
							contactCount += count;
							if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
								contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
							}
						} else if (count == -1) {
							contactCount = -1;
							break;
						}
						//childInstance.SetUserData(NULL);
						proxy.m_referenceCollision = NULL; 
					}
				}

			} else if (me->m_type == m_leaf) {
				void* const frontNode = treeCollision->GetFrontNode(other);
				void* const backNode = treeCollision->GetBackNode(other);
				if (backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

				} else if (backNode && !frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;

					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

				} else if (!backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

				} else {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				}

			} else if (treeNodeIsLeaf) {
				stackPool[stack].m_myNode = me->m_left;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack].m_myNode = me->m_right;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

			} else if (nodeProxi.m_area > me->m_area) {
				dgAssert (me->m_type == m_node);
				void* const frontNode = treeCollision->GetFrontNode(other);
				void* const backNode = treeCollision->GetBackNode(other);
				if (backNode && frontNode) {
					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
				} else if (backNode && !frontNode) {
					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me->m_left;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

					stackPool[stack].m_myNode = me->m_right;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

				} else if (!backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me->m_left;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

					stackPool[stack].m_myNode = me->m_right;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

				} else {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				}

			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack].m_myNode = me->m_left;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = treeNodeIsLeaf;
				stack++;

				stackPool[stack].m_myNode = me->m_right;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = treeNodeIsLeaf;
				stack++;
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;	
	return contactCount;
}



dgInt32 dgCollisionCompound::CalculateContactsToSingleContinue(dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	if (proxy.m_timestep < dgFloat32 (1.0e-4f)) {
		return 0;
	}
	dgContactPoint* const contacts = proxy.m_contacts;
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];
	dgContact* const constraint = pair->m_contact;

	dgBody* const compoundBody = constraint->GetBody0();
	dgBody* const otherBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = compoundBody->m_collision;
	dgCollisionInstance* const otherInstance = otherBody->m_collision;

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (otherInstance->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	proxy.m_referenceBody = compoundBody;

	proxy.m_floatingBody = otherBody;
	proxy.m_floatingCollision = otherBody->m_collision;

	dgInt32 contactCount = 0;

//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * compoundBody->m_matrix);
	const dgMatrix myMatrix = compoundInstance->GetGlobalMatrix();
	dgMatrix matrix (otherBody->m_collision->GetGlobalMatrix() * myMatrix.Inverse());

	dgVector boxP0;
	dgVector boxP1;
	otherInstance->CalcAABB(matrix, boxP0, boxP1);
	dgVector relVeloc (myMatrix.UnrotateVector (otherBody->GetVelocity() - compoundBody->GetVelocity()));
	dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), relVeloc);

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgFloat32 maxParam = proxy.m_timestep;
	dgFloat32 invMaxParam = dgFloat32 (1.0f) / maxParam; 

	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack];
		dgAssert (me);

		dgVector minBox (me->m_p0 - boxP1);
		dgVector maxBox (me->m_p1 - boxP0);
		if (ray.BoxTest (minBox, maxBox)) {
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();

				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, compoundBody, me->m_myNode, otherBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);

						closestDist = dgMin(closestDist, constraint->m_closestDistance);
						if (count) {
							dgFloat32 param = proxy.m_timestep;
							dgAssert (param >= dgFloat32 (0.0f));
							if (param < maxParam) {
								if (proxy.m_intersectionTestOnly) {
									maxParam = param;
									if (count == -1) {
										contactCount = -1;
										break;
									}

								} else {
									if (contactCount && ((param - maxParam) * invMaxParam) < dgFloat32(-1.0e-3f)) {
										for (dgInt32 i = 0; i < count; i ++) {
											contacts[i] = contacts[contactCount + i];
										}
										contactCount = 0;
									}
									maxParam = param;

									for (dgInt32 i = 0; i < count; i ++) {
										dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
										contacts[contactCount + i].m_collision0 = subShape;
									}
									contactCount += count;

									if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
										contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
									}

									if (maxParam == dgFloat32 (0.0f)) {
										break;
									}
								}
							}
						}
						proxy.m_referenceCollision = NULL; 
					}
				}
			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack] = me->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack] = me->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;
	return contactCount;
}



dgInt32 dgCollisionCompound::CalculateContactsToCompoundContinue(dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	if (proxy.m_timestep < dgFloat32 (1.0e-4f)) {
		return 0;
	}

	dgInt32 contactCount = 0;
	dgContactPoint* const contacts = proxy.m_contacts;
	const dgNodeBase* stackPool[4 * DG_COMPOUND_STACK_DEPTH][2];
	dgContact* const constraint = pair->m_contact;

	dgBody* const compoundBody = constraint->GetBody0();
	dgBody* const otherCompoundBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = compoundBody->m_collision;
	dgCollisionInstance* const otherCompoundInstance = otherCompoundBody->m_collision;

	dgCollisionCompound* const otherCompound = (dgCollisionCompound*)otherCompoundInstance->GetChildShape();

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (otherCompoundInstance->IsType (dgCollision::dgCollisionCompound_RTTI));

	proxy.m_referenceBody = compoundBody;
	proxy.m_floatingBody = otherCompoundBody;

//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * compoundBody->m_matrix);
//	dgMatrix otherMatrix (otherCompoundInstance->GetLocalMatrix() * otherCompoundBody->m_matrix);
//	dgMatrix matrix (otherCompoundInstance->GetGlobalMatrix() * myMatrix.Inverse());
	const dgMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	const dgMatrix& otherMatrix = otherCompoundInstance->GetGlobalMatrix();
	dgOOBBTestData data (otherMatrix * myMatrix.Inverse());

//	dgVector boxP0;
//	dgVector boxP1;
//	otherCompoundInstance->CalcAABB (data.m_matrix, boxP0, boxP1);
	dgVector relVeloc (myMatrix.UnrotateVector (otherCompoundBody->GetVelocity() - compoundBody->GetVelocity()));
	dgFastRayTest myCompoundRay (dgVector (dgFloat32 (0.0f)), relVeloc);
	dgFastRayTest otherCompoundRay (dgVector (dgFloat32 (0.0f)), data.m_matrix.UnrotateVector(relVeloc));

	dgInt32 stack = 1;
	stackPool[0][0] = m_root;
	stackPool[0][1] = otherCompound->m_root;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgFloat32 maxParam = proxy.m_timestep;
	dgFloat32 invMaxParam = dgFloat32 (1.0f) / maxParam; 

	dgFloat32 upperBound = dgFloat32 (1.0f);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);

	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack][0];
		const dgNodeBase* const other = stackPool[stack][1];
		dgAssert (me && other);

		dgFloat32 dist = me->RayBoxDistance (data, myCompoundRay, otherCompoundRay, other);
		if (dist <= upperBound) {
			if ((me->m_type == m_leaf) && (other->m_type == m_leaf)) {
				dgCollisionInstance* const subShape = me->GetShape();
				dgCollisionInstance* const otherSubShape = other->GetShape();

				if (subShape->GetCollisionMode() & otherSubShape->GetCollisionMode()) {

					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, compoundBody, me->m_myNode, otherCompoundBody, other->m_myNode, proxy.m_threadIndex);
					}

					if (processContacts) {

						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						dgCollisionInstance otherChildInstance (*otherSubShape, otherSubShape->GetChildShape());
						otherChildInstance.m_globalMatrix = otherChildInstance.GetLocalMatrix() * otherMatrix;
						proxy.m_floatingCollision = &otherChildInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);

						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						if (count) {
							dgFloat32 param = proxy.m_timestep;
							dgAssert (param >= dgFloat32 (0.0f));
							if (param < maxParam) {

								upperBound = param * invMaxParam;
								if (proxy.m_intersectionTestOnly) {
									maxParam = param;
									if (count == -1) {
										contactCount = -1;
										break;
									}

								} else {
									if (contactCount && ((param - maxParam) * invMaxParam) < dgFloat32(-1.0e-3f)) {
										for (dgInt32 i = 0; i < count; i ++) {
											contacts[i] = contacts[contactCount + i];
										}
										contactCount = 0;
									}
									maxParam = param;

									for (dgInt32 i = 0; i < count; i ++) {
										dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
										contacts[contactCount + i].m_collision0 = subShape;
									}
									contactCount += count;

									if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
										contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
									}

									if (maxParam == dgFloat32 (0.0f)) {
										break;
									}
								}
							}
						}
						proxy.m_referenceCollision = NULL; 
						proxy.m_floatingCollision = NULL; 
					}
				}
			} else if (me->m_type == m_leaf) {
				dgAssert (other->m_type == m_node);
				stackPool[stack][0] = me;
				stackPool[stack][1] = other->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me;
				stackPool[stack][1] = other->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

			} else if (other->m_type == m_leaf) {
				dgAssert (me->m_type == m_node);
				stackPool[stack][0] = me->m_left;
				stackPool[stack][1] = other;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_right;
				stackPool[stack][1] = other;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

			} else {
				dgAssert (me->m_type == m_node);
				dgAssert (other->m_type == m_node);

				stackPool[stack][0] = me->m_left;
				stackPool[stack][1] = other->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_left;
				stackPool[stack][1] = other->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_right;
				stackPool[stack][1] = other->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack][0] = me->m_right;
				stackPool[stack][1] = other->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;
	return contactCount;
}



dgInt32 dgCollisionCompound::CalculateContactsToCollisionTreeContinue (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	if (proxy.m_timestep < dgFloat32 (1.0e-4f)) {
		return 0;
	}

	dgContactPoint* const contacts = proxy.m_contacts;

	dgNodePairs stackPool[4 * DG_COMPOUND_STACK_DEPTH];

	dgInt32 contactCount = 0;

	dgContact* const constraint = pair->m_contact;
	dgBody* const myBody = constraint->GetBody0();
	dgBody* const treeBody = constraint->GetBody1();

	dgCollisionInstance* const compoundInstance = myBody->m_collision;
	dgCollisionInstance* const treeCollisionInstance = treeBody->m_collision;

	dgAssert (compoundInstance->GetChildShape() == this);
	dgAssert (treeCollisionInstance->IsType (dgCollision::dgCollisionBVH_RTTI));
	dgCollisionBVH* const treeCollision = (dgCollisionBVH*)treeCollisionInstance->GetChildShape();

	proxy.m_referenceBody = myBody;
	proxy.m_floatingBody = treeBody;
	proxy.m_floatingCollision = treeCollisionInstance;

//	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);
	const dgMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	const dgMatrix& otherMatrix = treeCollisionInstance->GetGlobalMatrix();
	dgOOBBTestData data (otherMatrix * myMatrix.Inverse());

	dgVector relVeloc (myMatrix.UnrotateVector (treeBody->GetVelocity() - myBody->GetVelocity()));
	dgFastRayTest myCompoundRay (dgVector (dgFloat32 (0.0f)), relVeloc);
	dgFastRayTest otherTreedRay (dgVector (dgFloat32 (0.0f)), data.m_matrix.UnrotateVector(relVeloc));

	dgInt32 stack = 1;
	stackPool[0].m_myNode = m_root;
	stackPool[0].m_treeNode = treeCollision->GetRootNode();
	stackPool[0].m_treeNodeIsLeaf = 0;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;

	const dgVector& treeScale = treeCollisionInstance->GetScale();
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgFloat32 maxParam = proxy.m_timestep;
	dgFloat32 invMaxParam = dgFloat32 (1.0f) / maxParam; 

	dgFloat32 upperBound = dgFloat32 (1.0f);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {

		stack --;
		const dgNodePairs* const stackEntry = &stackPool[stack];

		dgNodeBase* const me = stackEntry->m_myNode;
		const void* const other = stackEntry->m_treeNode;
		dgInt32 treeNodeIsLeaf = stackEntry->m_treeNodeIsLeaf;

		dgAssert (me && other);

		dgVector p0;
		dgVector p1;
/*
#ifdef _DEBUG
		treeCollision->GetNodeAABB(other, p0, p1);
		dgVector size = (p1 - p0).Scale3 (dgFloat32 (0.5f));
		dgVector origin = (p1 + p0).Scale3 (dgFloat32 (0.5f));
		dgVector size1 (size.m_y, size.m_z, size.m_x, dgFloat32 (0.0f));
		dgFloat32 area = size  % size1;
		dgAssert (dgAbsf(area - stackEntry->m_treeNodeArea) < dgFloat32 (1.0e-1f));
#endif

		nodeProxi.m_p0 = stackEntry->m_treeNodeP0;
		nodeProxi.m_p1 = stackEntry->m_treeNodeP1;
		nodeProxi.m_area = stackEntry->m_treeNodeArea;
		nodeProxi.m_size = stackEntry->m_treeNodeSize;
		nodeProxi.m_origin = stackEntry->m_treeNodeOrigin;
*/
		treeCollision->GetNodeAABB(other, p0, p1);
		nodeProxi.m_p0 = p0.CompProduct4(treeScale);
		nodeProxi.m_p1 = p1.CompProduct4(treeScale);

		p0 = nodeProxi.m_p0.CompProduct4(dgVector::m_half);
		p1 = nodeProxi.m_p1.CompProduct4(dgVector::m_half);
		nodeProxi.m_size = p1 - p0;
		nodeProxi.m_origin = p1 + p0;
		nodeProxi.m_area = nodeProxi.m_size.ShiftTripleRight().DotProduct4(nodeProxi.m_size).GetScalar();

		dgFloat32 dist = me->RayBoxDistance (data, myCompoundRay, otherTreedRay, &nodeProxi);
//		if (me->BoxTest (data, &nodeProxi)) {
		if (dist <= upperBound) {
			if ((me->m_type == m_leaf) && treeNodeIsLeaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me->m_myNode, treeBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {

						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						//dgCollisionInstance otherChildInstance (*otherSubShape, otherSubShape->GetChildShape());
						//otherChildInstance.m_globalMatrix = otherChildInstance.GetLocalMatrix() * otherMatrix;
						//proxy.m_floatingCollision = &otherChildInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = contacts ? &contacts[contactCount] : contacts;

						dgInt32 count = m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						//if (!proxy.m_intersectionTestOnly) {
						//	for (dgInt32 i = 0; i < count; i ++) {
						//		dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
						//		contacts[contactCount + i].m_collision0 = subShape;
						//	}
						//	contactCount += count;
						//	if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
						//		contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3);
						//	}
						//} else if (count == -1) {
						//	contactCount = -1;
						//	break;
						//}

						if (count) {
							dgFloat32 param = proxy.m_timestep;
							dgAssert (param >= dgFloat32 (0.0f));
							if (param < maxParam) {

								upperBound = param * invMaxParam;
								if (proxy.m_intersectionTestOnly) {
									maxParam = param;
									if (count == -1) {
										contactCount = -1;
										break;
									}

								} else {
									if (contactCount && ((param - maxParam) * invMaxParam) < dgFloat32(-1.0e-3f)) {
										for (dgInt32 i = 0; i < count; i ++) {
											contacts[i] = contacts[contactCount + i];
										}
										contactCount = 0;
									}
									maxParam = param;

									for (dgInt32 i = 0; i < count; i ++) {
										dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
										contacts[contactCount + i].m_collision0 = subShape;
									}
									contactCount += count;

									if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
										contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_world->m_contactTolerance);
									}

									if (maxParam == dgFloat32 (0.0f)) {
										break;
									}
								}
							}
						}
						proxy.m_referenceCollision = NULL; 
					}
				}

			} else if (me->m_type == m_leaf) {
				void* const frontNode = treeCollision->GetFrontNode(other);
				void* const backNode = treeCollision->GetBackNode(other);

				if (backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

				} else if (backNode && !frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				} else if (!backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				} else {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				}

			} else if (treeNodeIsLeaf) {

				stackPool[stack].m_myNode = me->m_left;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack].m_myNode = me->m_right;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

			} else if (nodeProxi.m_area > me->m_area) {

				dgAssert (me->m_type == m_node);
				void* const frontNode = treeCollision->GetFrontNode(other);
				void* const backNode = treeCollision->GetBackNode(other);
				if (backNode && frontNode) {
					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
				} else if (backNode && !frontNode) {
					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me->m_left;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

					stackPool[stack].m_myNode = me->m_right;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

				} else if (!backNode && frontNode) {

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_myNode = me->m_left;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

					stackPool[stack].m_myNode = me->m_right;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

				} else {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				}

			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack].m_myNode = me->m_left;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = treeNodeIsLeaf;
				stack++;

				stackPool[stack].m_myNode = me->m_right;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = treeNodeIsLeaf;
				stack++;
			}
		}
	}

	constraint->m_closestDistance = closestDist;
	proxy.m_contacts = contacts;	
	return contactCount;
}

dgFloat32 dgCollisionCompound::ConvexRayCast (const dgCollisionInstance* const convexInstance, const dgMatrix& instanceMatrix, const dgVector& instanceVeloc, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceInstance, void* const userData, dgInt32 threadId) const
{
	dgFloat32 dist = dgFloat32 (1.0e10f);
	if (m_root) {
		dgAssert (IsType (dgCollision::dgCollisionCompound_RTTI));
//		dgContact* const constraint = pair->m_contact;
//		dgBody* const body1 = constraint->GetBody1();
		if (convexInstance->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
			dist = ConvexRayCastSingleConvex (convexInstance, instanceMatrix, instanceVeloc, maxT, contactOut, referenceBody, referenceInstance, userData, threadId);
		} else if (convexInstance->IsType (dgCollision::dgCollisionCompound_RTTI)) {
			dgAssert (0);
//			contactCount = CalculateContactsToCompoundContinue (pair, proxy);
		} else if (convexInstance->IsType (dgCollision::dgCollisionBVH_RTTI)) {
//			contactCount = CalculateContactsToCollisionTreeContinue (pair, proxy);
		} else if (convexInstance->IsType (dgCollision::dgCollisionHeightField_RTTI)) {
			dgAssert (0);
			//				contactCount = CalculateContactsToHeightField (pair, proxy);
		} else {
			dgAssert (0);
//			dgAssert (body1->m_collision->IsType (dgCollision::dgCollisionUserMesh_RTTI));
			//				contactCount = CalculateContactsUserDefinedCollision (pair, proxy);
		}
	}
	return dist;
}

dgFloat32 dgCollisionCompound::ConvexRayCastSingleConvex (const dgCollisionInstance* const convexShape, const dgMatrix& shapeMatrix, const dgVector& shapeVeloc, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceInstance, void* const userData, dgInt32 threadId) const
{
	dgAssert (referenceInstance->GetChildShape() == this);

	if (!m_root) {
		return dgFloat32 (1.2f);
	}

	dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgAssert (referenceInstance->IsType(dgCollision::dgCollisionCompound_RTTI));
	const dgMatrix& compoundMatrix = referenceInstance->m_globalMatrix;
	dgMatrix shapeGlobalMatrix (convexShape->m_localMatrix * shapeMatrix);

	dgMatrix localMatrix (shapeGlobalMatrix * compoundMatrix.Inverse());
	dgVector localVeloc (compoundMatrix.UnrotateVector(shapeVeloc));

	dgVector shapeLocalP0; 
	dgVector shapeLocalP1; 
	convexShape->CalcAABB (localMatrix, shapeLocalP0, shapeLocalP1);

	dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), localVeloc);

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	dgVector minBox (m_root->m_p0 - shapeLocalP1);
	dgVector maxBox (m_root->m_p1 - shapeLocalP0);
	distance[0] = ray.BoxIntersect(minBox, maxBox);
	dgContactPoint tmpContact;
	while (stack) {
		stack --;
		dgFloat32 dist = distance[stack];
		if (dist > maxT) {
			break;
		} else {
			const dgNodeBase* const me = stackPool[stack];
			dgAssert (me);
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
				childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;
				dgFloat32 t = childInstance.ConvexRayCast (convexShape, shapeMatrix, shapeVeloc, maxT, tmpContact, NULL, referenceBody, userData, threadId);
				if (t < maxT) {
					contactOut = tmpContact;
					contactOut.m_collision0 = subShape;
					maxT = t;
				}

			} else {

				dgAssert (me->m_type == m_node);
				const dgNodeBase* const left = me->m_left;
				dgAssert (left);
				dgVector minBox (left->m_p0 - shapeLocalP1);
				dgVector maxBox (left->m_p1 - shapeLocalP0);
				dgFloat32 dist = ray.BoxIntersect(minBox, maxBox);
				if (dist < maxT) {
					dgInt32 j = stack;
					for ( ; j && (dist > distance[j - 1]); j --) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}

				const dgNodeBase* const right = me->m_right;
				dgAssert (right);
				minBox = right->m_p0 - shapeLocalP1;
				maxBox = right->m_p1 - shapeLocalP0;
				dist = ray.BoxIntersect(minBox, maxBox);
				if (dist < maxT) {
					dgInt32 j = stack;
					for ( ; j && (dist > distance[j - 1]); j --) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}

			}
		}
	}

	return maxT;
}
