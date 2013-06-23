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

class dgCollisionCompound::dgHeapNodePair
{
	public:
	dgNodeBase* m_nodeA;
	dgNodeBase* m_nodeB;
};


dgCollisionCompound::dgOOBBTestData::dgOOBBTestData (const dgMatrix& matrix)
	:m_matrix (matrix)
{
	for (dgInt32 i = 0; i < 3; i ++) {
		m_absMatrix[i][3] = dgFloat32 (0.0f);
		dgVector dir(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dir[i] = dgFloat32 (1.0f);
		for (dgInt32 j = 0; j < 3; j ++) {
			m_absMatrix[i][j] = dgAbsf (m_matrix[i][j]);
			dgVector axis (dir * m_matrix[j]);
			m_crossAxis[i][j] = axis;
			m_crossAxisAbs[i][j] = dgVector (dgAbsf (axis.m_x), dgAbsf (axis.m_y), dgAbsf (axis.m_z), dgFloat32 (0.0f));
			m_crossAxisDotAbs[i][j] = dgVector (dgAbsf (axis % matrix[0]), dgAbsf (axis % matrix[1]), dgAbsf (axis % matrix[2]), dgFloat32 (0.0f));
		}
	}
	m_absMatrix[3][3] = dgFloat32 (1.0f);
}


dgCollisionCompound::dgOOBBTestData::dgOOBBTestData (const dgMatrix& matrix, const dgVector& p0, const dgVector& p1)
	:m_matrix (matrix), m_localP0(p0), m_localP1(p1)
{
	m_size = (m_localP1 - m_localP0).CompProduct4 (dgVector::m_half);
	m_origin = (m_localP1 + m_localP0).CompProduct4 (dgVector::m_half);

	for (dgInt32 i = 0; i < 3; i ++) {
		m_absMatrix[i][3] = dgFloat32 (0.0f);
		dgVector dir(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dir[i] = dgFloat32 (1.0f);
		for (dgInt32 j = 0; j < 3; j ++) {
			m_absMatrix[i][j] = dgAbsf (m_matrix[i][j]);
			m_crossAxis[i][j] = dir * m_matrix[j];
		}
	}
	m_absMatrix[3][3] = dgFloat32 (1.0f);

	dgVector size (m_absMatrix.RotateVector(m_size));
	dgVector origin (m_matrix.TransformVector (m_origin));
	m_aabbP0 = origin - size;
	m_aabbP1 = origin + size;

	for (dgInt32 i = 0; i < 3; i ++) {
		for (dgInt32 j = 0; j < 3; j ++) {
			dgFloat32 d;
			dgFloat32 c;
			dgVector& axis = m_crossAxis[i][j];
			d = m_size.m_x * dgAbsf (axis % m_matrix[0]) + m_size.m_y * dgAbsf (axis % m_matrix[1]) + m_size.m_z * dgAbsf (axis % m_matrix[2]) + dgFloat32 (1.0e-3f); 
			c = origin % axis;

			m_extends[i][j] = dgVector (c - d, c + d, dgFloat32 (0.0f), dgFloat32 (0.0f));

			dgAssert (m_extends[i][j].m_x <= m_extends[i][j].m_y);
			m_crossAxisAbs[i][j] = dgVector (dgAbsf (axis.m_x), dgAbsf (axis.m_y), dgAbsf (axis.m_z), dgFloat32 (0.0f));
		}
	}
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
	dgVector p0;
	dgVector p1;
	m_shape->CalcAABB(m_shape->GetLocalMatrix (), p0, p1);
	SetBox (p0, p1);
}


dgCollisionCompound::dgNodeBase::dgNodeBase (dgNodeBase* const left, dgNodeBase* const right)
	:m_type(m_node)
	,m_left(left)
	,m_right(right)
	,m_parent(NULL)
	,m_shape(NULL)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

//	dgVector p0 (dgMin (left->m_p0.m_x, right->m_p0.m_x), dgMin (left->m_p0.m_y, right->m_p0.m_y), dgMin (left->m_p0.m_z, right->m_p0.m_z), dgFloat32 (0.0f));
//	dgVector p1 (dgMax (left->m_p1.m_x, right->m_p1.m_x), dgMax (left->m_p1.m_y, right->m_p1.m_y), dgMax (left->m_p1.m_z, right->m_p1.m_z), dgFloat32 (0.0f));
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

bool dgCollisionCompound::dgNodeBase::BoxTest (const dgOOBBTestData& data) const
{
	if (dgOverlapTest (data.m_aabbP0, data.m_aabbP1, m_p0, m_p1)) {
		dgVector origin (data.m_matrix.UntransformVector(m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);

		if (dgOverlapTest (p0, p1, data.m_localP0, data.m_localP1)) {
			for (dgInt32 i = 0; i < 3; i ++) {
				for (dgInt32 j = 0; j < 3; j ++) {
					const dgVector& axis = data.m_crossAxisAbs[i][j];
					dgFloat32 d = m_size.m_x * axis.m_x + m_size.m_y * axis.m_y + m_size.m_z * axis.m_z + dgFloat32 (1.0e-3f); 
					dgFloat32 c = m_origin % data.m_crossAxis[i][j];

					dgFloat32 x0 = c - d;
					dgFloat32 x1 = c + d;
					dgAssert (x0 <= x1);
					const dgVector& extend = data.m_extends[i][j];
					if ((x1 < extend.m_x) || (x0 > extend.m_y)) {
						return false;
					}
				}
			}
			return true;
		}
	}

	return false;
}


bool dgCollisionCompound::dgNodeBase::BoxTest (const dgOOBBTestData& data, const dgNodeBase* const otherNode) const
{
	dgVector otherOrigin (data.m_matrix.TransformVector(otherNode->m_origin));
	dgVector otherSize (data.m_absMatrix.RotateVector(otherNode->m_size));
	dgVector otherP0 (otherOrigin - otherSize);
	dgVector otherP1 (otherOrigin + otherSize);
	if (dgOverlapTest (m_p0, m_p1, otherP0, otherP1)) {

		dgVector origin (data.m_matrix.UntransformVector(m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);
		if (dgOverlapTest (p0, p1, otherNode->m_p0, otherNode->m_p1)) {
			for (dgInt32 i = 0; i < 3; i ++) {
				for (dgInt32 j = 0; j < 3; j ++) {
					const dgVector& axis = data.m_crossAxis[i][j];

					const dgVector& axisAbs = data.m_crossAxisAbs[i][j];
					dgFloat32 d = m_size.m_x * axisAbs.m_x + m_size.m_y * axisAbs.m_y + m_size.m_z * axisAbs.m_z + dgFloat32 (1.0e-3f); 
					dgFloat32 c = m_origin % axis;
					dgFloat32 x0 = c - d;
					dgFloat32 x1 = c + d;
					dgAssert (x0 <= x1);

					const dgVector& axisDotAbs = data.m_crossAxisDotAbs[i][j]; 
					d = otherNode->m_size.m_x * axisDotAbs.m_x + otherNode->m_size.m_y * axisDotAbs.m_y + otherNode->m_size.m_z * axisDotAbs.m_z + dgFloat32 (1.0e-3f); 
					c = otherOrigin % axis;
					dgFloat32 z0 = c - d;
					dgFloat32 z1 = c + d;
					dgAssert (z0 <= z1);

					if ((x1 < z0) || (x0 > z1)) {
						return false;
					}
				}
			}

			return true;
		}
	}
	return false;
}



dgCollisionCompound::dgCollisionCompound(dgWorld* const world)
	:dgCollision (world->GetAllocator(), 0, m_compoundCollision) 
	,m_boxMinRadius (dgFloat32 (0.0f))
	,m_boxMaxRadius (dgFloat32 (0.0f))
	,m_world(world)
	,m_root(NULL)
	,m_criticalSectionLock()
	,m_array (world->GetAllocator())
//	,m_preCollisionFilter(NULL)
{
	m_rtti |= dgCollisionCompound_RTTI;
}

dgCollisionCompound::dgCollisionCompound (const dgCollisionCompound& source)
	:dgCollision (source) 
	,m_boxMinRadius(source.m_boxMinRadius)
	,m_boxMaxRadius(source.m_boxMaxRadius)
	,m_world (source.m_world)	
	,m_root(NULL)
	,m_criticalSectionLock()
	,m_array (source.GetAllocator())
//	,m_preCollisionFilter(source.m_preCollisionFilter)
{
	m_rtti |= dgCollisionCompound_RTTI;

	dgTree<dgNodeBase*, dgInt32>::Iterator iter (source.m_array);
	for (iter.Begin(); iter; iter ++) {
		dgNodeBase* const node = iter.GetNode()->GetInfo();
		dgNodeBase* const newNode = new (m_allocator) dgNodeBase (node->GetShape());
		newNode->m_myNode = m_array.Insert(newNode, iter.GetNode()->GetKey());
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

dgCollisionCompound::dgCollisionCompound (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollision (world, deserialization, userData)
	,m_boxMinRadius (dgFloat32 (0.0f))
	,m_boxMaxRadius (dgFloat32 (0.0f))
	,m_world(world)
	,m_root(NULL)
	,m_criticalSectionLock()
	,m_array (world->GetAllocator())
//	,m_preCollisionFilter(NULL)
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
		boxMin = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		boxMax = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
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
		//dgVector size (m_root->m_size.m_x * dgAbsf(matrix[0][0]) + m_root->m_size.m_y * dgAbsf(matrix[1][0]) + m_root->m_size.m_z * dgAbsf(matrix[2][0]),  
		//			     m_root->m_size.m_x * dgAbsf(matrix[0][1]) + m_root->m_size.m_y * dgAbsf(matrix[1][1]) + m_root->m_size.m_z * dgAbsf(matrix[2][1]),  
		//			     m_root->m_size.m_x * dgAbsf(matrix[0][2]) + m_root->m_size.m_y * dgAbsf(matrix[1][2]) + m_root->m_size.m_z * dgAbsf(matrix[2][2]),
		//			     dgFloat32 (0.0f));
		dgVector size (matrix.m_front.Abs().Scale4(m_root->m_size.m_x) + matrix.m_up.Abs().Scale4(m_root->m_size.m_y) + matrix.m_right.Abs().Scale4(m_root->m_size.m_z));

		p0 = (origin - size) & dgVector::m_triplexMask;
		p1 = (origin + size) & dgVector::m_triplexMask;
	} else {
		p0 = dgVector (dgFloat32 (0.0f));
		p1 = dgVector (dgFloat32 (0.0f));
	}
}



dgInt32 dgCollisionCompound::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const
{
	return 0;
}


void dgCollisionCompound::DebugCollision (const dgMatrix& matrix, OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgCollisionInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		collision->DebugCollision (matrix, callback, userData);
	}
}


dgFloat32 dgCollisionCompound::RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
{
	if (!m_root) {
		return dgFloat32 (1.2f);
	}

	dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];
	
//	dgFloat32 maxParam = dgFloat32 (1.2f);
	dgFloat32 maxParam = maxT;
	dgFastRayTest ray (localP0, localP1);

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	distance[0] = ray.BoxIntersect(m_root->m_p0, m_root->m_p1);
	while (stack) {
		stack --;
		dgFloat32 dist = distance[stack];

		if (dist > maxParam) {
			break;
		} else {
			const dgNodeBase* const me = stackPool[stack];
			dgAssert (me);
			if (me->m_type == m_leaf) {
				dgContactPoint tmpContactOut;
				dgCollisionInstance* const shape = me->GetShape();
				dgVector p0 (shape->GetLocalMatrix().UntransformVector (localP0));
				dgVector p1 (shape->GetLocalMatrix().UntransformVector (localP1));
				dgFloat32 param = shape->RayCast (p0, p1, maxParam, tmpContactOut, NULL, body, userData);
				if (param < maxParam) {
					maxParam = param;
					contactOut.m_normal = shape->GetLocalMatrix().RotateVector (tmpContactOut.m_normal);;
//					contactOut.m_userId = tmpContactOut.m_userId;
					contactOut.m_shapeId0 = tmpContactOut.m_shapeId0;
				}

			} else {
				dgAssert (me->m_type == m_node);
				{
					const dgNodeBase* const node = me->m_left;
					dgAssert (node);
					dgFloat32 dist = ray.BoxIntersect(node->m_p0, node->m_p1);
					if (dist < maxParam) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = node;
						distance[j] = dist;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
					}
				}

				{
					const dgNodeBase* const node = me->m_right;
					dgAssert (node);
					dgFloat32 dist = ray.BoxIntersect(node->m_p0, node->m_p1);
					if (dist < maxParam) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = node;
						distance[j] = dist;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
					}
				}
			}
		}
	}
	return maxParam;
}

dgFloat32 dgCollisionCompound::ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& shapeMatrix, const dgVector& shapeVeloc, dgFloat32 minT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceShape, void* const userData, dgInt32 threadId) const
{
	dgAssert (referenceShape->GetChildShape() == this);

	if (!m_root) {
		return dgFloat32 (1.2f);
	}

	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];
	dgInt32 stack = 1;
	stackPool[0] = m_root;
//	dgFloat32 maxParam = dgFloat32 (1.2f);

	dgAssert (referenceShape->IsType(dgCollision::dgCollisionCompound_RTTI));
	const dgMatrix& compoundMatrix = referenceShape->m_globalMatrix;
	dgMatrix shapeGlobalMatrix (convexShape->m_localMatrix * shapeMatrix);

	dgMatrix localMatrix (shapeGlobalMatrix * compoundMatrix.Inverse());
	dgVector localVeloc (compoundMatrix.UnrotateVector(shapeVeloc));

	dgVector shapeLocalP0; 
	dgVector shapeLocalP1; 
	convexShape->CalcAABB (localMatrix, shapeLocalP0, shapeLocalP1);

	dgContactPoint tmpContact;
	dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), localVeloc);
	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack];
		dgAssert (me);

		dgVector minBox (me->m_p0 - shapeLocalP1);
		dgVector maxBox (me->m_p1 - shapeLocalP0);
		if (ray.BoxTest (minBox, maxBox)) {
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
				childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;
				dgFloat32 t = childInstance.ConvexRayCast (convexShape, shapeMatrix, shapeVeloc, minT, tmpContact, NULL, referenceBody, userData, threadId);
				if (t < minT) {
					contactOut = tmpContact;
dgAssert (0);
//					ray.Reset (t);
					minT = t;
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

	return minT;
}


dgFloat32 dgCollisionCompound::GetVolume () const
{
	dgFloat32 volume = dgFloat32 (0.0f);
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
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



dgVector dgCollisionCompound::CalculateVolumeIntegral (const dgMatrix& globalMatrix, GetBuoyancyPlane bouyancyPlane, void* const context) const
{

	dgVector totalVolume (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++)
	{
		dgCollisionConvex* const collision = (dgCollisionConvex*)iter.GetNode()->GetInfo()->GetShape()->GetChildShape();
		dgMatrix matrix (iter.GetNode()->GetInfo()->GetShape()->m_localMatrix * globalMatrix);
		dgVector vol (collision->CalculateVolumeIntegral (matrix, bouyancyPlane, context));
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


dgMatrix dgCollisionCompound::CalculateInertiaAndCenterOfMass (const dgVector& localScale, const dgMatrix& matrix) const
{
	dgVector inertiaII;
	dgVector crossInertia;
	dgVector centerOfMass;
	dgMatrix scaledMatrix(matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale3(localScale.m_x);
	scaledMatrix[1] = scaledMatrix[1].Scale3(localScale.m_y);
	scaledMatrix[2] = scaledMatrix[2].Scale3(localScale.m_z);
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
	dgVector origin_ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector inertia_ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector crossInertia_ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	dgPolyhedraMassProperties localData;
	DebugCollision (dgGetIdentityMatrix(), CalculateInertia, &localData);
	dgFloat32 volume_ = localData.MassProperties (origin_, inertia_, crossInertia_);
	dgAssert (volume_ > dgFloat32 (0.0f));

	dgFloat32 invVolume_ = dgFloat32 (1.0f)/volume_;
	m_centerOfMass = origin_.Scale3 (invVolume_);
	m_centerOfMass.m_w = volume_;
	m_inertia = inertia_.Scale3 (invVolume_);
	m_crossInertia = crossInertia_.Scale3(invVolume_);
#endif


	dgFloat32 volume = dgFloat32 (0.0f);
	dgVector origin (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector inertiaII (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector inertiaIJ (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
	for (iter.Begin(); iter; iter ++) {
		dgCollisionInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		dgMatrix shapeInertia (collision->CalculateInertia());
		dgFloat32 shapeVolume = collision->GetVolume();

		volume += shapeVolume;
		origin += shapeInertia.m_posit.Scale3(shapeVolume);
		inertiaII += dgVector (shapeInertia[0][0], shapeInertia[0][0], shapeInertia[0][0], dgFloat32 (0.0f)).Scale3 (shapeVolume);
		inertiaIJ += dgVector (shapeInertia[1][2], shapeInertia[0][2], shapeInertia[0][1], dgFloat32 (0.0f)).Scale3 (shapeVolume);
	}
	dgFloat32 invVolume = dgFloat32 (1.0f)/volume;
	m_inertia = inertiaII.Scale3 (invVolume);
	m_crossInertia = inertiaIJ.Scale3 (invVolume);
	m_centerOfMass = origin.Scale3 (invVolume);
	m_centerOfMass.m_w = volume;

	dgCollision::MassProperties ();
}


void dgCollisionCompound::BeginAddRemove ()
{
}


void dgCollisionCompound::EndAddRemove ()
{
	if (m_root) {
		if (m_root->m_type == m_node) {

			dgWorld* const world = m_world;
			dgInt32 maxPasses = 2 * dgExp2 (m_array.GetCount() * 2) + 1;
			dgFloat64 newCost = dgFloat32 (1.0e20f);
			dgFloat64 prevCost = newCost;

			dgList<dgNodeBase*> list (GetAllocator());
			//dgNodeBase* pool[DG_COMPOUND_STACK_DEPTH];
			dgList<dgNodeBase*> stack (GetAllocator());
			stack.Append(m_root);
			while (stack.GetCount()) {
				dgList<dgNodeBase*>::dgListNode* const stackNode = stack.GetLast();
				dgNodeBase* const node = stackNode->GetInfo();
				stack.Remove(stackNode);

				if (node->m_type == m_node) {
					list.Append(node);
				}

				if (node->m_type == m_node) {
					stack.Append(node->m_right);
					stack.Append(node->m_left);
				} 
			}

			{
				dgThreadHiveScopeLock lock (world, &m_criticalSectionLock);
				do {
					prevCost = newCost;
					for (dgList<dgNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
						dgNodeBase* const node = listNode->GetInfo();
						ImproveNodeFitness (node);
					}

					newCost = dgFloat32 (0.0f);
					for (dgList<dgNodeBase*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
						dgNodeBase* const node = listNode->GetInfo();
						newCost += node->m_area;
					}

					maxPasses --;
				} while (maxPasses && (newCost < (prevCost * dgFloat32 (0.9999f))));
			}


			while (m_root->m_parent) {
				m_root = m_root->m_parent;
			}
		}

		m_boxMinRadius = dgMin(m_root->m_size.m_x, m_root->m_size.m_y, m_root->m_size.m_z);
		m_boxMaxRadius = dgSqrt (m_root->m_size % m_root->m_size);

		MassProperties ();
	}
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::AddCollision (dgCollisionInstance* const shape)
{
	dgNodeBase* const newNode = new (m_allocator) dgNodeBase (shape);
	newNode->m_myNode = m_array.Insert(newNode, m_array.GetCount());

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


void dgCollisionCompound::RemoveCollision (dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node)
{
	if (node) {
		dgCollisionInstance* const instance = node->GetInfo()->GetShape();
		instance->AddRef();
		RemoveCollision (node->GetInfo());
		instance->Release();
		m_array.Remove(node);
	}
}

void dgCollisionCompound::SetCollisionMatrix (dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node, const dgMatrix& matrix)
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
			dgAssert (treeNode->m_parent->m_left == treeNode);
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

dgInt32 dgCollisionCompound::GetNodeIndex(dgTree<dgNodeBase*, dgInt32>::dgTreeNode* const node) const
{
	return node->GetKey();
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::FindNodeByIndex (dgInt32 index) const
{
	return m_array.Find (index);
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::GetFirstNode () const
{
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
	iter.Begin();

	dgTree<dgNodeBase*, dgInt32>::dgTreeNode* node = NULL;
	if (iter) {
		node = iter.GetNode();
	}
	return node; 
}

dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* dgCollisionCompound::GetNextNode (dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* const node) const
{
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
	iter.Set (node);
	iter ++;
	dgTree<dgNodeBase*, dgInt32>::dgTreeNode* nextNode = NULL;
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
	dgAssert (0);
	return dgVector (0,0,0,0);
/*
	dgFloat32 aabbProjection[DG_COMPOUND_STACK_DEPTH];
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	aabbProjection[0] = dgFloat32 (1.0e10f);

	dgFloat32 maxProj = dgFloat32 (-1.0e20f); 
	dgVector searchDir (m_offset.UnrotateVector(dir));

	dgInt32 ix = (searchDir[0] > dgFloat32 (0.0f)) ? 1 : 0;
	dgInt32 iy = (searchDir[1] > dgFloat32 (0.0f)) ? 1 : 0;
	dgInt32 iz = (searchDir[2] > dgFloat32 (0.0f)) ? 1 : 0;
	dgVector supportVertex (dgFloat32 (0.0f), dgFloat32 (0.0f),  dgFloat32 (0.0f),  dgFloat32 (0.0f));   

	while (stack) {

		stack--;
		dgFloat32 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) {
			const dgNodeBase* const me = stackPool[stack];

			if (me->m_type == m_leaf) {
				dgCollision* const shape = me->GetShape()->GetChildShape();

				dgVector newDir (shape->m_offset.UnrotateVector(searchDir)); 
				dgVector vertex (shape->m_offset.TransformVector (shape->SupportVertex(newDir)));		
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

	return m_offset.TransformVector (supportVertex);
*/
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
	dgTree<dgNodeBase*, dgInt32>::Iterator iter (m_array);
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
		if (body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
			if (proxy.m_continueCollision) {
				contactCount = CalculateContactsToSingleContinue (pair, proxy);
			} else {
				contactCount = CalculateContactsToSingle (pair, proxy);
			}

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
	pair->m_contactCount = contactCount;
	return contactCount;
}


void dgCollisionCompound::CalculateCollisionTreeArea(dgNodePairs& pairOut, const dgCollisionBVH* const collisionTree, const void* const treeNode) const
{
	collisionTree->GetNodeAABB(treeNode, pairOut.m_treeNodeP0, pairOut.m_treeNodeP1);
	pairOut.m_treeNodeSize = (pairOut.m_treeNodeP1 - pairOut.m_treeNodeP0).Scale3 (dgFloat32 (0.5f));
	pairOut.m_treeNodeOrigin = (pairOut.m_treeNodeP1 + pairOut.m_treeNodeP0).Scale3 (dgFloat32 (0.5f));
	dgVector size (pairOut.m_treeNodeSize.m_y, pairOut.m_treeNodeSize.m_z, pairOut.m_treeNodeSize.m_x, dgFloat32 (0.0f));
	pairOut.m_treeNodeArea = pairOut.m_treeNodeSize  % size;
	dgAssert (pairOut.m_treeNodeArea > dgFloat32 (0.0f));
}


dgInt32 dgCollisionCompound::ClosestDitance (dgBody* const compoundBody, dgTriplex& contactA, dgBody* const bodyB, dgTriplex& contactB, dgTriplex& normalAB) const
{
	if (m_root) {
		if (bodyB->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
			return ClosestDitanceToConvex (compoundBody, contactA, bodyB, contactB, normalAB);
		} else {
			return ClosestDitanceToCompound (compoundBody, contactA, bodyB, contactB, normalAB);
		}
	}
	return 0;
}


dgInt32 dgCollisionCompound::ClosestDitanceToConvex (dgBody* const compoundBody, dgTriplex& contactA, dgBody* const convexBodyB, dgTriplex& contactB, dgTriplex& normalAB) const
{
	dgAssert (0);
	return 0;
/*
	dgVector p0;
	dgVector p1;
	dgContactPoint contact0;
	dgContactPoint contact1;
	dgContactPoint contacts[16];
	dgCollisionParamProxy proxy(NULL, contacts, 0);
	proxy.m_referenceBody = compoundBody;
	proxy.m_floatingBody = convexBodyB;
	proxy.m_floatingCollision = convexBodyB->m_collision;
	proxy.m_floatingMatrix = convexBodyB->m_collisionWorldMatrix ;

	proxy.m_timestep = dgFloat32 (0.0f);
	proxy.m_penetrationPadding = dgFloat32 (0.0f);
//	proxy.m_unconditionalCast = 1;
	proxy.m_continueCollision = 0;
	proxy.m_maxContacts = 16;
	proxy.m_contacts = &contacts[0];

	dgMatrix myMatrix (m_offset * compoundBody->m_matrix);
	dgMatrix matrix (convexBodyB->m_collisionWorldMatrix * myMatrix.Inverse());
	convexBodyB->m_collision->CalcAABB(matrix, p0, p1);

	dgUnsigned8 pool[64 * (sizeof (dgNodeBase*) + sizeof (dgFloat32))];
	dgUpHeap<dgNodeBase*, dgFloat32> heap (pool, sizeof (pool));

	dgInt32 retFlag = 1;
	
	dgNodeBase* node = m_root;
	heap.Push(node, dgBoxDistanceToOrigin2(p0, p1, m_root->m_p0, m_root->m_p1));

	dgFloat32 minDist2 = dgFloat32 (1.0e10f);
	while (heap.GetCount() && (heap.Value() <= minDist2)) {
		const dgNodeBase* const node = heap[0];
		heap.Pop();
		if (node->m_type == m_leaf) {
			dgCollisionConvex* const collision = (dgCollisionConvex*) node->GetShape()->GetChildShape();
			retFlag = 0;
			proxy.m_referenceCollision = collision;
			proxy.m_referenceMatrix = collision->m_offset * myMatrix;
			dgInt32 flag = m_world->ClosestPoint (proxy);
			if (flag) {
				retFlag = 1;
				dgVector err (contacts[0].m_point - contacts[1].m_point);
				dgFloat32 dist2 = err % err;
				if (dist2 < minDist2) {
					minDist2 = dist2;
					contact0 = contacts[0];
					contact1 = contacts[1];
				}
			} else {
				dgAssert (0);
				break;
			}

		} else {
			dgNodeBase* left = node->m_left;
			dgNodeBase* right = node->m_right;
			heap.Push(left, dgBoxDistanceToOrigin2(p0, p1, left->m_p0, left->m_p1));
			heap.Push(right, dgBoxDistanceToOrigin2(p0, p1, right->m_p0, right->m_p1));
		}
	}

	if (retFlag) {
		contactA.m_x = contact0.m_point.m_x;
		contactA.m_y = contact0.m_point.m_y;
		contactA.m_z = contact0.m_point.m_z;

		contactB.m_x = contact1.m_point.m_x;
		contactB.m_y = contact1.m_point.m_y;
		contactB.m_z = contact1.m_point.m_z;

		normalAB.m_x = contact0.m_normal.m_x;
		normalAB.m_y = contact0.m_normal.m_y;
		normalAB.m_z = contact0.m_normal.m_z;
	}
	return retFlag;
*/
}

dgInt32 dgCollisionCompound::ClosestDitanceToCompound (dgBody* const compoundBodyA, dgTriplex& contactA, dgBody* const compoundBodyB, dgTriplex& contactB, dgTriplex& normalAB) const
{
	dgAssert (0);
	return 0;
/*
	dgCollisionCompound* const compoundCollisionB = (dgCollisionCompound *) compoundBodyB->m_collision;

	dgVector p0;
	dgVector p1;
	dgContactPoint contact0;
	dgContactPoint contact1;
	dgContactPoint contacts[16];
	dgCollisionParamProxy proxy(NULL, contacts, 0);
	
	proxy.m_referenceBody = compoundBodyA;
	proxy.m_floatingBody = compoundBodyB;
	proxy.m_timestep = dgFloat32 (0.0f);
	proxy.m_penetrationPadding = dgFloat32 (0.0f);
//	proxy.m_unconditionalCast = 1;
	proxy.m_continueCollision = 0;
	proxy.m_maxContacts = 16;
	proxy.m_contacts = &contacts[0];
	

	dgUnsigned8 pool[128 * (sizeof (dgHeapNodePair) + sizeof (dgFloat32))];
	dgUpHeap<dgHeapNodePair, dgFloat32> heap (pool, sizeof (pool));

	dgMatrix matrixA (m_offset * compoundBodyA->m_matrix);
	dgMatrix matrixB (compoundCollisionB->m_offset * compoundBodyB->m_matrix);
	dgMatrix matrixBA (matrixB * matrixA.Inverse());
	dgMatrix matrixAB (matrixBA.Inverse());
	dgVector pA0;
	dgVector pA1;
	matrixBA.TransformBBox (compoundCollisionB->m_root->m_p0, compoundCollisionB->m_root->m_p1, pA0, pA1);

	dgInt32 retFlag = 1;
	
	dgHeapNodePair pair; 
	pair.m_nodeA = m_root;
	pair.m_nodeB = compoundCollisionB->m_root;
	heap.Push(pair, dgBoxDistanceToOrigin2(pA0, pA1, m_root->m_p0, m_root->m_p1));

	dgFloat32 minDist2 = dgFloat32 (1.0e10f);
	while (heap.GetCount() && (heap.Value() <= minDist2)) {
		dgHeapNodePair pair = heap[0];
		heap.Pop();

		if ((pair.m_nodeA->m_type == m_leaf) && (pair.m_nodeB->m_type == m_leaf)) {
			retFlag = 0;
			dgCollisionConvex* const collisionA = (dgCollisionConvex*) pair.m_nodeA->GetShape()->GetChildShape();
			proxy.m_referenceCollision = collisionA;
			proxy.m_referenceMatrix = collisionA->m_offset * matrixA;

			dgCollisionConvex* const collisionB = (dgCollisionConvex*) pair.m_nodeB->GetShape()->GetChildShape();
			proxy.m_floatingCollision = collisionB;
			proxy.m_floatingMatrix = collisionB->m_offset * matrixB;

			dgInt32 flag = m_world->ClosestPoint (proxy);
			if (flag) {
				retFlag = 1;
				dgVector err (contacts[0].m_point - contacts[1].m_point);
				dgFloat32 dist2 = err % err;
				if (dist2 < minDist2) {
					minDist2 = dist2;
					contact0 = contacts[0];
					contact1 = contacts[1];
				}
			} else {
				dgAssert (0);
				break;
			}

		} else if (pair.m_nodeA->m_type == m_leaf) {
			dgVector pB0;
			dgVector pB1;

			dgHeapNodePair pair1; 
			pair1.m_nodeA = pair.m_nodeA;
			pair1.m_nodeB = pair.m_nodeB->m_left;
			matrixAB.TransformBBox (pair1.m_nodeA->m_p0, pair1.m_nodeA->m_p1, pB0, pB1);			
			heap.Push(pair1, dgBoxDistanceToOrigin2(pB0, pB1, pair1.m_nodeB->m_p0, pair1.m_nodeB->m_p1));

			dgHeapNodePair pair2; 
			pair2.m_nodeA = pair.m_nodeA;
			pair2.m_nodeB = pair.m_nodeB->m_right;
			matrixAB.TransformBBox (pair2.m_nodeA->m_p0, pair2.m_nodeA->m_p1, pA0, pA1);			
			heap.Push(pair2, dgBoxDistanceToOrigin2(pB0, pB1, pair2.m_nodeB->m_p0, pair2.m_nodeB->m_p1));

		} else if (pair.m_nodeB->m_type == m_leaf) {
			dgVector pA0;
			dgVector pA1;

			dgHeapNodePair pair1; 
			pair1.m_nodeA = pair.m_nodeA->m_left;
			pair1.m_nodeB = pair.m_nodeB;
			matrixBA.TransformBBox (pair1.m_nodeB->m_p0, pair1.m_nodeB->m_p1, pA0, pA1);			
			heap.Push(pair1, dgBoxDistanceToOrigin2(pA0, pA1, pair1.m_nodeA->m_p0, pair1.m_nodeA->m_p1));

			dgHeapNodePair pair2; 
			pair2.m_nodeA = pair.m_nodeA->m_left;
			pair2.m_nodeB = pair.m_nodeB;
			matrixBA.TransformBBox (pair2.m_nodeB->m_p0, pair2.m_nodeB->m_p1, pA0, pA1);			
			heap.Push(pair2, dgBoxDistanceToOrigin2(pA0, pA1, pair2.m_nodeA->m_p0, pair2.m_nodeA->m_p1));

		} else {

			dgVector pA0;
			dgVector pA1;

			dgHeapNodePair pair1; 
			pair1.m_nodeA = pair.m_nodeA->m_left;
			pair1.m_nodeB = pair.m_nodeB->m_left;
			matrixBA.TransformBBox (pair1.m_nodeB->m_p0, pair1.m_nodeB->m_p1, pA0, pA1);			
			heap.Push(pair1, dgBoxDistanceToOrigin2(pA0, pA1, pair1.m_nodeA->m_p0, pair1.m_nodeA->m_p1));

			dgHeapNodePair pair2; 
			pair2.m_nodeA = pair.m_nodeA->m_left;
			pair2.m_nodeB = pair.m_nodeB->m_right;
			matrixBA.TransformBBox (pair2.m_nodeB->m_p0, pair2.m_nodeB->m_p1, pA0, pA1);			
			heap.Push(pair2, dgBoxDistanceToOrigin2(pA0, pA1, pair2.m_nodeA->m_p0, pair2.m_nodeA->m_p1));

			dgHeapNodePair pair3; 
			pair3.m_nodeA = pair.m_nodeA->m_right;
			pair3.m_nodeB = pair.m_nodeB->m_left;
			matrixBA.TransformBBox (pair3.m_nodeB->m_p0, pair3.m_nodeB->m_p1, pA0, pA1);			
			heap.Push(pair3, dgBoxDistanceToOrigin2(pA0, pA1, pair3.m_nodeA->m_p0, pair3.m_nodeA->m_p1));

			dgHeapNodePair pair4; 
			pair4.m_nodeA = pair.m_nodeA->m_right;
			pair4.m_nodeB = pair.m_nodeB->m_right;
			matrixBA.TransformBBox (pair4.m_nodeB->m_p0, pair4.m_nodeB->m_p1, pA0, pA1);			
			heap.Push(pair4, dgBoxDistanceToOrigin2(pA0, pA1, pair4.m_nodeA->m_p0, pair4.m_nodeA->m_p1));
		}
	}

	if (retFlag) {
		contactA.m_x = contact0.m_point.m_x;
		contactA.m_y = contact0.m_point.m_y;
		contactA.m_z = contact0.m_point.m_z;

		contactB.m_x = contact1.m_point.m_x;
		contactB.m_y = contact1.m_point.m_y;
		contactB.m_z = contact1.m_point.m_z;

		normalAB.m_x = contact0.m_normal.m_x;
		normalAB.m_y = contact0.m_normal.m_y;
		normalAB.m_z = contact0.m_normal.m_z;
	}
	return retFlag;
*/
}




dgInt32 dgCollisionCompound::CalculateContactsToSingle (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgVector p0;
	dgVector p1;
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
	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * compoundBody->m_matrix);
	dgMatrix matrix (otherBody->m_collision->GetGlobalMatrix() * myMatrix.Inverse());
	otherInstance->CalcAABB(dgGetIdentityMatrix(), p0, p1);
	dgOOBBTestData data (matrix, p0, p1);

	dgInt32 stack = 1;
	stackPool[0] = m_root;
	const dgContactMaterial* const material = constraint->GetMaterial();

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
						processContacts = material->m_compoundAABBOverlap (*material, compoundBody, me, otherBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = &contacts[contactCount];

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);
						for (dgInt32 i = 0; i < count; i ++) {
							dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
							contacts[contactCount + i].m_collision0 = subShape;
						}
						contactCount += count;

						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
							contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, DG_REDUCE_CONTACT_TOLERANCE);
						}
						//childInstance.SetUserData(NULL);
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

dgInt32 dgCollisionCompound::CalculateContactsToSingleContinue(dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
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

	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * compoundBody->m_matrix);
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
						processContacts = material->m_compoundAABBOverlap (*material, compoundBody, me, otherBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = &contacts[contactCount];

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);

						closestDist = dgMin(closestDist, constraint->m_closestDistance);
						if (count) {
							dgFloat32 param = proxy.m_timestep;
							dgAssert (param >= dgFloat32 (0.0f));
							if (param < maxParam) {
dgAssert (0);
//								ray.Reset (param);

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
									contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, DG_REDUCE_CONTACT_TOLERANCE);
								}

								if (maxParam == dgFloat32 (0.0f)) {
									break;
								}
							}
						}
						//childInstance.SetUserData(NULL);
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

	dgMatrix myMatrix (myCompoundInstance->GetLocalMatrix() * myBody->m_matrix);
	dgMatrix otherMatrix (otherCompoundInstance->GetLocalMatrix() * otherBody->m_matrix);
	dgOOBBTestData data (otherMatrix * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0][0] = m_root;
	stackPool[0][1] = otherCompound->m_root;
	const dgContactMaterial* const material = constraint->GetMaterial();

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
					processContacts = material->m_compoundAABBOverlap (*material, myBody, me, otherBody, other, proxy.m_threadIndex);
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
						proxy.m_contacts = &contacts[contactCount];

						dgInt32 count = m_world->CalculateConvexToConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						for (dgInt32 i = 0; i < count; i ++) {
							dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
							dgAssert (contacts[contactCount + i].m_collision1 == &otherChildInstance);
							contacts[contactCount + i].m_collision0 = subShape;
							contacts[contactCount + i].m_collision1 = otherSubShape;
						}
						contactCount += count;
						if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
							contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, DG_REDUCE_CONTACT_TOLERANCE);
						}
						//childInstance.SetUserData(NULL);
						//otherChildInstance.SetUserData(NULL);
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
	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);
	dgOOBBTestData data (proxy.m_floatingCollision->m_globalMatrix * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0].m_myNode = m_root;
	stackPool[0].m_treeNode = treeCollision->GetRootNode();
	stackPool[0].m_treeNodeIsLeaf = 0;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;

	const dgContactMaterial* const material = constraint->GetMaterial();

	CalculateCollisionTreeArea(stackPool[0], treeCollision, stackPool[0].m_treeNode);

	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {

		stack --;
		const dgNodePairs* const stackEntry = &stackPool[stack];

		dgNodeBase* const me = stackEntry->m_myNode;
		const void* const other = stackEntry->m_treeNode;
		dgInt32 treeNodeIsLeaf = stackEntry->m_treeNodeIsLeaf;

		dgAssert (me && other);
#ifdef _DEBUG
		dgVector p0;
		dgVector p1;
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
		if (me->BoxTest (data, &nodeProxi)) {
			if ((me->m_type == m_leaf) && treeNodeIsLeaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me, treeBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = &contacts[contactCount];

						dgInt32 count = m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						for (dgInt32 i = 0; i < count; i ++) {
							dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
							contacts[contactCount + i].m_collision0 = subShape;
						}
						contactCount += count;


						if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
							contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, DG_REDUCE_CONTACT_TOLERANCE);
						}
						//childInstance.SetUserData(NULL);
					}
				}

			} else if (me->m_type == m_leaf) {
				void* const frontNode = treeCollision->GetFrontNode(other);
				void* const backNode = treeCollision->GetBackNode(other);
				if (backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, backNode);
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, frontNode);
					stack++;

				} else if (backNode && !frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, backNode);
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;

				} else if (!backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, frontNode);
					stack++;

					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;

				} else {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;
				}

			} else if (treeNodeIsLeaf) {
				stackPool[stack].m_myNode = me->m_left;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
				stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
				stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
				stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
				stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack].m_myNode = me->m_right;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
				stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
				stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
				stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
				stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
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
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, backNode);
					stack++;

					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, frontNode);
					stack++;
				} else if (backNode && !frontNode) {
					stackPool[stack].m_myNode = (dgNodeBase*) me;
					stackPool[stack].m_treeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, backNode);
					stack++;

					stackPool[stack].m_myNode = me->m_left;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;

					stackPool[stack].m_myNode = me->m_right;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;

				} else if (!backNode && frontNode) {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					CalculateCollisionTreeArea(stackPool[stack], treeCollision, frontNode);
					stack++;

					stackPool[stack].m_myNode = me->m_left;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;

					stackPool[stack].m_myNode = me->m_right;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;

				} else {
					stackPool[stack].m_myNode = me;
					stackPool[stack].m_treeNode = other;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
					stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
					stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
					stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
					stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
					stack++;
				}

			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack].m_myNode = me->m_left;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = treeNodeIsLeaf;
				stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
				stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
				stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
				stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
				stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
				stack++;

				stackPool[stack].m_myNode = me->m_right;
				stackPool[stack].m_treeNode = other;
				stackPool[stack].m_treeNodeIsLeaf = treeNodeIsLeaf;
				stackPool[stack].m_treeNodeP0 = nodeProxi.m_p0;
				stackPool[stack].m_treeNodeP1 = nodeProxi.m_p1;
				stackPool[stack].m_treeNodeArea = nodeProxi.m_area;
				stackPool[stack].m_treeNodeSize = nodeProxi.m_size;
				stackPool[stack].m_treeNodeOrigin = nodeProxi.m_origin;
				stack++;
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
	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);

	dgOOBBTestData data (terrainInstance->GetGlobalMatrix() * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0] = m_root;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;
	const dgContactMaterial* const material = constraint->GetMaterial();

	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	while (stack) {
		stack --;
		const dgNodeBase* const me = stackPool[stack];

		dgVector origin (data.m_matrix.UntransformVector(me->m_origin));
		dgVector size (data.m_absMatrix.UnrotateVector(me->m_size));
		dgVector p0 (origin - size);
		dgVector p1 (origin + size);

		terrainCollision->GetLocalAABB (p0, p1, nodeProxi.m_p0, nodeProxi.m_p1);
		nodeProxi.m_size = (nodeProxi.m_p1 - nodeProxi.m_p0).Scale3 (dgFloat32 (0.5f));
		nodeProxi.m_origin = (nodeProxi.m_p1 + nodeProxi.m_p0).Scale3 (dgFloat32 (0.5f));
		if (me->BoxTest (data, &nodeProxi)) {
			if (me->m_type == m_leaf) {
				dgCollisionInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) {
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me, terrainBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = &contacts[contactCount];

						dgInt32 count = 0;
						count += m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						for (dgInt32 i = 0; i < count; i ++) {
							dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
							contacts[contactCount + i].m_collision0 = subShape;
						}
						contactCount += count;

						if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
							contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, DG_REDUCE_CONTACT_TOLERANCE);
						}
						//childInstance.SetUserData(NULL);
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
	dgMatrix myMatrix (compoundInstance->GetLocalMatrix() * myBody->m_matrix);

	dgOOBBTestData data (userMeshInstance->GetGlobalMatrix() * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0] = m_root;

	dgNodeBase nodeProxi;
	nodeProxi.m_left = NULL;
	nodeProxi.m_right = NULL;
	const dgContactMaterial* const material = constraint->GetMaterial();

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
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me, userBody, NULL, proxy.m_threadIndex);
					}
					if (processContacts) {
						dgCollisionInstance childInstance (*subShape, subShape->GetChildShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;
						proxy.m_referenceCollision = &childInstance; 

						proxy.m_maxContacts = DG_MAX_CONTATCS - contactCount;
						proxy.m_contacts = &contacts[contactCount];

						dgInt32 count = 0;
						count += m_world->CalculateConvexToNonConvexContacts (proxy);
						closestDist = dgMin(closestDist, constraint->m_closestDistance);

						for (dgInt32 i = 0; i < count; i ++) {
							dgAssert (contacts[contactCount + i].m_collision0 == &childInstance);
							contacts[contactCount + i].m_collision0 = subShape;
						}
						contactCount += count;

						if (contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
							contactCount = m_world->ReduceContacts (contactCount, contacts, DG_CONSTRAINT_MAX_ROWS / 3, DG_REDUCE_CONTACT_TOLERANCE);
						}
						//childInstance.SetUserData(NULL);
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

