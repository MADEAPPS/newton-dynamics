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
#include "dgContact.h"
#include "dgCollisionBox.h"
#include "dgCollisionBVH.h"
#include "dgCollisionMesh.h"
#include "dgCollisionNull.h"
#include "dgCollisionCone.h"
#include "dgCollisionSphere.h"
#include "dgCollisionCapsule.h"
#include "dgCollisionCylinder.h"
#include "dgCollisionInstance.h"
#include "dgCollisionCompound.h"
#include "dgCollisionHeightField.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionTaperedCapsule.h"
#include "dgCollisionTaperedCylinder.h"
#include "dgCollisionChamferCylinder.h"
#include "dgCollisionDeformableSolidMesh.h"
#include "dgCollisionDeformableClothPatch.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


dgVector dgCollisionInstance::m_padding (DG_MAX_COLLISION_AABB_PADDING, DG_MAX_COLLISION_AABB_PADDING, DG_MAX_COLLISION_AABB_PADDING, dgFloat32 (0.0f));

dgCollisionInstance::dgCollisionInstance()
	:m_globalMatrix(dgGetIdentityMatrix())
	,m_localMatrix (dgGetIdentityMatrix())
	,m_scale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_invScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_maxScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_userDataID(0)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(NULL)
	,m_childShape (NULL)
	,m_scaleIsUnit(true)
	,m_scaleIsUniform(true)
	,m_collisionMode(true)
{
}


dgCollisionInstance::dgCollisionInstance(const dgWorld* const world, const dgCollision* const childCollision, dgInt32 shapeID, const dgMatrix &matrix)
	:m_globalMatrix(matrix)
	,m_localMatrix (matrix)
	,m_scale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_invScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_maxScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_userDataID(shapeID)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(world)
	,m_childShape (childCollision)
	,m_scaleIsUnit (true)
	,m_scaleIsUniform (true)
	,m_collisionMode(true)
{
	m_childShape->AddRef();
}

dgCollisionInstance::dgCollisionInstance(const dgCollisionInstance& instance)
	:m_globalMatrix(instance.m_globalMatrix)
	,m_localMatrix (instance.m_localMatrix)
	,m_scale(instance.m_scale)
	,m_invScale(instance.m_invScale)
	,m_maxScale(instance.m_maxScale)
	,m_userDataID(instance.m_userDataID)
	,m_refCount(1)
	,m_userData(instance.m_userData)
	,m_world(instance.m_world)
	,m_childShape (instance.m_childShape)
	,m_scaleIsUnit (instance.m_scaleIsUnit)
	,m_scaleIsUniform (instance.m_scaleIsUniform)
	,m_collisionMode(instance.m_collisionMode)
{
	if (m_childShape->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		if (m_childShape->IsType (dgCollision::dgCollisionCompoundBreakable_RTTI)) {
			dgAssert (0);
//			dgCollisionCompoundBreakable* const compound = (dgCollisionCompoundBreakable*) collision;
//			collision = new (m_world->GetAllocator()) dgCollisionCompoundBreakable (*compound);
		} else if (m_childShape->IsType (dgCollision::dgCollisionScene_RTTI)) {
			dgCollisionScene* const scene = (dgCollisionScene*) m_childShape;
			m_childShape = new (m_world->GetAllocator()) dgCollisionScene (*scene);
		} else {
			dgCollisionCompound *const compound = (dgCollisionCompound*) m_childShape;
			m_childShape = new (m_world->GetAllocator()) dgCollisionCompound (*compound);
		}
	} else if (m_childShape->IsType (dgCollision::dgCollisionDeformableClothPatch_RTTI)) {
		dgCollisionDeformableClothPatch* const deformable = (dgCollisionDeformableClothPatch*) m_childShape;
		m_childShape = new (m_world->GetAllocator()) dgCollisionDeformableClothPatch (*deformable);
	} else if (m_childShape->IsType (dgCollision::dgCollisionDeformableSolidMesh_RTTI)) {
		dgAssert (0);
		//dgCollisionDeformableSolidMesh* const deformable = (dgCollisionDeformableSolidMesh*) m_childShape;
		//m_childShape = new (m_world->GetAllocator()) dgCollisionDeformableSolidMesh (*deformable);
	} else {
		m_childShape->AddRef();
	}

	if (m_world->m_onCollisionInstanceCopyConstrutor) {
		m_world->m_onCollisionInstanceCopyConstrutor (m_world, this, &instance);
	}
}

dgCollisionInstance::dgCollisionInstance(const dgWorld* const constWorld, dgDeserialize deserialization, void* const userData)
	:m_globalMatrix(dgGetIdentityMatrix())
	,m_localMatrix (dgGetIdentityMatrix())
	,m_scale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_invScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_maxScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_userDataID(0)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(constWorld)
	,m_childShape (NULL)
	,m_scaleIsUnit (true)
	,m_scaleIsUniform (true)
	,m_collisionMode(true)
{
	dgInt32 saved;
	dgInt32 signature;
	dgInt32 primitive;
	
	deserialization (userData, &m_globalMatrix, sizeof (m_globalMatrix));
	deserialization (userData, &m_localMatrix, sizeof (m_localMatrix));
	deserialization (userData, &m_scale, sizeof (m_scale));
	deserialization (userData, &m_invScale, sizeof (m_scale));
	deserialization (userData, &m_maxScale, sizeof (m_maxScale));
	deserialization (userData, &m_userDataID, sizeof (m_userDataID));
	deserialization (userData, &m_flags, sizeof (m_flags));
	deserialization (userData, &primitive, sizeof (primitive));
	deserialization (userData, &signature, sizeof (signature));
	deserialization (userData, &saved, sizeof (saved));

	dgWorld* const world = (dgWorld*) constWorld;
	if (saved) {
		const dgCollision* collision = NULL;
		dgBodyCollisionList::dgTreeNode* node = world->dgBodyCollisionList::Find (dgUnsigned32 (signature));

		if (node) {
			collision = node->GetInfo();
			collision->AddRef();

		} else {

			dgCollisionID primitiveType = dgCollisionID(primitive);

			dgMemoryAllocator* const allocator = world->GetAllocator();
			switch (primitiveType)
			{
				case m_heightField:
				{
					collision = new (allocator) dgCollisionHeightField (world, deserialization, userData);
					break;
				}

				case m_boundingBoxHierachy:
				{
					collision = new (allocator) dgCollisionBVH (world, deserialization, userData);
					break;
				}

				case m_compoundCollision:
				{
					collision = new (allocator) dgCollisionCompound (world, deserialization, userData);
					break;
				}

				case m_sceneCollision:
				{
					collision = new (allocator) dgCollisionScene (world, deserialization, userData);
					break;
				}


				case m_sphereCollision:
				{
					collision = new (allocator) dgCollisionSphere (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_boxCollision:
				{
					collision = new (allocator) dgCollisionBox (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_coneCollision:
				{
					collision = new (allocator) dgCollisionCone (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_capsuleCollision:
				{
					collision = new (allocator) dgCollisionCapsule (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_taperedCapsuleCollision:
				{
					collision = new (allocator) dgCollisionTaperedCapsule (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_cylinderCollision:
				{
					collision = new (allocator) dgCollisionCylinder (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_chamferCylinderCollision:
				{
					collision = new (allocator) dgCollisionChamferCylinder (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_taperedCylinderCollision:
				{
					collision = new (allocator) dgCollisionTaperedCylinder (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_convexHullCollision:
				{
					collision = new (allocator) dgCollisionConvexHull (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_nullCollision:
				{
					collision = new (allocator) dgCollisionNull (world, deserialization, userData);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}


	/*
				case m_deformableMesh:
				{
					dgAssert (0);
					return NULL;
				}



				case m_compoundBreakable:
				{
					collision = new (allocator) dgCollisionCompoundBreakable (this, deserialization, userData);
					break;
				}
	*/

				default:
				dgAssert (0);
			}
		}
		m_childShape = collision;
	}
	dgDeserializeMarker (deserialization, userData);
}

dgCollisionInstance::~dgCollisionInstance()
{
	if (m_world->m_onCollisionInstanceDestruction) {
		m_world->m_onCollisionInstanceDestruction (m_world, this);
	}
	((dgWorld*) m_world)->ReleaseCollision(m_childShape);
}


void dgCollisionInstance::Serialize(dgSerialize callback, void* const userData, bool saveShape) const
{
	dgInt32 save = saveShape ? 1 : 0;
	dgInt32 primitiveType = m_childShape->GetCollisionPrimityType();
	dgInt32 signature = m_childShape->GetSignature();

	callback (userData, &m_globalMatrix, sizeof (m_globalMatrix));
	callback (userData, &m_localMatrix, sizeof (m_localMatrix));
	callback (userData, &m_scale, sizeof (m_scale));
	callback (userData, &m_invScale, sizeof (m_invScale));
	callback (userData, &m_maxScale, sizeof (m_maxScale));
	callback (userData, &m_userDataID, sizeof (m_userDataID));
	callback (userData, &m_flags, sizeof (m_flags));
	callback (userData, &primitiveType, sizeof (primitiveType));
	callback (userData, &signature, sizeof (signature));
	callback (userData, &save, sizeof (save));
	if (saveShape) {
		m_childShape->Serialize(callback, userData);
	}
	dgSerializeMarker(callback, userData);
}


void dgCollisionInstance::SetScale (const dgVector& scale)
{
	dgFloat32 scaleX = dgAbsf (scale.m_x);
	dgFloat32 scaleY = dgAbsf (scale.m_y);
	dgFloat32 scaleZ = dgAbsf (scale.m_z);
	dgAssert (scaleX > dgFloat32 (0.0f));
	dgAssert (scaleY > dgFloat32 (0.0f));
	dgAssert (scaleZ > dgFloat32 (0.0f));

	if ((dgAbsf (scaleX - scaleY) < dgFloat32 (1.0e-3f)) && (dgAbsf (scaleX - scaleZ) < dgFloat32 (1.0e-3f))) {
		m_scaleIsUniform = 1;
		if ((dgAbsf (scaleX - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f))) {
			m_scaleIsUnit = 1;
			m_scale	= dgVector (dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f));	
			m_maxScale = m_scale;	
			m_invScale = m_scale;
		} else {
			m_scaleIsUnit = 0;
			m_scale	= dgVector (scaleX, scaleX, scaleX, dgFloat32 (0.0f));	
			m_maxScale = m_scale;	
			m_invScale = dgVector (dgFloat32 (1.0f) / scaleX, dgFloat32 (1.0f) / scaleX, dgFloat32 (1.0f) / scaleX, dgFloat32 (0.0f));	
		}
	} else {
		m_scaleIsUnit = 0;
		m_scaleIsUniform = 0;
		m_maxScale = dgMax(scaleX, scaleY, scaleZ);
		m_scale	= dgVector (scaleX, scaleY, scaleZ, dgFloat32 (0.0f));	
		m_invScale = dgVector (dgFloat32 (1.0f) / scaleX, dgFloat32 (1.0f) / scaleY, dgFloat32 (1.0f) / scaleZ, dgFloat32 (0.0f));	
	}
}


void dgCollisionInstance::SetLocalMatrix (const dgMatrix& matrix)
{
	m_localMatrix = matrix;
	m_localMatrix[0][3] = dgFloat32 (0.0f);
	m_localMatrix[1][3] = dgFloat32 (0.0f);
	m_localMatrix[2][3] = dgFloat32 (0.0f);
	m_localMatrix[3][3] = dgFloat32 (1.0f);

#ifdef _DEBUG
	dgFloat32 det = (m_localMatrix.m_front * m_localMatrix.m_up) % m_localMatrix.m_right;
	dgAssert (det > dgFloat32 (0.999f));
	dgAssert (det < dgFloat32 (1.001f));
#endif
}


void dgCollisionInstance::DebugCollision (const dgMatrix& matrix, OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgMatrix scaledMatrix (m_localMatrix * matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale3 (m_scale[0]);
	scaledMatrix[1] = scaledMatrix[1].Scale3 (m_scale[1]);
	scaledMatrix[2] = scaledMatrix[2].Scale3 (m_scale[2]);
	m_childShape->DebugCollision (scaledMatrix, callback, userData);
}


dgMatrix dgCollisionInstance::CalculateInertia () const
{
	if (IsType(dgCollision::dgCollisionMesh_RTTI)) {
		return dgGetZeroMatrix();
	} else {
		return m_childShape->CalculateInertiaAndCenterOfMass (m_scale, m_localMatrix);
	}
}


dgInt32 dgCollisionInstance::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const
{
	dgInt32 count = 0;
	dgVector normal1 (normal);
	dgVector point1 (m_invScale.CompProduct4(point));
	if (!m_scaleIsUniform) {
		normal1 = m_scale.CompProduct3(normal1);
		normal1 = normal1.Scale3 (dgRsqrt (normal1 % normal1));
	}

	count = m_childShape->CalculatePlaneIntersection (normal1, point1, contactsOut);
	for (dgInt32 i = 0; i < count; i ++) {
		contactsOut[i] = m_scale.CompProduct4(contactsOut[i]);
	}
	return count;
}


void dgCollisionInstance::CalcAABB (const dgMatrix &matrix, dgVector& p0, dgVector& p1) const
{
	m_childShape->CalcAABB (matrix, p0, p1);
	p0 = (matrix.m_posit + (p0 - matrix.m_posit).CompProduct4(m_maxScale) - m_padding) & dgVector::m_triplexMask;
	p1 = (matrix.m_posit + (p1 - matrix.m_posit).CompProduct4(m_maxScale) + m_padding) & dgVector::m_triplexMask;
}


dgFloat32 dgCollisionInstance::RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const body, void* const userData) const
{
	if (!preFilter || preFilter(body, this, userData)) {
		if (m_scaleIsUnit) {
			dgFloat32 t = m_childShape->RayCast (localP0, localP1, contactOut, body, userData);
			if (t <=dgFloat32 (1.0f)) {
				if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
					contactOut.m_shapeId0 = GetUserDataID();
					contactOut.m_shapeId1 = GetUserDataID();
				}
				contactOut.m_collision0 = this;
				contactOut.m_collision1 = this;
			}
			return t;
		} else {
			dgVector p0 (localP0.CompProduct4(m_invScale));
			dgVector p1 (localP1.CompProduct4(m_invScale));
			dgFloat32 t = m_childShape->RayCast (p0, p1, contactOut, body, userData);
			if (t <=dgFloat32 (1.0f)) {
				if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
//					contactOut.m_userId = GetUserDataID();
					contactOut.m_shapeId0 = GetUserDataID();
					contactOut.m_shapeId1 = GetUserDataID();
					dgVector n (m_scale.CompProduct3 (contactOut.m_normal));
					contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
				}
				contactOut.m_collision0 = this;
				contactOut.m_collision1 = this;
			}
			return t;
		}
	}

	return dgFloat32 (1.2f);
}

dgFloat32 dgCollisionInstance::ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& convexShapeMatrix, const dgVector& localVeloc, dgFloat32 minT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const referenceBody, void* const userData, dgInt32 threadId) const
{
	if (!preFilter || preFilter(referenceBody, this, userData)) {
		if (m_scaleIsUnit) {
			dgFloat32 t = m_childShape->ConvexRayCast (convexShape, convexShapeMatrix, localVeloc, minT, contactOut, referenceBody, this, userData, threadId);
			if (t <= minT) {
				if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
					contactOut.m_shapeId0 = GetUserDataID();
					contactOut.m_shapeId1 = GetUserDataID();
				}
				contactOut.m_collision0 = this;
				contactOut.m_collision1 = this;
			}
			return t;
		} else {
			dgAssert (0);
		}
	}

	return dgFloat32 (1.2f);
}
