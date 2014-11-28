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
#include "dgCollisionCompoundFractured.h"
#include "dgCollisionDeformableSolidMesh.h"
#include "dgCollisionDeformableClothPatch.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


dgVector dgCollisionInstance::m_padding (DG_MAX_COLLISION_AABB_PADDING, DG_MAX_COLLISION_AABB_PADDING, DG_MAX_COLLISION_AABB_PADDING, dgFloat32 (0.0f));

dgCollisionInstance::dgCollisionInstance()
	:m_globalMatrix(dgGetIdentityMatrix())
	,m_localMatrix (dgGetIdentityMatrix())
	,m_aligmentMatrix (dgGetIdentityMatrix())
	,m_scale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_invScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_maxScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_userDataID(0)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(NULL)
	,m_childShape (NULL)
	,m_subCollisionHandle(NULL)
	,m_parent(NULL)
	,m_collisionMode(1)
	,m_scaleType(m_unit)
{
}


dgCollisionInstance::dgCollisionInstance(const dgWorld* const world, const dgCollision* const childCollision, dgInt32 shapeID, const dgMatrix& matrix)
	:m_globalMatrix(matrix)
	,m_localMatrix (matrix)
	,m_aligmentMatrix (dgGetIdentityMatrix())
	,m_scale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_invScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_maxScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_userDataID(shapeID)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(world)
	,m_childShape (childCollision)
	,m_subCollisionHandle(NULL)
	,m_parent(NULL)
	,m_collisionMode(1)
	,m_scaleType(m_unit)
{
	m_childShape->AddRef();
}

dgCollisionInstance::dgCollisionInstance(const dgCollisionInstance& instance)
	:m_globalMatrix(instance.m_globalMatrix)
	,m_localMatrix (instance.m_localMatrix)
	,m_aligmentMatrix (instance.m_aligmentMatrix)
	,m_scale(instance.m_scale)
	,m_invScale(instance.m_invScale)
	,m_maxScale(instance.m_maxScale)
	,m_userDataID(instance.m_userDataID)
	,m_refCount(1)
	,m_userData(instance.m_userData)
	,m_world(instance.m_world)
	,m_childShape (instance.m_childShape)
	,m_subCollisionHandle(NULL)
	,m_parent(NULL)
	,m_collisionMode(instance.m_collisionMode)
	,m_scaleType(instance.m_scaleType)
{
	if (m_childShape->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		if (m_childShape->IsType (dgCollision::dgCollisionCompoundBreakable_RTTI)) {
			dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*) m_childShape;
			m_childShape = new (m_world->GetAllocator()) dgCollisionCompoundFractured (*compound, this);
		} else if (m_childShape->IsType (dgCollision::dgCollisionScene_RTTI)) {
			dgCollisionScene* const scene = (dgCollisionScene*) m_childShape;
			m_childShape = new (m_world->GetAllocator()) dgCollisionScene (*scene, this);
		} else {
			dgCollisionCompound *const compound = (dgCollisionCompound*) m_childShape;
			m_childShape = new (m_world->GetAllocator()) dgCollisionCompound (*compound, this);
		}
	} else if (m_childShape->IsType (dgCollision::dgCollisionDeformableClothPatch_RTTI)) {
		dgCollisionDeformableClothPatch* const deformable = (dgCollisionDeformableClothPatch*) m_childShape;
		m_childShape = new (m_world->GetAllocator()) dgCollisionDeformableClothPatch (*deformable);
	} else if (m_childShape->IsType (dgCollision::dgCollisionDeformableSolidMesh_RTTI)) {
		dgCollisionDeformableSolidMesh* const deformable = (dgCollisionDeformableSolidMesh*) m_childShape;
		m_childShape = new (m_world->GetAllocator()) dgCollisionDeformableSolidMesh (*deformable);
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
	,m_aligmentMatrix (dgGetIdentityMatrix())
	,m_scale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_invScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_maxScale(dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f))
	,m_userDataID(0)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(constWorld)
	,m_childShape (NULL)
	,m_subCollisionHandle(NULL)
	,m_parent(NULL)
	,m_collisionMode(1)
	,m_scaleType(m_unit)
{
	dgInt32 saved;
	dgInt32 signature;
	dgInt32 primitive;
	dgInt32 scaleType;
	
	deserialization (userData, &m_globalMatrix, sizeof (m_globalMatrix));
	deserialization (userData, &m_localMatrix, sizeof (m_localMatrix));
	deserialization (userData, &m_aligmentMatrix, sizeof (m_aligmentMatrix));
	deserialization (userData, &m_scale, sizeof (m_scale));
	deserialization (userData, &m_invScale, sizeof (m_invScale));
	deserialization (userData, &m_maxScale, sizeof (m_maxScale));
	deserialization (userData, &m_userDataID, sizeof (m_userDataID));
	deserialization (userData, &m_collisionMode, sizeof (m_collisionMode));
	deserialization (userData, &scaleType, sizeof (scaleType));
	deserialization (userData, &primitive, sizeof (primitive));
	deserialization (userData, &signature, sizeof (signature));
	deserialization (userData, &saved, sizeof (saved));

	m_scaleType = dgScaleType(scaleType);


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
					collision = new (allocator) dgCollisionCompound (world, deserialization, userData, this);
					break;
				}

				case m_compoundFracturedCollision:
				{
					collision = new (allocator) dgCollisionCompoundFractured (world, deserialization, userData, this);
					break;
				}

				case m_sceneCollision:
				{
					collision = new (allocator) dgCollisionScene (world, deserialization, userData, this);
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

//				case m_deformableMesh:
//				{
//					dgAssert (0);
//					return NULL;
//				}

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
	dgInt32 scaleType = m_scaleType;

	callback (userData, &m_globalMatrix, sizeof (m_globalMatrix));
	callback (userData, &m_localMatrix, sizeof (m_localMatrix));
	callback (userData, &m_aligmentMatrix, sizeof (m_aligmentMatrix));
	callback (userData, &m_scale, sizeof (m_scale));
	callback (userData, &m_invScale, sizeof (m_invScale));
	callback (userData, &m_maxScale, sizeof (m_maxScale));
	callback (userData, &m_userDataID, sizeof (m_userDataID));
	callback (userData, &m_collisionMode, sizeof (m_collisionMode));
	callback (userData, &scaleType, sizeof (scaleType));
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

	if (IsType(dgCollision::dgCollisionCompound_RTTI)) {
		dgAssert (m_scaleType == m_unit);
		dgCollisionCompound* const compound = (dgCollisionCompound*) m_childShape;
		compound->ApplyScale(scale);
	} else if ((dgAbsf (scaleX - scaleY) < dgFloat32 (1.0e-4f)) && (dgAbsf (scaleX - scaleZ) < dgFloat32 (1.0e-4f))) {
		if ((dgAbsf (scaleX - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f))) {
			m_scaleType = m_unit;
			m_scale	= dgVector (dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (0.0f));	
			m_maxScale = m_scale;	
			m_invScale = m_scale;
		} else {
			m_scaleType = m_uniform;
			m_scale	= dgVector (scaleX, scaleX, scaleX, dgFloat32 (0.0f));	
			m_maxScale = m_scale;	
			m_invScale = dgVector (dgFloat32 (1.0f) / scaleX, dgFloat32 (1.0f) / scaleX, dgFloat32 (1.0f) / scaleX, dgFloat32 (0.0f));	
		}
	} else {
		m_scaleType = m_nonUniform;
		m_maxScale = dgMax(scaleX, scaleY, scaleZ);
		m_scale	= dgVector (scaleX, scaleY, scaleZ, dgFloat32 (0.0f));	
		m_invScale = dgVector (dgFloat32 (1.0f) / scaleX, dgFloat32 (1.0f) / scaleY, dgFloat32 (1.0f) / scaleZ, dgFloat32 (0.0f));	
	}
}

void dgCollisionInstance::SetGlobalScale (const dgVector& scale)
{
	// calculate current matrix
	dgMatrix matrix(dgGetIdentityMatrix());
	matrix[0][0] = m_scale.m_x;
	matrix[1][1] = m_scale.m_y;
	matrix[2][2] = m_scale.m_z;
	matrix = m_aligmentMatrix * matrix * m_localMatrix;

	// extract the original local matrix
	dgMatrix transpose (matrix.Transpose());
	dgVector globalScale (dgSqrt (transpose[0] % transpose[0]), dgSqrt (transpose[1] % transpose[1]), dgSqrt (transpose[2] % transpose[2]), dgFloat32 (1.0f));
	dgVector invGlobalScale (dgFloat32 (1.0f) / globalScale.m_x, dgFloat32 (1.0f) / globalScale.m_y, dgFloat32 (1.0f) / globalScale.m_z, dgFloat32 (1.0f));
	dgMatrix localMatrix (m_aligmentMatrix.Transpose() * m_localMatrix);
	localMatrix.m_posit = matrix.m_posit.CompProduct4(invGlobalScale) | dgVector::m_wOne;

	if ((dgAbsf (scale[0] - scale[1]) < dgFloat32 (1.0e-4f)) && (dgAbsf (scale[0] - scale[2]) < dgFloat32 (1.0e-4f))) {
		m_localMatrix = localMatrix;
		m_localMatrix.m_posit = m_localMatrix.m_posit.CompProduct4(scale) | dgVector::m_wOne;
		m_aligmentMatrix = dgGetIdentityMatrix();
		SetScale (scale);
	} else {
		
		// create a new scale matrix 
		localMatrix[0] = localMatrix[0].CompProduct4 (scale);
		localMatrix[1] = localMatrix[1].CompProduct4 (scale);
		localMatrix[2] = localMatrix[2].CompProduct4 (scale);
		localMatrix[3] = localMatrix[3].CompProduct4 (scale);
		localMatrix[3][3] = dgFloat32 (1.0f);

		// decompose into to align * scale * local
		localMatrix.PolarDecomposition (m_localMatrix, m_scale, m_aligmentMatrix);

		m_localMatrix = m_aligmentMatrix * m_localMatrix;
		m_aligmentMatrix = m_aligmentMatrix.Transpose();

		dgAssert (m_localMatrix.TestOrthogonal());
		dgAssert (m_aligmentMatrix.TestOrthogonal());

//dgMatrix xxx1 (dgGetIdentityMatrix());
//xxx1[0][0] = m_scale.m_x;
//xxx1[1][1] = m_scale.m_y;
//xxx1[2][2] = m_scale.m_z;
//dgMatrix xxx (m_aligmentMatrix * xxx1 * m_localMatrix);

		bool isIdentity = true;
		for (dgInt32 i = 0; i < 3; i ++) {
			isIdentity &= dgAbsf (m_aligmentMatrix[i][i] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f);
			isIdentity &= dgAbsf (m_aligmentMatrix[3][i]) < dgFloat32 (1.0e-5f);
		}
		m_scaleType = isIdentity ? m_nonUniform : m_global;

		m_maxScale = dgMax(m_scale[0], m_scale[1], m_scale[2]);
		m_invScale = dgVector (dgFloat32 (1.0f) / m_scale[0], dgFloat32 (1.0f) / m_scale[1], dgFloat32 (1.0f) / m_scale[2], dgFloat32 (0.0f));	
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


void dgCollisionInstance::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgMatrix scaledMatrix (m_localMatrix * matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale3 (m_scale[0]);
	scaledMatrix[1] = scaledMatrix[1].Scale3 (m_scale[1]);
	scaledMatrix[2] = scaledMatrix[2].Scale3 (m_scale[2]);
	m_childShape->DebugCollision (m_aligmentMatrix * scaledMatrix, callback, userData);
}


dgMatrix dgCollisionInstance::CalculateInertia () const
{
	if (IsType(dgCollision::dgCollisionMesh_RTTI)) {
		return dgGetZeroMatrix();
	} else {
		return m_childShape->CalculateInertiaAndCenterOfMass (m_aligmentMatrix, m_scale, m_localMatrix);
//		switch (m_scaleType)
//		{
//			case m_unit:
//			case m_uniform:
//			case m_nonUniform:
//			{
//				return m_childShape->CalculateInertiaAndCenterOfMass (m_aligmentMatrix____, m_scale, m_localMatrix);
//			}
//
//			default:
//			{
//				return m_childShape->CalculateInertiaAndCenterOfMass (m_aligmentMatrix____, m_scale, m_localMatrix);
//			}
//		}
	}
}


dgInt32 dgCollisionInstance::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgInt32 count = 0;
	dgAssert(normal.m_w == dgFloat32 (0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			count = m_childShape->CalculatePlaneIntersection (normal, point, contactsOut, normalSign);
			break;
		}
		case m_uniform:
		{
			dgVector point1 (m_invScale.CompProduct4(point));
			count = m_childShape->CalculatePlaneIntersection (normal, point1, contactsOut, normalSign);
			for (dgInt32 i = 0; i < count; i ++) {
				contactsOut[i] = m_scale.CompProduct4(contactsOut[i]);
			}
			break;
		}

		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			dgVector point1 (m_invScale.CompProduct4(point));
			dgVector normal1 (m_scale.CompProduct4(normal));
			normal1 = normal1.CompProduct4(normal1.InvMagSqrt());
			count = m_childShape->CalculatePlaneIntersection (normal1, point1, contactsOut, normalSign);
			for (dgInt32 i = 0; i < count; i ++) {
				contactsOut[i] = m_scale.CompProduct4(contactsOut[i]);
			}
			break;
		}

		case m_global:
		default:
		{
			dgVector point1 (m_aligmentMatrix.UntransformVector (m_invScale.CompProduct4(point)));
			dgVector normal1 (m_aligmentMatrix.UntransformVector (m_scale.CompProduct4(normal)));
			normal1 = normal1.CompProduct4(normal1.InvMagSqrt());
			count = m_childShape->CalculatePlaneIntersection (normal1, point1, contactsOut, normalSign);
			for (dgInt32 i = 0; i < count; i ++) {
				contactsOut[i] = m_scale.CompProduct4(m_aligmentMatrix.TransformVector(contactsOut[i]));
			}
		}
	}
	return count;
}


void dgCollisionInstance::CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
//	m_childShape->CalcAABB (matrix, p0, p1);
//	p0 = (matrix.m_posit + (p0 - matrix.m_posit).CompProduct4(m_maxScale) - m_padding) & dgVector::m_triplexMask;
//	p1 = (matrix.m_posit + (p1 - matrix.m_posit).CompProduct4(m_maxScale) + m_padding) & dgVector::m_triplexMask;

	switch (m_scaleType)
	{
		case m_unit:
		{
			m_childShape->CalcAABB (matrix, p0, p1);
			p0 -= m_padding;
			p1 += m_padding;
			break;
		}

		case m_uniform:
		case m_nonUniform:
		{
			dgMatrix matrix1 (matrix);
			matrix1[0] = matrix1[0].Scale4(m_scale.m_x);
			matrix1[1] = matrix1[1].Scale4(m_scale.m_y);
			matrix1[2] = matrix1[2].Scale4(m_scale.m_z);
			m_childShape->CalcAABB (matrix1, p0, p1);
			p0 -= m_padding;
			p1 += m_padding;
			break;
		}

		case m_global:
		default:
		{
			dgMatrix matrix1 (matrix);
			matrix1[0] = matrix1[0].Scale4(m_scale.m_x);
			matrix1[1] = matrix1[1].Scale4(m_scale.m_y);
			matrix1[2] = matrix1[2].Scale4(m_scale.m_z);
			m_childShape->CalcAABB (m_aligmentMatrix * matrix1, p0, p1);
			p0 -= m_padding;
			p1 += m_padding;
			break;
		}
	}

	dgAssert (p0.m_w == dgFloat32 (0.0f));
	dgAssert (p1.m_w == dgFloat32 (0.0f));
}


dgFloat32 dgCollisionInstance::RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const body, void* const userData) const
{
	if (!preFilter || preFilter(body, this, userData)) {
		switch(m_scaleType)
		{
			case m_unit:
			{
				dgFloat32 t = m_childShape->RayCast (localP0, localP1, maxT, contactOut, body, userData, preFilter);
				if (t <= maxT) {
					if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
						contactOut.m_shapeId0 = GetUserDataID();
						contactOut.m_shapeId1 = GetUserDataID();
					}
					if (!m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI)) {
						contactOut.m_collision0 = this;
						contactOut.m_collision1 = this;
					}
				}
				return t;
			}

			case m_uniform:
			{
				dgVector p0 (localP0.CompProduct4(m_invScale));
				dgVector p1 (localP1.CompProduct4(m_invScale));
				dgFloat32 t = m_childShape->RayCast (p0, p1, maxT, contactOut, body, userData, preFilter);
				if (t <= maxT) {
					if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
						contactOut.m_shapeId0 = GetUserDataID();
						contactOut.m_shapeId1 = GetUserDataID();
					}
					if (!m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI)) {
						contactOut.m_collision0 = this;
						contactOut.m_collision1 = this;
					}
				}
				return t;
			}

			case m_nonUniform:
			{
				dgVector p0 (localP0.CompProduct4(m_invScale));
				dgVector p1 (localP1.CompProduct4(m_invScale));
				dgFloat32 t = m_childShape->RayCast (p0, p1, maxT, contactOut, body, userData, preFilter);
				if (t <= maxT) {
					if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
						contactOut.m_shapeId0 = GetUserDataID();
						contactOut.m_shapeId1 = GetUserDataID();
						dgVector n (m_invScale.CompProduct4 (contactOut.m_normal));
						contactOut.m_normal = n.CompProduct4(n.InvMagSqrt());
					}
					if (!m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI)) {
						contactOut.m_collision0 = this;
						contactOut.m_collision1 = this;
					}
				}
				return t;
			}

			case m_global:
			default:
			{
				dgVector p0 (m_aligmentMatrix.UntransformVector (localP0.CompProduct4(m_invScale)));
				dgVector p1 (m_aligmentMatrix.UntransformVector (localP1.CompProduct4(m_invScale)));
				dgFloat32 t = m_childShape->RayCast (p0, p1, maxT, contactOut, body, userData, preFilter);
				if (t <= maxT) {
					if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
						contactOut.m_shapeId0 = GetUserDataID();
						contactOut.m_shapeId1 = GetUserDataID();
						dgVector n (m_aligmentMatrix.RotateVector(m_invScale.CompProduct4 (contactOut.m_normal)));
						contactOut.m_normal = n.CompProduct4(n.InvMagSqrt());
					}
					if (!(m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
						contactOut.m_collision0 = this;
						contactOut.m_collision1 = this;
					}
				}
				return t;
			}
		}
	}
	return dgFloat32 (1.2f);

}

dgFloat32 dgCollisionInstance::ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& convexShapeMatrix, const dgVector& localVeloc, dgFloat32 minT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const referenceBody, void* const userData, dgInt32 threadId) const
{
	dgFloat32 t = dgFloat32 (1.2f);
	if ((GetCollisionPrimityType() != m_nullCollision) && (!preFilter || preFilter(referenceBody, this, userData))) {
		t = m_childShape->ConvexRayCast (convexShape, convexShapeMatrix, localVeloc, minT, contactOut, referenceBody, this, userData, threadId);
		if (t <= minT) {
			if (!(m_childShape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_childShape->IsType(dgCollision::dgCollisionCompound_RTTI))) {
				contactOut.m_shapeId0 = GetUserDataID();
				//contactOut.m_shapeId1 = GetUserDataID();
				contactOut.m_shapeId1 = convexShape->GetUserDataID();
			}
			contactOut.m_collision0 = this;
			//contactOut.m_collision1 = this;
			contactOut.m_collision1 = convexShape;
		}
	}
	return t;
}

void dgCollisionInstance::CalculateBuoyancyAcceleration (const dgMatrix& matrix, const dgVector& origin, const dgVector& gravity, const dgVector& fluidPlane, dgFloat32 fluidDensity, dgFloat32 fluidViscosity, dgVector& accel, dgVector& alpha)
{
	dgMatrix globalMatrix (m_localMatrix * matrix);

	accel = dgVector (dgFloat32 (0.0f));
	alpha = dgVector (dgFloat32 (0.0f));
	dgVector volumeIntegral (m_childShape->CalculateVolumeIntegral (globalMatrix, fluidPlane, *this));
	if (volumeIntegral.m_w > dgFloat32 (0.0f)) {
		dgVector buoyanceCenter (volumeIntegral - origin);

		dgVector force (gravity.Scale3 (-fluidDensity * volumeIntegral.m_w));
		dgVector torque (buoyanceCenter * force);

		accel += force;
		alpha += torque;
	}
}