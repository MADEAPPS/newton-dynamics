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
#include "ndRayCastNotify.h"

#if 0
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
#include "ndShapeInstance.h"
#include "dgCollisionCompound.h"
#include "dgCollisionHeightField.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionChamferCylinder.h"
#include "dgCollisionCompoundFractured.h"
#include "dgCollisionDeformableSolidMesh.h"
#include "dgCollisionMassSpringDamperSystem.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
ndShapeInstance::ndShapeInstance(const dgWorld* const world, const dgCollision* const childCollision, dInt32 shapeID, const dMatrix& matrix)
	:m_globalMatrix(matrix)
	,m_localMatrix (matrix)
	,m_aligmentMatrix (dGetIdentityMatrix())
	,m_scale(dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f))
	,m_invScale(dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f))
	,m_maxScale(dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f))
	,m_material()
	,m_world(world)
	,m_shape (childCollision)
	,m_subCollisionHandle(nullptr)
	,m_parent(nullptr)
	,m_skinThickness(dFloat32 (0.0f))
	,m_collisionMode(1)
	,m_refCount(1)
	,m_scaleType(m_unit)
	,m_isExternal(true)
{
	m_material.m_userId = shapeID;
	m_shape->AddRef();
}

ndShapeInstance::ndShapeInstance(const dgWorld* const constWorld, dgDeserialize serialize, void* const userData, dInt32 revisionNumber)
	:m_globalMatrix(dGetIdentityMatrix())
	,m_localMatrix (dGetIdentityMatrix())
	,m_aligmentMatrix (dGetIdentityMatrix())
	,m_scale(dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f))
	,m_invScale(dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f))
	,m_maxScale(dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f))
	,m_material()
	,m_world(constWorld)
	,m_shape (nullptr)
	,m_subCollisionHandle(nullptr)
	,m_parent(nullptr)
	,m_skinThickness(dFloat32 (0.0f))
	,m_collisionMode(1)
	,m_refCount(1)
	,m_scaleType(m_unit)
	,m_isExternal(true)
{
	dInt32 saved;
	dInt32 signature;
	dInt32 primitive;
	dInt32 scaleType;
	
	serialize (userData, &m_globalMatrix, sizeof (m_globalMatrix));
	serialize (userData, &m_localMatrix, sizeof (m_localMatrix));
	serialize (userData, &m_aligmentMatrix, sizeof (m_aligmentMatrix));
	serialize (userData, &m_scale, sizeof (m_scale));
	serialize (userData, &m_invScale, sizeof (m_invScale));
	serialize (userData, &m_maxScale, sizeof (m_maxScale));
	serialize (userData, &m_skinThickness, sizeof (m_skinThickness));
	serialize (userData, &m_material, sizeof (m_material));
	serialize (userData, &m_collisionMode, sizeof (m_collisionMode));
	serialize (userData, &scaleType, sizeof (scaleType));
	serialize (userData, &primitive, sizeof (primitive));
	serialize (userData, &signature, sizeof (signature));
	serialize (userData, &saved, sizeof (saved));

	m_scaleType = dgScaleType(scaleType);

	dgWorld* const world = (dgWorld*) constWorld;
	if (saved) {
		const dgCollision* collision = nullptr;
		dgBodyCollisionList::dTreeNode* node = world->dgBodyCollisionList::Find (dUnsigned32 (signature));

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
					collision = new (allocator) dgCollisionHeightField (world, serialize, userData, revisionNumber);
					break;
				}

				case m_boundingBoxHierachy:
				{
					collision = new (allocator) dgCollisionBVH (world, serialize, userData, revisionNumber);
					break;
				}

				case m_compoundCollision:
				{
					collision = new (allocator) dgCollisionCompound (world, serialize, userData, this, revisionNumber);
					break;
				}

				case m_compoundFracturedCollision:
				{
					collision = new (allocator) dgCollisionCompoundFractured (world, serialize, userData, this, revisionNumber);
					break;
				}

				case m_sceneCollision:
				{
					collision = new (allocator) dgCollisionScene (world, serialize, userData, this, revisionNumber);
					break;
				}


				case m_sphereCollision:
				{
					collision = new (allocator) dgCollisionSphere (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_boxCollision:
				{
					collision = new (allocator) dgCollisionBox (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_coneCollision:
				{
					collision = new (allocator) dgCollisionCone (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_capsuleCollision:
				{
					collision = new (allocator) dgCollisionCapsule (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_cylinderCollision:
				{
					collision = new (allocator) dgCollisionCylinder (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_chamferCylinderCollision:
				{
					collision = new (allocator) dgCollisionChamferCylinder (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_convexHullCollision:
				{
					collision = new (allocator) dgCollisionConvexHull (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

				case m_nullCollision:
				{
					collision = new (allocator) dgCollisionNull (world, serialize, userData, revisionNumber);
					node = world->dgBodyCollisionList::Insert (collision, collision->GetSignature());
					collision->AddRef();
					break;
				}

//				case m_deformableMesh:
//				{
//					dAssert (0);
//					return nullptr;
//				}

				default:
				dAssert (0);
			}
		}
		m_shape = collision;
	}
	dgDeserializeMarker (serialize, userData);
}

ndShapeInstance::~ndShapeInstance()
{
	if (m_world->m_onCollisionInstanceDestruction && m_isExternal) {
		m_world->m_onCollisionInstanceDestruction (m_world, this);
	}
	dgWorld* const world = (dgWorld*)m_world;
	world->ReleaseCollision(m_shape);
}

void ndShapeInstance::Serialize(dgSerialize serialize, void* const userData, bool saveShape) const
{
	dInt32 save = saveShape ? 1 : 0;
	dInt32 primitiveType = m_shape->GetCollisionPrimityType();
	dInt32 signature = m_shape->GetSignature();
	dInt32 scaleType = m_scaleType;

	serialize (userData, &m_globalMatrix, sizeof (m_globalMatrix));
	serialize (userData, &m_localMatrix, sizeof (m_localMatrix));
	serialize (userData, &m_aligmentMatrix, sizeof (m_aligmentMatrix));
	serialize (userData, &m_scale, sizeof (m_scale));
	serialize (userData, &m_invScale, sizeof (m_invScale));
	serialize (userData, &m_maxScale, sizeof (m_maxScale));
	serialize (userData, &m_skinThickness, sizeof (m_skinThickness));
	serialize (userData, &m_material, sizeof (m_material));
	serialize (userData, &m_collisionMode, sizeof (m_collisionMode));
	serialize (userData, &scaleType, sizeof (scaleType));
	serialize (userData, &primitiveType, sizeof (primitiveType));
	serialize (userData, &signature, sizeof (signature));
	serialize (userData, &save, sizeof (save));
	if (saveShape) {
		m_shape->Serialize(serialize, userData);
	}
	dgSerializeMarker(serialize, userData);
}



void ndShapeInstance::SetGlobalScale (const dVector& scale)
{
	// calculate current matrix
	dMatrix matrix(dGetIdentityMatrix());
	matrix[0][0] = m_scale.m_x;
	matrix[1][1] = m_scale.m_y;
	matrix[2][2] = m_scale.m_z;
	matrix = m_aligmentMatrix * matrix * m_localMatrix;

	// extract the original local matrix
	dMatrix transpose (matrix.Transpose());
	dVector globalScale (dgSqrt (transpose[0].DotProduct(transpose[0]).GetScalar()), dgSqrt (transpose[1].DotProduct(transpose[1]).GetScalar()), dgSqrt (transpose[2].DotProduct(transpose[2]).GetScalar()), dFloat32 (1.0f));
	dVector invGlobalScale (dFloat32 (1.0f) / globalScale.m_x, dFloat32 (1.0f) / globalScale.m_y, dFloat32 (1.0f) / globalScale.m_z, dFloat32 (1.0f));
	dMatrix localMatrix (m_aligmentMatrix.Transpose() * m_localMatrix);
	localMatrix.m_posit = matrix.m_posit * invGlobalScale;
	dAssert (localMatrix.m_posit.m_w == dFloat32 (1.0f));

	if ((dAbs (scale[0] - scale[1]) < dFloat32 (1.0e-4f)) && (dAbs (scale[0] - scale[2]) < dFloat32 (1.0e-4f))) {
		m_localMatrix = localMatrix;
		m_localMatrix.m_posit = m_localMatrix.m_posit * scale | dVector::m_wOne;
		m_aligmentMatrix = dGetIdentityMatrix();
		SetScale (scale);
	} else {
		
		// create a new scale matrix 
		localMatrix[0] = localMatrix[0] * scale;
		localMatrix[1] = localMatrix[1] * scale;
		localMatrix[2] = localMatrix[2] * scale;
		localMatrix[3] = localMatrix[3] * scale;
		localMatrix[3][3] = dFloat32 (1.0f);

		// decompose into to align * scale * local
		localMatrix.PolarDecomposition (m_localMatrix, m_scale, m_aligmentMatrix);

		m_localMatrix = m_aligmentMatrix * m_localMatrix;
		m_aligmentMatrix = m_aligmentMatrix.Transpose();

		dAssert (m_localMatrix.TestOrthogonal());
		dAssert (m_aligmentMatrix.TestOrthogonal());

//dMatrix xxx1 (dGetIdentityMatrix());
//xxx1[0][0] = m_scale.m_x;
//xxx1[1][1] = m_scale.m_y;
//xxx1[2][2] = m_scale.m_z;
//dMatrix xxx (m_aligmentMatrix * xxx1 * m_localMatrix);

		bool isIdentity = true;
		for (dInt32 i = 0; i < 3; i ++) {
			isIdentity &= dAbs (m_aligmentMatrix[i][i] - dFloat32 (1.0f)) < dFloat32 (1.0e-5f);
			isIdentity &= dAbs (m_aligmentMatrix[3][i]) < dFloat32 (1.0e-5f);
		}
		m_scaleType = isIdentity ? m_nonUniform : m_global;

		m_maxScale = dMax(m_scale[0], m_scale[1], m_scale[2]);
		m_invScale = dVector (dFloat32 (1.0f) / m_scale[0], dFloat32 (1.0f) / m_scale[1], dFloat32 (1.0f) / m_scale[2], dFloat32 (0.0f));	
	}
}


void ndShapeInstance::SetLocalMatrix (const dMatrix& matrix)
{
	m_localMatrix = matrix;
	m_localMatrix[0][3] = dFloat32 (0.0f);
	m_localMatrix[1][3] = dFloat32 (0.0f);
	m_localMatrix[2][3] = dFloat32 (0.0f);
	m_localMatrix[3][3] = dFloat32 (1.0f);
	dAssert(m_localMatrix.TestOrthogonal());
}


void ndShapeInstance::CalculateImplicitContacts(dInt32 count, dgContactPoint* const contactPoints) const
{
	switch (m_scaleType)
	{
		case m_unit:
		{
		   for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_globalMatrix.UntransformVector(contactPoints[i].m_point);
			}
			m_shape->CalculateImplicitContacts(count, contactPoints);
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_globalMatrix.TransformVector(contactPoints[i].m_point);
				contactPoints[i].m_normal = m_globalMatrix.RotateVector(contactPoints[i].m_normal);
			}
			break;
		}

		case m_uniform:
		{
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_invScale * m_globalMatrix.UntransformVector(contactPoints[i].m_point);
			}
			m_shape->CalculateImplicitContacts(count, contactPoints);
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_globalMatrix.TransformVector(contactPoints[i].m_point * m_scale);
				contactPoints[i].m_normal = m_globalMatrix.RotateVector(contactPoints[i].m_normal);
			}
			break;
		}

		case m_nonUniform:
		{
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_invScale * m_globalMatrix.UntransformVector(contactPoints[i].m_point);
			}
			m_shape->CalculateImplicitContacts(count, contactPoints);
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_globalMatrix.TransformVector(contactPoints[i].m_point * m_scale);
				contactPoints[i].m_normal = m_globalMatrix.RotateVector(contactPoints[i].m_normal * m_invScale).Normalize();
			}
			break;
		}

		case m_global:
		default:
		{
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_invScale * m_globalMatrix.UntransformVector(m_aligmentMatrix.UntransformVector(contactPoints[i].m_point));
			}
			m_shape->CalculateImplicitContacts(count, contactPoints);
			for (dInt32 i = 0; i < count; i++) {
				contactPoints[i].m_point = m_globalMatrix.TransformVector(m_aligmentMatrix.TransformVector(contactPoints[i].m_point) * m_scale);
				contactPoints[i].m_normal = m_globalMatrix.RotateVector(m_aligmentMatrix.RotateVector(contactPoints[i].m_normal) * m_invScale).Normalize();
			}
		}
	}
}

#endif

dVector ndShapeInstance::m_padding(D_MAX_SHAPE_AABB_PADDING, D_MAX_SHAPE_AABB_PADDING, D_MAX_SHAPE_AABB_PADDING, dFloat32(0.0f));

ndShapeInstance::ndShapeInstance(ndShape* const shape)
	:dClassAlloc()
	,m_globalMatrix(dGetIdentityMatrix())
	,m_localMatrix(dGetIdentityMatrix())
	,m_aligmentMatrix(dGetIdentityMatrix())
	,m_scale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_invScale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_maxScale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_shape(shape->AddRef())
	,m_ownerBody(nullptr)
	//,m_subCollisionHandle(nullptr)
	,m_skinThickness(dFloat32(0.0f))
	,m_scaleType(m_unit)
	,m_collisionMode(true)
	//,m_isExternal(true)
{
}

ndShapeInstance::ndShapeInstance(const ndShapeInstance& instance)
	:dClassAlloc()
	,m_globalMatrix(instance.m_globalMatrix)
	,m_localMatrix(instance.m_localMatrix)
	,m_aligmentMatrix(instance.m_aligmentMatrix)
	,m_scale(instance.m_scale)
	,m_invScale(instance.m_invScale)
	,m_maxScale(instance.m_maxScale)
	,m_shape(instance.m_shape->AddRef())
	,m_ownerBody(instance.m_ownerBody)
	,m_skinThickness(instance.m_skinThickness)
	,m_scaleType(instance.m_scaleType)
	,m_collisionMode(instance.m_collisionMode)
{
}

ndShapeInstance::~ndShapeInstance()
{
	m_shape->Release();
}

ndShapeInstance& ndShapeInstance::operator=(const ndShapeInstance& instance)
{
	m_globalMatrix = instance.m_globalMatrix;
	m_localMatrix = instance.m_localMatrix;
	m_aligmentMatrix = instance.m_aligmentMatrix;
	m_scale = instance.m_scale;
	m_invScale = instance.m_invScale;
	m_maxScale = instance.m_maxScale;

	m_shape->Release();
	m_shape = instance.m_shape->AddRef();
	m_ownerBody = instance.m_ownerBody;
	return *this;
}

void ndShapeInstance::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	debugCallback.m_instance = this;
	m_shape->DebugShape(GetScaledTransform(matrix), debugCallback);
}

ndShapeInfo ndShapeInstance::GetShapeInfo() const
{
	ndShapeInfo info(m_shape->GetShapeInfo());
	info.m_offsetMatrix = m_localMatrix;
	//info.m_collisionMaterial = m_material;
	return info;
}

void ndShapeInstance::CalculateAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const
{
	for (dInt32 i = 0; i < 3; i++) 
	{
		dVector minSupport(matrix.TransformVector(SupportVertex(matrix[i].Scale(dFloat32(-1.0f)))));
		minP[i] = minSupport[i];
		dVector maxSupport (matrix.TransformVector(SupportVertex(matrix[i])));
		maxP[i] = maxSupport[i];
	}
	minP = minP & dVector::m_triplexMask;
	maxP = maxP & dVector::m_triplexMask;
}

dMatrix ndShapeInstance::CalculateInertia() const
{
	ndShape* const shape = (ndShape*)m_shape;
	if (shape->GetAsShapeNull() || !shape->GetAsShapeConvex()) 
	{
		return dGetZeroMatrix();
	}
	else 
	{
		return m_shape->CalculateInertiaAndCenterOfMass(m_aligmentMatrix, m_scale, m_localMatrix);
	}
}

dFloat32 ndShapeInstance::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	dFloat32 t = dFloat32(1.2f);
	if (callback.OnRayPrecastAction(body, this))
	{
		switch (m_scaleType)
		{
			case m_unit:
			{
				t = m_shape->RayCast(callback, localP0, localP1, maxT, body, contactOut);
				if (t <= maxT) 
				{
					//dAssert(((ndShape*)m_shape)->GetAsShapeBox() || ((ndShape*)m_shape)->GetAsShapeSphere());
					dAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//	}
				//	if (!m_shape->IsType(dgCollision::dgCollisionCompound_RTTI)) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_uniform:
			{
				dAssert(0);
				//dVector p0(localP0 * m_invScale);
				//dVector p1(localP1 * m_invScale);
				//dFloat32 t = m_shape->RayCast(p0, p1, maxT, contactOut, body, userData, preFilter);
				//if (t <= maxT) 
				//{
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//	}
				//	if (!m_shape->IsType(dgCollision::dgCollisionCompound_RTTI)) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}
				//}
				break;
			}

			case m_nonUniform:
			{
				dAssert(0);
				//dVector p0(localP0 * m_invScale);
				//dVector p1(localP1 * m_invScale);
				//dFloat32 t = m_shape->RayCast(p0, p1, maxT, contactOut, body, userData, preFilter);
				//if (t <= maxT) 
				//{
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//		dVector n(m_invScale * contactOut.m_normal);
				//		contactOut.m_normal = n.Normalize();
				//	}
				//	if (!m_shape->IsType(dgCollision::dgCollisionCompound_RTTI)) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}
				//}
				break;
			}

			case m_global:
			default:
			{
				dAssert(0);
				//dVector p0(m_aligmentMatrix.UntransformVector(localP0 * m_invScale));
				//dVector p1(m_aligmentMatrix.UntransformVector(localP1 * m_invScale));
				//dFloat32 t = m_shape->RayCast(p0, p1, maxT, contactOut, body, userData, preFilter);
				//if (t <= maxT) 
				//{
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//		dVector n(m_aligmentMatrix.RotateVector(m_invScale * contactOut.m_normal));
				//		contactOut.m_normal = n.Normalize();
				//	}
				//	if (!(m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}
				//}
				break;
			}
		}
	}
	return t;
}

dInt32 ndShapeInstance::CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const
{
	dInt32 count = 0;
	dAssert(normal.m_w == dFloat32(0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			count = m_shape->CalculatePlaneIntersection(normal, point, contactsOut);
			break;
		}
		case m_uniform:
		{
			dVector point1(m_invScale * point);
			count = m_shape->CalculatePlaneIntersection(normal, point1, contactsOut);
			for (dInt32 i = 0; i < count; i++) {
				contactsOut[i] = m_scale * contactsOut[i];
			}
			break;
		}

		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			dVector point1(m_invScale * point);
			dVector normal1(m_scale * normal);
			normal1 = normal1.Normalize();
			count = m_shape->CalculatePlaneIntersection(normal1, point1, contactsOut);
			for (dInt32 i = 0; i < count; i++) {
				contactsOut[i] = m_scale * contactsOut[i];
			}
			break;
		}

		case m_global:
		default:
		{
			dVector point1(m_aligmentMatrix.UntransformVector(m_invScale * point));
			dVector normal1(m_aligmentMatrix.UntransformVector(m_scale * normal));
			normal1 = normal1.Normalize();
			count = m_shape->CalculatePlaneIntersection(normal1, point1, contactsOut);
			for (dInt32 i = 0; i < count; i++) {
				contactsOut[i] = m_scale * m_aligmentMatrix.TransformVector(contactsOut[i]);
			}
		}
	}
	return count;
}

void ndShapeInstance::SetScale(const dVector& scale)
{
	dFloat32 scaleX = dAbs(scale.m_x);
	dFloat32 scaleY = dAbs(scale.m_y);
	dFloat32 scaleZ = dAbs(scale.m_z);
	dAssert(scaleX > dFloat32(0.0f));
	dAssert(scaleY > dFloat32(0.0f));
	dAssert(scaleZ > dFloat32(0.0f));

	//if (IsType(dgCollision::dgCollisionCompound_RTTI)) 
	if (((ndShape*)m_shape)->GetAsShapeCompound())
	{
		dAssert(0);
		//dAssert(m_scaleType == m_unit);
		//dgCollisionCompound* const compound = (dgCollisionCompound*)m_shape;
		//compound->ApplyScale(scale);
	}
	else if ((dAbs(scaleX - scaleY) < dFloat32(1.0e-4f)) && (dAbs(scaleX - scaleZ) < dFloat32(1.0e-4f))) 
	{
		if ((dAbs(scaleX - dFloat32(1.0f)) < dFloat32(1.0e-4f))) 
		{
			m_scaleType = m_unit;
			m_scale = dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f));
			m_maxScale = m_scale;
			m_invScale = m_scale;
		}
		else 
		{
			m_scaleType = m_uniform;
			m_scale = dVector(scaleX, scaleX, scaleX, dFloat32(0.0f));
			m_maxScale = m_scale;
			m_invScale = dVector(dFloat32(1.0f) / scaleX, dFloat32(1.0f) / scaleX, dFloat32(1.0f) / scaleX, dFloat32(0.0f));
		}
	}
	else 
	{
		m_scaleType = m_nonUniform;
		m_maxScale = dMax(scaleX, scaleY, scaleZ);
		m_scale = dVector(scaleX, scaleY, scaleZ, dFloat32(0.0f));
		m_invScale = dVector(dFloat32(1.0f) / scaleX, dFloat32(1.0f) / scaleY, dFloat32(1.0f) / scaleZ, dFloat32(0.0f));
	}
}


