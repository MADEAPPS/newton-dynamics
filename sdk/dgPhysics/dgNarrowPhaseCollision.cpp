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

#include "dgPhysicsStdafx.h"

#include "dgWorld.h"
#include "dgCollision.h"
#include "dgMeshEffect.h"
#include "dgDynamicBody.h"
#include "dgCollisionBVH.h"
#include "dgCollisionBox.h"
#include "dgContactSolver.h"
#include "dgKinematicBody.h"
#include "dgCollisionCone.h"
#include "dgCollisionNull.h"
#include "dgBodyMasterList.h"
#include "dgCollisionScene.h"
#include "dgCollisionSphere.h"
#include "dgCollisionCapsule.h"
#include "dgCollisionCylinder.h"
#include "dgCollisionCompound.h"
#include "dgCollisionUserMesh.h"
#include "dgCollisionInstance.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionConvexHull.h"
#include "dgCollisionHeightField.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableMesh.h"
#include "dgCollisionChamferCylinder.h"
#include "dgCollisionCompoundFractured.h"
#include "dgCollisionDeformableSolidMesh.h"
#include "dgCollisionMassSpringDamperSystem.h"
#include "dgCollisionIncompressibleParticles.h"


DG_MSC_VECTOR_ALIGMENT
class dgCollisionContactCloud: public dgCollisionConvex
{
	public:
	dgCollisionContactCloud(dgMemoryAllocator* const allocator, dgInt32 count, const dgContactPoint* const contacts)
		:dgCollisionConvex(allocator, 0, m_contactCloud)
		,m_contacts(contacts)
		,m_count(count)
	{
	}
	~dgCollisionContactCloud()
	{
	}

	dgInt32 CalculateSignature() const
	{
		dgAssert (0);
		return 0;
	}
	void Serialize(dgSerialize callback, void* const userData) const
	{
		dgAssert (0);
	}
	void SetCollisionBBox(const dgVector& p0, const dgVector& p1)
	{
		dgAssert (0);
	}
	dgFloat32 RayCast(const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
	{
		dgAssert (0);
		return 0;
	}

	dgVector SupportVertex(const dgVector& dir, dgInt32* const vertexIndex) const
	{
		dgInt32 index = 0;
		dgFloat32 dist = dgFloat32 (-1.0e10f);
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgFloat32 dist1 = dir.DotProduct(m_contacts[i].m_point).GetScalar();
			if (dist1 > dist) {
				index = i;
				dist = dist1;
			}
		}
		return m_contacts[index].m_point;
	}

	dgInt32 CalculatePlaneIntersection(const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const
	{
		*contactsOut = point;
		return 1;
	}

	dgFloat32 GetVolume() const
	{
		dgAssert (0);
		return 0;
	}
	dgFloat32 GetBoxMinRadius() const
	{
		dgAssert (0);
		return 0;
	}
	dgFloat32 GetBoxMaxRadius() const
	{
		dgAssert(0);
		return 0;
	}

	const dgContactPoint* m_contacts;
	dgInt32 m_count;

	static dgVector m_pruneUpDir;
	static dgVector m_pruneSupportX;

} DG_GCC_VECTOR_ALIGMENT;

dgVector dgCollisionContactCloud::m_pruneUpDir(dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f));
dgVector dgCollisionContactCloud::m_pruneSupportX(dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f));


dgCollisionInstance* dgWorld::CreateNull ()
{
	dgUnsigned32 crc = dgCollision::dgCollisionNull_RTTI;
	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* const collision = new  (m_allocator) dgCollisionNull (m_allocator, crc);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), 0, dgGetIdentityMatrix());
}

dgCollisionInstance* dgWorld::CreateSphere(dgFloat32 radii, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionSphere::CalculateSignature (radii);
	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* const collision = new  (m_allocator) dgCollisionSphere (m_allocator, crc, dgAbs(radii));
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateBox(dgFloat32 dx, dgFloat32 dy, dgFloat32 dz, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionBox::CalculateSignature(dx, dy, dz);
	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* const collision = new  (m_allocator) dgCollisionBox (m_allocator, crc, dx, dy, dz);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateCapsule (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionCapsule::CalculateSignature(dgAbs (radio0), dgAbs (radio1), dgAbs (height) * dgFloat32 (0.5f));

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* collision = new  (m_allocator) dgCollisionCapsule (m_allocator, crc, radio0, radio1, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateCylinder (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionCylinder::CalculateSignature(dgAbs (radio0), dgAbs (radio1), dgAbs (height) * dgFloat32 (0.5f));

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* collision = new  (m_allocator) dgCollisionCylinder (m_allocator, crc, radio0, radio1, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateChamferCylinder (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionChamferCylinder::CalculateSignature(dgAbs (radius), dgAbs (height) * dgFloat32 (0.5f));

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* collision = new  (m_allocator) dgCollisionChamferCylinder (m_allocator, crc, radius, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateCone (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionCone::CalculateSignature (dgAbs (radius), dgAbs (height) * dgFloat32 (0.5f));
	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* const collision = new (m_allocator) dgCollisionCone (m_allocator, crc, radius, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateConvexHull (dgInt32 count, const dgFloat32* const vertexArray, dgInt32 strideInBytes, dgFloat32 tolerance, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionConvexHull::CalculateSignature (count, vertexArray, strideInBytes);

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);

	if (!node) {
		// shape not found create a new one and add to the cache
		dgCollisionConvexHull* const collision = new (m_allocator) dgCollisionConvexHull (m_allocator, crc, count, strideInBytes, tolerance, vertexArray);
		if (collision->GetConvexVertexCount()) {
			node = dgBodyCollisionList::Insert (collision, crc);
		} else {
			//most likely the point cloud is a plane or a line
			//could not make the shape destroy the shell and return NULL 
			//note this is the only newton shape that can return NULL;
			collision->Release();
			return NULL;
		}
	}

	// add reference to the shape and return the collision pointer
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateCompound ()
{
	// compound collision are not cached
	dgCollisionCompound* const collision = new  (m_allocator) dgCollisionCompound (this);
	dgCollisionInstance* const instance = CreateInstance (collision, 0, dgGetIdentityMatrix()); 
	collision->SetParent(instance);
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateScene ()
{
	dgCollisionScene* const collision = new (m_allocator) dgCollisionScene(this);
	dgCollisionInstance* const instance = CreateInstance (collision, 0, dgGetIdentityMatrix()); 
	collision->SetParent(instance);
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateFracturedCompound (dgMeshEffect* const solidMesh, int shapeID, int fracturePhysicsMaterialID, int pointcloudCount, const dgFloat32* const vertexCloud, int strideInBytes, int materialID, const dgMatrix& textureMatrix,
													  dgCollisionCompoundFractured::OnEmitFractureChunkCallBack emitFracfuredChunk, 
													  dgCollisionCompoundFractured::OnEmitNewCompundFractureCallBack emitFracturedCompound,
													  dgCollisionCompoundFractured::OnReconstructFractureMainMeshCallBack reconstructMainMesh)
{
	dgCollisionCompoundFractured* const collision = new (m_allocator) dgCollisionCompoundFractured (this, solidMesh, fracturePhysicsMaterialID, pointcloudCount, vertexCloud, strideInBytes, materialID, textureMatrix, emitFracfuredChunk, emitFracturedCompound, reconstructMainMesh);
	dgCollisionInstance* const instance = CreateInstance (collision, shapeID, dgGetIdentityMatrix()); 
	collision->SetParent(instance);
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateMassSpringDamperSystem (dgInt32 shapeID, dgInt32 pointCount, const dgFloat32* const points, dgInt32 strideInBytes, const dgFloat32* const pointsMass, dgInt32 linksCount, const dgInt32* const links, const dgFloat32* const linksSpring, const dgFloat32* const LinksDamper)
{
	dgCollision* const collision = new (m_allocator)dgCollisionMassSpringDamperSystem(this, shapeID, pointCount, points, strideInBytes, pointsMass, linksCount, links, linksSpring, LinksDamper);
	dgCollisionInstance* const instance = CreateInstance(collision, shapeID, dgGetIdentityMatrix());
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateDeformableSolid (dgMeshEffect* const mesh, dgInt32 shapeID)
{
	dgAssert (m_allocator == mesh->GetAllocator());
	dgCollision* const collision = new (m_allocator) dgCollisionDeformableSolidMesh (this, mesh);
	dgCollisionInstance* const instance = CreateInstance (collision, shapeID, dgGetIdentityMatrix()); 
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateBVH ()	
{
	// collision tree are not cached
	dgCollision* const collision = new  (m_allocator) dgCollisionBVH (this);
	dgCollisionInstance* const instance = CreateInstance (collision, 0, dgGetIdentityMatrix()); 
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateStaticUserMesh (const dgVector& boxP0, const dgVector& boxP1, const dgUserMeshCreation& data)
{
	dgCollision* const collision = new (m_allocator) dgCollisionUserMesh(this, boxP0, boxP1, data);
	dgCollisionInstance* const instance = CreateInstance (collision, 0, dgGetIdentityMatrix()); 
	collision->Release();
	return instance;

}

dgCollisionInstance* dgWorld::CreateHeightField(
	dgInt32 width, dgInt32 height, dgInt32 contructionMode, dgInt32 elevationDataType, 
	const void* const elevationMap, const dgInt8* const atributeMap, 
	dgFloat32 verticalScale, dgFloat32 horizontalScale_x, dgFloat32 horizontalScale_z)
{
	dgCollision* const collision = new  (m_allocator) dgCollisionHeightField (this, width, height, contructionMode, elevationMap, 
																			  elevationDataType	? dgCollisionHeightField::m_unsigned16Bit : dgCollisionHeightField::m_float32Bit,	
																			  verticalScale, atributeMap, horizontalScale_x, horizontalScale_z);
	dgCollisionInstance* const instance = CreateInstance (collision, 0, dgGetIdentityMatrix()); 
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateInstance (const dgCollision* const child, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgAssert (dgAbs (offsetMatrix[0].DotProduct(offsetMatrix[0]).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));
	dgAssert (dgAbs (offsetMatrix[1].DotProduct(offsetMatrix[1]).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));
	dgAssert (dgAbs (offsetMatrix[2].DotProduct(offsetMatrix[2]).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));
	dgAssert (dgAbs (offsetMatrix[2].DotProduct(offsetMatrix[0].CrossProduct(offsetMatrix[1])).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));

	dgAssert (offsetMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (offsetMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (offsetMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (offsetMatrix[3][3] == dgFloat32 (1.0f));
	dgCollisionInstance* const instance = new  (m_allocator) dgCollisionInstance (this, child, shapeID, offsetMatrix);
	return instance;
}

void dgWorld::SerializeCollision(dgCollisionInstance* const shape, dgSerialize serialization, void* const userData) const
{
	dgSerializeMarker(serialization, userData);
	shape->Serialize(serialization, userData);
}

dgCollisionInstance* dgWorld::CreateCollisionFromSerialization (dgDeserialize deserialization, void* const userData)
{
	dgInt32 revision = dgDeserializeMarker (deserialization, userData);
	dgCollisionInstance* const instance = new  (m_allocator) dgCollisionInstance (this, deserialization, userData, revision);
	return instance;
}

dgContactMaterial* dgWorld::GetMaterial (dgUnsigned32 bodyGroupId0, dgUnsigned32 bodyGroupId1)	const
{
	if (bodyGroupId0 > bodyGroupId1) {
		dgSwap (bodyGroupId0, bodyGroupId1);
	}

	dgUnsigned32 key = (bodyGroupId1 << 16) + bodyGroupId0;
	dgBodyMaterialList::dgTreeNode *const node = dgBodyMaterialList::Find (key);

	return node ? &node->GetInfo() : NULL;
}

dgContactMaterial* dgWorld::GetFirstMaterial () const
{
	dgBodyMaterialList::dgTreeNode *const node = dgBodyMaterialList::Minimum();
	dgAssert (node);
	return &node->GetInfo();
}

dgContactMaterial* dgWorld::GetNextMaterial (dgContactMaterial* material) const
{
	dgBodyMaterialList::dgTreeNode *const thisNode = dgBodyMaterialList::GetNodeFromInfo (*material);
	dgAssert (thisNode);
	dgBodyMaterialList::dgTreeNode *const node = (dgBodyMaterialList::dgTreeNode *)thisNode->Next();
	if (node) {
		return &node->GetInfo();
	}

	return NULL;
}

dgUnsigned32 dgWorld::GetDefualtBodyGroupID() const
{
	return m_defualtBodyGroupID;
}

dgUnsigned32 dgWorld::CreateBodyGroupID()
{
	dgContactMaterial pairMaterial;

	pairMaterial.m_aabbOverlap = NULL;
	pairMaterial.m_processContactPoint = NULL;
	pairMaterial.m_compoundAABBOverlap = NULL;

	dgUnsigned32 newId = m_bodyGroupID;
	m_bodyGroupID += 1;
	for (dgUnsigned32 i = 0; i < m_bodyGroupID ; i ++) {
		dgUnsigned32 key = (newId  << 16) + i;

		dgBodyMaterialList::Insert(pairMaterial, key);
	}

	return newId;
}

void dgWorld::ReleaseCollision(const dgCollision* const collision)
{
	dgInt32 ref = collision->Release();
	if (ref == 1) {
		dgBodyCollisionList::dgTreeNode* const node = dgBodyCollisionList::Find (collision->m_signature);
		if (node) {
			dgAssert (node->GetInfo() == collision);
			collision->Release();
			dgBodyCollisionList::Remove (node);
		}
	}
}

// ********************************************************************************
//
// separate collision system 
//
// ********************************************************************************
dgInt32 dgWorld::ClosestPoint (dgTriplex& point, const dgCollisionInstance* const collision, const dgMatrix& matrix, dgTriplex& contact, dgTriplex& normal, dgInt32 threadIndex)
{
	dgTriplex contactA;
	dgMatrix pointMatrix (dgGetIdentityMatrix());

	contact = point;
	pointMatrix.m_posit.m_x = point.m_x;
	pointMatrix.m_posit.m_y = point.m_y;
	pointMatrix.m_posit.m_z = point.m_z;
	return ClosestPoint(collision, matrix, m_pointCollision, pointMatrix, contact, contactA, normal, threadIndex);
}

dgInt32 dgWorld::ClosestPoint(const dgCollisionInstance* const collisionSrcA, const dgMatrix& matrixA, 
							  const dgCollisionInstance* const collisionSrcB, const dgMatrix& matrixB, 
							  dgTriplex& contactA, dgTriplex& contactB, dgTriplex& normalAB,dgInt32 threadIndex)
{
	dgKinematicBody collideBodyA;
	dgKinematicBody collideBodyB;
	dgContactPoint contacts[DG_MAX_CONTATCS];

	dgCollisionInstance collisionA(*collisionSrcA, collisionSrcA->GetChildShape());
	dgCollisionInstance collisionB(*collisionSrcB, collisionSrcB->GetChildShape());

	collideBodyA.m_matrix = matrixA;
	collideBodyA.m_collision = &collisionA;
	collisionA.SetGlobalMatrix(collisionA.GetLocalMatrix() * matrixA);

	collideBodyB.m_matrix = matrixB;
	collideBodyB.m_collision = &collisionB;
	collisionB.SetGlobalMatrix (collisionB.GetLocalMatrix() * matrixB);


	dgContactMaterial material;
	material.m_penetration = dgFloat32 (0.0f);

	dgContact contactJoint (this, &material, &collideBodyB, &collideBodyA);
//	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgCollisionParamProxy proxy(&contactJoint, contacts, threadIndex, false, false);

	proxy.m_body0 = &collideBodyA;
	proxy.m_instance0 = collideBodyA.m_collision;
	proxy.m_body1 = &collideBodyB;
	proxy.m_instance1 = collideBodyB.m_collision;
	proxy.m_timestep = dgFloat32 (0.0f);
	proxy.m_skinThickness = dgFloat32 (0.0f);
	proxy.m_maxContacts = 16;
	
	dgInt32 flag = 0;
	if (collisionA.IsType (dgCollision::dgCollisionCompound_RTTI)) {
		flag = ClosestCompoundPoint (proxy);
	} else if (collisionB.IsType (dgCollision::dgCollisionCompound_RTTI)) {
		dgSwap (proxy.m_body0, proxy.m_body1);
		dgSwap (proxy.m_instance0, proxy.m_instance1);
		flag = ClosestCompoundPoint (proxy);
		normalAB.m_x *= dgFloat32 (-1.0f);
		normalAB.m_y *= dgFloat32 (-1.0f);
		normalAB.m_z *= dgFloat32 (-1.0f);
	} else if (collisionA.IsType (dgCollision::dgCollisionConvexShape_RTTI) && collisionB.IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		flag = ClosestPoint (proxy);
	}

	if (flag) {
		contactA.m_x = contacts[0].m_point.m_x;
		contactA.m_y = contacts[0].m_point.m_y;
		contactA.m_z = contacts[0].m_point.m_z;
		contactB.m_x = contacts[1].m_point.m_x;
		contactB.m_y = contacts[1].m_point.m_y;
		contactB.m_z = contacts[1].m_point.m_z;
		normalAB.m_x = contacts[0].m_normal.m_x;
		normalAB.m_y = contacts[0].m_normal.m_y;
		normalAB.m_z = contacts[0].m_normal.m_z;
	}
	return flag;
}


bool dgWorld::IntersectionTest (const dgCollisionInstance* const collisionSrcA, const dgMatrix& matrixA, 
							    const dgCollisionInstance* const collisionSrcB, const dgMatrix& matrixB, 
							    dgInt32 threadIndex)
{
	dgKinematicBody collideBodyA;
	dgKinematicBody collideBodyB;
	dgCollisionInstance collisionA(*collisionSrcA, collisionSrcA->GetChildShape());
	dgCollisionInstance collisionB(*collisionSrcB, collisionSrcB->GetChildShape());

	collideBodyA.m_world = this;
	collideBodyA.SetContinueCollisionMode(false); 
	collideBodyA.m_matrix = matrixA;
	collideBodyA.m_collision = &collisionA;
	collideBodyA.UpdateCollisionMatrix(dgFloat32 (0.0f), 0);

	collideBodyB.m_world = this;
	collideBodyB.SetContinueCollisionMode(false); 
	collideBodyB.m_matrix = matrixB;
	collideBodyB.m_collision = &collisionB;
	collideBodyB.UpdateCollisionMatrix(dgFloat32 (0.0f), 0);

	dgContactMaterial material; 
	material.m_penetration = dgFloat32 (0.0f);

	dgContact contactJoint (this, &material, &collideBodyB, &collideBodyA);
//	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgBroadPhase::dgPair pair;
	pair.m_contactCount = 0;
	pair.m_contact = &contactJoint;
	pair.m_contactBuffer = NULL; 
	pair.m_timestep = dgFloat32 (0.0f);
	pair.m_cacheIsValid = 0;
	CalculateContacts (&pair, threadIndex, false, true);
	return (pair.m_contactCount == -1) ? true : false;
}

dgInt32 dgWorld::ClosestCompoundPoint (dgCollisionParamProxy& proxy) const
{
	dgCollisionInstance* const instance = proxy.m_instance0;
	dgAssert (instance->IsType(dgCollision::dgCollisionCompound_RTTI));
	dgCollisionCompound* const collision = (dgCollisionCompound*) instance->GetChildShape();
	return collision->ClosestDistance (proxy);
}

// **********************************************************************************
//
// dynamics collision system
//
// **********************************************************************************
static inline dgInt32 CompareContact (const dgContactPoint* const contactA, const dgContactPoint* const contactB, void* dommy)
{
	if (contactA->m_point[0] < contactB->m_point[0]) {
		return -1;
	} else if (contactA->m_point[0] > contactB->m_point[0]) {
		return 1;
	} else {
		return 0;
	}
}


DG_INLINE dgInt32 dgWorld::PruneSupport(int count, const dgVector& dir, const dgVector* points) const
{
	dgInt32 index = 0;
	dgFloat32 maxVal = dgFloat32(-1.0e20f);
	for (dgInt32 i = 0; i < count; i++) {
		dgFloat32 dist = dir.DotProduct(points[i]).GetScalar();
		if (dist > maxVal) {
			index = i;
			maxVal = dist;
		}
	}
	return index;
}

dgInt32 dgWorld::Prune2dContacts(const dgMatrix& matrix, dgInt32 count, dgContactPoint* const contactArray, int maxCount, dgFloat32 tol) const
{
	class dgConveFaceNode
	{
		public:
		dgVector m_point2d;
		dgContactPoint m_contact;
		dgConveFaceNode* m_next;
		dgConveFaceNode* m_prev;
		dgInt32 m_mask;
	};

	class dgHullStackSegment
	{
		public:
		dgVector m_p0;
		dgVector m_p1;
		dgConveFaceNode* m_edgeP0;
	};

	dgVector xyMask(dgVector::m_xMask | dgVector::m_yMask);

	dgVector array[DG_MAX_CONTATCS];
	dgHullStackSegment stack[DG_MAX_CONTATCS];

	dgConveFaceNode convexHull[32];
	dgContactPoint buffer[32];

	dgFloat32 maxPenetration = dgFloat32(0.0f);
	for (dgInt32 i = 0; i < count; i++) {
		array[i] = matrix.UntransformVector(contactArray[i].m_point) & xyMask;
		maxPenetration = dgMax (maxPenetration, contactArray[i].m_penetration);
	}

	dgInt32 i0 = PruneSupport(count, dgCollisionContactCloud::m_pruneSupportX, array);
	count--;
	convexHull[0].m_point2d = array[i0];
	convexHull[0].m_contact = contactArray[i0];
	stack[0].m_p0 = array[i0];
	array[i0] = array[count];
	contactArray[i0] = contactArray[count];

	dgInt32 i1 = PruneSupport(count, dgCollisionContactCloud::m_pruneSupportX.Scale(dgFloat32(-1.0f)), array);
	count--;
	convexHull[1].m_point2d = array[i1];
	convexHull[1].m_contact = contactArray[i1];
	stack[0].m_p1 = array[i1];
	array[i1] = array[count];
	contactArray[i1] = contactArray[count];

	stack[0].m_edgeP0 = &convexHull[0];
	convexHull[0].m_next = &convexHull[1];
	convexHull[0].m_prev = &convexHull[1];
	convexHull[1].m_next = &convexHull[0];
	convexHull[1].m_prev = &convexHull[0];

	stack[1].m_edgeP0 = &convexHull[1];
	stack[1].m_p0 = stack[0].m_p1;
	stack[1].m_p1 = stack[0].m_p0;

	int hullCount = 2;
	dgInt32 stackIndex = 2;
	dgFloat32 totalArea = dgFloat32 (0.0f);
	while (stackIndex && count && (hullCount < sizeof (convexHull)/sizeof (convexHull[0]))) {
		stackIndex--;

		dgHullStackSegment segment(stack[stackIndex]);
		dgVector p1p0((segment.m_p1 - segment.m_p0));
		dgFloat32 mag2 = p1p0.DotProduct(p1p0).GetScalar();
		if (mag2 > dgFloat32 (1.0e-5f)) {	
			dgVector dir(dgCollisionContactCloud::m_pruneUpDir.CrossProduct(p1p0));
			dgInt32 newIndex = PruneSupport(count, dir, array);

			dgVector edge(array[newIndex] - segment.m_p0);
			dgVector normal(p1p0.CrossProduct(edge));
			if (normal.m_z > dgFloat32(1.e-4f)) {
				totalArea += normal.m_z;
				dgAssert(stackIndex < (DG_MAX_CONTATCS - 2));
				convexHull[hullCount].m_point2d = array[newIndex];
				convexHull[hullCount].m_contact = contactArray[newIndex];
				convexHull[hullCount].m_next = segment.m_edgeP0->m_next;
				segment.m_edgeP0->m_next->m_prev = &convexHull[hullCount];

				convexHull[hullCount].m_prev = segment.m_edgeP0;
				segment.m_edgeP0->m_next = &convexHull[hullCount];

				stack[stackIndex + 0].m_p0 = segment.m_p0;
				stack[stackIndex + 0].m_p1 = array[newIndex];
				stack[stackIndex + 0].m_edgeP0 = segment.m_edgeP0;

				stack[stackIndex + 1].m_p0 = array[newIndex];
				stack[stackIndex + 1].m_p1 = segment.m_p1;
				stack[stackIndex + 1].m_edgeP0 = &convexHull[hullCount];

				hullCount++;
				stackIndex += 2;
				count--;
				array[newIndex] = array[count];
				contactArray[newIndex] = contactArray[count];
			}
		}
	}
	dgAssert (hullCount < sizeof (convexHull)/sizeof (convexHull[0]));

	dgUpHeap<dgConveFaceNode*, dgFloat32> sortHeap(array, sizeof (array));
	dgConveFaceNode* hullPoint = &convexHull[0];

	bool hasLinearCombination = true;
	while (hasLinearCombination) {
		sortHeap.Flush();
		hasLinearCombination = false;
		dgConveFaceNode* ptr = hullPoint;
		dgVector e0 (ptr->m_next->m_point2d - ptr->m_point2d);
		do {
			dgVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			dgFloat32 area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (sortHeap.Value() * dgFloat32 (16.0f) < totalArea)) {
			dgConveFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) {
				if (hullPoint == corner) {
					hullPoint = corner->m_prev;
				}
				hullCount --;
				hasLinearCombination = true;
				corner->m_prev->m_mask = 0; 
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}
	
	while (hullCount > maxCount) {
		sortHeap.Flush();
		dgConveFaceNode* ptr = hullPoint;
		dgVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do {
			dgVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			dgFloat32 area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (hullCount > maxCount)) {
			dgConveFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) {
				if (hullPoint == corner) {
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

	hullCount = 0;
	dgConveFaceNode* ptr = hullPoint;
	do {
		contactArray[hullCount] = ptr->m_contact;
		//contactArray[hullCount].m_normal = averageNormal;
		contactArray[hullCount].m_penetration = maxPenetration;
		hullCount ++;
		ptr = ptr->m_next;
	} while (ptr != hullPoint);
	return hullCount;
}


DG_MSC_VECTOR_ALIGMENT
class dgMinkFace___
{
	public:
	dgPlane m_plane;
	dgMinkFace___* m_twin[3];
	dgInt16 m_vertex[3];
	dgInt8 m_mark;
	dgInt8 m_alive;
} DG_GCC_VECTOR_ALIGMENT;

#define DG_PRUNE_SIZE 64
class dgContactSolver___: public dgDownHeap<dgMinkFace___*, dgFloat32>  
{
	class dgFaceFreeList
	{
		public:
		dgFaceFreeList* m_next;
	};

	public:
	dgContactSolver___()
		:dgDownHeap<dgMinkFace___*, dgFloat32>(m_heapBuffer, sizeof (m_heapBuffer))
		,m_freeFace(NULL)
		,m_faceIndex(0)
	{
	}

	dgMinkFace___* NewFace()
	{
		dgMinkFace___* face = (dgMinkFace___*)m_freeFace;
		if (m_freeFace) {
			m_freeFace = m_freeFace->m_next;
		} else {
			face = &m_facePool[m_faceIndex];
			m_faceIndex++;
			if (m_faceIndex >= DG_CONVEX_MINK_MAX_FACES) {
				return NULL;
			}
		}

#ifdef _DEBUG
		memset(face, 0, sizeof (dgMinkFace));
#endif
		return face;
	}

	void DeleteFace(dgMinkFace___* const face)
	{
		dgFaceFreeList* const freeFace = (dgFaceFreeList*)face;
		freeFace->m_next = m_freeFace;
		m_freeFace = freeFace;
	}

	dgMinkFace___* AddFace(dgInt32 v0, dgInt32 v1, dgInt32 v2)
	{
		dgMinkFace___* const face = NewFace();
		face->m_mark = 0;
		face->m_vertex[0] = dgInt16(v0);
		face->m_vertex[1] = dgInt16(v1);
		face->m_vertex[2] = dgInt16(v2);
		return face;
	}

	dgMinkFace___ m_facePool[DG_PRUNE_SIZE];
	dgInt8 m_heapBuffer[DG_PRUNE_SIZE * (sizeof (dgFloat32) + sizeof (dgMinkFace___ *))];
	dgFaceFreeList* m_freeFace; 
	dgInt32 m_faceIndex;
};

dgInt32 dgWorld::Prune3dContacts(const dgMatrix& matrix, dgInt32 count, dgContactPoint* const contactArray, int maxCount, dgFloat32 distTol) const
{
	dgVector array[DG_MAX_CONTATCS];
	dgFloat32 max_x = dgFloat32 (1.0e20f);
	dgInt32 maxIndex = 0;
	for (dgInt32 i = 0; i < count; i++) {
		array[i] = matrix.UntransformVector(contactArray[i].m_point);
		array[i].m_w = dgFloat32 (i);
		if (array[i].m_x < max_x) {
			maxIndex = i;
			max_x = array[i].m_x;
		}
	}
	dgSwap (array[0], array[maxIndex]);

	for (dgInt32 i = 2; i < count; i++) {
		dgInt32 j = i;
		dgVector tmp (array[i]);
		for (; array[j - 1].m_x > tmp.m_x; j--) {
			dgAssert(j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}
	
	dgFloat32 window = dgFloat32 (2.5e-3f);
	do {
		dgInt32 packContacts = 0;
		window *= dgFloat32 (2.0f);
		const dgFloat32 window2 = window * window;

		dgUnsigned8 mask[DG_MAX_CONTATCS];
		memset (mask, 0, count * sizeof (dgUnsigned8));
		for (dgInt32 i = 0; i < count; i++) {
			if (!mask[i]) {
				const dgFloat32 val = array[i].m_x + window;
				for (dgInt32 j = i + 1; (j < count) && (array[j].m_x < val); j++) {
					if (!mask[j]) {
						dgVector dp((array[j] - array[i]) & dgVector::m_triplexMask);
						dgAssert(dp.m_w == dgFloat32(0.0f));
						dgFloat32 dist2 = dp.DotProduct(dp).GetScalar();
						if (dist2 < window2) {
							mask[j] = 1;
							packContacts = 1;
						}
					}
				}
			}
		}

		if (packContacts) {
			dgInt32 j = 0;
			for (dgInt32 i = 0; i < count; i++) {
				if (!mask[i]) {
					array[j] = array[i];
					j++;
				}
			}
			count = j;
		}
	} while (count > maxCount);

	dgContactPoint tmpContact [16];
	for (dgInt32 i = 0; i < count; i++) {
		dgInt32 index = dgInt32 (array[i].m_w);
		tmpContact[i] = contactArray[index];
	}

	memcpy (contactArray, tmpContact, count * sizeof (dgContactPoint));


//	dgVector dir0(dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f));
//	dgInt32 i0 = PruneSupport(count, dir0, array);
//	count--;
////	hull[hullCount] = contactArray[i0];
////	hullCount++;
////	stack[0][0] = array[i0];
//	array[i0] = array[count];
//	contactArray[i0] = contactArray[count];
//
//	dgInt32 i1 = PruneSupport(count, dir0.Scale(dgFloat32(-1.0f)), array);
////	count--;
////	hull[hullCount] = contactArray[i1];
////	hullCount++;
////	stack[0][1] = array[i1];
//	array[i1] = array[count];
//	contactArray[i1] = contactArray[count];
//
//	dgVector dir1(dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f));
//	dgInt32 i2 = PruneSupport(count, dir0.Scale(dgFloat32(-1.0f)), array);
////	count--;
////	hull[hullCount] = contactArray[i2];
////	hullCount++;
////	stack[0][1] = array[i2];
//	array[i2] = array[count];
//	contactArray[i2] = contactArray[count];
//
//	dgContactSolver___ heap;
//
//
//	// clear the face cache!!
////	Flush();
////	m_faceIndex = 0;
////	m_vertexIndex = 4;
////	m_freeFace = NULL;
////
////	dgMinkFace* const f0 = AddFace(0, 1, 2);
////	dgMinkFace* const f1 = AddFace(0, 2, 3);
////	dgMinkFace* const f2 = AddFace(2, 1, 3);
////	dgMinkFace* const f3 = AddFace(1, 0, 3);
////
////	f0->m_twin[0] = f3;
////	f0->m_twin[1] = f2;
////	f0->m_twin[2] = f1;
////
////	f1->m_twin[0] = f0;
////	f1->m_twin[1] = f2;
////	f1->m_twin[2] = f3;
////
////	f2->m_twin[0] = f0;
////	f2->m_twin[1] = f3;
////	f2->m_twin[2] = f1;
////
////	f3->m_twin[0] = f0;
////	f3->m_twin[1] = f1;
////	f3->m_twin[2] = f2;
////
////	PushFace(f0);
////	PushFace(f1);
////	PushFace(f2);
////	PushFace(f3);
//	dgMinkFace___* const f0 = heap.AddFace(0, 1, 2);
//	dgMinkFace___* const f1 = heap.AddFace(2, 1, 0);
//
//	f0->m_twin[0] = f1;
//	f0->m_twin[1] = f1;
//	f0->m_twin[2] = f1;
//
//	f1->m_twin[0] = f0;
//	f1->m_twin[1] = f0;
//	f1->m_twin[2] = f0;
//
////	dgInt32 cycling = 0;
////	dgInt32 iterCount = 0;
////	dgFloat32 cyclingMem[4];
////	cyclingMem[0] = dgFloat32(1.0e10f);
////	cyclingMem[1] = dgFloat32(1.0e10f);
////	cyclingMem[2] = dgFloat32(1.0e10f);
////	cyclingMem[3] = dgFloat32(1.0e10f);
////
////	const dgFloat32 resolutionScale = dgFloat32(0.125f);
////	const dgFloat32 minTolerance = DG_PENETRATION_TOL;
//
//	while (heap.GetCount()) {
//		dgMinkFace___* const faceNode = heap[0];
//		heap.Pop();
//
//		if (faceNode->m_alive) {
///*
//			SupportVertex(faceNode->m_plane & dgVector::m_triplexMask, m_vertexIndex);
//			const dgVector& p = m_hullDiff[m_vertexIndex];
//			dgFloat32 dist = faceNode->m_plane.Evalue(p);
//			dgFloat32 distTolerance = dgMax(dgAbs(faceNode->m_plane.m_w) * resolutionScale, minTolerance);
//
//			if (dist < distTolerance) {
//				dgVector sum[3];
//				dgVector diff[3];
//				m_normal = faceNode->m_plane & dgVector::m_triplexMask;
//				for (dgInt32 i = 0; i < 3; i++) {
//					dgInt32 j = faceNode->m_vertex[i];
//					sum[i] = m_hullSum[j];
//					diff[i] = m_hullDiff[j];
//				}
//				for (dgInt32 i = 0; i < 3; i++) {
//					m_hullSum[i] = sum[i];
//					m_hullDiff[i] = diff[i];
//				}
//				return 3;
//			}
//
//			iterCount++;
//			bool isCycling = false;
//			cyclingMem[cycling] = dist;
//			if (iterCount > 10) {
//				dgInt32 cyclingIndex = cycling;
//				for (dgInt32 i = 0; i < 3; i++) {
//					dgInt32 cyclingIndex0 = (cyclingIndex - 1) & 3;
//					if (((cyclingMem[cyclingIndex0] - cyclingMem[cyclingIndex]) < dgFloat32(-1.0e-5f))) {
//						isCycling = true;
//						cyclingMem[0] = dgFloat32(1.0e10f);
//						cyclingMem[1] = dgFloat32(1.0e10f);
//						cyclingMem[2] = dgFloat32(1.0e10f);
//						cyclingMem[3] = dgFloat32(1.0e10f);
//						break;
//					}
//					cyclingIndex = cyclingIndex0;
//				}
//			}
//			cycling = (cycling + 1) & 3;
//
//			if (!isCycling) {
//				m_faceStack[0] = faceNode;
//				dgInt32 stackIndex = 1;
//				dgInt32 deletedCount = 0;
//
//				while (stackIndex) {
//					stackIndex--;
//					dgMinkFace* const face = m_faceStack[stackIndex];
//
//					if (!face->m_mark && (face->m_plane.Evalue(p) > dgFloat32(0.0f))) {
//#ifdef _DEBUG
//						for (dgInt32 i = 0; i < deletedCount; i++) {
//							dgAssert(m_deletedFaceList[i] != face);
//						}
//#endif
//
//						m_deletedFaceList[deletedCount] = face;
//						deletedCount++;
//						dgAssert(deletedCount < sizeof (m_deletedFaceList) / sizeof (m_deletedFaceList[0]));
//						face->m_mark = 1;
//
//						for (dgInt32 i = 0; i < 3; i++) {
//							dgMinkFace* const twinFace = face->m_twin[i];
//							if (twinFace && !twinFace->m_mark) {
//								m_faceStack[stackIndex] = twinFace;
//								stackIndex++;
//								dgAssert(stackIndex < sizeof (m_faceStack) / sizeof (m_faceStack[0]));
//							}
//						}
//					}
//				}
//
//				//dgAssert (SanityCheck());
//				dgInt32 newCount = 0;
//				for (dgInt32 i = 0; i < deletedCount; i++) {
//					dgMinkFace* const face = m_deletedFaceList[i];
//					face->m_alive = 0;
//					dgAssert(face->m_mark == 1);
//					dgInt32 j0 = 2;
//					for (dgInt32 j1 = 0; j1 < 3; j1++) {
//						dgMinkFace* const twinFace = face->m_twin[j0];
//						if (twinFace && !twinFace->m_mark) {
//							//dgMinkFace* const newFace = AddFace(m_vertexIndex, face->m_vertex[j0], face->m_vertex[j1]);
//							dgMinkFace* const newFace = NewFace();
//							if (newFace) {
//								newFace->m_mark = 0;
//								newFace->m_vertex[0] = dgInt16(m_vertexIndex);
//								newFace->m_vertex[1] = dgInt16(face->m_vertex[j0]);
//								newFace->m_vertex[2] = dgInt16(face->m_vertex[j1]);
//								PushFace(newFace);
//
//								newFace->m_twin[1] = twinFace;
//								dgInt32 index = (twinFace->m_twin[0] == face) ? 0 : ((twinFace->m_twin[1] == face) ? 1 : 2);
//								twinFace->m_twin[index] = newFace;
//
//								m_coneFaceList[newCount] = newFace;
//								newCount++;
//								dgAssert(newCount < sizeof(m_coneFaceList) / sizeof(m_coneFaceList[0]));
//							}
//							else {
//								// this is very rare but is does happend with some degenerated faces.
//								return -1;
//							}
//						}
//						j0 = j1;
//					}
//				}
//
//				dgInt32 i0 = newCount - 1;
//				for (dgInt32 i1 = 0; i1 < newCount; i1++) {
//					dgMinkFace* const faceA = m_coneFaceList[i0];
//					dgAssert(faceA->m_mark == 0);
//
//					dgInt32 j0 = newCount - 1;
//					for (dgInt32 j1 = 0; j1 < newCount; j1++) {
//						if (i0 != j0) {
//							dgMinkFace* const faceB = m_coneFaceList[j0];
//							dgAssert(faceB->m_mark == 0);
//							if (faceA->m_vertex[2] == faceB->m_vertex[1]) {
//								faceA->m_twin[2] = faceB;
//								faceB->m_twin[0] = faceA;
//								break;
//							}
//						}
//						j0 = j1;
//					}
//					i0 = i1;
//				}
//
//				m_vertexIndex++;
//				dgAssert(m_vertexIndex < sizeof (m_hullDiff) / sizeof (m_hullDiff[0]));
//
//				dgAssert(SanityCheck());
//			}
//*/
//		} else {
//			heap.DeleteFace(faceNode);
//		}
//	}

	return count;
}

dgInt32 dgWorld::PruneContacts (dgInt32 count, dgContactPoint* const contactArray, dgFloat32 distTolerenace, dgInt32 maxCount) const
{
	dgVector origin(dgVector::m_zero);
	for (dgInt32 i = 0; i < count; i++) {
		origin += contactArray[i].m_point;
	}
	dgVector scale (dgFloat32(1.0) / count);
	origin = origin * scale;
	origin.m_w = dgFloat32 (1.0f);

	dgMatrix covariance(dgGetZeroMatrix());
	for (dgInt32 i = 0; i < count; i++) {
		dgVector p (contactArray[i].m_point - origin);
		dgAssert (p.m_w == dgFloat32 (0.0f));
		dgMatrix matrix(p, p);
		covariance.m_front += matrix.m_front;
		covariance.m_up += matrix.m_up;
		covariance.m_right += matrix.m_right;
	}

	for (dgInt32 i = 0; i < 3; i++) {
		if (dgAbs(covariance[i][i]) < (1.0e-6f)) {
			for (dgInt32 j = 0; j < 3; j++) {
				covariance[i][j] = dgFloat32(0.0f);
				covariance[j][i] = dgFloat32(0.0f);
			}
		}
	}
	dgVector eigen (covariance.EigenVectors());
	covariance.m_posit = origin;

	if (eigen[1] < eigen[2]) {
		dgSwap(eigen[1], eigen[2]);
		dgSwap(covariance[1], covariance[2]);
	}
	if (eigen[0] < eigen[1]) {
		dgSwap(eigen[0], eigen[1]);
		dgSwap(covariance[0], covariance[1]);
	}
	if (eigen[1] < eigen[2]) {
		dgSwap(eigen[1], eigen[2]);
		dgSwap(covariance[1], covariance[2]);
	}

	const dgFloat32 eigenValueError = dgFloat32 (1.0e-4f);
	if (eigen[2] > eigenValueError) {
		// 3d convex Hull
		return Prune3dContacts(covariance, count, contactArray, maxCount, distTolerenace);
	} else if (eigen[1] > eigenValueError) {
		// is a 2d or 1d convex hull
		return Prune2dContacts(covariance, count, contactArray, maxCount, distTolerenace);
	} else if (eigen[0] > eigenValueError) {
		// is a 1d or 1d convex hull
		if (count > 2) {
			dgFloat32 maxValue = dgFloat32(-1.0e10f);
			dgFloat32 minValue = dgFloat32(-1.0e10f);
			dgInt32 j0 = 0;
			dgInt32 j1 = 0;
			for (dgInt32 i = 0; i < count; i++) {
				dgFloat32 dist = contactArray[i].m_point.DotProduct(covariance.m_front).GetScalar();
				if (dist > maxValue) {
					j0 = i;
					maxValue = dist;
				}
				if (-dist > minValue) {
					j1 = i;
					minValue = -dist;
				}
			}
			dgContactPoint c0(contactArray[j0]);
			dgContactPoint c1(contactArray[j1]);
			contactArray[0] = c0;
			contactArray[1] = c1;
		}
		return 2;
	}
	return 1;
}

dgInt32 dgWorld::PruneContactsByRank(dgInt32 count, dgCollisionParamProxy& proxy, dgInt32 maxCount) const
{
	dgJacobian jt[DG_CONSTRAINT_MAX_ROWS / 3];
	dgFloat32 massMatrix[DG_CONSTRAINT_MAX_ROWS * DG_CONSTRAINT_MAX_ROWS / 9];

	dgAssert (count > maxCount);
	const dgBody* const body = proxy.m_body0;
	dgVector com(body->m_globalCentreOfMass);
	for (dgInt32 i = 0; i < count; i++) {
		jt[i].m_linear = proxy.m_contacts[i].m_normal;
		jt[i].m_angular = (proxy.m_contacts[i].m_point - com).CrossProduct(proxy.m_contacts[i].m_normal);
	}


	dgInt32 index = 0;
	for (dgInt32 i = 0; i < count; i++) {
		dgFloat32* const row = &massMatrix[index];
		const dgJacobian& gInvMass = jt[i];
		dgVector aii(gInvMass.m_linear * jt[i].m_linear + gInvMass.m_angular * jt[i].m_angular);
		row[i] = aii.AddHorizontal().GetScalar() * dgFloat32(1.0001f);
		for (dgInt32 j = i + 1; j < count; j++) {
			dgVector aij(gInvMass.m_linear * jt[j].m_linear + gInvMass.m_angular * jt[j].m_angular);
			dgFloat32 b = aij.AddHorizontal().GetScalar();
			row[j] = b;
			massMatrix[j * count + i] = b;
		}
		index += count;
	}

	dgFloat32 eigenValues[DG_CONSTRAINT_MAX_ROWS / 3];
	dgEigenValues(count, count, massMatrix, eigenValues);

	for (dgInt32 i = 1; i < count; i++) {
		dgFloat32 value = eigenValues[i];
		dgContactPoint point (proxy.m_contacts[i]);

		dgInt32 j = i - 1;
		for (; (j >= 0) && (eigenValues[j] < value); j--) {
			eigenValues[j + 1] = eigenValues[j];
			proxy.m_contacts[j + 1] = proxy.m_contacts[j];
		}
		eigenValues[j + 1] = value;
		proxy.m_contacts[j + 1] = point;
	}

	return maxCount;
}


void dgWorld::ProcessCachedContacts (dgContact* const contact, dgFloat32 timestep, dgInt32 threadIndex) const
{
	dgAssert (contact);
	dgAssert (contact->m_body0);
	dgAssert (contact->m_body1);
	dgAssert (contact->m_material);
	dgAssert (contact->m_body0 != contact->m_body1);

	dgList<dgContactMaterial>& list = *contact;
	const dgContactMaterial* const material = contact->m_material;

	dgList<dgContactMaterial>::dgListNode* nextContactNode;
	for (dgList<dgContactMaterial>::dgListNode *contactNode = list.GetFirst(); contactNode; contactNode = nextContactNode) {
		nextContactNode = contactNode->GetNext();
		dgContactMaterial& contactMaterial = contactNode->GetInfo();

		dgAssert (dgCheckFloat(contactMaterial.m_point.m_x));
		dgAssert (dgCheckFloat(contactMaterial.m_point.m_y));
		dgAssert (dgCheckFloat(contactMaterial.m_point.m_z));
		dgAssert (contactMaterial.m_body0);
		dgAssert (contactMaterial.m_body1);
		dgAssert (contactMaterial.m_collision0);
		dgAssert (contactMaterial.m_collision1);
		dgAssert (contactMaterial.m_body0 == contact->m_body0);
		dgAssert (contactMaterial.m_body1 == contact->m_body1);
		//dgAssert (contactMaterial.m_userId != 0xffffffff);

		contactMaterial.m_softness = material->m_softness;
		contactMaterial.m_restitution = material->m_restitution;
		contactMaterial.m_staticFriction0 = material->m_staticFriction0;
		contactMaterial.m_staticFriction1 = material->m_staticFriction1;
		contactMaterial.m_dynamicFriction0 = material->m_dynamicFriction0;
		contactMaterial.m_dynamicFriction1 = material->m_dynamicFriction1;

		contactMaterial.m_flags = dgContactMaterial::m_collisionEnable | (material->m_flags & (dgContactMaterial::m_friction0Enable | dgContactMaterial::m_friction1Enable));
		contactMaterial.m_userData = material->m_userData;
	}

	contact->m_maxDOF = dgUnsigned32 (3 * contact->GetCount());
	if (material->m_processContactPoint) {
		material->m_processContactPoint(*contact, timestep, threadIndex);
	}
}

void dgWorld::PopulateContacts (dgBroadPhase::dgPair* const pair, dgInt32 threadIndex)
{
	dgContact* const contact = pair->m_contact;
	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
	dgAssert (body0 != body1);
	dgAssert (body0);
	dgAssert (body1);
	dgAssert (contact->m_body0 == body0);
	dgAssert (contact->m_body1 == body1);

	const dgContactMaterial* const material = contact->m_material;
	const dgContactPoint* const contactArray = pair->m_contactBuffer;

	dgInt32 contactCount = pair->m_contactCount;
	dgList<dgContactMaterial>& list = *contact;

	contact->m_timeOfImpact = pair->m_timestep;

	dgInt32 count = 0;
	dgVector cachePosition [DG_MAX_CONTATCS];
	dgList<dgContactMaterial>::dgListNode *nodes[DG_MAX_CONTATCS];

	for (dgList<dgContactMaterial>::dgListNode *contactNode = list.GetFirst(); contactNode; contactNode = contactNode->GetNext()) {

		nodes[count] = contactNode;
		cachePosition[count] = contactNode->GetInfo().m_point;
		count ++;
	}

	const dgVector& v0 = body0->m_veloc;
	const dgVector& w0 = body0->m_omega;
	const dgVector& com0 = body0->m_globalCentreOfMass;

	const dgVector& v1 = body1->m_veloc;
	const dgVector& w1 = body1->m_omega;
	const dgVector& com1 = body1->m_globalCentreOfMass;

	dgVector controlDir0 (dgFloat32 (0.0f));
	dgVector controlDir1 (dgFloat32 (0.0f));
	dgVector controlNormal (contactArray[0].m_normal);
	dgVector vel0 (v0 + w0.CrossProduct(contactArray[0].m_point - com0));
	dgVector vel1 (v1 + w1.CrossProduct(contactArray[0].m_point - com1));
	dgVector vRel (vel1 - vel0);
	dgAssert (controlNormal.m_w == dgFloat32 (0.0f));
	dgVector tangDir (vRel - controlNormal * vRel.DotProduct(controlNormal));
	dgAssert (tangDir.m_w == dgFloat32 (0.0f));
	dgFloat32 diff = tangDir.DotProduct(tangDir).GetScalar();

	dgInt32 staticMotion = 0;
	if (diff <= dgFloat32 (1.0e-2f)) {
		staticMotion = 1;
		if (dgAbs (controlNormal.m_z) > dgFloat32 (0.577f)) {
			tangDir = dgVector (-controlNormal.m_y, controlNormal.m_z, dgFloat32 (0.0f), dgFloat32 (0.0f));
		} else {
			tangDir = dgVector (-controlNormal.m_y, controlNormal.m_x, dgFloat32 (0.0f), dgFloat32 (0.0f));
		}
		controlDir0 = controlNormal.CrossProduct(tangDir);
		dgAssert (controlDir0.m_w == dgFloat32 (0.0f));
		dgAssert (controlDir0.DotProduct(controlDir0).GetScalar() > dgFloat32 (1.0e-8f));
		controlDir0 = controlDir0.Normalize();
		controlDir1 = controlNormal.CrossProduct(controlDir0);
		dgAssert (dgAbs(controlNormal.DotProduct(controlDir0.CrossProduct(controlDir1)).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	}

	dgFloat32 maxImpulse = dgFloat32 (-1.0f);
//	dgFloat32 breakImpulse0 = dgFloat32 (0.0f);
//	dgFloat32 breakImpulse1 = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < contactCount; i ++) {

		dgList<dgContactMaterial>::dgListNode* contactNode = NULL;
		dgFloat32 min = dgFloat32 (1.0e20f);
		dgInt32 index = -1;
		for (dgInt32 j = 0; j < count; j ++) {
			dgVector v (cachePosition[j] - contactArray[i].m_point);
			dgAssert (v.m_w == dgFloat32 (0.0f));
			diff = v.DotProduct(v).GetScalar();
			if (diff < min) {
				min = diff;
				index = j;
				contactNode = nodes[j];
			}
		}

		if (contactNode) {
			count --;
			dgAssert (index != -1);
			nodes[index] = nodes[count];
			cachePosition[index] = cachePosition[count];
		} else {
			GlobalLock();
			contactNode = list.Append ();
			GlobalUnlock();
		}

		dgContactMaterial* const contactMaterial = &contactNode->GetInfo();

		dgAssert (dgCheckFloat(contactArray[i].m_point.m_x));
		dgAssert (dgCheckFloat(contactArray[i].m_point.m_y));
		dgAssert (dgCheckFloat(contactArray[i].m_point.m_z));
		dgAssert (contactArray[i].m_body0);
		dgAssert (contactArray[i].m_body1);
		dgAssert (contactArray[i].m_collision0);
		dgAssert (contactArray[i].m_collision1);
		dgAssert (contactArray[i].m_body0 == body0);
		dgAssert (contactArray[i].m_body1 == body1);

		contactMaterial->m_point = contactArray[i].m_point;
		contactMaterial->m_normal = contactArray[i].m_normal;
		contactMaterial->m_penetration = contactArray[i].m_penetration;
		contactMaterial->m_body0 = contactArray[i].m_body0;
		contactMaterial->m_body1 = contactArray[i].m_body1;
		contactMaterial->m_collision0 = contactArray[i].m_collision0;
		contactMaterial->m_collision1 = contactArray[i].m_collision1;
		contactMaterial->m_shapeId0 = contactArray[i].m_shapeId0;
		contactMaterial->m_shapeId1 = contactArray[i].m_shapeId1;
		contactMaterial->m_softness = material->m_softness;
		contactMaterial->m_restitution = material->m_restitution;
		contactMaterial->m_staticFriction0 = material->m_staticFriction0;
		contactMaterial->m_staticFriction1 = material->m_staticFriction1;
		contactMaterial->m_dynamicFriction0 = material->m_dynamicFriction0;
		contactMaterial->m_dynamicFriction1 = material->m_dynamicFriction1;

		dgAssert (dgAbs(contactMaterial->m_normal.DotProduct(contactMaterial->m_normal).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-1f));

		//contactMaterial.m_collisionEnable = true;
		//contactMaterial.m_friction0Enable = material->m_friction0Enable;
		//contactMaterial.m_friction1Enable = material->m_friction1Enable;
		//contactMaterial.m_override0Accel = false;
		//contactMaterial.m_override1Accel = false;
		//contactMaterial.m_overrideNormalAccel = false;
		contactMaterial->m_flags = dgContactMaterial::m_collisionEnable | (material->m_flags & (dgContactMaterial::m_friction0Enable | dgContactMaterial::m_friction1Enable));
		contactMaterial->m_userData = material->m_userData;

		if (staticMotion) {
			if (contactMaterial->m_normal.DotProduct(controlNormal).GetScalar() > dgFloat32 (0.9995f)) {
				contactMaterial->m_dir0 = controlDir0;
				contactMaterial->m_dir1 = controlDir1;
			} else {
				if (dgAbs (contactMaterial->m_normal.m_z) > dgFloat32 (0.577f)) {
					tangDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_z, dgFloat32 (0.0f), dgFloat32 (0.0f));
				} else {
					tangDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_x, dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
				contactMaterial->m_dir0 = contactMaterial->m_normal.CrossProduct(tangDir);
				dgAssert (contactMaterial->m_dir0.m_w == dgFloat32 (0.0f));
				dgAssert (contactMaterial->m_dir0.DotProduct(contactMaterial->m_dir0).GetScalar() > dgFloat32 (1.0e-8f));
				contactMaterial->m_dir0 = contactMaterial->m_dir0.Normalize();
				contactMaterial->m_dir1 = contactMaterial->m_normal.CrossProduct(contactMaterial->m_dir0);
				dgAssert (dgAbs(contactMaterial->m_normal.DotProduct(contactMaterial->m_dir0.CrossProduct(contactMaterial->m_dir1)).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			}
		} else {
			dgVector veloc0 (v0 + w0.CrossProduct(contactMaterial->m_point - com0));
			dgVector veloc1 (v1 + w1.CrossProduct(contactMaterial->m_point - com1));
			dgVector relReloc (veloc1 - veloc0);

			dgAssert (contactMaterial->m_normal.m_w == dgFloat32 (0.0f));
			dgFloat32 impulse = relReloc.DotProduct(contactMaterial->m_normal).GetScalar();
			if (dgAbs (impulse) > maxImpulse) {
				maxImpulse = dgAbs (impulse); 
//				breakImpulse0 = contactMaterial->m_collision0->GetBreakImpulse();
//				breakImpulse1 = contactMaterial->m_collision1->GetBreakImpulse();
			}
			
			dgVector tangentDir (relReloc - contactMaterial->m_normal.Scale (impulse));
			dgAssert (tangentDir.m_w == dgFloat32 (0.0f));
			diff = tangentDir.DotProduct(tangentDir).GetScalar();
			if (diff > dgFloat32 (1.0e-2f)) {
				dgAssert (tangentDir.m_w == dgFloat32 (0.0f));
				//contactMaterial->m_dir0 = tangentDir.Scale (dgRsqrt (diff));
				contactMaterial->m_dir0 = tangentDir.Normalize();
			} else {
				if (dgAbs (contactMaterial->m_normal.m_z) > dgFloat32 (0.577f)) {
					tangentDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_z, dgFloat32 (0.0f), dgFloat32 (0.0f));
				} else {
					tangentDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_x, dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
				contactMaterial->m_dir0 = contactMaterial->m_normal.CrossProduct(tangentDir);
				dgAssert (contactMaterial->m_dir0.m_w == dgFloat32 (0.0f));
				dgAssert (contactMaterial->m_dir0.DotProduct(contactMaterial->m_dir0).GetScalar() > dgFloat32 (1.0e-8f));
				contactMaterial->m_dir0 = contactMaterial->m_dir0.Normalize();
			}
			contactMaterial->m_dir1 = contactMaterial->m_normal.CrossProduct(contactMaterial->m_dir0);
			dgAssert (dgAbs(contactMaterial->m_normal.DotProduct(contactMaterial->m_dir0.CrossProduct(contactMaterial->m_dir1)).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
		}
		dgAssert (contactMaterial->m_dir0.m_w == dgFloat32 (0.0f));
		dgAssert (contactMaterial->m_dir0.m_w == dgFloat32 (0.0f));
		dgAssert (contactMaterial->m_normal.m_w == dgFloat32 (0.0f));
		//contactMaterial->m_normal.m_w = dgFloat32 (0.0f);
		//contactMaterial->m_dir0.m_w = dgFloat32 (0.0f); 
		//contactMaterial->m_dir1.m_w = dgFloat32 (0.0f); 
	}

	if (count) {
		GlobalLock();
		for (dgInt32 i = 0; i < count; i ++) {
			list.Remove(nodes[i]);
		}
		GlobalUnlock();
	}

	contact->m_maxDOF = dgUnsigned32 (3 * contact->GetCount());
	if (material->m_processContactPoint) {
		material->m_processContactPoint(*contact, pair->m_timestep, threadIndex);
	}

/*
	if (maxImpulse > dgFloat32 (1.0f)) {
		maxImpulse /= (body0->m_invMass.m_w + body1->m_invMass.m_w) ;
		if ((maxImpulse > breakImpulse0) || (maxImpulse > breakImpulse1)) {
			GetLock(threadIndex);
			if (maxImpulse > breakImpulse0) {
				AddToBreakQueue (contact, body0, maxImpulse);
			}
			if (maxImpulse > breakImpulse1) {
				AddToBreakQueue (contact, body1, maxImpulse);
			}
			ReleaseLock();
		}
	}
*/
}

void dgWorld::ProcessContacts (dgBroadPhase::dgPair* const pair, dgInt32 threadIndex)
{
	dgAssert (pair->m_contact);
	dgAssert (pair->m_contact->m_body0);
	dgAssert (pair->m_contact->m_body1);
	dgAssert (pair->m_contact->m_body0 != pair->m_contact->m_body1);

	pair->m_contact->m_positAcc = dgVector::m_zero;
	pair->m_contact->m_rotationAcc = dgQuaternion();
	PopulateContacts (pair, threadIndex);
}

void dgWorld::ConvexContacts (dgBroadPhase::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;

	dgBody* const otherBody = constraint->m_body1;
	dgBody* const convexBody = constraint->m_body0;
	
	proxy.m_body1 = otherBody;
	proxy.m_body0 = convexBody;
	proxy.m_instance0 = proxy.m_body0->m_collision;
	proxy.m_instance1 = proxy.m_body1->m_collision;
	dgAssert (proxy.m_instance0->IsType (dgCollision::dgCollisionConvexShape_RTTI));
	if (proxy.m_instance1->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		dgAssert (convexBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
		dgAssert (otherBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
		pair->m_contactCount = CalculateConvexToConvexContacts (proxy);
	} else {
		dgAssert (constraint->m_body0->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
		dgAssert (convexBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
		pair->m_contactCount = CalculateConvexToNonConvexContacts (proxy);
	}
}

void dgWorld::CompoundContacts (dgBroadPhase::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;
	pair->m_contactCount = 0;

	dgCollisionInstance* const instance = constraint->m_body0->GetCollision();
	dgCollisionCompound* const compound = (dgCollisionCompound*) instance->GetChildShape();
	dgAssert (compound->IsType(dgCollision::dgCollisionCompound_RTTI));
	compound->CalculateContacts (pair, proxy);

	proxy.m_contactJoint->m_separationDistance = dgFloat32 (0.0f);
}

void dgWorld::SceneChildContacts (dgBroadPhase::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgAssert (pair->m_contact->GetBody1()->GetCollision()->IsType(dgCollision::dgCollisionScene_RTTI));
	dgContactPoint* const savedBuffer = proxy.m_contacts;

	proxy.m_maxContacts = ((DG_MAX_CONTATCS - pair->m_contactCount) > 32) ? 32 : DG_MAX_CONTATCS - pair->m_contactCount;
	proxy.m_contacts = &savedBuffer[pair->m_contactCount];

	if (proxy.m_instance1->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		pair->m_contactCount += CalculateConvexToConvexContacts (proxy);
	} else {
		pair->m_contactCount += CalculateConvexToNonConvexContacts (proxy);
	}

	proxy.m_contacts = savedBuffer;
	if (pair->m_contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
		pair->m_contactCount = PruneContacts(pair->m_contactCount, proxy.m_contacts, proxy.m_contactJoint->GetPruningTolerance(), 16);
	}
}

void dgWorld::SceneContacts (dgBroadPhase::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;
	pair->m_contactCount = 0;

	dgBody* const sceneBody = constraint->m_body1;
	dgBody* const otherBody = constraint->m_body0;

	dgCollisionInstance* const sceneInstance = sceneBody->GetCollision();
	dgCollisionInstance* const otherInstance = otherBody->GetCollision();
	dgAssert (sceneInstance->IsType(dgCollision::dgCollisionScene_RTTI));
	dgAssert (!otherInstance->IsType(dgCollision::dgCollisionScene_RTTI));
	if (otherInstance->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		proxy.m_body0 = otherBody;
		proxy.m_body1 = sceneBody;
		proxy.m_instance0 = otherBody->m_collision;
		proxy.m_instance1 = NULL;

		dgCollisionScene* const scene = (dgCollisionScene*)sceneInstance->GetChildShape();
		scene->CollidePair (pair, proxy);
	} else if (otherInstance->IsType (dgCollision::dgCollisionCompound_RTTI) & ~otherInstance->IsType (dgCollision::dgCollisionScene_RTTI)) {
		proxy.m_body0 = otherBody;
		proxy.m_body1 = sceneBody;
		proxy.m_instance0 = NULL;
		proxy.m_instance1 = NULL;

		dgCollisionScene* const scene = (dgCollisionScene*)sceneInstance->GetChildShape();
		scene->CollideCompoundPair (pair, proxy);
	} else {
		dgAssert (0);
	}
}


void dgWorld::CalculateContacts (dgBroadPhase::dgPair* const pair, dgInt32 threadIndex, bool ccdMode, bool intersectionTestOnly)
{
	dgContact* const contact = pair->m_contact;
	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
	const dgContactMaterial* const material = contact->m_material;
	dgCollisionParamProxy proxy(contact, pair->m_contactBuffer, threadIndex, ccdMode, intersectionTestOnly);

	pair->m_flipContacts = false;
	proxy.m_timestep = pair->m_timestep;
	proxy.m_maxContacts = DG_MAX_CONTATCS;
	proxy.m_skinThickness = material->m_skinThickness;

	if (body1->m_collision->IsType(dgCollision::dgCollisionScene_RTTI)) {
		dgAssert(contact->m_body1->GetInvMass().m_w == dgFloat32(0.0f));
		SceneContacts(pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		contact->SwapBodies();
		pair->m_flipContacts = -1;
		dgAssert(contact->m_body1->GetInvMass().m_w == dgFloat32(0.0f));
		SceneContacts (pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		CompoundContacts (pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		contact->SwapBodies();
		pair->m_flipContacts = -1;
		CompoundContacts (pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		ConvexContacts (pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		contact->SwapBodies();
		pair->m_flipContacts = -1;
		ConvexContacts (pair, proxy);
	}

	if (pair->m_contactCount > 1) {
		pair->m_contactCount = PruneContacts (pair->m_contactCount, pair->m_contactBuffer, contact->GetPruningTolerance(), 16);
	}
	pair->m_timestep = proxy.m_timestep;
}

dgFloat32 dgWorld::CalculateTimeToImpact (dgContact* const contact, dgFloat32 timestep, dgInt32 threadIndex, dgVector& p, dgVector& q, dgVector& normal, dgFloat32 dist) const
{
	dgBroadPhase::dgPair pair;

	dgInt32 isActive = contact->m_isActive;
	dgInt32 contactCount = contact->m_maxDOF;
	dgFloat32 separationDistance = contact->m_separationDistance;

	contact->m_maxDOF = 0;
	pair.m_contact = contact;
	pair.m_cacheIsValid = false;
	pair.m_contactBuffer = NULL;

	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
//	const dgContactMaterial* const material = contact->m_material;
	dgCollisionParamProxy proxy(contact, NULL, threadIndex, true, true);

	proxy.m_maxContacts = 0;
	proxy.m_timestep = timestep;
//	proxy.m_skinThickness = material->m_skinThickness;
	proxy.m_skinThickness = dist;

	if (body0->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		contact->SwapBodies();
		SceneContacts (&pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		SceneContacts (&pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		CompoundContacts (&pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		contact->SwapBodies();
		CompoundContacts (&pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		ConvexContacts (&pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		contact->SwapBodies();
		ConvexContacts (&pair, proxy);
	}

	if (contact->m_body0 == body0) {
		normal = proxy.m_normal;
		p = proxy.m_closestPointBody0;
		q = proxy.m_closestPointBody1;
	} else {
		contact->m_body0 = body0;
		contact->m_body1 = body1;
		//normal = proxy.m_normal.Scale(dgFloat32 (-1.0f));
		normal = proxy.m_normal * dgVector::m_negOne;
		p = proxy.m_closestPointBody1;
		q = proxy.m_closestPointBody0;
	}

	contact->m_maxDOF = contactCount;
	contact->m_isActive = isActive;
	contact->m_separationDistance = separationDistance;
	return proxy.m_timestep;
}


dgInt32 dgWorld::CollideContinue (
	const dgCollisionInstance* const collisionSrcA, const dgMatrix& matrixA, const dgVector& velocA, const dgVector& omegaA, 
	const dgCollisionInstance* const collisionSrcB, const dgMatrix& matrixB, const dgVector& velocB, const dgVector& omegaB, 
	dgFloat32& retTimeStep, dgTriplex* const points, dgTriplex* const normals, dgFloat32* const penetration, 
	dgInt64* const attibuteA, dgInt64* const attibuteB, dgInt32 maxContacts, dgInt32 threadIndex)
{
	dgKinematicBody collideBodyA;
	dgKinematicBody collideBodyB;
	dgCollisionInstance collisionA(*collisionSrcA, collisionSrcA->GetChildShape());
	dgCollisionInstance collisionB(*collisionSrcB, collisionSrcB->GetChildShape());

//	collisionA.SetCollisionMode(true);
//	collisionB.SetCollisionMode(true);

	dgContactPoint contacts[DG_MAX_CONTATCS];

	dgInt32 count = 0;
	maxContacts = dgMin (DG_MAX_CONTATCS, maxContacts);

	collideBodyA.m_world = this;
	collideBodyA.SetContinueCollisionMode(true); 
	collideBodyA.m_matrix = matrixA;
	collideBodyA.m_collision = &collisionA;
	collideBodyA.m_masterNode = NULL;
	collideBodyA.m_broadPhaseNode = NULL;
	collideBodyA.m_veloc = dgVector (velocA[0], velocA[1], velocA[2], dgFloat32 (0.0f));
	collideBodyA.m_omega = dgVector (omegaA[0], omegaA[1], omegaA[2], dgFloat32 (0.0f));
	collisionA.SetGlobalMatrix(collisionA.GetLocalMatrix() * matrixA);

	collideBodyB.m_world = this;
	collideBodyB.SetContinueCollisionMode(true); 
	collideBodyB.m_matrix = matrixB;
	collideBodyB.m_collision = &collisionB;
	collideBodyB.m_masterNode = NULL;
	collideBodyB.m_broadPhaseNode = NULL;
	collideBodyB.m_veloc = dgVector (velocB[0], velocB[1], velocB[2], dgFloat32 (0.0f));
	collideBodyB.m_omega = dgVector (omegaB[0], omegaB[1], omegaB[2], dgFloat32 (0.0f));
	collisionB.SetGlobalMatrix(collisionB.GetLocalMatrix() * matrixB);

	dgContactMaterial material;
	material.m_penetration = dgFloat32 (0.0f);

	dgContact contactJoint (this, &material, &collideBodyB, &collideBodyA);
//	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgBroadPhase::dgPair pair;
	pair.m_contact = &contactJoint;
	pair.m_contactBuffer = contacts;
	pair.m_timestep = retTimeStep;
	pair.m_contactCount = 0;
	pair.m_cacheIsValid = 0;
	CalculateContacts (&pair, threadIndex, true, maxContacts ? false : true);

	if (pair.m_timestep < retTimeStep) {
		retTimeStep = pair.m_timestep;
	}

	count = pair.m_contactCount;
	if (count) {
		if (count > maxContacts) {
			count = PruneContacts (count, contacts, contactJoint.GetPruningTolerance(), maxContacts);
		}

		if (pair.m_flipContacts) {
 			for (dgInt32 i = 0; i < count; i++) {
				dgVector step ((collideBodyA.m_veloc - collideBodyB.m_veloc).Scale (pair.m_timestep));
				points[i].m_x = contacts[i].m_point.m_x + step.m_x;
				points[i].m_y = contacts[i].m_point.m_y + step.m_y;
				points[i].m_z = contacts[i].m_point.m_z + step.m_z;
				normals[i].m_x = -contacts[i].m_normal.m_x;
				normals[i].m_y = -contacts[i].m_normal.m_y;
				normals[i].m_z = -contacts[i].m_normal.m_z;
				penetration[i] = contacts[i].m_penetration;
				attibuteA[i] = contacts[i].m_shapeId1;
				attibuteB[i] = contacts[i].m_shapeId0;
			}
		} else {
			for (dgInt32 i = 0; i < count; i ++) {
				points[i].m_x = contacts[i].m_point.m_x; 
				points[i].m_y = contacts[i].m_point.m_y; 
				points[i].m_z = contacts[i].m_point.m_z; 
				normals[i].m_x = contacts[i].m_normal.m_x;  
				normals[i].m_y = contacts[i].m_normal.m_y;  
				normals[i].m_z = contacts[i].m_normal.m_z;  
				penetration[i] = contacts[i].m_penetration; 
				attibuteA[i] = contacts[i].m_shapeId0;
				attibuteB[i] = contacts[i].m_shapeId1;
			}
		} 
	} 
	return count;
}

dgInt32 dgWorld::Collide (
	const dgCollisionInstance* const collisionSrcA, const dgMatrix& matrixA, 
	const dgCollisionInstance* const collisionSrcB, const dgMatrix& matrixB, 
	dgTriplex* const points, dgTriplex* const normals, dgFloat32* const penetration, 
	dgInt64* const attibuteA, dgInt64* const attibuteB, dgInt32 maxContacts, dgInt32 threadIndex)
{
	dgKinematicBody collideBodyA;
	dgKinematicBody collideBodyB;
	dgCollisionInstance collisionA(*collisionSrcA, collisionSrcA->GetChildShape());
	dgCollisionInstance collisionB(*collisionSrcB, collisionSrcB->GetChildShape());

	collisionA.SetCollisionMode(true);
	collisionB.SetCollisionMode(true);

	dgContactPoint contacts[DG_MAX_CONTATCS];

	dgInt32 count = 0;
	maxContacts = dgMin (DG_MAX_CONTATCS, maxContacts);
		
	collideBodyA.m_world = this;
	collideBodyA.SetContinueCollisionMode(false); 
	collideBodyA.m_matrix = matrixA;
	collideBodyA.m_collision = &collisionA;
	collideBodyA.UpdateCollisionMatrix(dgFloat32 (0.0f), 0);

	collideBodyB.m_world = this;
	collideBodyB.SetContinueCollisionMode(false); 
	collideBodyB.m_matrix = matrixB;
	collideBodyB.m_collision = &collisionB;
	collideBodyB.UpdateCollisionMatrix(dgFloat32 (0.0f), 0);

	dgContactMaterial material; 
	material.m_penetration = dgFloat32 (0.0f);

	dgContact contactJoint (this, &material, &collideBodyB, &collideBodyA);
//	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgBroadPhase::dgPair pair;
	pair.m_contactCount = 0;
	pair.m_contact = &contactJoint;
	pair.m_contactBuffer = contacts; 
	pair.m_timestep = dgFloat32 (0.0f);
	pair.m_cacheIsValid = 0;
	CalculateContacts (&pair, threadIndex, false, false);

	count = pair.m_contactCount;

	dgFloat32 swapContactScale = (contactJoint.GetBody0() != &collideBodyA) ? dgFloat32 (-1.0f) : dgFloat32 (1.0f);
	for (dgInt32 i = 0; i < count; i ++) {
		points[i].m_x = contacts[i].m_point.m_x; 
		points[i].m_y = contacts[i].m_point.m_y; 
		points[i].m_z = contacts[i].m_point.m_z; 
		normals[i].m_x = contacts[i].m_normal.m_x * swapContactScale;  
		normals[i].m_y = contacts[i].m_normal.m_y * swapContactScale;  
		normals[i].m_z = contacts[i].m_normal.m_z * swapContactScale;  
		penetration[i] = contacts[i].m_penetration; 
		attibuteA[i] = contacts[i].m_shapeId0; 
		attibuteB[i] = contacts[i].m_shapeId1; 
	}

	return count;
}




// *************************************************************************
//
// 
// *************************************************************************
 
dgInt32 dgWorld::ClosestPoint (dgCollisionParamProxy& proxy) const	
{
	dgCollisionInstance* const collision0 = proxy.m_instance0;
	dgCollisionInstance* const collision1 = proxy.m_instance1;

	if (!(collision0->GetConvexVertexCount() && collision1->GetConvexVertexCount())) {
		return 0;
	}

	dgAssert(collision0->GetCollisionPrimityType() < m_nullCollision);
	dgAssert(collision1->GetCollisionPrimityType() < m_nullCollision);
	dgAssert(proxy.m_instance1->IsType(dgCollision::dgCollisionConvexShape_RTTI));

	dgContact* const contactJoint = proxy.m_contactJoint;
	dgAssert(contactJoint);
	contactJoint->m_closestDistance = dgFloat32(1.0e10f);
	contactJoint->m_separationDistance = dgFloat32(0.0f);

	dgCollisionInstance instance0(*collision0, collision0->m_childShape);
	dgCollisionInstance instance1(*collision1, collision1->m_childShape);
	proxy.m_instance0 = &instance0;
	proxy.m_instance1 = &instance1;

	dgVector origin(instance0.m_globalMatrix.m_posit & dgVector::m_triplexMask);
	instance0.m_globalMatrix.m_posit = dgVector::m_wOne;
	instance1.m_globalMatrix.m_posit -= origin;

	contactJoint->m_separtingVector = collision0->GetGlobalMatrix().m_up;

	dgContactSolver contactSolver(&proxy);
	bool retVal = contactSolver.CalculateClosestPoints();
	if (retVal) {
		proxy.m_closestPointBody0 = contactSolver.GetPoint0() + origin;
		proxy.m_closestPointBody1 = contactSolver.GetPoint1() + origin;
		proxy.m_normal = contactSolver.GetNormal().Scale(-1.0f);

		dgContactPoint* const contactOut = proxy.m_contacts;
		contactOut[0].m_normal = proxy.m_normal;
		contactOut[0].m_point = proxy.m_closestPointBody0;

		contactOut[1].m_normal = contactSolver.GetNormal();
		contactOut[1].m_point = proxy.m_closestPointBody1;

		contactJoint->m_closestDistance = (contactOut[1].m_point - contactOut[0].m_point).DotProduct(contactSolver.GetNormal()).GetScalar();
		contactJoint->m_separationDistance = dgFloat32(0.0f);

		instance0.m_material.m_userData = NULL;
		instance1.m_material.m_userData = NULL;
		proxy.m_instance0 = collision0;
		proxy.m_instance1 = collision1;
		retVal = contactJoint->m_closestDistance >= dgFloat32(0.0f);
	}
	return retVal ? 1 : 0;
}

dgInt32 dgWorld::CalculateUserContacts(dgCollisionParamProxy& proxy) const
{
	dgContactMaterial::dgUserContactPoint buffer[16];
	dgContact* const contactJoint = proxy.m_contactJoint;
	int count = contactJoint->m_material->m_contactGeneration(*contactJoint->m_material, *proxy.m_body0, proxy.m_instance0, *proxy.m_body1, proxy.m_instance1, buffer, sizeof (buffer) / sizeof (buffer[0]), proxy.m_threadIndex);
	if (count) {
		proxy.m_contactJoint->m_isActive = 1;
		dgContactPoint* const contactOut = proxy.m_contacts;
		for (dgInt32 i = 0; i < count; i++) {
			dgAssert (buffer[i].m_normal.m_w == dgFloat32 (0.0f));
			dgAssert(dgAbs(buffer[i].m_normal.DotProduct(buffer[i].m_normal).GetScalar() - dgFloat32(1.0f)) < dgFloat32(1.0e-4f));
			contactOut[i].m_point = buffer[i].m_point;
			contactOut[i].m_normal = buffer[i].m_normal;
			contactOut[i].m_penetration = buffer[i].m_penetration;
			contactOut[i].m_shapeId0 = buffer[i].m_shapeId0;
			contactOut[i].m_shapeId1 = buffer[i].m_shapeId1;
			contactOut[i].m_body0 = proxy.m_body0;
			contactOut[i].m_body1 = proxy.m_body1;
			contactOut[i].m_collision0 = proxy.m_instance0;
			contactOut[i].m_collision1 = proxy.m_instance1;
		}
	}
	return count;
}
 

dgInt32 dgWorld::CalculateConvexToConvexContacts(dgCollisionParamProxy& proxy) const
{
	dgInt32 count = 0;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgAssert(contactJoint);

	dgCollisionInstance* const collision0 = proxy.m_instance0;
	dgCollisionInstance* const collision1 = proxy.m_instance1;
	dgAssert(collision0->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert(collision1->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	contactJoint->m_closestDistance = dgFloat32(1.0e10f);
	contactJoint->m_separationDistance = dgFloat32(0.0f);

//	if (!(collision0->GetConvexVertexCount() && collision1->GetConvexVertexCount() && proxy.m_instance0->GetCollisionMode() && proxy.m_instance1->GetCollisionMode())) {
	if (!(collision0->GetConvexVertexCount() && collision1->GetConvexVertexCount())) {
		return count;
	}

	dgAssert(collision0->GetCollisionPrimityType() != m_nullCollision);
	dgAssert(collision1->GetCollisionPrimityType() != m_nullCollision);
	dgAssert(proxy.m_instance1->IsType(dgCollision::dgCollisionConvexShape_RTTI));

	if (!contactJoint->m_material->m_contactGeneration) {
		dgCollisionInstance instance0(*collision0, collision0->m_childShape);
		dgCollisionInstance instance1(*collision1, collision1->m_childShape);

		proxy.m_instance0 = &instance0;
		proxy.m_instance1 = &instance1;

		dgVector origin(instance0.m_globalMatrix.m_posit & dgVector::m_triplexMask);
		instance0.m_globalMatrix.m_posit = dgVector::m_wOne;
		instance1.m_globalMatrix.m_posit -= origin;

		if (instance0.GetCollisionPrimityType() == instance1.GetCollisionPrimityType()) {
			switch (instance0.GetCollisionPrimityType())
			{
				case m_sphereCollision:
				{
					dgVector diff(instance1.GetGlobalMatrix().m_posit - instance0.GetGlobalMatrix().m_posit);
					dgFloat32 mag2 = diff.DotProduct(diff).GetScalar();
					if (mag2 < dgFloat32(1.0e-6f)) {
						dgMatrix tmp(instance1.GetGlobalMatrix());
						tmp.m_posit.m_x += dgFloat32(1.0e-3f);
						instance1.SetGlobalMatrix(tmp);
					}
					break;
				}
				case m_capsuleCollision:
				{
					dgMatrix diff(instance1.GetGlobalMatrix() * instance0.GetGlobalMatrix().Inverse());
					if (dgAbs(diff[0][0]) > dgFloat32(0.9999f)) {
						if (dgAbs(diff.m_posit.m_x) < dgFloat32(1.0e-3f)) {
							diff.m_posit.m_y += dgFloat32(1.0e-3f);
							instance1.SetGlobalMatrix(diff * instance0.GetGlobalMatrix());
						}
					}
					break;
				}
				case m_chamferCylinderCollision:
				{
					dgMatrix diff(instance1.GetGlobalMatrix() * instance0.GetGlobalMatrix().Inverse());
					if (dgAbs(diff[0][0]) > dgFloat32(0.9999f)) {
						if (dgAbs(diff.m_posit.m_x) < dgFloat32(1.0e-3f)) {
							diff.m_posit.m_x += dgFloat32(1.0e-3f);
							instance1.SetGlobalMatrix(diff * instance0.GetGlobalMatrix());
						}
					}
					break;
				}
				default:;
			}
		}
		if (contactJoint->m_isNewContact) {
			contactJoint->m_isNewContact = false;
			dgVector v((proxy.m_instance0->m_globalMatrix.m_posit - proxy.m_instance1->m_globalMatrix.m_posit) & dgVector::m_triplexMask);
			dgFloat32 mag2 = v.DotProduct(v).m_x;
			if (mag2 > dgFloat32(0.0f)) {
				contactJoint->m_separtingVector = v.Scale(dgRsqrt(mag2));
			} else {
				contactJoint->m_separtingVector = proxy.m_instance0->m_globalMatrix.m_up;
			}
		}

		dgContactSolver contactSolver(&proxy);
		if (proxy.m_continueCollision) {
			count = contactSolver.CalculateConvexCastContacts();
		} else {
			count = contactSolver.CalculateConvexToConvexContacts();
		}

		proxy.m_closestPointBody0 += origin;
		proxy.m_closestPointBody1 += origin;
		dgContactPoint* const contactOut = proxy.m_contacts;
		for (dgInt32 i = 0; i < count; i++) {
			contactOut[i].m_point += origin;
			contactOut[i].m_body0 = proxy.m_body0;
			contactOut[i].m_body1 = proxy.m_body1;
			contactOut[i].m_collision0 = collision0;
			contactOut[i].m_collision1 = collision1;
			contactOut[i].m_shapeId0 = collision0->GetUserDataID();
			contactOut[i].m_shapeId1 = collision1->GetUserDataID();
		}

		instance0.m_material.m_userData = NULL;
		instance1.m_material.m_userData = NULL;
		proxy.m_instance0 = collision0;
		proxy.m_instance1 = collision1;

	} else {
		count = CalculateUserContacts(proxy);
		dgContactPoint* const contactOut = proxy.m_contacts;
		for (dgInt32 i = 0; i < count; i++) {
			contactOut[i].m_body0 = proxy.m_body0;
			contactOut[i].m_body1 = proxy.m_body1;
			contactOut[i].m_collision0 = collision0;
			contactOut[i].m_collision1 = collision1;
			contactOut[i].m_shapeId0 = collision0->GetUserDataID();
			contactOut[i].m_shapeId1 = collision1->GetUserDataID();
		}
	}

	return count;
}

dgInt32 dgWorld::CalculateConvexToNonConvexContacts(dgCollisionParamProxy& proxy) const
{
	dgInt32 count = 0;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgAssert(contactJoint);

	dgCollisionInstance* const collision0 = proxy.m_instance0;
	dgCollisionInstance* const collision1 = proxy.m_instance1;
	dgAssert(collision1->IsType(dgCollision::dgCollisionMesh_RTTI));
	dgAssert(collision0->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	contactJoint->m_closestDistance = dgFloat32(1.0e10f);
	if (!collision0->GetConvexVertexCount()) {
		return count;
	}

	dgFloat32 separationDistance = dgFloat32 (0.0f);
	if (!contactJoint->m_material->m_contactGeneration) {
		dgCollisionInstance instance0(*collision0, collision0->m_childShape);
		dgCollisionInstance instance1(*collision1, collision1->m_childShape);
		proxy.m_instance0 = &instance0;
		proxy.m_instance1 = &instance1;

		dgVector origin(instance0.m_globalMatrix.m_posit & dgVector::m_triplexMask);
		instance0.m_globalMatrix.m_posit = dgVector::m_wOne;
		instance1.m_globalMatrix.m_posit -= origin;

		dgAssert(proxy.m_timestep <= dgFloat32(2.0f));
		dgAssert(proxy.m_timestep >= dgFloat32(0.0f));

		dgPolygonMeshDesc data(proxy, NULL);
		if (proxy.m_continueCollision) {
			data.m_doContinuesCollisionTest = true;

			const dgVector& hullVeloc = data.m_objBody->m_veloc;
			const dgVector& hullOmega = data.m_objBody->m_omega;

			dgFloat32 baseLinearSpeed = dgSqrt(hullVeloc.DotProduct(hullVeloc).GetScalar());
			if (baseLinearSpeed > dgFloat32(1.0e-6f)) {
				const dgFloat32 minRadius = instance0.GetBoxMinRadius();
				const dgFloat32 maxRadius = instance0.GetBoxMaxRadius();
				dgFloat32 maxAngularSpeed = dgSqrt(hullOmega.DotProduct(hullOmega).GetScalar());
				dgFloat32 angularSpeedBound = maxAngularSpeed * (maxRadius - minRadius);

				dgFloat32 upperBoundSpeed = baseLinearSpeed + dgSqrt(angularSpeedBound);
				dgVector upperBoundVeloc(hullVeloc.Scale(proxy.m_timestep * upperBoundSpeed / baseLinearSpeed));
				data.SetDistanceTravel(upperBoundVeloc);
			}
		}

		dgCollisionMesh* const polysoup = (dgCollisionMesh *)data.m_polySoupInstance->GetChildShape();
		polysoup->GetCollidingFaces(&data);

		if (data.m_faceCount) {
			proxy.m_polyMeshData = &data;

			if (proxy.m_continueCollision) {
				count = CalculateConvexToNonConvexContactsContinue(proxy);
			} else {
				count = CalculatePolySoupToHullContactsDescrete(proxy);
			}

			if (count > 0) {
				proxy.m_contactJoint->m_isActive = 1;
			}
		}

		proxy.m_closestPointBody0 += origin;
		proxy.m_closestPointBody1 += origin;
		separationDistance = data.GetSeparetionDistance();
		dgContactPoint* const contactOut = proxy.m_contacts;
		for (dgInt32 i = 0; i < count; i++) {
			contactOut[i].m_point += origin;
			contactOut[i].m_body0 = proxy.m_body0;
			contactOut[i].m_body1 = proxy.m_body1;
			contactOut[i].m_collision0 = collision0;
			contactOut[i].m_collision1 = collision1;
			contactOut[i].m_shapeId0 = collision0->GetUserDataID();
			//contactOut[i].m_shapeId1 = collision1->GetUserDataID();
		}

		instance0.m_material.m_userData = NULL;
		instance1.m_material.m_userData = NULL;
		proxy.m_instance0 = collision0;
		proxy.m_instance1 = collision1;
	} else {
		count = CalculateUserContacts(proxy);

		dgContactPoint* const contactOut = proxy.m_contacts;
		for (dgInt32 i = 0; i < count; i++) {
			contactOut[i].m_body0 = proxy.m_body0;
			contactOut[i].m_body1 = proxy.m_body1;
			contactOut[i].m_collision0 = collision0;
			contactOut[i].m_collision1 = collision1;
			contactOut[i].m_shapeId0 = collision0->GetUserDataID();
			//contactOut[i].m_shapeId1 = collision1->GetUserDataID();
		}
	}

	contactJoint->m_separationDistance = separationDistance;
	return count;
}

dgInt32 dgWorld::CalculatePolySoupToHullContactsDescrete (dgCollisionParamProxy& proxy) const
{
	dgAssert (proxy.m_instance1->IsType (dgCollision::dgCollisionMesh_RTTI));
	dgAssert (proxy.m_instance0->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	dgCollisionInstance* const polySoupInstance = proxy.m_instance1;
	dgPolygonMeshDesc& data = *proxy.m_polyMeshData;

	dgAssert (data.m_faceCount); 

	dgCollisionConvexPolygon polygon (m_allocator);
	dgCollisionInstance polyInstance (*polySoupInstance, &polygon);
	polyInstance.SetScale(dgVector (dgFloat32 (1.0f)));
	polyInstance.m_localMatrix = dgGetIdentityMatrix();
	polyInstance.m_globalMatrix = dgGetIdentityMatrix();

	proxy.m_instance1 = &polyInstance;
	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

	dgInt32 count = 0;
	dgInt32 maxContacts = proxy.m_maxContacts;
//	dgInt32 maxReduceLimit = maxContacts >> 2;
	dgInt32 maxReduceLimit = maxContacts - 16;
	dgInt32 countleft = maxContacts;

	const dgVector& polygonInstanceScale = polySoupInstance->GetScale();
	const dgMatrix polySoupGlobalMatrix = polySoupInstance->m_globalMatrix;
	const dgMatrix polySoupGlobalAligmentMatrix = polySoupInstance->m_aligmentMatrix;

	dgMatrix polySoupScaledMatrix (polySoupGlobalAligmentMatrix[0] * polygonInstanceScale, 
								   polySoupGlobalAligmentMatrix[1] * polygonInstanceScale,
								   polySoupGlobalAligmentMatrix[2] * polygonInstanceScale, 
								   polySoupGlobalAligmentMatrix[3]);
	polySoupScaledMatrix = polySoupScaledMatrix * polySoupGlobalMatrix;

	dgAssert (proxy.m_contactJoint);
	dgVector separatingVector (proxy.m_instance0->m_globalMatrix.m_up);

	const dgInt32 stride = polygon.m_stride;
	const dgFloat32* const vertex = polygon.m_vertex;
	dgAssert (polyInstance.m_scaleType == dgCollisionInstance::m_unit);
	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	dgContactPoint* const contactOut = proxy.m_contacts;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;
	data.SortFaceArray();

	for (dgInt32 i = data.m_faceCount - 1; (i >= 0) && (count < 32); i --) {
		dgInt32 address = data.m_faceIndexStart[i];
		const dgInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = data.m_faceIndexCount[i];
		polygon.m_paddedCount = polygon.m_count;
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray (localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId (localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize (localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex (localIndexArray, polygon.m_count);
		polygon.m_normal = polygon.CalculateGlobalNormal (polySoupInstance, dgVector (&vertex[polygon.m_faceNormalIndex * stride]) & dgVector::m_triplexMask);
		dgAssert (polygon.m_normal.m_w == dgFloat32 (0.0f));
		for (dgInt32 j = 0; j < polygon.m_count; j++) {
			polygon.m_localPoly[j] = polySoupScaledMatrix.TransformVector(dgVector(&vertex[localIndexArray[j] * stride]) & dgVector::m_triplexMask);
		}
		contactJoint->m_separtingVector = separatingVector;
		proxy.m_maxContacts = countleft;
		proxy.m_contacts = &contactOut[count];
		dgInt32 count1 = polygon.CalculateContactToConvexHullDescrete (this, polySoupInstance, proxy);
		closestDist = dgMin(closestDist, contactJoint->m_closestDistance);

		if (count1 > 0) {
			count += count1;
			countleft -= count1;
			dgAssert (countleft >= 0); 
			if (count >= maxReduceLimit) {
				count = PruneContacts(count, contactOut, dgFloat32 (1.0e-2f), 16);
				countleft = maxContacts - count;
				dgAssert (countleft >= 0); 
				proxy.m_maxContacts = countleft;
			}
		} else if (count1 == -1) {
			count = -1;
			break;
		}
	}

	contactJoint->m_closestDistance = closestDist;

	bool contactsValid = true;

	for (dgInt32 i = 0; (i < count) && contactsValid; i++) {
		const dgVector& normal = contactOut[i].m_normal;
		for (dgInt32 j = i + 1; (j < count) && contactsValid; j++) {
			const dgFloat32 project = (normal.DotProduct(contactOut[j].m_normal)).GetScalar();
			contactsValid = contactsValid && (project > dgFloat32(-0.1f));
		}
	}

	if (!contactsValid) {
		dgCollisionContactCloud contactCloud (GetAllocator(), count, contactOut);
		dgCollisionInstance cloudInstance (*polySoupInstance, &contactCloud);
		cloudInstance.m_globalMatrix = dgGetIdentityMatrix();
		cloudInstance.SetScale(dgVector::m_one);
		bool saveintersectionTestOnly = proxy.m_intersectionTestOnly;
		proxy.m_instance1 = &cloudInstance;
		proxy.m_intersectionTestOnly = true;
		dgContactSolver contactSolver (&proxy);
		contactSolver.CalculateConvexToConvexContacts();
		dgVector normal (contactSolver.GetNormal() * dgVector::m_negOne);
		for (dgInt32 i = 0; i < count; i ++) {
			contactOut[i].m_normal = normal;
		}

		proxy.m_intersectionTestOnly = saveintersectionTestOnly;
		cloudInstance.m_material.m_userData = NULL;
	}

 	proxy.m_contacts = contactOut;

	// restore the pointer
	polyInstance.m_material.m_userData = NULL;
	proxy.m_instance1 = polySoupInstance;
	return count;
}

dgInt32 dgWorld::CalculateConvexToNonConvexContactsContinue(dgCollisionParamProxy& proxy) const
{
	dgAssert(proxy.m_instance1->IsType(dgCollision::dgCollisionMesh_RTTI));
	dgAssert(proxy.m_instance0->IsType(dgCollision::dgCollisionConvexShape_RTTI));

	dgCollisionInstance* const polySoupInstance = proxy.m_instance1;
	dgPolygonMeshDesc& data = *proxy.m_polyMeshData;

	dgAssert(data.m_faceCount);

	dgCollisionConvexPolygon polygon(m_allocator);
	dgCollisionInstance polyInstance(*polySoupInstance, &polygon);
	polyInstance.SetScale(dgVector(dgFloat32(1.0f)));
	polyInstance.m_localMatrix = dgGetIdentityMatrix();
	polyInstance.m_globalMatrix = dgGetIdentityMatrix();

	proxy.m_instance1 = &polyInstance;
	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32(data.m_vertexStrideInBytes / sizeof (dgFloat32));

	dgInt32 count = 0;
	dgInt32 maxContacts = proxy.m_maxContacts;
	dgInt32 maxReduceLimit = maxContacts >> 2;
	dgInt32 countleft = maxContacts;

	const dgVector& polygonInstanceScale = polySoupInstance->GetScale();
	const dgMatrix polySoupGlobalMatrix = polySoupInstance->m_globalMatrix;
	const dgMatrix polySoupGlobalAligmentMatrix = polySoupInstance->m_aligmentMatrix;

	dgMatrix polySoupScaledMatrix (polySoupGlobalAligmentMatrix[0] * polygonInstanceScale, 
								   polySoupGlobalAligmentMatrix[1] * polygonInstanceScale,
								   polySoupGlobalAligmentMatrix[2] * polygonInstanceScale, 
								   polySoupGlobalAligmentMatrix[3]);
	polySoupScaledMatrix = polySoupScaledMatrix * polySoupGlobalMatrix;

	dgAssert (proxy.m_contactJoint);
	dgVector separatingVector (proxy.m_instance0->m_globalMatrix.m_up);

	const dgInt32 stride = polygon.m_stride;
	const dgFloat32* const vertex = polygon.m_vertex;
	dgAssert(polyInstance.m_scaleType == dgCollisionInstance::m_unit);

	dgContactPoint* const contactOut = proxy.m_contacts;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;
	data.SortFaceArray();

	dgVector n(dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f));
	dgVector p(dgFloat32(0.0f));
	dgVector q(dgFloat32(0.0f));
	dgFloat32 closestDist = dgFloat32(1.0e10f);
	dgFloat32 minTimeStep = proxy.m_timestep;

	dgFloat32 timeNormalizer = proxy.m_timestep;
	dgFloat32 epsilon = dgFloat32(-1.0e-3f) * proxy.m_timestep;

	for (dgInt32 i = 0; (i < data.m_faceCount) && (proxy.m_timestep >= (data.m_hitDistance[i] * timeNormalizer)); i++) {
		dgInt32 address = data.m_faceIndexStart[i];
		const dgInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = data.m_faceIndexCount[i];
		polygon.m_paddedCount = polygon.m_count;
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray(localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId(localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize(localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex(localIndexArray, polygon.m_count);
		polygon.m_normal = polygon.CalculateGlobalNormal(polySoupInstance, dgVector(&vertex[polygon.m_faceNormalIndex * stride]) & dgVector::m_triplexMask);
		dgAssert(polygon.m_normal.m_w == dgFloat32(0.0f));
		for (dgInt32 j = 0; j < polygon.m_count; j++) {
			polygon.m_localPoly[j] = polySoupScaledMatrix.TransformVector(dgVector(&vertex[localIndexArray[j] * stride]) & dgVector::m_triplexMask);
		}
		contactJoint->m_separtingVector = separatingVector;
		proxy.m_timestep = minTimeStep;
		proxy.m_maxContacts = countleft;
		proxy.m_contacts = &contactOut[count];

		dgInt32 count1 = polygon.CalculateContactToConvexHullContinue(this, polySoupInstance, proxy);

		if (count1 > 0) {
			dgFloat32 error = proxy.m_timestep - minTimeStep;
			if (error < epsilon) {
				count = 0;
				countleft = maxContacts;
				for (dgInt32 j = 0; j < count1; j++) {
					contactOut[j] = proxy.m_contacts[j];
				}
			}
			count += count1;
			countleft -= count1;
			dgAssert(countleft >= 0);
			if (count >= maxReduceLimit) {
				count = PruneContacts(count, contactOut, dgFloat32(1.0e-2f), 16);
				countleft = maxContacts - count;
				dgAssert(countleft >= 0);
			}
		}

		closestDist = dgMin(closestDist, contactJoint->m_closestDistance);
		if (proxy.m_timestep <= minTimeStep) {
			minTimeStep = proxy.m_timestep;
			n = proxy.m_normal;
			p = proxy.m_closestPointBody0;
			q = proxy.m_closestPointBody1;
		}
	}

	polyInstance.m_material.m_userData = NULL;
	proxy.m_contacts = contactOut;

	proxy.m_normal = n;
	proxy.m_closestPointBody0 = p;
	proxy.m_closestPointBody1 = q;
	proxy.m_timestep = minTimeStep;
	return count;
}

