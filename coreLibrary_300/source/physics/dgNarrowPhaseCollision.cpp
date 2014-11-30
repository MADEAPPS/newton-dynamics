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
#include "dgCollision.h"
#include "dgMeshEffect.h"
#include "dgDynamicBody.h"
#include "dgCollisionBVH.h"
#include "dgCollisionBox.h"
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
#include "dgDeformableContact.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionConvexHull.h"
#include "dgCollisionHeightField.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableMesh.h"
#include "dgCollisionTaperedCapsule.h"
#include "dgCollisionTaperedCylinder.h"
#include "dgCollisionChamferCylinder.h"
#include "dgCollisionCompoundFractured.h"
#include "dgCollisionDeformableSolidMesh.h"
#include "dgCollisionDeformableClothPatch.h"

#define DG_CONTACT_TRANSLATION_ERROR (dgFloat32 (1.0e-3f))
#define DG_CONTACT_ANGULAR_ERROR (dgFloat32 (0.25f * 3.141592f / 180.0f))
dgVector dgWorld::m_angularContactError2 (DG_CONTACT_ANGULAR_ERROR * DG_CONTACT_ANGULAR_ERROR);
dgVector dgWorld::m_linearContactError2 (DG_CONTACT_TRANSLATION_ERROR * DG_CONTACT_TRANSLATION_ERROR);

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
		dgCollision* const collision = new  (m_allocator) dgCollisionSphere (m_allocator, crc, dgAbsf(radii));
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}


dgCollisionInstance* dgWorld::CreateBox(dgFloat32 dx, dgFloat32 dy, dgFloat32 dz, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionBox::CalculateSignature(dx, dy, dz);
//	dgUnsigned32 pinNumber = dgCollisionBox::CalculateSignature(dy, dz, dx);
	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);

/*
	if (node && (node->GetInfo().m_pinNumber != pinNumber)) {
		// shape was found but it is a CRC collision simple single out this shape as a unique entry in the cache
		dgTrace (("we have a CRC collision simple single out this shape as a unique entry in the cache\n"));
		dgCollision* const collision = new  (m_allocator) dgCollisionBox (m_allocator, pinNumber, dx, dy, dz);
		node = dgBodyCollisionList::Insert (CollisionKeyPair(collision, pinNumber), pinNumber);
	}
*/
	if (!node) {
		dgCollision* const collision = new  (m_allocator) dgCollisionBox (m_allocator, crc, dx, dy, dz);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}


dgCollisionInstance* dgWorld::CreateCapsule (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	//dgUnsigned32 crc = dgCollisionCapsule::CalculateSignature(dgAbsf (radius), dgMax (dgFloat32(0.01f), dgAbsf (height * dgFloat32 (0.5f)) - dgAbsf (radius)));  
	//dgUnsigned32 pinNumber = dgCollisionCapsule::CalculateSignature(dgMax (dgFloat32(0.01f), dgAbsf (height * dgFloat32 (0.5f)) - dgAbsf (radius)), dgAbsf (radius));  
	dgUnsigned32 crc = dgCollisionCapsule::CalculateSignature (dgAbsf (radius), dgAbsf (height * dgFloat32 (0.5f)));  

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);

/*
	if (node && (node->GetInfo().m_pinNumber != pinNumber)) {
		// shape was found but it is a CRC collision simple single out this shape as a unique entry in the cache
		dgTrace (("we have a CRC collision simple single out this shape as a unique entry in the cache\n"));
		dgCollision* const collision = new  (m_allocator) dgCollisionCapsule (m_allocator, pinNumber, radius, height);
		node = dgBodyCollisionList::Insert (CollisionKeyPair(collision, pinNumber), pinNumber);
	}
*/
	if (!node) {
		dgCollision* const collision = new  (m_allocator) dgCollisionCapsule (m_allocator, crc, radius, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}

	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateCylinder (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionCylinder::CalculateSignature(dgAbsf (radius), dgAbsf (height) * dgFloat32 (0.5f));
	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* const collision = new (m_allocator) dgCollisionCylinder (m_allocator, crc, radius, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}


dgCollisionInstance* dgWorld::CreateTaperedCapsule (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionTaperedCapsule::CalculateSignature(dgAbsf (radio0), dgAbsf (radio1), dgAbsf (height) * dgFloat32 (0.5f));

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* collision = new  (m_allocator) dgCollisionTaperedCapsule (m_allocator, crc, radio0, radio1, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}


dgCollisionInstance* dgWorld::CreateTaperedCylinder (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionTaperedCylinder::CalculateSignature(dgAbsf (radio0), dgAbsf (radio1), dgAbsf (height) * dgFloat32 (0.5f));

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* collision = new  (m_allocator) dgCollisionTaperedCylinder (m_allocator, crc, radio0, radio1, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}


dgCollisionInstance* dgWorld::CreateChamferCylinder (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionChamferCylinder::CalculateSignature(dgAbsf (radius), dgAbsf (height) * dgFloat32 (0.5f));

	dgBodyCollisionList::dgTreeNode* node = dgBodyCollisionList::Find (crc);
	if (!node) {
		dgCollision* collision = new  (m_allocator) dgCollisionChamferCylinder (m_allocator, crc, radius, height);
		node = dgBodyCollisionList::Insert (collision, crc);
	}
	return CreateInstance (node->GetInfo(), shapeID, offsetMatrix);
}

dgCollisionInstance* dgWorld::CreateCone (dgFloat32 radius, dgFloat32 height, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgUnsigned32 crc = dgCollisionCone::CalculateSignature (dgAbsf (radius), dgAbsf (height) * dgFloat32 (0.5f));
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

/*
	if (node && (node->GetInfo().m_pinNumber != pinNumber)) {
		// shape was found but it is a CRC collision simple single out this shape as a unique entry in the cache
		dgTrace (("we have a CRC collision simple single out this shape as a unique entry in the cache\n"));
		dgCollisionConvexHull* const collision = new (m_allocator) dgCollisionConvexHull (m_allocator, pinNumber, count, strideInBytes, tolerance, vertexArray);
		if (collision->GetConvexVertexCount()) {
			node = dgBodyCollisionList::Insert (CollisionKeyPair(collision, pinNumber), pinNumber);
		} else {
			//most likely the point cloud is a plane or a line
			//could not make the shape destroy the shell and return NULL 
			//note this is the only newton shape that can return NULL;
			collision->Release();
			return NULL;
		}
	}
*/

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


dgCollisionInstance* dgWorld::CreateClothPatchMesh (dgMeshEffect* const mesh, dgInt32 shapeID, const dgClothPatchMaterial& structuralMaterial, const dgClothPatchMaterial& bendMaterial)
{
	dgAssert (m_allocator == mesh->GetAllocator());
	dgCollision* const collision = new (m_allocator) dgCollisionDeformableClothPatch (this, mesh, structuralMaterial, bendMaterial);
	dgCollisionInstance* const instance = CreateInstance (collision, shapeID, dgGetIdentityMatrix()); 
	collision->Release();
	return instance;
}

dgCollisionInstance* dgWorld::CreateDeformableMesh (dgMeshEffect* const mesh, dgInt32 shapeID)
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
	const void* const elevationMap, const dgInt8* const atributeMap, dgFloat32 verticalScale, dgFloat32 horizontalScale)
{
	dgCollision* const collision = new  (m_allocator) dgCollisionHeightField (this, width, height, contructionMode, elevationMap, 
																			  elevationDataType	? dgCollisionHeightField::m_unsigned16Bit : dgCollisionHeightField::m_float32Bit,	
																			  verticalScale, atributeMap, horizontalScale);
	dgCollisionInstance* const instance = CreateInstance (collision, 0, dgGetIdentityMatrix()); 
	collision->Release();
	return instance;
}



dgCollisionInstance* dgWorld::CreateInstance (const dgCollision* const child, dgInt32 shapeID, const dgMatrix& offsetMatrix)
{
	dgAssert (dgAbsf (offsetMatrix[0] % offsetMatrix[0] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));
	dgAssert (dgAbsf (offsetMatrix[1] % offsetMatrix[1] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));
	dgAssert (dgAbsf (offsetMatrix[2] % offsetMatrix[2] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));
	dgAssert (dgAbsf ((offsetMatrix[0] * offsetMatrix[1]) % offsetMatrix[2] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f));

	dgAssert (offsetMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (offsetMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (offsetMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (offsetMatrix[3][3] == dgFloat32 (1.0f));
	dgCollisionInstance* const instance = new  (m_allocator) dgCollisionInstance (this, child, shapeID, offsetMatrix);
	return instance;
}

void dgWorld::SerializeCollision(dgCollisionInstance* const shape, dgSerialize serialization, void* const userData) const
{
	shape->Serialize(serialization, userData);
	dgSerializeMarker(serialization, userData);
}

dgCollisionInstance* dgWorld::CreateCollisionFromSerialization (dgDeserialize deserialization, void* const userData)
{
	dgCollisionInstance* const instance = new  (m_allocator) dgCollisionInstance (this, deserialization, userData);
	dgDeserializeMarker (deserialization, userData);
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
	pairMaterial.m_contactPoint = NULL;
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

	dgContact contactJoint (this, &material);
	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgCollisionParamProxy proxy(&contactJoint, contacts, threadIndex, false, false);

	proxy.m_referenceBody = &collideBodyA;
	proxy.m_referenceCollision = collideBodyA.m_collision;
	proxy.m_floatingBody = &collideBodyB;
	proxy.m_floatingCollision = collideBodyB.m_collision;
	proxy.m_timestep = dgFloat32 (0.0f);
	proxy.m_skinThickness = dgFloat32 (0.0f);
	proxy.m_maxContacts = 16;
	
	dgInt32 flag = 0;
	if (collisionA.IsType (dgCollision::dgCollisionCompound_RTTI)) {
		//return ClosestCompoundPoint (&collideBodyA, &collideBodyB, contactA, contactB, normalAB, threadIndex);
		flag = ClosestCompoundPoint (proxy);
	} else if (collisionB.IsType (dgCollision::dgCollisionCompound_RTTI)) {
		dgAssert (0);
		//dgInt32 flag = ClosestCompoundPoint (&collideBodyB, &collideBodyA, contactB, contactA, normalAB, threadIndex);
		dgSwap (proxy.m_referenceBody, proxy.m_floatingBody);
		dgSwap (proxy.m_referenceCollision, proxy.m_floatingCollision);
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

	dgContact contactJoint (this, &material);
	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgCollidingPairCollector::dgPair pair;
	pair.m_contactCount = 0;
	pair.m_contact = &contactJoint;
	pair.m_contactBuffer = NULL; 
	pair.m_timeOfImpact = dgFloat32 (0.0f);
	pair.m_isDeformable = 0;
	pair.m_cacheIsValid = 0;
	CalculateContacts (&pair, dgFloat32 (0.0f), threadIndex, false, true);
	return (pair.m_contactCount == -1) ? true : false;
}


//dgInt32 dgWorld::ClosestCompoundPoint (dgBody* const compoundConvexA, dgBody* const collisionB, dgTriplex& contactA, dgTriplex& contactB, dgTriplex& normalAB, dgInt32 threadIndex) const
dgInt32 dgWorld::ClosestCompoundPoint (dgCollisionParamProxy& proxy) const
{
//	dgCollisionInstance* const instance = compoundConvexA->m_collision;
	dgCollisionInstance* const instance = proxy.m_referenceCollision;
	dgAssert (instance->IsType(dgCollision::dgCollisionCompound_RTTI));
	dgCollisionCompound* const collision = (dgCollisionCompound*) instance->GetChildShape();
//	return collision->ClosestDistance (compoundConvexA, contactA, collisionB, contactB, normalAB);
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


dgInt32 dgWorld::ReduceContacts (dgInt32 count, dgContactPoint* const contact,  dgInt32 maxCount, dgFloat32 tol, dgInt32 arrayIsSorted) const
{
	if ((count > maxCount) && (maxCount > 1)) {
		dgUnsigned8 mask[DG_MAX_CONTATCS];

		if (!arrayIsSorted) {
			dgSort (contact, count, CompareContact, NULL);
		}

		dgInt32 index = 0;
		dgFloat32 window = tol;
		dgFloat32 window2 = window * window;
		dgInt32 countOver = count - maxCount;

		dgAssert (countOver >= 0);
		memset (mask, 0, size_t (count));
		do {
			for (dgInt32 i = 0; (i < count) && countOver; i ++) {
				if (!mask[i]) {
					dgFloat32 val = contact[i].m_point[index] + window;
					for (dgInt32 j = i + 1; (j < count) && countOver && (contact[j].m_point[index] < val) ; j ++) {
						if (!mask[j]) {
							dgVector dp (contact[j].m_point - contact[i].m_point);
							dgFloat32 dist2 = dp % dp;
							if (dist2 < window2) {
								mask[j] = 1;
								countOver --;
							}
						}
					}
				}
			}
			window *= dgFloat32(2.0f);
			window2 = window * window;

		} while (countOver);

		dgInt32 j = 0;
		for (dgInt32 i = 0; i < count; i ++) {
			if (!mask[i]) {
				contact[j] = contact[i];
				j ++;
			}
		}
		dgAssert (j == maxCount);

	} else {
		maxCount = count;
	}

	return maxCount;
}


dgInt32 dgWorld::PruneContacts (dgInt32 count, dgContactPoint* const contact, dgInt32 maxCount) const
{
	if (count > 1) {
		dgUnsigned8 mask[DG_MAX_CONTATCS];

		dgInt32 index = 0;
		dgInt32 packContacts = 0;
		dgFloat32 window = m_contactTolerance;
		dgFloat32 window2 = window * window;

		memset (mask, 0, size_t (count));
		dgSort (contact, count, CompareContact, NULL);

		for (dgInt32 i = 0; i < count; i ++) {
			if (!mask[i]) {
				dgFloat32 val = contact[i].m_point[index] + window;
				for (dgInt32 j = i + 1; (j < count) && (contact[j].m_point[index] < val) ; j ++) {
					if (!mask[j]) {
						dgVector dp (contact[j].m_point - contact[i].m_point);
						dgFloat32 dist2 = dp % dp;
						if (dist2 < window2) {
							if (contact[i].m_penetration < contact[j].m_penetration ) {
								contact[i].m_point = contact[j].m_point;
								contact[i].m_normal = contact[j].m_normal;
								contact[i].m_penetration =contact[j].m_penetration;
							}
							mask[j] = 1;
							packContacts = 1;
						}
					}
				}
			}
		}

		if (packContacts) {
			dgInt32 j = 0;
			for (dgInt32 i = 0; i < count; i ++) {
				if (!mask[i]) {
					contact[j] = contact[i];
					j ++;
				}
			}
			count = j;
		}

		if (count > maxCount) {
			count = ReduceContacts (count, contact, maxCount, window * dgFloat32 (2.0f), 1);
		}
	}
	return count;
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
	if (material->m_contactPoint) {
		material->m_contactPoint(*contact, timestep, threadIndex);
	}
}


void dgWorld::PopulateContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex)
{
	dgContact* const contact = pair->m_contact;
	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
	dgAssert (body0 != body1);
	dgAssert (body0);
	dgAssert (body1);
	dgAssert (contact->m_body0 == body0);
	dgAssert (contact->m_body1 == body1);
	dgAssert (contact->m_broadphaseLru == GetBroadPhase()->m_lru);

	const dgContactMaterial* const material = contact->m_material;
	const dgContactPoint* const contactArray = pair->m_contactBuffer;

	dgInt32 contactCount = pair->m_contactCount;
	dgList<dgContactMaterial>& list = *contact;

	contact->m_timeOfImpact = pair->m_timeOfImpact;

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
	//const dgMatrix& matrix0 = body0->m_matrix;
	const dgVector& com0 = body0->m_globalCentreOfMass;

	const dgVector& v1 = body1->m_veloc;
	const dgVector& w1 = body1->m_omega;
	//const dgMatrix& matrix1 = body1->m_matrix;
	const dgVector& com1 = body1->m_globalCentreOfMass;

	dgVector controlDir0 (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector controlDir1 (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector controlNormal (contactArray[0].m_normal);
	//dgVector vel0 (v0 + w0 * (contactArray[0].m_point - matrix0.m_posit));
	//dgVector vel1 (v1 + w1 * (contactArray[0].m_point - matrix1.m_posit));
	dgVector vel0 (v0 + w0 * (contactArray[0].m_point - com0));
	dgVector vel1 (v1 + w1 * (contactArray[0].m_point - com1));
	dgVector vRel (vel1 - vel0);
	dgVector tangDir (vRel - controlNormal.Scale3 (vRel % controlNormal));
	dgFloat32 diff = tangDir % tangDir;

	dgInt32 staticMotion = 0;
	if (diff <= dgFloat32 (1.0e-2f)) {
		staticMotion = 1;
		if (dgAbsf (controlNormal.m_z) > dgFloat32 (0.577f)) {
			tangDir = dgVector (-controlNormal.m_y, controlNormal.m_z, dgFloat32 (0.0f), dgFloat32 (0.0f));
		} else {
			tangDir = dgVector (-controlNormal.m_y, controlNormal.m_x, dgFloat32 (0.0f), dgFloat32 (0.0f));
		}
		controlDir0 = controlNormal * tangDir;
		dgAssert (controlDir0 % controlDir0 > dgFloat32 (1.0e-8f));
		controlDir0 = controlDir0.Scale3 (dgRsqrt (controlDir0 % controlDir0));
		controlDir1 = controlNormal * controlDir0;
		dgAssert (dgAbsf((controlDir0 * controlDir1) % controlNormal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
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
			diff = v % v;
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

		dgAssert ((dgAbsf(contactMaterial->m_normal % contactMaterial->m_normal) - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));

		//contactMaterial.m_collisionEnable = true;
		//contactMaterial.m_friction0Enable = material->m_friction0Enable;
		//contactMaterial.m_friction1Enable = material->m_friction1Enable;
		//contactMaterial.m_override0Accel = false;
		//contactMaterial.m_override1Accel = false;
		//contactMaterial.m_overrideNormalAccel = false;
		contactMaterial->m_flags = dgContactMaterial::m_collisionEnable | (material->m_flags & (dgContactMaterial::m_friction0Enable | dgContactMaterial::m_friction1Enable));
		contactMaterial->m_userData = material->m_userData;

		if (staticMotion) {
			if ((contactMaterial->m_normal % controlNormal) > dgFloat32 (0.9995f)) {
				contactMaterial->m_dir0 = controlDir0;
				contactMaterial->m_dir1 = controlDir1;
			} else {
				if (dgAbsf (contactMaterial->m_normal.m_z) > dgFloat32 (0.577f)) {
					tangDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_z, dgFloat32 (0.0f), dgFloat32 (0.0f));
				} else {
					tangDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_x, dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
				contactMaterial->m_dir0 = contactMaterial->m_normal * tangDir;
				dgAssert (contactMaterial->m_dir0 % contactMaterial->m_dir0 > dgFloat32 (1.0e-8f));
				contactMaterial->m_dir0 = contactMaterial->m_dir0.Scale3 (dgRsqrt (contactMaterial->m_dir0 % contactMaterial->m_dir0));
				contactMaterial->m_dir1 = contactMaterial->m_normal * contactMaterial->m_dir0;
				dgAssert (dgAbsf((contactMaterial->m_dir0 * contactMaterial->m_dir1) % contactMaterial->m_normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			}
		} else {

			//dgVector vel0 (v0 + w0 * (contactMaterial->m_point - matrix0.m_posit));
			//dgVector vel1 (v1 + w1 * (contactMaterial->m_point - matrix1.m_posit));
			dgVector vel0 (v0 + w0 * (contactMaterial->m_point - com0));
			dgVector vel1 (v1 + w1 * (contactMaterial->m_point - com1));
			dgVector vRel (vel1 - vel0);

			dgFloat32 impulse = vRel % contactMaterial->m_normal;
			if (dgAbsf (impulse) > maxImpulse) {
				maxImpulse = dgAbsf (impulse); 
//				breakImpulse0 = contactMaterial->m_collision0->GetBreakImpulse();
//				breakImpulse1 = contactMaterial->m_collision1->GetBreakImpulse();
			}

			dgVector tangDir (vRel - contactMaterial->m_normal.Scale3 (impulse));
			diff = tangDir % tangDir;

			if (diff > dgFloat32 (1.0e-2f)) {
				contactMaterial->m_dir0 = tangDir.Scale3 (dgRsqrt (diff));
			} else {
				if (dgAbsf (contactMaterial->m_normal.m_z) > dgFloat32 (0.577f)) {
					tangDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_z, dgFloat32 (0.0f), dgFloat32 (0.0f));
				} else {
					tangDir = dgVector (-contactMaterial->m_normal.m_y, contactMaterial->m_normal.m_x, dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
				contactMaterial->m_dir0 = contactMaterial->m_normal * tangDir;
				dgAssert (contactMaterial->m_dir0 % contactMaterial->m_dir0 > dgFloat32 (1.0e-8f));
				contactMaterial->m_dir0 = contactMaterial->m_dir0.Scale3 (dgRsqrt (contactMaterial->m_dir0 % contactMaterial->m_dir0));
			}
			contactMaterial->m_dir1 = contactMaterial->m_normal * contactMaterial->m_dir0;
			dgAssert (dgAbsf((contactMaterial->m_dir0 * contactMaterial->m_dir1) % contactMaterial->m_normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
		}
		contactMaterial->m_normal.m_w = dgFloat32 (0.0f);
		contactMaterial->m_dir0.m_w = dgFloat32 (0.0f); 
		contactMaterial->m_dir1.m_w = dgFloat32 (0.0f); 
	}

	if (count) {
		GlobalLock();
		for (dgInt32 i = 0; i < count; i ++) {
			list.Remove(nodes[i]);
		}
		GlobalUnlock();
	}

	contact->m_maxDOF = dgUnsigned32 (3 * contact->GetCount());
	if (material->m_contactPoint) {
		material->m_contactPoint(*contact, timestep, threadIndex);
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

void dgWorld::ProcessContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex)
{
	dgContact* const contact = pair->m_contact;
	dgAssert (contact->m_body0);
	dgAssert (contact->m_body1);
	dgAssert (contact->m_body0 != contact->m_body1);
	dgAssert (contact->m_broadphaseLru == GetBroadPhase()->m_lru);

	contact->m_positAcc = dgVector (dgFloat32 (0.0f));
	contact->m_rotationAcc = dgVector (dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	PopulateContacts (pair, timestep, threadIndex);
}


void dgWorld::ProcessDeformableContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex)
{
	dgAssert (0);
	/*
	dgDeformableContact* contact = (dgDeformableContact*) pair->m_contact;
	if (!contact) {
	GetLock(threadIndex);
	contact = new (m_allocator) dgDeformableContact (this);
	pair->m_contact = contact;
	AttachConstraint (contact, pair->m_body0, pair->m_body1);
	ReleaseLock();
	} else if (contact->GetBody0() != pair->m_body0) {
	dgAssert (0);
	dgAssert (contact->GetBody1() == pair->m_body0);
	dgAssert (contact->GetBody0() == pair->m_body1);
	Swap (contact->m_body0, contact->m_body1);
	Swap (contact->m_link0, contact->m_link1);
	}

	PopulateContacts (contact, pair, timestep, threadIndex);
	*/
}

dgInt32 dgWorld::ValidateContactCache (dgContact* const contact, dgFloat32 timestep) const
{
	dgAssert (contact && (contact->GetId() == dgConstraint::m_contactConstraint));

	dgBody* const body0 = contact->GetBody0();
	dgBody* const body1 = contact->GetBody1();

	dgVector deltaTime (timestep);
	dgVector positStep ((body0->m_veloc - body1->m_veloc).CompProduct4 (deltaTime));
	dgVector rotationStep ((body0->m_omega - body1->m_omega).CompProduct4 (deltaTime));
	contact->m_positAcc += positStep;
	contact->m_rotationAcc = contact->m_rotationAcc * dgQuaternion (dgFloat32 (1.0f), rotationStep.m_x, rotationStep.m_y, rotationStep.m_z);

	dgVector angle (contact->m_rotationAcc.m_q1, contact->m_rotationAcc.m_q2, contact->m_rotationAcc.m_q3, dgFloat32 (0.0f)); 

	dgVector positError2 (contact->m_positAcc.DotProduct4 (contact->m_positAcc));
	dgVector rotatError2 (angle.DotProduct4(angle));

	dgVector mask ((positError2 < m_linearContactError2) & (rotatError2 < m_angularContactError2));

	dgList<dgContactMaterial>& list = *contact;
	dgInt32 testMask = mask.GetSignMask() ? 1 : 0;
	contact->m_contactActive |= testMask;
	return testMask * list.GetCount();
}



void dgWorld::DeformableContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;

	pair->m_isDeformable = 1;
	pair->m_contactCount = 0;

	if (constraint) {
		//dgInt32 contactCount = ValidateContactCache (pair->m_body0, pair->m_body1, constraint);
		//if (contactCount) {
		//pair->m_isDeformable = 1;
		//pair->m_contactCount = 0;
		//dgAssert (pair->m_contactBufferIndex == -1);
		//pair->m_contactBufferIndex = 0;
		//return ;
		//}
	}
	dgAssert (constraint);
	dgAssert (constraint->m_body0);
	dgCollisionDeformableMesh* const deformable = (dgCollisionDeformableMesh*) constraint->m_body0->GetCollision()->GetChildShape();
	dgAssert (constraint->m_body0->GetCollision()->IsType(dgCollision::dgCollisionDeformableMesh_RTTI));
	deformable->CalculateContacts (pair, proxy);
	//	if (pair->m_contactCount) {
	//		// prune close contacts
	//		pair->m_contactCount = dgInt16 (PruneContacts (pair->m_contactCount, proxy.m_contacts));
	//	}
}


void dgWorld::ConvexContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;
	if (constraint->m_maxDOF != 0) {
		dgInt32 contactCount = ValidateContactCache (constraint, proxy.m_timestep);
		if (contactCount) {
			pair->m_cacheIsValid = true;
			pair->m_isDeformable = 0;
			pair->m_contactCount = 0;
			return ;
		}
	}

	dgBody* const convexBody = constraint->m_body0;
	dgBody* const otherBody = constraint->m_body1;
	if (otherBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		dgAssert (convexBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
		dgAssert (otherBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

		proxy.m_referenceBody = convexBody;
		proxy.m_floatingBody = otherBody;
		proxy.m_referenceCollision = convexBody->m_collision;
		proxy.m_floatingCollision = otherBody->m_collision;
		pair->m_contactCount = CalculateConvexToConvexContacts (proxy);

	} else {
		dgAssert (constraint->m_body0->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
		dgAssert (convexBody->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

		proxy.m_referenceBody = convexBody;
		proxy.m_floatingBody = otherBody;
		proxy.m_referenceCollision = convexBody->m_collision;
		proxy.m_floatingCollision = otherBody->m_collision;
		pair->m_contactCount = CalculateConvexToNonConvexContacts (proxy);
	}
}


void dgWorld::CompoundContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;

	pair->m_isDeformable = 0;
	pair->m_contactCount = 0;

	if (constraint->m_maxDOF != 0) {
		dgInt32 contactCount = ValidateContactCache (constraint, proxy.m_timestep);
		if (contactCount) {
			pair->m_cacheIsValid = true;
			pair->m_isDeformable = 0;
			pair->m_contactCount = 0;
			return ;
		}
	}

	dgCollisionInstance* const instance = constraint->m_body0->GetCollision();
	dgCollisionCompound* const compound = (dgCollisionCompound*) instance->GetChildShape();
	dgAssert (compound->IsType(dgCollision::dgCollisionCompound_RTTI));
	compound->CalculateContacts (pair, proxy);
	if (pair->m_contactCount) {
		// prune close contacts
		pair->m_contactCount = PruneContacts (pair->m_contactCount, proxy.m_contacts);
	}
}



void dgWorld::SceneChildContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgAssert (pair->m_contact->GetBody1()->GetCollision()->IsType(dgCollision::dgCollisionScene_RTTI));
	dgContactPoint* const savedBuffer = proxy.m_contacts;

	proxy.m_maxContacts = ((DG_MAX_CONTATCS - pair->m_contactCount) > 32) ? 32 : DG_MAX_CONTATCS - pair->m_contactCount;
	proxy.m_contacts = &savedBuffer[pair->m_contactCount];

	if (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		pair->m_contactCount += CalculateConvexToConvexContacts (proxy);
	} else {
		pair->m_contactCount += CalculateConvexToNonConvexContacts (proxy);
	}

	proxy.m_contacts = savedBuffer;
	if (pair->m_contactCount > (DG_MAX_CONTATCS - 2 * (DG_CONSTRAINT_MAX_ROWS / 3))) {
		pair->m_contactCount = dgInt16 (ReduceContacts (pair->m_contactCount, proxy.m_contacts, DG_CONSTRAINT_MAX_ROWS / 3, m_contactTolerance));
	}
}


void dgWorld::SceneContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	dgContact* const constraint = pair->m_contact;
	pair->m_isDeformable = 0;
	pair->m_contactCount = 0;

	if (constraint->m_maxDOF != 0) {
		dgInt32 contactCount = ValidateContactCache (constraint, proxy.m_timestep);
		if (contactCount) {
			pair->m_cacheIsValid = true;
			pair->m_isDeformable = 0;
			pair->m_contactCount = 0;
			return ;
		}
	}

	dgBody* const sceneBody = constraint->m_body1;
	dgBody* const otherBody = constraint->m_body0;

	dgCollisionInstance* const sceneInstance = sceneBody->GetCollision();
	dgCollisionInstance* const otherInstance = otherBody->GetCollision();
	dgAssert (sceneInstance->IsType(dgCollision::dgCollisionScene_RTTI));
	dgAssert (!otherInstance->IsType(dgCollision::dgCollisionScene_RTTI));
	if (otherInstance->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		proxy.m_referenceBody = otherBody;
		proxy.m_floatingBody = sceneBody;
		proxy.m_referenceCollision = otherBody->m_collision;
		proxy.m_floatingCollision = NULL;

		dgCollisionScene* const scene = (dgCollisionScene*)sceneInstance->GetChildShape();
		scene->CollidePair (pair, proxy);
		if (pair->m_contactCount > 0) {
			// prune close contacts
			pair->m_contactCount = dgInt16 (PruneContacts (pair->m_contactCount, proxy.m_contacts));
		}
	} else if (otherInstance->IsType (dgCollision::dgCollisionCompound_RTTI) & ~otherInstance->IsType (dgCollision::dgCollisionScene_RTTI)) {
		proxy.m_referenceBody = otherBody;
		proxy.m_floatingBody = sceneBody;
		proxy.m_referenceCollision = NULL;
		proxy.m_floatingCollision = NULL;

		dgCollisionScene* const scene = (dgCollisionScene*)sceneInstance->GetChildShape();
		scene->CollideCompoundPair (pair, proxy);
		if (pair->m_contactCount > 0) {
			// prune close contacts
			pair->m_contactCount = dgInt16 (PruneContacts (pair->m_contactCount, proxy.m_contacts));
		}
	} else {
		dgAssert (0);
	}
}


void dgWorld::CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgFloat32 timestep, dgInt32 threadIndex, bool ccdMode, bool intersectionTestOnly)
{
	dgContact* const contact = pair->m_contact;
	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
	const dgContactMaterial* const material = contact->m_material;
	dgCollisionParamProxy proxy(contact, pair->m_contactBuffer, threadIndex, ccdMode, intersectionTestOnly);

	proxy.m_timestep = timestep;
	proxy.m_maxContacts = DG_MAX_CONTATCS;
	proxy.m_skinThickness = material->m_skinThickness;

	if (body0->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		contact->SwapBodies();
		SceneContacts (pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		SceneContacts (pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionDeformableMesh_RTTI)) {
		DeformableContacts (pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionDeformableMesh_RTTI)) {
		contact->SwapBodies();
		DeformableContacts (pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		CompoundContacts (pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		contact->SwapBodies();
		CompoundContacts (pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		ConvexContacts (pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
		contact->SwapBodies();
		ConvexContacts (pair, proxy);
	}

	pair->m_timeOfImpact = proxy.m_timestep;
}


dgFloat32 dgWorld::CalculateTimeToImpact (dgContact* const contact, dgFloat32 timestep, dgInt32 threadIndex, dgVector& p, dgVector& q, dgVector& normal) const
{
	dgCollidingPairCollector::dgPair pair;

	dgInt32 isActive = contact->m_contactActive;
	dgInt32 contactCount = contact->m_maxDOF;

	contact->m_maxDOF = 0;
	pair.m_contact = contact;
	pair.m_cacheIsValid = false;
	pair.m_contactBuffer = NULL;

	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
	const dgContactMaterial* const material = contact->m_material;
	dgCollisionParamProxy proxy(contact, NULL, threadIndex, true, false);

	proxy.m_maxContacts = 0;
	proxy.m_timestep = timestep;
	proxy.m_skinThickness = material->m_skinThickness;

	if (body0->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		contact->SwapBodies();
		SceneContacts (&pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionScene_RTTI)) {
		SceneContacts (&pair, proxy);
	} else if (body0->m_collision->IsType (dgCollision::dgCollisionDeformableMesh_RTTI)) {
		DeformableContacts (&pair, proxy);
	} else if (body1->m_collision->IsType (dgCollision::dgCollisionDeformableMesh_RTTI)) {
		contact->SwapBodies();
		DeformableContacts (&pair, proxy);
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
		normal = proxy.m_normal.Scale3(dgFloat32 (-1.0f));
		p = proxy.m_closestPointBody1;
		q = proxy.m_closestPointBody0;
	}

	contact->m_contactActive = isActive;
	contact->m_maxDOF = contactCount;
	return proxy.m_timestep;
}


// ***************************************************************************
//
// ***************************************************************************

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

	collisionA.SetCollisionMode(true);
	collisionB.SetCollisionMode(true);

	dgContactPoint contacts[DG_MAX_CONTATCS];

	dgInt32 count = 0;
	dgFloat32 time = retTimeStep;
	retTimeStep = dgFloat32 (1.0f);
	maxContacts = dgMin (DG_MAX_CONTATCS, maxContacts);

	collideBodyA.m_world = this;
	collideBodyA.SetContinueCollisionMode(true); 
	collideBodyA.m_matrix = matrixA;
	collideBodyA.m_collision = &collisionA;
	collideBodyA.m_masterNode = NULL;
	collideBodyA.m_collisionCell = NULL;
	collideBodyA.m_veloc = dgVector (velocA[0], velocA[1], velocA[2], dgFloat32 (0.0f));
	collideBodyA.m_omega = dgVector (omegaA[0], omegaA[1], omegaA[2], dgFloat32 (0.0f));
	collisionA.SetGlobalMatrix(collisionA.GetLocalMatrix() * matrixA);

	collideBodyB.m_world = this;
	collideBodyB.SetContinueCollisionMode(true); 
	collideBodyB.m_matrix = matrixB;
	collideBodyB.m_collision = &collisionB;
	collideBodyB.m_masterNode = NULL;
	collideBodyB.m_collisionCell = NULL;
	collideBodyB.m_veloc = dgVector (velocB[0], velocB[1], velocB[2], dgFloat32 (0.0f));
	collideBodyB.m_omega = dgVector (omegaB[0], omegaB[1], omegaB[2], dgFloat32 (0.0f));
	collisionB.SetGlobalMatrix(collisionB.GetLocalMatrix() * matrixB);

	dgContactMaterial material;
	material.m_penetration = dgFloat32 (0.0f);

	dgContact contactJoint (this, &material);
	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgCollidingPairCollector::dgPair pair;
	pair.m_contact = &contactJoint;
	pair.m_contactBuffer = contacts;
	pair.m_timeOfImpact = dgFloat32 (0.0f);
	pair.m_contactCount = 0;
	pair.m_isDeformable = 0;
	pair.m_cacheIsValid = 0;
	CalculateContacts (&pair, time, threadIndex, true, false);

	if (pair.m_timeOfImpact < dgFloat32 (1.0f)) {
		retTimeStep = pair.m_timeOfImpact;
	}

	count = pair.m_contactCount;
	if (count) {
		if (count > maxContacts) {
			count = PruneContacts (count, contacts, maxContacts);
		}
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
	} 
	return count;
}


//dgInt32 dgWorld::Collide (const dgCollisionInstance* const collisionSrcA, const dgMatrix& matrixA, const dgCollisionInstance* const collisionSrcB, const dgMatrix& matrixB, dgTriplex* const points, dgTriplex* const normals, dgFloat32* const penetration, dgInt32* const attibute, dgInt32 maxSize, dgInt32 threadIndex)
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

	dgContact contactJoint (this, &material);
	contactJoint.SetBodies (&collideBodyA, &collideBodyB);

	dgCollidingPairCollector::dgPair pair;
	pair.m_contactCount = 0;
	pair.m_contact = &contactJoint;
	pair.m_contactBuffer = contacts; 
	pair.m_timeOfImpact = dgFloat32 (0.0f);
	pair.m_isDeformable = 0;
	pair.m_cacheIsValid = 0;
	CalculateContacts (&pair, dgFloat32 (0.0f), threadIndex, false, false);

	count = pair.m_contactCount;
	if (count > maxContacts) {
		count = ReduceContacts (count, contacts, maxContacts, m_contactTolerance);
		count = dgMin (count, maxContacts);
	}

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



dgInt32 dgWorld::ClosestPoint (dgCollisionParamProxy& proxy) const	
{
	dgCollisionInstance* const collision1 = proxy.m_referenceCollision;
	dgCollisionInstance* const collision2 = proxy.m_floatingCollision;

	dgContact* const contactJoint = proxy.m_contactJoint;
	dgAssert (contactJoint);
	contactJoint->m_closestDistance = dgFloat32 (1.0e10f);

	if (!(collision1->GetConvexVertexCount() && collision2->GetConvexVertexCount())) {
		return 0;
	}

	dgAssert (contactJoint);
	dgAssert (collision1->GetCollisionPrimityType() != m_nullCollision);
	dgAssert (collision2->GetCollisionPrimityType() != m_nullCollision);
	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	dgCollisionID id1 = collision1->GetCollisionPrimityType();
	dgCollisionID id2 = collision2->GetCollisionPrimityType();
	dgAssert (id1 < m_nullCollision);
	dgAssert (id2 < m_nullCollision);

	bool flipShape = dgCollisionConvex::m_priorityOrder.m_swapPriority[id1][id2];
	if (flipShape) {
		dgCollisionParamProxy tmp(proxy.m_contactJoint, proxy.m_contacts, proxy.m_threadIndex, proxy.m_continueCollision, proxy.m_intersectionTestOnly);
		tmp.m_referenceBody = proxy.m_floatingBody;
		tmp.m_floatingBody = proxy.m_referenceBody;
		tmp.m_referenceCollision = proxy.m_floatingCollision;
		tmp.m_floatingCollision = proxy.m_referenceCollision;
		tmp.m_timestep = proxy.m_timestep;
		tmp.m_skinThickness = proxy.m_skinThickness;
		tmp.m_maxContacts = proxy.m_maxContacts;
		tmp.m_matrix = collision1->GetGlobalMatrix() * collision2->GetGlobalMatrix().Inverse();

		dgCollisionConvex* const convexShape = (dgCollisionConvex*) collision2->m_childShape;

		dgVector v (tmp.m_matrix.m_posit & dgVector::m_triplexMask);
		dgFloat32 mag2 = v.DotProduct4(v).m_x;
		if (mag2 > dgFloat32 (0.0f)) {
			contactJoint->m_separtingVector = v.Scale3 (dgRsqrt (mag2));
		} else {
			contactJoint->m_separtingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		}

		bool state = convexShape->CalculateClosestPoints (tmp);
		if (state) {
			proxy.m_contacts[0].m_normal = tmp.m_contacts[0].m_normal.Scale3 (dgFloat32 (-1.0f));
			proxy.m_contacts[1].m_normal = tmp.m_contacts[1].m_normal.Scale3 (dgFloat32 (-1.0f));
			dgSwap (proxy.m_contacts[0].m_point, proxy.m_contacts[1].m_point);
			return 1;
		}
	} else {
		proxy.m_matrix = collision2->GetGlobalMatrix() * collision1->GetGlobalMatrix().Inverse();
		dgCollisionConvex* const convexShape = (dgCollisionConvex*) collision1->m_childShape;

		dgVector v (proxy.m_matrix.m_posit & dgVector::m_triplexMask);
		dgFloat32 mag2 = v.DotProduct4(v).m_x;
		if (mag2 > dgFloat32 (0.0f)) {
			contactJoint->m_separtingVector = v.Scale3 (dgRsqrt (mag2));
		} else {
			contactJoint->m_separtingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		}

		bool state = convexShape->CalculateClosestPoints (proxy);
		return state ? 1 : 0;
	}

	return 0;
}




dgInt32 dgWorld::CalculateConvexToConvexContacts (dgCollisionParamProxy& proxy) const
{
	dgAssert (proxy.m_referenceCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	dgInt32 count = 0;
	dgCollisionInstance* const collision1 = proxy.m_referenceCollision;
	dgCollisionInstance* const collision2 = proxy.m_floatingCollision;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgAssert (contactJoint);

	contactJoint->m_closestDistance = dgFloat32 (1.0e10f);
	if (!(collision1->GetConvexVertexCount() && collision2->GetConvexVertexCount())) {
		return count;
	}

	dgAssert (collision1->GetCollisionPrimityType() != m_nullCollision);
	dgAssert (collision2->GetCollisionPrimityType() != m_nullCollision);
	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	dgCollisionID id1 = collision1->GetCollisionPrimityType();
	dgCollisionID id2 = collision2->GetCollisionPrimityType();
	dgAssert (id1 < m_nullCollision);
	dgAssert (id2 < m_nullCollision);
	bool flipShape = dgCollisionConvex::m_priorityOrder.m_swapPriority[id1][id2];
	if (proxy.m_continueCollision) {
		if (flipShape) {
			dgCollisionParamProxy tmp(proxy.m_contactJoint, proxy.m_contacts, proxy.m_threadIndex, proxy.m_continueCollision, proxy.m_intersectionTestOnly);
			tmp.m_referenceBody = proxy.m_floatingBody;
			tmp.m_floatingBody = proxy.m_referenceBody;
			tmp.m_referenceCollision = proxy.m_floatingCollision;
			tmp.m_floatingCollision = proxy.m_referenceCollision;
			tmp.m_timestep = proxy.m_timestep;
			tmp.m_skinThickness = proxy.m_skinThickness;
			tmp.m_maxContacts = proxy.m_maxContacts;
			tmp.m_matrix = collision1->GetGlobalMatrix() * collision2->GetGlobalMatrix().Inverse();

			dgCollisionConvex* const convexShape = (dgCollisionConvex*) collision2->m_childShape;
			if (contactJoint->m_isNewContact) {
				contactJoint->m_isNewContact = false;
				dgVector v (tmp.m_matrix.m_posit & dgVector::m_triplexMask);
				dgFloat32 mag2 = v.DotProduct4(v).m_x;
				if (mag2 > dgFloat32 (0.0f)) {
					contactJoint->m_separtingVector = v.Scale3 (dgRsqrt (mag2));
				} else {
					contactJoint->m_separtingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
			}
			count = convexShape->CalculateConvexCastContacts (tmp);
			proxy.m_timestep = tmp.m_timestep;

			dgVector step (tmp.m_floatingBody->m_veloc.Scale3 (tmp.m_timestep));
			dgContactPoint* const contactOut = proxy.m_contacts;
			for (dgInt32 i = 0; i < count; i ++) {
				contactOut[i].m_normal = contactOut[i].m_normal.Scale3 (dgFloat32 (-1.0f));
				contactOut[i].m_point += step;
			}

			proxy.m_normal = tmp.m_normal.Scale3(dgFloat32 (-1.0f));
			proxy.m_closestPointBody0 = tmp.m_closestPointBody1;
			proxy.m_closestPointBody1 = tmp.m_closestPointBody0;

		} else {
			proxy.m_matrix = collision2->GetGlobalMatrix() * collision1->GetGlobalMatrix().Inverse();
			dgCollisionConvex* const convexShape = (dgCollisionConvex*) collision1->m_childShape;

			if (contactJoint->m_isNewContact) {
				contactJoint->m_isNewContact = false;
				dgVector v (proxy.m_matrix.m_posit & dgVector::m_triplexMask);
				dgFloat32 mag2 = v.DotProduct4(v).m_x;
				if (mag2 > dgFloat32 (0.0f)) {
					contactJoint->m_separtingVector = v.Scale3 (dgRsqrt (mag2));
				} else {
					contactJoint->m_separtingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
			}
			count = convexShape->CalculateConvexCastContacts (proxy);
		}
	} else {
		if (flipShape) {
			dgCollisionParamProxy tmp(proxy.m_contactJoint, proxy.m_contacts, proxy.m_threadIndex, proxy.m_continueCollision, proxy.m_intersectionTestOnly);
			tmp.m_referenceBody = proxy.m_floatingBody;
			tmp.m_floatingBody = proxy.m_referenceBody;
			tmp.m_referenceCollision = proxy.m_floatingCollision;
			tmp.m_floatingCollision = proxy.m_referenceCollision;
			tmp.m_timestep = proxy.m_timestep;
			tmp.m_skinThickness = proxy.m_skinThickness;
			tmp.m_maxContacts = proxy.m_maxContacts;
			tmp.m_matrix = collision1->GetGlobalMatrix() * collision2->GetGlobalMatrix().Inverse();

			dgCollisionConvex* const convexShape = (dgCollisionConvex*) collision2->m_childShape;

			if (contactJoint->m_isNewContact) {
				contactJoint->m_isNewContact = false;
				dgVector v (tmp.m_matrix.m_posit & dgVector::m_triplexMask);
				dgFloat32 mag2 = v.DotProduct4(v).m_x;
				if (mag2 > dgFloat32 (0.0f)) {
					contactJoint->m_separtingVector = v.Scale3 (dgRsqrt (mag2));
				} else {
					contactJoint->m_separtingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
			}


			count = convexShape->CalculateConvexToConvexContact (tmp);
			proxy.m_timestep = tmp.m_timestep;

			dgContactPoint* const contactOut = proxy.m_contacts;
			for (dgInt32 i = 0; i < count; i ++) {
				contactOut[i].m_normal = contactOut[i].m_normal.Scale3 (dgFloat32 (-1.0f));
			}

			proxy.m_normal = tmp.m_normal.Scale3(dgFloat32 (-1.0f));
			proxy.m_closestPointBody0 = tmp.m_closestPointBody1;
			proxy.m_closestPointBody1 = tmp.m_closestPointBody0;

		} else {
			proxy.m_matrix = collision2->GetGlobalMatrix() * collision1->GetGlobalMatrix().Inverse();
			dgCollisionConvex* const convexShape = (dgCollisionConvex*) collision1->m_childShape;

			if (contactJoint->m_isNewContact) {
				contactJoint->m_isNewContact = false;
				dgVector v (proxy.m_matrix.m_posit & dgVector::m_triplexMask);
				dgFloat32 mag2 = v.DotProduct4(v).m_x;
				if (mag2 > dgFloat32 (0.0f)) {
					contactJoint->m_separtingVector = v.Scale3 (dgRsqrt (mag2));
				} else {
					contactJoint->m_separtingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				}
			}
			count = convexShape->CalculateConvexToConvexContact (proxy);
		}
	}

	dgContactPoint* const contactOut = proxy.m_contacts;
	for (dgInt32 i = 0; i < count; i ++) {
		contactOut[i].m_body0 = proxy.m_referenceBody;
		contactOut[i].m_body1 = proxy.m_floatingBody;
		contactOut[i].m_collision0 = collision1;
		contactOut[i].m_collision1 = collision2;
		contactOut[i].m_shapeId0 = collision1->GetUserDataID();
		contactOut[i].m_shapeId1 = collision2->GetUserDataID();
	}
	return count;
}



dgInt32 dgWorld::CalculateConvexToNonConvexContacts (dgCollisionParamProxy& proxy) const
{
	dgInt32 count = 0;
	if (proxy.m_referenceCollision->GetCollisionMode() & proxy.m_floatingCollision->GetCollisionMode()) {
		dgCollisionInstance* const convexInstance = proxy.m_referenceCollision;

		dgContact* const contactJoint = proxy.m_contactJoint;
		dgAssert (contactJoint);
		contactJoint->m_closestDistance = dgFloat32 (1.0e10f);

		if (!convexInstance->GetConvexVertexCount()) {
			return count;
		}

		dgAssert (proxy.m_timestep <= dgFloat32 (1.0f));
		dgAssert (proxy.m_timestep >= dgFloat32 (0.0f));

		//dgPolygonMeshDesc data(proxy, proxy.m_floatingCollision->GetUserData());
		dgPolygonMeshDesc data(proxy, NULL);
		if (proxy.m_continueCollision) {
			data.m_doContinuesCollisionTest = true;

			const dgVector& hullVeloc = data.m_objBody->m_veloc;
			const dgVector& hullOmega = data.m_objBody->m_omega;

			dgFloat32 baseLinearSpeed = dgSqrt (hullVeloc % hullVeloc);
			if (baseLinearSpeed > dgFloat32 (1.0e-6f)) {
				dgFloat32 minRadius = convexInstance->GetBoxMinRadius();
				dgFloat32 maxAngularSpeed = dgSqrt (hullOmega % hullOmega);
				dgFloat32 angularSpeedBound = maxAngularSpeed * (convexInstance->GetBoxMaxRadius() - minRadius);

				dgFloat32 upperBoundSpeed = baseLinearSpeed + dgSqrt (angularSpeedBound);
				dgVector upperBoundVeloc (hullVeloc.Scale3 (proxy.m_timestep * upperBoundSpeed / baseLinearSpeed));

				//const dgMatrix& soupMatrix = data.m_polySoupCollision->GetGlobalMatrix();
				//data.m_boxDistanceTravelInMeshSpace = data.m_polySoupCollision->GetInvScale().CompProduct4(soupMatrix.UnrotateVector(upperBoundVeloc.CompProduct4(data.m_objCollision->GetInvScale())));
				data.SetDistanceTravel (upperBoundVeloc);
			}
		}

		dgCollisionMesh* const polysoup = (dgCollisionMesh *) data.m_polySoupCollision->GetChildShape();
		polysoup->GetCollidingFaces (&data);

		if (data.m_faceCount) {
			proxy.m_polyMeshData = &data;
			proxy.m_matrix = proxy.m_matrix.Inverse();

			if (proxy.m_continueCollision) {
				count = CalculateConvexToNonConvexContactsContinue (proxy);
			} else {
				count = CalculatePolySoupToHullContactsDescrete (proxy);
			}

			if (count > 0) {
				proxy.m_contactJoint->m_contactActive = 1;
				count = PruneContacts (count, proxy.m_contacts);
				dgContactPoint* const contactOut = proxy.m_contacts;
				for (dgInt32 i = 0; i < count; i ++) {
					dgAssert ((dgAbsf(contactOut[i].m_normal % contactOut[i].m_normal) - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
					contactOut[i].m_body0 = proxy.m_referenceBody;
					contactOut[i].m_body1 = proxy.m_floatingBody;
					contactOut[i].m_collision0 = proxy.m_referenceCollision;
					contactOut[i].m_collision1 = proxy.m_floatingCollision;
				}
			}
		}
	}
	return count;
}

dgInt32 dgWorld::CalculateConvexToNonConvexContactsContinue (dgCollisionParamProxy& proxy) const
{
	dgInt32 count = 0;

	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionMesh_RTTI));
	dgAssert (proxy.m_referenceCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	dgCollisionInstance* const polySoupInstance = proxy.m_floatingCollision;

	dgPolygonMeshDesc& data = *proxy.m_polyMeshData;
	dgAssert (data.m_faceCount); 

	dgCollisionConvexPolygon polygon (m_allocator);
	dgCollisionInstance polyInstance (*polySoupInstance, &polygon);
	polyInstance.SetScale (dgVector (1.0f));

	proxy.m_floatingCollision = &polyInstance;

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

	dgInt32 maxContacts = proxy.m_maxContacts;
	dgInt32 maxReduceLimit = maxContacts >> 2;
	dgInt32 countleft = maxContacts;

	dgAssert (proxy.m_contactJoint);
	dgVector separatingVector (proxy.m_matrix.m_posit & dgVector::m_triplexMask);
	dgFloat32 mag2 = separatingVector.DotProduct4 (separatingVector).m_x;
	if (mag2 > dgFloat32 (0.0f)) {
		separatingVector = separatingVector.Scale3 (dgRsqrt (mag2));
	} else {
		separatingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	}

	const dgVector& scale = polySoupInstance->GetScale();
	const dgVector& invScale = polySoupInstance->GetInvScale();
	const dgInt32 stride = polygon.m_stride;
	const dgFloat32* const vertex = polygon.m_vertex;

	dgContactPoint* const contactOut = proxy.m_contacts;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;
	data.SortFaceArray();

	dgVector n (dgFloat32 (0.0f));
	dgVector p (dgFloat32 (0.0f));
	dgVector q (dgFloat32 (0.0f));
	dgUnsigned64 shapeFaceID = dgUnsigned64(-1);

	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	dgFloat32 minTimeStep = proxy.m_timestep;
	dgFloat32 timeNormalizer = proxy.m_timestep;
	dgFloat32 epsilon = dgFloat32 (-1.0e-3f) * proxy.m_timestep;

	for (dgInt32 i = 0; (i < data.m_faceCount) && (proxy.m_timestep >= (data.m_hitDistance[i] * timeNormalizer)); i ++) {
		dgInt32 address = data.m_faceIndexStart[i];
		const dgInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = data.m_faceIndexCount[i];
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray (localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId (localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize (localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex (localIndexArray, polygon.m_count);
		polygon.m_normal = dgVector (&vertex[polygon.m_faceNormalIndex * stride]);
		dgAssert (polygon.m_normal.m_w == dgFloat32 (0.0f));
		contactJoint->m_separtingVector = separatingVector;

		proxy.m_maxContacts = countleft;
		proxy.m_contacts = &contactOut[count];

		dgInt32 count1 = polygon.CalculateContactToConvexHullContinue(proxy, scale, invScale);
		if (count1 > 0) {
			dgFloat32 error = proxy.m_timestep - minTimeStep;
			if (error < epsilon) {
				count = 0;
				countleft = maxContacts;
				for (dgInt32 i = 0; i < count1; i ++) {
					contactOut[i] = proxy.m_contacts[i];
				}
			}
			count += count1;
			countleft -= count1;
			dgAssert (countleft >= 0); 
			if (count >= maxReduceLimit) {
				count = ReduceContacts (count, contactOut, maxReduceLimit >> 1, dgFloat32 (1.0e-2f));
				countleft = maxContacts - count;
				dgAssert (countleft >= 0); 
			}
		}

		closestDist = dgMin (closestDist, contactJoint->m_closestDistance);

		if (proxy.m_timestep <= minTimeStep) {
			minTimeStep = proxy.m_timestep;
			n = proxy.m_normal;
			p = proxy.m_closestPointBody0;
			q = proxy.m_closestPointBody1;
			shapeFaceID = proxy.m_shapeFaceID;
		}
	}


	// check for extreme obtuse contacts 
	//	dgFloat32 penetrations[DG_MAX_CONTATCS];
	//	const dgCollisionInstance* const convexInstance = proxy.m_referenceCollision;
	//	const dgMatrix& matrix = convexInstance->GetGlobalMatrix();
	//	for (dgInt32 i = 0; i < count; i ++) {
	//		const dgVector& normal = contactOut[i].m_normal;
	//		dgVector minPenetration (contactOut[i].m_point - matrix.TransformVector(convexInstance->SupportVertex (matrix.UnrotateVector(normal.Scale3 (dgFloat32 (-1.0f))), NULL)));
	//		penetrations[i] = minPenetration % normal;
	//	}
	//	for (dgInt32 i = 0; i < count; i ++) {
	//		const dgVector& n0 = contactOut[i].m_normal;
	//		for (dgInt32 j = i + 1; j < count; j ++) {
	//			const dgVector& n1 = contactOut[j].m_normal;
	//			dgFloat32 dir = n0 % n1;
	//			if (dir < dgFloat32 (-0.995f)) {
	//				dgFloat32 dist0 = penetrations[i];
	//				dgFloat32 dist1 = penetrations[j];
	//				count --;
	//				if (dist0 <= dist1) {
	//					contactOut[j] = contactOut[count];
	//					penetrations[j] = penetrations[count];
	//					j --;
	//				} else {
	//					contactOut[i] = contactOut[count];
	//					penetrations[i] = penetrations[count];
	//					i --;
	//					break;
	//				}
	//			}
	//		}
	//	} 
	//	proxy.m_contacts = contactOut;

	proxy.m_contacts = contactOut;
	contactJoint->m_closestDistance = closestDist;

	// restore the pointer
	proxy.m_normal = n;
	proxy.m_closestPointBody0 = p;
	proxy.m_closestPointBody1 = q;
	proxy.m_floatingCollision = polySoupInstance;
	proxy.m_shapeFaceID = shapeFaceID;

	return count;
}



dgInt32 dgWorld::CalculatePolySoupToHullContactsDescrete (dgCollisionParamProxy& proxy) const
{
	dgInt32 count = 0;
	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionMesh_RTTI));
	dgAssert (proxy.m_referenceCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	dgCollisionInstance* const polySoupInstance = proxy.m_floatingCollision;
	dgPolygonMeshDesc& data = *proxy.m_polyMeshData;

	dgAssert (data.m_faceCount); 

	dgCollisionConvexPolygon polygon (m_allocator);
	dgCollisionInstance polyInstance (*polySoupInstance, &polygon);
	polyInstance.SetScale(dgVector (dgFloat32 (1.0f)));

	proxy.m_floatingCollision = &polyInstance;

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

	dgInt32 maxContacts = proxy.m_maxContacts;
	dgInt32 maxReduceLimit = maxContacts >> 2;
	dgInt32 countleft = maxContacts;

	dgAssert (proxy.m_contactJoint);
	dgVector separatingVector (proxy.m_matrix.m_posit & dgVector::m_triplexMask);
	dgFloat32 mag2 = separatingVector.DotProduct4 (separatingVector).m_x;
	if (mag2 > dgFloat32 (0.0f)) {
		separatingVector = separatingVector.Scale3 (dgRsqrt (mag2));
	} else {
		separatingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	}

	const dgVector& scale = polySoupInstance->GetScale();
	const dgVector& invScale = polySoupInstance->GetInvScale();
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
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray (localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId (localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize (localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex (localIndexArray, polygon.m_count);
		polygon.m_normal = dgVector (&vertex[polygon.m_faceNormalIndex * stride]);
		dgAssert (dgAbsf(polygon.m_normal % polygon.m_normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (polygon.m_normal.m_w == dgFloat32 (0.0f));
		contactJoint->m_separtingVector = separatingVector;

		proxy.m_maxContacts = countleft;
		proxy.m_contacts = &contactOut[count];
		dgInt32 count1 = polygon.CalculateContactToConvexHullDescrete (proxy, scale, invScale);
		closestDist = dgMin(closestDist, contactJoint->m_closestDistance);

		if (count1 > 0) {
			count += count1;
			countleft -= count1;
			dgAssert (countleft >= 0); 
			if (count >= maxReduceLimit) {
				count = ReduceContacts (count, contactOut, maxReduceLimit >> 1, dgFloat32 (1.0e-2f));
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

	// check for extreme obtuse contacts 
	dgFloat32 penetrations[DG_MAX_CONTATCS];
	const dgCollisionInstance* const convexInstance = proxy.m_referenceCollision;
	const dgMatrix& matrix = convexInstance->GetGlobalMatrix();
	for (dgInt32 i = 0; i < count; i ++) {
		const dgVector& normal = contactOut[i].m_normal;
		dgVector minPenetration (contactOut[i].m_point - matrix.TransformVector(convexInstance->SupportVertex (matrix.UnrotateVector(normal.Scale3 (dgFloat32 (-1.0f))), NULL)));
		penetrations[i] = minPenetration % normal;
	}

	for (dgInt32 i = 0; i < count; i ++) {
		const dgVector& n0 = contactOut[i].m_normal;
		for (dgInt32 j = i + 1; j < count; j ++) {
			const dgVector& n1 = contactOut[j].m_normal;
			dgFloat32 dir = n0 % n1;
			if (dir < dgFloat32 (-0.995f)) {
				dgFloat32 dist0 = penetrations[i];
				dgFloat32 dist1 = penetrations[j];
				count --;
				if (dist0 <= dist1) {
					contactOut[j] = contactOut[count];
					penetrations[j] = penetrations[count];
					j --;
				} else {
					contactOut[i] = contactOut[count];
					penetrations[i] = penetrations[count];
					i --;
					break;
				}
			}
		}
	} 

	proxy.m_contacts = contactOut;

	// restore the pointer
	proxy.m_floatingCollision = polySoupInstance;

	return count;
}
