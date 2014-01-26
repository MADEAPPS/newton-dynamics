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
#include "dgIntersections.h"
#include "dgCollisionScene.h"
#include "dgCollisionInstance.h"


dgCollisionScene::dgCollisionScene (dgWorld* const world)
	:dgCollisionCompound(world)
{
	m_collisionId = m_sceneCollision;
	m_rtti |= dgCollisionScene_RTTI;
}

dgCollisionScene::dgCollisionScene (const dgCollisionScene& source, const dgCollisionInstance* const myInstance)
	:dgCollisionCompound(source, myInstance)
{
	m_rtti |= dgCollisionScene_RTTI;
}

dgCollisionScene::dgCollisionScene (dgWorld* const world, dgDeserialize deserialization, void* const userData, const dgCollisionInstance* const myInstance)
	:dgCollisionCompound(world, deserialization, userData, myInstance)
{
	dgAssert (m_rtti | dgCollisionScene_RTTI);
}

dgCollisionScene::~dgCollisionScene()
{
}

dgFloat32 dgCollisionScene::GetBoxMinRadius () const
{
	return dgFloat32 (0.0f);  
}

dgFloat32 dgCollisionScene::GetBoxMaxRadius () const
{
	return dgFloat32 (0.0f);  
}


void dgCollisionScene::MassProperties ()
{
	m_inertia = dgVector (dgFloat32 (0.0f));
	m_centerOfMass = dgVector (dgFloat32 (0.0f));
	m_crossInertia = dgVector (dgFloat32 (0.0f));
}

void dgCollisionScene::CollidePair (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	const dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgAssert (proxy.m_contactJoint == pair->m_contact);
	dgContact* const constraint = pair->m_contact;

	dgBody* const otherBody = constraint->GetBody0();
	dgBody* const sceneBody = constraint->GetBody1();
	dgAssert (sceneBody->GetCollision()->GetChildShape() == this);
	dgAssert (sceneBody->GetCollision()->IsType(dgCollision::dgCollisionScene_RTTI));

	dgCollisionInstance* const sceneInstance = sceneBody->m_collision;
	dgCollisionInstance* const otherInstance = otherBody->m_collision;

	dgAssert (sceneInstance->GetChildShape() == this);
	dgAssert (otherInstance->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	const dgContactMaterial* const material = constraint->GetMaterial();

	const dgMatrix& myMatrix = sceneInstance->GetGlobalMatrix();
	const dgMatrix& otherMatrix = otherInstance->GetGlobalMatrix();
	dgMatrix matrix (otherMatrix * myMatrix.Inverse());

	const dgVector& hullVeloc = otherBody->m_veloc;
	dgFloat32 baseLinearSpeed = dgSqrt (hullVeloc % hullVeloc);

	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	if (proxy.m_continueCollision && (baseLinearSpeed > dgFloat32 (1.0e-6f))) {
		dgVector p0;
		dgVector p1;
		otherInstance->CalcAABB (matrix, p0, p1);

		const dgVector& hullOmega = otherBody->m_omega;

		dgFloat32 minRadius = otherInstance->GetBoxMinRadius();
		dgFloat32 maxAngularSpeed = dgSqrt (hullOmega % hullOmega);
		dgFloat32 angularSpeedBound = maxAngularSpeed * (otherInstance->GetBoxMaxRadius() - minRadius);

		dgFloat32 upperBoundSpeed = baseLinearSpeed + dgSqrt (angularSpeedBound);
		dgVector upperBoundVeloc (hullVeloc.Scale3 (proxy.m_timestep * upperBoundSpeed / baseLinearSpeed));

		dgVector boxDistanceTravelInMeshSpace (myMatrix.UnrotateVector(upperBoundVeloc.CompProduct4(otherInstance->m_invScale)));

		dgInt32 stack = 1;
		stackPool[0] = m_root;
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), boxDistanceTravelInMeshSpace);

		while (stack) {
			stack--;
			const dgNodeBase* const me = stackPool[stack];
			dgAssert (me);

			if (me->BoxIntersect (ray, p0, p1)) {
				if (me->m_type == m_leaf) {
					dgAssert (!me->m_right);
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, sceneBody, me->m_myNode, otherBody, NULL, proxy.m_threadIndex);
					}

					if (processContacts) {
						const dgCollisionInstance* const myInstance =  me->GetShape();
						dgCollisionInstance childInstance (*myInstance, myInstance->GetChildShape());
						childInstance.SetGlobalMatrix(childInstance.GetLocalMatrix() * myMatrix);
						proxy.m_floatingCollision = &childInstance;
						dgInt32 count = pair->m_contactCount;
						m_world->SceneChildContacts (pair, proxy);
						if (pair->m_contactCount > count) {
							dgContactPoint* const buffer = proxy.m_contacts;
							for (dgInt32 i = count; i < pair->m_contactCount; i ++) {
								dgAssert (buffer[i].m_collision0 == proxy.m_referenceCollision);
								//if (buffer[i].m_collision1 == proxy.m_floatingCollision) {
								//	buffer[i].m_collision1 = myInstance;
								//}
								if (buffer[i].m_collision1->GetChildShape() == myInstance->GetChildShape()) {
									buffer[i].m_collision1 = myInstance;
								}
							}
						}
						closestDist = dgMin(closestDist, constraint->m_closestDistance);
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

	} else {
		dgVector origin;
		dgVector size;


		otherInstance->CalcObb(origin, size);
		dgOOBBTestData data (matrix, origin, size);
		dgInt32 stack = 1;
		stackPool[0] = m_root;
		while (stack) {

			stack --;
			const dgNodeBase* const me = stackPool[stack];
			dgAssert (me);

			if (me->BoxTest (data)) {
				if (me->m_type == m_leaf) {
					dgAssert (!me->m_right);
					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, sceneBody, me->m_myNode, otherBody, NULL, proxy.m_threadIndex);
					}

					if (processContacts) {
						const dgCollisionInstance* const myInstance =  me->GetShape();
						dgCollisionInstance childInstance (*myInstance, myInstance->GetChildShape());
						childInstance.SetGlobalMatrix(childInstance.GetLocalMatrix() * myMatrix);
						proxy.m_floatingCollision = &childInstance;
						dgInt32 count = pair->m_contactCount;
						m_world->SceneChildContacts (pair, proxy);
						if (pair->m_contactCount > count) {
							dgContactPoint* const buffer = proxy.m_contacts;
							for (dgInt32 i = count; i < pair->m_contactCount; i ++) {
								dgAssert (buffer[i].m_collision0 == proxy.m_referenceCollision);
								//if (buffer[i].m_collision1 == proxy.m_floatingCollision) {
								//	buffer[i].m_collision1 = myInstance;
								//}
								if (buffer[i].m_collision1->GetChildShape() == myInstance->GetChildShape()) {
									buffer[i].m_collision1 = myInstance;
								}
							}
						} else if (pair->m_contactCount == -1) {
							break;
						}
						closestDist = dgMin(closestDist, constraint->m_closestDistance);
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
	}
	constraint->m_closestDistance = closestDist;
}


void dgCollisionScene::CollideCompoundPair (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const
{
	const dgNodeBase* stackPool[4 * DG_COMPOUND_STACK_DEPTH][2];

	dgContact* const constraint = pair->m_contact;
	dgBody* const myBody = constraint->GetBody1();
	dgBody* const otherBody = constraint->GetBody0();

	dgAssert (myBody == proxy.m_floatingBody);
	dgAssert (otherBody == proxy.m_referenceBody);

	dgCollisionInstance* const myCompoundInstance = myBody->m_collision;
	dgCollisionInstance* const otherCompoundInstance = otherBody->m_collision;

	dgAssert (myCompoundInstance->GetChildShape() == this);
	dgAssert (otherCompoundInstance->IsType (dgCollision::dgCollisionCompound_RTTI));
	dgCollisionCompound* const otherCompound = (dgCollisionCompound*)otherCompoundInstance->GetChildShape();

	const dgContactMaterial* const material = constraint->GetMaterial();

	dgMatrix myMatrix (myCompoundInstance->GetLocalMatrix() * myBody->m_matrix);
	dgMatrix otherMatrix (otherCompoundInstance->GetLocalMatrix() * otherBody->m_matrix);
	dgOOBBTestData data (otherMatrix * myMatrix.Inverse());

	dgInt32 stack = 1;
	stackPool[0][0] = m_root;
	stackPool[0][1] = otherCompound->m_root;

	const dgVector& hullVeloc = otherBody->m_veloc;
	dgFloat32 baseLinearSpeed = dgSqrt (hullVeloc % hullVeloc);

	dgFloat32 closestDist = dgFloat32 (1.0e10f);
	if (proxy.m_continueCollision && (baseLinearSpeed > dgFloat32 (1.0e-6f))) {
		dgAssert (0);
	} else {
		while (stack) {
			stack --;
			const dgNodeBase* const me = stackPool[stack][0];
			const dgNodeBase* const other = stackPool[stack][1];

			dgAssert (me && other);

			if (me->BoxTest (data, other)) {

				if ((me->m_type == m_leaf) && (other->m_type == m_leaf)) {
					dgAssert (!me->m_right);

					bool processContacts = true;
					if (material->m_compoundAABBOverlap) {
						processContacts = material->m_compoundAABBOverlap (*material, myBody, me->m_myNode, otherBody, other->m_myNode, proxy.m_threadIndex);
					}

					if (processContacts) {
						const dgCollisionInstance* const mySrcInstance =  me->GetShape();
						const dgCollisionInstance* const otherSrcInstance =  other->GetShape();
						//dgCollisionInstance childInstance (*mySrcInstance, mySrcInstance->GetChildShape());
						//dgCollisionInstance otherInstance (*otherSrcInstance, otherSrcInstance->GetChildShape());
						dgCollisionInstance childInstance (*me->GetShape(), me->GetShape()->GetChildShape());
						dgCollisionInstance otherInstance (*other->GetShape(), other->GetShape()->GetChildShape());

						childInstance.SetGlobalMatrix(childInstance.GetLocalMatrix() * myMatrix);
						otherInstance.SetGlobalMatrix(otherInstance.GetLocalMatrix() * otherMatrix);
						proxy.m_floatingCollision = &childInstance;
						proxy.m_referenceCollision = &otherInstance;

						dgInt32 count = pair->m_contactCount;
						m_world->SceneChildContacts (pair, proxy);
						if (pair->m_contactCount > count) {
							dgContactPoint* const buffer = proxy.m_contacts;
							for (dgInt32 i = count; i < pair->m_contactCount; i ++) {
								//if (buffer[i].m_collision0 == proxy.m_floatingCollision) {
								//	buffer[i].m_collision0 = mySrcInstance;
								//}
								//if (buffer[i].m_collision1 == proxy.m_referenceCollision) {
								//	buffer[i].m_collision1 = otherSrcInstance;
								//}
								if (buffer[i].m_collision1->GetChildShape() == otherSrcInstance->GetChildShape()) {
									dgAssert(buffer[i].m_collision0->GetChildShape() == mySrcInstance->GetChildShape());
									buffer[i].m_collision0 = mySrcInstance;
									buffer[i].m_collision1 = otherSrcInstance;
								} else {
									dgAssert(buffer[i].m_collision1->GetChildShape() == mySrcInstance->GetChildShape());
									buffer[i].m_collision1 = mySrcInstance;
									buffer[i].m_collision0 = otherSrcInstance;
								}
							}
						}

						closestDist = dgMin(closestDist, constraint->m_closestDistance);
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
	}
	constraint->m_closestDistance = closestDist;
}



void dgCollisionScene::Serialize(dgSerialize callback, void* const userData) const
{
	dgCollisionCompound::Serialize(callback, userData);
}