/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
#include "ndScene.h"
#include "ndWorld.h"
#include "ndShapeNull.h"
#include "ndBodyNotify.h"
#include "ndShapeCompound.h"
#include "ndBodyKinematic.h"
#include "ndContactNotify.h"
#include "ndContactSolver.h"
#include "ndRayCastNotify.h"
#include "ndBodyParticleSet.h"
#include "ndConvexCastNotify.h"
#include "ndSkeletonContainer.h"
#include "ndBodyTriggerVolume.h"
#include "ndBodiesInAabbNotify.h"
#include "ndJointBilateralConstraint.h"
#include "ndShapeStaticProceduralMesh.h"

#define D_CONTACT_DELAY_FRAMES		4
#define D_NARROW_PHASE_DIST			ndFloat32 (0.2f)
#define D_CONTACT_TRANSLATION_ERROR	ndFloat32 (1.0e-3f)
#define D_CONTACT_ANGULAR_ERROR		(ndFloat32 (0.25f * ndDegreeToRad))

ndVector ndScene::m_velocTol(ndFloat32(1.0e-16f));
ndVector ndScene::m_angularContactError2(D_CONTACT_ANGULAR_ERROR * D_CONTACT_ANGULAR_ERROR);
ndVector ndScene::m_linearContactError2(D_CONTACT_TRANSLATION_ERROR * D_CONTACT_TRANSLATION_ERROR);

ndScene::ndScene()
	:ndThreadPool("newtonWorker")
	,m_bodyList()
	,m_particleSetList()
	,m_contactArray()
	,m_bvhSceneManager()
	,m_scratchBuffer(1024 * sizeof (void*))
	,m_sceneBodyArray(1024)
	,m_activeConstraintArray(1024)
	,m_specialUpdateList()
	,m_newPairs(1024)
	,m_lock()
	,m_rootNode(nullptr)
	,m_sentinelBody(nullptr)
	,m_contactNotifyCallback(new ndContactNotify(nullptr))
	,m_backgroundThread(nullptr)
	,m_timestep(ndFloat32 (0.0f))
	,m_lru(D_CONTACT_DELAY_FRAMES)
	,m_frameNumber(0)
	,m_subStepNumber(0)
	,m_forceBalanceSceneCounter(0)
{
	m_sentinelBody = new ndBodySentinel;
	m_contactNotifyCallback->m_scene = this;

	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
	{
		m_partialNewPairs[i].Resize(256);
	}
	ndAssert(ndMemory::CheckMemory(this));
}

ndScene::ndScene(const ndScene& src)
	:ndThreadPool("newtonWorker")
	,m_bodyList(src.m_bodyList)
	,m_particleSetList()
	,m_contactArray(src.m_contactArray)
	,m_bvhSceneManager(src.m_bvhSceneManager)
	,m_scratchBuffer()
	,m_sceneBodyArray()
	,m_activeConstraintArray()
	,m_specialUpdateList()
	,m_newPairs(1024)
	,m_lock()
	,m_rootNode(nullptr)
	,m_sentinelBody(nullptr)
	,m_contactNotifyCallback(nullptr)
	,m_backgroundThread(nullptr)
	,m_timestep(ndFloat32(0.0f))
	,m_lru(src.m_lru)
	,m_frameNumber(src.m_frameNumber)
	,m_subStepNumber(src.m_subStepNumber)
	,m_forceBalanceSceneCounter(0)
{
	ndScene* const stealData = (ndScene*)&src;

	SetThreadCount(src.GetThreadCount());
	//m_backgroundThread.SetThreadCount(m_backgroundThread.GetThreadCount());

	m_scratchBuffer.Swap(stealData->m_scratchBuffer);
	m_sceneBodyArray.Swap(stealData->m_sceneBodyArray);
	m_activeConstraintArray.Swap(stealData->m_activeConstraintArray);

	ndSwap(m_rootNode, stealData->m_rootNode);
	ndSwap(m_sentinelBody, stealData->m_sentinelBody);
	ndSwap(m_contactNotifyCallback, stealData->m_contactNotifyCallback);
	m_contactNotifyCallback->m_scene = this;

	ndSpecialList<ndBodyKinematic>::ndNode* nextNode;
	for (ndSpecialList<ndBodyKinematic>::ndNode* node = stealData->m_specialUpdateList.GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		stealData->m_specialUpdateList.Unlink(node);
		m_specialUpdateList.Append(node);
	}

	ndBodyList::ndNode* nextParticleNode;
	for (ndBodyList::ndNode* node = stealData->m_particleSetList.GetFirst(); node; node = nextParticleNode)
	{
		nextParticleNode = node->GetNext();
		stealData->m_particleSetList.Unlink(node);
		ndBodyParticleSet* const particle = (*node->GetInfo())->GetAsBodyParticleSet();
		particle->m_listNode = m_particleSetList.Append(node);
	}

	for (ndBodyListView::ndNode* node = m_bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		body->m_sceneForceUpdate = 1;
		ndScene* const sceneNode = body->GetScene();
		if (sceneNode)
		{
			body->SetSceneNodes(this, node);
		}
		ndAssert (body->GetContactMap().SanityCheck());
	}

	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
	{
		m_partialNewPairs[i].Resize(256);
	}
	ndAssert(ndMemory::CheckMemory(this));
}

ndScene::~ndScene()
{
	ndAssert(ndMemory::CheckMemory(this));
	Cleanup();
	Finish();
	if (m_contactNotifyCallback)
	{
		delete m_contactNotifyCallback;
	}
	ndFreeListAlloc::Flush();
}

void ndScene::PrepareCleanup()
{
}

bool ndScene::IsValid() const
{
	return true;
}

bool ndScene::IsHighPerformanceCompute() const
{
	return false;
}

ndWorld* ndScene::GetWorld() const
{
	return nullptr;
}

ndInt32 ndScene::GetThreadCount() const
{
	const ndThreadPool& pool = *this;
	return pool.GetThreadCount();
}

ndArray<ndUnsigned8>& ndScene::GetScratchBuffer()
{
	return m_scratchBuffer;
}

const ndBodyList& ndScene::GetParticleList() const
{
	return m_particleSetList;
}

const ndBodyListView& ndScene::GetBodyList() const
{
	return m_bodyList;
}

ndArray<ndConstraint*>& ndScene::GetActiveContactArray()
{
	return m_activeConstraintArray;
}

const ndContactArray& ndScene::GetContactArray() const
{
	return m_contactArray;
}

const ndArray<ndConstraint*>& ndScene::GetActiveContactArray() const
{
	return m_activeConstraintArray;
}

ndArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray()
{
	return m_bodyList.GetView();
}

const ndArray<ndBodyKinematic*>& ndScene::GetActiveBodyArray() const
{
	return m_bodyList.GetView();
}

ndFloat32 ndScene::GetTimestep() const
{
	return m_timestep;
}

void ndScene::SetTimestep(ndFloat32 timestep)
{
	m_timestep = timestep;
}

ndBodyKinematic* ndScene::GetSentinelBody() const
{
	return m_sentinelBody;
}


void ndScene::Sync()
{
	ndThreadPool::Sync();
}

void ndScene::Begin()
{
	ndThreadPool::Begin();
}

void ndScene::End()
{
	ndThreadPool::End();
	m_frameNumber++;
}

bool ndScene::AddParticle(const ndSharedPtr<ndBody>& particle)
{
	ndBodyParticleSet* const particleSet = particle->GetAsBodyParticleSet();
	ndAssert(particleSet->m_listNode == nullptr);
	ndBodyList::ndNode* const node = m_particleSetList.Append(particle);
	particleSet->m_listNode = node;
	return true;
}

bool ndScene::RemoveParticle(const ndSharedPtr<ndBody>& particle)
{
	ndBodyParticleSet* const particleSet = particle->GetAsBodyParticleSet();
	ndAssert(particleSet->m_listNode);
	m_particleSetList.Remove(particleSet->m_listNode);
	return true;
}

ndContactNotify* ndScene::GetContactNotify() const
{
	return m_contactNotifyCallback;
}

void ndScene::SetContactNotify(ndContactNotify* const notify)
{
	ndAssert(m_contactNotifyCallback);
	delete m_contactNotifyCallback;
	
	if (notify)
	{
		m_contactNotifyCallback = notify;
	}
	else
	{
		m_contactNotifyCallback = new ndContactNotify(nullptr);
	}
	m_contactNotifyCallback->m_scene = this;
}

void ndScene::DebugScene(ndSceneTreeNotiFy* const notify)
{
	const ndBvhNodeArray& array = m_bvhSceneManager.GetNodeArray();
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		ndBvhNode* const node = array[i];
		if (node->GetAsSceneBodyNode())
		{
			notify->OnDebugNode(node);
		}
	}
}

bool ndScene::AddBody(const ndSharedPtr<ndBody>& body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		if ((kinematicBody->m_scene == nullptr) && (kinematicBody->m_sceneNode == nullptr))
		{
			ndBodyListView::ndNode* const node = m_bodyList.AddItem(body);
			kinematicBody->SetSceneNodes(this, node);
			m_contactNotifyCallback->OnBodyAdded(kinematicBody);
			kinematicBody->UpdateCollisionMatrix();

			m_rootNode = m_bvhSceneManager.AddBody(kinematicBody, m_rootNode);
			if (kinematicBody->GetAsBodyKinematicSpecial())
			{
				kinematicBody->m_spetialUpdateNode = m_specialUpdateList.Append(kinematicBody);
			}

			m_forceBalanceSceneCounter = 0;

			return true;
		}
	}
	else if (body->GetAsBodyParticleSet())
	{
		return AddParticle(body);
	}
	return false;
}

ndSharedPtr<ndBody> ndScene::GetBody(ndBody* const body) const
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return kinematicBody->m_sceneNode->GetInfo();
	}
	else
	{
		ndAssert(0);
		return ndSharedPtr<ndBody>(nullptr);
	}
}

bool ndScene::RemoveBody(const ndSharedPtr<ndBody>& body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		m_forceBalanceSceneCounter = 0;
		m_bvhSceneManager.RemoveBody(kinematicBody);

		//ndAssert(0);
		ndBodyKinematic::ndContactMap& contactMap = kinematicBody->GetContactMap();
		while (contactMap.GetRoot())
		{
			ndContact* const contact = contactMap.GetRoot()->GetInfo();
			m_contactArray.DetachContact(contact);
		}

		ndBodyListView::ndNode* const sceneNode = kinematicBody->m_sceneNode;
		if (kinematicBody->m_scene && sceneNode)
		{
			if (kinematicBody->GetAsBodyKinematicSpecial())
			{
				m_specialUpdateList.Remove(kinematicBody->m_spetialUpdateNode);
				kinematicBody->m_spetialUpdateNode = nullptr;
			}

			m_contactNotifyCallback->OnBodyRemoved(kinematicBody);
			kinematicBody->SetSceneNodes(nullptr, nullptr);
			m_bodyList.RemoveItem(sceneNode);
			return true;
		}
	}
	else
	{
		//ndBodyParticleSet* const particle = body->GetAsBodyParticleSet();
		ndAssert(body->GetAsBodyParticleSet());
		RemoveParticle(body);
	}
	return false;
}

void ndScene::BalanceScene()
{
	D_TRACKTIME();
	UpdateBodyList();
	if (m_bvhSceneManager.GetNodeArray().GetCount() > 2)
	{
		if (!m_forceBalanceSceneCounter)
		{
			m_rootNode = m_bvhSceneManager.BuildBvhTree(*this);
		}
		const ndInt32 sceneUpdatePeriod = 64;
		m_forceBalanceSceneCounter = (m_forceBalanceSceneCounter < sceneUpdatePeriod) ? m_forceBalanceSceneCounter + 1 : 0;
		ndAssert(!m_rootNode || !m_rootNode->m_parent);
	}

	if (!m_bodyList.GetCount())
	{
		m_rootNode = nullptr;
	}
}

void ndScene::UpdateTransformNotify(ndInt32 threadIndex, ndBodyKinematic* const body)
{
	if (body->m_transformIsDirty)
	{
		body->m_transformIsDirty = 0;
		ndBodyNotify* const notify = body->GetNotifyCallback();
		if (notify)
		{
			notify->OnTransform(threadIndex, body->GetMatrix());
		}
	}
}

bool ndScene::ValidateContactCache(ndContact* const contact, const ndVector& timestep) const
{
	ndAssert(contact && (contact->GetAsContact()));

	if (contact->m_maxDof)
	{
		ndBodyKinematic* const body0 = contact->GetBody0();
		ndBodyKinematic* const body1 = contact->GetBody1();

		ndVector positStep(timestep * (body0->m_veloc - body1->m_veloc));
		positStep = ((positStep.DotProduct(positStep)) > m_velocTol) & positStep;
		contact->m_positAcc += positStep;

		ndVector positError2(contact->m_positAcc.DotProduct(contact->m_positAcc));
		ndVector positSign(ndVector::m_negOne & (positError2 < m_linearContactError2));
		if (positSign.GetSignMask())
		{
			ndVector rotationStep(timestep * (body0->m_omega - body1->m_omega));
			rotationStep = ((rotationStep.DotProduct(rotationStep)) > m_velocTol) & rotationStep;
			contact->m_rotationAcc = contact->m_rotationAcc * ndQuaternion(rotationStep.m_x, rotationStep.m_y, rotationStep.m_z, ndFloat32(1.0f));

			ndVector angle(contact->m_rotationAcc & ndVector::m_triplexMask);
			ndVector rotatError2(angle.DotProduct(angle));
			ndVector rotationSign(ndVector::m_negOne & (rotatError2 < m_linearContactError2));
			if (rotationSign.GetSignMask())
			{
				return true;
			}
		}
	}
	return false;
}

void ndScene::CalculateJointContacts(ndInt32 threadIndex, ndContact* const contact)
{
	ndBodyKinematic* const body0 = contact->GetBody0();
	ndBodyKinematic* const body1 = contact->GetBody1();
	
	ndAssert(body0->GetScene() == this);
	ndAssert(body1->GetScene() == this);

	ndAssert(contact->m_material);
	ndAssert(m_contactNotifyCallback);

	bool processContacts = m_contactNotifyCallback->OnAabbOverlap(contact, m_timestep);
	if (processContacts)
	{
		//ndAssert(!body0->GetCollisionShape().GetShape()->GetAsShapeNull());
		//ndAssert(!body1->GetCollisionShape().GetShape()->GetAsShapeNull());

		ndContactPoint contactBuffer[D_MAX_CONTATCS];
		ndContactSolver contactSolver(contact, m_contactNotifyCallback, m_timestep, threadIndex);
		contactSolver.m_separatingVector = contact->m_separatingVector;
		contactSolver.m_contactBuffer = contactBuffer;
		contactSolver.m_intersectionTestOnly = body0->m_contactTestOnly | body1->m_contactTestOnly;

		ndInt32 count = contactSolver.CalculateContactsDiscrete ();
		if (count)
		{
			contact->SetActive(true);
			if (contactSolver.m_intersectionTestOnly)
			{
				ndBodyKinematic* otherBody = body0;
				ndBodyTriggerVolume* trigger = body1->GetAsBodyTriggerVolume();
				if (!trigger)
				{
					otherBody = body1;
					trigger = body0->GetAsBodyTriggerVolume();
				}

				if (trigger && !contact->m_inTrigger)
				{
					contact->m_inTrigger = 1;
					trigger->OnTriggerEnter(otherBody, m_timestep);
				}
				contact->m_isIntersetionTestOnly = 1;
			}
			else
			{
				ndAssert(count <= (D_CONSTRAINT_MAX_ROWS / 3));
				ProcessContacts(threadIndex, count, &contactSolver);
				ndAssert(contact->m_maxDof);
				contact->m_isIntersetionTestOnly = 0;
			}
		}
		else
		{
			if (contactSolver.m_intersectionTestOnly)
			{
				ndBodyKinematic* otherBody = body0;
				ndBodyTriggerVolume* trigger = body1->GetAsBodyTriggerVolume();
				if (!trigger)
				{
					otherBody = body1;
					trigger = body0->GetAsBodyTriggerVolume();
				}
				
				if (trigger && contact->m_inTrigger)
				{
					contact->m_inTrigger = 0;
					ndAssert(contact->m_isIntersetionTestOnly);
					trigger->GetAsBodyTriggerVolume()->OnTriggerExit(otherBody, m_timestep);
				}
				contact->m_isIntersetionTestOnly = 1;
			}
			contact->m_maxDof = 0;
		}
	}
}

void ndScene::ProcessContacts(ndInt32, ndInt32 contactCount, ndContactSolver* const contactSolver)
{
	ndContact* const contact = contactSolver->m_contact;
	contact->m_positAcc = ndVector::m_zero;
	contact->m_rotationAcc = ndQuaternion();

	ndBodyKinematic* const body0 = contact->m_body0;
	ndBodyKinematic* const body1 = contact->m_body1;
	ndAssert(body0);
	ndAssert(body1);
	ndAssert(body0 != body1);

	contact->m_material = m_contactNotifyCallback->GetMaterial(contact, body0->GetCollisionShape(), body1->GetCollisionShape());
	const ndContactPoint* const contactArray = contactSolver->m_contactBuffer;
	
	ndInt32 count = 0;
	ndVector cachePosition[D_MAX_CONTATCS];
	ndContactPointList::ndNode* nodes[D_MAX_CONTATCS];
	ndContactPointList& contactPointList = contact->m_contacPointsList;
	for (ndContactPointList::ndNode* contactNode = contactPointList.GetFirst(); contactNode; contactNode = contactNode->GetNext()) 
	{
		nodes[count] = contactNode;
		cachePosition[count] = contactNode->GetInfo().m_point;
		count++;
	}
	
	const ndVector& v0 = body0->m_veloc;
	const ndVector& w0 = body0->m_omega;
	const ndVector& com0 = body0->m_globalCentreOfMass;
	
	const ndVector& v1 = body1->m_veloc;
	const ndVector& w1 = body1->m_omega;
	const ndVector& com1 = body1->m_globalCentreOfMass;

	ndVector controlDir0(ndVector::m_zero);
	ndVector controlDir1(ndVector::m_zero);
	ndVector controlNormal(contactArray[0].m_normal);
	ndVector vel0(v0 + w0.CrossProduct(contactArray[0].m_point - com0));
	ndVector vel1(v1 + w1.CrossProduct(contactArray[0].m_point - com1));
	ndVector vRel(vel1 - vel0);
	ndAssert(controlNormal.m_w == ndFloat32(0.0f));
	ndVector tangDir(vRel - controlNormal * vRel.DotProduct(controlNormal));
	ndAssert(tangDir.m_w == ndFloat32(0.0f));
	ndFloat32 diff = tangDir.DotProduct(tangDir).GetScalar();
	
	ndInt32 staticMotion = 0;
	if (diff <= ndFloat32(1.0e-2f)) 
	{
		staticMotion = 1;
		if (ndAbs(controlNormal.m_z) > ndFloat32(0.577f)) 
		{
			tangDir = ndVector(-controlNormal.m_y, controlNormal.m_z, ndFloat32(0.0f), ndFloat32(0.0f));
		}
		else 
		{
			tangDir = ndVector(-controlNormal.m_y, controlNormal.m_x, ndFloat32(0.0f), ndFloat32(0.0f));
		}
		controlDir0 = controlNormal.CrossProduct(tangDir);
		ndAssert(controlDir0.m_w == ndFloat32(0.0f));
		ndAssert(controlDir0.DotProduct(controlDir0).GetScalar() > ndFloat32(1.0e-8f));
		controlDir0 = controlDir0.Normalize();
		controlDir1 = controlNormal.CrossProduct(controlDir0);
		ndAssert(ndAbs(controlNormal.DotProduct(controlDir0.CrossProduct(controlDir1)).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
	}
	
	ndFloat32 maxImpulse = ndFloat32(-1.0f);
	for (ndInt32 i = 0; i < contactCount; ++i) 
	{
		ndInt32 index = -1;
		ndFloat32 min = ndFloat32(1.0e20f);
		ndContactPointList::ndNode* contactNode = nullptr;
		for (ndInt32 j = 0; j < count; ++j) 
		{
			ndVector v(ndVector::m_triplexMask & (cachePosition[j] - contactArray[i].m_point));
			ndAssert(v.m_w == ndFloat32(0.0f));
			diff = v.DotProduct(v).GetScalar();
			if (diff < min) 
			{
				index = j;
				min = diff;
				contactNode = nodes[j];
			}
		}
	
		if (contactNode) 
		{
			count--;
			ndAssert(index != -1);
			nodes[index] = nodes[count];
			cachePosition[index] = cachePosition[count];
		}
		else 
		{
			contactNode = contactPointList.Append();
		}

		ndContactMaterial* const contactPoint = &contactNode->GetInfo();
	
		ndAssert(ndCheckFloat(contactArray[i].m_point.m_x));
		ndAssert(ndCheckFloat(contactArray[i].m_point.m_y));
		ndAssert(ndCheckFloat(contactArray[i].m_point.m_z));
		ndAssert(contactArray[i].m_body0);
		ndAssert(contactArray[i].m_body1);
		ndAssert(contactArray[i].m_shapeInstance0);
		ndAssert(contactArray[i].m_shapeInstance1);
		ndAssert(contactArray[i].m_body0 == body0);
		ndAssert(contactArray[i].m_body1 == body1);
		contactPoint->m_point = contactArray[i].m_point;
		contactPoint->m_normal = contactArray[i].m_normal;
		contactPoint->m_penetration = contactArray[i].m_penetration;
		contactPoint->m_body0 = contactArray[i].m_body0;
		contactPoint->m_body1 = contactArray[i].m_body1;
		contactPoint->m_shapeInstance0 = contactArray[i].m_shapeInstance0;
		contactPoint->m_shapeInstance1 = contactArray[i].m_shapeInstance1;
		contactPoint->m_shapeId0 = contactArray[i].m_shapeId0;
		contactPoint->m_shapeId1 = contactArray[i].m_shapeId1;
		contactPoint->m_material = *contact->m_material;
	
		if (staticMotion) 
		{
			if (contactPoint->m_normal.DotProduct(controlNormal).GetScalar() > ndFloat32(0.9995f)) 
			{
				contactPoint->m_dir0 = controlDir0;
				contactPoint->m_dir1 = controlDir1;
			}
			else 
			{
				if (ndAbs(contactPoint->m_normal.m_z) > ndFloat32(0.577f))
				{
					tangDir = ndVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_z, ndFloat32(0.0f), ndFloat32(0.0f));
				}
				else 
				{
					tangDir = ndVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_x, ndFloat32(0.0f), ndFloat32(0.0f));
				}
				contactPoint->m_dir0 = contactPoint->m_normal.CrossProduct(tangDir);
				ndAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
				ndAssert(contactPoint->m_dir0.DotProduct(contactPoint->m_dir0).GetScalar() > ndFloat32(1.0e-8f));
				contactPoint->m_dir0 = contactPoint->m_dir0.Normalize();
				contactPoint->m_dir1 = contactPoint->m_normal.CrossProduct(contactPoint->m_dir0);
				ndAssert(ndAbs(contactPoint->m_normal.DotProduct(contactPoint->m_dir0.CrossProduct(contactPoint->m_dir1)).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
			}
		}
		else 
		{
			ndVector veloc0(v0 + w0.CrossProduct(contactPoint->m_point - com0));
			ndVector veloc1(v1 + w1.CrossProduct(contactPoint->m_point - com1));
			ndVector relReloc(veloc1 - veloc0);
	
			ndAssert(contactPoint->m_normal.m_w == ndFloat32(0.0f));
			ndFloat32 impulse = relReloc.DotProduct(contactPoint->m_normal).GetScalar();
			if (ndAbs(impulse) > maxImpulse) 
			{
				maxImpulse = ndAbs(impulse);
			}
	
			ndVector tangentDir(relReloc - contactPoint->m_normal.Scale(impulse));
			ndAssert(tangentDir.m_w == ndFloat32(0.0f));
			diff = tangentDir.DotProduct(tangentDir).GetScalar();
			if (diff > ndFloat32(1.0e-2f)) 
			{
				ndAssert(tangentDir.m_w == ndFloat32(0.0f));
				contactPoint->m_dir0 = tangentDir.Normalize();
			}
			else 
			{
				if (ndAbs(contactPoint->m_normal.m_z) > ndFloat32(0.577f)) 
				{
					tangentDir = ndVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_z, ndFloat32(0.0f), ndFloat32(0.0f));
				}
				else 
				{
					tangentDir = ndVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_x, ndFloat32(0.0f), ndFloat32(0.0f));
				}
				contactPoint->m_dir0 = contactPoint->m_normal.CrossProduct(tangentDir);
				ndAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
				ndAssert(contactPoint->m_dir0.DotProduct(contactPoint->m_dir0).GetScalar() > ndFloat32(1.0e-8f));
				contactPoint->m_dir0 = contactPoint->m_dir0.Normalize();
			}
			contactPoint->m_dir1 = contactPoint->m_normal.CrossProduct(contactPoint->m_dir0);
			ndAssert(ndAbs(contactPoint->m_normal.DotProduct(contactPoint->m_dir0.CrossProduct(contactPoint->m_dir1)).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
		}
		ndAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
		ndAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
		ndAssert(contactPoint->m_normal.m_w == ndFloat32(0.0f));
	}
	
	for (ndInt32 i = 0; i < count; ++i) 
	{
		contactPointList.Remove(nodes[i]);
	}
	
	//contact->m_maxDof = ndUnsigned32(3 * contactPointList.GetCount());
	contact->m_maxDof = ndUnsigned8(3 * contactPointList.GetCount());
	m_contactNotifyCallback->OnContactCallback(contact, m_timestep);
}

void ndScene::SubmitPairs(ndBvhLeafNode* const leafNode, ndBvhNode* const node, bool forward, ndInt32 threadId)
{
	ndBvhNode* pool[D_SCENE_MAX_STACK_DEPTH];

	ndBodyKinematic* const body0 = leafNode->GetBody() ? leafNode->GetBody() : nullptr;
	ndAssert(body0);

	const ndVector boxP0(leafNode->m_minBox);
	const ndVector boxP1(leafNode->m_maxBox);
	const ndUnsigned8 test0 = ndUnsigned8(!body0->m_equilibrium);
	const ndUnsigned8 fowardTest = forward ? ndUnsigned8(1) : ndUnsigned8(0);

	ndBodyNotify* const notify = body0->GetNotifyCallback();

	pool[0] = node;
	ndInt32 stack = 1;
	while (stack && (stack < (D_SCENE_MAX_STACK_DEPTH - 16)))
	{
		stack--;
		ndBvhNode* const rootNode = pool[stack];
		if (ndOverlapTest(rootNode->m_minBox, rootNode->m_maxBox, boxP0, boxP1)) 
		{
			if (rootNode->GetAsSceneBodyNode()) 
			{
				ndAssert(!rootNode->GetRight());
				ndAssert(!rootNode->GetLeft());
				
				ndBodyKinematic* const body1 = rootNode->GetBody();
				ndAssert(body1);
				const ndUnsigned8 test = ndUnsigned8((body1->m_sceneEquilibrium | fowardTest) & (test0 | ndUnsigned8(!body1->m_equilibrium)));
				if (test)
				{
					//if (notify->OnSceneAabbOverlap(body1))
					if (!notify || notify->OnSceneAabbOverlap(body1))
					{
						AddPair(body0, body1, threadId);
					}
				}
			}
			else 
			{
				ndBvhInternalNode* const tmpNode = rootNode->GetAsSceneTreeNode();
				ndAssert(tmpNode->m_left);
				ndAssert(tmpNode->m_right);
		
				pool[stack] = tmpNode->m_left;
				stack++;
				ndAssert(stack < ndInt32(sizeof(pool) / sizeof(pool[0])));
		
				pool[stack] = tmpNode->m_right;
				stack++;
				ndAssert(stack < ndInt32(sizeof(pool) / sizeof(pool[0])));
			}
		}
	}

	if (stack)
	{
		m_forceBalanceSceneCounter = 0;
	}
}

ndJointBilateralConstraint* ndScene::FindBilateralJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const
{
	if (body0->m_jointList.GetCount() <= body1->m_jointList.GetCount())
	{
		for (ndBodyKinematic::ndJointList::ndNode* node = body0->m_jointList.GetFirst(); node; node = node->GetNext())
		{
			ndJointBilateralConstraint* const joint = node->GetInfo();
			if ((joint->GetBody0() == body1) || (joint->GetBody1() == body1))
			{
				return joint;
			}
		}
	}
	else
	{
		for (ndBodyKinematic::ndJointList::ndNode* node = body1->m_jointList.GetFirst(); node; node = node->GetNext())
		{
			ndJointBilateralConstraint* const joint = node->GetInfo();
			if ((joint->GetBody0() == body0) || (joint->GetBody1() == body0))
			{
				return joint;
			}
		}
	}
	return nullptr;
}

void ndScene::FindCollidingPairs(ndBodyKinematic* const body, ndInt32 threadId)
{
	ndBvhLeafNode* const bodyNode = m_bvhSceneManager.GetLeafNode(body);
	ndAssert(bodyNode->GetAsSceneBodyNode());
	for (ndBvhNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndBvhInternalNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		ndAssert(!parent->GetAsSceneBodyNode());
		ndBvhNode* const sibling = parent->m_right;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling, true, threadId);
		}
	}
}

void ndScene::FindCollidingPairsForward(ndBodyKinematic* const body, ndInt32 threadId)
{
	ndBvhLeafNode* const bodyNode = m_bvhSceneManager.GetLeafNode(body);
	ndAssert(bodyNode->GetAsSceneBodyNode());
	for (ndBvhNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndBvhInternalNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		ndAssert(!parent->GetAsSceneBodyNode());
		ndBvhNode* const sibling = parent->m_right;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling, true, threadId);
		}
	}
}

void ndScene::FindCollidingPairsBackward(ndBodyKinematic* const body, ndInt32 threadId)
{
	ndBvhLeafNode* const bodyNode = m_bvhSceneManager.GetLeafNode(body);
	ndAssert(bodyNode->GetAsSceneBodyNode());
	for (ndBvhNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndBvhInternalNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		ndAssert(!parent->GetAsSceneBodyNode());
		ndBvhNode* const sibling = parent->m_left;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling, false, threadId);
		}
	}
}

void ndScene::UpdateTransform()
{
	D_TRACKTIME();
	for (ndBodyList::ndNode* node = m_particleSetList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyParticleSet* const particleSet = node->GetInfo()->GetAsBodyParticleSet();
		ndAssert(particleSet);
		ndBodyNotify* const notify = particleSet->GetNotifyCallback();
		if (notify)
		{
			notify->OnTransform(0, particleSet->GetMatrix());
		}
	}

	ndAtomic<ndInt32> iterator(0);
	auto TransformUpdate = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(TransformUpdate);
		const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();

		const ndInt32 count = ndInt32(bodyArray.GetCount()) - 1;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[i + j];
				UpdateTransformNotify(threadIndex, body);
			}
		}
	});
	ParallelExecute(TransformUpdate);
}

void ndScene::CalculateContacts(ndInt32 threadIndex, ndContact* const contact)
{
	const ndUnsigned32 lru = m_lru - D_CONTACT_DELAY_FRAMES;

	ndVector deltaTime(m_timestep);
	ndBodyKinematic* const body0 = contact->GetBody0();
	ndBodyKinematic* const body1 = contact->GetBody1();

	ndAssert(!contact->m_isDead);
	if (!(body0->m_equilibrium & body1->m_equilibrium))
	{
		bool active = contact->IsActive();
		if (ValidateContactCache(contact, deltaTime))
		{
			contact->m_sceneLru = m_lru;
			contact->m_timeOfImpact = ndFloat32(1.0e10f);
		}
		else
		{
			contact->SetActive(false);
			contact->m_positAcc = ndVector::m_zero;
			contact->m_rotationAcc = ndQuaternion();

			ndFloat32 distance = contact->m_separationDistance;
			if (distance >= D_NARROW_PHASE_DIST)
			{
				const ndVector veloc0(body0->GetVelocity());
				const ndVector veloc1(body1->GetVelocity());
				
				const ndVector veloc(veloc1 - veloc0);
				const ndVector omega0(body0->GetOmega());
				const ndVector omega1(body1->GetOmega());
				const ndShapeInstance* const collision0 = &body0->GetCollisionShape();
				const ndShapeInstance* const collision1 = &body1->GetCollisionShape();
				const ndVector scale(ndFloat32(1.0f), ndFloat32(3.5f) * collision0->GetBoxMaxRadius(), ndFloat32(3.5f) * collision1->GetBoxMaxRadius(), ndFloat32(0.0f));
				const ndVector velocMag2(veloc.DotProduct(veloc).GetScalar(), omega0.DotProduct(omega0).GetScalar(), omega1.DotProduct(omega1).GetScalar(), ndFloat32(0.0f));
				const ndVector velocMag(velocMag2.GetMax(ndVector::m_epsilon).InvSqrt() * velocMag2 * scale);
				const ndFloat32 speed = velocMag.AddHorizontal().GetScalar() + ndFloat32(0.5f);
				
				distance -= speed * m_timestep;
				contact->m_separationDistance = distance;
			}
			if (distance < D_NARROW_PHASE_DIST)
			{
				CalculateJointContacts(threadIndex, contact);
				if (contact->m_maxDof || contact->m_isIntersetionTestOnly)
				{
					contact->SetActive(true);
					contact->m_timeOfImpact = ndFloat32(1.0e10f);
				}
				contact->m_sceneLru = m_lru;
			}
			else
			{
				const ndBvhLeafNode* const bodyNode0 = m_bvhSceneManager.GetLeafNode(contact->GetBody0());
				const ndBvhLeafNode* const bodyNode1 = m_bvhSceneManager.GetLeafNode(contact->GetBody1());
				ndAssert(bodyNode0 && bodyNode0->GetAsSceneBodyNode());
				ndAssert(bodyNode1 && bodyNode1->GetAsSceneBodyNode());
				if (ndOverlapTest(bodyNode0->m_minBox, bodyNode0->m_maxBox, bodyNode1->m_minBox, bodyNode1->m_maxBox)) 
				{
					contact->m_sceneLru = m_lru;
				}
				else if (contact->m_sceneLru < lru) 
				{
					contact->m_isDead = 1;
				}
			}
		}

		if (active ^ contact->IsActive())
		{
			ndAssert(body0->GetInvMass() > ndFloat32(0.0f));
			body0->m_equilibrium = 0;
			if (body1->GetInvMass() > ndFloat32(0.0f))
			{
				body1->m_equilibrium = 0;
			}
		}
	}
	else
	{
		contact->m_sceneLru = m_lru;
	}

	if (!contact->m_isDead && (body0->m_equilibrium & body1->m_equilibrium & !contact->IsActive()))
	{
		const ndBvhLeafNode* const bodyNode0 = m_bvhSceneManager.GetLeafNode(contact->GetBody0());
		const ndBvhLeafNode* const bodyNode1 = m_bvhSceneManager.GetLeafNode(contact->GetBody1());
		ndAssert(bodyNode0->GetAsSceneBodyNode());
		ndAssert(bodyNode1->GetAsSceneBodyNode());
		if (!ndOverlapTest(bodyNode0->m_minBox, bodyNode0->m_maxBox, bodyNode1->m_minBox, bodyNode1->m_maxBox))
		{
			contact->m_isDead = 1;
		}
	}
}

void ndScene::UpdateSpecial()
{
	for (ndSpecialList<ndBodyKinematic>::ndNode* node = m_specialUpdateList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo();
		body->SpecialUpdate(m_timestep);
	}
}

bool ndScene::ConvexCast(ndConvexCastNotify& callback, const ndBvhNode** stackPool, ndFloat32* const stackDistance, ndInt32 stack, const ndFastRay& ray, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const
{
	ndVector boxP0;
	ndVector boxP1;

	ndAssert(globalOrigin.TestOrthogonal());
	convexShape.CalculateAabb(globalOrigin, boxP0, boxP1);
	
	callback.m_contacts.SetCount(0);
	callback.m_param = ndFloat32(1.2f);
	callback.m_cachedScene = (ndScene*)this;
	while (stack && (stack < (D_SCENE_MAX_STACK_DEPTH - 4)))
	{
		stack--;
		ndFloat32 dist = stackDistance[stack];
		
		if (dist > callback.m_param)
		{
			break;
		}
		else 
		{
			const ndBvhNode* const me = stackPool[stack];
		
			ndBody* const body = me->GetBody();
			if (body) 
			{
				if (callback.OnRayPrecastAction (body, &convexShape)) 
				{
					// save contacts and try new set
					ndConvexCastNotify savedNotification(callback);
					ndBodyKinematic* const kinBody = body->GetAsBodyKinematic();
					callback.m_contacts.SetCount(0);
					if (callback.CastShape(convexShape, globalOrigin, globalDest, kinBody))
					{
						// found new contacts, see how the are managed
						if (ndAbs(savedNotification.m_param - callback.m_param) < ndFloat32(-1.0e-3f))
						{
							// merge contact
							for (ndInt32 i = 0; i < savedNotification.m_contacts.GetCount(); ++i)
							{
								const ndContactPoint& contact = savedNotification.m_contacts[i];
								bool newPoint = true;
								for (ndInt32 j = callback.m_contacts.GetCount() - 1; j >= 0; ++j)
								{
									const ndVector diff(callback.m_contacts[j].m_point - contact.m_point);
									ndFloat32 mag2 = diff.DotProduct(diff & ndVector::m_triplexMask).GetScalar();
									newPoint = newPoint & (mag2 > ndFloat32(1.0e-5f));
								}
								if (newPoint && (callback.m_contacts.GetCount() < callback.m_contacts.GetCapacity()))
								{
									callback.m_contacts.PushBack(contact);
								}
							}
						}
						else if (callback.m_param > savedNotification.m_param)
						{
							// restore contacts
							callback.m_normal = savedNotification.m_normal;
							callback.m_closestPoint0 = savedNotification.m_closestPoint0;
							callback.m_closestPoint1 = savedNotification.m_closestPoint1;
							callback.m_param = savedNotification.m_param;
							for (ndInt32 i = 0; i < savedNotification.m_contacts.GetCount(); ++i)
							{
								callback.m_contacts[i] = savedNotification.m_contacts[i];
							}
						}
					}
					else
					{
						// no new contacts restore old ones,
						// in theory it should no copy, by the notification may change
						// the previous found contacts
						callback.m_normal = savedNotification.m_normal;
						callback.m_closestPoint0 = savedNotification.m_closestPoint0;
						callback.m_closestPoint1 = savedNotification.m_closestPoint1;
						callback.m_param = savedNotification.m_param;
						for (ndInt32 i = 0; i < savedNotification.m_contacts.GetCount(); ++i)
						{
							callback.m_contacts[i] = savedNotification.m_contacts[i];
						}
					}

					if (callback.m_param < ndFloat32 (1.0e-8f)) 
					{
						break;
					}
				}
			}
			else 
			{
				{
					const ndBvhNode* const left = me->GetLeft();
					ndAssert(left);
					const ndVector minBox(left->m_minBox - boxP1);
					const ndVector maxBox(left->m_maxBox - boxP0);
					ndFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
					if (dist1 < callback.m_param)
					{
						ndInt32 j = stack;
						for (; j && (dist1 > stackDistance[j - 1]); j--)
						{
							stackPool[j] = stackPool[j - 1];
							stackDistance[j] = stackDistance[j - 1];
						}
						stackPool[j] = left;
						stackDistance[j] = dist1;
						stack++;
						ndAssert(stack < D_SCENE_MAX_STACK_DEPTH);
					}
				}
		
				{
					const ndBvhNode* const right = me->GetRight();
					ndAssert(right);
					const ndVector minBox(right->m_minBox - boxP1);
					const ndVector maxBox = right->m_maxBox - boxP0;
					ndFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
					if (dist1 < callback.m_param)
					{
						ndInt32 j = stack;
						for (; j && (dist1 > stackDistance[j - 1]); j--) 
						{
							stackPool[j] = stackPool[j - 1];
							stackDistance[j] = stackDistance[j - 1];
						}
						stackPool[j] = right;
						stackDistance[j] = dist1;
						stack++;
						ndAssert(stack < D_SCENE_MAX_STACK_DEPTH);
					}
				}
			}
		}
	}

	callback.m_cachedScene = nullptr;
	return callback.m_contacts.GetCount() > 0;
}

bool ndScene::RayCast(ndRayCastNotify& callback, const ndBvhNode** stackPool, ndFloat32* const stackDistance, ndInt32 stack, const ndFastRay& ray) const
{
	bool state = false;
	while (stack && (stack < (D_SCENE_MAX_STACK_DEPTH - 4)))
	{
		stack--;
		ndFloat32 dist = stackDistance[stack];
		if (dist > callback.m_param)
		{
			break;
		}
		else
		{
			const ndBvhNode* const me = stackPool[stack];
			ndAssert(me);
			ndBodyKinematic* const body = me->GetBody();
			if (body)
			{
				ndAssert(!me->GetLeft());
				ndAssert(!me->GetRight());

				if (body->RayCast(callback, ray, callback.m_param))
				{
					state = true;
					if (callback.m_param < ndFloat32(1.0e-8f))
					{
						break;
					}
				}
			}
			else
			{
				const ndBvhNode* const left = me->GetLeft();
				ndAssert(left);
				ndFloat32 dist1 = ray.BoxIntersect(left->m_minBox, left->m_maxBox);
				if (dist1 < callback.m_param)
				{
					ndInt32 j = stack;
					for (; j && (dist1 > stackDistance[j - 1]); j--)
					{
						stackPool[j] = stackPool[j - 1];
						stackDistance[j] = stackDistance[j - 1];
					}
					stackPool[j] = left;
					stackDistance[j] = dist1;
					stack++;
					ndAssert(stack < D_SCENE_MAX_STACK_DEPTH);
				}
	
				const ndBvhNode* const right = me->GetRight();
				ndAssert(right);
				dist1 = ray.BoxIntersect(right->m_minBox, right->m_maxBox);
				if (dist1 < callback.m_param)
				{
					ndInt32 j = stack;
					for (; j && (dist1 > stackDistance[j - 1]); j--)
					{
						stackPool[j] = stackPool[j - 1];
						stackDistance[j] = stackDistance[j - 1];
					}
					stackPool[j] = right;
					stackDistance[j] = dist1;
					stack++;
					ndAssert(stack < D_SCENE_MAX_STACK_DEPTH);
				}
			}
		}
	}
	return state;
}

void ndScene::BodiesInAabb(ndBodiesInAabbNotify& callback, const ndVector& minBox, const ndVector& maxBox) const
{
	callback.Reset();
	if (m_rootNode)
	{
		const ndBvhNode* stackPool[D_SCENE_MAX_STACK_DEPTH];
		stackPool[0] = m_rootNode;
		ndInt32 stack = 1;
		while (stack && (stack < (D_SCENE_MAX_STACK_DEPTH - 4)))
		{
			stack--;
			
			const ndBvhNode* const rootNode = stackPool[stack];
			ndAssert(rootNode);
			if (ndOverlapTest(rootNode->m_minBox, rootNode->m_maxBox, minBox, maxBox))
			{
				ndBodyKinematic* const body = rootNode->GetBody();
				if (body)
				{
					ndAssert(!rootNode->GetLeft());
					ndAssert(!rootNode->GetRight());
					if (ndOverlapTest(body->m_minAabb, body->m_maxAabb, minBox, maxBox))
					{
						callback.OnOverlap(body);
					}
				}
				else
				{
					const ndBvhNode* const left = rootNode->GetLeft();
					ndAssert(left);
					stackPool[stack] = left;
					stack++;
					ndAssert(stack < D_SCENE_MAX_STACK_DEPTH);

					const ndBvhNode* const right = rootNode->GetRight();
					ndAssert(right);
					stackPool[stack] = right;
					stack++;
					ndAssert(stack < D_SCENE_MAX_STACK_DEPTH);
				}
			}
		}
	}
}

void ndScene::Cleanup()
{
	Sync();
	if (m_backgroundThread)
	{
		m_backgroundThread->Terminate();
	}
	PrepareCleanup();
	
	m_frameNumber = 0;
	m_subStepNumber = 0;

	if (m_sentinelBody)
	{
		delete m_sentinelBody;
		m_sentinelBody = nullptr;
	}

	m_bvhSceneManager.CleanUp();
	m_contactArray.DeleteAllContacts();

	ndFreeListAlloc::Flush();
	m_sceneBodyArray.Resize(1024);
	m_activeConstraintArray.Resize(1024);
	m_scratchBuffer.Resize(1024 * sizeof(void*));

	m_scratchBuffer.SetCount(0);
	m_sceneBodyArray.SetCount(0);
	m_activeConstraintArray.SetCount(0);
}

bool ndScene::RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const
{
	const ndVector p0(globalOrigin & ndVector::m_triplexMask);
	const ndVector p1(globalDest & ndVector::m_triplexMask);

	bool state = false;
	callback.m_param = ndFloat32(1.2f);
	if (m_rootNode)
	{
		const ndVector segment(p1 - p0);
		ndFloat32 dist2 = segment.DotProduct(segment).GetScalar();
		if (dist2 > ndFloat32(1.0e-8f))
		{
			ndFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
			const ndBvhNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

			ndFastRay ray(p0, p1);

			stackPool[0] = m_rootNode;
			distance[0] = ray.BoxIntersect(m_rootNode->m_minBox, m_rootNode->m_maxBox);
			state = RayCast(callback, stackPool, distance, 1, ray);
		}
	}
	return state;
}

bool ndScene::ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const
{
	bool state = false;
	callback.m_param = ndFloat32(1.2f);
	if (m_rootNode)
	{
		ndVector boxP0;
		ndVector boxP1;
		ndAssert(globalOrigin.TestOrthogonal());
		convexShape.CalculateAabb(globalOrigin, boxP0, boxP1);

		ndFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
		const ndBvhNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

		const ndVector velocB(ndVector::m_zero);
		const ndVector velocA((globalDest - globalOrigin.m_posit) & ndVector::m_triplexMask);
		const ndVector minBox(m_rootNode->m_minBox - boxP1);
		const ndVector maxBox(m_rootNode->m_maxBox - boxP0);
		ndFastRay ray(ndVector::m_zero, velocA);

		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);
		state = ConvexCast(callback, stackPool, distance, 1, ray, convexShape, globalOrigin, globalDest);
	}
	return state;
}

void ndScene::SendBackgroundTask(ndBackgroundTask* const job)
{
	if (m_backgroundThread)
	{
		m_backgroundThread->SendTask(job);
	}
	else
	{
		ndAssert(0);
		delete job;
	}
}

void ndScene::AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1, ndInt32 threadId)
{
	const ndBodyKinematic::ndContactMap& contactMap0 = body0->GetContactMap();
	const ndBodyKinematic::ndContactMap& contactMap1 = body1->GetContactMap();

	ndContact* const contact = (contactMap0.GetCount() <= contactMap1.GetCount()) ? contactMap0.FindContact(body0, body1) : contactMap1.FindContact(body1, body0);
	if (!contact)
	{
		const ndJointBilateralConstraint* const bilateral = FindBilateralJoint(body0, body1);
		const bool isCollidable = bilateral ? bilateral->IsCollidable() : true;
		if (isCollidable)
		{
			bool selfSkelCollidable = body0->GetSeletonSelfCollision() && body1->GetSeletonSelfCollision();
			if (!selfSkelCollidable)
			{
				const ndSkeletonContainer* skel0 = body0->GetSkeleton();
				const ndSkeletonContainer* skel1 = body1->GetSkeleton();
				auto FindMissingSkeletorm = [](const ndSkeletonContainer* const skeleton, const ndBodyKinematic* const body)
				{
					bool test = (body->GetInvMass() == ndFloat32(0.0f)) && (skeleton->FindBoneIndex(body) != -1);
					return test ? skeleton : nullptr;
				};

				if (skel0 && !skel1)
				{
					skel1 = FindMissingSkeletorm(skel0, body1);
				}
				else if (!skel0 && skel1)
				{
					skel0 = FindMissingSkeletorm(skel1, body0);
				}
				selfSkelCollidable = !(skel0 && skel1 && (skel0 == skel1));
			}
			if (selfSkelCollidable)
			{
				ndArray<ndContactPairs>& particalPairs = m_partialNewPairs[threadId];
				ndContactPairs pair(ndUnsigned32(body0->m_index), ndUnsigned32(body1->m_index));
				particalPairs.PushBack(pair);
			}
		}
	}
}

void ndScene::FindCollidingPairs()
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator0(0);
	auto FindPairsForward = ndMakeObject::ndFunction([this, &iterator0](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(FindPairsForward);
		const ndArray<ndBodyKinematic*>& bodyArray = m_sceneBodyArray;

		const ndInt32 count = ndInt32(m_sceneBodyArray.GetCount());
		for (ndInt32 i = iterator0.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator0.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[i + j];
				FindCollidingPairsForward(body, threadIndex);
			}
		}
	});

	ndAtomic<ndInt32> iterator1(0);
	auto FindPairsBackward = ndMakeObject::ndFunction([this, &iterator1](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(FindPairsBackward);
		const ndArray<ndBodyKinematic*>& bodyArray = m_sceneBodyArray;

		const ndInt32 count = ndInt32(m_sceneBodyArray.GetCount());
		for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[i + j];
				FindCollidingPairsBackward(body, threadIndex);
			}
		}
	});

	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		m_partialNewPairs[i].SetCount(0);
	}

	const ndInt32 threadCount = GetThreadCount();

	ParallelExecute(FindPairsForward);
	ParallelExecute(FindPairsBackward);

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		sum += ndInt32(m_partialNewPairs[i].GetCount());
	}
	m_newPairs.SetCount(sum);

	sum = 0;
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		const ndArray<ndContactPairs>& newPairs = m_partialNewPairs[i];
		const ndInt32 count = ndInt32(newPairs.GetCount());
		if (count)
		{
			ndMemCpy(&m_newPairs[sum], &newPairs[0], count);
			sum += count;
		}
	}
}

void ndScene::UpdateBodyList()
{
	if (m_bodyList.UpdateView())
	{
		ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
		view.PushBack(m_sentinelBody);
	}
}

void ndScene::ApplyExtForce()
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator(0);
	auto ApplyForce = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(ApplyForce);
		const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();

		const ndFloat32 timestep = m_timestep;

		const ndInt32 count = ndInt32(view.GetCount()) - 1;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = view[i + j];
				body->ApplyExternalForces(threadIndex, timestep);
			}
		}
	});
	ParallelExecute(ApplyForce);
}

void ndScene::InitBodyArray()
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator(0);
	auto BuildBodyArray = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(BuildBodyArray);
		const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();

		ndBvhNodeArray& array = m_bvhSceneManager.GetNodeArray();
		const ndInt32 count = ndInt32(view.GetCount()) - 1;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = view[i + j];
				body->PrepareStep(i + j);
				ndUnsigned8 sceneEquilibrium = 1;
				ndUnsigned8 sceneForceUpdate = body->m_sceneForceUpdate;
				ndUnsigned8 moving = ndUnsigned8(!body->m_equilibrium);
				if (moving | sceneForceUpdate)
				{
					ndBvhLeafNode* const bodyNode = (ndBvhLeafNode*)array[body->m_bodyNodeIndex];
					ndAssert(bodyNode->GetAsSceneBodyNode());
					ndAssert(bodyNode->m_body == body);
					ndAssert(!bodyNode->GetLeft());
					ndAssert(!bodyNode->GetRight());

					body->UpdateCollisionMatrix();
					const ndInt32 test = ndBoxInclusionTest(body->m_minAabb, body->m_maxAabb, bodyNode->m_minBox, bodyNode->m_maxBox);
					if (!test)
					{
						bodyNode->SetAabb(body->m_minAabb, body->m_maxAabb);
					}
					sceneEquilibrium = ndUnsigned8(!sceneForceUpdate & (test != 0));
				}
				body->m_sceneForceUpdate = 0;
				body->m_sceneEquilibrium = sceneEquilibrium;
			}
		}
	});

	ParallelExecute(BuildBodyArray);

	ndUnsigned32 scans[4];
	class ndSortCompactKey
	{
		public:
		ndSortCompactKey(const void* const)
		{
		}

		ndInt32 GetKey(const ndBodyKinematic* const body) const
		{
			return body->m_sceneEquilibrium;
		}
	};

	ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
	ndInt32 movingBodyCount = 0;
	ndInt32 sceneBodyCount = ndInt32(view.GetCount()) - 1;
	if (sceneBodyCount)
	{
		m_sceneBodyArray.SetCount(sceneBodyCount);
		ndCountingSort<ndBodyKinematic*, ndSortCompactKey, 1>(*this, &view[0], &m_sceneBodyArray[0], sceneBodyCount, scans, nullptr);
		movingBodyCount = ndInt32(scans[1] - scans[0]);
		m_sceneBodyArray.SetCount(movingBodyCount);
	}

	if (m_rootNode && m_rootNode->GetAsSceneTreeNode())
	{
		const ndInt32 bodyCount = m_bodyList.GetCount();
		const ndInt32 cutoffCount = (ndExp2(bodyCount) + 1) * movingBodyCount;
		if (cutoffCount < bodyCount)
		{
			ndAtomic<ndInt32> iterator1(0);
			auto UpdateSceneBvh = ndMakeObject::ndFunction([this, &iterator1](ndInt32, ndInt32)
			{
				D_TRACKTIME_NAMED(UpdateSceneBvh);
				const ndArray<ndBodyKinematic*>& view = m_sceneBodyArray;
				ndBvhNodeArray& array = m_bvhSceneManager.GetNodeArray();

				const ndInt32 count = ndInt32(view.GetCount());
				for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
				{
					const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
					for (ndInt32 j = 0; j < maxSpan; ++j)
					{
						ndBodyKinematic* const body = view[i + j];
						ndBvhLeafNode* const bodyNode = (ndBvhLeafNode*)array[body->m_bodyNodeIndex];
						ndAssert(bodyNode->GetAsSceneBodyNode());
						ndAssert(bodyNode->GetBody() == body);

						const ndBvhNode* const root = (m_rootNode->GetLeft() && m_rootNode->GetRight()) ? nullptr : m_rootNode;
						ndAssert(root == nullptr);
						for (ndBvhInternalNode* parent = (ndBvhInternalNode*)bodyNode->m_parent; parent != root; parent = (ndBvhInternalNode*)parent->m_parent)
						{
							ndAssert(parent->GetAsSceneTreeNode());
							ndScopeSpinLock lock(parent->m_lock);
							const ndVector minBox(parent->m_left->m_minBox.GetMin(parent->m_right->m_minBox));
							const ndVector maxBox(parent->m_left->m_maxBox.GetMax(parent->m_right->m_maxBox));
							if (ndBoxInclusionTest(minBox, maxBox, parent->m_minBox, parent->m_maxBox))
							{
								break;
							}
							parent->m_minBox = minBox;
							parent->m_maxBox = maxBox;
						}
					}
				}
			});
	
			D_TRACKTIME_NAMED(UpdateSceneBvhLight);
			ParallelExecute(UpdateSceneBvh);
		}
		else
		{
			m_bvhSceneManager.UpdateScene(*this);
		}
	}
	
	ndBodyKinematic* const sentinelBody = m_sentinelBody;
	sentinelBody->PrepareStep(ndInt32(GetActiveBodyArray().GetCount()) - 1);
	
	sentinelBody->m_isStatic = 1;
	sentinelBody->m_autoSleep = 1;
	sentinelBody->m_equilibrium = 1;
	sentinelBody->m_equilibrium0 = 1;
	sentinelBody->m_isJointFence0 = 1;
	sentinelBody->m_isJointFence1 = 1;
	sentinelBody->m_isConstrained = 0;
	sentinelBody->m_sceneEquilibrium = 1;
	sentinelBody->m_weigh = ndFloat32(0.0f);
}

void ndScene::CreateNewContacts()
{
	D_TRACKTIME();
	const ndInt32 contactCount = ndInt32(m_contactArray.GetCount());
	m_scratchBuffer.SetCount(ndInt32((contactCount + m_newPairs.GetCount() + 16) * sizeof(ndContact*)));

	ndContact** const tmpJointsArray = (ndContact**)&m_scratchBuffer[0];
	if (m_newPairs.GetCount())
	{ 
		ndAtomic<ndInt32> iterator(0);
		auto CreateNewContacts = ndMakeObject::ndFunction([this, &iterator, tmpJointsArray](ndInt32, ndInt32)
		{
			D_TRACKTIME_NAMED(CreateNewContacts);
			const ndArray<ndContactPairs>& newPairs = m_newPairs;
			ndBodyKinematic** const bodyArray = &GetActiveBodyArray()[0];

			const ndInt32 count = ndInt32(newPairs.GetCount());
			for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
			{
				const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
				for (ndInt32 j = 0; j < maxSpan; ++j)
				{
					const ndContactPairs& pair = newPairs[i + j];
					ndBodyKinematic* const body0 = bodyArray[pair.m_body0];
					ndBodyKinematic* const body1 = bodyArray[pair.m_body1];
					ndAssert(ndUnsigned32(body0->m_index) == pair.m_body0);
					ndAssert(ndUnsigned32(body1->m_index) == pair.m_body1);

					ndContact* const contact = new ndContact;
					contact->SetBodies(body0, body1);
					contact->AttachToBodies();

					ndAssert(contact->m_body0->GetInvMass() != ndFloat32(0.0f));
					contact->m_material = m_contactNotifyCallback->GetMaterial(contact, body0->GetCollisionShape(), body1->GetCollisionShape());
					tmpJointsArray[i + j] = contact;
				}
			}
		});
		ParallelExecute(CreateNewContacts);
	}

	if (contactCount)
	{
		D_TRACKTIME_NAMED(CopyContactArray)
		const ndInt32 start = ndInt32(m_newPairs.GetCount());
		ndContact** const contactArray = &m_contactArray[0];
		for (ndInt32 i = 0; i < contactCount; ++i)
		{
			ndContact* const contact = contactArray[i];
			tmpJointsArray[start + i] = contact;
		}
	}
}

void ndScene::CalculateContacts()
{
	D_TRACKTIME();
	m_activeConstraintArray.SetCount(0);
	ndScopeSpinLock lock(m_contactArray.GetLock());
	const ndInt32 contactCount = ndInt32(m_contactArray.GetCount() + m_newPairs.GetCount());
	m_contactArray.SetCount(contactCount);
	if (contactCount)
	{
		ndContact** const tmpJointsArray = (ndContact**)&m_scratchBuffer[0];

		ndAtomic<ndInt32> iterator(0);
		auto CalculateContactPoints = ndMakeObject::ndFunction([this, &iterator, tmpJointsArray](ndInt32 threadIndex, ndInt32)
		{
			D_TRACKTIME_NAMED(CalculateContactPoints);

			const ndInt32 jointCount = ndInt32(m_contactArray.GetCount());
			for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < jointCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
			{
				const ndInt32 maxSpan = ((jointCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : jointCount - i;
				for (ndInt32 j = 0; j < maxSpan; ++j)
				{
					ndContact* const contact = tmpJointsArray[i + j];
					ndAssert(contact);
					if (!contact->m_isDead)
					{
						CalculateContacts(threadIndex, contact);
					}
				}
			}
		});
		ParallelExecute(CalculateContactPoints);
	}
}

void ndScene::DeleteDeadContacts()
{
	enum ndPairGroup
	{
		m_active,
		m_inactive,
		m_dead,
	};

	class ndJointActive
	{
		public:
		ndJointActive(void* const)
		{
			m_code[0] = m_active;
			m_code[1] = m_inactive;
			m_code[2] = m_dead;
			m_code[3] = m_dead;
		}

		ndInt32 GetKey(const ndContact* const contact) const
		{
			//const ndUnsigned32 inactive = ndUnsigned32(!contact->IsActive() | (contact->m_maxDOF ? 0 : 1));
			const ndUnsigned32 inactive = ndUnsigned32(!contact->IsActive() | (contact->m_maxDof ? 0 : 1));
			const ndUnsigned32 idDead = contact->m_isDead;
			return m_code[idDead * 2 + inactive];
		}
		ndInt32 m_code[4];
	};
	ndUnsigned32 prefixScan[5];

	ndScopeSpinLock lock(m_contactArray.GetLock());
	if (m_contactArray.GetCount())
	{
		D_TRACKTIME();
		ndContact** const tmpJointsArray = (ndContact**)&m_scratchBuffer[0];
		ndCountingSort<ndContact*, ndJointActive, 2>(*this, tmpJointsArray, &m_contactArray[0], ndInt32(m_contactArray.GetCount()), prefixScan, nullptr);
		if (prefixScan[m_dead + 1] != prefixScan[m_dead])
		{
			ndAtomic<ndInt32> iterator(0);
			auto DeleteContactArray = ndMakeObject::ndFunction([this, &iterator, &prefixScan](ndInt32, ndInt32)
			{
				D_TRACKTIME_NAMED(DeleteContactArray);
				ndArray<ndContact*>& contactArray = m_contactArray;

				const ndInt32 start = ndInt32(prefixScan[m_dead]);
				const ndInt32 count = ndInt32(prefixScan[m_dead + 1] - start);
				for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
				{
					const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
					for (ndInt32 j = 0; j < maxSpan; ++j)
					{
						ndContact* const contact = contactArray[start + i + j];
						ndAssert(contact->m_isDead);
						if (contact->m_isAttached)
						{
							contact->DetachFromBodies();
						}
						delete contact;
					}
				}
			});

			ParallelExecute(DeleteContactArray);
			m_contactArray.SetCount(ndInt32(prefixScan[m_inactive + 1]));
		}

		m_activeConstraintArray.SetCount(ndInt32(prefixScan[m_active + 1]));
		if (m_activeConstraintArray.GetCount())
		{
			ndConstraint** constraintArray = (ndConstraint**)& m_contactArray[0];
			ndMemCpy(&m_activeConstraintArray[0], constraintArray, m_activeConstraintArray.GetCount());
		}
	}
}

void ndScene::ParticleUpdate(ndFloat32 timestep)
{
	D_TRACKTIME();
	for (ndBodyList::ndNode* node = m_particleSetList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyParticleSet* const body = node->GetInfo()->GetAsBodyParticleSet();
		body->Update(this, timestep);
	}
}

bool ndScene::ValidateScene()
{
	m_bodyList.m_listIsDirty = true;
	UpdateBodyList();

	ndTree<const ndBodyKinematic*, const ndBodyKinematic*> filter;

	ndInt32 index = 0;
	const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
	if (view[view.GetCount() - 1] != m_sentinelBody)
	{
		return false;
	}
	for (ndBodyListView::ndNode* node = m_bodyList.GetFirst(); node; node = node->GetNext())
	{
		const ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		if (body != view[index])
		{
			return false;
		}
		filter.Insert(body, body);
		index++;
	}

	if (index != ndInt32(view.GetCount() - 1))
	{
		return false;
	}

	filter.Insert(m_sentinelBody, m_sentinelBody);
	for (ndJointList::ndNode* node = GetWorld()->GetJointList().GetFirst(); node; node = node->GetNext())
	{
		const ndConstraint* const joint = *node->GetInfo();
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();
		if (!filter.Find(body0))
		{
			return false;
		}
		if (!filter.Find(body1))
		{
			return false;
		}
	}

	return true;
}