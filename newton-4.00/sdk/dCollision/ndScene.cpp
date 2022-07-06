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
#include "ndScene.h"
#include "ndShapeNull.h"
#include "ndBodyNotify.h"
#include "ndShapeCompound.h"
#include "ndBodyKinematic.h"
#include "ndContactNotify.h"
#include "ndContactSolver.h"
#include "ndRayCastNotify.h"
#include "ndConvexCastNotify.h"
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

ndScene::ndFitnessList::ndFitnessList()
	:ndArray<ndSceneNode*>(1024)
	,m_buildArray(1024)
	,m_scansCount(0)
	,m_isDirty(true)
{
}

ndScene::ndFitnessList::ndFitnessList(const ndFitnessList& src)
	:ndArray<ndSceneNode*>(1024)
	,m_buildArray(1024)
	,m_scansCount(0)
	,m_isDirty(true)
{
	Swap((ndFitnessList&)src);
	m_buildArray.Swap(((ndFitnessList&)src).m_buildArray);
}

ndScene::ndFitnessList::~ndFitnessList()
{
	CleanUp();
}

void ndScene::ndFitnessList::AddNode(ndSceneNode* const node)
{
	m_isDirty = true;
	node->m_isDead = 0;
	PushBack(node);
	m_buildArray.PushBack(node->Clone());
}

void ndScene::ndFitnessList::CleanUp()
{
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		ndSceneNode* const node = (*this)[i];
		dAssert(node->m_isDead);
		delete node;
	}
	
	for (ndInt32 i = m_buildArray.GetCount() - 1; i >= 0; --i)
	{
		ndSceneNode* const node = m_buildArray[i];
		delete node;
	}
	SetCount(0);
	m_buildArray.SetCount(0);
}

void ndScene::ndFitnessList::Update(ndThreadPool& threadPool)
{
	if (m_isDirty && GetCount())
	{
		D_TRACKTIME();
		const ndInt32 count = GetCount();
		SetCount(count * 2);
		ndSceneNode** const src = &(*this)[0];
		ndSceneNode** const tmp = &src[count];
		
		class ndSortSceneNodeKey
		{
			public:
			ndSortSceneNodeKey(const void* const)
			{
				m_keyCode[0] = 0;
				m_keyCode[1] = 1;
				m_keyCode[2] = 2;
				m_keyCode[3] = 2;
			}
		
			ndUnsigned32 GetKey(const ndSceneNode* const node) const
			{
				ndUnsigned32 code = node->m_isDead * 2 + (((ndSceneNode*)node)->GetAsSceneBodyNode() ? 1 : 0);
				return m_keyCode[code];
			}

			ndUnsigned32 m_keyCode[3];
		};
		
		ndUnsigned32 scans[5];
		ndCountingSortInPlace<ndSceneNode*, ndSortSceneNodeKey, 2>(threadPool, src, tmp, count, scans, nullptr);
		
		const ndInt32 alivedStart = scans[2];
		const ndInt32 deadCount = scans[3] - alivedStart;
		for (ndInt32 i = 0; i < deadCount; i++)
		{
			ndSceneNode* const node = (*this)[alivedStart + i];
			delete node;
		}
		SetCount(alivedStart);

		if (GetCount())
		{
			auto EnumerateNodes = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(MarkCellBounds);
				const ndInt32 baseCount = GetCount() / 2;
				ndSceneNode** const nodes = &(*this)[0];
				const ndStartEnd startEnd(baseCount, threadIndex, threadCount);

				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndSceneTreeNode* const sceneNode = (ndSceneTreeNode*)nodes[i];
					ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)nodes[baseCount + i];
					dAssert(bodyNode->GetAsSceneBodyNode());
					dAssert(sceneNode->GetAsSceneTreeNode());

					ndBodyKinematic* const body = bodyNode->m_body;
					body->m_sceneNodeIndex = i;
					body->m_bodyNodeIndex = baseCount + i;
				}
			});
			threadPool.ParallelExecute(EnumerateNodes);
		}

		m_isDirty = 0;
	}
}

ndScene::ndScene()
	:ndThreadPool("newtonWorker")
	,m_bodyList()
	,m_contactArray()
	,m_scratchBuffer(1024 * sizeof (void*))
	,m_sceneBodyArray(1024)
	,m_activeConstraintArray(1024)
	,m_specialUpdateList()
	,m_backgroundThread()
	,m_newPairs(1024)
#ifdef D_NEW_SCENE
	,m_bvhBuildState()
#else
	,m_cellBuffer0(1024)
	,m_cellBuffer1(1024)
	,m_cellCounts0(1024)
	,m_cellCounts1(1024)
#endif
	,m_lock()
	,m_rootNode(nullptr)
	,m_sentinelBody(nullptr)
	,m_contactNotifyCallback(new ndContactNotify())
	,m_fitness()
	,m_timestep(ndFloat32 (0.0f))
	,m_lru(D_CONTACT_DELAY_FRAMES)
	,m_frameIndex(0)
	,m_subStepIndex(0)
	,m_forceBalanceSceneCounter(0)
{
	m_sentinelBody = new ndBodySentinel;
	m_contactNotifyCallback->m_scene = this;

	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
	{
		m_partialNewPairs[i].Resize(256);
	}
}

ndScene::ndScene(const ndScene& src)
	:ndThreadPool("newtonWorker")
	,m_bodyList(src.m_bodyList)
	,m_contactArray(src.m_contactArray)
	,m_scratchBuffer()
	,m_sceneBodyArray()
	,m_activeConstraintArray()
	,m_specialUpdateList()
	,m_backgroundThread()
	,m_newPairs(1024)
#ifdef D_NEW_SCENE
	,m_bvhBuildState()
#else
	,m_cellBuffer0(1024)
	,m_cellBuffer1(1024)
	,m_cellCounts0(1024)
	,m_cellCounts1(1024)
#endif
	,m_lock()
	,m_rootNode(nullptr)
	,m_sentinelBody(nullptr)
	,m_contactNotifyCallback(nullptr)
	,m_fitness(src.m_fitness)
	,m_timestep(ndFloat32(0.0f))
	,m_lru(src.m_lru)
	,m_frameIndex(src.m_frameIndex)
	,m_subStepIndex(src.m_subStepIndex)
	,m_forceBalanceSceneCounter(0)
{
	ndScene* const stealData = (ndScene*)&src;

	SetThreadCount(src.GetThreadCount());
	m_backgroundThread.SetThreadCount(m_backgroundThread.GetThreadCount());

	m_scratchBuffer.Swap(stealData->m_scratchBuffer);
	m_sceneBodyArray.Swap(stealData->m_sceneBodyArray);
	m_activeConstraintArray.Swap(stealData->m_activeConstraintArray);

	ndSwap(m_rootNode, stealData->m_rootNode);
	ndSwap(m_sentinelBody, stealData->m_sentinelBody);
	ndSwap(m_contactNotifyCallback, stealData->m_contactNotifyCallback);
	m_contactNotifyCallback->m_scene = this;

	ndList<ndBodyKinematic*>::ndNode* nextNode;
	for (ndList<ndBodyKinematic*>::ndNode* node = stealData->m_specialUpdateList.GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		stealData->m_specialUpdateList.Unlink(node);
		m_specialUpdateList.Append(node);
	}

	for (ndBodyList::ndNode* node = m_bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo();
		body->m_sceneForceUpdate = 1;
		ndScene* const sceneNode = body->GetScene();
		if (sceneNode)
		{
			body->SetSceneNodes(this, node);
		}
		dAssert (body->GetContactMap().SanityCheck());
	}

	for (ndInt32 i = 0; i < D_MAX_THREADS_COUNT; ++i)
	{
		m_partialNewPairs[i].Resize(256);
	}
}

ndScene::~ndScene()
{
	Cleanup();
	Finish();
	if (m_contactNotifyCallback)
	{
		delete m_contactNotifyCallback;
	}
	ndFreeListAlloc::Flush();
}

void ndScene::Sync()
{
	ndThreadPool::Sync();
}

void ndScene::CollisionOnlyUpdate()
{
	D_TRACKTIME();
	Begin();
	m_lru = m_lru + 1;
	InitBodyArray();
	BalanceScene();
	FindCollidingPairs();
	CalculateContacts();
	End();
}

void ndScene::ThreadFunction()
{
	D_TRACKTIME();
	CollisionOnlyUpdate();
}

void ndScene::Begin()
{
	ndThreadPool::Begin();
}

void ndScene::End()
{
	ndThreadPool::End();
	m_frameIndex++;
}

void ndScene::Update(ndFloat32 timestep)
{
	// wait until previous update complete.
	Sync();

	// save time state for use by the update callback
	m_timestep = timestep;

	// update the next frame asynchronous 
	TickOne();
}

ndContactNotify* ndScene::GetContactNotify() const
{
	return m_contactNotifyCallback;
}

void ndScene::SetContactNotify(ndContactNotify* const notify)
{
	dAssert(m_contactNotifyCallback);
	delete m_contactNotifyCallback;
	
	if (notify)
	{
		m_contactNotifyCallback = notify;
	}
	else
	{
		m_contactNotifyCallback = new ndContactNotify();
	}
	m_contactNotifyCallback->m_scene = this;
}

void ndScene::DebugScene(ndSceneTreeNotiFy* const notify)
{
	for (ndInt32 i = 0; i < m_fitness.GetCount(); ++i)
	{
		ndSceneNode* const node = m_fitness[i];
		if (node->GetAsSceneBodyNode())
		{
			notify->OnDebugNode(node);
		}
	}
}

bool ndScene::AddBody(ndBodyKinematic* const body)
{
	if ((body->m_scene == nullptr) && (body->m_sceneNode == nullptr))
	{
		ndBodyList::ndNode* const node = m_bodyList.AddItem(body);
		body->SetSceneNodes(this, node);
		m_contactNotifyCallback->OnBodyAdded(body);
		body->UpdateCollisionMatrix();

		ndSceneTreeNode* const sceneNode = new ndSceneTreeNode();
		ndSceneBodyNode* const bodyNode = new ndSceneBodyNode(body);

		m_fitness.AddNode(sceneNode);
		body->m_sceneNodeIndex = m_fitness.GetCount() - 1;

		m_fitness.AddNode(bodyNode);
		body->m_bodyNodeIndex = m_fitness.GetCount() - 1;

		AddNode(sceneNode, bodyNode);

		if (body->GetAsBodyTriggerVolume() || body->GetAsBodyPlayerCapsule())
		{
			body->m_spetialUpdateNode = m_specialUpdateList.Append(body);
		}

		m_forceBalanceSceneCounter = 0;

		return true;
	}
	return false;
}

bool ndScene::RemoveBody(ndBodyKinematic* const body)
{
	m_fitness.m_isDirty = 1;
	m_forceBalanceSceneCounter = 0;

	ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)m_fitness[body->m_bodyNodeIndex];
	ndSceneTreeNode* const sceneNode = (ndSceneTreeNode*)m_fitness[body->m_sceneNodeIndex];
	dAssert(bodyNode->GetAsSceneBodyNode());
	dAssert(sceneNode->GetAsSceneTreeNode());
	bodyNode->Kill();
	sceneNode->Kill();

	ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
	while (contactMap.GetRoot())
	{
		ndContact* const contact = contactMap.GetRoot()->GetInfo();
		m_contactArray.DeleteContact(contact);
	}

	if (body->m_scene && body->m_sceneNode)
	{
		if (body->GetAsBodyTriggerVolume() || body->GetAsBodyPlayerCapsule())
		{
			m_specialUpdateList.Remove(body->m_spetialUpdateNode);
			body->m_spetialUpdateNode = nullptr;
		}

		m_bodyList.RemoveItem(body->m_sceneNode);
		body->SetSceneNodes(nullptr, nullptr);
		m_contactNotifyCallback->OnBodyRemoved(body);
		return true;
	}
	return false;
}

void ndScene::BalanceScene()
{
	D_TRACKTIME();

	//dAssert(m_rootNode->SanityCheck(0));
	if (m_fitness.GetCount() > 2)
	{
		if (!m_forceBalanceSceneCounter)
		{
			m_rootNode = BuildBvhTree();
		}
		m_forceBalanceSceneCounter = (m_forceBalanceSceneCounter < 64) ? m_forceBalanceSceneCounter + 1 : 0;
		dAssert(!m_rootNode || !m_rootNode->m_parent);
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
	dAssert(contact && (contact->GetAsContact()));

	if (contact->m_maxDOF)
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
	
	dAssert(body0->GetScene() == this);
	dAssert(body1->GetScene() == this);

	dAssert(contact->m_material);
	dAssert(m_contactNotifyCallback);

	bool processContacts = m_contactNotifyCallback->OnAabbOverlap(contact, m_timestep);
	if (processContacts)
	{
		dAssert(!body0->GetAsBodyTriggerVolume());
		dAssert(!body0->GetCollisionShape().GetShape()->GetAsShapeNull());
		dAssert(!body1->GetCollisionShape().GetShape()->GetAsShapeNull());
			
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
				if (!contact->m_isIntersetionTestOnly)
				{
					ndBodyTriggerVolume* const trigger = body1->GetAsBodyTriggerVolume();
					if (trigger)
					{
						trigger->OnTriggerEnter(body0, m_timestep);
					}
				}
				contact->m_isIntersetionTestOnly = 1;
			}
			else
			{
				dAssert(count <= (D_CONSTRAINT_MAX_ROWS / 3));
				ProcessContacts(threadIndex, count, &contactSolver);
				dAssert(contact->m_maxDOF);
				contact->m_isIntersetionTestOnly = 0;
			}
		}
		else
		{
			if (contactSolver.m_intersectionTestOnly)
			{
				ndBodyTriggerVolume* const trigger = body1->GetAsBodyTriggerVolume();
				if (trigger)
				{
					body1->GetAsBodyTriggerVolume()->OnTriggerExit(body0, m_timestep);
				}
				contact->m_isIntersetionTestOnly = 1;
			}
			contact->m_maxDOF = 0;
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
	dAssert(body0);
	dAssert(body1);
	dAssert(body0 != body1);

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
	dAssert(controlNormal.m_w == ndFloat32(0.0f));
	ndVector tangDir(vRel - controlNormal * vRel.DotProduct(controlNormal));
	dAssert(tangDir.m_w == ndFloat32(0.0f));
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
		dAssert(controlDir0.m_w == ndFloat32(0.0f));
		dAssert(controlDir0.DotProduct(controlDir0).GetScalar() > ndFloat32(1.0e-8f));
		controlDir0 = controlDir0.Normalize();
		controlDir1 = controlNormal.CrossProduct(controlDir0);
		dAssert(ndAbs(controlNormal.DotProduct(controlDir0.CrossProduct(controlDir1)).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
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
			dAssert(v.m_w == ndFloat32(0.0f));
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
			dAssert(index != -1);
			nodes[index] = nodes[count];
			cachePosition[index] = cachePosition[count];
		}
		else 
		{
			contactNode = contactPointList.Append();
		}

		ndContactMaterial* const contactPoint = &contactNode->GetInfo();
	
		dAssert(dCheckFloat(contactArray[i].m_point.m_x));
		dAssert(dCheckFloat(contactArray[i].m_point.m_y));
		dAssert(dCheckFloat(contactArray[i].m_point.m_z));
		dAssert(contactArray[i].m_body0);
		dAssert(contactArray[i].m_body1);
		dAssert(contactArray[i].m_shapeInstance0);
		dAssert(contactArray[i].m_shapeInstance1);
		dAssert(contactArray[i].m_body0 == body0);
		dAssert(contactArray[i].m_body1 == body1);
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
				dAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
				dAssert(contactPoint->m_dir0.DotProduct(contactPoint->m_dir0).GetScalar() > ndFloat32(1.0e-8f));
				contactPoint->m_dir0 = contactPoint->m_dir0.Normalize();
				contactPoint->m_dir1 = contactPoint->m_normal.CrossProduct(contactPoint->m_dir0);
				dAssert(ndAbs(contactPoint->m_normal.DotProduct(contactPoint->m_dir0.CrossProduct(contactPoint->m_dir1)).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
			}
		}
		else 
		{
			ndVector veloc0(v0 + w0.CrossProduct(contactPoint->m_point - com0));
			ndVector veloc1(v1 + w1.CrossProduct(contactPoint->m_point - com1));
			ndVector relReloc(veloc1 - veloc0);
	
			dAssert(contactPoint->m_normal.m_w == ndFloat32(0.0f));
			ndFloat32 impulse = relReloc.DotProduct(contactPoint->m_normal).GetScalar();
			if (ndAbs(impulse) > maxImpulse) 
			{
				maxImpulse = ndAbs(impulse);
			}
	
			ndVector tangentDir(relReloc - contactPoint->m_normal.Scale(impulse));
			dAssert(tangentDir.m_w == ndFloat32(0.0f));
			diff = tangentDir.DotProduct(tangentDir).GetScalar();
			if (diff > ndFloat32(1.0e-2f)) 
			{
				dAssert(tangentDir.m_w == ndFloat32(0.0f));
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
				dAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
				dAssert(contactPoint->m_dir0.DotProduct(contactPoint->m_dir0).GetScalar() > ndFloat32(1.0e-8f));
				contactPoint->m_dir0 = contactPoint->m_dir0.Normalize();
			}
			contactPoint->m_dir1 = contactPoint->m_normal.CrossProduct(contactPoint->m_dir0);
			dAssert(ndAbs(contactPoint->m_normal.DotProduct(contactPoint->m_dir0.CrossProduct(contactPoint->m_dir1)).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
		}
		dAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
		dAssert(contactPoint->m_dir0.m_w == ndFloat32(0.0f));
		dAssert(contactPoint->m_normal.m_w == ndFloat32(0.0f));
	}
	
	for (ndInt32 i = 0; i < count; ++i) 
	{
		contactPointList.Remove(nodes[i]);
	}
	
	contact->m_maxDOF = ndUnsigned32(3 * contactPointList.GetCount());
	m_contactNotifyCallback->OnContactCallback(contact, m_timestep);
}

void ndScene::SubmitPairs(ndSceneBodyNode* const leafNode, ndSceneNode* const node, ndInt32 threadId)
{
	ndSceneNode* pool[D_SCENE_MAX_STACK_DEPTH];

	ndBodyKinematic* const body0 = leafNode->GetBody() ? leafNode->GetBody() : nullptr;
	dAssert(body0);

	const ndVector boxP0(leafNode->m_minBox);
	const ndVector boxP1(leafNode->m_maxBox);
	const bool test0 = (body0->m_invMass.m_w != ndFloat32(0.0f)) & body0->GetCollisionShape().GetCollisionMode();

	pool[0] = node;
	ndInt32 stack = 1;
	while (stack && (stack < (D_SCENE_MAX_STACK_DEPTH - 16)))
	{
		stack--;
		ndSceneNode* const rootNode = pool[stack];
		if (dOverlapTest(rootNode->m_minBox, rootNode->m_maxBox, boxP0, boxP1)) 
		{
			if (rootNode->GetAsSceneBodyNode()) 
			{
				dAssert(!rootNode->GetRight());
				dAssert(!rootNode->GetLeft());
				
				ndBodyKinematic* const body1 = rootNode->GetBody();
				dAssert(body1);
				const bool test1 = (body1->m_invMass.m_w != ndFloat32(0.0f)) & body1->GetCollisionShape().GetCollisionMode();
				const bool test = test0 | test1;
				if (test)
				{
					AddPair(body0, body1, threadId);
				}
			}
			else 
			{
				ndSceneTreeNode* const tmpNode = rootNode->GetAsSceneTreeNode();
				dAssert(tmpNode->m_left);
				dAssert(tmpNode->m_right);
		
				pool[stack] = tmpNode->m_left;
				stack++;
				dAssert(stack < ndInt32(sizeof(pool) / sizeof(pool[0])));
		
				pool[stack] = tmpNode->m_right;
				stack++;
				dAssert(stack < ndInt32(sizeof(pool) / sizeof(pool[0])));
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
		for (ndJointList::ndNode* node = body0->m_jointList.GetFirst(); node; node = node->GetNext())
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
		for (ndJointList::ndNode* node = body1->m_jointList.GetFirst(); node; node = node->GetNext())
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
	ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)m_fitness[body->m_bodyNodeIndex];
	dAssert(bodyNode->GetAsSceneBodyNode());
	for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		dAssert(!parent->GetAsSceneBodyNode());
		ndSceneNode* const sibling = parent->m_right;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling, threadId);
		}
	}
}

void ndScene::FindCollidingPairsForward(ndBodyKinematic* const body, ndInt32 threadId)
{
	ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)m_fitness[body->m_bodyNodeIndex];
	dAssert(bodyNode->GetAsSceneBodyNode());
	for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		dAssert(!parent->GetAsSceneBodyNode());
		ndSceneNode* const sibling = parent->m_right;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling, threadId);
		}
	}
}

void ndScene::FindCollidingPairsBackward(ndBodyKinematic* const body, ndInt32 threadId)
{
	ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)m_fitness[body->m_bodyNodeIndex];
	dAssert(bodyNode->GetAsSceneBodyNode());
	for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		dAssert(!parent->GetAsSceneBodyNode());
		ndSceneNode* const sibling = parent->m_left;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling, threadId);
		}
	}
}

void ndScene::UpdateTransform()
{
	D_TRACKTIME();
	auto TransformUpdate = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(TransformUpdate);
		const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
		const ndStartEnd startEnd(bodyArray.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			UpdateTransformNotify(threadIndex, body);
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

	dAssert(!contact->m_isDead);
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
				if (contact->m_maxDOF || contact->m_isIntersetionTestOnly)
				{
					contact->SetActive(true);
					contact->m_timeOfImpact = ndFloat32(1.0e10f);
				}
				contact->m_sceneLru = m_lru;
			}
			else
			{
				const ndSceneBodyNode* const bodyNode0 = (ndSceneBodyNode*)m_fitness[contact->GetBody0()->m_bodyNodeIndex];
				const ndSceneBodyNode* const bodyNode1 = (ndSceneBodyNode*)m_fitness[contact->GetBody1()->m_bodyNodeIndex];
				dAssert(bodyNode0 && bodyNode0->GetAsSceneBodyNode());
				dAssert(bodyNode1 && bodyNode1->GetAsSceneBodyNode());
				if (dOverlapTest(bodyNode0->m_minBox, bodyNode0->m_maxBox, bodyNode1->m_minBox, bodyNode1->m_maxBox)) 
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
			dAssert(body0->GetInvMass() > ndFloat32(0.0f));
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
		const ndSceneBodyNode* const bodyNode0 = (ndSceneBodyNode*)m_fitness[contact->GetBody0()->m_bodyNodeIndex];
		const ndSceneBodyNode* const bodyNode1 = (ndSceneBodyNode*)m_fitness[contact->GetBody1()->m_bodyNodeIndex];
		dAssert(bodyNode0->GetAsSceneBodyNode());
		dAssert(bodyNode1->GetAsSceneBodyNode());
		if (!dOverlapTest(bodyNode0->m_minBox, bodyNode0->m_maxBox, bodyNode1->m_minBox, bodyNode1->m_maxBox))
		{
			contact->m_isDead = 1;
		}
	}
}

void ndScene::UpdateSpecial()
{
	for (ndList<ndBodyKinematic*>::ndNode* node = m_specialUpdateList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo();
		body->SpecialUpdate(m_timestep);
	}
}

bool ndScene::ConvexCast(ndConvexCastNotify& callback, const ndSceneNode** stackPool, ndFloat32* const stackDistance, ndInt32 stack, const ndFastRay& ray, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const
{
	ndVector boxP0;
	ndVector boxP1;

	dAssert(globalOrigin.TestOrthogonal());
	convexShape.CalculateAabb(globalOrigin, boxP0, boxP1);

	callback.m_contacts.SetCount(0);
	callback.m_param = ndFloat32(1.2f);
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
			const ndSceneNode* const me = stackPool[stack];
		
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
					const ndSceneNode* const left = me->GetLeft();
					dAssert(left);
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
						dAssert(stack < D_SCENE_MAX_STACK_DEPTH);
					}
				}
		
				{
					const ndSceneNode* const right = me->GetRight();
					dAssert(right);
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
						dAssert(stack < D_SCENE_MAX_STACK_DEPTH);
					}
				}
			}
		}
	}
	return callback.m_contacts.GetCount() > 0;
}

bool ndScene::RayCast(ndRayCastNotify& callback, const ndSceneNode** stackPool, ndFloat32* const stackDistance, ndInt32 stack, const ndFastRay& ray) const
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
			const ndSceneNode* const me = stackPool[stack];
			dAssert(me);
			ndBodyKinematic* const body = me->GetBody();
			if (body)
			{
				dAssert(!me->GetLeft());
				dAssert(!me->GetRight());

				//callback.TraceShape(ray.m_p0, ray.m_p1, body->GetCollisionShape(), body->GetMatrix());
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
				const ndSceneNode* const left = me->GetLeft();
				dAssert(left);
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
					dAssert(stack < D_SCENE_MAX_STACK_DEPTH);
				}
	
				const ndSceneNode* const right = me->GetRight();
				dAssert(right);
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
					dAssert(stack < D_SCENE_MAX_STACK_DEPTH);
				}
			}
		}
	}
	return state;
}

void ndScene::BodiesInAabb(ndBodiesInAabbNotify& callback, const ndSceneNode** stackPool, ndInt32 stack) const
{
	callback.m_bodyArray.SetCount(0);
	while (stack && (stack < (D_SCENE_MAX_STACK_DEPTH - 4)))
	{
		stack--;
		
		const ndSceneNode* const me = stackPool[stack];
		dAssert(me);
		ndBodyKinematic* const body = me->GetBody();
		if (body)
		{
			dAssert(!me->GetLeft());
			dAssert(!me->GetRight());
			if (callback.OnOverlap(body))
			{
				callback.m_bodyArray.PushBack(body);
			}
		}
		else
		{
			const ndSceneNode* const left = me->GetLeft();
			dAssert(left);
			stackPool[stack] = left;
			stack++;
			dAssert(stack < D_SCENE_MAX_STACK_DEPTH);

			const ndSceneNode* const right = me->GetRight();
			dAssert(right);
			stackPool[stack] = right;
			stack++;
			dAssert(stack < D_SCENE_MAX_STACK_DEPTH);
		}
	}
}

void ndScene::Cleanup()
{
	Sync();
	m_frameIndex = 0;
	m_subStepIndex = 0;

	m_backgroundThread.Terminate();

	while (m_bodyList.GetFirst())
	{
		ndBodyKinematic* const body = m_bodyList.GetFirst()->GetInfo();
		RemoveBody(body);
		delete body;
	}
	if (m_sentinelBody)
	{
		delete m_sentinelBody;
		m_sentinelBody = nullptr;
	}

	m_fitness.CleanUp();
	m_contactArray.DeleteAllContacts();

	ndFreeListAlloc::Flush();
	m_contactArray.Resize(1024);
	m_sceneBodyArray.Resize(1024);
	m_activeConstraintArray.Resize(1024);
	m_scratchBuffer.Resize(1024 * sizeof(void*));

	m_contactArray.SetCount(0);
	m_scratchBuffer.SetCount(0);
	m_sceneBodyArray.SetCount(0);
	m_activeConstraintArray.SetCount(0);
}

void ndScene::AddNode(ndSceneTreeNode* const childNode, ndSceneBodyNode* const bodyNode)
{
	if (m_rootNode)
	{
		childNode->m_minBox = bodyNode->m_minBox;
		childNode->m_maxBox = bodyNode->m_maxBox;
		childNode->m_left = bodyNode;
		bodyNode->m_parent = childNode;

		ndUnsigned32 depth = 0;
		ndSceneNode* parent = m_rootNode;
		while (1)
		{
			ndSceneTreeNode* const sceneNode = parent->GetAsSceneTreeNode();
			if (sceneNode && dBoxInclusionTest(childNode->m_minBox, childNode->m_maxBox, parent->m_minBox, parent->m_maxBox))
			{
				const ndVector minLeftBox (sceneNode->m_left->m_minBox.GetMin(childNode->m_minBox));
				const ndVector maxLeftBox (sceneNode->m_left->m_maxBox.GetMax(childNode->m_maxBox));
				const ndVector minRightBox(sceneNode->m_right->m_minBox.GetMin(childNode->m_minBox));
				const ndVector maxRightBox(sceneNode->m_right->m_maxBox.GetMax(childNode->m_maxBox));
				const ndVector leftSize(maxLeftBox - minLeftBox);
				const ndVector rightSize(maxRightBox - minRightBox);
				const ndFloat32 leftArea = leftSize.DotProduct(leftSize.ShiftTripleRight()).GetScalar();
				const ndFloat32 rightArea = rightSize.DotProduct(rightSize.ShiftTripleRight()).GetScalar();

				parent = (leftArea < rightArea) ? sceneNode->m_left : sceneNode->m_right;
				depth++;
			}
			else
			{
				if (parent->m_parent)
				{
					if (parent->m_parent->GetLeft() == parent)
					{
						parent->m_parent->GetAsSceneTreeNode()->m_left = childNode;
					}
					else
					{
						parent->m_parent->GetAsSceneTreeNode()->m_right = childNode;
					}
					childNode->m_right = parent;
					childNode->m_parent = parent->m_parent;
					parent->m_parent = childNode;

					const ndVector minBox(childNode->m_left->m_minBox.GetMin(childNode->m_right->m_minBox));
					const ndVector maxBox(childNode->m_left->m_maxBox.GetMax(childNode->m_right->m_maxBox));
					childNode->m_minBox = minBox;
					childNode->m_maxBox = maxBox;
				}
				else
				{
					const ndVector minBox(parent->m_minBox.GetMin(childNode->m_minBox));
					const ndVector maxBox(parent->m_maxBox.GetMax(childNode->m_maxBox));
					childNode->m_minBox = minBox;
					childNode->m_maxBox = maxBox;
					childNode->m_right = parent;
					childNode->m_parent = nullptr;
					parent->m_parent = childNode;
					m_rootNode = childNode;
				}
				break;
			}
		}
#ifdef _DEBUG
		//dAssert(depth < 128);
		if (depth >= 256)
		{
			dTrace(("This may be a pathological scene, consider balancing the scene\n"));
		}
#endif
	}
	else
	{
		m_rootNode = bodyNode;
	}
}

void ndScene::RemoveNode(ndSceneNode* const node)
{
	if (node->m_parent)
	{
		ndSceneTreeNode* const parent = (ndSceneTreeNode*)node->m_parent;
		if (parent->m_parent)
		{
			ndSceneTreeNode* const grandParent = (ndSceneTreeNode*)parent->m_parent;
			if (grandParent->m_left == parent)
			{
				if (parent->m_right == node)
				{
					grandParent->m_left = parent->m_left;
					parent->m_left->m_parent = grandParent;
					parent->m_left = nullptr;
					parent->m_parent = nullptr;
				}
				else
				{
					grandParent->m_left = parent->m_right;
					parent->m_right->m_parent = grandParent;
					parent->m_right = nullptr;
					parent->m_parent = nullptr;
				}
			}
			else
			{
				if (parent->m_right == node)
				{
					grandParent->m_right = parent->m_left;
					parent->m_left->m_parent = grandParent;
					parent->m_left = nullptr;
					parent->m_parent = nullptr;
				}
				else
				{
					grandParent->m_right = parent->m_right;
					parent->m_right->m_parent = grandParent;
					parent->m_right = nullptr;
					parent->m_parent = nullptr;
				}
			}
		}
		else
		{
			dAssert(!node->m_parent->GetAsSceneBodyNode());
			ndSceneTreeNode* const parent1 = node->m_parent->GetAsSceneTreeNode();
			if (parent1->m_right == node)
			{
				m_rootNode = parent1->m_left;
				m_rootNode->m_parent = nullptr;
				parent1->m_left = nullptr;
			}
			else
			{
				m_rootNode = parent1->m_right;
				m_rootNode->m_parent = nullptr;
				parent1->m_right = nullptr;
			}
		}

		dAssert(parent->GetAsSceneTreeNode());
		parent->Kill();
		m_fitness.m_isDirty = 1;
	}
	else
	{
		delete node;
		m_rootNode = nullptr;
	}
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
			const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

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
		dAssert(globalOrigin.TestOrthogonal());
		convexShape.CalculateAabb(globalOrigin, boxP0, boxP1);

		ndFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
		const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

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

void ndScene::BodiesInAabb(ndBodiesInAabbNotify& callback) const
{
	callback.m_bodyArray.SetCount(0);

	if (m_rootNode)
	{
		const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];
		stackPool[0] = m_rootNode;
		ndScene::BodiesInAabb(callback, stackPool, 1);
	}
}

void ndScene::SendBackgroundTask(ndBackgroundTask* const job)
{
	m_backgroundThread.SendTask(job);
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
			ndArray<ndContactPairs>& particalPairs = m_partialNewPairs[threadId];
			ndContactPairs pair(body0->m_index, body1->m_index);
			particalPairs.PushBack(pair);
		}
	}
}

void ndScene::CalculateContacts()
{
	D_TRACKTIME();
	ndInt32 contactCount = m_contactArray.GetCount();
	m_scratchBuffer.SetCount((contactCount + m_newPairs.GetCount() + 16) * sizeof(ndContact*));

	ndContact** const tmpJointsArray = (ndContact**)&m_scratchBuffer[0];
	
	auto CreateNewContacts = ndMakeObject::ndFunction([this, tmpJointsArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CreateNewContacts);
		const ndArray<ndContactPairs>& newPairs = m_newPairs;
		ndBodyKinematic** const bodyArray = &GetActiveBodyArray()[0];

		const ndUnsigned32 count = newPairs.GetCount();
		const ndStartEnd startEnd(count, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndContactPairs& pair = newPairs[i];

			ndBodyKinematic* const body0 = bodyArray[pair.m_body0];
			ndBodyKinematic* const body1 = bodyArray[pair.m_body1];
			dAssert(ndUnsigned32(body0->m_index) == pair.m_body0);
			dAssert(ndUnsigned32(body1->m_index) == pair.m_body1);
	
			ndContact* const contact = new ndContact;
			contact->SetBodies(body0, body1);
			contact->AttachToBodies();
	
			dAssert(contact->m_body0->GetInvMass() != ndFloat32(0.0f));
			contact->m_material = m_contactNotifyCallback->GetMaterial(contact, body0->GetCollisionShape(), body1->GetCollisionShape());
			tmpJointsArray[i] = contact;
		}
	});
	
	auto CopyContactArray = ndMakeObject::ndFunction([this, tmpJointsArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CopyContactArray);
		ndContact** const contactArray = &m_contactArray[0];

		const ndUnsigned32 start = m_newPairs.GetCount();
		const ndUnsigned32 count = m_contactArray.GetCount();
		
		const ndStartEnd startEnd(count, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndContact* const contact = contactArray[i];
			tmpJointsArray[start + i] = contact;
		}
	});
	
	ParallelExecute(CreateNewContacts);
	if (contactCount)
	{
		ParallelExecute(CopyContactArray);
	}

	m_activeConstraintArray.SetCount(0);
	m_contactArray.SetCount(contactCount + m_newPairs.GetCount());
	if (m_contactArray.GetCount())
	{
		auto CalculateContactPoints = ndMakeObject::ndFunction([this, tmpJointsArray, &contactCount](ndInt32 threadIndex, ndInt32 threadCount)
		{
			D_TRACKTIME_NAMED(CalculateContactPoints);
			const ndUnsigned32 jointCount = m_contactArray.GetCount();

			const ndStartEnd startEnd(jointCount, threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndContact* const contact = tmpJointsArray[i];
				dAssert(contact);
				if (!contact->m_isDead)
				{
					CalculateContacts(threadIndex, contact);
				}
			}
		});
		ParallelExecute(CalculateContactPoints);

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

			ndUnsigned32 GetKey(const ndContact* const contact) const
			{
				const ndUnsigned32 inactive = !contact->IsActive() | (contact->m_maxDOF ? 0 : 1);
				const ndUnsigned32 idDead = contact->m_isDead;
				return m_code[idDead * 2 + inactive];
			}

			ndUnsigned32 m_code[4];
		};

		ndUnsigned32 prefixScan[5];
		ndCountingSort<ndContact*, ndJointActive, 2>(*this, tmpJointsArray, &m_contactArray[0], m_contactArray.GetCount(), prefixScan, nullptr);

		if (prefixScan[m_dead + 1] != prefixScan[m_dead])
		{
			auto DeleteContactArray = ndMakeObject::ndFunction([this, &prefixScan](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(DeleteContactArray);
				ndArray<ndContact*>& contactArray = m_contactArray;
				const ndUnsigned32 start = prefixScan[m_dead];
				const ndUnsigned32 count = prefixScan[m_dead + 1] - start;

				const ndStartEnd startEnd(count, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndContact* const contact = contactArray[start + i];
					dAssert(contact->m_isDead);
					if (contact->m_isAttached)
					{
						contact->DetachFromBodies();
					}
					contact->m_isDead = 1;
					delete contact;
				}
			});

			ParallelExecute(DeleteContactArray);
			m_contactArray.SetCount(prefixScan[m_inactive + 1]);
		}

		m_activeConstraintArray.SetCount(prefixScan[m_active + 1]);
		if (m_activeConstraintArray.GetCount())
		{
			auto CopyActiveContact = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(CopyActiveContact);
				const ndArray<ndContact*>& constraintArray = m_contactArray;
				ndArray<ndConstraint*>& activeConstraintArray = m_activeConstraintArray;
				const ndUnsigned32 activeJointCount = activeConstraintArray.GetCount();

				const ndStartEnd startEnd(activeJointCount, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					activeConstraintArray[i] = constraintArray[i];
				}
			});
			ParallelExecute(CopyActiveContact);
		}
	}
}

void ndScene::FindCollidingPairs()
{
	D_TRACKTIME();
	auto FindPairs = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(FindPairs);
		const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
		const ndStartEnd startEnd(bodyArray.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			FindCollidingPairs(body, threadIndex);
		}
	});

	auto FindPairsForward = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(FindPairsForward);
		const ndArray<ndBodyKinematic*>& bodyArray = m_sceneBodyArray;
		const ndStartEnd startEnd(bodyArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			FindCollidingPairsForward(body, threadIndex);
		}
	});

	auto FindPairsBackward = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(FindPairsBackward);
		const ndArray<ndBodyKinematic*>& bodyArray = m_sceneBodyArray;
		const ndStartEnd startEnd(bodyArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			FindCollidingPairsBackward(body, threadIndex);
		}
	});

	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		m_partialNewPairs[i].SetCount(0);
	}

	const ndInt32 threadCount = GetThreadCount();
	const ndArray<ndBodyKinematic*>& activeBodies = GetActiveBodyArray();
	const bool fullScan = (2 * m_sceneBodyArray.GetCount()) > activeBodies.GetCount();
	if (fullScan)
	{
		ParallelExecute(FindPairs);

		ndUnsigned32 sum = 0;
		ndUnsigned32 scanCounts[D_MAX_THREADS_COUNT + 1];
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			const ndArray<ndContactPairs>& newPairs = m_partialNewPairs[i];
			scanCounts[i] = sum;
			sum += newPairs.GetCount();
		}
		scanCounts[threadCount] = sum;
		m_newPairs.SetCount(sum);

		if (sum)
		{
			auto CopyPartialCounts = ndMakeObject::ndFunction([this, &scanCounts](ndInt32 threadIndex, ndInt32)
			{
				D_TRACKTIME_NAMED(CopyPartialCounts);
				const ndArray<ndContactPairs>& newPairs = m_partialNewPairs[threadIndex];

				const ndUnsigned32 count = newPairs.GetCount();
				const ndUnsigned32 start = scanCounts[threadIndex];
				dAssert((scanCounts[threadIndex + 1] - start) == ndUnsigned32(newPairs.GetCount()));
				for (ndUnsigned32 i = 0; i < count; ++i)
				{
					m_newPairs[start + i] = newPairs[i];
				}
			});
			ParallelExecute(CopyPartialCounts);
		}
	}
	else
	{
		ParallelExecute(FindPairsForward);
		ParallelExecute(FindPairsBackward);

		ndUnsigned32 sum = 0;
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			sum += m_partialNewPairs[i].GetCount();
		}
		m_newPairs.SetCount(sum);

		sum = 0;
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			const ndArray<ndContactPairs>& newPairs = m_partialNewPairs[i];
			const ndUnsigned32 count = newPairs.GetCount();
			for (ndUnsigned32 j = 0; j < count; ++j)
			{
				m_newPairs[sum + j] = newPairs[j];
			}
			sum += count;
		}

		if (sum)
		{
			class CompareKey
			{
				public:
				int Compare(const ndContactPairs& a, const ndContactPairs& b, void*) const
				{
					union Key
					{
						ndUnsigned64 m_key;
						struct
						{
							ndUnsigned32 m_low;
							ndUnsigned32 m_high;
						};
					};

					Key keyA;
					Key keyB;
					keyA.m_low = a.m_body0;
					keyA.m_high = a.m_body1;
					keyB.m_low = b.m_body0;
					keyB.m_high = b.m_body1;

					if (keyA.m_key < keyB.m_key)
					{
						return -1;
					}
					else if (keyA.m_key > keyB.m_key)
					{
						return 1;
					}
					return 0;
				}
			};
			ndSort<ndContactPairs, CompareKey>(&m_newPairs[0], sum, nullptr);

			CompareKey comparator;
			for (ndInt32 i = sum - 2; i >= 0; i--)
			{
				if (comparator.Compare(m_newPairs[i], m_newPairs[i + 1], nullptr) == 0)
				{
					sum--;
					m_newPairs[i] = m_newPairs[sum];
				}
			}
			m_newPairs.SetCount(sum);

			#ifdef _DEBUG
			for (ndInt32 i = 1; i < m_newPairs.GetCount(); ++i)
			{
				dAssert(comparator.Compare(m_newPairs[i], m_newPairs[i - 1], nullptr));
			}
			#endif	
		}
	}
}

void ndScene::UpdateBodyList()
{
	if (m_bodyList.UpdateView())
	{
		//D_TRACKTIME();
		ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
		//for (ndInt32 i = 0; i < view.GetCount(); ++i)
		//{
		//	ndBodyKinematic* const body = view[i];
		//	dAssert(!body->GetCollisionShape().GetShape()->GetAsShapeNull());
		//	bool inScene = true;
		//	if (!body->GetSceneBodyNode())
		//	{
		//		inScene = AddBody(body);
		//	}
		//	dAssert(inScene && body->GetSceneBodyNode());
		//}
		view.PushBack(m_sentinelBody);
	}
	m_fitness.Update(*this);
}

void ndScene::ApplyExtForce()
{
	D_TRACKTIME();

	auto ApplyForce = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(ApplyForce);
		const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();

		const ndFloat32 timestep = m_timestep;
		const ndStartEnd startEnd(view.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = view[i];
			body->ApplyExternalForces(threadIndex, timestep);
		}
	});
	ParallelExecute(ApplyForce);
}

void ndScene::InitBodyArray()
{
	D_TRACKTIME();
	ndInt32 scans[D_MAX_THREADS_COUNT][2];
	auto BuildBodyArray = ndMakeObject::ndFunction([this, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(BuildBodyArray);
		const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
	
		ndInt32* const scan = &scans[threadIndex][0];
		scan[0] = 0;
		scan[1] = 0;
			
		const ndStartEnd startEnd(view.GetCount() - 1, threadIndex, threadCount);
	
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = view[i];
	
			body->PrepareStep(i);
			ndUnsigned8 sceneEquilibrium = 1;
			ndUnsigned8 sceneForceUpdate = body->m_sceneForceUpdate;
			if (ndUnsigned8(!body->m_equilibrium) | sceneForceUpdate)
			{
				ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)m_fitness[body->m_bodyNodeIndex];
				dAssert(bodyNode->GetAsSceneBodyNode());
				dAssert(bodyNode->m_body == body);
				dAssert(!bodyNode->GetLeft());
				dAssert(!bodyNode->GetRight());
				dAssert(!body->GetCollisionShape().GetShape()->GetAsShapeNull());
			
				body->UpdateCollisionMatrix();
				const ndInt32 test = dBoxInclusionTest(body->m_minAabb, body->m_maxAabb, bodyNode->m_minBox, bodyNode->m_maxBox);
				if (!test)
				{
					bodyNode->SetAabb(body->m_minAabb, body->m_maxAabb);
				}
				sceneEquilibrium = !sceneForceUpdate & (test != 0);
			}
			body->m_sceneForceUpdate = 0;
			body->m_sceneEquilibrium = sceneEquilibrium;
			scan[sceneEquilibrium] ++;
		}
	});
	
	auto CompactMovingBodies = ndMakeObject::ndFunction([this, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CompactMovingBodies);
		const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
		ndBodyKinematic** const sceneBodyArray = &m_sceneBodyArray[0];
		ndInt32* const scan = &scans[threadIndex][0];
	
		const ndStartEnd startEnd(view.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = view[i];
			const ndInt32 key = body->m_sceneEquilibrium;
			const ndInt32 index = scan[key];
			sceneBodyArray[index] = body;
			scan[key] ++;
		}
	});

	ParallelExecute(BuildBodyArray);
	ndInt32 sum = 0;
	ndInt32 threadCount = GetThreadCount();
	for (ndInt32 j = 0; j < 2; ++j)
	{
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			const ndInt32 count = scans[i][j];
			scans[i][j] = sum;
			sum += count;
		}
	}
	
	ndInt32 movingBodyCount = scans[0][1] - scans[0][0];
	m_sceneBodyArray.SetCount(m_bodyList.GetCount());
	if (movingBodyCount)
	{
		ParallelExecute(CompactMovingBodies);
	}
	m_sceneBodyArray.SetCount(movingBodyCount);
	
	if (m_rootNode && m_rootNode->GetAsSceneTreeNode())
	{
		const ndInt32 bodyCount = m_bodyList.GetCount();
		const ndInt32 cutoffCount = (ndExp2(bodyCount) + 1) * movingBodyCount;
		if (cutoffCount < bodyCount)
		{
			auto UpdateSceneBvh = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(UpdateSceneBvh);
				const ndArray<ndBodyKinematic*>& view = m_sceneBodyArray;
				const ndStartEnd startEnd(view.GetCount(), threadIndex, threadCount);
				
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndBodyKinematic* const body = view[i];
					ndSceneBodyNode* const bodyNode = (ndSceneBodyNode*)m_fitness[body->m_bodyNodeIndex];
					dAssert(bodyNode->GetAsSceneBodyNode());
					dAssert(bodyNode->GetBody() == body);
				
					const ndSceneNode* const root = (m_rootNode->GetLeft() && m_rootNode->GetRight()) ? nullptr : m_rootNode;
					dAssert(root == nullptr);
					for (ndSceneTreeNode* parent = (ndSceneTreeNode*)bodyNode->m_parent; parent != root; parent = (ndSceneTreeNode*)parent->m_parent)
					{
						dAssert(parent->GetAsSceneTreeNode());
						ndScopeSpinLock lock(parent->m_lock);
						const ndVector minBox(parent->m_left->m_minBox.GetMin(parent->m_right->m_minBox));
						const ndVector maxBox(parent->m_left->m_maxBox.GetMax(parent->m_right->m_maxBox));
						if (dBoxInclusionTest(minBox, maxBox, parent->m_minBox, parent->m_maxBox))
						{
							break;
						}
						parent->m_minBox = minBox;
						parent->m_maxBox = maxBox;
					}
				}
			});
	
			D_TRACKTIME_NAMED(UpdateSceneBvhLight);
			ParallelExecute(UpdateSceneBvh);
		}
		else
		{
			ndUnsigned32 start = 0;
			ndUnsigned32 count = 0;
			auto UpdateSceneBvh = ndMakeObject::ndFunction([this, &start, &count](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(UpdateSceneBvh);
				ndSceneTreeNode** const nodes = (ndSceneTreeNode**)&m_fitness[start];
				const ndStartEnd startEnd(count, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndSceneTreeNode* const node = nodes[i];
					dAssert(node && node->GetAsSceneNode());
				
					const ndVector minBox(node->m_left->m_minBox.GetMin(node->m_right->m_minBox));
					const ndVector maxBox(node->m_left->m_maxBox.GetMax(node->m_right->m_maxBox));
					if (!dBoxInclusionTest(minBox, maxBox, node->m_minBox, node->m_maxBox))
					{
						node->m_minBox = minBox;
						node->m_maxBox = maxBox;
					}
				}
			});
	
			D_TRACKTIME_NAMED(UpdateSceneBvhFull);
			for (ndUnsigned32 i = 0; i < m_fitness.m_scansCount; ++i)
			{
				start = m_fitness.m_scans[i];
				count = m_fitness.m_scans[i + 1] - start;
				ParallelExecute(UpdateSceneBvh);
			}
		}
	}
	
	ndBodyKinematic* const sentinelBody = m_sentinelBody;
	sentinelBody->PrepareStep(GetActiveBodyArray().GetCount() - 1);
	
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

#ifndef D_NEW_SCENE

ndUnsigned32 ndScene::BuildSmallBvhTree(ndSceneNode** const parentsArray, ndUnsigned32 bashCount)
{
	ndUnsigned32 depthLevel[D_MAX_THREADS_COUNT];
	auto SmallBhvNodes = ndMakeObject::ndFunction([this, parentsArray, bashCount, &depthLevel](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(SmallBhvNodes);
		const ndCellScanPrefix* const srcCellNodes = &m_cellCounts0[0];
		const ndCellScanPrefix* const newParentsDest = &m_cellCounts1[0];
		const ndBottomUpCell* const nodesCells = &m_cellBuffer0[0];
		dAssert(m_cellCounts0.GetCount() == m_cellCounts1.GetCount());

		auto MakeTwoNodesTree = [](ndSceneTreeNode* const root, ndSceneNode* const left, ndSceneNode* const right)
		{
			left->m_bhvLinked = 1;
			right->m_bhvLinked = 1;
			root->m_bhvLinked = 1;
			root->m_left = left;
			root->m_right = right;

			left->m_parent = root;
			right->m_parent = root;
			root->m_parent = nullptr;

			root->m_minBox = left->m_minBox.GetMin(right->m_minBox);
			root->m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
		};

		auto MakeThreeNodesTree = [](ndSceneTreeNode* const root, ndSceneTreeNode* const subRoot, ndSceneNode* const node0, ndSceneNode* const node1, ndSceneNode* const node2)
		{
			class ndNodeOrder
			{
				public:
				ndVector m_p0;
				ndVector m_p1;
				ndSceneNode* m_node0;
				ndSceneNode* m_node1;
				ndSceneNode* m_node2;
				ndFloat32 m_area;
			};

			ndNodeOrder order[3];

			order[0].m_node0 = node0;
			order[0].m_node1 = node1;
			order[0].m_node2 = node2;

			order[1].m_node0 = node1;
			order[1].m_node1 = node2;
			order[1].m_node2 = node0;

			order[2].m_node0 = node2;
			order[2].m_node1 = node0;
			order[2].m_node2 = node1;

			for (ndInt32 i = 0; i < 3; ++i)
			{
				order[i].m_p0 = order[i].m_node0->m_minBox.GetMin(order[i].m_node1->m_minBox);
				order[i].m_p1 = order[i].m_node0->m_maxBox.GetMax(order[i].m_node1->m_maxBox);
				const ndVector size(order[i].m_p1 - order[i].m_p0);
				order[i].m_area = size.DotProduct(size.ShiftTripleRight()).GetScalar();
			}

			if (order[2].m_area < order[1].m_area)
			{
				ndSwap(order[1], order[2]);
			}
			if (order[1].m_area < order[0].m_area)
			{
				ndSwap(order[1], order[0]);
			}

			root->m_bhvLinked = 1;
			node0->m_bhvLinked = 1;
			node1->m_bhvLinked = 1;
			node2->m_bhvLinked = 1;
			subRoot->m_bhvLinked = 1;

			subRoot->m_parent = root;
			subRoot->m_left = order[0].m_node0;
			subRoot->m_right = order[0].m_node1;
			subRoot->m_minBox = order[0].m_p0;
			subRoot->m_maxBox = order[0].m_p1;
			subRoot->m_left->m_parent = subRoot;
			subRoot->m_right->m_parent = subRoot;

			root->m_parent = nullptr;
			root->m_right = subRoot;
			root->m_left = order[0].m_node2;
			root->m_left->m_parent = root;

			root->m_minBox = root->m_left->m_minBox.GetMin(root->m_right->m_minBox);
			root->m_maxBox = root->m_left->m_maxBox.GetMax(root->m_right->m_maxBox);
		};

		ndUnsigned32 maxDepth = 0;
		const ndStartEnd startEnd(bashCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndUnsigned32 nodesCount = newParentsDest[i + 1].m_location - newParentsDest[i].m_location;

			ndUnsigned32 depth = 0;
			if (nodesCount == 1)
			{
				const ndUnsigned32 childIndex = srcCellNodes[i].m_location;
				const ndUnsigned32 parentIndex = newParentsDest[i].m_location;
				ndSceneNode* const node0 = nodesCells[childIndex + 0].m_node;
				ndSceneNode* const node1 = nodesCells[childIndex + 1].m_node;
				ndSceneTreeNode* const root = parentsArray[parentIndex]->GetAsSceneTreeNode();
				dAssert(root);
				dAssert(!root->m_bhvLinked);
				MakeTwoNodesTree(root, node0, node1);
				root->m_bhvLinked = 0;
			}
			else if (nodesCount == 2)
			{
				const ndUnsigned32 childIndex = srcCellNodes[i].m_location;
				const ndUnsigned32 parentIndex = newParentsDest[i].m_location;

				ndSceneNode* const node0 = nodesCells[childIndex + 0].m_node;
				ndSceneNode* const node1 = nodesCells[childIndex + 1].m_node;
				ndSceneNode* const node2 = nodesCells[childIndex + 2].m_node;

				ndSceneTreeNode* const root = parentsArray[parentIndex + 0]->GetAsSceneTreeNode();
				ndSceneTreeNode* const subParent = parentsArray[parentIndex + 1]->GetAsSceneTreeNode();

				dAssert(root);
				dAssert(!root->m_bhvLinked);
				MakeThreeNodesTree(root, subParent, node0, node1, node2);
				root->m_bhvLinked = 0;
				depth += 1;
			}
			else if (nodesCount > 2)
			{
				class ndBlockSegment
				{
					public:
					ndInt32 m_start;
					ndInt32 m_count;
					ndInt32 m_rootNodeIndex;
					ndUnsigned32 m_depth;
				};

				ndBlockSegment stackPool[8];

				ndUnsigned32 stack = 1;
				ndUnsigned32 rootNodeIndex = newParentsDest[i].m_location;
				stackPool[0].m_depth = 0;
				stackPool[0].m_rootNodeIndex = rootNodeIndex;
				stackPool[0].m_start = srcCellNodes[i].m_location;
				stackPool[0].m_count = srcCellNodes[i + 1].m_location - srcCellNodes[i].m_location;

				ndSceneNode* const rootNode = parentsArray[rootNodeIndex];
				rootNodeIndex++;

				ndUnsigned32 maxStack = 0;
				while (stack)
				{
					stack--;
					const ndBlockSegment block (stackPool[stack]);
					dAssert(block.m_count > 2);

					maxStack = ndMax(maxStack, stack);

					ndVector minP(ndFloat32(1.0e15f));
					ndVector maxP(ndFloat32(-1.0e15f));
					ndVector median(ndVector::m_zero);
					ndVector varian(ndVector::m_zero);

					for (ndInt32 j = 0; j < block.m_count; ++j)
					{
						ndSceneNode* const node = nodesCells[block.m_start + j].m_node;
						minP = minP.GetMin(node->m_minBox);
						maxP = maxP.GetMax(node->m_maxBox);
						ndVector p(ndVector::m_half * (node->m_minBox + node->m_maxBox));
						median += p;
						varian += p * p;
					}

					ndSceneTreeNode* const root = parentsArray[block.m_rootNodeIndex]->GetAsSceneTreeNode();
					root->m_minBox = minP;
					root->m_maxBox = maxP;
					root->m_bhvLinked = 1;

					ndInt32 index = 0;
					ndFloat32 maxVarian = ndFloat32(-1.0e15f);
					varian = varian.Scale(ndFloat32(block.m_count)) - median * median;
					for (ndInt32 j = 0; j < 3; ++j)
					{
						if (varian[j] > maxVarian)
						{
							index = j;
							maxVarian = varian[j];
						}
					}

					ndInt32 index0 = block.m_start + block.m_count / 2;
					if (maxVarian > ndFloat32(1.0e-3f))
					{
						class ndCompareContext
						{
							public:
							ndFloat32 m_midPoint;
							ndUnsigned32 m_index;
						};

						class ndCompareKey
						{
							public:
							ndCompareKey(const void* const context)
							{
								const ndCompareContext* const info = (ndCompareContext*)context;

								m_index = info->m_index;
								m_midPoint = info->m_midPoint;
							}

							ndUnsigned32 GetKey(const ndBottomUpCell& cell)
							{
								const ndSceneNode* const node = cell.m_node;
								const ndVector p(ndVector::m_half * (node->m_minBox + node->m_maxBox));
								ndUnsigned32 key = p[m_index] >= m_midPoint;
								return key;
							}

							ndFloat32 m_midPoint;
							ndUnsigned32 m_index;
						};

						ndCompareContext info;
						ndUnsigned32 scan[8];

						info.m_index = index;
						info.m_midPoint = median[index] / ndFloat32(block.m_count);
						ndCountingSortInPlace<ndBottomUpCell, ndCompareKey, 2>(&m_cellBuffer0[block.m_start], &m_cellBuffer1[block.m_start], block.m_count, scan, &info);
						index0 = block.m_start + scan[1];
					}

					dAssert(index0 > block.m_start);
					dAssert(index0 < (block.m_start + block.m_count));

					ndUnsigned32 count0 = index0 - block.m_start;

					dAssert(count0);
					if (count0 == 1)
					{
						ndSceneNode* const node = m_cellBuffer0[block.m_start].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_left = node;
					}
					else if (count0 == 2)
					{
						ndSceneNode* const node0 = m_cellBuffer0[block.m_start + 0].m_node;
						ndSceneNode* const node1 = m_cellBuffer0[block.m_start + 1].m_node;
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeTwoNodesTree(parent, node0, node1);
						parent->m_parent = root;
						root->m_left = parent;
						maxStack = ndMax(maxStack, block.m_depth + 1);
					}
					else if (count0 == 3)
					{
						ndSceneNode* const node0 = m_cellBuffer0[block.m_start + 0].m_node;
						ndSceneNode* const node1 = m_cellBuffer0[block.m_start + 1].m_node;
						ndSceneNode* const node2 = m_cellBuffer0[block.m_start + 2].m_node;

						ndSceneTreeNode* const grandParent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeThreeNodesTree(grandParent, parent, node0, node1, node2);
						grandParent->m_parent = root;
						root->m_left = grandParent;
						maxStack = ndMax(maxStack, block.m_depth + 2);
					}
					else
					{
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						parent->m_bhvLinked = 1;
						parent->m_parent = root;
						parent->m_left = nullptr;
						parent->m_right = nullptr;
						root->m_left = parent;
						
						stackPool[stack].m_count = count0;
						stackPool[stack].m_start = block.m_start;
						stackPool[stack].m_depth = block.m_depth + 1;
						stackPool[stack].m_rootNodeIndex = rootNodeIndex;

						stack++;
						rootNodeIndex++;
						dAssert(stack < sizeof(stackPool) / sizeof(stackPool[0]));
					}

					ndUnsigned32 count1 = block.m_start + block.m_count - index0;
					dAssert(count1);
					if (count1 == 1)
					{
						ndSceneNode* const node = m_cellBuffer0[index0].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_right = node;
					}
					else if (count1 == 2)
					{
						ndSceneNode* const node0 = m_cellBuffer0[index0 + 0].m_node;
						ndSceneNode* const node1 = m_cellBuffer0[index0 + 1].m_node;
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeTwoNodesTree(parent, node0, node1);
						parent->m_parent = root;
						root->m_right = parent;
						maxStack = ndMax(maxStack, block.m_depth + 1);
					}
					else if (count1 == 3)
					{
						ndSceneNode* const node0 = m_cellBuffer0[index0 + 0].m_node;
						ndSceneNode* const node1 = m_cellBuffer0[index0 + 1].m_node;
						ndSceneNode* const node2 = m_cellBuffer0[index0 + 2].m_node;

						ndSceneTreeNode* const grandParent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeThreeNodesTree(grandParent, parent, node0, node1, node2);
						grandParent->m_parent = root;
						root->m_right = grandParent;
						maxStack = ndMax(maxStack, block.m_depth + 2);
					}
					else
					{
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						parent->m_bhvLinked = 1;
						parent->m_parent = root;
						parent->m_left = nullptr;
						parent->m_right = nullptr;
						root->m_right = parent;

						stackPool[stack].m_start = index0;
						stackPool[stack].m_count = count1;
						stackPool[stack].m_depth = block.m_depth + 1;
						stackPool[stack].m_rootNodeIndex = rootNodeIndex;

						stack++;
						rootNodeIndex++;
						dAssert(stack < sizeof(stackPool) / sizeof(stackPool[0]));
					}
				}
				rootNode->m_bhvLinked = 0;
				depth = maxStack;
			}
			#ifdef _DEBUG
			else if (nodesCount == 0)
			{
				ndUnsigned32 index = srcCellNodes[i].m_location;
				ndSceneNode* const node = nodesCells[index].m_node;
				dAssert(!node->m_bhvLinked);
			}
			#endif		

			maxDepth = ndMax(maxDepth, depth);
		}
		depthLevel[threadIndex] = maxDepth;
	});
	ParallelExecute(SmallBhvNodes);

	ndUnsigned32 depth = 0;
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		depth = ndMax(depth, depthLevel[i]);
	}
	return depth;
}

bool ndScene::BuildBvhTreeInitNodes(ndSceneNode** const srcArray, ndSceneNode** const parentsArray)
{
	D_TRACKTIME();
	auto CopyBodyNodes = ndMakeObject::ndFunction([this, srcArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CopyBodyNodes);
		const ndUnsigned32 baseCount = m_fitness.GetCount() / 2;
		dAssert(baseCount == ndUnsigned32(GetActiveBodyArray().GetCount() - 1));
		ndSceneBodyNode** const bodySceneNodes = (ndSceneBodyNode**)&m_fitness[baseCount];

		const ndStartEnd startEnd(baseCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndSceneBodyNode* const node = bodySceneNodes[i];
			dAssert(node && node->GetAsSceneBodyNode());
			ndBodyKinematic* const body = node->GetBody();

			body->m_sceneNodeIndex = i;
			body->m_bodyNodeIndex = baseCount + i;
			node->m_bhvLinked = 0;
			node->m_depthLevel = 0;
			node->m_parent = nullptr;
			node->SetAabb(body->m_minAabb, body->m_maxAabb);
			srcArray[i] = node;
		}
	});
	
	auto CopySceneNode = ndMakeObject::ndFunction([this, parentsArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CopySceneNode);
		const ndUnsigned32 baseCount = m_fitness.GetCount() / 2;
		ndSceneTreeNode** const sceneNodes = (ndSceneTreeNode**)&m_fitness[0];
		
		const ndStartEnd startEnd(baseCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndSceneTreeNode* const node = sceneNodes[i];
			dAssert(node && node->GetAsSceneTreeNode());
			parentsArray[i] = node;
			node->m_bhvLinked = 0;
			node->m_depthLevel = 0;
			node->m_left = nullptr;
			node->m_right = nullptr;
			node->m_parent = nullptr;
		}
	});
	
	UpdateBodyList();
	bool ret = false;
	if (m_fitness.GetCount())
	{
		ret = true;
		ParallelExecute(CopyBodyNodes);
		ParallelExecute(CopySceneNode);
	}
	return ret;
}

ndScene::BoxInfo ndScene::BuildBvhTreeCalculateLeafBoxes(ndSceneNode** const srcArray)
{
	D_TRACKTIME();
	ndVector boxes[D_MAX_THREADS_COUNT][2];
	ndFloat32 boxSizes[D_MAX_THREADS_COUNT];
	auto CalculateBoxSize = ndMakeObject::ndFunction([this, srcArray, &boxSizes, &boxes](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateBoxSize);
		ndVector minP(ndFloat32(1.0e15f));
		ndVector maxP(ndFloat32(-1.0e15f));
		ndFloat32 minSize = ndFloat32(1.0e15f);

		ndUnsigned32 leafNodesCount = GetActiveBodyArray().GetCount() - 1;
		const ndStartEnd startEnd(leafNodesCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndSceneBodyNode* const node = (ndSceneBodyNode*)srcArray[i];
			dAssert(((ndSceneNode*)node)->GetAsSceneBodyNode());
			minP = minP.GetMin(node->m_minBox);
			maxP = maxP.GetMax(node->m_maxBox);
			const ndVector size(node->m_maxBox - node->m_minBox);
			ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
			minSize = ndMin(maxDim, minSize);
		}
		boxes[threadIndex][0] = minP;
		boxes[threadIndex][1] = maxP;
		boxSizes[threadIndex] = minSize;
	});
	ParallelExecute(CalculateBoxSize);

	ndVector minP(ndFloat32(1.0e15f));
	ndVector maxP(ndFloat32(-1.0e15f));
	ndFloat32 minBoxSize = ndFloat32(1.0e15f);
	const ndInt32 threadCount = GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		minP = minP.GetMin(boxes[i][0]);
		maxP = maxP.GetMax(boxes[i][1]);
		minBoxSize = ndMin(minBoxSize, boxSizes[i]);
	}

	BoxInfo info;
	info.m_origin = minP;
	info.m_size = ndVector::m_triplexMask & ndVector(minBoxSize);
	return info;
}

ndSceneNode* ndScene::BuildBvhTree()
{
	D_TRACKTIME();
	const ndUnsigned32 baseCount = m_bodyList.GetCount();
	m_scratchBuffer.SetCount(4 * (baseCount + 4) * sizeof(ndSceneNode*));
	
	ndSceneNode** srcArray = (ndSceneNode**)&m_scratchBuffer[0];
	ndSceneNode** tmpArray = &srcArray[2 * (baseCount + 4)];
	ndSceneNode** parentsArray = &srcArray[baseCount];
	
	ndUnsigned32 leafNodesCount = baseCount;
	
	enum
	{
		m_linkedCell,
		m_insideCell,
		m_outsideCell,
	};
	
	class ndGridClassifier
	{
		public:
		ndGridClassifier(void* const boxInfo)
		{
			const BoxInfo* const info = (BoxInfo*)boxInfo;
			m_size = info->m_size;
			m_origin = info->m_origin;
			m_invSize = ndVector::m_triplexMask & ndVector(ndFloat32(1.0f) / m_size.m_x);
	
			m_code[0] = m_outsideCell;
			m_code[1] = m_insideCell;
			m_code[2] = m_linkedCell;
			m_code[3] = m_linkedCell;
		}
	
		ndUnsigned32 GetKey(const ndSceneNode* const node) const
		{
			const ndVector minPosit((m_invSize * (node->m_minBox - m_origin)).GetInt());
			const ndVector maxPosit((m_invSize * (node->m_maxBox - m_origin)).GetInt());
			const ndInt32 x0 = minPosit.m_ix;
			const ndInt32 y0 = minPosit.m_iy;
			const ndInt32 z0 = minPosit.m_iz;
			const ndInt32 x1 = maxPosit.m_ix;
			const ndInt32 y1 = maxPosit.m_iy;
			const ndInt32 z1 = maxPosit.m_iz;
	
			dAssert(x0 >= 0);
			dAssert(y0 >= 0);
			dAssert(z0 >= 0);
			dAssert(x1 >= 0);
			dAssert(y1 >= 0);
			dAssert(z1 >= 0);
	
			const ndUnsigned32 test_x = (((x1 - x0)) >> 1) == 0;
			const ndUnsigned32 test_y = (((y1 - y0)) >> 1) == 0;
			const ndUnsigned32 test_z = (((z1 - z0)) >> 1) == 0;
			const ndUnsigned32 test = test_x & test_y & test_z;
			const ndUnsigned32 codeIndex = node->m_bhvLinked * 2 + test;
			return m_code[codeIndex];
		}
	
		ndVector m_size;
		ndVector m_invSize;
		ndVector m_origin;
		ndUnsigned32 m_code[4];
	};
	
	class ndSortCell_xlow
	{
		public:
		ndSortCell_xlow(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return cell.m_x & 0xff;
		}
	};
	
	class ndSortCell_ylow
	{
		public:
		ndSortCell_ylow(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return cell.m_y & 0xff;
		}
	};
	
	class ndSortCell_zlow
	{
		public:
		ndSortCell_zlow(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return cell.m_z & 0xff;
		}
	};
	
	class ndSortCell_xMid
	{
		public:
		ndSortCell_xMid(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return (cell.m_x >> 8) & 0xff;
		}
	};
	
	class ndSortCell_yMid
	{
		public:
		ndSortCell_yMid(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return (cell.m_y >> 8) & 0xff;
		}
	};
	
	class ndSortCell_zMid
	{
		public:
		ndSortCell_zMid(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return (cell.m_z >> 8) & 0xff;
		}
	};
	
	class ndSortCellCount
	{
		public:
		ndSortCellCount(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndCellScanPrefix& cell) const
		{
			return cell.m_cellTest;
		}
	};

	if (!BuildBvhTreeInitNodes(srcArray, parentsArray))
	{
		return nullptr;
	}
	BoxInfo info(BuildBvhTreeCalculateLeafBoxes(srcArray));
	
	ndUnsigned32 prefixScan[8];
	ndInt32 maxGrids[D_MAX_THREADS_COUNT][3];
	
	ndUnsigned32 depthLevel = 1;
	const ndInt32 threadCount = GetThreadCount();
	while (leafNodesCount > 1)
	{
		info.m_size = info.m_size * ndVector::m_two;
		ndCountingSortInPlace<ndSceneNode*, ndGridClassifier, 2>(*this, srcArray, tmpArray, leafNodesCount, prefixScan, &info);
		ndUnsigned32 insideCellsCount = prefixScan[m_insideCell + 1] - prefixScan[m_insideCell];
		if (insideCellsCount)
		{
			ndGridClassifier gridClassifier(&info);
			m_cellBuffer0.SetCount(insideCellsCount);
			m_cellBuffer1.SetCount(insideCellsCount);
			const ndUnsigned32 linkedNodes = prefixScan[m_linkedCell + 1] - prefixScan[m_linkedCell];
			srcArray += linkedNodes;
			leafNodesCount -= linkedNodes;
			auto MakeGrids = ndMakeObject::ndFunction([this, srcArray, &gridClassifier, &maxGrids](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(MakeGrids);
				const ndVector origin(gridClassifier.m_origin);
				const ndVector invSize(gridClassifier.m_invSize);
	
				const ndStartEnd startEnd(m_cellBuffer0.GetCount(), threadIndex, threadCount);
				ndInt32 max_x = 0;
				ndInt32 max_y = 0;
				ndInt32 max_z = 0;
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndSceneNode* const node = srcArray[i];
					const ndVector dist(node->m_minBox - origin);
					const ndVector posit(invSize * dist);
					const ndVector intPosit(posit.GetInt());
					m_cellBuffer0[i].m_x = intPosit.m_ix;
					m_cellBuffer0[i].m_y = intPosit.m_iy;
					m_cellBuffer0[i].m_z = intPosit.m_iz;
					m_cellBuffer0[i].m_node = node;
					max_x = ndMax(intPosit.m_ix, max_x);
					max_y = ndMax(intPosit.m_iy, max_y);
					max_z = ndMax(intPosit.m_iz, max_z);
				}
				maxGrids[threadIndex][0] = max_x;
				maxGrids[threadIndex][1] = max_y;
				maxGrids[threadIndex][2] = max_z;
			});
			ParallelExecute(MakeGrids);
	
			for (ndInt32 i = 1; i < threadCount; ++i)
			{
				maxGrids[0][0] = ndMax(maxGrids[i][0], maxGrids[0][0]);
				maxGrids[0][1] = ndMax(maxGrids[i][1], maxGrids[0][1]);
				maxGrids[0][2] = ndMax(maxGrids[i][2], maxGrids[0][2]);
			}
	
			ndCountingSort<ndBottomUpCell, ndSortCell_xlow, 8>(*this, m_cellBuffer0, m_cellBuffer1, nullptr, nullptr);
			if (maxGrids[0][0] > 256)
			{
				ndCountingSort<ndBottomUpCell, ndSortCell_xMid, 8>(*this, m_cellBuffer0, m_cellBuffer1, nullptr, nullptr);
				dAssert(maxGrids[0][0] < 256 * 256);
			}
	
			ndCountingSort<ndBottomUpCell, ndSortCell_ylow, 8>(*this, m_cellBuffer0, m_cellBuffer1, nullptr, nullptr);
			if (maxGrids[0][1] > 256)
			{
				ndCountingSort<ndBottomUpCell, ndSortCell_yMid, 8>(*this, m_cellBuffer0, m_cellBuffer1, nullptr, nullptr);
				dAssert(maxGrids[0][1] < 256 * 256);
			}
	
			ndCountingSort<ndBottomUpCell, ndSortCell_zlow, 8>(*this, m_cellBuffer0, m_cellBuffer1, nullptr, nullptr);
			if (maxGrids[0][2] > 256)
			{
				ndCountingSort<ndBottomUpCell, ndSortCell_zMid, 8>(*this, m_cellBuffer0, m_cellBuffer1, nullptr, nullptr);
				dAssert(maxGrids[0][2] < 256 * 256);
			}
	
			ndBottomUpCell sentinelCell;
			sentinelCell.m_x = ndUnsigned32(-1);
			sentinelCell.m_y = ndUnsigned32(-1);
			sentinelCell.m_z = ndUnsigned32(-1);
			sentinelCell.m_node = nullptr;
	
			m_cellBuffer0.PushBack(sentinelCell);
			m_cellBuffer1.PushBack(sentinelCell);
			m_cellCounts0.SetCount(m_cellBuffer0.GetCount());
			m_cellCounts1.SetCount(m_cellBuffer1.GetCount());
			auto MarkCellBounds = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(MarkCellBounds);
				ndCellScanPrefix* const dst = &m_cellCounts0[0];
				const ndStartEnd startEnd(m_cellBuffer0.GetCount() - 1, threadIndex, threadCount);
	
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					const ndBottomUpCell& cell0 = m_cellBuffer0[i + 1];
					const ndBottomUpCell& cell1 = m_cellBuffer0[i];
					const ndUnsigned8 test = (cell0.m_x == cell1.m_x) & (cell0.m_y == cell1.m_y) & (cell0.m_z == cell1.m_z) & (cell1.m_node != nullptr);
					dst[i + 1].m_cellTest = test;
					dst[i + 1].m_location = i + 1;
				}
			});
			ParallelExecute(MarkCellBounds);
	
			m_cellCounts0[0].m_cellTest = 0;
			m_cellCounts0[0].m_location = 0;
			ndCountingSort<ndCellScanPrefix, ndSortCellCount, 1>(*this, m_cellCounts0, m_cellCounts1, prefixScan, nullptr);
	
			ndUnsigned32 sum = 0;
			const ndUnsigned32 bashCount = prefixScan[1] - 1;
			for (ndUnsigned32 i = 0; i < bashCount; ++i)
			{
				const ndUnsigned32 count = m_cellCounts0[i + 1].m_location - m_cellCounts0[i].m_location - 1;
				m_cellCounts1[i].m_location = sum;
				sum += count;
			}
			if (sum)
			{
				m_cellCounts1[bashCount].m_location = sum;
				ndUnsigned32 subTreeDepth = BuildSmallBvhTree(parentsArray, bashCount);
				depthLevel += subTreeDepth;
				auto EnumerateSmallBvh = ndMakeObject::ndFunction([this, parentsArray, sum, depthLevel](ndInt32 threadIndex, ndInt32 threadCount)
				{
					D_TRACKTIME_NAMED(EnumerateSmallBvh);
					const ndStartEnd startEnd(sum, threadIndex, threadCount);
					for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
					{
						ndSceneTreeNode* const root = parentsArray[i]->GetAsSceneTreeNode();
						dAssert(root);
						if (!root->m_parent)
						{
							dAssert(root->m_depthLevel == 0);
							class StackLevel
							{
								public:
								ndSceneTreeNode* m_node;
								ndInt32 m_depthLevel;
							};
	
							ndInt32 stack = 1;
							StackLevel m_stackPool[32];
	
							m_stackPool[0].m_node = root;
							m_stackPool[0].m_depthLevel = depthLevel;
	
							while (stack)
							{
								stack--;
								const StackLevel level(m_stackPool[stack]);
	
								ndSceneTreeNode* const node = level.m_node;
								node->m_depthLevel = level.m_depthLevel;
	
								ndSceneTreeNode* const left = node->m_left->GetAsSceneTreeNode();
								if (left && left->m_depthLevel == 0)
								{
									m_stackPool[stack].m_node = left;
									m_stackPool[stack].m_depthLevel = level.m_depthLevel - 1;
									stack++;
									dAssert(stack < sizeof(m_stackPool) / sizeof(m_stackPool[0]));
								}
	
								ndSceneTreeNode* const right = node->m_right->GetAsSceneTreeNode();
								if (right && right->m_depthLevel == 0)
								{
									m_stackPool[stack].m_node = right;
									m_stackPool[stack].m_depthLevel = level.m_depthLevel - 1;
									stack++;
									dAssert(stack < sizeof(m_stackPool) / sizeof(m_stackPool[0]));
								}
							}
						}
					}
				});
				ParallelExecute(EnumerateSmallBvh);
	
				parentsArray += sum;
				leafNodesCount += sum;
				depthLevel ++;
			}
		}
	}
	
	ndSceneNode* const root = srcArray[0];
	
	class ndSortGetDethpKey
	{
		public:
		ndSortGetDethpKey(const void* const)
		{
		}
	
		ndUnsigned32 GetKey(const ndSceneTreeNode* const node) const
		{
			return node->m_depthLevel;
		}
	};
	
	ndSceneTreeNode** const view = (ndSceneTreeNode**)&m_fitness[0];
	ndSceneTreeNode** const tmpBuffer = (ndSceneTreeNode**)&m_scratchBuffer[0];
	
	ndUnsigned32 scans[257];
	const ndUnsigned32 sceneNodeCount = ndUnsigned32(m_fitness.GetCount() / 2 - 1);
	ndCountingSortInPlace<ndSceneTreeNode*, ndSortGetDethpKey, 8>(*this, &view[0], tmpBuffer, sceneNodeCount, scans, nullptr);
	
	m_fitness.m_scansCount = 0;
	for (ndInt32 i = 1; (i < 257) && (scans[i] < sceneNodeCount); ++i)
	{
		m_fitness.m_scans[i - 1] = scans[i];
		m_fitness.m_scansCount++;
	}
	m_fitness.m_scans[m_fitness.m_scansCount] = scans[m_fitness.m_scansCount + 1];
	dAssert(m_fitness.m_scans[0] == 0);

	dAssert(root->SanityCheck(0));
	return root;
}

ndSceneNode* ndScene::BuildIncrementalBvhTree()
{
	return BuildBvhTree();
}

#else


ndScene::BuildBvhTreeBuildState::BuildBvhTreeBuildState()
	:m_size(ndVector::m_zero)
	,m_origin(ndVector::m_zero)
	,m_cellBuffer0(1024)
	,m_cellBuffer1(1024)
	,m_cellCounts0(1024)
	,m_cellCounts1(1024)
	,m_tempNodeBuffer(1024)
	,m_root(nullptr)
	,m_srcArray(nullptr)
	,m_tmpArray(nullptr)
	,m_parentsArray(nullptr)
	,m_depthLevel(0)
	,m_leafNodesCount(0)
	,m_state(m_beginBuild)
{
}

void ndScene::BuildBvhTreeBuildState::Init(ndUnsigned32 maxCount)
{
	m_depthLevel = 1;
	m_tempNodeBuffer.SetCount(4 * (maxCount + 4));

	m_root = nullptr;
	m_srcArray = &m_tempNodeBuffer[0];
	m_tmpArray = &m_srcArray[2 * (maxCount + 4)];
	m_parentsArray = &m_srcArray[maxCount];
	m_leafNodesCount = maxCount;
}

ndUnsigned32 ndScene::BuildSmallBvhTree(ndSceneNode** const parentsArray, ndUnsigned32 bashCount)
{
	ndUnsigned32 depthLevel[D_MAX_THREADS_COUNT];
	auto SmallBhvNodes = ndMakeObject::ndFunction([this, parentsArray, bashCount, &depthLevel](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(SmallBhvNodes);

		const ndCellScanPrefix* const srcCellNodes = &m_bvhBuildState.m_cellCounts0[0];
		const ndCellScanPrefix* const newParentsDest = &m_bvhBuildState.m_cellCounts1[0];
		const ndBottomUpCell* const nodesCells = &m_bvhBuildState.m_cellBuffer0[0];

		dAssert(m_bvhBuildState.m_cellCounts0.GetCount() == m_bvhBuildState.m_cellCounts1.GetCount());

		auto MakeTwoNodesTree = [](ndSceneTreeNode* const root, ndSceneNode* const left, ndSceneNode* const right)
		{
			left->m_bhvLinked = 1;
			right->m_bhvLinked = 1;
			root->m_bhvLinked = 1;
			root->m_left = left;
			root->m_right = right;

			left->m_parent = root;
			right->m_parent = root;
			root->m_parent = nullptr;

			root->m_minBox = left->m_minBox.GetMin(right->m_minBox);
			root->m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
		};

		auto MakeThreeNodesTree = [](ndSceneTreeNode* const root, ndSceneTreeNode* const subRoot, ndSceneNode* const node0, ndSceneNode* const node1, ndSceneNode* const node2)
		{
			class ndNodeOrder
			{
			public:
				ndVector m_p0;
				ndVector m_p1;
				ndSceneNode* m_node0;
				ndSceneNode* m_node1;
				ndSceneNode* m_node2;
				ndFloat32 m_area;
			};

			ndNodeOrder order[3];

			order[0].m_node0 = node0;
			order[0].m_node1 = node1;
			order[0].m_node2 = node2;

			order[1].m_node0 = node1;
			order[1].m_node1 = node2;
			order[1].m_node2 = node0;

			order[2].m_node0 = node2;
			order[2].m_node1 = node0;
			order[2].m_node2 = node1;

			for (ndInt32 i = 0; i < 3; ++i)
			{
				order[i].m_p0 = order[i].m_node0->m_minBox.GetMin(order[i].m_node1->m_minBox);
				order[i].m_p1 = order[i].m_node0->m_maxBox.GetMax(order[i].m_node1->m_maxBox);
				const ndVector size(order[i].m_p1 - order[i].m_p0);
				order[i].m_area = size.DotProduct(size.ShiftTripleRight()).GetScalar();
			}

			if (order[2].m_area < order[1].m_area)
			{
				ndSwap(order[1], order[2]);
			}
			if (order[1].m_area < order[0].m_area)
			{
				ndSwap(order[1], order[0]);
			}

			root->m_bhvLinked = 1;
			node0->m_bhvLinked = 1;
			node1->m_bhvLinked = 1;
			node2->m_bhvLinked = 1;
			subRoot->m_bhvLinked = 1;

			subRoot->m_parent = root;
			subRoot->m_left = order[0].m_node0;
			subRoot->m_right = order[0].m_node1;
			subRoot->m_minBox = order[0].m_p0;
			subRoot->m_maxBox = order[0].m_p1;
			subRoot->m_left->m_parent = subRoot;
			subRoot->m_right->m_parent = subRoot;

			root->m_parent = nullptr;
			root->m_right = subRoot;
			root->m_left = order[0].m_node2;
			root->m_left->m_parent = root;

			root->m_minBox = root->m_left->m_minBox.GetMin(root->m_right->m_minBox);
			root->m_maxBox = root->m_left->m_maxBox.GetMax(root->m_right->m_maxBox);
		};

		ndUnsigned32 maxDepth = 0;
		const ndStartEnd startEnd(bashCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndUnsigned32 nodesCount = newParentsDest[i + 1].m_location - newParentsDest[i].m_location;

			ndUnsigned32 depth = 0;
			if (nodesCount == 1)
			{
				const ndUnsigned32 childIndex = srcCellNodes[i].m_location;
				const ndUnsigned32 parentIndex = newParentsDest[i].m_location;
				ndSceneNode* const node0 = nodesCells[childIndex + 0].m_node;
				ndSceneNode* const node1 = nodesCells[childIndex + 1].m_node;
				ndSceneTreeNode* const root = parentsArray[parentIndex]->GetAsSceneTreeNode();
				dAssert(root);
				dAssert(!root->m_bhvLinked);
				MakeTwoNodesTree(root, node0, node1);
				root->m_bhvLinked = 0;
			}
			else if (nodesCount == 2)
			{
				const ndUnsigned32 childIndex = srcCellNodes[i].m_location;
				const ndUnsigned32 parentIndex = newParentsDest[i].m_location;

				ndSceneNode* const node0 = nodesCells[childIndex + 0].m_node;
				ndSceneNode* const node1 = nodesCells[childIndex + 1].m_node;
				ndSceneNode* const node2 = nodesCells[childIndex + 2].m_node;

				ndSceneTreeNode* const root = parentsArray[parentIndex + 0]->GetAsSceneTreeNode();
				ndSceneTreeNode* const subParent = parentsArray[parentIndex + 1]->GetAsSceneTreeNode();

				dAssert(root);
				dAssert(!root->m_bhvLinked);
				MakeThreeNodesTree(root, subParent, node0, node1, node2);
				root->m_bhvLinked = 0;
				depth += 1;
			}
			else if (nodesCount > 2)
			{
				class ndBlockSegment
				{
				public:
					ndInt32 m_start;
					ndInt32 m_count;
					ndInt32 m_rootNodeIndex;
					ndUnsigned32 m_depth;
				};

				ndBlockSegment stackPool[8];

				ndUnsigned32 stack = 1;
				ndUnsigned32 rootNodeIndex = newParentsDest[i].m_location;
				stackPool[0].m_depth = 0;
				stackPool[0].m_rootNodeIndex = rootNodeIndex;
				stackPool[0].m_start = srcCellNodes[i].m_location;
				stackPool[0].m_count = srcCellNodes[i + 1].m_location - srcCellNodes[i].m_location;

				ndSceneNode* const rootNode = parentsArray[rootNodeIndex];
				rootNodeIndex++;

				ndUnsigned32 maxStack = 0;
				while (stack)
				{
					stack--;
					const ndBlockSegment block(stackPool[stack]);
					dAssert(block.m_count > 2);

					maxStack = ndMax(maxStack, stack);

					ndVector minP(ndFloat32(1.0e15f));
					ndVector maxP(ndFloat32(-1.0e15f));
					ndVector median(ndVector::m_zero);
					ndVector varian(ndVector::m_zero);

					for (ndInt32 j = 0; j < block.m_count; ++j)
					{
						ndSceneNode* const node = nodesCells[block.m_start + j].m_node;
						minP = minP.GetMin(node->m_minBox);
						maxP = maxP.GetMax(node->m_maxBox);
						ndVector p(ndVector::m_half * (node->m_minBox + node->m_maxBox));
						median += p;
						varian += p * p;
					}

					ndSceneTreeNode* const root = parentsArray[block.m_rootNodeIndex]->GetAsSceneTreeNode();
					root->m_minBox = minP;
					root->m_maxBox = maxP;
					root->m_bhvLinked = 1;

					ndInt32 index = 0;
					ndFloat32 maxVarian = ndFloat32(-1.0e15f);
					varian = varian.Scale(ndFloat32(block.m_count)) - median * median;
					for (ndInt32 j = 0; j < 3; ++j)
					{
						if (varian[j] > maxVarian)
						{
							index = j;
							maxVarian = varian[j];
						}
					}

					ndInt32 index0 = block.m_start + block.m_count / 2;
					if (maxVarian > ndFloat32(1.0e-3f))
					{
						class ndCompareContext
						{
						public:
							ndFloat32 m_midPoint;
							ndUnsigned32 m_index;
						};

						class ndCompareKey
						{
						public:
							ndCompareKey(const void* const context)
							{
								const ndCompareContext* const info = (ndCompareContext*)context;

								m_index = info->m_index;
								m_midPoint = info->m_midPoint;
							}

							ndUnsigned32 GetKey(const ndBottomUpCell& cell)
							{
								const ndSceneNode* const node = cell.m_node;
								const ndVector p(ndVector::m_half * (node->m_minBox + node->m_maxBox));
								ndUnsigned32 key = p[m_index] >= m_midPoint;
								return key;
							}

							ndFloat32 m_midPoint;
							ndUnsigned32 m_index;
						};

						ndCompareContext info;
						ndUnsigned32 scan[8];

						info.m_index = index;
						info.m_midPoint = median[index] / ndFloat32(block.m_count);
						ndCountingSortInPlace<ndBottomUpCell, ndCompareKey, 2>(&m_bvhBuildState.m_cellBuffer0[block.m_start], &m_bvhBuildState.m_cellBuffer1[block.m_start], block.m_count, scan, &info);
						index0 = block.m_start + scan[1];
					}

					dAssert(index0 > block.m_start);
					dAssert(index0 < (block.m_start + block.m_count));

					ndUnsigned32 count0 = index0 - block.m_start;

					dAssert(count0);
					if (count0 == 1)
					{
						ndSceneNode* const node = m_bvhBuildState.m_cellBuffer0[block.m_start].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_left = node;
					}
					else if (count0 == 2)
					{
						ndSceneNode* const node0 = m_bvhBuildState.m_cellBuffer0[block.m_start + 0].m_node;
						ndSceneNode* const node1 = m_bvhBuildState.m_cellBuffer0[block.m_start + 1].m_node;
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeTwoNodesTree(parent, node0, node1);
						parent->m_parent = root;
						root->m_left = parent;
						maxStack = ndMax(maxStack, block.m_depth + 1);
					}
					else if (count0 == 3)
					{
						ndSceneNode* const node0 = m_bvhBuildState.m_cellBuffer0[block.m_start + 0].m_node;
						ndSceneNode* const node1 = m_bvhBuildState.m_cellBuffer0[block.m_start + 1].m_node;
						ndSceneNode* const node2 = m_bvhBuildState.m_cellBuffer0[block.m_start + 2].m_node;

						ndSceneTreeNode* const grandParent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeThreeNodesTree(grandParent, parent, node0, node1, node2);
						grandParent->m_parent = root;
						root->m_left = grandParent;
						maxStack = ndMax(maxStack, block.m_depth + 2);
					}
					else
					{
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						parent->m_bhvLinked = 1;
						parent->m_parent = root;
						parent->m_left = nullptr;
						parent->m_right = nullptr;
						root->m_left = parent;

						stackPool[stack].m_count = count0;
						stackPool[stack].m_start = block.m_start;
						stackPool[stack].m_depth = block.m_depth + 1;
						stackPool[stack].m_rootNodeIndex = rootNodeIndex;

						stack++;
						rootNodeIndex++;
						dAssert(stack < sizeof(stackPool) / sizeof(stackPool[0]));
					}

					ndUnsigned32 count1 = block.m_start + block.m_count - index0;
					dAssert(count1);
					if (count1 == 1)
					{
						ndSceneNode* const node = m_bvhBuildState.m_cellBuffer0[index0].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_right = node;
					}
					else if (count1 == 2)
					{
						ndSceneNode* const node0 = m_bvhBuildState.m_cellBuffer0[index0 + 0].m_node;
						ndSceneNode* const node1 = m_bvhBuildState.m_cellBuffer0[index0 + 1].m_node;
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeTwoNodesTree(parent, node0, node1);
						parent->m_parent = root;
						root->m_right = parent;
						maxStack = ndMax(maxStack, block.m_depth + 1);
					}
					else if (count1 == 3)
					{
						ndSceneNode* const node0 = m_bvhBuildState.m_cellBuffer0[index0 + 0].m_node;
						ndSceneNode* const node1 = m_bvhBuildState.m_cellBuffer0[index0 + 1].m_node;
						ndSceneNode* const node2 = m_bvhBuildState.m_cellBuffer0[index0 + 2].m_node;

						ndSceneTreeNode* const grandParent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						rootNodeIndex++;

						dAssert(root);
						MakeThreeNodesTree(grandParent, parent, node0, node1, node2);
						grandParent->m_parent = root;
						root->m_right = grandParent;
						maxStack = ndMax(maxStack, block.m_depth + 2);
					}
					else
					{
						ndSceneTreeNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
						parent->m_bhvLinked = 1;
						parent->m_parent = root;
						parent->m_left = nullptr;
						parent->m_right = nullptr;
						root->m_right = parent;

						stackPool[stack].m_start = index0;
						stackPool[stack].m_count = count1;
						stackPool[stack].m_depth = block.m_depth + 1;
						stackPool[stack].m_rootNodeIndex = rootNodeIndex;

						stack++;
						rootNodeIndex++;
						dAssert(stack < sizeof(stackPool) / sizeof(stackPool[0]));
					}
				}
				rootNode->m_bhvLinked = 0;
				depth = maxStack;
			}
#ifdef _DEBUG
			else if (nodesCount == 0)
			{
				ndUnsigned32 index = srcCellNodes[i].m_location;
				ndSceneNode* const node = nodesCells[index].m_node;
				dAssert(!node->m_bhvLinked);
			}
#endif		

			maxDepth = ndMax(maxDepth, depth);
		}
		depthLevel[threadIndex] = maxDepth;
	});
	ParallelExecute(SmallBhvNodes);

	ndUnsigned32 depth = 0;
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		depth = ndMax(depth, depthLevel[i]);
	}
	return depth;
}

bool ndScene::BuildBvhTreeInitNodes()
{
	D_TRACKTIME();
	auto CopyBodyNodes = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CopyBodyNodes);
		const ndUnsigned32 baseCount = m_fitness.GetCount() / 2;
		ndSceneNode** const srcArray = m_bvhBuildState.m_srcArray;
		dAssert(baseCount == ndUnsigned32(GetActiveBodyArray().GetCount() - 1));
		ndSceneBodyNode** const bodySceneNodes = (ndSceneBodyNode**)&m_fitness[baseCount];

		const ndStartEnd startEnd(baseCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndSceneBodyNode* const node = bodySceneNodes[i];
			dAssert(node && node->GetAsSceneBodyNode());
			ndBodyKinematic* const body = node->GetBody();

			node->m_bhvLinked = 0;
			node->m_depthLevel = 0;
			node->m_parent = nullptr;
			node->SetAabb(body->m_minAabb, body->m_maxAabb);
			srcArray[i] = node;
		}
	});

	auto CopySceneNode = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CopySceneNode);
		const ndUnsigned32 baseCount = m_fitness.GetCount() / 2;
		ndSceneTreeNode** const sceneNodes = (ndSceneTreeNode**)&m_fitness[0];

		ndSceneNode** const parentsArray = m_bvhBuildState.m_parentsArray;
		const ndStartEnd startEnd(baseCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndSceneTreeNode* const node = sceneNodes[i];
			dAssert(node && node->GetAsSceneTreeNode());

			parentsArray[i] = node;
			node->m_bhvLinked = 0;
			node->m_depthLevel = 0;
			node->m_parent = nullptr;
		}
	});

	UpdateBodyList();
	bool ret = false;
	if (m_fitness.GetCount())
	{
		ret = true;
		m_bvhBuildState.Init(m_bodyList.GetCount());
		ParallelExecute(CopyBodyNodes);
		ParallelExecute(CopySceneNode);
	}
	return ret;
}

void ndScene::BuildBvhTreeCalculateLeafBoxes()
{
	D_TRACKTIME();
	ndVector boxes[D_MAX_THREADS_COUNT][2];
	ndFloat32 boxSizes[D_MAX_THREADS_COUNT];

	auto CalculateBoxSize = ndMakeObject::ndFunction([this, &boxSizes, &boxes](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(CalculateBoxSize);
		ndVector minP(ndFloat32(1.0e15f));
		ndVector maxP(ndFloat32(-1.0e15f));
		ndFloat32 minSize = ndFloat32(1.0e15f);

		ndSceneNode** const srcArray = m_bvhBuildState.m_srcArray;
		const ndUnsigned32 leafNodesCount = m_bvhBuildState.m_leafNodesCount;
		const ndStartEnd startEnd(leafNodesCount, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndSceneNode* const node = srcArray[i];
			dAssert(((ndSceneNode*)node)->GetAsSceneBodyNode());
			minP = minP.GetMin(node->m_minBox);
			maxP = maxP.GetMax(node->m_maxBox);
			const ndVector size(node->m_maxBox - node->m_minBox);
			ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
			minSize = ndMin(maxDim, minSize);
		}
		boxes[threadIndex][0] = minP;
		boxes[threadIndex][1] = maxP;
		boxSizes[threadIndex] = minSize;
	});
	ParallelExecute(CalculateBoxSize);

	ndVector minP(ndFloat32(1.0e15f));
	ndVector maxP(ndFloat32(-1.0e15f));
	ndFloat32 minBoxSize = ndFloat32(1.0e15f);
	const ndInt32 threadCount = GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		minP = minP.GetMin(boxes[i][0]);
		maxP = maxP.GetMax(boxes[i][1]);
		minBoxSize = ndMin(minBoxSize, boxSizes[i]);
	}

	m_bvhBuildState.m_origin = minP;
	m_bvhBuildState.m_size = ndVector::m_triplexMask & ndVector(minBoxSize);
}

void ndScene::BuildBvhGenerateLayerGrids()
{
	D_TRACKTIME();
	enum
	{
		m_linkedCell,
		m_insideCell,
		m_outsideCell,
	};

	class ndGridClassifier
	{
		public:
		ndGridClassifier(void* const data)
		{
			const BuildBvhTreeBuildState* const info = (BuildBvhTreeBuildState*)data;
			m_size = info->m_size;
			m_origin = info->m_origin;
			m_invSize = ndVector::m_triplexMask & ndVector(ndFloat32(1.0f) / m_size.m_x);

			m_code[0] = m_outsideCell;
			m_code[1] = m_insideCell;
			m_code[2] = m_linkedCell;
			m_code[3] = m_linkedCell;
		}

		ndUnsigned32 GetKey(const ndSceneNode* const node) const
		{
			const ndVector minPosit((m_invSize * (node->m_minBox - m_origin)).GetInt());
			const ndVector maxPosit((m_invSize * (node->m_maxBox - m_origin)).GetInt());
			const ndInt32 x0 = minPosit.m_ix;
			const ndInt32 y0 = minPosit.m_iy;
			const ndInt32 z0 = minPosit.m_iz;
			const ndInt32 x1 = maxPosit.m_ix;
			const ndInt32 y1 = maxPosit.m_iy;
			const ndInt32 z1 = maxPosit.m_iz;

			dAssert(x0 >= 0);
			dAssert(y0 >= 0);
			dAssert(z0 >= 0);
			dAssert(x1 >= 0);
			dAssert(y1 >= 0);
			dAssert(z1 >= 0);

			const ndUnsigned32 test_x = (((x1 - x0)) >> 1) == 0;
			const ndUnsigned32 test_y = (((y1 - y0)) >> 1) == 0;
			const ndUnsigned32 test_z = (((z1 - z0)) >> 1) == 0;
			const ndUnsigned32 test = test_x & test_y & test_z;
			const ndUnsigned32 codeIndex = node->m_bhvLinked * 2 + test;
			return m_code[codeIndex];
		}

		ndVector m_size;
		ndVector m_invSize;
		ndVector m_origin;
		ndUnsigned32 m_code[4];
	};

	class ndSortCell_xlow
	{
		public:
		ndSortCell_xlow(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return cell.m_x & 0xff;
		}
	};

	class ndSortCell_ylow
	{
		public:
		ndSortCell_ylow(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return cell.m_y & 0xff;
		}
	};

	class ndSortCell_zlow
	{
		public:
		ndSortCell_zlow(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return cell.m_z & 0xff;
		}
	};

	class ndSortCell_xMid
	{
	public:
		ndSortCell_xMid(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return (cell.m_x >> 8) & 0xff;
		}
	};

	class ndSortCell_yMid
	{
		public:
		ndSortCell_yMid(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return (cell.m_y >> 8) & 0xff;
		}
	};

	class ndSortCell_zMid
	{
		public:
		ndSortCell_zMid(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndBottomUpCell& cell) const
		{
			return (cell.m_z >> 8) & 0xff;
		}
	};

	class ndSortCellCount
	{
		public:
		ndSortCellCount(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndCellScanPrefix& cell) const
		{
			return cell.m_cellTest;
		}
	};

	ndUnsigned32 prefixScan[8];
	ndInt32 maxGrids[D_MAX_THREADS_COUNT][3];

	ndCountingSortInPlace<ndSceneNode*, ndGridClassifier, 2>(*this, m_bvhBuildState.m_srcArray, m_bvhBuildState.m_tmpArray, m_bvhBuildState.m_leafNodesCount, prefixScan, &m_bvhBuildState);
	ndUnsigned32 insideCellsCount = prefixScan[m_insideCell + 1] - prefixScan[m_insideCell];
	if (insideCellsCount)
	{
		m_bvhBuildState.m_cellBuffer0.SetCount(insideCellsCount);
		m_bvhBuildState.m_cellBuffer1.SetCount(insideCellsCount);
		const ndUnsigned32 linkedNodes = prefixScan[m_linkedCell + 1] - prefixScan[m_linkedCell];
		m_bvhBuildState.m_srcArray += linkedNodes;
		m_bvhBuildState.m_leafNodesCount -= linkedNodes;
		auto MakeGrids = ndMakeObject::ndFunction([this, &maxGrids](ndInt32 threadIndex, ndInt32 threadCount)
		{
			D_TRACKTIME_NAMED(MakeGrids);

			const ndGridClassifier gridClassifier(&m_bvhBuildState);
			const ndVector origin(gridClassifier.m_origin);
			const ndVector invSize(gridClassifier.m_invSize);

			ndSceneNode** const srcArray = m_bvhBuildState.m_srcArray;

			const ndStartEnd startEnd(m_bvhBuildState.m_cellBuffer0.GetCount(), threadIndex, threadCount);
			ndInt32 max_x = 0;
			ndInt32 max_y = 0;
			ndInt32 max_z = 0;
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndSceneNode* const node = srcArray[i];
				const ndVector dist(node->m_minBox - origin);
				const ndVector posit(invSize * dist);
				const ndVector intPosit(posit.GetInt());
				m_bvhBuildState.m_cellBuffer0[i].m_x = intPosit.m_ix;
				m_bvhBuildState.m_cellBuffer0[i].m_y = intPosit.m_iy;
				m_bvhBuildState.m_cellBuffer0[i].m_z = intPosit.m_iz;
				m_bvhBuildState.m_cellBuffer0[i].m_node = node;
				max_x = ndMax(intPosit.m_ix, max_x);
				max_y = ndMax(intPosit.m_iy, max_y);
				max_z = ndMax(intPosit.m_iz, max_z);
			}
			maxGrids[threadIndex][0] = max_x;
			maxGrids[threadIndex][1] = max_y;
			maxGrids[threadIndex][2] = max_z;
		});
		ParallelExecute(MakeGrids);

		const ndInt32 threadCount = GetThreadCount();
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			maxGrids[0][0] = ndMax(maxGrids[i][0], maxGrids[0][0]);
			maxGrids[0][1] = ndMax(maxGrids[i][1], maxGrids[0][1]);
			maxGrids[0][2] = ndMax(maxGrids[i][2], maxGrids[0][2]);
		}

		ndCountingSort<ndBottomUpCell, ndSortCell_xlow, 8>(*this, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, nullptr);
		if (maxGrids[0][0] > 256)
		{
			ndCountingSort<ndBottomUpCell, ndSortCell_xMid, 8>(*this, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, nullptr);
			dAssert(maxGrids[0][0] < 256 * 256);
		}

		ndCountingSort<ndBottomUpCell, ndSortCell_ylow, 8>(*this, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, nullptr);
		if (maxGrids[0][1] > 256)
		{
			ndCountingSort<ndBottomUpCell, ndSortCell_yMid, 8>(*this, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, nullptr);
			dAssert(maxGrids[0][1] < 256 * 256);
		}

		ndCountingSort<ndBottomUpCell, ndSortCell_zlow, 8>(*this, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, nullptr);
		if (maxGrids[0][2] > 256)
		{
			ndCountingSort<ndBottomUpCell, ndSortCell_zMid, 8>(*this, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, nullptr);
			dAssert(maxGrids[0][2] < 256 * 256);
		}

		ndBottomUpCell sentinelCell;
		sentinelCell.m_x = ndUnsigned32(-1);
		sentinelCell.m_y = ndUnsigned32(-1);
		sentinelCell.m_z = ndUnsigned32(-1);
		sentinelCell.m_node = nullptr;

		m_bvhBuildState.m_cellBuffer0.PushBack(sentinelCell);
		m_bvhBuildState.m_cellBuffer1.PushBack(sentinelCell);
		m_bvhBuildState.m_cellCounts0.SetCount(m_bvhBuildState.m_cellBuffer0.GetCount());
		m_bvhBuildState.m_cellCounts1.SetCount(m_bvhBuildState.m_cellBuffer1.GetCount());
		auto MarkCellBounds = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
		{
			D_TRACKTIME_NAMED(MarkCellBounds);
			ndCellScanPrefix* const dst = &m_bvhBuildState.m_cellCounts0[0];
			const ndStartEnd startEnd(m_bvhBuildState.m_cellBuffer0.GetCount() - 1, threadIndex, threadCount);

			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndBottomUpCell& cell0 = m_bvhBuildState.m_cellBuffer0[i + 1];
				const ndBottomUpCell& cell1 = m_bvhBuildState.m_cellBuffer0[i];
				const ndUnsigned8 test = (cell0.m_x == cell1.m_x) & (cell0.m_y == cell1.m_y) & (cell0.m_z == cell1.m_z) & (cell1.m_node != nullptr);
				dst[i + 1].m_cellTest = test;
				dst[i + 1].m_location = i + 1;
			}
		});
		ParallelExecute(MarkCellBounds);

		m_bvhBuildState.m_cellCounts0[0].m_cellTest = 0;
		m_bvhBuildState.m_cellCounts0[0].m_location = 0;
		ndCountingSort<ndCellScanPrefix, ndSortCellCount, 1>(*this, m_bvhBuildState.m_cellCounts0, m_bvhBuildState.m_cellCounts1, prefixScan, nullptr);

		ndUnsigned32 sum = 0;
		const ndUnsigned32 bashCount = prefixScan[1] - 1;
		for (ndUnsigned32 i = 0; i < bashCount; ++i)
		{
			const ndUnsigned32 count = m_bvhBuildState.m_cellCounts0[i + 1].m_location - m_bvhBuildState.m_cellCounts0[i].m_location - 1;
			m_bvhBuildState.m_cellCounts1[i].m_location = sum;
			sum += count;
		}
		if (sum)
		{
			m_bvhBuildState.m_cellCounts1[bashCount].m_location = sum;
			ndUnsigned32 subTreeDepth = BuildSmallBvhTree(m_bvhBuildState.m_parentsArray, bashCount);
			m_bvhBuildState.m_depthLevel += subTreeDepth;
			auto EnumerateSmallBvh = ndMakeObject::ndFunction([this, sum](ndInt32 threadIndex, ndInt32 threadCount)
			{
				D_TRACKTIME_NAMED(EnumerateSmallBvh);

				ndUnsigned32 depthLevel = m_bvhBuildState.m_depthLevel;
				ndSceneNode** const parentsArray = m_bvhBuildState.m_parentsArray;

				const ndStartEnd startEnd(sum, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndSceneTreeNode* const root = parentsArray[i]->GetAsSceneTreeNode();
					dAssert(root);
					if (!root->m_parent)
					{
						dAssert(root->m_depthLevel == 0);
						class StackLevel
						{
						public:
							ndSceneTreeNode* m_node;
							ndInt32 m_depthLevel;
						};

						ndInt32 stack = 1;
						StackLevel m_stackPool[32];

						m_stackPool[0].m_node = root;
						m_stackPool[0].m_depthLevel = depthLevel;

						while (stack)
						{
							stack--;
							const StackLevel level(m_stackPool[stack]);

							ndSceneTreeNode* const node = level.m_node;
							node->m_depthLevel = level.m_depthLevel;

							ndSceneTreeNode* const left = node->m_left->GetAsSceneTreeNode();
							if (left && left->m_depthLevel == 0)
							{
								m_stackPool[stack].m_node = left;
								m_stackPool[stack].m_depthLevel = level.m_depthLevel - 1;
								stack++;
								dAssert(stack < sizeof(m_stackPool) / sizeof(m_stackPool[0]));
							}

							ndSceneTreeNode* const right = node->m_right->GetAsSceneTreeNode();
							if (right && right->m_depthLevel == 0)
							{
								m_stackPool[stack].m_node = right;
								m_stackPool[stack].m_depthLevel = level.m_depthLevel - 1;
								stack++;
								dAssert(stack < sizeof(m_stackPool) / sizeof(m_stackPool[0]));
							}
						}
					}
				}
			});
			ParallelExecute(EnumerateSmallBvh);

			m_bvhBuildState.m_parentsArray += sum;
			m_bvhBuildState.m_leafNodesCount += sum;
			m_bvhBuildState.m_depthLevel++;
		}
	}
}

void ndScene::BuildBvhTreeSetNodesDepth()
{
	class ndSortGetDethpKey
	{
		public:
		ndSortGetDethpKey(const void* const)
		{
		}

		ndUnsigned32 GetKey(const ndSceneTreeNode* const node) const
		{
			return node->m_depthLevel;
		}
	};

	ndUnsigned32 sceneNodeCount = m_fitness.GetCount() / 2 - 1;
	ndSceneTreeNode** const view = (ndSceneTreeNode**)&m_fitness[0];
	ndSceneTreeNode** tmpBuffer = (ndSceneTreeNode**)&m_bvhBuildState.m_tempNodeBuffer[0];

	ndUnsigned32 scans[257];
	ndCountingSortInPlace<ndSceneTreeNode*, ndSortGetDethpKey, 8>(*this, &view[0], tmpBuffer, sceneNodeCount, scans, nullptr);

	m_fitness.m_scansCount = 0;
	for (ndInt32 i = 1; (i < 257) && (scans[i] < sceneNodeCount); ++i)
	{
		m_fitness.m_scans[i - 1] = scans[i];
		m_fitness.m_scansCount++;
	}
	m_fitness.m_scans[m_fitness.m_scansCount] = scans[m_fitness.m_scansCount + 1];
	dAssert(m_fitness.m_scans[0] == 0);
}

ndSceneNode* ndScene::BuildBvhTree()
{
	D_TRACKTIME();

	//while (!BuildIncrementalBvhTree());

	if (!BuildBvhTreeInitNodes())
	{
		return nullptr;
	}
	BuildBvhTreeCalculateLeafBoxes();
	while (m_bvhBuildState.m_leafNodesCount > 1)
	{
		m_bvhBuildState.m_size = m_bvhBuildState.m_size * ndVector::m_two;
		BuildBvhGenerateLayerGrids();
	}

	m_bvhBuildState.m_root = m_bvhBuildState.m_srcArray[0];

	BuildBvhTreeSetNodesDepth();
	dAssert(m_bvhBuildState.m_root->SanityCheck(0));

	return m_bvhBuildState.m_root;
}

ndSceneNode* ndScene::BuildIncrementalBvhTree()
{
	D_TRACKTIME();
	ndSceneNode* root = nullptr;
	switch (m_bvhBuildState.m_state)
	{
		case BuildBvhTreeBuildState::m_beginBuild:
		{
			if (BuildBvhTreeInitNodes())
			{
				m_bvhBuildState.m_state = m_bvhBuildState.m_calculateBoxes;
			}
			break;
		}

		case BuildBvhTreeBuildState::m_calculateBoxes:
		{
			BuildBvhTreeCalculateLeafBoxes();
			m_bvhBuildState.m_state = m_bvhBuildState.m_buildLayer;
			break;
		}

		case BuildBvhTreeBuildState::m_buildLayer:
		{
			if (m_bvhBuildState.m_leafNodesCount > 1)
			{
				m_bvhBuildState.m_size = m_bvhBuildState.m_size * ndVector::m_two;
				BuildBvhGenerateLayerGrids();
			}
			else
			{
				m_bvhBuildState.m_root = m_bvhBuildState.m_srcArray[0];
				m_bvhBuildState.m_state = m_bvhBuildState.m_enumarateLayers;
			}
			break;
		}

		case BuildBvhTreeBuildState::m_enumarateLayers:
		{
			BuildBvhTreeSetNodesDepth();
			m_bvhBuildState.m_state = m_bvhBuildState.m_endBuild;
			break;
		}

		case BuildBvhTreeBuildState::m_endBuild:
		{
			root = m_bvhBuildState.m_root;
			dAssert(m_bvhBuildState.m_root->SanityCheck(0));
			break;
		}

		default:
			dAssert(0);
	}

	return root;
}

#endif

