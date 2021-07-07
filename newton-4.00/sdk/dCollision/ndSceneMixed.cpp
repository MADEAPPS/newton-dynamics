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


#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBodyKinematic.h"
#include "ndSceneNode.h"
#include "ndSceneMixed.h"

ndSceneMixed::ndSceneMixed()
	:ndScene()
	,m_treeEntropy(dFloat32(0.0f))
	,m_fitness()
{
}

ndSceneMixed::~ndSceneMixed()
{
	Cleanup();
}

void ndSceneMixed::AddNode(ndSceneNode* const newNode)
{
	if (m_rootNode) 
	{
		ndSceneTreeNode* const node = InsertNode(m_rootNode, newNode);
		m_fitness.AddNode(node);
		if (!node->m_parent) 
		{
			m_rootNode = node;
		}
	}
	else
	{
		m_rootNode = newNode;
	}
}

void ndSceneMixed::RemoveNode(ndSceneNode* const node)
{
	if (node->m_parent) 
	{
		if (!node->m_parent->GetAsSceneAggregate()) 
		{
			ndSceneTreeNode* const parent = (ndSceneTreeNode*)node->m_parent;
			if (parent->m_parent) 
			{
				ndSceneAggregate* const aggregate = parent->m_parent->GetAsSceneAggregate();
				if (aggregate)
				{
					dAssert(0);
#if 0
					if (parent->m_left == node) 
					{
						dAssert(parent->m_right);
						aggregate->m_root = parent->m_right;
						parent->m_right->m_parent = aggregate;
						parent->m_right = nullptr;
					}
					else 
					{
						dAssert(parent->m_right == node);
						aggregate->m_root = parent->m_left;
						parent->m_left->m_parent = aggregate;
						parent->m_left = nullptr;
					}
					parent->m_parent = nullptr;
#endif
				}
				else 
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

			if (parent->m_fitnessNode) 
			{
				ndBodyKinematic* const body = node->GetBody();
				if (body && body->GetSceneAggregate()) 
				{
					dAssert(0);
					//body->GetSceneAggregate()->m_fitnessList.Remove(parent->m_fitnessNode);
					//body->SetSceneAggregate(nullptr);
				}
				else 
				{
					m_fitness.RemoveNode(parent);
				}
			}
			delete parent;
		}
		else 
		{
			dAssert(0);
#if 0
			ndSceneAggregate* const aggregate = (ndSceneAggregate*)node->m_parent;
			dBody* const body = node->GetBody();
			dAssert(body);
			dAssert(body->GetSceneAggregate() == aggregate);
			body->SetSceneAggregate(nullptr);
			aggregate->m_root = nullptr;
			node->m_parent = nullptr;
			delete node;
#endif
		}
	}
	else 
	{
		delete node;
		m_rootNode = nullptr;
	}
}

bool ndSceneMixed::AddBody(ndBodyKinematic* const body)
{
	if (ndScene::AddBody(body))
	{
		body->UpdateCollisionMatrix();
		ndSceneBodyNode* const bodyNode = new ndSceneBodyNode(body);
		AddNode(bodyNode);
		return true;
	}
	return false;
}

void ndSceneMixed::Cleanup()
{
	Sync();
	m_contactList.DeleteAllContacts();
	while (m_bodyList.GetFirst())
	{
		ndBodyKinematic* const body = m_bodyList.GetFirst()->GetInfo();
		RemoveBody(body);
		delete body;
	}
	ndContact::FlushFreeList();
	ndBodyList::FlushFreeList();
	ndFitnessList::FlushFreeList();
	ndBodyKinematic::ReleaseMemory();
	m_activeBodyArray.Resize(256);
	m_activeConstraintArray.Resize(256);
}

bool ndSceneMixed::RemoveBody(ndBodyKinematic* const body)
{
	ndSceneBodyNode* const node = body->GetSceneBodyNode();
	if (node)
	{
		RemoveNode(node);
	}
	return ndScene::RemoveBody(body);
}

void ndSceneMixed::BalanceScene()
{
	D_TRACKTIME();
	UpdateFitness(m_fitness, m_treeEntropy, &m_rootNode);
}

void ndSceneMixed::FindCollidinPairs(dInt32, ndBodyKinematic* const body, bool oneWay)
{
	//ndSceneNode* const leafNode = body->GetSceneBodyNode();
	ndSceneBodyNode* const bodyNode = body->GetSceneBodyNode();
	body->m_broaphaseEquilibrium = 1;

	ndSceneAggregate* const aggregateNode = bodyNode->GetAsSceneAggregate();
	if (aggregateNode)
	{
		dAssert(0);
		//aggregateNode->SubmitSelfPairs(timestep, threadID);
	}

	if (oneWay)
	{
		for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
		{
			ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
			dAssert(!parent->GetAsSceneBodyNode());
			dAssert(!parent->GetAsSceneAggregate());
			ndSceneNode* const sibling = parent->m_right;
			if (sibling != ptr)
			{
				SubmitPairs(bodyNode, sibling);
			}
		}
	}
	else
	{
		for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
		{
			ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
			dAssert(!parent->GetAsSceneBodyNode());
			dAssert(!parent->GetAsSceneAggregate());
			ndSceneNode* const rightSibling = parent->m_right;
			if (rightSibling != ptr)
			{
				SubmitPairs(bodyNode, rightSibling);
			}
			else 
			{
				SubmitPairs(bodyNode, parent->m_left);
			}
		}
	}
}

void ndSceneMixed::DebugScene(ndSceneTreeNotiFy* const notify)
{
	for (ndFitnessList::dNode* node = m_fitness.GetFirst(); node; node = node->GetNext())
	{
		//notify->OnDebugNode(node->GetInfo());
		if (node->GetInfo()->GetLeft()->GetAsSceneBodyNode())
		{
			notify->OnDebugNode(node->GetInfo()->GetLeft());
		}
		if (node->GetInfo()->GetRight()->GetAsSceneBodyNode())
		{
			notify->OnDebugNode(node->GetInfo()->GetRight());
		}
	}
}
/*
void ndSceneMixed::FindCollidinPairs(dInt32 threadIndex, ndBodyKinematic* const body)
{
	ndSceneNode* const leafNode = body->GetSceneBodyNode();

	if (m_fullScan)
	{
		ndSceneAggregate* const aggregateNode = leafNode->GetAsSceneAggregate();
		if (aggregateNode)
		{
			dAssert(0);
			//aggregateNode->SubmitSelfPairs(timestep, threadID);
		}

		for (ndSceneNode* ptr = leafNode; ptr->m_parent; ptr = ptr->m_parent)
		{
			ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
			dAssert(!parent->GetAsSceneBodyNode());
			ndSceneNode* const sibling = parent->m_right;
			if (sibling != ptr)
			{
				//SubmitPairs(bodyNode, sibling, timestep, 0, threadIndex);
				SubmitPairs(leafNode, sibling);
			}
		}
	}
	else
	{
		dAssert(0);
		//const dgBodyInfo* const bodyArray = &m_world->m_bodiesMemory[0];
		//const dInt32 bodyCount = descriptor->m_atomicPendingBodiesCount;
		//dInt32* const atomicIndex = &descriptor->m_atomicIndex;
		//
		//for (dInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		//	ndSceneNode* const sceneNode = bodyArray[i].m_body->GetSceneNode();
		//	dAssert(sceneNode->ndSceneBodyNode());
		//	dAssert(!sceneNode->GetBody() || (sceneNode->GetBody()->GetSceneNode() == sceneNode));
		//
		//	for (ndSceneNode* ptr = sceneNode; ptr->m_parent; ptr = ptr->m_parent) {
		//		ndSceneTreeNode* const parent = (ndSceneTreeNode*)ptr->m_parent;
		//		if (!parent->IsAggregate()) {
		//			dAssert(!parent->ndSceneBodyNode());
		//			ndSceneNode* const rightSibling = parent->m_right;
		//			if (rightSibling != ptr) {
		//				SubmitPairs(sceneNode, rightSibling, timestep, threadCount, threadID);
		//			}
		//			else {
		//				SubmitPairs(sceneNode, parent->m_left, timestep, threadCount, threadID);
		//			}
		//		}
		//	}
		//}
	}
}
*/

dFloat32 ndSceneMixed::RayCast(ndRayCastNotify& callback, const dVector& globalOrigin, const dVector& globalDest) const
{
	dVector p0(globalOrigin & dVector::m_triplexMask);
	dVector p1(globalDest & dVector::m_triplexMask);

	dFloat32 param = dFloat32(1.2f);
	if (m_rootNode) 
	{
		dVector segment(p1 - p0);
		dAssert(segment.m_w == dFloat32(0.0f));
		dFloat32 dist2 = segment.DotProduct(segment).GetScalar();
		if (dist2 > dFloat32(1.0e-8f)) 
		{
			dFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
			const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

			dFastRayTest ray(p0, p1);

			stackPool[0] = m_rootNode;
			distance[0] = ray.BoxIntersect(m_rootNode->m_minBox, m_rootNode->m_maxBox);
			param = ndScene::RayCast(callback, stackPool, distance, 1, ray);
		}
	}
	return param;
}

bool ndSceneMixed::ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const dMatrix& globalOrigin, const dVector& globalDest) const
{
	bool state = false;
	if (m_rootNode) 
	{
		dVector boxP0;
		dVector boxP1;
		dAssert(globalOrigin.TestOrthogonal());
		convexShape.CalculateAABB(globalOrigin, boxP0, boxP1);
	
		dFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
		const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];
	
		dVector velocA((globalDest - globalOrigin.m_posit) & dVector::m_triplexMask);
		dVector velocB(dVector::m_zero);
		dFastRayTest ray(dVector::m_zero, velocA);
	
		dVector minBox(m_rootNode->m_minBox - boxP1);
		dVector maxBox(m_rootNode->m_maxBox - boxP0);
		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);
		state = ndScene::ConvexCast(callback, stackPool, distance, 1, ray, convexShape, globalOrigin, globalDest);
	}
	
	return state;
}

