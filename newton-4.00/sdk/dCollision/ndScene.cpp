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

#define D_CONTACT_DELAY_FRAMES		4
#define D_NARROW_PHASE_DIST			dFloat32 (0.2f)
#define D_CONTACT_TRANSLATION_ERROR	dFloat32 (1.0e-3f)
#define D_CONTACT_ANGULAR_ERROR		(dFloat32 (0.25f * dDegreeToRad))

dVector ndScene::m_velocTol(dFloat32(1.0e-16f));
dVector ndScene::m_angularContactError2(D_CONTACT_ANGULAR_ERROR * D_CONTACT_ANGULAR_ERROR);
dVector ndScene::m_linearContactError2(D_CONTACT_TRANSLATION_ERROR * D_CONTACT_TRANSLATION_ERROR);

D_MSV_NEWTON_ALIGN_32
class ndScene::ndSpliteInfo
{
	public:
	ndSpliteInfo(ndSceneNode** const boxArray, dInt32 boxCount)
	{
		dVector minP(dFloat32(1.0e15f));
		dVector maxP(-dFloat32(1.0e15f));

		if (boxCount == 2)
		{
			m_axis = 1;
			for (dInt32 i = 0; i < boxCount; i++)
			{
				ndSceneNode* const node = boxArray[i];
				dAssert(node->GetAsSceneBodyNode());
				minP = minP.GetMin(node->m_minBox);
				maxP = maxP.GetMax(node->m_maxBox);
			}
		}
		else
		{
			dVector median(dVector::m_zero);
			dVector varian(dVector::m_zero);
			for (dInt32 i = 0; i < boxCount; i++)
			{
				ndSceneNode* const node = boxArray[i];
				dAssert(node->GetAsSceneBodyNode());
				minP = minP.GetMin(node->m_minBox);
				maxP = maxP.GetMax(node->m_maxBox);
				dVector p(dVector::m_half * (node->m_minBox + node->m_maxBox));
				median += p;
				varian += p * p;
			}

			varian = varian.Scale(dFloat32(boxCount)) - median * median;

			dInt32 index = 0;
			dFloat32 maxVarian = dFloat32(-1.0e15f);
			for (dInt32 i = 0; i < 3; i++)
			{
				if (varian[i] > maxVarian)
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			dVector center = median.Scale(dFloat32(1.0f) / dFloat32(boxCount));

			dFloat32 test = center[index];

			dInt32 i0 = 0;
			dInt32 i1 = boxCount - 1;
			do
			{
				for (; i0 <= i1; i0++)
				{
					ndSceneNode* const node = boxArray[i0];
					dFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dFloat32(0.5f);
					if (val > test)
					{
						break;
					}
				}

				for (; i1 >= i0; i1--)
				{
					ndSceneNode* const node = boxArray[i1];
					dFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dFloat32(0.5f);
					if (val < test)
					{
						break;
					}
				}

				if (i0 < i1)
				{
					dSwap(boxArray[i0], boxArray[i1]);
					i0++;
					i1--;
				}

			} while (i0 <= i1);

			if (i0 > 0)
			{
				i0--;
			}
			if ((i0 + 1) >= boxCount)
			{
				i0 = boxCount - 2;
			}
			m_axis = i0 + 1;
		}

		dAssert(maxP.m_x - minP.m_x >= dFloat32(0.0f));
		dAssert(maxP.m_y - minP.m_y >= dFloat32(0.0f));
		dAssert(maxP.m_z - minP.m_z >= dFloat32(0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	dVector m_p0;
	dVector m_p1;
	dInt32 m_axis;
} D_GCC_NEWTON_ALIGN_32 ;

ndScene::ndFitnessList::ndFitnessList()
	:dList <ndSceneTreeNode*, dContainersFreeListAlloc<ndSceneTreeNode*>>()
	,m_currentCost(dFloat32(0.0f))
	,m_currentNode(nullptr)
	,m_index(0)
{
}

void ndScene::ndFitnessList::AddNode(ndSceneTreeNode* const node)
{
	node->m_fitnessNode = Append(node);
}

void ndScene::ndFitnessList::RemoveNode(ndSceneTreeNode* const node)
{
	dAssert(node->m_fitnessNode);
	if (node->m_fitnessNode == m_currentNode)
	{
		m_currentNode = node->m_fitnessNode->GetNext();
	}
	Remove(node->m_fitnessNode);
	node->m_fitnessNode = nullptr;
}

dFloat64 ndScene::ndFitnessList::TotalCost() const
{
	D_TRACKTIME();
	dFloat64 cost = dFloat32(0.0f);
	for (dNode* node = GetFirst(); node; node = node->GetNext()) {
		ndSceneNode* const box = node->GetInfo();
		cost += box->m_surfaceArea;
	}
	return cost;
}
	
ndScene::ndScene()
	:dClassAlloc()
	,dThreadPool("newtonWorker")
	,m_bodyList()
	,m_contactList()
	,m_activeBodyArray(1024)
	,m_activeConstraintArray()
	,m_contactLock()
	,m_rootNode(nullptr)
	,m_contactNotifyCallback(new ndContactNotify())
	,m_treeEntropy(dFloat32(0.0f))
	,m_fitness()
	,m_timestep(dFloat32 (0.0f))
	,m_sleepBodies(0)
	,m_lru(D_CONTACT_DELAY_FRAMES)
	,m_fullScan(true)
{
	m_contactNotifyCallback->m_scene = this;
}

ndScene::~ndScene()
{
	Cleanup();
	Finish();
	delete m_contactNotifyCallback;
	ndContactList::FlushFreeList();
	ndContactPointList::FlushFreeList();
	ndShapeCompound::ndTreeArray::FlushFreeList();
}

void ndScene::CollisionOnlyUpdate()
{
	D_TRACKTIME();
	Begin();
	m_lru = m_lru + 1;
	BuildBodyArray();
	UpdateAabb();
	BalanceScene();
	FindCollidingPairs();
	BuildContactArray();
	CalculateContacts();
	DeleteDeadContact();
	End();
}

void ndScene::ThreadFunction()
{
	D_TRACKTIME();
	CollisionOnlyUpdate();
}

void ndScene::Update(dFloat32 timestep)
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
	for (ndFitnessList::dNode* node = m_fitness.GetFirst(); node; node = node->GetNext())
	{
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

bool ndScene::AddBody(ndBodyKinematic* const body)
{
	if ((body->m_scene == nullptr) && (body->m_sceneNode == nullptr))
	{
		ndBodyList::dNode* const node = m_bodyList.Append(body);
		body->SetSceneNodes(this, node);
		m_contactNotifyCallback->OnBodyAdded(body);

		body->UpdateCollisionMatrix();
		ndSceneBodyNode* const bodyNode = new ndSceneBodyNode(body);
		AddNode(bodyNode);
		return true;
	}
	return false;
}

bool ndScene::RemoveBody(ndBodyKinematic* const body)
{
	ndSceneBodyNode* const node = body->GetSceneBodyNode();
	if (node)
	{
		RemoveNode(node);
	}

	ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
	while (contactMap.GetRoot())
	{
		ndContact* const contact = contactMap.GetRoot()->GetInfo();
		m_contactList.DeleteContact(contact);
	}

	if (body->m_scene && body->m_sceneNode)
	{
		m_bodyList.Remove(body->m_sceneNode);
		body->SetSceneNodes(nullptr, nullptr);
		m_contactNotifyCallback->OnBodyRemoved(body);
		return true;
	}
	return false;
}

ndSceneTreeNode* ndScene::InsertNode(ndSceneNode* const root, ndSceneNode* const node)
{
	dVector p0;
	dVector p1;

	ndSceneNode* sibling = root;
	dFloat32 surfaceArea = CalculateSurfaceArea(node, sibling, p0, p1);
	while (!sibling->GetAsSceneBodyNode() && (surfaceArea >= sibling->m_surfaceArea))
	{
		sibling->m_minBox = p0;
		sibling->m_maxBox = p1;
		sibling->m_surfaceArea = surfaceArea;
	
		dVector leftP0;
		dVector leftP1;
		dFloat32 leftSurfaceArea = CalculateSurfaceArea(node, sibling->GetLeft(), leftP0, leftP1);
		
		dVector rightP0;
		dVector rightP1;
		dFloat32 rightSurfaceArea = CalculateSurfaceArea(node, sibling->GetRight(), rightP0, rightP1);
	
		if (leftSurfaceArea < rightSurfaceArea) 
		{
			p0 = leftP0;
			p1 = leftP1;
			sibling = sibling->GetLeft();
			surfaceArea = leftSurfaceArea;
		}
		else 
		{
			p0 = rightP0;
			p1 = rightP1;
			sibling = sibling->GetRight();
			surfaceArea = rightSurfaceArea;
		}
	}
	
	ndSceneTreeNode* const parent = new ndSceneTreeNode(sibling, node);
	return parent;
}

void ndScene::RotateLeft(ndSceneTreeNode* const node, ndSceneNode** const root)
{
	dVector cost1P0;
	dVector cost1P1;

	ndSceneTreeNode* const parent = (ndSceneTreeNode*)node->m_parent;
	dAssert(parent && !parent->GetAsSceneBodyNode());
	dFloat32 cost1 = CalculateSurfaceArea(node->m_left, parent->m_left, cost1P0, cost1P1);

	dVector cost2P0;
	dVector cost2P1;
	dFloat32 cost2 = CalculateSurfaceArea(node->m_right, parent->m_left, cost2P0, cost2P1);

	dFloat32 cost0 = node->m_surfaceArea;
	if ((cost1 <= cost0) && (cost1 <= cost2)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ndSceneTreeNode* const grandParent = (ndSceneTreeNode*)parent->m_parent;
		if (grandParent) 
		{
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_left->m_parent = parent;
		parent->m_right = node->m_left;
		node->m_left = parent;

		parent->m_minBox = cost1P0;
		parent->m_maxBox = cost1P1;
		parent->m_surfaceArea = cost1;
	}
	else if ((cost2 <= cost0) && (cost2 <= cost1)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ndSceneTreeNode* const grandParent = (ndSceneTreeNode*)parent->m_parent;
		if (grandParent) 
		{
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_right->m_parent = parent;
		parent->m_right = node->m_right;
		node->m_right = parent;

		parent->m_minBox = cost2P0;
		parent->m_maxBox = cost2P1;
		parent->m_surfaceArea = cost2;
	}
}

void ndScene::RotateRight(ndSceneTreeNode* const node, ndSceneNode** const root)
{
	dVector cost1P0;
	dVector cost1P1;

	ndSceneTreeNode* const parent = (ndSceneTreeNode*)node->m_parent;
	dAssert(parent && !parent->GetAsSceneBodyNode());

	dFloat32 cost1 = CalculateSurfaceArea(node->m_right, parent->m_right, cost1P0, cost1P1);

	dVector cost2P0;
	dVector cost2P1;
	dFloat32 cost2 = CalculateSurfaceArea(node->m_left, parent->m_right, cost2P0, cost2P1);

	dFloat32 cost0 = node->m_surfaceArea;
	if ((cost1 <= cost0) && (cost1 <= cost2)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ndSceneTreeNode* const grandParent = (ndSceneTreeNode*)parent->m_parent;
		if (grandParent) 
		{
			dAssert(!grandParent->GetAsSceneBodyNode());
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_right->m_parent = parent;
		parent->m_left = node->m_right;
		node->m_right = parent;
		parent->m_minBox = cost1P0;
		parent->m_maxBox = cost1P1;
		parent->m_surfaceArea = cost1;

	}
	else if ((cost2 <= cost0) && (cost2 <= cost1)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ndSceneTreeNode* const grandParent = (ndSceneTreeNode*)parent->m_parent;
		if (parent->m_parent) 
		{
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_left->m_parent = parent;
		parent->m_left = node->m_left;
		node->m_left = parent;

		parent->m_minBox = cost2P0;
		parent->m_maxBox = cost2P1;
		parent->m_surfaceArea = cost2;
	}
}

void ndScene::ImproveNodeFitness(ndSceneTreeNode* const node, ndSceneNode** const root)
{
	dAssert(node->GetLeft());
	dAssert(node->GetRight());

	ndSceneNode* const parent = node->m_parent;
	if (parent && parent->m_parent) 
	{
		dAssert(!parent->GetAsSceneBodyNode());
		if (parent->GetLeft() == node) 
		{
			RotateRight(node, root);
		}
		else 
		{
			RotateLeft(node, root);
		}
	}
	dAssert(!m_rootNode->m_parent);
}

dFloat64 ndScene::ReduceEntropy(ndFitnessList& fitness, ndSceneNode** const root)
{
	D_TRACKTIME();

	if (!fitness.m_currentNode) 
	{
		fitness.m_currentCost = fitness.TotalCost();
		fitness.m_currentNode = fitness.GetFirst();
	}
	else
	{
		dInt32 count = 0;
		ndFitnessList::dNode* node = fitness.m_currentNode;
		for ( ;node && count < 64; node = node->GetNext())
		{
			count++;
			ImproveNodeFitness(node->GetInfo(), root);
		}
		fitness.m_currentNode = node;
	}
	return fitness.m_currentCost;
}

dInt32 ndScene::CompareNodes(const ndSceneNode* const nodeA, const ndSceneNode* const nodeB, void* const)
{
	dFloat32 areaA = nodeA->m_surfaceArea;
	dFloat32 areaB = nodeB->m_surfaceArea;
	if (areaA < areaB) 
	{
		return 1;
	}
	if (areaA > areaB) 
	{
		return -1;
	}
	return 0;
}

void ndScene::UpdateFitness(ndFitnessList& fitness, dFloat64& oldEntropy, ndSceneNode** const root)
{
	if (*root) 
	{
		D_TRACKTIME();
		ndSceneNode* const parent = (*root)->m_parent;

		(*root)->m_parent = nullptr;
		dFloat64 entropy = ReduceEntropy(fitness, root);

		if ((entropy > (oldEntropy * dFloat32(1.5f))) || (entropy < (oldEntropy * dFloat32(0.75f)))) 
		{
			if (fitness.GetFirst()) 
			{
				ndSceneNode** const leafArray = dAlloca(ndSceneNode*, fitness.GetCount() * 2 + 16);

				dInt32 leafNodesCount = 0;
				for (ndFitnessList::dNode* nodePtr = fitness.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) 
				{
					ndSceneNode* const node = nodePtr->GetInfo();
					ndSceneNode* const leftNode = node->GetLeft();

					ndBodyKinematic* const leftBody = leftNode->GetBody();
					if (leftBody) 
					{
						node->SetAabb(leftBody->m_minAabb, leftBody->m_maxAabb);
						leafArray[leafNodesCount] = leftNode;
						leafNodesCount++;
					}

					ndSceneNode* const rightNode = node->GetRight();
					ndBodyKinematic* const rightBody = rightNode->GetBody();
					if (rightBody) 
					{
						rightNode->SetAabb(rightBody->m_minAabb, rightBody->m_maxAabb);
						leafArray[leafNodesCount] = rightNode;
						leafNodesCount++;
					}
				}
				
				ndFitnessList::dNode* nodePtr = fitness.GetFirst();
				dSortIndirect(leafArray, leafNodesCount, CompareNodes);
				
				*root = BuildTopDownBig(leafArray, 0, leafNodesCount - 1, &nodePtr);
				dAssert(!(*root)->m_parent);
				entropy = fitness.TotalCost();
				fitness.m_currentCost = entropy;
			}
			oldEntropy = entropy;
		}
		(*root)->m_parent = parent;
	}
}

void ndScene::BalanceScene()
{
	D_TRACKTIME();
	UpdateFitness(m_fitness, m_treeEntropy, &m_rootNode);
}

ndSceneNode* ndScene::BuildTopDown(ndSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ndFitnessList::dNode** const nextNode)
{
	dAssert(firstBox >= 0);
	dAssert(lastBox >= 0);

	if (lastBox == firstBox) 
	{
		return leafArray[firstBox];
	}
	else 
	{
		ndSpliteInfo info(&leafArray[firstBox], lastBox - firstBox + 1);

		ndSceneTreeNode* const parent = (*nextNode)->GetInfo();
		parent->m_parent = nullptr;
		*nextNode = (*nextNode)->GetNext();

		parent->SetAabb(info.m_p0, info.m_p1);

		parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + info.m_axis - 1, nextNode);
		parent->m_left->m_parent = parent;

		parent->m_right = BuildTopDown(leafArray, firstBox + info.m_axis, lastBox, nextNode);
		parent->m_right->m_parent = parent;
		return parent;
	}
}

ndSceneNode* ndScene::BuildTopDownBig(ndSceneNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ndFitnessList::dNode** const nextNode)
{
	if (lastBox == firstBox) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, nextNode);
	}

	dInt32 midPoint = -1;
	const dFloat32 scale = dFloat32(1.0f / 64.0f);
	const ndSceneNode* const node0 = leafArray[firstBox];
	const dInt32 count = lastBox - firstBox;
	dFloat32 area0 = scale * node0->m_surfaceArea;
	for (dInt32 i = 1; i <= count; i++) 
	{
		const ndSceneNode* const node1 = leafArray[firstBox + i];
		dFloat32 area1 = node1->m_surfaceArea;
		if (area0 > area1) 
		{
			midPoint = i - 1;
			break;
		}
	}

	if (midPoint == -1) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, nextNode);
	}
	else 
	{
		ndSceneTreeNode* const parent = (*nextNode)->GetInfo();

		parent->m_parent = nullptr;
		*nextNode = (*nextNode)->GetNext();

		parent->m_right = BuildTopDown(leafArray, firstBox, firstBox + midPoint, nextNode);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDownBig(leafArray, firstBox + midPoint + 1, lastBox, nextNode);
		parent->m_left->m_parent = parent;

		dVector minP(parent->m_left->m_minBox.GetMin(parent->m_right->m_minBox));
		dVector maxP(parent->m_left->m_maxBox.GetMax(parent->m_right->m_maxBox));
		parent->SetAabb(minP, maxP);

		return parent;
	}
}

void ndScene::UpdateTransformNotify(dInt32 threadIndex, ndBodyKinematic* const body)
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

void ndScene::UpdateAabb(dInt32, ndBodyKinematic* const body)
{
	ndSceneBodyNode* const bodyNode = body->GetSceneBodyNode();
	body->UpdateCollisionMatrix();
		
	dAssert(!bodyNode->GetLeft());
	dAssert(!bodyNode->GetRight());
	dAssert(!body->GetCollisionShape().GetShape()->GetAsShapeNull());

	const dInt32 test = dBoxInclusionTest(body->m_minAabb, body->m_maxAabb, bodyNode->m_minBox, bodyNode->m_maxBox);
	if (!test) 
	{
		body->m_broaphaseEquilibrium = 0;
		bodyNode->SetAabb(body->m_minAabb, body->m_maxAabb);

		if (!m_rootNode->GetAsSceneBodyNode()) 
		{
			const ndSceneNode* const root = (m_rootNode->GetLeft() && m_rootNode->GetRight()) ? nullptr : m_rootNode;
			dAssert(root == nullptr);
			for (ndSceneNode* parent = bodyNode->m_parent; parent != root; parent = parent->m_parent) 
			{
				dScopeSpinLock lock(parent->m_lock);
				dVector minBox;
				dVector maxBox;
				dFloat32 area = CalculateSurfaceArea(parent->GetLeft(), parent->GetRight(), minBox, maxBox);
				if (dBoxInclusionTest(minBox, maxBox, parent->m_minBox, parent->m_maxBox)) 
				{
					break;
				}
				parent->m_minBox = minBox;
				parent->m_maxBox = maxBox;
				parent->m_surfaceArea = area;
			}
		}
	}
}

bool ndScene::ValidateContactCache(ndContact* const contact, const dVector& timestep) const
{
	dAssert(contact && (contact->GetAsContact()));

	ndBodyKinematic* const body0 = contact->GetBody0();
	ndBodyKinematic* const body1 = contact->GetBody1();

	dVector positStep(timestep * (body0->m_residualVeloc - body1->m_residualVeloc));
	positStep = ((positStep.DotProduct(positStep)) > m_velocTol) & positStep;
	contact->m_positAcc += positStep;
	
	dVector positError2(contact->m_positAcc.DotProduct(contact->m_positAcc));
	dVector positSign(dVector::m_negOne & (positError2 < m_linearContactError2));
	if (positSign.GetSignMask())
	{
		dVector rotationStep(timestep * (body0->m_residualOmega - body1->m_residualOmega));
		rotationStep = ((rotationStep.DotProduct(rotationStep)) > m_velocTol) & rotationStep;
		contact->m_rotationAcc = contact->m_rotationAcc * dQuaternion(rotationStep.m_x, rotationStep.m_y, rotationStep.m_z, dFloat32(1.0f));
	
		dVector angle(contact->m_rotationAcc & dVector::m_triplexMask);
		dVector rotatError2(angle.DotProduct(angle));
		dVector rotationSign(dVector::m_negOne & (rotatError2 < m_linearContactError2));
		if (rotationSign.GetSignMask())
		{
			return true;
		}
	}
	return false;
}

void ndScene::CalculateJointContacts(dInt32 threadIndex, ndContact* const contact)
{
	//DG_TRACKTIME();
	ndBodyKinematic* const body0 = contact->GetBody0();
	ndBodyKinematic* const body1 = contact->GetBody1();
	
	dAssert(body0->GetScene() == this);
	dAssert(body1->GetScene() == this);
	//if (!(body0->m_collideWithLinkedBodies & body1->m_collideWithLinkedBodies)) 
	//{
	//	dAssert(0);
	////	if (world->AreBodyConnectedByJoints(body0, body1)) {
	////		return;
	////	}
	//}

	dAssert(m_contactNotifyCallback);
	bool processContacts = m_contactNotifyCallback->OnAabbOverlap(contact, m_timestep);
	if (processContacts)
	{
		dAssert(!body0->GetAsBodyTriggerVolume());
		dAssert(!body0->GetCollisionShape().GetShape()->GetAsShapeNull());
		dAssert(!body1->GetCollisionShape().GetShape()->GetAsShapeNull());
			
		ndContactPoint contactBuffer[D_MAX_CONTATCS];
		ndContactSolver contactSolver(contact, m_contactNotifyCallback, m_timestep);
		contactSolver.m_separatingVector = contact->m_separatingVector;
		contactSolver.m_contactBuffer = contactBuffer;
		contactSolver.m_intersectionTestOnly = body0->m_contactTestOnly | body1->m_contactTestOnly;
		
		dInt32 count = contactSolver.CalculateContactsDiscrete ();
		if (count)
		{
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
			if (contact->m_isIntersetionTestOnly)
			{
				ndBodyTriggerVolume* const trigger = body1->GetAsBodyTriggerVolume();
				if (trigger)
				{
					body1->GetAsBodyTriggerVolume()->OnTriggerExit(body0, m_timestep);
				}
				contact->m_isIntersetionTestOnly = 0;
			}
			contact->m_maxDOF = 0;
		}
	}
}

void ndScene::ProcessContacts(dInt32 threadIndex, dInt32 contactCount, ndContactSolver* const contactSolver)
{
	ndContact* const contact = contactSolver->m_contact;
	contact->m_positAcc = dVector::m_zero;
	contact->m_rotationAcc = dQuaternion();

	ndBodyKinematic* const body0 = contact->m_body0;
	ndBodyKinematic* const body1 = contact->m_body1;
	dAssert(body0);
	dAssert(body1);
	dAssert(body0 != body1);

	contact->m_material = m_contactNotifyCallback->GetMaterial(contact, body0->GetCollisionShape(), body1->GetCollisionShape());
	const ndContactPoint* const contactArray = contactSolver->m_contactBuffer;
	
	dInt32 count = 0;
	dVector cachePosition[D_MAX_CONTATCS];
	ndContactPointList::dNode* nodes[D_MAX_CONTATCS];
	ndContactPointList& contactPointList = contact->m_contacPointsList;
	for (ndContactPointList::dNode* contactNode = contactPointList.GetFirst(); contactNode; contactNode = contactNode->GetNext()) 
	{
		nodes[count] = contactNode;
		cachePosition[count] = contactNode->GetInfo().m_point;
		count++;
	}
	
	const dVector& v0 = body0->m_veloc;
	const dVector& w0 = body0->m_omega;
	const dVector& com0 = body0->m_globalCentreOfMass;
	
	const dVector& v1 = body1->m_veloc;
	const dVector& w1 = body1->m_omega;
	const dVector& com1 = body1->m_globalCentreOfMass;

	dVector controlDir0(dVector::m_zero);
	dVector controlDir1(dVector::m_zero);
	dVector controlNormal(contactArray[0].m_normal);
	dVector vel0(v0 + w0.CrossProduct(contactArray[0].m_point - com0));
	dVector vel1(v1 + w1.CrossProduct(contactArray[0].m_point - com1));
	dVector vRel(vel1 - vel0);
	dAssert(controlNormal.m_w == dFloat32(0.0f));
	dVector tangDir(vRel - controlNormal * vRel.DotProduct(controlNormal));
	dAssert(tangDir.m_w == dFloat32(0.0f));
	dFloat32 diff = tangDir.DotProduct(tangDir).GetScalar();
	
	dInt32 staticMotion = 0;
	if (diff <= dFloat32(1.0e-2f)) 
	{
		staticMotion = 1;
		if (dAbs(controlNormal.m_z) > dFloat32(0.577f)) 
		{
			tangDir = dVector(-controlNormal.m_y, controlNormal.m_z, dFloat32(0.0f), dFloat32(0.0f));
		}
		else 
		{
			tangDir = dVector(-controlNormal.m_y, controlNormal.m_x, dFloat32(0.0f), dFloat32(0.0f));
		}
		controlDir0 = controlNormal.CrossProduct(tangDir);
		dAssert(controlDir0.m_w == dFloat32(0.0f));
		dAssert(controlDir0.DotProduct(controlDir0).GetScalar() > dFloat32(1.0e-8f));
		controlDir0 = controlDir0.Normalize();
		controlDir1 = controlNormal.CrossProduct(controlDir0);
		dAssert(dAbs(controlNormal.DotProduct(controlDir0.CrossProduct(controlDir1)).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	}
	
	dFloat32 maxImpulse = dFloat32(-1.0f);
	for (dInt32 i = 0; i < contactCount; i++) 
	{
		dInt32 index = -1;
		dFloat32 min = dFloat32(1.0e20f);
		ndContactPointList::dNode* contactNode = nullptr;
		for (dInt32 j = 0; j < count; j++) 
		{
			dVector v(cachePosition[j] - contactArray[i].m_point);
			dAssert(v.m_w == dFloat32(0.0f));
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
			//dScopeSpinLock lock(m_contactLock);
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
		contactPoint->m_material = contact->m_material;
	
		if (staticMotion) 
		{
			if (contactPoint->m_normal.DotProduct(controlNormal).GetScalar() > dFloat32(0.9995f)) 
			{
				contactPoint->m_dir0 = controlDir0;
				contactPoint->m_dir1 = controlDir1;
			}
			else 
			{
				if (dAbs(contactPoint->m_normal.m_z) > dFloat32(0.577f))
				{
					tangDir = dVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_z, dFloat32(0.0f), dFloat32(0.0f));
				}
				else 
				{
					tangDir = dVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_x, dFloat32(0.0f), dFloat32(0.0f));
				}
				contactPoint->m_dir0 = contactPoint->m_normal.CrossProduct(tangDir);
				dAssert(contactPoint->m_dir0.m_w == dFloat32(0.0f));
				dAssert(contactPoint->m_dir0.DotProduct(contactPoint->m_dir0).GetScalar() > dFloat32(1.0e-8f));
				contactPoint->m_dir0 = contactPoint->m_dir0.Normalize();
				contactPoint->m_dir1 = contactPoint->m_normal.CrossProduct(contactPoint->m_dir0);
				dAssert(dAbs(contactPoint->m_normal.DotProduct(contactPoint->m_dir0.CrossProduct(contactPoint->m_dir1)).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
			}
		}
		else 
		{
			dVector veloc0(v0 + w0.CrossProduct(contactPoint->m_point - com0));
			dVector veloc1(v1 + w1.CrossProduct(contactPoint->m_point - com1));
			dVector relReloc(veloc1 - veloc0);
	
			dAssert(contactPoint->m_normal.m_w == dFloat32(0.0f));
			dFloat32 impulse = relReloc.DotProduct(contactPoint->m_normal).GetScalar();
			if (dAbs(impulse) > maxImpulse) 
			{
				maxImpulse = dAbs(impulse);
			}
	
			dVector tangentDir(relReloc - contactPoint->m_normal.Scale(impulse));
			dAssert(tangentDir.m_w == dFloat32(0.0f));
			diff = tangentDir.DotProduct(tangentDir).GetScalar();
			if (diff > dFloat32(1.0e-2f)) 
			{
				dAssert(tangentDir.m_w == dFloat32(0.0f));
				contactPoint->m_dir0 = tangentDir.Normalize();
			}
			else 
			{
				if (dAbs(contactPoint->m_normal.m_z) > dFloat32(0.577f)) 
				{
					tangentDir = dVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_z, dFloat32(0.0f), dFloat32(0.0f));
				}
				else 
				{
					tangentDir = dVector(-contactPoint->m_normal.m_y, contactPoint->m_normal.m_x, dFloat32(0.0f), dFloat32(0.0f));
				}
				contactPoint->m_dir0 = contactPoint->m_normal.CrossProduct(tangentDir);
				dAssert(contactPoint->m_dir0.m_w == dFloat32(0.0f));
				dAssert(contactPoint->m_dir0.DotProduct(contactPoint->m_dir0).GetScalar() > dFloat32(1.0e-8f));
				contactPoint->m_dir0 = contactPoint->m_dir0.Normalize();
			}
			contactPoint->m_dir1 = contactPoint->m_normal.CrossProduct(contactPoint->m_dir0);
			dAssert(dAbs(contactPoint->m_normal.DotProduct(contactPoint->m_dir0.CrossProduct(contactPoint->m_dir1)).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
		}
		dAssert(contactPoint->m_dir0.m_w == dFloat32(0.0f));
		dAssert(contactPoint->m_dir0.m_w == dFloat32(0.0f));
		dAssert(contactPoint->m_normal.m_w == dFloat32(0.0f));
	}
	
	if (count) 
	{
		//dScopeSpinLock lock(m_contactLock);
		for (dInt32 i = 0; i < count; i++) 
		{
			contactPointList.Remove(nodes[i]);
		}
	}
	
	contact->m_maxDOF = dUnsigned32(3 * contactPointList.GetCount());
	m_contactNotifyCallback->OnContactCallback(threadIndex, contact, m_timestep);
}

void ndScene::SubmitPairs(ndSceneNode* const leafNode, ndSceneNode* const node)
{
	ndBodyKinematic* const body0 = leafNode->GetBody() ? leafNode->GetBody() : nullptr;
	const dVector boxP0(body0 ? body0->m_minAabb : leafNode->m_minBox);
	const dVector boxP1(body0 ? body0->m_maxAabb : leafNode->m_maxBox);
	const bool test0 = body0 ? (body0->m_invMass.m_w != dFloat32(0.0f)) : true;

	dInt32 stack = 1;
	ndSceneNode* pool[D_SCENE_MAX_STACK_DEPTH];
	pool[0] = node;

	while (stack) 
	{
		stack--;
		ndSceneNode* const rootNode = pool[stack];
		if (dOverlapTest(rootNode->m_minBox, rootNode->m_maxBox, boxP0, boxP1)) 
		{
			if (rootNode->GetAsSceneBodyNode()) 
			{
				dAssert(!rootNode->GetRight());
				dAssert(!rootNode->GetLeft());
				ndBodyKinematic* const body1 = rootNode->GetBody() ? rootNode->GetBody() : nullptr;
				if (body0) 
				{
					if (body1) 
					{
						if (test0 || (body1->m_invMass.m_w != dFloat32(0.0f)))
						{
							const bool test = TestOverlaping(body0, body1);
							if (test)
							{
								AddPair(body0, body1);
							}
						}
					}
				}
			}
			else 
			{
				ndSceneTreeNode* const tmpNode = rootNode->GetAsSceneTreeNode();
				dAssert(tmpNode->m_left);
				dAssert(tmpNode->m_right);
		
				pool[stack] = tmpNode->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(pool) / sizeof(pool[0])));
		
				pool[stack] = tmpNode->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(pool) / sizeof(pool[0])));
			}
		}
	}
}

bool ndScene::TestOverlaping(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const
{
	//bool mass0 = (body0->m_invMass.m_w != dFloat32(0.0f));
	//bool mass1 = (body1->m_invMass.m_w != dFloat32(0.0f));
	//bool isDynamic0 = body0->IsRTTIType(ntBodyKinematic::m_dynamicBodyRTTI) != 0;
	//bool isDynamic1 = body1->IsRTTIType(ntBodyKinematic::m_dynamicBodyRTTI) != 0;
	//bool isKinematic0 = body0->IsRTTIType(ntBodyKinematic::m_kinematicBodyRTTI) != 0;
	//bool isKinematic1 = body1->IsRTTIType(ntBodyKinematic::m_kinematicBodyRTTI) != 0;
	//
	//dAssert(!body0->GetCollision()->IsType(dgCollision::dgCollisionnullptr_RTTI));
	//dAssert(!body1->GetCollision()->IsType(dgCollision::dgCollisionnullptr_RTTI));
	//
	//bool tier1 = true;
	//bool tier2 = !(body0->m_sleeping & body1->m_sleeping);
	//bool tier3 = (agreggate0 != agreggate1) || !agreggate0 || (agreggate0 && agreggate0->GetSelfCollision());
	//bool tier4 = isDynamic0 & mass0;
	//bool tier5 = isDynamic1 & mass1;
	//bool tier6 = isKinematic0 & mass1;
	//bool tier7 = isKinematic1 & mass0;
	//bool ret = tier1 & tier2  & tier3 & (tier4 | tier5 | tier6 | tier7);
	//
	//if (ret) 
	//{
	//	const dgCollisionInstance* const instance0 = body0->GetCollision();
	//	const dgCollisionInstance* const instance1 = body1->GetCollision();
	//
	//	if (body0->m_continueCollisionMode | body1->m_continueCollisionMode) {
	//		dVector velRelative(body1->GetVelocity() - body0->GetVelocity());
	//		if (velRelative.DotProduct(velRelative).GetScalar() > dFloat32(0.25f)) {
	//			dVector box0_p0;
	//			dVector box0_p1;
	//			dVector box1_p0;
	//			dVector box1_p1;
	//
	//			instance0->CalculateAabb(instance0->GetGlobalMatrix(), box0_p0, box0_p1);
	//			instance1->CalculateAabb(instance1->GetGlobalMatrix(), box1_p0, box1_p1);
	//
	//			dVector boxp0(box0_p0 - box1_p1);
	//			dVector boxp1(box0_p1 - box1_p0);
	//			dFastRayTest ray(dVector::m_zero, velRelative.Scale(timestep * dFloat32(4.0f)));
	//			dFloat32 distance = ray.BoxIntersect(boxp0, boxp1);
	//			ret = (distance < dFloat32(1.0f));
	//		}
	//		else {
	//			ret = dgOverlapTest(body0->m_minAabb, body0->m_maxAabb, body1->m_minAabb, body1->m_maxAabb) ? 1 : 0;
	//		}
	//	}
	//	else {
	//		ret = dgOverlapTest(body0->m_minAabb, body0->m_maxAabb, body1->m_minAabb, body1->m_maxAabb) ? 1 : 0;
	//	}
	//}
	//return ret;

	bool test = body0->GetCollisionShape().GetCollisionMode() & body1->GetCollisionShape().GetCollisionMode();
	return test && dOverlapTest(body0->m_minAabb, body0->m_maxAabb, body1->m_minAabb, body1->m_maxAabb) ? true : false;
}

ndJointBilateralConstraint* ndScene::FindBilateralJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const
{
	if (body0->m_jointList.GetCount() <= body1->m_jointList.GetCount())
	{
		for (ndJointList::dNode* node = body0->m_jointList.GetFirst(); node; node = node->GetNext())
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
		for (ndJointList::dNode* node = body1->m_jointList.GetFirst(); node; node = node->GetNext())
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

ndContact* ndScene::FindContactJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const
{
	if (body1->GetInvMass() != dFloat32(0.0f))
	{
		ndContact* const contact = body1->FindContact(body0);
		dAssert(!contact || (body0->FindContact(body1) == contact));
		return contact;
	}

	dAssert(body0->GetInvMass() != dFloat32(0.0f));
	ndContact* const contact = body0->FindContact(body1);
	dAssert(!contact || (body1->FindContact(body0) == contact));
	return contact;
}

void ndScene::AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1)
{
#ifdef D_USE_GLOBAL_LOCK
	dScopeSpinLock lock(m_contactLock);
#endif
	ndContact* const contact = FindContactJoint(body0, body1);
	if (!contact) 
	{
		const ndJointBilateralConstraint* const bilateral = FindBilateralJoint(body0, body1);

		const bool isCollidable = bilateral ? bilateral->IsCollidable() : true;
		if (isCollidable) 
		{
			m_contactList.CreateContact(body0, body1);
		}
	}
}

void ndScene::BuildBodyArray()
{
	D_TRACKTIME();
	class ndBuildBodyArray : public ndBaseJob
	{
		public:
		#define D_LOCAL_POOL_SIZE 256
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThreadId();

			ndBodyList::dNode* node = m_owner->m_bodyList.GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			dAtomic<dUnsigned32>& activeBodyCount = *((dAtomic<dUnsigned32>*)m_context);
			dArray<ndBodyKinematic*>& activeBodyArray = m_owner->m_activeBodyArray;

			dInt32 bodyCount = 0;
			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndBodyKinematic* const body = node->GetInfo();
				body->m_bodyIsConstrained = 0;
				if (body)
				{
					const ndShape* const shape = body->GetCollisionShape().GetShape()->GetAsShapeNull();
					if (!shape)
					{
						bool inScene = true;
						if (!body->GetSceneBodyNode())
						{
							dScopeSpinLock lock(m_owner->m_contactLock);
							inScene = m_owner->AddBody(body);
						}
						if (inScene)
						{
							m_buffer[bodyCount] = body;
							bodyCount++;
							if (bodyCount >= D_LOCAL_POOL_SIZE)
							{
								bodyCount = 0;
								const dInt32 baseIndex = activeBodyCount.fetch_add(D_LOCAL_POOL_SIZE);
								for (dInt32 i = 0; i < D_LOCAL_POOL_SIZE; i++)
								{
									const dInt32 index = baseIndex + i;
									m_buffer[i]->PrepareStep(index);
									activeBodyArray[index] = m_buffer[i];
								}
							}
						}
					}
				}
				for (dInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}

			if (bodyCount) 
			{
				const dInt32 baseIndex = activeBodyCount.fetch_add(bodyCount);
				for (dInt32 i = 0; i < bodyCount; i++)
				{
					const dInt32 index = baseIndex + i;
					m_buffer[i]->PrepareStep(index);
					activeBodyArray[index] = m_buffer[i];
				}
			}
		}

		ndBodyKinematic* m_buffer[D_LOCAL_POOL_SIZE];
	};

	dAtomic<dUnsigned32> activeBodyCount(0);
	m_activeBodyArray.SetCount(m_bodyList.GetCount());
	SubmitJobs<ndBuildBodyArray>(&activeBodyCount);
	m_activeBodyArray.SetCount(activeBodyCount);
}

void ndScene::CalculateContacts()
{
	D_TRACKTIME();
	class ndCalculateContacts : public ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndConstraintArray& activeContacts = m_owner->m_activeConstraintArray;
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 contactCount = activeContacts.GetCount();
			for (dInt32 i = threadIndex; i < contactCount; i += threadCount)
			{
				ndContact* const contact = activeContacts[i]->GetAsContact();
				dAssert(contact);
				m_owner->CalculateContacts(threadIndex, contact);
			}
		}
	};

	SubmitJobs<ndCalculateContacts>();
}

void ndScene::UpdateAabb()
{
	D_TRACKTIME();
	class ndUpdateAabbJob : public ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - 1;

			dUnsigned32* const sleepBodiesLane = (dUnsigned32*)m_context;
			for (dInt32 i = threadIndex; i < bodyCount; i += threadCount)
			{
				ndBodyKinematic* const body = bodyArray[i];
				if (!body->m_equilibrium)
				{
					m_owner->UpdateAabb(threadIndex, body);
					if (body->m_broaphaseEquilibrium)
					{
						sleepBodiesLane[threadIndex] += 1;
					}
				}
				else
				{
					dAssert(body->m_broaphaseEquilibrium);
					sleepBodiesLane[threadIndex] += 1;
				}
			}
		}
	};

	dUnsigned32 sleepBodiesLane[D_MAX_THREADS_COUNT];
	memset(sleepBodiesLane, 0, sizeof(sleepBodiesLane));
	SubmitJobs<ndUpdateAabbJob>(sleepBodiesLane);

	m_sleepBodies = 0;
	for (dInt32 i = 0; i < GetThreadCount(); i++)
	{
		m_sleepBodies += sleepBodiesLane[i];
	}
}

void ndScene::FindCollidingPairs(ndBodyKinematic* const body)
{
	body->m_broaphaseEquilibrium = 1;
	ndSceneBodyNode* const bodyNode = body->GetSceneBodyNode();
	for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		dAssert(!parent->GetAsSceneBodyNode());
		ndSceneNode* const sibling = parent->m_right;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling);
		}
	}
}

void ndScene::FindCollidingPairsForward(ndBodyKinematic* const body)
{
	body->m_broaphaseEquilibrium = 1;
	ndSceneBodyNode* const bodyNode = body->GetSceneBodyNode();
	for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		dAssert(!parent->GetAsSceneBodyNode());
		ndSceneNode* const sibling = parent->m_right;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling);
		}
	}
}

void ndScene::FindCollidingPairsBackward(ndBodyKinematic* const body)
{
	body->m_broaphaseEquilibrium = 1;
	ndSceneBodyNode* const bodyNode = body->GetSceneBodyNode();
	for (ndSceneNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent)
	{
		ndSceneTreeNode* const parent = ptr->m_parent->GetAsSceneTreeNode();
		dAssert(!parent->GetAsSceneBodyNode());
		ndSceneNode* const sibling = parent->m_left;
		if (sibling != ptr)
		{
			SubmitPairs(bodyNode, sibling);
		}
	}
}

void ndScene::FindCollidingPairs()
{
	D_TRACKTIME();
	class ndFindCollidindPairs : public ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - 1;
			for (dInt32 i = threadIndex; i < bodyCount; i += threadCount)
			{
				ndBodyKinematic* const body = bodyArray[i];
				m_owner->FindCollidingPairs(body);
			}
		}
	};

	class ndFindCollidindPairsForward : public ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - 1;
			for (dInt32 i = threadIndex; i < bodyCount; i += threadCount)
			{
				ndBodyKinematic* const body = bodyArray[i];
				if (!body->m_broaphaseEquilibrium)
				{
					dAssert(0);
//					m_owner->FindCollidingPairsForward(body);
				}
			}
		}
	};

	class ndFindCollidindPairsBackward : public ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - 1;
			for (dInt32 i = threadIndex; i < bodyCount; i += threadCount)
			{
				ndBodyKinematic* const body = bodyArray[i];
				if (!body->m_broaphaseEquilibrium)
				{
					dAssert(0);
					//m_owner->FindCollidingPairsBackward(body);
				}
			}
		}
	};

	m_fullScan = (3 * m_sleepBodies) < (2 * dUnsigned32(m_activeBodyArray.GetCount()));
m_fullScan = 1;
	if (m_fullScan)
	{
		SubmitJobs<ndFindCollidindPairs>();
	}
	else
	{
		SubmitJobs<ndFindCollidindPairsForward>();
		SubmitJobs<ndFindCollidindPairsBackward>();
	}
}

void ndScene::UpdateTransform()
{
	D_TRACKTIME();
	class ndTransformUpdate : public ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - 1;
			for (dInt32 i = threadIndex; i < bodyCount; i += threadCount)
			{
				ndBodyKinematic* const body = bodyArray[i];
				m_owner->UpdateTransformNotify(threadIndex, body);
			}
		}
	};

	SubmitJobs<ndTransformUpdate>();
}

void ndScene::CalculateContacts(dInt32 threadIndex, ndContact* const contact)
{
	const dUnsigned32 lru = m_lru - D_CONTACT_DELAY_FRAMES;

	dVector deltaTime(m_timestep);
	ndBodyKinematic* const body0 = contact->GetBody0();
	ndBodyKinematic* const body1 = contact->GetBody1();

	dAssert(!contact->m_isDead);
	if (!(body0->m_equilibrium & body1->m_equilibrium))
	{
		bool active = contact->IsActive();
		if (ValidateContactCache(contact, deltaTime))
		{
			contact->m_sceneLru = m_lru;
			contact->m_timeOfImpact = dFloat32(1.0e10f);
		}
		else
		{
			//contact->m_active = false;
			contact->SetActive(false);
			contact->m_positAcc = dVector::m_zero;
			contact->m_rotationAcc = dQuaternion();

			dFloat32 distance = contact->m_separationDistance;
			if (distance >= D_NARROW_PHASE_DIST)
			{
				const dVector veloc0(body0->GetVelocity());
				const dVector veloc1(body1->GetVelocity());
				
				const dVector veloc(veloc1 - veloc0);
				const dVector omega0(body0->GetOmega());
				const dVector omega1(body1->GetOmega());
				const ndShapeInstance* const collision0 = &body0->GetCollisionShape();
				const ndShapeInstance* const collision1 = &body1->GetCollisionShape();
				const dVector scale(dFloat32(1.0f), dFloat32(3.5f) * collision0->GetBoxMaxRadius(), dFloat32(3.5f) * collision1->GetBoxMaxRadius(), dFloat32(0.0f));
				const dVector velocMag2(veloc.DotProduct(veloc).GetScalar(), omega0.DotProduct(omega0).GetScalar(), omega1.DotProduct(omega1).GetScalar(), dFloat32(0.0f));
				const dVector velocMag(velocMag2.GetMax(dVector::m_epsilon).InvSqrt() * velocMag2 * scale);
				const dFloat32 speed = velocMag.AddHorizontal().GetScalar() + dFloat32(0.5f);
				
				distance -= speed * m_timestep;
				contact->m_separationDistance = distance;
			}
			if (distance < D_NARROW_PHASE_DIST)
			{
				CalculateJointContacts(threadIndex, contact);
				if (contact->m_maxDOF || contact->m_isIntersetionTestOnly)
				{
					contact->SetActive(true);
					contact->m_timeOfImpact = dFloat32(1.0e10f);
				}
				contact->m_sceneLru = m_lru;
			}
			else
			{
				const ndSceneBodyNode* const bodyNode0 = contact->GetBody0()->m_sceneBodyBodyNode;
				const ndSceneBodyNode* const bodyNode1 = contact->GetBody1()->m_sceneBodyBodyNode;
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
			dAssert(body0->GetInvMass() > dFloat32(0.0f));
			body0->m_equilibrium = false;
			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				body1->m_equilibrium = false;
			}
		}
	}
	else
	{
		contact->m_sceneLru = m_lru;
	}

	if (!contact->m_isDead && (body0->m_equilibrium & body1->m_equilibrium & !contact->IsActive()))
	{
		const ndSceneBodyNode* const bodyNode0 = contact->GetBody0()->m_sceneBodyBodyNode;
		const ndSceneBodyNode* const bodyNode1 = contact->GetBody1()->m_sceneBodyBodyNode;
		if (!dOverlapTest(bodyNode0->m_minBox, bodyNode0->m_maxBox, bodyNode1->m_minBox, bodyNode1->m_maxBox))
		{
			contact->m_isDead = 1;
		}
	}
}

void ndScene::BuildContactArray()
{
	D_TRACKTIME();
	dInt32 count = 0;
	m_activeConstraintArray.SetCount(m_contactList.GetCount());
	for (ndContactList::dNode* node = m_contactList.GetFirst(); node; node = node->GetNext())
	{
		ndContact* const contact = &node->GetInfo();
		dAssert(contact->m_isAttached);
		m_activeConstraintArray[count] = contact;
		count++;
		ndBodyTriggerVolume* const trigger = contact->GetBody1()->GetAsBodyTriggerVolume();
		if (trigger)
		{
			trigger->OnTrigger(contact->GetBody0(), m_timestep);
		}
	}
	m_activeConstraintArray.SetCount(count);
}

void ndScene::DeleteDeadContact()
{
	D_TRACKTIME();
	dInt32 activeCount = m_activeConstraintArray.GetCount();
	for (dInt32 i = activeCount - 1; i >= 0; i--)
	{
		ndContact* const contact = m_activeConstraintArray[i]->GetAsContact();
		dAssert(contact);
		if (contact->m_isDead)
		{
			activeCount--;
			m_contactList.DeleteContact(contact);
			m_activeConstraintArray[i] = m_activeConstraintArray[activeCount];
		}
		else if (!contact->IsActive() || !contact->m_maxDOF)
		{
			activeCount--;
			m_activeConstraintArray[i] = m_activeConstraintArray[activeCount];
		}
	}
	m_activeConstraintArray.SetCount(activeCount);
}

bool ndScene::ConvexCast(ndConvexCastNotify& callback, const ndSceneNode** stackPool, dFloat32* const stackDistance, dInt32 stack, const dFastRayTest& ray, const ndShapeInstance& convexShape, const dMatrix& globalOrigin, const dVector& globalDest) const
{
	dVector boxP0;
	dVector boxP1;

	dAssert(globalOrigin.TestOrthogonal());
	convexShape.CalculateAabb(globalOrigin, boxP0, boxP1);

	callback.m_contacts.SetCount(0);
	callback.m_param = dFloat32(1.2f);
	while (stack) 
	{
		stack--;
		dFloat32 dist = stackDistance[stack];
		
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
					ndConvexCastNotify castShape;
					ndBodyKinematic* const kinBody = body->GetAsBodyKinematic();
					if (castShape.CastShape(convexShape, globalOrigin, globalDest, kinBody->GetCollisionShape(), kinBody->GetMatrix()))
					{
						if ((castShape.m_param - callback.m_param) < dFloat32(-1.0e-3f))
						{
							callback.m_contacts.SetCount(0);
						}

						callback.m_param = castShape.m_param;
						if ((castShape.m_contacts.GetCount() + callback.m_contacts.GetCount()) >= callback.m_contacts.GetCapacity())
						{
							dAssert(0);
							//count = maxContacts - totalCount;
						}

						for (dInt32 i = castShape.m_contacts.GetCount() - 1; i >= 0; i--)
						{
							callback.m_contacts.PushBack(castShape.m_contacts[i]);
						}
						callback.m_normal = castShape.m_normal;
						callback.m_closestPoint0 = castShape.m_closestPoint0;
						callback.m_closestPoint1 = castShape.m_closestPoint1;
					}
					if (callback.m_param < dFloat32 (1.0e-8f)) 
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
					const dVector minBox(left->m_minBox - boxP1);
					const dVector maxBox(left->m_maxBox - boxP0);
					dFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
					if (dist1 < callback.m_param)
					{
						dInt32 j = stack;
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
					//const dgBroadPhaseNode* const right = node->m_right;
					const ndSceneNode* const right = me->GetRight();
					dAssert(right);
					const dVector minBox(right->m_minBox - boxP1);
					const dVector maxBox = right->m_maxBox - boxP0;
					dFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
					if (dist1 < callback.m_param)
					{
						dInt32 j = stack;
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

bool ndScene::RayCast(ndRayCastNotify& callback, const ndSceneNode** stackPool, dFloat32* const stackDistance, dInt32 stack, const dFastRayTest& ray) const
{
	bool state = false;
	while (stack)
	{
		stack--;
		dFloat32 dist = stackDistance[stack];
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
					if (callback.m_param < dFloat32(1.0e-8f))
					{
						break;
					}
				}
			}
			else
			{
				const ndSceneNode* const left = me->GetLeft();
				dAssert(left);
				dFloat32 dist1 = ray.BoxIntersect(left->m_minBox, left->m_maxBox);
				if (dist1 < callback.m_param)
				{
					dInt32 j = stack;
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
					dInt32 j = stack;
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

void ndScene::BodiesInAabb(ndBodiesInAabbNotify& callback, const ndSceneNode** stackPool, dInt32 stack) const
{
	callback.m_bodyArray.SetCount(0);
	while (stack)
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

void ndScene::AddNode(ndSceneNode* const newNode)
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

		if (parent->m_fitnessNode)
		{
			m_fitness.RemoveNode(parent);
		}
		delete parent;
	}
	else
	{
		delete node;
		m_rootNode = nullptr;
	}
}

bool ndScene::RayCast(ndRayCastNotify& callback, const dVector& globalOrigin, const dVector& globalDest) const
{
	const dVector p0(globalOrigin & dVector::m_triplexMask);
	const dVector p1(globalDest & dVector::m_triplexMask);

	bool state = false;
	callback.m_param = dFloat32(1.2f);
	if (m_rootNode)
	{
		const dVector segment(p1 - p0);
		dFloat32 dist2 = segment.DotProduct(segment).GetScalar();
		if (dist2 > dFloat32(1.0e-8f))
		{
			dFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
			const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

			dFastRayTest ray(p0, p1);

			stackPool[0] = m_rootNode;
			distance[0] = ray.BoxIntersect(m_rootNode->m_minBox, m_rootNode->m_maxBox);
			state = RayCast(callback, stackPool, distance, 1, ray);
		}
	}
	return state;
}

bool ndScene::ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const dMatrix& globalOrigin, const dVector& globalDest) const
{
	bool state = false;
	callback.m_param = dFloat32(1.2f);
	if (m_rootNode)
	{
		dVector boxP0;
		dVector boxP1;
		dAssert(globalOrigin.TestOrthogonal());
		convexShape.CalculateAabb(globalOrigin, boxP0, boxP1);

		dFloat32 distance[D_SCENE_MAX_STACK_DEPTH];
		const ndSceneNode* stackPool[D_SCENE_MAX_STACK_DEPTH];

		const dVector velocB(dVector::m_zero);
		const dVector velocA((globalDest - globalOrigin.m_posit) & dVector::m_triplexMask);
		const dVector minBox(m_rootNode->m_minBox - boxP1);
		const dVector maxBox(m_rootNode->m_maxBox - boxP0);
		dFastRayTest ray(dVector::m_zero, velocA);

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