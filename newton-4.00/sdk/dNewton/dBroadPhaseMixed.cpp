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

#include "dNewtonStdafx.h"
#include "dBody.h"
#include "dNewton.h"
#include "dBroadPhaseNode.h"
#include "dBroadPhaseMixed.h"

dBroadPhaseMixed::dBroadPhaseMixed(dNewton* const world)
	:dBroadPhase(world)
	,m_treeEntropy(dFloat32(0.0f))
	,m_fitness()
{
}

dBroadPhaseMixed::~dBroadPhaseMixed()
{
}

void dBroadPhaseMixed::AddNode(dBroadPhaseNode* const newNode)
{
	if (m_rootNode) 
	{
		dBroadPhaseTreeNode* const node = InsertNode(m_rootNode, newNode);
		node->m_fitnessNode = m_fitness.Append(node);
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

void dBroadPhaseMixed::RemoveNode(dBroadPhaseNode* const node)
{
	if (node->m_parent) 
	{
		if (!node->m_parent->GetAsBroadPhaseAggregate()) 
		{
			dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)node->m_parent;
			if (parent->m_parent) 
			{
				dBroadPhaseAggregate* const aggregate = parent->m_parent->GetAsBroadPhaseAggregate();
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
					dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*)parent->m_parent;
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
				dAssert(!node->m_parent->GetAsBroadPhaseBodyNode());
				dBroadPhaseTreeNode* const parent1 = node->m_parent->GetAsBroadPhaseTreeNode();
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
				dBody* const body = node->GetBody();
				if (body && body->GetBroadPhaseAggregate()) 
				{
					dAssert(0);
					//body->GetBroadPhaseAggregate()->m_fitnessList.Remove(parent->m_fitnessNode);
					//body->SetBroadPhaseAggregate(nullptr);
				}
				else 
				{
					m_fitness.Remove(parent->m_fitnessNode);
				}
			}
			delete parent;
		}
		else 
		{
			dAssert(0);
#if 0
			dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*)node->m_parent;
			dBody* const body = node->GetBody();
			dAssert(body);
			dAssert(body->GetBroadPhaseAggregate() == aggregate);
			body->SetBroadPhaseAggregate(nullptr);
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


void dBroadPhaseMixed::AddBody(dBody* const body)
{
	body->UpdateCollisionMatrix();
	dBroadPhaseBodyNode* const bodyNode = new dBroadPhaseBodyNode(body);
//	bodyNode->m_updateNode = m_updateList.Append(bodyNode);
	AddNode(bodyNode);
}

void dBroadPhaseMixed::RemoveBody(dBody* const body)
{
	dBroadPhaseBodyNode* const node = body->GetBroadPhaseNode();
	if (node)
	{
		//if (node->m_updateNode) 
		//{
		//	m_updateList.Remove(node->m_updateNode);
		//}
		RemoveNode(node);
	}
}

void dBroadPhaseMixed::BalanceBroadPhase()
{
	UpdateFitness(m_fitness, m_treeEntropy, &m_rootNode);
}