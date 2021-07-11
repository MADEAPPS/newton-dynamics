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
#include "ndShape.h"
#include "ndContact.h"
#include "ndShapePoint.h"
#include "ndShapeConvex.h"
#include "ndBodyKinematic.h"
#include "ndContactSolver.h"
#include "ndShapeStaticBVH.h"
#include "ndShapeStaticMesh.h"
#include "ndShapeHeightfield.h"
#include "ndShapeConvexPolygon.h"
#include "ndShapeCompoundConvex.h"

dVector ndContactSolver::m_pruneUpDir(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
dVector ndContactSolver::m_pruneSupportX(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));

dVector ndContactSolver::m_hullDirs[] =
{
	dVector(dFloat32(0.577350f), dFloat32(-0.577350f), dFloat32(0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(-0.577350f), dFloat32(-0.577350f), dFloat32(-0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(0.577350f), dFloat32(-0.577350f), dFloat32(-0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(-0.577350f), dFloat32(0.577350f), dFloat32(0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(0.577350f), dFloat32(0.577350f), dFloat32(-0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(-0.577350f), dFloat32(0.577350f), dFloat32(-0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(-0.577350f), dFloat32(-0.577350f), dFloat32(0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(0.577350f), dFloat32(0.577350f), dFloat32(0.577350f), dFloat32(0.0f)),
	dVector(dFloat32(0.000000f), dFloat32(-1.000000f), dFloat32(0.000000f), dFloat32(0.0f)),
	dVector(dFloat32(0.000000f), dFloat32(1.000000f), dFloat32(0.000000f), dFloat32(0.0f)),
	dVector(dFloat32(1.000000f), dFloat32(0.000000f), dFloat32(0.000000f), dFloat32(0.0f)),
	dVector(dFloat32(-1.000000f), dFloat32(0.000000f), dFloat32(0.000000f), dFloat32(0.0f)),
	dVector(dFloat32(0.000000f), dFloat32(0.000000f), dFloat32(1.000000f), dFloat32(0.0f)),
	dVector(dFloat32(0.000000f), dFloat32(0.000000f), dFloat32(-1.000000f), dFloat32(0.0f)),
};

dInt32 ndContactSolver::m_rayCastSimplex[4][4] =
{
	{ 0, 1, 2, 3 },
	{ 0, 2, 3, 1 },
	{ 2, 1, 3, 0 },
	{ 1, 0, 3, 2 },
};

ndContactSolver::ndContactSolver(ndShapeInstance* const instance, ndContactNotify* const notification, dFloat32 timestep)
	:dDownHeap<ndMinkFace*, dFloat32>(m_heapBuffer, sizeof (m_heapBuffer))
	,m_instance0(*instance, (ndShape*)instance->GetShape())
	,m_instance1(*instance, (ndShape*)instance->GetShape())
	,m_separatingVector(ndContact::m_initialSeparatingVector)
	,m_contact(nullptr)
	,m_freeFace(nullptr)
	,m_notification(notification)
	,m_contactBuffer(nullptr)
	,m_timestep(timestep)
	,m_skinThickness(dFloat32(0.0f))
	,m_separationDistance(dFloat32(0.0f))
	,m_maxCount(D_MAX_CONTATCS)
	,m_vertexIndex(0)
	,m_pruneContacts(1)
	,m_intersectionTestOnly(0)
{
}

ndContactSolver::ndContactSolver(ndContact* const contact, ndContactNotify* const notification, dFloat32 timestep)
	:dDownHeap<ndMinkFace*, dFloat32>(m_heapBuffer, sizeof(m_heapBuffer))
	,m_instance0(contact->GetBody0()->GetCollisionShape(), contact->GetBody0()->GetCollisionShape().GetShape())
	,m_instance1(contact->GetBody1()->GetCollisionShape(), contact->GetBody1()->GetCollisionShape().GetShape())
	,m_closestPoint0(dVector::m_zero)
	,m_closestPoint1(dVector::m_zero)
	,m_separatingVector(ndContact::m_initialSeparatingVector)
	,m_contact(contact)
	,m_freeFace(nullptr)
	,m_notification(notification)
	,m_contactBuffer(nullptr)
	,m_timestep(timestep)
	,m_skinThickness(dFloat32(0.0f))
	,m_separationDistance(dFloat32(0.0f))
	,m_maxCount(D_MAX_CONTATCS)
	,m_vertexIndex(0)
	,m_pruneContacts(1)
	,m_intersectionTestOnly(0)
{
}

ndContactSolver::ndContactSolver(const ndContactSolver& src, const ndShapeInstance& instance0, const ndShapeInstance& instance1)
	:dDownHeap<ndMinkFace*, dFloat32>(m_heapBuffer, sizeof(m_heapBuffer))
	,m_instance0(instance0, ((ndShapeInstance*)&instance0)->GetShape())
	,m_instance1(instance1, ((ndShapeInstance*)&instance1)->GetShape())
	,m_closestPoint0(dVector::m_zero)
	,m_closestPoint1(dVector::m_zero)
	,m_separatingVector(src.m_separatingVector)
	,m_contact(src.m_contact)
	,m_freeFace(nullptr)
	,m_notification(src.m_notification)
	,m_contactBuffer(src.m_contactBuffer)
	,m_timestep(src.m_timestep)
	,m_skinThickness(src.m_skinThickness)
	,m_separationDistance(src.m_separationDistance)
	,m_maxCount(D_MAX_CONTATCS)
	,m_vertexIndex(0)
	,m_pruneContacts(src.m_pruneContacts)
	,m_intersectionTestOnly(src.m_intersectionTestOnly)
{
}

dInt32 ndContactSolver::ConvexToCompoundContactsDiscrete()
{
	const ndShapeCompoundConvex::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const otherBody = contactJoint->GetBody0();
	ndBodyKinematic* const compoundBody = contactJoint->GetBody1();
	ndShapeInstance* const otherInstance = &otherBody->GetCollisionShape();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();

	const dMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	dMatrix matrix(otherInstance->GetGlobalMatrix() * compoundMatrix.Inverse());

	dVector size;
	dVector origin;
	otherInstance->CalculateObb(origin, size);
	ndShapeCompoundConvex::ndOOBBTestData data(matrix, origin, size);

	ndShapeCompoundConvex* const compoundShape = m_instance1.GetShape()->GetAsShapeCompoundConvex();
	dAssert(compoundShape);

	dInt32 stack = 1;
	dInt32 contactCount = 0;
	stackPool[0] = compoundShape->m_root;

	//const dFloat32 timestep = m_scene->GetTimestep();
	dFloat32 closestDist = dFloat32(1.0e10f);

	while (stack)
	{
		stack--;
		const ndShapeCompoundConvex::ndNodeBase* const node = stackPool[stack];
		dAssert(node);

		if (node->BoxTest(data))
		{
			if (node->m_type == ndShapeCompoundConvex::m_leaf)
			{
				ndShapeInstance* const subShape = node->GetShape();
				if (subShape->GetCollisionMode())
				{
					bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, otherInstance, subShape);
					if (processContacts)
					{
						ndShapeInstance childInstance(*subShape, subShape->GetShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;

						ndContactSolver contactSolver(*this, m_instance0, childInstance);
						contactSolver.m_pruneContacts = 0;
						contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
						contactSolver.m_contactBuffer += contactCount;

						dInt32 count = contactSolver.ConvexContactsDiscrete();

						closestDist = dMin(closestDist, contactSolver.m_separationDistance);
						if (!m_intersectionTestOnly)
						{
							for (dInt32 i = 0; i < count; i++)
							{
								contacts[contactCount].m_shapeInstance0 = subShape;
							}
							contactCount += count;
							if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
							{
								contactCount = PruneContacts(contactCount, 16);
							}
						}
						else if (count == -1)
						{
							dAssert(0);
							contactCount = -1;
							break;
						}
					}
				}
			}
			else
			{
				dAssert(node->m_type == ndShapeCompoundConvex::m_node);
				stackPool[stack] = node->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(ndShapeCompoundConvex::ndNodeBase*)));

				stackPool[stack] = node->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(ndShapeCompoundConvex::ndNodeBase*)));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	contactJoint->m_separationDistance = closestDist;
	return contactCount;
}

dInt32 ndContactSolver::CompoundToConvexContactsDiscrete()
{
	const ndShapeCompoundConvex::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];
	
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const otherBody = contactJoint->GetBody1();
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndShapeInstance* const otherInstance = &otherBody->GetCollisionShape();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();

	const dMatrix& myMatrix = compoundInstance->GetGlobalMatrix();
	dMatrix matrix(otherInstance->GetGlobalMatrix() * myMatrix.Inverse());
	
	dVector size;
	dVector origin;
	otherInstance->CalculateObb(origin, size);
	ndShapeCompoundConvex::ndOOBBTestData data(matrix, origin, size);

	ndShapeCompoundConvex* const compoundShape = m_instance0.GetShape()->GetAsShapeCompoundConvex();
	dAssert(compoundShape);
	
	dInt32 stack = 1;
	dInt32 contactCount = 0;
	stackPool[0] = compoundShape->m_root;

	//dAssert((contacts != nullptr) ^ proxy.m_intersectionTestOnly);
	//const dFloat32 timestep = m_scene->GetTimestep();

	dFloat32 closestDist = dFloat32(1.0e10f);
	while (stack) 
	{
		stack--;
		const ndShapeCompoundConvex::ndNodeBase* const me = stackPool[stack];
		dAssert(me);
	
		if (me->BoxTest(data)) 
		{
			if (me->m_type == ndShapeCompoundConvex::m_leaf)
			{
				ndShapeInstance* const subShape = me->GetShape();
				if (subShape->GetCollisionMode()) 
				{
					bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, otherInstance);
					if (processContacts) 
					{
						ndShapeInstance childInstance(*subShape, subShape->GetShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * myMatrix;

						ndContactSolver contactSolver(*this, childInstance, m_instance1);
						contactSolver.m_pruneContacts = 0;
						contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
						contactSolver.m_contactBuffer += contactCount;
				
						dInt32 count = contactSolver.ConvexContactsDiscrete();
						closestDist = dMin(closestDist, contactSolver.m_separationDistance);
						if (!m_intersectionTestOnly) 
						{
							for (dInt32 i = 0; i < count; i++) 
							{
								contacts[contactCount].m_shapeInstance0 = subShape;
							}
							contactCount += count;
							if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3))) 
							{
								contactCount = PruneContacts(contactCount, 16);
							}
						}
						else if (count == -1) 
						{
							dAssert(0);
							contactCount = -1;
							break;
						}
					}
				}
			}
			else 
			{
				dAssert(me->m_type == ndShapeCompoundConvex::m_node);
				stackPool[stack] = me->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(ndShapeCompoundConvex::ndNodeBase*)));
	
				stackPool[stack] = me->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(ndShapeCompoundConvex::ndNodeBase*)));
			}
		}
	}
	
	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	contactJoint->m_separationDistance = closestDist;
	return contactCount;
}

dInt32 ndContactSolver::CompoundToCompoundContactsDiscrete()
{
	class ndStackEntry
	{
		public:
		const ndShapeCompoundConvex::ndNodeBase* m_node0;
		const ndShapeCompoundConvex::ndNodeBase* m_node1;
	};
	ndStackEntry stackPool[2 * D_COMPOUND_STACK_DEPTH];
	
	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody0 = contactJoint->GetBody0();
	ndBodyKinematic* const compoundBody1 = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance0 = &compoundBody0->GetCollisionShape();
	ndShapeInstance* const compoundInstance1 = &compoundBody1->GetCollisionShape();

	const dMatrix& matrix0 = compoundInstance0->GetGlobalMatrix();
	const dMatrix& matrix1 = compoundInstance1->GetGlobalMatrix();
	ndShapeCompoundConvex::ndOOBBTestData data(matrix1 * matrix0.Inverse());

	ndShapeCompoundConvex* const compoundShape0 = m_instance0.GetShape()->GetAsShapeCompoundConvex();
	ndShapeCompoundConvex* const compoundShape1 = m_instance1.GetShape()->GetAsShapeCompoundConvex();
	dAssert(compoundShape0);
	dAssert(compoundShape1);
	
	dInt32 stack = 1;
	dInt32 contactCount = 0;

	stackPool[0].m_node0 = compoundShape0->m_root;
	stackPool[0].m_node1 = compoundShape1->m_root;
	dFloat32 closestDist = dFloat32(1.0e10f);
	
	while (stack)
	{
		stack--;
		const ndShapeCompoundConvex::ndNodeBase* const node0 = stackPool[stack].m_node0;
		const ndShapeCompoundConvex::ndNodeBase* const node1 = stackPool[stack].m_node1;

		dAssert(node0 && node1);
	
		if (node0->BoxTest(data, node1))
		{
			if ((node0->m_type == ndShapeCompoundConvex::m_leaf) && (node1->m_type == ndShapeCompoundConvex::m_leaf)) 
			{
				ndShapeInstance* const subShape0 = node0->GetShape();
				ndShapeInstance* const subShape1 = node1->GetShape();
				
				if (subShape0->GetCollisionMode() & subShape1->GetCollisionMode())
				{
					bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape0, subShape1);
					if (processContacts)
					{
						ndShapeInstance childInstance0(*subShape0, subShape0->GetShape());
						ndShapeInstance childInstance1(*subShape1, subShape1->GetShape());
						childInstance0.m_globalMatrix = childInstance0.GetLocalMatrix() * matrix0;
						childInstance1.m_globalMatrix = childInstance1.GetLocalMatrix() * matrix1;
	
						ndContactSolver contactSolver(*this, childInstance0, childInstance1);
						contactSolver.m_pruneContacts = 0;
						contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
						contactSolver.m_contactBuffer += contactCount;
	
						dInt32 count = contactSolver.ConvexContactsDiscrete();
	
						closestDist = dMin(closestDist, contactSolver.m_separationDistance);
						if (!m_intersectionTestOnly)
						{
							for (dInt32 i = 0; i < count; i++)
							{
								contacts[contactCount].m_shapeInstance0 = subShape0;
								contacts[contactCount].m_shapeInstance1 = subShape1;
							}
							contactCount += count;
							if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
							{
								contactCount = PruneContacts(contactCount, 16);
							}
						}
						else if (count == -1)
						{
							dAssert(0);
							contactCount = -1;
							break;
						}
					}
				}
			}
			else if (node0->m_type == ndShapeCompoundConvex::m_leaf)
			{
				dAssert(node1->m_type == ndShapeCompoundConvex::m_node);

				stackPool[stack].m_node0 = node0;
				stackPool[stack].m_node1 = node1->m_left;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));

				stackPool[stack].m_node0 = node0;
				stackPool[stack].m_node1 = node1->m_right;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));
			}
			else if (node1->m_type == ndShapeCompoundConvex::m_leaf)
			{
				dAssert(node0->m_type == ndShapeCompoundConvex::m_node);

				stackPool[stack].m_node0 = node0->m_left;
				stackPool[stack].m_node1 = node1;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));

				stackPool[stack].m_node0 = node0->m_right;
				stackPool[stack].m_node1 = node1;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));
			}
			else
			{
				dAssert(node0->m_type == ndShapeCompoundConvex::m_node);
				dAssert(node1->m_type == ndShapeCompoundConvex::m_node);

				stackPool[stack].m_node0 = node0->m_left;
				stackPool[stack].m_node1 = node1->m_left;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));

				stackPool[stack].m_node0 = node0->m_left;
				stackPool[stack].m_node1 = node1->m_right;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));

				stackPool[stack].m_node0 = node0->m_right;
				stackPool[stack].m_node1 = node1->m_left;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));

				stackPool[stack].m_node0 = node0->m_right;
				stackPool[stack].m_node1 = node1->m_right;
				stack++;
				dAssert(stack < (sizeof (stackPool)/sizeof (stackPool[0])));
			}
		}
	}
	
	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	contactJoint->m_separationDistance = closestDist;
	return contactCount;
}

dInt32 ndContactSolver::CompoundToShapeStaticBvhContactsDiscrete()
{
	class ndStackEntry
	{
		public:
		const ndShapeCompoundConvex::ndNodeBase* m_compoundNode;
		const dAabbPolygonSoup::dNode* m_collisionTreeNode;
		dInt32 m_treeNodeIsLeaf;
	};
	ndStackEntry stackPool[2 * D_COMPOUND_STACK_DEPTH];

	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndBodyKinematic* const bvhTreeBody = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();
	ndShapeInstance* const bvhTreeInstance = &bvhTreeBody->GetCollisionShape();
	ndShapeStaticBVH* const bvhTreeCollision = m_instance1.GetShape()->GetAsShapeStaticBVH();
	ndShapeCompoundConvex* const compoundShape = m_instance0.GetShape()->GetAsShapeCompoundConvex();

	dAssert(bvhTreeCollision);
	dAssert(compoundShape);

	const dMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	const dMatrix& treeMatrix = bvhTreeInstance->GetGlobalMatrix();
	ndShapeCompoundConvex::ndOOBBTestData data(treeMatrix * compoundMatrix.Inverse());

	dInt32 stack = 1;
	dInt32 contactCount = 0;

	stackPool[0].m_treeNodeIsLeaf = 0;
	stackPool[0].m_compoundNode = compoundShape->m_root;
	stackPool[0].m_collisionTreeNode = bvhTreeCollision->GetRootNode();
	
	dFloat32 closestDist = dFloat32(1.0e10f);
	const dVector treeScale (bvhTreeInstance->GetScale());
	
	ndShapeCompoundConvex::ndNodeBase nodeProxi;
	while (stack)
	{
		stack--;
		const ndShapeCompoundConvex::ndNodeBase* const compoundNode = stackPool[stack].m_compoundNode;
		const dAabbPolygonSoup::dNode* const collisionTreeNode = stackPool[stack].m_collisionTreeNode;
		const dInt32 treeNodeIsLeaf = stackPool[stack].m_treeNodeIsLeaf;
	
		dAssert(compoundNode && collisionTreeNode);

		dVector p0;
		dVector p1;

		bvhTreeCollision->GetNodeAABB(collisionTreeNode, p0, p1);
		nodeProxi.m_p0 = p0 * treeScale;
		nodeProxi.m_p1 = p1 * treeScale;

		p0 = nodeProxi.m_p0 * dVector::m_half;
		p1 = nodeProxi.m_p1 * dVector::m_half;
		nodeProxi.m_size = p1 - p0;
		nodeProxi.m_origin = p1 + p0;
		nodeProxi.m_area = nodeProxi.m_size.ShiftTripleRight().DotProduct(nodeProxi.m_size).GetScalar();
	
		if (compoundNode->BoxTest(data, &nodeProxi))
		{
			if ((compoundNode->m_type == ndShapeCompoundConvex::m_leaf) && treeNodeIsLeaf)
			{
				ndShapeInstance* const subShape = compoundNode->GetShape();
				if (subShape->GetCollisionMode())
				{
					bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, bvhTreeInstance);
					if (processContacts)
					{
						ndShapeInstance childInstance(*subShape, subShape->GetShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;
				
						ndContactSolver contactSolver(*this, childInstance, m_instance1);
						contactSolver.m_pruneContacts = 0;
						contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
						contactSolver.m_contactBuffer += contactCount;
				
						dInt32 count = contactSolver.ConvexToSaticStaticBvhContactsNodeDescrete(collisionTreeNode);
						closestDist = dMin(closestDist, contactSolver.m_separationDistance);
						if (!m_intersectionTestOnly)
						{
							for (dInt32 i = 0; i < count; i++)
							{
								contacts[contactCount].m_shapeInstance0 = subShape;
							}
							contactCount += count;
							if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
							{
								contactCount = PruneContacts(contactCount, 16);
							}
						}
						else if (count == -1)
						{
							dAssert(0);
							contactCount = -1;
							break;
						}
					}
				}
			}
			else if (compoundNode->m_type == ndShapeCompoundConvex::m_leaf)
			{
				dAssert(!treeNodeIsLeaf);
				
				const dAabbPolygonSoup::dNode* const backNode = bvhTreeCollision->GetBackNode(collisionTreeNode);
				const dAabbPolygonSoup::dNode* const frontNode = bvhTreeCollision->GetFrontNode(collisionTreeNode);
				
				if (backNode && frontNode)
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
					
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				}
				else if (backNode && !frontNode)
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));

					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				}
				else if (!backNode && frontNode)
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));

					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				}
				else
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				}
			}
			else if (treeNodeIsLeaf)
			{
				dAssert(compoundNode->m_type == ndShapeCompoundConvex::m_node);
				
				stackPool[stack].m_compoundNode = compoundNode->m_left;
				stackPool[stack].m_collisionTreeNode = collisionTreeNode;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stack++;
				dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				
				stackPool[stack].m_compoundNode = compoundNode->m_right;
				stackPool[stack].m_collisionTreeNode = collisionTreeNode;
				stackPool[stack].m_treeNodeIsLeaf = 1;
				stack++;
				dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
			}
			else if (nodeProxi.m_area > compoundNode->m_area)
			{
				dAssert(compoundNode->m_type == ndShapeCompoundConvex::m_node);
				dAssert(!treeNodeIsLeaf);
				
				const dAabbPolygonSoup::dNode* const backNode = bvhTreeCollision->GetBackNode(collisionTreeNode);
				const dAabbPolygonSoup::dNode* const frontNode = bvhTreeCollision->GetFrontNode(collisionTreeNode);
				
				if (backNode && frontNode)
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
					
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				}
				else if (backNode && !frontNode)
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = backNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_compoundNode = compoundNode->m_left;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

					stackPool[stack].m_compoundNode = compoundNode->m_right;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				}
				else if (!backNode && frontNode)
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = frontNode;
					stackPool[stack].m_treeNodeIsLeaf = 0;
					stack++;

					stackPool[stack].m_compoundNode = compoundNode->m_left;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;

					stackPool[stack].m_compoundNode = compoundNode->m_right;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
				}
				else
				{
					stackPool[stack].m_compoundNode = compoundNode;
					stackPool[stack].m_collisionTreeNode = collisionTreeNode;
					stackPool[stack].m_treeNodeIsLeaf = 1;
					stack++;
					dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
				}
			}
			else
			{
				dAssert(!treeNodeIsLeaf);
				dAssert(compoundNode->m_left);
				dAssert(compoundNode->m_right);
				stackPool[stack].m_compoundNode = compoundNode->m_left;
				stackPool[stack].m_collisionTreeNode = collisionTreeNode;
				stackPool[stack].m_treeNodeIsLeaf = 0;
				stack++;
				dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));

				stackPool[stack].m_compoundNode = compoundNode->m_right;
				stackPool[stack].m_collisionTreeNode = collisionTreeNode;
				stackPool[stack].m_treeNodeIsLeaf = 0;
				stack++;
				dAssert(stack < (sizeof(stackPool) / sizeof(stackPool[0])));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	contactJoint->m_separationDistance = closestDist;
	return contactCount;
}

dInt32 ndContactSolver::CompoundToStaticHeighfieldContactsDiscrete()
{
	const ndShapeCompoundConvex::ndNodeBase* stackPool[D_COMPOUND_STACK_DEPTH];

	ndContact* const contactJoint = m_contact;
	ndContactPoint* const contacts = m_contactBuffer;
	ndBodyKinematic* const compoundBody = contactJoint->GetBody0();
	ndBodyKinematic* const heightfieldBody = contactJoint->GetBody1();
	ndShapeInstance* const compoundInstance = &compoundBody->GetCollisionShape();
	ndShapeInstance* const heightfieldInstance = &heightfieldBody->GetCollisionShape();

	ndShapeHeightfield* const heightfieldShape = heightfieldInstance->GetShape()->GetAsShapeHeightfield();
	ndShapeCompoundConvex* const compoundShape = compoundInstance->GetShape()->GetAsShapeCompoundConvex();
	
	dInt32 stack = 1;
	dInt32 contactCount = 0;
	dFloat32 closestDist = dFloat32(1.0e10f);
	stackPool[0] = compoundShape->m_root;
	
	ndShapeCompoundConvex::ndNodeBase nodeProxi;
	nodeProxi.m_left = nullptr;
	nodeProxi.m_right = nullptr;
	const dVector heighFieldScale(heightfieldInstance->GetScale());
	const dVector heighFieldInvScale(heightfieldInstance->GetInvScale());

	const dMatrix& compoundMatrix = compoundInstance->GetGlobalMatrix();
	dMatrix matrix(heightfieldInstance->GetGlobalMatrix() * compoundMatrix.Inverse());

	ndShapeCompoundConvex::ndOOBBTestData data(matrix);

	while (stack) 
	{
		stack--;
		const ndShapeCompoundConvex::ndNodeBase* const node = stackPool[stack];
		dAssert(node);

		dVector origin(heighFieldInvScale * data.m_matrix.UntransformVector(node->m_origin));
		dVector size(heighFieldInvScale * data.m_absMatrix.UnrotateVector(node->m_size));
		dVector p0(origin - size);
		dVector p1(origin + size);
		heightfieldShape->GetLocalAABB(p0, p1, nodeProxi.m_p0, nodeProxi.m_p1);
		nodeProxi.m_p0 *= heighFieldScale;
		nodeProxi.m_p1 *= heighFieldScale;
		nodeProxi.m_size = dVector::m_half * (nodeProxi.m_p1 - nodeProxi.m_p0);
		nodeProxi.m_origin = dVector::m_half * (nodeProxi.m_p1 + nodeProxi.m_p0);
		if (node->BoxTest(data, &nodeProxi)) 
		{
			if (node->m_type == ndShapeCompoundConvex::m_leaf) 
			{
				ndShapeInstance* const subShape = node->GetShape();
				if (subShape->GetCollisionMode())
				{
					bool processContacts = m_notification->OnCompoundSubShapeOverlap(contactJoint, m_timestep, subShape, heightfieldInstance);
					if (processContacts)
					{
						ndShapeInstance childInstance(*subShape, subShape->GetShape());
						childInstance.m_globalMatrix = childInstance.GetLocalMatrix() * compoundMatrix;

						ndContactSolver contactSolver(*this, childInstance, m_instance1);
						contactSolver.m_pruneContacts = 0;
						contactSolver.m_maxCount = D_MAX_CONTATCS - contactCount;
						contactSolver.m_contactBuffer += contactCount;

						dInt32 count = contactSolver.ConvexContactsDiscrete();

						closestDist = dMin(closestDist, contactSolver.m_separationDistance);
						if (!m_intersectionTestOnly)
						{
							for (dInt32 i = 0; i < count; i++)
							{
								contacts[contactCount].m_shapeInstance0 = subShape;
							}
							contactCount += count;
							if (contactCount > (D_MAX_CONTATCS - 2 * (D_CONSTRAINT_MAX_ROWS / 3)))
							{
								contactCount = PruneContacts(contactCount, 16);
							}
						}
						else if (count == -1)
						{
							dAssert(0);
							contactCount = -1;
							break;
						}
					}
				}
			}
			else 
			{
				dAssert(node->m_type == ndShapeCompoundConvex::m_node);
				stackPool[stack] = node->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(ndShapeCompoundConvex::ndNodeBase*)));
	
				stackPool[stack] = node->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(ndShapeCompoundConvex::ndNodeBase*)));
			}
		}
	}

	if (m_pruneContacts && (contactCount > 1))
	{
		contactCount = PruneContacts(contactCount, 16);
	}
	contactJoint->m_separationDistance = closestDist;
	return contactCount;
}

dInt32 ndContactSolver::ConvexContactsDiscrete()
{
	const dVector origin0(m_instance0.m_globalMatrix.m_posit);
	const dVector origin1(m_instance1.m_globalMatrix.m_posit);
	m_instance0.m_globalMatrix.m_posit = dVector::m_wOne;
	m_instance1.m_globalMatrix.m_posit -= (origin0 & dVector::m_triplexMask);

	// handle rare case of two shapes located exactly at the same origin
	const dVector error(m_instance1.m_globalMatrix.m_posit - m_instance0.m_globalMatrix.m_posit);
	if (error.DotProduct(error).GetScalar() < dFloat32(1.0e-6f))
	{
		m_instance1.m_globalMatrix.m_posit.m_y += dFloat32(1.0e-3f);
	}

	dInt32 count = 0;
	if (m_instance1.GetShape()->GetAsShapeConvex())
	{
		dAssert(m_instance0.GetShape()->GetAsShapeConvex());
		count = ConvexToConvexContactsDiscrete();
	}
	else
	{
		if (m_instance1.GetShape()->GetAsShapeStaticMeshShape())
		{
			count = ConvexToStaticMeshContactsDiscrete();
		}
		else
		{
			dAssert(0);
			count = 0;
		}
	}

	if (m_pruneContacts && (count > 1))
	{
		count = PruneContacts(count, 16);
	}

	const dVector offset (origin0 & dVector::m_triplexMask);
	m_closestPoint0 += offset;
	m_closestPoint1 += offset;

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (dInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_point += offset;
		}
	}

	m_instance0.m_globalMatrix.m_posit = origin0;
	m_instance1.m_globalMatrix.m_posit = origin1;
	return count;
}

dInt32 ndContactSolver::ConvexContactsContinue()
{
	const dVector origin0(m_instance0.m_globalMatrix.m_posit);
	const dVector origin1(m_instance1.m_globalMatrix.m_posit);
	m_instance0.m_globalMatrix.m_posit = dVector::m_wOne;
	m_instance1.m_globalMatrix.m_posit -= (origin0 & dVector::m_triplexMask);

	// handle rare case of two shapes located exactly at the same origin
	const dVector error(m_instance1.m_globalMatrix.m_posit - m_instance0.m_globalMatrix.m_posit);
	if (error.DotProduct(error).GetScalar() < dFloat32(1.0e-6f))
	{
		m_instance1.m_globalMatrix.m_posit.m_y += dFloat32(1.0e-3f);
	}

	dInt32 count = 0;
	if (m_instance1.GetShape()->GetAsShapeConvex())
	{
		dAssert(m_instance0.GetShape()->GetAsShapeConvex());
		count = ConvexToConvexContactsContinue();
	}
	else
	{
		ndShapeStaticMesh* const meshShape = m_instance1.GetShape()->GetAsShapeStaticMeshShape();
		if (meshShape)
		{
			count = ConvexToStaticMeshContactsContinue();
		}
		else
		{
			dAssert(0);
			count = 0;
		}
	}

	if (m_pruneContacts && (count > 1))
	{
		count = PruneContacts(count, 16);
	}

	const dVector offset(origin0 & dVector::m_triplexMask);
	m_closestPoint0 += offset;
	m_closestPoint1 += offset;

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (dInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_point += offset;
		}
	}

	m_instance0.m_globalMatrix.m_posit = origin0;
	m_instance1.m_globalMatrix.m_posit = origin1;
	return count;
}

dInt32 ndContactSolver::ConvexToSaticStaticBvhContactsNodeDescrete(const dAabbPolygonSoup::dNode* const node)
{
	dVector origin0(m_instance0.m_globalMatrix.m_posit);
	dVector origin1(m_instance1.m_globalMatrix.m_posit);
	m_instance0.m_globalMatrix.m_posit = dVector::m_wOne;
	m_instance1.m_globalMatrix.m_posit -= (origin0 & dVector::m_triplexMask);

	dAssert (m_instance1.GetShape()->GetAsShapeStaticBVH());

	ndShapeStaticBVH* const polysoup = m_instance1.GetShape()->GetAsShapeStaticBVH();
	dAssert(polysoup);

	ndPolygonMeshDesc data(*this, false);

	data.m_me = polysoup;
	data.m_vertex = polysoup->GetLocalVertexPool();
	data.m_vertexStrideInBytes = polysoup->GetStrideInBytes();
	data.m_faceCount = 0;
	data.m_globalIndexCount = 0;
	data.m_faceIndexCount = data.m_meshData.m_globalFaceIndexCount;
	data.m_faceIndexStart = data.m_meshData.m_globalFaceIndexStart;
	data.m_faceVertexIndex = data.m_globalFaceVertexIndex;
	data.m_hitDistance = data.m_meshData.m_globalHitDistance;
	polysoup->ForThisSector(node, data, data.m_boxDistanceTravelInMeshSpace, data.m_maxT, polysoup->GetPolygon, &data);

	dInt32 count = 0;
	if (data.m_faceCount)
	{
		count = CalculatePolySoupToHullContactsDescrete(data);
	}

	ndBodyKinematic* const body0 = m_contact->GetBody0();
	ndBodyKinematic* const body1 = m_contact->GetBody1();
	ndShapeInstance* const instance0 = &body0->GetCollisionShape();
	ndShapeInstance* const instance1 = &body1->GetCollisionShape();

	//if (!m_intersectionTestOnly)
	//{
	//	ndContactPoint* const contactOut = m_contactBuffer;
	//	for (dInt32 i = count - 1; i >= 0; i--)
	//	{
	//		contactOut[i].m_body0 = body0;
	//		contactOut[i].m_body1 = body1;
	//		contactOut[i].m_shapeInstance0 = instance0;
	//		contactOut[i].m_shapeInstance1 = instance1;
	//	}
	//}

	dVector offset = (origin0 & dVector::m_triplexMask);
	m_closestPoint0 += offset;
	m_closestPoint1 += offset;

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (dInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_point += offset;
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
		}
	}

	m_instance0.m_globalMatrix.m_posit = origin0;
	m_instance1.m_globalMatrix.m_posit = origin1;
	return count;
}

dInt32 ndContactSolver::CompoundContactsDiscrete()
{
	if (!m_instance1.GetShape()->GetAsShapeCompoundConvex())
	{
		dAssert(m_instance0.GetShape()->GetAsShapeCompoundConvex());
		if (m_instance1.GetShape()->GetAsShapeConvex())
		{
			return CompoundToConvexContactsDiscrete();
		}
		else if (m_instance1.GetShape()->GetAsShapeStaticBVH())
		{
			return CompoundToShapeStaticBvhContactsDiscrete();
		}
		else if (m_instance1.GetShape()->GetAsShapeHeightfield())
		{
			return CompoundToStaticHeighfieldContactsDiscrete();
		}
		else
		{
			dAssert(0);
		}
	}
	else if (!m_instance0.GetShape()->GetAsShapeCompoundConvex())
	{
		dAssert(m_instance1.GetShape()->GetAsShapeCompoundConvex());
		if (m_instance0.GetShape()->GetAsShapeConvex())
		{
			return ConvexToCompoundContactsDiscrete();
		}
		else
		{
			dAssert(0);
		}
	}
	else
	{
		dAssert(m_instance0.GetShape()->GetAsShapeCompoundConvex() && m_instance1.GetShape()->GetAsShapeCompoundConvex());
		return CompoundToCompoundContactsDiscrete();
	}
	return 0;
}

dInt32 ndContactSolver::CalculateContactsDiscrete ()
{
	dInt32 count = 0;
	if (m_instance0.GetShape()->GetAsShapeCompoundConvex() || m_instance1.GetShape()->GetAsShapeCompoundConvex())
	{
		count = CompoundContactsDiscrete();
	}
	else if (m_instance0.GetShape()->GetAsShapeConvex())
	{
		count = ConvexContactsDiscrete();
	}
	else
	{
		//dTrace(("!!!!!Fix compound contact\n"));
		dAssert(0);
	}

	m_contact->m_timeOfImpact = m_timestep;
	m_contact->m_separatingVector = m_separatingVector;
	m_contact->m_separationDistance = m_separationDistance;

	return count;
}

dInt32 ndContactSolver::CalculateContactsContinue()
{
	dInt32 count = 0;
	if (m_instance0.GetShape()->GetAsShapeCompoundConvex() || m_instance1.GetShape()->GetAsShapeCompoundConvex())
	{
		dAssert(0);
		//count = CompoundContacts();
	}
	else if (m_instance0.GetShape()->GetAsShapeConvex())
	{
		count = ConvexContactsContinue();
	}
	else
	{
		//dTrace(("!!!!!Fix compound contact\n"));
		dAssert(0);
	}

	m_contact->m_timeOfImpact = m_timestep;
	m_contact->m_separatingVector = m_separatingVector;
	m_contact->m_separationDistance = m_separationDistance;

	return count;
}

D_INLINE void ndContactSolver::TranslateSimplex(const dVector& step)
{
	m_instance1.m_globalMatrix.m_posit -= step;
	for (dInt32 i = 0; i < m_vertexIndex; i++) 
	{
		m_hullSum[i] -= step;
		m_hullDiff[i] += step;
	}
}

dInt32 ndContactSolver::ConvexToConvexContactsContinue()
{
	dAssert(m_instance0.GetConvexVertexCount() && m_instance1.GetConvexVertexCount());
	dAssert(m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(m_instance1.GetShape()->GetAsShapeConvex());
	dAssert(!m_instance0.GetShape()->GetAsShapeNull());
	dAssert(!m_instance1.GetShape()->GetAsShapeNull());

	const dVector savedPosition1(m_instance1.m_globalMatrix.m_posit);
	const ndBodyKinematic* const body0 = m_contact->m_body0;
	const ndBodyKinematic* const body1 = m_contact->m_body1;
	const dVector relVeloc(body0->GetVelocity() - body1->GetVelocity());
	dVector closestPoint1;

	dInt32 iter = 0;
	dInt32 count = 0;
	dFloat32 tacc = dFloat32(0.0f);
	dFloat32 timestep = m_timestep;
	m_contact->m_separationDistance = dFloat32(1.0e10f);
	do 
	{
		bool state = CalculateClosestPoints();
		if (!state) 
		{
			break;
		}
		dAssert(m_separatingVector.m_w == dFloat32(0.0f));
		dFloat32 den = m_separatingVector.DotProduct(relVeloc).GetScalar();
		if (den <= dFloat32(1.0e-6f)) 
		{
			// bodies are residing from each other, even if they are touching 
			// they are not considered to be colliding because the motion will 
			// move them apart get the closet point and the normal at contact point
			m_timestep = dFloat32(1.0e10f);
			m_separatingVector = m_separatingVector * dVector::m_negOne;
			break;
		}
		
		dFloat32 num = m_separatingVector.DotProduct(m_closestPoint1 - m_closestPoint0).GetScalar() - m_skinThickness;
		if ((num <= dFloat32(1.0e-5f)) && (tacc <= timestep)) 
		{
			// bodies collide at time tacc, but we do not set it yet
			dVector step(relVeloc.Scale(tacc));
			m_timestep = tacc;
			closestPoint1 = m_closestPoint1 + step;
			m_separatingVector = m_separatingVector * dVector::m_negOne;
			dFloat32 penetration = dMax(num * dFloat32(-1.0f) + D_PENETRATION_TOL, dFloat32(0.0f));
			m_contact->m_separationDistance = penetration;
			if (m_contactBuffer && !m_intersectionTestOnly) 
			{
				if (m_instance0.GetCollisionMode() & m_instance1.GetCollisionMode()) 
				{
					//m_normal = m_normal.Scale(dFloat32(-1.0f));
					//m_proxy->m_contactJoint->m_isActive = 1;
					count = CalculateContacts(m_closestPoint0, m_closestPoint1, m_separatingVector);
					if (count) 
					{
						count = dMin(m_maxCount, count);
						ndContactPoint* const contactOut = m_contactBuffer;
			
						for (int i = 0; i < count; i++) 
						{
							contactOut[i].m_point = m_hullDiff[i] + step;
							contactOut[i].m_normal = m_separatingVector;
							contactOut[i].m_penetration = penetration;
						}
					}
				}
			}
			break;
		}
		
		dAssert(den > dFloat32(0.0f));
		dFloat32 dt = num / den;
		if ((tacc + dt) >= timestep) 
		{
			// object do not collide on this timestep
			m_timestep = tacc + dt;
			closestPoint1 = m_closestPoint1;
			m_separatingVector = m_separatingVector * dVector::m_negOne;
			break;
		}
		
		tacc += dt;
		dVector step(relVeloc.Scale(dt));
		TranslateSimplex(step);
		
		iter++;
	} while (iter < D_SEPARATION_PLANES_ITERATIONS);

	m_closestPoint1 = closestPoint1;
	m_instance1.m_globalMatrix.m_posit = savedPosition1;
	return count;
}

dInt32 ndContactSolver::ConvexToConvexContactsDiscrete()
{
	dAssert(m_instance0.GetConvexVertexCount() && m_instance1.GetConvexVertexCount());
	dAssert(m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(m_instance1.GetShape()->GetAsShapeConvex());
	dAssert(!m_instance0.GetShape()->GetAsShapeNull());
	dAssert(!m_instance1.GetShape()->GetAsShapeNull());

	dInt32 count = 0;
	bool colliding = CalculateClosestPoints();
	dFloat32 penetration = m_separatingVector.DotProduct(m_closestPoint1 - m_closestPoint0).GetScalar() - m_skinThickness - D_PENETRATION_TOL;
	if (m_intersectionTestOnly)
	{
		m_separationDistance = penetration;
		count = (penetration <= dFloat32(0.0f)) ? 1 : 0;
	}
	else
	{
		if (colliding)
		{
			if (penetration <= dFloat32(1.0e-5f))
			{
				if (m_instance0.GetCollisionMode() & m_instance1.GetCollisionMode())
				{
					count = CalculateContacts(m_closestPoint0, m_closestPoint1, m_separatingVector * dVector::m_negOne);
				}
			}

			m_separationDistance = penetration;

			penetration = -penetration;
			count = dMin(m_maxCount, count);
			ndContactPoint* const contactOut = m_contactBuffer;

			ndBodyKinematic* const body0 = m_contact->GetBody0();
			ndBodyKinematic* const body1 = m_contact->GetBody1();
			ndShapeInstance* const instance0 = &body0->GetCollisionShape();
			ndShapeInstance* const instance1 = &body1->GetCollisionShape();

			dVector normal(m_separatingVector * dVector::m_negOne);
			for (dInt32 i = count - 1; i >= 0; i--)
			{
				contactOut[i].m_point = m_buffer[i];
				contactOut[i].m_normal = normal;
				contactOut[i].m_body0 = body0;
				contactOut[i].m_body1 = body1;
				contactOut[i].m_shapeInstance0 = instance0;
				contactOut[i].m_shapeInstance1 = instance1;
				contactOut[i].m_penetration = penetration;
			}
		}
	}
	return count;
}

dInt32 ndContactSolver::ConvexToStaticMeshContactsDiscrete()
{
	dAssert(m_instance0.GetConvexVertexCount());
	dAssert(!m_instance0.GetShape()->GetAsShapeNull());
	dAssert(m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(m_instance1.GetShape()->GetAsShapeStaticMeshShape());

	dInt32 count = 0;
	ndPolygonMeshDesc data(*this, false);
	ndShapeStaticMesh* const polysoup = m_instance1.GetShape()->GetAsShapeStaticMeshShape();
	polysoup->GetCollidingFaces(&data);

	if (data.m_faceCount) 
	{
		count = CalculatePolySoupToHullContactsDescrete(data);
	}

	ndBodyKinematic* const body0 = m_contact->GetBody0();
	ndBodyKinematic* const body1 = m_contact->GetBody1();
	ndShapeInstance* const instance0 = &body0->GetCollisionShape();
	ndShapeInstance* const instance1 = &body1->GetCollisionShape();

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (dInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
		}
	}

	return count;
}

dInt32 ndContactSolver::ConvexToStaticMeshContactsContinue()
{
	dAssert(m_instance0.GetConvexVertexCount());
	dAssert(!m_instance0.GetShape()->GetAsShapeNull());
	dAssert(m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(m_instance1.GetShape()->GetAsShapeStaticMeshShape());

	dInt32 count = 0;
	ndPolygonMeshDesc data(*this, true);

	//const dVector& hullVeloc = data.m_objBody->m_veloc;
	//const dVector& hullOmega = data.m_objBody->m_omega;

	dVector relVeloc(m_contact->m_body0->GetVelocity() - m_contact->m_body1->GetVelocity());
	dFloat32 baseLinearSpeed = dSqrt(relVeloc.DotProduct(relVeloc).GetScalar());
	if (baseLinearSpeed > dFloat32(1.0e-6f)) 
	{
		//const dFloat32 minRadius = m_instance0.GetBoxMinRadius();
		//const dFloat32 maxRadius = m_instance0.GetBoxMaxRadius();
		//dFloat32 maxAngularSpeed = dSqrt(hullOmega.DotProduct(hullOmega).GetScalar());
		//dFloat32 maxAngularSpeed = dFloat32 (0.0f);
		//dFloat32 angularSpeedBound = maxAngularSpeed * (maxRadius - minRadius);
		//dFloat32 upperBoundSpeed = baseLinearSpeed + dgSqrt(angularSpeedBound);
		dFloat32 upperBoundSpeed = baseLinearSpeed;
		dVector upperBoundVeloc(relVeloc.Scale(m_timestep * upperBoundSpeed / baseLinearSpeed));
		data.SetDistanceTravel(upperBoundVeloc);
	}

	ndShapeStaticMesh* const polysoup = m_instance1.GetShape()->GetAsShapeStaticMeshShape();
	polysoup->GetCollidingFaces(&data);

	if (data.m_faceCount)
	{
		count = CalculatePolySoupToHullContactsContinue(data);
	}

	ndBodyKinematic* const body0 = m_contact->GetBody0();
	ndBodyKinematic* const body1 = m_contact->GetBody1();
	ndShapeInstance* const instance0 = &body0->GetCollisionShape();
	ndShapeInstance* const instance1 = &body1->GetCollisionShape();

	if (!m_intersectionTestOnly)
	{
		ndContactPoint* const contactOut = m_contactBuffer;
		for (dInt32 i = count - 1; i >= 0; i--)
		{
			contactOut[i].m_body0 = body0;
			contactOut[i].m_body1 = body1;
			contactOut[i].m_shapeInstance0 = instance0;
			contactOut[i].m_shapeInstance1 = instance1;
		}
	}

	return count;
}

D_INLINE void ndContactSolver::SupportVertex(const dVector& dir0, dInt32 vertexIndex)
{
	dAssert(dir0.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir0.DotProduct(dir0).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	dVector dir1 (dir0.Scale(dFloat32 (-1.0f)));
	
	const dMatrix& matrix0 = m_instance0.m_globalMatrix;
	const dMatrix& matrix1 = m_instance1.m_globalMatrix;
	dVector p(matrix0.TransformVector(m_instance0.SupportVertexSpecial(matrix0.UnrotateVector (dir0), nullptr)) & dVector::m_triplexMask);
	dVector q(matrix1.TransformVector(m_instance1.SupportVertexSpecial(matrix1.UnrotateVector (dir1), nullptr)) & dVector::m_triplexMask);
	m_hullDiff[vertexIndex] = p - q;
	m_hullSum[vertexIndex] = p + q;
}

D_INLINE dBigVector ndContactSolver::ReduceLine(dInt32& indexOut)
{
	const dBigVector p0(m_hullDiff[0]);
	const dBigVector p1(m_hullDiff[1]);
	const dBigVector dp(p1 - p0);
	dBigVector v;

	const dFloat64 mag2 = dp.DotProduct(dp).GetScalar();
	dAssert(mag2 > dFloat64(0.0f));
	if (mag2 < dFloat32(1.0e-24f)) 
	{
		v = p0;
		indexOut = 1;
	}
	else 
	{
		const dFloat64 alpha0 = -p0.DotProduct(dp).GetScalar();
		if (alpha0 > mag2) 
		{
			v = p1;
			indexOut = 1;
			m_hullSum[0] = m_hullSum[1];
			m_hullDiff[0] = m_hullDiff[1];
		}
		else if (alpha0 < dFloat64(0.0f)) 
		{
			v = p0;
			indexOut = 1;
		}
		else 
		{
			v = p0 + dp.Scale(alpha0 / mag2);
		}
	}
	return v;
}

D_INLINE dBigVector ndContactSolver::ReduceTriangle(dInt32& indexOut)
{
	const dBigVector p0(m_hullDiff[0]);
	const dBigVector p1(m_hullDiff[1]);
	const dBigVector p2(m_hullDiff[2]);
	const dBigVector e10(p1 - p0);
	const dBigVector e20(p2 - p0);
	const dFloat64 a00 = e10.DotProduct(e10).GetScalar();
	const dFloat64 a11 = e20.DotProduct(e20).GetScalar();
	const dFloat64 a01 = e10.DotProduct(e20).GetScalar();

	const dFloat64 det = a00 * a11 - a01 * a01;
	dAssert(det >= dFloat32(0.0f));
	if (dAbs(det) > dFloat32(1.0e-24f)) 
	{
		const dFloat64 b0 = -e10.DotProduct(p0).GetScalar();
		const dFloat64 b1 = -e20.DotProduct(p0).GetScalar();

		const dFloat64 u2 = b1 * a00 - a01 * b0;
		const dFloat64 u1 = b0 * a11 - a01 * b1;

		if (u2 < dFloat32(0.0f)) 
		{
			// this looks funny but it is correct
		}
		else if (u1 < dFloat32(0.0f)) 
		{
			m_hullSum[1] = m_hullSum[2];
			m_hullDiff[1] = m_hullDiff[2];
		}
		else if ((u1 + u2) > det) 
		{
			m_hullSum[0] = m_hullSum[2];
			m_hullDiff[0] = m_hullDiff[2];
		}
		else 
		{
			return p0 + (e10.Scale(u1) + e20.Scale(u2)).Scale(dFloat64(1.0f) / det);
		}
		indexOut = 2;
		return ReduceLine(indexOut);
	}
	// this is a degenerated triangle. this should never happens
	dAssert(0);
	return dBigVector::m_zero;
}

D_INLINE dBigVector ndContactSolver::ReduceTetrahedrum(dInt32& indexOut)
{
	const dBigVector p0(m_hullDiff[0]);
	const dBigVector p1(m_hullDiff[1]);
	const dBigVector p2(m_hullDiff[2]);
	const dBigVector p3(m_hullDiff[3]);
	const dBigVector e10(p1 - p0);
	const dBigVector e20(p2 - p0);
	const dBigVector e30(p3 - p0);

	const dFloat64 d0 = sqrt(e10.DotProduct(e10).GetScalar());
	if (d0 > dFloat64(0.0f)) 
	{
		const dFloat64 invd0 = dFloat64(1.0f) / d0;
		const dFloat64 l10 = e20.DotProduct(e10).GetScalar() * invd0;
		const dFloat64 l20 = e30.DotProduct(e10).GetScalar() * invd0;
		const dFloat64 desc11 = e20.DotProduct(e20).GetScalar() - l10 * l10;
		if (desc11 > dFloat64(0.0f)) 
		{
			const dFloat64 d1 = sqrt(desc11);
			const dFloat64 invd1 = dFloat64(1.0f) / d1;
			const dFloat64 l21 = (e30.DotProduct(e20).GetScalar() - l20 * l10) * invd1;
			const dFloat64 desc22 = e30.DotProduct(e30).GetScalar() - l20 * l20 - l21 * l21;
			if (desc22 > dFloat64(0.0f)) 
			{
				const dFloat64 d2 = sqrt(desc22);
				const dFloat64 invd2 = dFloat64(1.0f) / d2;
				const dFloat64 b0 = -e10.DotProduct(p0).GetScalar();
				const dFloat64 b1 = -e20.DotProduct(p0).GetScalar();
				const dFloat64 b2 = -e30.DotProduct(p0).GetScalar();

				dFloat64 u1 = b0 * invd0;
				dFloat64 u2 = (b1 - l10 * u1) * invd1;
				dFloat64 u3 = (b2 - l20 * u1 - l21 * u2) * invd2 * invd2;
				u2 = (u2 - l21 * u3) * invd1;
				u1 = (u1 - l10 * u2 - l20 * u3) * invd0;
				if (u3 < dFloat64(0.0f)) 
				{
					// this looks funny but it is correct
				}
				else if (u2 < dFloat64(0.0f)) 
				{
					m_hullSum[2] = m_hullSum[3];
					m_hullDiff[2] = m_hullDiff[3];
				}
				else if (u1 < dFloat64(0.0f)) 
				{
					m_hullSum[1] = m_hullSum[3];
					m_hullDiff[1] = m_hullDiff[3];
				}
				else if (u1 + u2 + u3 > dFloat64(1.0f)) 
				{
					m_hullSum[0] = m_hullSum[3];
					m_hullDiff[0] = m_hullDiff[3];
				}
				else 
				{
					return dBigVector::m_zero;
				}
				indexOut = 3;
				return ReduceTriangle(indexOut);
			}
		}
	}
	// this is a degenerated tetra. this should never happens.
	// it seems this does happens about once per several millions calls, 
	// I will assume is acceptable. No fall back needed
	return dBigVector::m_zero;
}

dInt32 ndContactSolver::CalculateClosestSimplex()
{
	dBigVector v(dBigVector::m_zero);
	dInt32 index = 1;
	if (m_vertexIndex <= 0) 
	{
		SupportVertex (m_separatingVector, 0);
						
		v = m_hullDiff[0];
	} 
	else 
	{
		switch (m_vertexIndex) 
		{
			case 1:
			{
				v = m_hullDiff[0];
				break;
			}
	
			case 2:
			{
				v = ReduceLine (m_vertexIndex);
				break;
			}
	
			case 3:
			{
				v = ReduceTriangle (m_vertexIndex);
				break;
			}
	
			case 4:
			{
				v = ReduceTetrahedrum (m_vertexIndex);
				break;
			}
		}
		index = m_vertexIndex;
	}
	
	dVector bestNormal (m_separatingVector);
	
	dInt32 iter = 0;
	dInt32 cycling = 0;
	dFloat64 minDist = dFloat32 (1.0e20f);
	dFloat64 bestNormalDist = dFloat32 (1.0e20f);
	do 
	{
		dFloat64 dist = v.DotProduct(v).GetScalar();
		if (dist < dFloat32 (1.0e-9f)) 
		{
			// very deep penetration, resolve with generic Minkowski solver
			return -index; 
		}
	
		if (dist < minDist) 
		{
			minDist = dist;
			cycling = -1;
		}
		cycling ++;
		if (cycling > 4) 
		{
			return -index;
		}
	
		const dVector dir (v.Scale (-dRsqrt(dist)));
		dAssert (dir.m_w == dFloat32 (0.0f));
		SupportVertex (dir, index);
	
		const dBigVector w (m_hullDiff[index]);
		const dVector wv (w - v);
		dAssert (wv.m_w == dFloat32 (0.0f));
		const dFloat64 dist1 = dir.DotProduct(wv).GetScalar();
		if (dist1 < dFloat64 (1.0e-3f)) 
		{
			m_separatingVector = dir;
			return index;
		}
	
		if (dist1 < bestNormalDist) 
		{
			bestNormal = dir;
			bestNormalDist = dist1;
		}
	
		index ++;
		switch (index) 
		{
			case 2:
			{
				v = ReduceLine (index);
				break;
			}
	
			case 3:
			{
				v = ReduceTriangle (index);
				break;
			}
	
			case 4:
			{
				v = ReduceTetrahedrum (index);
				break;
			}
		}
	
		iter ++;
	} while (iter < D_CONNICS_CONTATS_ITERATIONS); 

	m_separatingVector = bestNormal;
	return (index < 4) ? index : -4;
}

D_INLINE void ndContactSolver::CalculateContactFromFeacture(dInt32 featureType)
{
	dVector d;
	dVector s;
	switch (featureType)
	{
		case 3:
		{
			const dBigVector p0(m_hullDiff[0]);
			const dBigVector p1(m_hullDiff[1]);
			const dBigVector p2(m_hullDiff[2]);
			const dBigVector e10(p1 - p0);
			const dBigVector e20(p2 - p0);
			const dFloat64 a00 = e10.DotProduct(e10).GetScalar();
			const dFloat64 a11 = e20.DotProduct(e20).GetScalar();
			const dFloat64 a01 = e10.DotProduct(e20).GetScalar();

			const dFloat64 det = a00 * a11 - a01 * a01;
			dAssert(det >= dFloat32(0.0f));
			if (dAbs(det) > dFloat32(1.0e-24f))
			{
				const dFloat64 b0 = -e10.DotProduct(p0).GetScalar();
				const dFloat64 b1 = -e20.DotProduct(p0).GetScalar();

				const dFloat64 u2 = b1 * a00 - a01 * b0;
				const dFloat64 u1 = b0 * a11 - a01 * b1;

				if (u2 < dFloat32(0.0f))
				{
					// this looks funny but it is correct
				}
				else if (u1 < dFloat32(0.0f))
				{
					m_hullSum[1] = m_hullSum[2];
					m_hullDiff[1] = m_hullDiff[2];
				}
				else if ((u1 + u2) > det)
				{
					m_hullSum[0] = m_hullSum[2];
					m_hullDiff[0] = m_hullDiff[2];
				}
				else
				{
					const dBigVector invDet(dFloat64(1.0f) / det);
					const dBigVector q0(m_hullSum[0]);
					const dBigVector q1(m_hullSum[1]);
					const dBigVector q2(m_hullSum[2]);
					const dBigVector q10(q1 - q0);
					const dBigVector q20(q2 - q0);

					d = dVector(p0 + (e10.Scale(u1) + e20.Scale(u2)) * invDet);
					s = dVector(q0 + (q10.Scale(u1) + q20.Scale(u2)) * invDet);
					break;
				}
			}
			else
			{
				// find extreme and reduce line
				dAssert ((a00 > dFloat32(0.0f)) || (a11 > dFloat32(0.0f)));
				const dBigVector dir ((a00 > a11) ? e10 : e20);
				dInt32 maxIndex = 0;
				dInt32 minIndex = 0;
				dFloat64 maxVal = dFloat32(-1.0e20f);
				dFloat64 minVal = dFloat32( 1.0e20f);
				for (dInt32 i = 0; i < 3; i++)
				{
					dFloat64 val = dir.DotProduct(m_hullDiff[i]).GetScalar();
					if (val > maxVal) 
					{
						maxIndex = i;
						maxVal = val;
					}
					if (val < minVal)
					{
						minIndex = i;
						minVal = val;
					}
				}
				const dBigVector mindiff(m_hullDiff[minIndex]);
				const dBigVector minSum(m_hullSum[minIndex]);
				const dBigVector maxdiff(m_hullDiff[maxIndex]);
				const dBigVector maxSum(m_hullSum[maxIndex]);

				m_hullDiff[0] = mindiff;
				m_hullSum[0] = minSum;
				m_hullDiff[1] = maxdiff;
				m_hullSum[1] = maxSum;
			}
		}

		case 2:
		{
			const dBigVector p0(m_hullDiff[0]);
			const dBigVector p1(m_hullDiff[1]);
			const dBigVector dp(p1 - p0);

			const dFloat64 mag2 = dp.DotProduct(dp).GetScalar();
			dAssert(mag2 > dFloat64(0.0f));
			if (mag2 < dFloat32(1.0e-24f))
			{
				s = m_hullSum[0];
				d = m_hullDiff[0];
			}
			else
			{
				const dFloat64 alpha0 = -p0.DotProduct(dp).GetScalar();
				if (alpha0 > mag2)
				{
					s = m_hullSum[1];
					d = m_hullDiff[1];
				}
				else if (alpha0 < dFloat64(0.0f))
				{
					s = m_hullSum[0];
					d = m_hullDiff[0];
				}
				else
				{
					const dBigVector scale(alpha0 / mag2);
					const dBigVector q0(m_hullSum[0]);
					const dBigVector q1(m_hullSum[1]);
					const dBigVector dq(q1 - q0);
					d = dVector(p0 + dp * scale);
					s = dVector(q0 + dq * scale);
				}
			}
			break;
		}

		case 1:
		default:
		{
			s = m_hullSum[0];
			d = m_hullDiff[0];
			break;
		}
	}

	m_closestPoint0 = dVector::m_half * (s + d);
	m_closestPoint1 = dVector::m_half * (s - d);
	dAssert(m_separatingVector.m_w == dFloat32(0.0f));
	dAssert(dAbs(m_separatingVector.DotProduct(m_separatingVector).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
}

dInt32 ndContactSolver::CalculateContacts(const dVector& point0, const dVector& point1, const dVector& normal)
{
	dAssert(m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(m_instance1.GetShape()->GetAsShapeConvex());

	dInt32 count = 0;
	dInt32 count1 = 0;
	const dInt32 baseCount = 16;
	dVector* const contactsOut = &m_buffer[0];
	dVector* const shape1 = &contactsOut[baseCount];
	dAssert(normal.m_w == dFloat32(0.0f));

	dVector origin((point0 + point1).Scale(dFloat32(0.5f)));
	const dMatrix& matrix1 = m_instance1.m_globalMatrix;
	dVector pointOnInstance1(matrix1.UntransformVector(origin));
	dVector normalOnInstance1(matrix1.UnrotateVector(normal));
	dFloat32 dist = (normal.DotProduct(point0 - point1)).GetScalar();
	//if (dist < dFloat32(0.0f)) 
	if (dist < (D_PENETRATION_TOL * dFloat32 (-0.5f)))
	{
		count1 = m_instance1.CalculatePlaneIntersection(normalOnInstance1, pointOnInstance1, shape1);
	}
	if (!count1) 
	{
		dVector step(normal.Scale(D_PENETRATION_TOL * dFloat32(2.0f)));
		dVector alternatePoint(point1);
		for (dInt32 i = 0; (i < 3) && !count1; i++) 
		{
			alternatePoint -= step;
			dVector alternatePointOnInstance1(matrix1.UntransformVector(alternatePoint));
			count1 = m_instance1.CalculatePlaneIntersection(normalOnInstance1, alternatePointOnInstance1, shape1);
		}
		step = matrix1.UnrotateVector(normal * ((alternatePoint - origin).DotProduct(normal)));
		for (dInt32 i = 0; i < count1; i++) 
		{
			shape1[i] -= step;
		}
	}

	if (count1) 
	{
		for (dInt32 i = 0; i < count1; i++) 
		{
			shape1[i] = matrix1.TransformVector(shape1[i]);
		}

		dInt32 count0 = 0;
		dVector* const shape0 = &contactsOut[baseCount + count1];

		const dMatrix& matrix0 = m_instance0.m_globalMatrix;
		dVector pointOnInstance0(matrix0.UntransformVector(origin));
		dVector normalOnInstance0(matrix0.UnrotateVector(normal.Scale(dFloat32(-1.0f))));
		//if (dist < dFloat32(0.0f)) 
		if (dist < (D_PENETRATION_TOL * dFloat32(-0.5f)))
		{
			count0 = m_instance0.CalculatePlaneIntersection(normalOnInstance0, pointOnInstance0, shape0);
		}
		if (!count0) 
		{
			dVector step(normal.Scale(D_PENETRATION_TOL * dFloat32(2.0f)));
			dVector alternatePoint(point0);
			for (dInt32 i = 0; (i < 3) && !count0; i++) 
			{
				alternatePoint += step;
				dVector alternatePointOnInstance0(matrix0.UntransformVector(alternatePoint));
				count0 = m_instance0.CalculatePlaneIntersection(normalOnInstance0, alternatePointOnInstance0, shape0);
			}
			//dTrace (("If this is a frequent event, this routine should return the translation distance as the contact point\n"))
			//dAssert(count0);
			step = matrix0.UnrotateVector(normal * ((alternatePoint - origin).DotProduct(normal)));
			for (dInt32 i = 0; i < count0; i++) 
			{
				shape0[i] -= step;
			}
		}

		if (count0) 
		{
			for (dInt32 i = 0; i < count0; i++) 
			{
				shape0[i] = matrix0.TransformVector(shape0[i]);
			}

			if (count1 == 1) 
			{
				count = 1;
				contactsOut[0] = shape1[0];
			}
			else if (count0 == 1) 
			{
				count = 1;
				contactsOut[0] = shape0[0];
			}
			else if ((count1 == 2) && (count0 == 2)) 
			{
				dVector p0(shape1[0]);
				dVector p1(shape1[1]);
				const dVector& q0 = shape0[0];
				const dVector& q1 = shape0[1];
				dVector p10(p1 - p0);
				dVector q10(q1 - q0);
				dAssert(p10.m_w == dFloat32(0.0f));
				dAssert(q10.m_w == dFloat32(0.0f));
				p10 = p10.Scale(dRsqrt(p10.DotProduct(p10).GetScalar() + dFloat32(1.0e-8f)));
				q10 = q10.Scale(dRsqrt(q10.DotProduct(q10).GetScalar() + dFloat32(1.0e-8f)));
				dFloat32 dot = q10.DotProduct(p10).GetScalar();
				if (dAbs(dot) > dFloat32(0.998f)) 
				{
					dFloat32 pl0 = p0.DotProduct(p10).GetScalar();
					dFloat32 pl1 = p1.DotProduct(p10).GetScalar();
					dFloat32 ql0 = q0.DotProduct(p10).GetScalar();
					dFloat32 ql1 = q1.DotProduct(p10).GetScalar();
					if (pl0 > pl1) 
					{
						dSwap(pl0, pl1);
						dSwap(p0, p1);
						p10 = p10.Scale(dFloat32(-1.0f));
					}
					if (ql0 > ql1) 
					{
						dSwap(ql0, ql1);
					}
					if (!((ql0 > pl1) && (ql1 < pl0))) 
					{
						dFloat32 clip0 = (ql0 > pl0) ? ql0 : pl0;
						dFloat32 clip1 = (ql1 < pl1) ? ql1 : pl1;

						count = 2;
						contactsOut[0] = p0 + p10.Scale(clip0 - pl0);
						contactsOut[1] = p0 + p10.Scale(clip1 - pl0);
					}
				}
				else 
				{
					count = 1;
					dVector c0;
					dVector c1;
					dRayToRayDistance(p0, p1, q0, q1, c0, c1);
					contactsOut[0] = (c0 + c1).Scale(dFloat32(0.5f));
				}
			}
			else 
			{
				dAssert((count1 >= 2) && (count0 >= 2));
				count = ConvexPolygonsIntersection(normal, count0, shape0, count1, shape1, contactsOut, baseCount);
			}
		}
	}

	//if (!count && m_ccdMode) 
	//{
	//	count = 1;
	//	contactsOut[0] = origin;
	//}

	return count;
}

dInt32 ndContactSolver::ConvexPolygonToLineIntersection(const dVector& normal, dInt32 count1, dVector* const shape1, dInt32 count2, dVector* const shape2, dVector* const contactOut, dVector* const mem) const
{
	dInt32 count = 0;
	dVector* output = mem;

	dAssert(count1 >= 3);
	dAssert(count2 <= 2);
	dAssert(normal.m_w == dFloat32(0.0f));

	dVector* ptr = nullptr;
	// face line intersection
	if (count2 == 2) 
	{
		ptr = (dVector*)&shape2[0];
		dInt32 i0 = count1 - 1;
		for (dInt32 i1 = 0; i1 < count1; i1++) 
		{
			dVector n(normal.CrossProduct(shape1[i1] - shape1[i0]));
			dAssert(n.m_w == dFloat32(0.0f));
			dAssert(n.DotProduct(n).GetScalar() > dFloat32(0.0f));
			dPlane plane(n, -n.DotProduct(shape1[i0]).GetScalar());

			dFloat32 test0 = plane.Evalue(ptr[0]);
			dFloat32 test1 = plane.Evalue(ptr[1]);
			if (test0 >= dFloat32(0.0f)) 
			{
				if (test1 >= dFloat32(0.0f)) 
				{
					output[count + 0] = ptr[0];
					output[count + 1] = ptr[1];
					count += 2;
				}
				else 
				{
					dVector dp(ptr[1] - ptr[0]);
					dAssert(dp.m_w == dFloat32(0.0f));
					dFloat32 den = plane.DotProduct(dp).GetScalar();
					if (dAbs(den) < dFloat32 (1.0e-10f)) 
					{
						den = dFloat32(1.0e-10f);
					}
					output[count + 0] = ptr[0];
					dAssert(dp.m_w == dFloat32(0.0f));
					output[count + 1] = ptr[0] - dp.Scale(test0 / den);
					count += 2;
				}
			}
			else if (test1 >= dFloat32(0.0f)) 
			{
				dVector dp(ptr[1] - ptr[0]);
				dAssert(dp.m_w == dFloat32(0.0f));
				dFloat32 den = plane.DotProduct(dp).GetScalar();
				if (dAbs(den) < dFloat32(1.0e-10f))
				{
					den = dFloat32(1.0e-10f);
				}
				dAssert(dp.m_w == dFloat32(0.0f));
				output[count] = ptr[0] - dp.Scale(test0 / den);
				count++;
				output[count] = ptr[1];
				count++;
			}
			else 
			{
				return 0;
			}


			count2 = count;
			ptr = output;
			output = &output[count];
			count = 0;
			i0 = i1;
			//dAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
		}
	}
	else if (count2 == 1) 
	{
		const dVector& p = shape2[0];
		dInt32 i0 = count1 - 1;
		for (dInt32 i1 = 0; i1 < count1; i1++) 
		{
			dVector n(normal.CrossProduct(shape1[i1] - shape1[i0]));
			dAssert(n.m_w == dFloat32(0.0f));
			dAssert(n.DotProduct(n).GetScalar() > dFloat32(0.0f));
			dPlane plane(n, -n.DotProduct(shape1[i0]).GetScalar());
			dFloat32 test0 = plane.Evalue(p);
			if (test0 < dFloat32(-1.e-3f)) 
			{
				return 0;
			}
			i0 = i1;
		}
		ptr = output;
		output[count] = p;
		count++;
	}
	else 
	{
		count2 = 0;
	}

	for (dInt32 i0 = 0; i0 < count2; i0++) 
	{
		contactOut[i0] = ptr[i0];
	}
	return count2;
}

dInt32 ndContactSolver::ConvexPolygonsIntersection(const dVector& normal, dInt32 count0, dVector* const shape0, dInt32 count1, dVector* const shape1, dVector* const contactOut, dInt32 maxContacts) const
{
	dInt32 count = 0;
	if (count1 <= 2) 
	{
		count = ConvexPolygonToLineIntersection(normal.Scale(dFloat32(-1.0f)), count0, shape0, count1, shape1, contactOut, &contactOut[count0 + count1 + maxContacts]);
	}
	else if (count0 <= 2) 
	{
		count = ConvexPolygonToLineIntersection(normal, count1, shape1, count0, shape0, contactOut, &contactOut[count0 + count1 + maxContacts]);
	}
	else 
	{
		dAssert(count0 >= 3);
		dAssert(count1 >= 3);

		dgPerimenterEdge subdivision[128];
		dAssert((2 * (count0 + count1)) < dInt32(sizeof(subdivision) / sizeof(subdivision[0])));

		for (dInt32 i0 = 1; i0 < count1; i0++) 
		{
			subdivision[i0].m_vertex = &shape1[i0];
			subdivision[i0].m_prev = &subdivision[i0 - 1];
			subdivision[i0].m_next = &subdivision[i0 + 1];
		}
		subdivision[0].m_vertex = &shape1[0];
		subdivision[0].m_prev = &subdivision[count1 - 1];
		subdivision[0].m_next = &subdivision[1];

		subdivision[count1 - 1].m_next = &subdivision[0];

		dgPerimenterEdge* edgeClipped[2];
		dVector* output = &contactOut[count0 + count1 + maxContacts];

		edgeClipped[0] = nullptr;
		edgeClipped[1] = nullptr;
		dInt32 j0 = 0;
		dInt32 edgeIndex = count1;
		dgPerimenterEdge* poly = &subdivision[0];
		for (dInt32 j1 = count0 - 1; j1 >= 0; j1--) 
		{
			dVector n(normal.CrossProduct(shape0[j1] - shape0[j0]));
			dAssert(n.m_w == 0.0f);
			dPlane plane(n, -n.DotProduct(shape0[j0]).GetScalar());
			j0 = j1;
			count = 0;
			dgPerimenterEdge* tmp = poly;
			dInt32 isInside = 0;
			dFloat32 test0 = plane.Evalue(*tmp->m_vertex);
			do 
			{
				dFloat32 test1 = plane.Evalue(*tmp->m_next->m_vertex);

				if (test0 >= dFloat32(0.0f)) 
				{
					isInside |= 1;
					if (test1 < dFloat32(0.0f)) 
					{
						const dVector& p0 = *tmp->m_vertex;
						const dVector& p1 = *tmp->m_next->m_vertex;
						dVector dp(p1 - p0);
						dAssert(dp.m_w == 0.0f);
						dFloat32 den = plane.DotProduct(dp).GetScalar();
						if (dAbs(den) < dFloat32(1.0e-24f)) 
						{
							den = (den >= dFloat32(0.0f)) ? dFloat32(1.0e-24f) : dFloat32(-1.0e-24f);
						}

						den = test0 / den;
						if (den >= dFloat32(0.0f)) 
						{
							den = dFloat32(0.0f);
						}
						else if (den <= -1.0f) 
						{
							den = dFloat32(-1.0f);
						}
						dAssert(dp.m_w == dFloat32(0.0f));
						output[0] = p0 - dp.Scale(den);
						edgeClipped[0] = tmp;
						count++;
					}
				}
				else if (test1 >= dFloat32(0.0f)) 
				{
					const dVector& p0 = *tmp->m_vertex;
					const dVector& p1 = *tmp->m_next->m_vertex;
					isInside |= 1;
					dVector dp(p1 - p0);
					dAssert(dp.m_w == 0.0f);
					dFloat32 den = plane.DotProduct(dp).GetScalar();
					if (dAbs(den) < dFloat32(1.0e-24f)) 
					{
						den = (den >= dFloat32(0.0f)) ? dFloat32(1.0e-24f) : dFloat32(-1.0e-24f);
					}
					den = test0 / den;
					if (den >= dFloat32(0.0f)) 
					{
						den = dFloat32(0.0f);
					}
					else if (den <= -1.0f) 
					{
						den = dFloat32(-1.0f);
					}
					dAssert(dp.m_w == dFloat32(0.0f));
					output[1] = p0 - dp.Scale(den);
					edgeClipped[1] = tmp;
					count++;
				}

				test0 = test1;
				tmp = tmp->m_next;
			} while (tmp != poly && (count < 2));

			if (!isInside) 
			{
				return 0;
			}

			if (count == 2) 
			{
				dgPerimenterEdge* const newEdge = &subdivision[edgeIndex];
				newEdge->m_next = edgeClipped[1];
				newEdge->m_prev = edgeClipped[0];
				edgeClipped[0]->m_next = newEdge;
				edgeClipped[1]->m_prev = newEdge;

				newEdge->m_vertex = &output[0];
				edgeClipped[1]->m_vertex = &output[1];
				poly = newEdge;

				output += 2;
				edgeIndex++;
				//dAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
				dAssert(edgeIndex < dInt32(sizeof(subdivision) / sizeof(subdivision[0])));
			}
		}

		dAssert(poly);
		count = 0;
		dgPerimenterEdge* intersection = poly;
		do 
		{
			contactOut[count] = *intersection->m_vertex;
			count++;
			intersection = intersection->m_next;
		} while (intersection != poly);
	}
	return count;
}

dInt32 ndContactSolver::PruneSupport(dInt32 count, const dVector& dir, const dVector* const points) const
{
	dInt32 index = 0;
	dFloat32 maxVal = dFloat32(-1.0e20f);
	for (dInt32 i = 0; i < count; i++) 
	{
		dFloat32 dist = dir.DotProduct(points[i]).GetScalar();
		if (dist > maxVal) 
		{
			index = i;
			maxVal = dist;
		}
	}
	return index;
}

dInt32 ndContactSolver::Prune2dContacts(const dMatrix& matrix, dInt32 count, ndContactPoint* const contactArray, dInt32 maxCount) const
{
	class ndConvexFaceNode
	{
		public:
		dVector m_point2d;
		ndContactPoint m_contact;
		ndConvexFaceNode* m_next;
		ndConvexFaceNode* m_prev;
		dInt32 m_mask;
	};

	class ndHullStackSegment
	{
		public:
		dVector m_p0;
		dVector m_p1;
		ndConvexFaceNode* m_edgeP0;
	};

	dVector xyMask(dVector::m_xMask | dVector::m_yMask);

	dVector array[D_MAX_CONTATCS];
	ndHullStackSegment stack[D_MAX_CONTATCS];

	ndConvexFaceNode convexHull[32];
	ndContactPoint buffer[32];

	// it is a big mistake to set contact to deepest penetration because si cause unwanted pops.
	// is better to present the original contact penetrations
	//dFloat32 maxPenetration = dFloat32(0.0f);
	for (dInt32 i = 0; i < count; i++) 
	{
		array[i] = matrix.UntransformVector(contactArray[i].m_point) & xyMask;
	}

	dInt32 i0 = PruneSupport(count, m_pruneSupportX, array);
	count--;
	convexHull[0].m_point2d = array[i0];
	convexHull[0].m_contact = contactArray[i0];
	stack[0].m_p0 = array[i0];
	array[i0] = array[count];
	contactArray[i0] = contactArray[count];

	dInt32 i1 = PruneSupport(count, m_pruneSupportX.Scale(dFloat32(-1.0f)), array);
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

	dInt32 hullCount = 2;
	dInt32 stackIndex = 2;
	dFloat32 totalArea = dFloat32(0.0f);
	while (stackIndex && count && (hullCount < dInt32 (sizeof(convexHull) / sizeof(convexHull[0])))) 
	{
		stackIndex--;

		ndHullStackSegment segment(stack[stackIndex]);
		dVector p1p0((segment.m_p1 - segment.m_p0));
		dFloat32 mag2 = p1p0.DotProduct(p1p0).GetScalar();
		if (mag2 > dFloat32(1.0e-5f)) 
		{
			dVector dir(m_pruneUpDir.CrossProduct(p1p0));
			dInt32 newIndex = PruneSupport(count, dir, array);

			dVector edge(array[newIndex] - segment.m_p0);
			dVector normal(p1p0.CrossProduct(edge));
			if (normal.m_z > dFloat32(1.e-4f)) 
			{
				totalArea += normal.m_z;
				dAssert(stackIndex < (D_MAX_CONTATCS - 2));
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
	dAssert(hullCount < dInt32 (sizeof(convexHull) / sizeof(convexHull[0])));

	dUpHeap<ndConvexFaceNode*, dFloat32> sortHeap(array, sizeof(array));
	ndConvexFaceNode* hullPoint = &convexHull[0];

	bool hasLinearCombination = true;
	while (hasLinearCombination) 
	{
		sortHeap.Flush();
		hasLinearCombination = false;
		ndConvexFaceNode* ptr = hullPoint;
		dVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do 
		{
			dVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			dFloat32 area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (sortHeap.Value() * dFloat32(16.0f) < totalArea)) 
		{
			ndConvexFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) 
			{
				if (hullPoint == corner) 
				{
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

	while (hullCount > maxCount) 
	{
		sortHeap.Flush();
		ndConvexFaceNode* ptr = hullPoint;
		dVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do 
		{
			dVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			dFloat32 area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (hullCount > maxCount)) 
		{
			ndConvexFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) 
			{
				if (hullPoint == corner) 
				{
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
	ndConvexFaceNode* ptr = hullPoint;
	do 
	{
		contactArray[hullCount] = ptr->m_contact;
		hullCount++;
		ptr = ptr->m_next;
	} while (ptr != hullPoint);
	return hullCount;
}

dInt32 ndContactSolver::Prune3dContacts(const dMatrix& matrix, dInt32 count, ndContactPoint* const contactArray, dInt32 maxCount) const
{
	dVector array[D_MAX_CONTATCS];
	dFloat32 max_x = dFloat32(1.0e20f);
	dInt32 maxIndex = 0;
	for (dInt32 i = 0; i < count; i++) 
	{
		array[i] = matrix.UntransformVector(contactArray[i].m_point);
		array[i].m_w = dFloat32(i);
		if (array[i].m_x < max_x) 
		{
			maxIndex = i;
			max_x = array[i].m_x;
		}
	}
	dSwap(array[0], array[maxIndex]);

	for (dInt32 i = 2; i < count; i++) 
	{
		dInt32 j = i;
		dVector tmp(array[i]);
		for (; array[j - 1].m_x > tmp.m_x; j--) 
		{
			dAssert(j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

	dFloat32 window = dFloat32(2.5e-3f);
	do 
	{
		dInt32 packContacts = 0;
		window *= dFloat32(2.0f);
		const dFloat32 window2 = window * window;

		dUnsigned8 mask[D_MAX_CONTATCS];
		memset(mask, 0, count * sizeof(dUnsigned8));
		for (dInt32 i = 0; i < count; i++) 
		{
			if (!mask[i]) 
			{
				const dFloat32 val = array[i].m_x + window;
				for (dInt32 j = i + 1; (j < count) && (array[j].m_x < val); j++) 
				{
					if (!mask[j]) 
					{
						dVector dp((array[j] - array[i]) & dVector::m_triplexMask);
						dAssert(dp.m_w == dFloat32(0.0f));
						dFloat32 dist2 = dp.DotProduct(dp).GetScalar();
						if (dist2 < window2) 
						{
							mask[j] = 1;
							packContacts = 1;
						}
					}
				}
			}
		}

		if (packContacts) 
		{
			dInt32 j = 0;
			for (dInt32 i = 0; i < count; i++) 
			{
				if (!mask[i]) 
				{
					array[j] = array[i];
					j++;
				}
			}
			count = j;
		}
	} while (count > maxCount);

	ndContactPoint tmpContact[16];
	for (dInt32 i = 0; i < count; i++) 
	{
		dInt32 index = dInt32(array[i].m_w);
		tmpContact[i] = contactArray[index];
	}

	memcpy(contactArray, tmpContact, count * sizeof(ndContactPoint));
	return count;
}

dInt32 ndContactSolver::PruneContacts(dInt32 count, dInt32 maxCount) const
{
	ndContactPoint* const contactArray = m_contactBuffer;
	//dFloat32 distTolerenace = m_contact->GetPruningTolerance();
	dVector origin(dVector::m_zero);
	for (dInt32 i = 0; i < count; i++) 
	{
		origin += contactArray[i].m_point;
	}
	dVector scale(dFloat32(1.0f) / count);
	origin = origin * scale;
	origin.m_w = dFloat32(1.0f);

	dMatrix covariance(dGetZeroMatrix());
	for (dInt32 i = 0; i < count; i++) 
	{
		dVector p((contactArray[i].m_point - origin) & dVector::m_triplexMask);
		dAssert(p.m_w == dFloat32(0.0f));
		dMatrix matrix(p, p);
		covariance.m_front += matrix.m_front;
		covariance.m_up += matrix.m_up;
		covariance.m_right += matrix.m_right;
	}

	for (dInt32 i = 0; i < 3; i++) 
	{
		if (dAbs(covariance[i][i]) < (1.0e-6f)) 
		{
			for (dInt32 j = 0; j < 3; j++) 
			{
				covariance[i][j] = dFloat32(0.0f);
				covariance[j][i] = dFloat32(0.0f);
			}
		}
	}

	dVector eigen(covariance.EigenVectors());
	covariance.m_posit = origin;
	if (eigen[1] < eigen[2]) 
	{
		dSwap(eigen[1], eigen[2]);
		dSwap(covariance[1], covariance[2]);
	}
	if (eigen[0] < eigen[1]) 
	{
		dSwap(eigen[0], eigen[1]);
		dSwap(covariance[0], covariance[1]);
	}
	if (eigen[1] < eigen[2]) 
	{
		dSwap(eigen[1], eigen[2]);
		dSwap(covariance[1], covariance[2]);
	}

	const dFloat32 eigenValueError = dFloat32(1.0e-4f);
	if (eigen[2] > eigenValueError) 
	{
		// 3d convex Hull
		return Prune3dContacts(covariance, count, contactArray, maxCount);
	}
	else if (eigen[1] > eigenValueError) 
	{
		// is a 2d or 1d convex hull
		return Prune2dContacts(covariance, count, contactArray, maxCount);
	}
	else if (eigen[0] > eigenValueError) 
	{
		// is a line or a point convex hull
		if (count > 2) 
		{
			dFloat32 maxValue = dFloat32(-1.0e10f);
			dFloat32 minValue = dFloat32(-1.0e10f);
			dInt32 j0 = 0;
			dInt32 j1 = 0;
			for (dInt32 i = 0; i < count; i++) 
			{
				dFloat32 dist = contactArray[i].m_point.DotProduct(covariance.m_front).GetScalar();
				if (dist > maxValue) 
				{
					j0 = i;
					maxValue = dist;
				}
				if (-dist > minValue) 
				{
					j1 = i;
					minValue = -dist;
				}
			}
			ndContactPoint c0(contactArray[j0]);
			ndContactPoint c1(contactArray[j1]);
			contactArray[0] = c0;
			contactArray[1] = c1;
			count = 2;
		}
		if (count == 2)
		{
			dVector segment(contactArray[1].m_point - contactArray[0].m_point);
			dFloat32 dist2 = segment.DotProduct(segment).GetScalar();
			if (dist2 < dFloat32(5.0e-3f * 5.0e-3f))
			{
				count = 1;
			}
		}
		//dAssert(count <= 1);
		return count;
	}

	return 1;
}

dFloat32 ndContactSolver::RayCast(const dVector& localP0, const dVector& localP1, ndContactPoint& contactOut)
{
	dVector point(localP0);
	dVector point0(localP0);
	dVector normal(dVector::m_zero);
	dVector p0p1(localP0 - localP1);

	// avoid NaN as a result of a division by zero
	if ((p0p1.TestZero().GetSignMask() & 7) == 7) 
	{
		return dFloat32(1.2f);
	}

	dFloat32 param = dFloat32(0.0f);

	dInt32 index = 0;
	memset(m_hullSum, 0, 4 * sizeof(m_hullSum[0]));
	const ndShapeConvex* const shape = m_instance0.GetShape()->GetAsShapeConvex();

	dVector dir1(p0p1.Normalize());
	m_hullDiff[0] = shape->SupportVertex(dir1, nullptr) - point;
	dBigVector v(m_hullDiff[0]);
	index = 1;
	do 
	{
		dInt32 iter = 0;
		dInt32 cycling = 0;
		dFloat64 minDist = dFloat32(1.0e20f);

		do 
		{
			dAssert(v.m_w == dFloat32(0.0f));
			const dFloat64 distance = v.DotProduct(v).GetScalar();
			if (distance < dFloat32(1.0e-9f)) 
			{
				index = -1;
				break;
			}

			if (distance < minDist) 
			{
				minDist = distance;
				cycling = -1;
			}
			cycling++;
			if (cycling > 4) 
			{
				index = -1;
				break;
			}

			dVector dir(v.Scale(-dRsqrt(dFloat32(distance))));
			dAssert(dir.m_w == dFloat32(0.0f));
			m_hullDiff[index] = shape->SupportVertex(dir, nullptr) - point;
			const dBigVector w(m_hullDiff[index]);
			const dVector wv(w - v);
			dAssert(wv.m_w == dFloat32(0.0f));
			const dFloat32 distance1 = dir.DotProduct(wv).GetScalar();
			if (distance1 < dFloat64(1.0e-3f)) 
			{
				normal = dir;
				break;
			}

			index++;
			switch (index)
			{
				case 2:
				{
					v = ReduceLine(index);
					break;
				}

				case 3:
				{
					v = ReduceTriangle(index);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum(index);
					break;
				}
			}

			iter++;
		} while (iter < D_CONNICS_CONTATS_ITERATIONS);

		dAssert(index);
		if (index > 0) 
		{
			dVector q(v + point);
			dFloat32 den = normal.DotProduct(p0p1).GetScalar();
			if (dAbs(den) < dFloat32(1.0e-12f)) 
			{
				den = dSign(den) * dFloat32(1.0e-12f);
			}
			dAssert(normal.m_w == dFloat32(0.0f));
			dFloat32 t1 = normal.DotProduct(localP0 - q).GetScalar() / den;

			if (t1 < param) 
			{
				index = -1;
				t1 = dFloat32(0.0f);
			}
			else if (t1 > dFloat32 (1.0f)) 
			{
				index = -1;
				t1 = dFloat32(1.0f);
			}
			param = t1;

			point = localP0 - p0p1.Scale(param);
			dVector step(point0 - point);
			point0 = point;
			for (dInt32 i = 0; i < index; i++) 
			{
				m_hullDiff[i] += step;
			}

			switch (index)
			{
				case 1:
				{
					v = m_hullDiff[0];
					break;
				}

				case 2:
				{
					v = ReduceLine(index);
					break;
				}

				case 3:
				{
					v = ReduceTriangle(index);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum(index);
					break;
				}
			}
		}
	} while (index >= 0);

	if ((param > dFloat32(0.0f)) && (param < dFloat32(1.0f)))
	{
		contactOut.m_normal = normal;
	}
	else 
	{
		param = dFloat32(1.2f);
	}
	return param;
}

D_INLINE ndMinkFace* ndContactSolver::NewFace()
{
	ndMinkFace* face = (ndMinkFace*)m_freeFace;
	if (m_freeFace) 
	{
		m_freeFace = m_freeFace->m_next;
	}
	else 
	{
		face = &m_facePool[m_faceIndex];
		m_faceIndex++;
		if (m_faceIndex >= D_CONVEX_MINK_MAX_FACES) 
		{
			return nullptr;
		}
	}

#ifdef _DEBUG
	memset(face, 0, sizeof(ndMinkFace));
#endif
	return face;
}

D_INLINE ndMinkFace* ndContactSolver::AddFace(dInt32 v0, dInt32 v1, dInt32 v2)
{
	ndMinkFace* const face = NewFace();
	face->m_mark = 0;
	face->m_vertex[0] = dInt16(v0);
	face->m_vertex[1] = dInt16(v1);
	face->m_vertex[2] = dInt16(v2);
	return face;
}

D_INLINE void ndContactSolver::PushFace(ndMinkFace* const face)
{
	dInt32 i0 = face->m_vertex[0];
	dInt32 i1 = face->m_vertex[1];
	dInt32 i2 = face->m_vertex[2];

	dPlane plane(m_hullDiff[i0], m_hullDiff[i1], m_hullDiff[i2]);
	dFloat32 mag2 = plane.DotProduct(plane & dVector::m_triplexMask).GetScalar();
	face->m_alive = 1;
	if (mag2 > dFloat32(1.0e-16f)) 
	{
		face->m_plane = plane.Scale(dRsqrt(mag2));
		ndMinkFace* face1 = face;
		Push(face1, face->m_plane.m_w);
	}
	else 
	{
		face->m_plane = dPlane(dVector::m_zero);
	}
}

D_INLINE void ndContactSolver::DeleteFace(ndMinkFace* const face)
{
	dgFaceFreeList* const freeFace = (dgFaceFreeList*)face;
	freeFace->m_next = m_freeFace;
	m_freeFace = freeFace;
}

dInt32 ndContactSolver::CalculateIntersectingPlane(dInt32 count)
{
	dAssert(count >= 1);
	if (count == 1) 
	{
		SupportVertex(m_contact->m_initialSeparatingVector.Scale(dFloat32(-1.0f)), 1);
		dVector err(m_hullDiff[1] - m_hullDiff[0]);
		dAssert(err.m_w == dFloat32(0.0f));
		if (err.DotProduct(err).GetScalar() < dFloat32(1.0e-8f)) 
		{
			return -1;
		}
		count = 2;
	}

	if (count == 2) 
	{
		dVector e0(m_hullDiff[1] - m_hullDiff[0]);
		dAssert(e0.m_w == dFloat32(0.0f));
		dAssert(e0.DotProduct(e0).GetScalar() > dFloat32(0.0f));
		dMatrix matrix(e0.Scale(dRsqrt(e0.DotProduct(e0).GetScalar())));
		dMatrix rotation(dPitchMatrix(dFloat32(45.0f * dDegreeToRad)));
		dFloat32 maxArea = dFloat32(0.0f);
		for (dInt32 i = 0; i < 8; i++) 
		{
			SupportVertex(matrix[1], 3);
			dVector e1(m_hullDiff[3] - m_hullDiff[0]);
			dAssert(e1.m_w == dFloat32(0.0f));
			dVector area(e0.CrossProduct(e1));
			dFloat32 area2 = area.DotProduct(area).GetScalar();
			if (area2 > maxArea) 
			{
				m_hullSum[2] = m_hullSum[3];
				m_hullDiff[2] = m_hullDiff[3];
				maxArea = area2;
			}
			matrix = rotation * matrix;
		}
		if (dAbs(maxArea) < dFloat32(1e-15f)) 
		{
			return -1;
		}
		dAssert(maxArea > dFloat32(0.0f));
		count++;
	}

	dFloat32 volume = dFloat32(0.0f);
	if (count == 3) 
	{
		dVector e10(m_hullDiff[1] - m_hullDiff[0]);
		dVector e20(m_hullDiff[2] - m_hullDiff[0]);
		dVector normal(e10.CrossProduct(e20));
		dAssert(normal.m_w == dFloat32(0.0f));
		dFloat32 mag2 = normal.DotProduct(normal).GetScalar();
		dAssert(mag2 > dFloat32(0.0f));
		normal = normal.Scale(dRsqrt(mag2));
		SupportVertex(normal, 3);
		volume = normal.DotProduct(m_hullDiff[3] - m_hullDiff[0]).GetScalar();
		if (dAbs(volume) < dFloat32(1.0e-10f)) 
		{
			normal = normal.Scale(dFloat32(-1.0f));
			SupportVertex(normal, 3);
			volume = -normal.DotProduct(m_hullDiff[3] - m_hullDiff[0]).GetScalar();
			if (dAbs(volume) < dFloat32(1.0e-10f)) 
			{
				volume = dFloat32(0.0f);
			}
		}
		count = 4;
	}
	else if (count == 4) 
	{
		dVector e0(m_hullDiff[1] - m_hullDiff[0]);
		dVector e1(m_hullDiff[2] - m_hullDiff[0]);
		dVector e2(m_hullDiff[3] - m_hullDiff[0]);
		dVector n(e1.CrossProduct(e2));
		dAssert(n.m_w == dFloat32(0.0f));
		volume = e0.DotProduct(n).GetScalar();
	}

	dAssert(count == 4);
	if (volume > dFloat32(0.0f)) 
	{
		dSwap(m_hullSum[1], m_hullSum[0]);
		dSwap(m_hullDiff[1], m_hullDiff[0]);
	}

	if (dAbs(volume) < dFloat32(1e-15f)) 
	{
		// this volume is unrealizable, let us build  a different tetrahedron using the method of core 200
		dVector e1;
		dVector e2;
		dVector e3;
		dVector normal(dFloat32(0.0f));

		const dInt32 nCount = dInt32(sizeof(m_hullDirs) / sizeof(m_hullDirs[0]));
		const dFloat32 D_CALCULATE_SEPARATING_PLANE_ERROR = dFloat32(1.0f / 1024.0f);

		dFloat32 error2 = dFloat32(0.0f);
		SupportVertex(m_hullDirs[0], 0);

		dInt32 i = 1;
		for (; i < nCount; i++) 
		{
			SupportVertex(m_hullDirs[i], 1);
			e1 = m_hullDiff[1] - m_hullDiff[0];
			dAssert(e1.m_w == dFloat32(0.0f));
			error2 = e1.DotProduct(e1).GetScalar();
			if (error2 > D_CALCULATE_SEPARATING_PLANE_ERROR) 
			{
				break;
			}
		}

		for (i++; i < nCount; i++) 
		{
			SupportVertex(m_hullDirs[i], 2);
			e2 = m_hullDiff[2] - m_hullDiff[0];
			normal = e1.CrossProduct(e2);
			dAssert(normal.m_w == dFloat32(0.0f));
			error2 = normal.DotProduct(normal).GetScalar();
			if (error2 > D_CALCULATE_SEPARATING_PLANE_ERROR) 
			{
				break;
			}
		}

		error2 = dFloat32(0.0f);
		for (i++; i < nCount; i++) 
		{
			SupportVertex(m_hullDirs[i], 3);
			e3 = m_hullDiff[3] - m_hullDiff[0];
			dAssert(normal.m_w == dFloat32(0.0f));
			error2 = normal.DotProduct(e3).GetScalar();
			if (dAbs(error2) > D_CALCULATE_SEPARATING_PLANE_ERROR) 
			{
				break;
			}
		}

		if (i >= nCount) 
		{
			return -1;
		}

		if (error2 > dFloat32(0.0f)) 
		{
			dSwap(m_hullSum[1], m_hullSum[2]);
			dSwap(m_hullDiff[1], m_hullDiff[2]);
		}

		#ifdef _DEBUG
		{
			dVector f0(m_hullDiff[1] - m_hullDiff[0]);
			dVector f1(m_hullDiff[2] - m_hullDiff[0]);
			dVector f2(m_hullDiff[3] - m_hullDiff[0]);
			dVector n(f1.CrossProduct(f2));
			dAssert(n.m_w == dFloat32(0.0f));
			dFloat32 volume1 = f0.DotProduct(n).GetScalar();
			dAssert(volume1 < dFloat32(0.0f));
		}
		#endif
	}

	// clear the face cache!!
	Flush();
	m_faceIndex = 0;
	m_vertexIndex = 4;
	m_freeFace = nullptr;

	ndMinkFace* const f0 = AddFace(0, 1, 2);
	ndMinkFace* const f1 = AddFace(0, 2, 3);
	ndMinkFace* const f2 = AddFace(2, 1, 3);
	ndMinkFace* const f3 = AddFace(1, 0, 3);

	f0->m_twin[0] = f3;
	f0->m_twin[1] = f2;
	f0->m_twin[2] = f1;

	f1->m_twin[0] = f0;
	f1->m_twin[1] = f2;
	f1->m_twin[2] = f3;

	f2->m_twin[0] = f0;
	f2->m_twin[1] = f3;
	f2->m_twin[2] = f1;

	f3->m_twin[0] = f0;
	f3->m_twin[1] = f1;
	f3->m_twin[2] = f2;

	PushFace(f0);
	PushFace(f1);
	PushFace(f2);
	PushFace(f3);

	dInt32 cycling = 0;
	dInt32 iterCount = 0;
	dFloat32 cyclingMem[4];
	cyclingMem[0] = dFloat32(1.0e10f);
	cyclingMem[1] = dFloat32(1.0e10f);
	cyclingMem[2] = dFloat32(1.0e10f);
	cyclingMem[3] = dFloat32(1.0e10f);

	const dFloat32 resolutionScale = dFloat32(0.125f);
	const dFloat32 minTolerance = D_PENETRATION_TOL;

	while (GetCount()) 
	{
		ndMinkFace* const faceNode = (*this)[0];
		Pop();

		if (faceNode->m_alive) {
			SupportVertex(faceNode->m_plane & dVector::m_triplexMask, m_vertexIndex);
			const dVector& p = m_hullDiff[m_vertexIndex];
			dFloat32 dist = faceNode->m_plane.Evalue(p);
			dFloat32 distTolerance = dMax(dAbs(faceNode->m_plane.m_w) * resolutionScale, minTolerance);

			if (dist < distTolerance) 
			{
				dVector sum[3];
				dVector diff[3];
				m_separatingVector = faceNode->m_plane & dVector::m_triplexMask;
				for (dInt32 i = 0; i < 3; i++) 
				{
					dInt32 j = faceNode->m_vertex[i];
					sum[i] = m_hullSum[j];
					diff[i] = m_hullDiff[j];
				}
				for (dInt32 i = 0; i < 3; i++) 
				{
					m_hullSum[i] = sum[i];
					m_hullDiff[i] = diff[i];
				}
				return 3;
			}

			iterCount++;
			bool isCycling = false;
			cyclingMem[cycling] = dist;
			if (iterCount > 10) 
			{
				dInt32 cyclingIndex = cycling;
				for (dInt32 i = 0; i < 3; i++) 
				{
					dInt32 cyclingIndex0 = (cyclingIndex - 1) & 3;
					if (((cyclingMem[cyclingIndex0] - cyclingMem[cyclingIndex]) < dFloat32(-1.0e-5f))) 
					{
						isCycling = true;
						cyclingMem[0] = dFloat32(1.0e10f);
						cyclingMem[1] = dFloat32(1.0e10f);
						cyclingMem[2] = dFloat32(1.0e10f);
						cyclingMem[3] = dFloat32(1.0e10f);
						break;
					}
					cyclingIndex = cyclingIndex0;
				}
			}
			cycling = (cycling + 1) & 3;

			if (!isCycling) 
			{
				m_faceStack[0] = faceNode;
				dInt32 stackIndex = 1;
				dInt32 deletedCount = 0;

				while (stackIndex) 
				{
					stackIndex--;
					ndMinkFace* const face = m_faceStack[stackIndex];

					if (!face->m_mark && (face->m_plane.Evalue(p) > dFloat32(0.0f))) 
					{
						#ifdef _DEBUG
						for (dInt32 i = 0; i < deletedCount; i++) 
						{
							dAssert(m_deletedFaceList[i] != face);
						}
						#endif

						m_deletedFaceList[deletedCount] = face;
						deletedCount++;
						dAssert(deletedCount < dInt32 (sizeof(m_deletedFaceList) / sizeof(m_deletedFaceList[0])));
						face->m_mark = 1;

						for (dInt32 i = 0; i < 3; i++) 
						{
							ndMinkFace* const twinFace = face->m_twin[i];
							if (twinFace && !twinFace->m_mark) 
							{
								m_faceStack[stackIndex] = twinFace;
								stackIndex++;
								dAssert(stackIndex < dInt32 (sizeof(m_faceStack) / sizeof(m_faceStack[0])));
							}
						}
					}
				}

				//dAssert (SanityCheck());
				dInt32 newCount = 0;
				for (dInt32 i = 0; i < deletedCount; i++) 
				{
					ndMinkFace* const face = m_deletedFaceList[i];
					face->m_alive = 0;
					dAssert(face->m_mark == 1);
					dInt32 j0 = 2;
					for (dInt32 j1 = 0; j1 < 3; j1++) 
					{
						ndMinkFace* const twinFace = face->m_twin[j0];
						if (twinFace && !twinFace->m_mark) 
						{
							ndMinkFace* const newFace = NewFace();
							if (newFace) 
							{
								newFace->m_mark = 0;
								newFace->m_vertex[0] = dInt16(m_vertexIndex);
								newFace->m_vertex[1] = dInt16(face->m_vertex[j0]);
								newFace->m_vertex[2] = dInt16(face->m_vertex[j1]);
								PushFace(newFace);

								newFace->m_twin[1] = twinFace;
								dInt32 index = (twinFace->m_twin[0] == face) ? 0 : ((twinFace->m_twin[1] == face) ? 1 : 2);
								twinFace->m_twin[index] = newFace;

								m_coneFaceList[newCount] = newFace;
								newCount++;
								dAssert(newCount < dInt32 (sizeof(m_coneFaceList) / sizeof(m_coneFaceList[0])));
							}
							else 
							{
								// this is very rare but is does happend with some degenerated faces.
								return -1;
							}
						}
						j0 = j1;
					}
				}

				dInt32 i0 = newCount - 1;
				for (dInt32 i1 = 0; i1 < newCount; i1++) 
				{
					ndMinkFace* const faceA = m_coneFaceList[i0];
					dAssert(faceA->m_mark == 0);

					dInt32 j0 = newCount - 1;
					for (dInt32 j1 = 0; j1 < newCount; j1++) 
					{
						if (i0 != j0) 
						{
							ndMinkFace* const faceB = m_coneFaceList[j0];
							dAssert(faceB->m_mark == 0);
							if (faceA->m_vertex[2] == faceB->m_vertex[1]) 
							{
								faceA->m_twin[2] = faceB;
								faceB->m_twin[0] = faceA;
								break;
							}
						}
						j0 = j1;
					}
					i0 = i1;
				}

				m_vertexIndex++;
				dAssert(m_vertexIndex < dInt32 (sizeof(m_hullDiff) / sizeof(m_hullDiff[0])));
				//dAssert(SanityCheck());
			}
		}
		else 
		{
			DeleteFace(faceNode);
		}
	}
	return -1;
}

dInt32 ndContactSolver::CalculatePolySoupToHullContactsDescrete(ndPolygonMeshDesc& data)
{
	dAssert(data.m_faceCount);
	
	ndShapeConvexPolygon polygon;
	ndShapeInstance polySoupInstance(m_instance1);
	m_instance1.m_shape = &polygon;
	m_instance1.SetScale(dVector::m_one);
	m_instance1.m_localMatrix = dGetIdentityMatrix();
	m_instance1.m_globalMatrix = dGetIdentityMatrix();

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dInt32(data.m_vertexStrideInBytes / sizeof(dFloat32));
	
	dInt32 count = 0;
	dInt32 maxContacts = m_maxCount;
	dInt32 countleft = maxContacts;
	dInt32 maxReduceLimit = maxContacts - 16;
	
	const dVector& polygonInstanceScale = polySoupInstance.GetScale();
	const dMatrix polySoupGlobalMatrix = polySoupInstance.m_globalMatrix;
	const dMatrix polySoupGlobalAligmentMatrix = polySoupInstance.m_aligmentMatrix;
	
	dMatrix polySoupScaledMatrix(
		polySoupGlobalAligmentMatrix[0] * polygonInstanceScale,
		polySoupGlobalAligmentMatrix[1] * polygonInstanceScale,
		polySoupGlobalAligmentMatrix[2] * polygonInstanceScale,
		polySoupGlobalAligmentMatrix[3]);
	polySoupScaledMatrix = polySoupScaledMatrix * polySoupGlobalMatrix;
	
	dAssert(m_contact);
	dVector separatingVector(m_instance0.m_globalMatrix.m_up);
	
	const dInt32 stride = polygon.m_stride;
	const dFloat32* const vertex = polygon.m_vertex;
	dAssert(m_instance1.m_scaleType == ndShapeInstance::m_unit);
	dFloat32 closestDist = dFloat32(1.0e10f);
	ndContactPoint* const contactOut = m_contactBuffer;
	ndContact* const contactJoint = m_contact;
	dInt32* const indexArray = (dInt32*)data.m_faceVertexIndex;

	data.SortFaceArray();
	for (dInt32 i = data.m_faceCount - 1; (i >= 0) && (count < 32); i--) 
	{
		dInt32 address = data.m_faceIndexStart[i];
		const dInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = data.m_faceIndexCount[i];
		polygon.m_paddedCount = polygon.m_count;
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray(localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId(localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize(localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex(localIndexArray, polygon.m_count);
		polygon.m_normal = polygon.CalculateGlobalNormal(&polySoupInstance, dVector(&vertex[polygon.m_faceNormalIndex * stride]) & dVector::m_triplexMask);
		dAssert(polygon.m_normal.m_w == dFloat32(0.0f));
		for (dInt32 j = 0; j < polygon.m_count; j++) 
		{
			polygon.m_localPoly[j] = polySoupScaledMatrix.TransformVector(dVector(&vertex[localIndexArray[j] * stride]) & dVector::m_triplexMask);
		}
		
		contactJoint->m_separatingVector = separatingVector;
		m_maxCount = countleft;
		m_vertexIndex = 0;
		m_contactBuffer = &contactOut[count];
		dInt32 count1 = polygon.CalculateContactToConvexHullDescrete(&polySoupInstance, *this);
		closestDist = dMin(closestDist, contactJoint->m_separationDistance);

		if (count1 > 0)
		{
			if (!m_intersectionTestOnly)
			{
				count += count1;
				countleft -= count1;
				dAssert(countleft >= 0);
				if (count >= maxReduceLimit)
				{
					dAssert(0);
					//count = PruneContacts(count, contactOut, dFloat32(1.0e-2f), 16);
					//countleft = maxContacts - count;
					//dAssert(countleft >= 0);
					//proxy.m_maxContacts = countleft;
				}
			}
			else
			{
				//count = -1;
				count = 1;
				break;
			}
		}
	}

	m_contactBuffer = contactOut;
	m_separationDistance = closestDist;
	m_instance1.m_shape = polySoupInstance.m_shape;
	m_instance1 = polySoupInstance;

	return count;
}

dInt32 ndContactSolver::CalculatePolySoupToHullContactsContinue(ndPolygonMeshDesc& data)
{
	dAssert(data.m_faceCount);

	ndShapeConvexPolygon polygon;
	ndShapeInstance polySoupInstance(m_instance1);
	m_instance1.m_shape = &polygon;
	m_instance1.SetScale(dVector::m_one);
	m_instance1.m_localMatrix = dGetIdentityMatrix();
	m_instance1.m_globalMatrix = dGetIdentityMatrix();

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dInt32(data.m_vertexStrideInBytes / sizeof(dFloat32));

	dInt32 count = 0;
	dInt32 maxContacts = m_maxCount;
	dInt32 countleft = maxContacts;
	dInt32 maxReduceLimit = maxContacts - 16;

	const dVector& polygonInstanceScale = polySoupInstance.GetScale();
	const dMatrix polySoupGlobalMatrix = polySoupInstance.m_globalMatrix;
	const dMatrix polySoupGlobalAligmentMatrix = polySoupInstance.m_aligmentMatrix;

	dMatrix polySoupScaledMatrix(
		polySoupGlobalAligmentMatrix[0] * polygonInstanceScale,
		polySoupGlobalAligmentMatrix[1] * polygonInstanceScale,
		polySoupGlobalAligmentMatrix[2] * polygonInstanceScale,
		polySoupGlobalAligmentMatrix[3]);
	polySoupScaledMatrix = polySoupScaledMatrix * polySoupGlobalMatrix;

	dAssert(m_contact);
	dVector separatingVector(m_instance0.m_globalMatrix.m_up);

	const dInt32 stride = polygon.m_stride;
	const dFloat32* const vertex = polygon.m_vertex;
	dAssert(m_instance1.m_scaleType == ndShapeInstance::m_unit);
	
	ndContactPoint* const contactOut = m_contactBuffer;
	ndContact* const contactJoint = m_contact;
	dInt32* const indexArray = (dInt32*)data.m_faceVertexIndex;
	data.SortFaceArray();

	dVector n(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector p(dVector::m_zero);
	dVector q(dVector::m_zero);
	dFloat32 closestDist = dFloat32(1.0e10f);
	dFloat32 minTimeStep = m_timestep;

	dFloat32 timeNormalizer = m_timestep;
	dFloat32 epsilon = dFloat32(-1.0e-3f) * m_timestep;
	
	for (dInt32 i = 0; (i < data.m_faceCount) && (m_timestep >= (data.m_hitDistance[i] * timeNormalizer)); i++) 
	{
		dInt32 address = data.m_faceIndexStart[i];
		const dInt32* const localIndexArray = &indexArray[address];
		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = data.m_faceIndexCount[i];
		polygon.m_paddedCount = polygon.m_count;
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray(localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId(localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize(localIndexArray, polygon.m_count);
		polygon.m_faceNormalIndex = data.GetNormalIndex(localIndexArray, polygon.m_count);
		polygon.m_normal = polygon.CalculateGlobalNormal(&polySoupInstance, dVector(&vertex[polygon.m_faceNormalIndex * stride]) & dVector::m_triplexMask);
		dAssert(polygon.m_normal.m_w == dFloat32(0.0f));
		for (dInt32 j = 0; j < polygon.m_count; j++)
		{
			polygon.m_localPoly[j] = polySoupScaledMatrix.TransformVector(dVector(&vertex[localIndexArray[j] * stride]) & dVector::m_triplexMask);
		}

		contactJoint->m_separatingVector = separatingVector;
		m_maxCount = countleft;
		m_vertexIndex = 0;
		m_contactBuffer = &contactOut[count];
		dInt32 count1 = polygon.CalculateContactToConvexHullContinue(&polySoupInstance, *this);
		
		if (count1 > 0)
		{
			dFloat32 error = m_timestep - minTimeStep;
			if (error < epsilon) 
			{
				count = 0;
				countleft = maxContacts;
				for (dInt32 j = 0; j < count1; j++) 
				{
					contactOut[j] = m_contactBuffer[j];
				}
			}
			count += count1;
			countleft -= count1;
			dAssert(countleft >= 0);
			if (count >= maxReduceLimit) 
			{
				dAssert(0);
				//count = PruneContacts(count, contactOut, dFloat32(1.0e-2f), 16);
				//countleft = maxContacts - count;
				//dAssert(countleft >= 0);
				//proxy.m_maxContacts = countleft;
			}
		}

		closestDist = dMin(closestDist, contactJoint->m_separationDistance);
		if (m_timestep <= minTimeStep) 
		{
			minTimeStep = m_timestep;
			n = m_separatingVector;
			p = m_closestPoint0;
			q = m_closestPoint1;
		}
	}

	m_contactBuffer = contactOut;
	m_separationDistance = closestDist;
	m_instance1.m_shape = polySoupInstance.m_shape;
	m_instance1 = polySoupInstance;

	return count;
}

bool ndContactSolver::CalculateClosestPoints()
{
	dInt32 simplexPointCount = CalculateClosestSimplex();
	if (simplexPointCount < 0)
	{
		simplexPointCount = CalculateIntersectingPlane(-simplexPointCount);
	}

	if (simplexPointCount > 0)
	{
		dAssert((simplexPointCount > 0) && (simplexPointCount <= 3));
		CalculateContactFromFeacture(simplexPointCount);

		const dMatrix& matrix0 = m_instance0.m_globalMatrix;
		const dMatrix& matrix1 = m_instance1.m_globalMatrix;
		m_closestPoint0 = matrix0.TransformVector(m_instance0.SupportVertexSpecialProjectPoint(matrix0.UntransformVector(m_closestPoint0), matrix0.UnrotateVector(m_separatingVector)));
		m_closestPoint1 = matrix1.TransformVector(m_instance1.SupportVertexSpecialProjectPoint(matrix1.UntransformVector(m_closestPoint1), matrix1.UnrotateVector(m_separatingVector * dVector::m_negOne)));
		m_vertexIndex = simplexPointCount;
	}
	return simplexPointCount >= 0;
}
