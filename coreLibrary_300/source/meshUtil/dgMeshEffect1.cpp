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
#include "dgMeshEffect.h"
#include "dgCollisionBVH.h"
#include "dgCollisionCompound.h"
#include "dgCollisionConvexHull.h"


dgMeshEffect::dgMeshBVH::dgFitnessList::dgFitnessList (dgMemoryAllocator* const allocator)
	:dgTree <dgMeshBVHNode*, dgMeshBVHNode*>(allocator)
{
}

dgFloat64 dgMeshEffect::dgMeshBVH::dgFitnessList::TotalCost () const
{
	dgFloat64 cost = dgFloat32 (0.0f);
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgTreeNode* const node = iter.GetNode();
		dgMeshBVHNode* const box = node->GetInfo();
		cost += box->m_area;
	}
	return cost;
}


dgMeshEffect::dgMeshBVH::dgMeshBVHNode::dgMeshBVHNode (const dgMeshEffect* const mesh, dgEdge* const face, void* const userData)
	:m_area(dgFloat32 (0.0f))
	,m_face (face)
	,m_userData(userData)
	,m_left (NULL)
	,m_right(NULL)
	,m_parent(NULL)
{
	dgBigVector p0(1.0e30, 1.0e30, 1.0e30, 0.0);
	dgBigVector p1(-1.0e30, -1.0e30, -1.0e30, 0.0);

	const dgBigVector* const points = (dgBigVector*) mesh->GetVertexPool();

	dgEdge* ptr = m_face;
	do {
		dgInt32 i = ptr->m_incidentVertex;
		const dgBigVector& p = points[i];
		p0.m_x = dgMin(p.m_x, p0.m_x);
		p0.m_y = dgMin(p.m_y, p0.m_y);
		p0.m_z = dgMin(p.m_z, p0.m_z);

		p1.m_x = dgMax(p.m_x, p1.m_x);
		p1.m_y = dgMax(p.m_y, p1.m_y);
		p1.m_z = dgMax(p.m_z, p1.m_z);

		ptr = ptr->m_next;
	} while (ptr != face);


	SetBox (dgVector (dgFloat32 (p0.m_x) - dgFloat32 (0.1f), dgFloat32 (p0.m_y) - dgFloat32 (0.1f), dgFloat32 (p0.m_z) - dgFloat32 (0.1f), 0.0f), 
			dgVector (dgFloat32 (p1.m_x) + dgFloat32 (0.1f), dgFloat32 (p1.m_y) + dgFloat32 (0.1f), dgFloat32 (p1.m_z) + dgFloat32 (0.1f), 0.0f));
}

dgMeshEffect::dgMeshBVH::dgMeshBVHNode::dgMeshBVHNode (dgMeshBVHNode* const left, dgMeshBVHNode* const right)
	:m_area(dgFloat32 (0.0f))
	,m_face (NULL)
	,m_left (left)
	,m_right(right)
	,m_parent(NULL)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	dgVector p0 (dgMin (left->m_p0.m_x, right->m_p0.m_x), dgMin (left->m_p0.m_y, right->m_p0.m_y), dgMin (left->m_p0.m_z, right->m_p0.m_z), dgFloat32 (0.0f));
	dgVector p1 (dgMax (left->m_p1.m_x, right->m_p1.m_x), dgMax (left->m_p1.m_y, right->m_p1.m_y), dgMax (left->m_p1.m_z, right->m_p1.m_z), dgFloat32 (0.0f));
	SetBox(p0, p1);
}


dgMeshEffect::dgMeshBVH::dgMeshBVHNode::~dgMeshBVHNode ()
{
	if (m_left) {
		delete m_left;
	}
	if (m_right) {
		delete m_right;
	}
}


void dgMeshEffect::dgMeshBVH::dgMeshBVHNode::SetBox (const dgVector& p0, const dgVector& p1)
{
	m_p0 = p0;
	m_p1 = p1;
	m_p0.m_w = 0.0f;
	m_p1.m_w = 0.0f;

	dgVector size ((m_p1 - m_p0).Scale3 (dgFloat32 (0.5f)));
	dgVector size1(size.m_y, size.m_z, size.m_x, dgFloat32 (0.0f));
	m_area = size % size1;
}



dgMeshEffect::dgMeshBVH::dgMeshBVH (dgMeshEffect* const mesh)
	:m_mesh(mesh)
	,m_rootNode(NULL)
	,m_fitness(m_mesh->GetAllocator())
{
}


dgMeshEffect::dgMeshBVH::~dgMeshBVH()
{
	Cleanup ();
}

dgMeshEffect* dgMeshEffect::CreateFromSerialization (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData)
{
	return new (allocator) dgMeshEffect(allocator, deserialization, userData);
}

void dgMeshEffect::Serialize (dgSerialize callback, void* const userData) const
{
	dgInt32 faceCount = 0;
	dgTree<dgEdge*, dgEdge*>filter(GetAllocator());
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const face = &iter.GetNode()->GetInfo();
		if (!filter.Find(face) && (face->m_incidentFace > 0)) {
			faceCount ++;
			dgEdge* edge = face; 
			do {
				filter.Insert(edge, edge);
				edge = edge->m_next;
			} while (edge != face);
		}
	}

	callback (userData, &faceCount, sizeof (dgInt32));
	callback (userData, &m_pointCount, sizeof (dgInt32));
	callback (userData, &m_atribCount, sizeof (dgInt32));
	callback (userData, &m_atribCount, sizeof (dgInt32));

	callback (userData, m_points, m_pointCount * sizeof (dgBigVector));
	callback (userData, m_attrib, m_atribCount * sizeof (dgVertexAtribute));

	filter.RemoveAll();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const face = &iter.GetNode()->GetInfo();
		if (!filter.Find(face) && (face->m_incidentFace > 0)) {
			dgInt32 indices[1024];
			dgInt64 attibuteIndex[1024];
			dgInt32 vertexCount = 0;
			dgEdge* edge = face; 
			do {
				indices[vertexCount] = edge->m_incidentVertex;
				attibuteIndex[vertexCount] = edge->m_userData;
				vertexCount ++;
				filter.Insert(edge, edge);
				edge = edge->m_next;
			} while (edge != face);

			callback (userData, &vertexCount, sizeof (dgInt32));
			callback (userData, indices, vertexCount * sizeof (dgInt32));
			callback (userData, attibuteIndex, vertexCount * sizeof (dgInt64));
		}
	}
}

void dgMeshEffect::dgMeshBVH::Build ()
{
	for (void* faceNode = m_mesh->GetFirstFace (); faceNode; faceNode = m_mesh->GetNextFace(faceNode)) {
		if (!m_mesh->IsFaceOpen(faceNode)) {
			dgEdge* const face = &((dgTreeNode*)faceNode)->GetInfo();
			AddFaceNode(face, NULL);
		}
	}
	ImproveNodeFitness ();
}

void dgMeshEffect::dgMeshBVH::Cleanup ()
{
	if (m_rootNode) {
		delete m_rootNode;
	}
}

dgFloat32 dgMeshEffect::dgMeshBVH::CalculateSurfaceArea (dgMeshBVHNode* const node0, dgMeshBVHNode* const node1, dgVector& minBox, dgVector& maxBox) const
{
	minBox = dgVector (dgMin (node0->m_p0.m_x, node1->m_p0.m_x), dgMin (node0->m_p0.m_y, node1->m_p0.m_y), dgMin (node0->m_p0.m_z, node1->m_p0.m_z), dgFloat32 (0.0f));
	maxBox = dgVector (dgMax (node0->m_p1.m_x, node1->m_p1.m_x), dgMax (node0->m_p1.m_y, node1->m_p1.m_y), dgMax (node0->m_p1.m_z, node1->m_p1.m_z), dgFloat32 (0.0f));		
	dgVector side0 ((maxBox - minBox).Scale3 (dgFloat32 (0.5f)));
	dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
	return side0 % side1;
}



void dgMeshEffect::dgMeshBVH::ImproveNodeFitness (dgMeshBVHNode* const node)
{
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (node->m_parent)	{
		if (node->m_parent->m_left == node) {
			dgFloat32 cost0 = node->m_area;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_left = node->m_right;
				node->m_right = parent;
				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;		
				parent->m_area = cost1;


			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_left = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;		
				parent->m_area = cost2;
			}
		} else {
			dgFloat32 cost0 = node->m_area;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_right = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;		
				parent->m_area = cost1;

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_right = node->m_right;
				node->m_right = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;		
				parent->m_area = cost2;
			}
		}
	}
	//	dgAssert (SanityCheck());
}


void dgMeshEffect::dgMeshBVH::ImproveNodeFitness ()
{
	dgFloat64 cost0 = m_fitness.TotalCost ();
	dgFloat64 cost1 = cost0;
	do {
		cost0 = cost1;
		dgFitnessList::Iterator iter (m_fitness);
		for (iter.Begin(); iter; iter ++) {
			dgFitnessList::dgTreeNode* const node = iter.GetNode();
			ImproveNodeFitness (node->GetInfo());
		}
		cost1 = m_fitness.TotalCost ();
	} while (cost1 < (dgFloat32 (0.95f)) * cost0);
}




dgMeshEffect::dgMeshBVH::dgMeshBVHNode* dgMeshEffect::dgMeshBVH::AddFaceNode (dgEdge* const face, void* const userData)
{
	dgMemoryAllocator* const allocator = m_mesh->GetAllocator();

	dgMeshBVHNode* const newNode = new (allocator) dgMeshBVHNode (m_mesh, face, userData);
	if (!m_rootNode) {
		m_rootNode = newNode;
	} else {

		dgVector p0;
		dgVector p1;		
		dgMeshBVHNode* sibling = m_rootNode;

		dgFloat32 surfaceArea = dgMeshBVH::CalculateSurfaceArea (newNode, sibling, p0, p1);
		while(sibling->m_left && sibling->m_right) {

			if (surfaceArea > sibling->m_area) {
				break;
			} 

			sibling->SetBox (p0, p1);

			dgVector leftP0;
			dgVector leftP1;		
			dgFloat32 leftSurfaceArea = CalculateSurfaceArea (newNode, sibling->m_left, leftP0, leftP1);

			dgVector rightP0;
			dgVector rightP1;		
			dgFloat32 rightSurfaceArea = CalculateSurfaceArea (newNode, sibling->m_right, rightP0, rightP1);

			if (leftSurfaceArea < rightSurfaceArea) {
				sibling = sibling->m_left;
				p0 = leftP0;
				p1 = leftP1;
				surfaceArea = leftSurfaceArea;
			} else {
				sibling = sibling->m_right;
				p0 = rightP0;
				p1 = rightP1;
				surfaceArea = rightSurfaceArea;
			}
		} 

		if (!sibling->m_parent) {
			m_rootNode = new (allocator) dgMeshBVHNode (sibling, newNode);
			m_fitness.Insert(m_rootNode, m_rootNode);
		} else {
			dgMeshBVHNode* const parent = sibling->m_parent;
			if (parent->m_left == sibling) {
				dgMeshBVHNode* const node = new (allocator) dgMeshBVHNode (sibling, newNode);
				m_fitness.Insert(node, node);
				parent->m_left = node;
				node->m_parent = parent;
			} else {
				dgAssert (parent->m_right == sibling); 
				dgMeshBVHNode* const node = new (allocator) dgMeshBVHNode (sibling, newNode);
				m_fitness.Insert(node, node);
				parent->m_right = node;
				node->m_parent = parent;
			}
		}
	}

	return newNode;
}


void dgMeshEffect::dgMeshBVH::RemoveNode (dgMeshBVHNode* const treeNode)
{
	if (!treeNode->m_parent) {
		delete (m_rootNode);
		m_rootNode = NULL;
	} else if (!treeNode->m_parent->m_parent) {
		dgMeshBVHNode* const root = m_rootNode;
		if (treeNode->m_parent->m_left == treeNode) {
			m_rootNode = treeNode->m_parent->m_right;
			treeNode->m_parent->m_right = NULL;
		} else {
			dgAssert (treeNode->m_parent->m_right == treeNode);
			m_rootNode = treeNode->m_parent->m_left;
			treeNode->m_parent->m_left= NULL;
		}
		m_rootNode->m_parent = NULL;
		dgAssert (m_fitness.Find(root));
		m_fitness.Remove(root);
		delete (root);

	} else {
		dgMeshBVHNode* const root = treeNode->m_parent->m_parent;
		if (treeNode->m_parent == root->m_left) {
			if (treeNode->m_parent->m_right == treeNode) {
				root->m_left = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = NULL;
			} else {
				dgAssert (treeNode->m_parent->m_left == treeNode);
				root->m_left = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = NULL;
			}
			root->m_left->m_parent = root;
		} else {
			if (treeNode->m_parent->m_right == treeNode) {
				root->m_right = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = NULL;
			} else {
				dgAssert (treeNode->m_parent->m_left == treeNode);
				root->m_right = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = NULL;
			}
			root->m_right->m_parent = root;
		}

		dgAssert (m_fitness.Find(treeNode->m_parent));
		m_fitness.Remove(treeNode->m_parent);
		delete (treeNode->m_parent);
	}

	//dgAssert (SanityCheck());
}



bool dgMeshEffect::dgMeshBVH::SanityCheck() const
{
	#ifdef _DEBUG
	dgAssert (m_mesh->Sanity ());

	if (!m_rootNode) {
		return false;
	}

	if ((!m_rootNode->m_left && m_rootNode->m_right) || (m_rootNode->m_left && !m_rootNode->m_right)) {
		return false;
	}

	if (m_rootNode->m_left && m_rootNode->m_right) {
		if (m_rootNode->m_left->m_parent != m_rootNode) {
			return false;
		}
		if (m_rootNode->m_right->m_parent != m_rootNode) {
			return false;
		}

		dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

		dgInt32 stack = 2;
		stackPool[0] = m_rootNode->m_left;
		stackPool[1] = m_rootNode->m_right;
		while (stack) {
			stack --;
			dgMeshBVHNode* const node = stackPool[stack];

			if ((node->m_parent->m_left != node) && (node->m_parent->m_right != node)){
				return false;
			}

			if (node->m_left) {
				dgAssert (node->m_right);
				stackPool[stack] = node->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				stackPool[stack] = node->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
			}
		}
	}
	#endif
	return true;
}


void dgMeshEffect::dgMeshBVH::GetOverlapNodes (dgList<dgMeshBVHNode*>& overlapNodes, const dgBigVector& p0, const dgBigVector& p1) const
{
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	dgInt32 stack = 1;
	stackPool[0] = m_rootNode;

	dgVector l0(p0);
	dgVector l1(p1);

	while (stack) {
		stack --;
		dgMeshBVHNode* const me = stackPool[stack];

		if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {

			if (!me->m_left) {
				dgAssert (!me->m_right);
				overlapNodes.Append(me);
			} else {
				dgAssert (me->m_left);
				dgAssert (me->m_right);
				stackPool[stack] = me->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
			}
		}
	}
}

dgFloat64 dgMeshEffect::dgMeshBVH::VertexRayCast (const dgBigVector& p0, const dgBigVector& p1) const
{
	dgAssert (0);
/*
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	dgInt32 stack = 1;
	stackPool[0] = m_rootNode;

	dgVector l0(p0);
	dgVector l1(p1);
	dgBigVector p1p0 (p1 - p0);
	dgFloat64 den = p1p0 % p1p0;

	const dgBigVector* const points = (dgBigVector*) m_mesh->GetVertexPool();
	while (stack) {
		stack --;
		dgMeshBVHNode* const me = stackPool[stack];

		if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
			if (!me->m_left) {
				dgAssert (!me->m_right);

				dgEdge* ptr = me->m_face;
				do {
					dgInt32 index = ptr->m_incidentVertex;
					const dgBigVector& q0 = points[index];
					dgBigVector q0p0 (q0 - p0);
					dgFloat64 alpha = q0p0 % p1p0;
					if ((alpha > (DG_BOOLEAN_ZERO_TOLERANCE * den)) && (alpha < (den - DG_BOOLEAN_ZERO_TOLERANCE))) {
						dgBigVector dist (p0 + p1p0.Scale3 (alpha / den) - q0);
						dgFloat64 dist2 = dist % dist;
						if (dist2 < (DG_BOOLEAN_ZERO_TOLERANCE * DG_BOOLEAN_ZERO_TOLERANCE)) {
							return alpha / den;
						}
					}

					ptr = ptr->m_next;
				} while (ptr != me->m_face);
			} else {
				dgAssert (me->m_left);
				dgAssert (me->m_right);
				stackPool[stack] = me->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
			}
		}
	}
*/
	return 1.2f;
}



bool dgMeshEffect::dgMeshBVH::RayRayIntersect (dgEdge* const edge, const dgMeshEffect* const otherMesh, dgEdge* const otherEdge, dgFloat64& param, dgFloat64& otherParam) const
{
	dgAssert (0);
/*
	dgBigVector ray_p0 (m_mesh->m_points[edge->m_incidentVertex]);
	dgBigVector ray_p1 (m_mesh->m_points[edge->m_twin->m_incidentVertex]);

	dgBigVector ray_q0 (otherMesh->m_points[otherEdge->m_incidentVertex]);
	dgBigVector ray_q1 (otherMesh->m_points[otherEdge->m_twin->m_incidentVertex]);

	dgBigVector p1p0 (ray_p1 - ray_p0);
	dgBigVector q1q0 (ray_q1 - ray_q0);
	dgBigVector p0q0 (ray_p0 - ray_q0);

	dgFloat64 a = p1p0 % p1p0;        // always >= 0
	dgFloat64 c = q1q0 % q1q0;        // always >= 0
	dgFloat64 b = p1p0 % q1q0;

	dgFloat64 d = (p1p0 % p0q0);
	dgFloat64 e = (q1q0 % p0q0);
	dgFloat64 den = a * c - b * b;   // always >= 0
	// compute the line parameters of the two closest points
	if (den < DG_BOOLEAN_ZERO_TOLERANCE) { 
		// the lines are almost parallel
		return false;
	} else {         
		// get the closest points on the infinite lines
		dgFloat64 t = b * e - c * d;
		dgFloat64 s = a * e - b * d;

		if (t < (DG_BOOLEAN_ZERO_TOLERANCE * den) || (s < (DG_BOOLEAN_ZERO_TOLERANCE * den)) || (t > (den - DG_BOOLEAN_ZERO_TOLERANCE)) ||  (s > (den - DG_BOOLEAN_ZERO_TOLERANCE))) {
			return false;
		}
		//dgBigVector normal (p1p0 * q1q0);
		//dgFloat64 dist0 = normal % (p1p0.Scale3 (t / den) - ray_p0);
		//dgFloat64 dist1 = normal % (q1q0.Scale3 (s / den) - ray_q0);
		dgBigVector r0 = ray_p0 + p1p0.Scale3 (t / den);
		dgBigVector r1 = ray_q0 + q1q0.Scale3 (s / den);
		dgBigVector r1r0 (r1 - r0);
		dgFloat64 dist2 = r1r0 % r1r0;
		if (dist2 > (DG_BOOLEAN_ZERO_TOLERANCE * DG_BOOLEAN_ZERO_TOLERANCE)) {
			return false;
		}

		param = t / den;
		otherParam = s / den;
	}
*/
	return true;
}


dgFloat64 dgMeshEffect::dgMeshBVH::RayFaceIntersect (const dgMeshBVHNode* const faceNode, const dgBigVector& p0, const dgBigVector& p1, bool doubleSidedFaces) const
{
	dgBigVector normal (m_mesh->FaceNormal(faceNode->m_face, m_mesh->GetVertexPool(), sizeof(dgBigVector)));

	dgBigVector diff (p1 - p0);

	dgFloat64 tOut = 2.0f;
	const dgBigVector* const points = (dgBigVector*) m_mesh->GetVertexPool();
	dgFloat64 dir = normal % diff;
	if (dir < 0.0f) {
		dgEdge* ptr = faceNode->m_face;
		do {
			dgInt32 index0 = ptr->m_incidentVertex;
			dgInt32 index1 = ptr->m_next->m_incidentVertex;
			dgBigVector p0v0 (points[index0] - p0);
			dgBigVector p0v1 (points[index1] - p0);
			dgFloat64 alpha = (diff * p0v1) % p0v0;
			if (alpha <= 0.0f) {
				return 1.2f;
			}

			ptr = ptr->m_next;
		} while (ptr != faceNode->m_face);

		dgInt32 index0 = ptr->m_incidentVertex;
		dgBigVector p0v0 (points[index0] - p0);
		tOut = normal % p0v0;
		dgFloat64 dist = normal % diff;
		tOut = tOut / dist;

	} else if (doubleSidedFaces && (dir > 0.0f)) {
		dgEdge* ptr = faceNode->m_face;
		do {
			dgInt32 index0 = ptr->m_incidentVertex;
			dgInt32 index1 = ptr->m_prev->m_incidentVertex;
			dgBigVector p0v0 (points[index0] - p0);
			dgBigVector p0v1 (points[index1] - p0);
			dgFloat64 alpha = (diff * p0v1) % p0v0;
			if (alpha <= 0.0f) {
				return 1.2f;
			}

			ptr = ptr->m_prev;
		} while (ptr != faceNode->m_face);

		dgInt32 index0 = ptr->m_incidentVertex;
		dgBigVector p0v0 (points[index0] - p0);
		tOut = normal % p0v0;
		dgFloat64 dist = normal % diff;
		tOut = tOut / dist;
	}

	if (tOut < 1.e-12f) {
		tOut = 2.0f;
	} else if (tOut > (1.0 - 1.e-12f)) {
		tOut = 2.0f;
	}
	return tOut;
}


dgMeshEffect::dgMeshBVH::dgMeshBVHNode* dgMeshEffect::dgMeshBVH::FaceRayCast (const dgBigVector& p0, const dgBigVector& p1, dgFloat64& paramOut, bool doubleSidedFaces) const
{
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	dgInt32 stack = 1;
	dgMeshBVHNode* node = NULL;

	stackPool[0] = m_rootNode;
	dgFloat64 maxParam = dgFloat32 (1.2f);

	dgVector l0(p0);
	dgVector l1(p1);
	l0 = l0 & dgVector::m_triplexMask;
	l1 = l1 & dgVector::m_triplexMask;
	dgFastRayTest ray (l0, l1);
	while (stack) {
		stack --;
		dgMeshBVHNode* const me = stackPool[stack];

		if (me && ray.BoxTest (me->m_p0, me->m_p1)) {

			if (!me->m_left) {
				dgAssert (!me->m_right);
				dgFloat64 param = RayFaceIntersect (me, p0, p1, doubleSidedFaces);
				if (param < maxParam) {
					node = me;
					maxParam = param;
				}

			} else {
				dgAssert (me->m_left);
				dgAssert (me->m_right);
				stackPool[stack] = me->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
			}
		}
	}

	paramOut = maxParam;
	return node;
}




dgMeshEffect::dgMeshEffect(dgMemoryAllocator* const allocator)
	:dgPolyhedra(allocator)
{
	Init();
}

dgMeshEffect::dgMeshEffect (dgMemoryAllocator* const allocator, const dgMatrix& planeMatrix, dgFloat32 witdth, dgFloat32 breadth, dgInt32 material, const dgMatrix& textureMatrix0, const dgMatrix& textureMatrix1)
	:dgPolyhedra(allocator)
{
	dgInt32 index[4];
	dgInt64 attrIndex[4];
	dgBigVector face[4];

	Init();

	face[0] = dgBigVector (dgFloat32 (0.0f), -witdth, -breadth, dgFloat32 (0.0f));
	face[1] = dgBigVector (dgFloat32 (0.0f),  witdth, -breadth, dgFloat32 (0.0f));
	face[2] = dgBigVector (dgFloat32 (0.0f),  witdth,  breadth, dgFloat32 (0.0f));
	face[3] = dgBigVector (dgFloat32 (0.0f), -witdth,  breadth, dgFloat32 (0.0f));

	for (dgInt32 i = 0; i < 4; i ++) {
		dgBigVector uv0 (textureMatrix0.TransformVector(face[i]));
		dgBigVector uv1 (textureMatrix1.TransformVector(face[i]));

		m_points[i] = planeMatrix.TransformVector(face[i]);

		m_attrib[i].m_vertex.m_x = m_points[i].m_x;
		m_attrib[i].m_vertex.m_y = m_points[i].m_y;
		m_attrib[i].m_vertex.m_z = m_points[i].m_z;
		m_attrib[i].m_vertex.m_w = dgFloat64 (0.0f);

		m_attrib[i].m_normal_x = planeMatrix.m_front.m_x;
		m_attrib[i].m_normal_y = planeMatrix.m_front.m_y;
		m_attrib[i].m_normal_z = planeMatrix.m_front.m_z;
		
		m_attrib[i].m_u0 = uv0.m_y;
		m_attrib[i].m_v0 = uv0.m_z;

		m_attrib[i].m_u1 = uv1.m_y;
		m_attrib[i].m_v1 = uv1.m_z;

		m_attrib[i].m_material = material;

		index[i] = i;
		attrIndex[i] = i;
	}

	m_pointCount = 4;
	m_atribCount = 4;
	BeginFace();
	AddFace (4, index, attrIndex);
	EndFace();
}


dgMeshEffect::dgMeshEffect(dgPolyhedra& mesh, const dgMeshEffect& source)
	:dgPolyhedra (mesh) 
{
	m_pointCount = source.m_pointCount;
	m_maxPointCount = source.m_maxPointCount;
	m_points = (dgBigVector*) GetAllocator()->MallocLow(dgInt32 (m_maxPointCount * sizeof(dgBigVector)));
	memcpy (m_points, source.m_points, m_pointCount * sizeof(dgBigVector));

	m_atribCount = source.m_atribCount;
	m_maxAtribCount = source.m_maxAtribCount;
	m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
	memcpy (m_attrib, source.m_attrib, m_atribCount * sizeof(dgVertexAtribute));
}


dgMeshEffect::dgMeshEffect(const dgMeshEffect& source)
	:dgPolyhedra (source) 
{
	m_pointCount = source.m_pointCount;
	m_maxPointCount = source.m_maxPointCount;
	m_points = (dgBigVector*) GetAllocator()->MallocLow(dgInt32 (m_maxPointCount * sizeof(dgBigVector)));
	memcpy (m_points, source.m_points, m_pointCount * sizeof(dgBigVector));

	m_atribCount = source.m_atribCount;
	m_maxAtribCount = source.m_maxAtribCount;
	m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
	memcpy (m_attrib, source.m_attrib, m_atribCount * sizeof(dgVertexAtribute));
}


dgMeshEffect::dgMeshEffect(dgCollisionInstance* const collision)
	:dgPolyhedra (collision->GetAllocator()) 
{
	class dgMeshEffectBuilder
	{
		public:
		dgMeshEffectBuilder ()
		{
			m_brush = 0;
			m_faceCount = 0;
			m_vertexCount = 0;
			m_maxFaceCount = 32;
			m_maxVertexCount = 32;
			m_vertex = (dgVector*) dgMallocStack(m_maxVertexCount * sizeof(dgVector));
			m_faceIndexCount = (dgInt32*) dgMallocStack(m_maxFaceCount * sizeof(dgInt32));
		}

		~dgMeshEffectBuilder ()
		{
			dgFreeStack (m_faceIndexCount);
			dgFreeStack (m_vertex);
		}

		static void GetShapeFromCollision (void* userData, dgInt32 vertexCount, const dgFloat32* faceVertex, dgInt32 id)
		{
			dgInt32 vertexIndex; 
			dgMeshEffectBuilder& builder = *((dgMeshEffectBuilder*)userData);


			if (builder.m_faceCount >= builder.m_maxFaceCount) {
				dgInt32* index;

				builder.m_maxFaceCount *= 2;
				index = (dgInt32*) dgMallocStack(builder.m_maxFaceCount * sizeof(dgInt32));
				memcpy (index, builder.m_faceIndexCount, builder.m_faceCount * sizeof(dgInt32));
				dgFreeStack(builder.m_faceIndexCount);
				builder.m_faceIndexCount = index;
			}
			builder.m_faceIndexCount[builder.m_faceCount] = vertexCount;
			builder.m_faceCount = builder.m_faceCount + 1;

			vertexIndex = builder.m_vertexCount; 
			dgFloat32 brush = dgFloat32 (builder.m_brush);
			for (dgInt32 i = 0; i < vertexCount; i ++) {
				if (vertexIndex >= builder.m_maxVertexCount) {
					builder.m_maxVertexCount *= 2;
					dgVector* const points = (dgVector*) dgMallocStack(builder.m_maxVertexCount * sizeof(dgVector));
					memcpy (points, builder.m_vertex, vertexIndex * sizeof(dgVector));
					dgFreeStack(builder.m_vertex);
					builder.m_vertex = points;
				}

				builder.m_vertex[vertexIndex].m_x = faceVertex[i * 3 + 0];
				builder.m_vertex[vertexIndex].m_y = faceVertex[i * 3 + 1];
				builder.m_vertex[vertexIndex].m_z = faceVertex[i * 3 + 2];
				builder.m_vertex[vertexIndex].m_w = brush;
				vertexIndex ++;
			}

			builder.m_vertexCount = vertexIndex;
		}

		dgInt32 m_brush;
		dgInt32 m_vertexCount;
		dgInt32 m_maxVertexCount;

		dgInt32 m_faceCount;
		dgInt32 m_maxFaceCount;

		dgVector* m_vertex;
		dgInt32* m_faceIndexCount;
	};

	dgMeshEffectBuilder builder;

	if (collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
		dgCollisionInfo collisionInfo;
		collision->GetCollisionInfo (&collisionInfo);

		dgInt32 brush = 0;
		dgMatrix matrix (collisionInfo.m_offsetMatrix);
		dgCollisionCompound* const compoundCollision = (dgCollisionCompound*) collision->GetChildShape();
		for (dgTree<dgCollisionCompound::dgNodeBase*, dgInt32>::dgTreeNode* node = compoundCollision->GetFirstNode(); node; node = compoundCollision->GetNextNode(node)) {
			builder.m_brush = brush;
			brush ++;
			dgCollisionInstance* const childShape = compoundCollision->GetCollisionFromNode(node);
			childShape->DebugCollision (matrix, (dgCollision::OnDebugCollisionMeshCallback) dgMeshEffectBuilder::GetShapeFromCollision, &builder);
		}

	} else {
		dgMatrix matrix (dgGetIdentityMatrix());
		collision->DebugCollision (matrix, (dgCollision::OnDebugCollisionMeshCallback) dgMeshEffectBuilder::GetShapeFromCollision, &builder);
	}

	dgStack<dgInt32>indexList (builder.m_vertexCount);

	dgVertexListToIndexList (&builder.m_vertex[0].m_x, sizeof (dgVector), sizeof (dgVector), 0, builder.m_vertexCount, &indexList[0], DG_VERTEXLIST_INDEXLIST_TOL);	

	dgStack<dgInt32> materialIndex(builder.m_faceCount);
	dgStack<dgInt32> m_normalUVIndex(builder.m_vertexCount);

	dgVector normalUV(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	memset (&materialIndex[0], 0, size_t (materialIndex.GetSizeInBytes()));
	memset (&m_normalUVIndex[0], 0, size_t (m_normalUVIndex.GetSizeInBytes()));

	Init();
	BuildFromVertexListIndexList(builder.m_faceCount, builder.m_faceIndexCount, &materialIndex[0],
								 &builder.m_vertex[0].m_x, sizeof (dgVector), &indexList[0],
								 &normalUV.m_x, sizeof (dgVector), &m_normalUVIndex[0],
								 &normalUV.m_x, sizeof (dgVector), &m_normalUVIndex[0],
								 &normalUV.m_x, sizeof (dgVector), &m_normalUVIndex[0]);

    RepairTJoints();
	CalculateNormals(dgFloat32 (45.0f * 3.141592f/180.0f));
}


dgMeshEffect::dgMeshEffect(dgMemoryAllocator* const allocator, const char* const fileName)
	:dgPolyhedra (allocator) 
{
	class ParceOFF
	{
		public:
		enum Token
		{
			m_off,	
			m_value,
			m_end,
		};

		ParceOFF (FILE* const file)
			:m_file (file)
		{
		}

		Token GetToken(char* const buffer) const
		{
			while (!feof (m_file) && fscanf (m_file, "%s", buffer)) {
				if (buffer[0] == '#') {
					SkipLine();
				} else {
					if (!_stricmp (buffer, "OFF")) {
						return m_off;
					}
					return m_value;
				}
			}
			return m_end;
		}

		char* SkipLine() const
		{
			char tmp[1024];
			return fgets (tmp, sizeof (tmp), m_file);
		}

		dgInt32 GetInteger() const
		{
			char buffer[1024];
			GetToken(buffer);
			return atoi (buffer);	
		}

		dgFloat64 GetFloat() const
		{
			char buffer[1024];
			GetToken(buffer);
			return atof (buffer);	
		}

		FILE* m_file;
	};

	Init();
	FILE* const file = fopen (fileName, "rb");
	if (file) {
		ParceOFF parcel (file);

		dgInt32 vertexCount = 0;
		dgInt32 faceCount = 0;
//		dgInt32 edgeCount = 0;

		char buffer[1024];
		bool stillData = true;
		while (stillData) {
			ParceOFF::Token token = parcel.GetToken(buffer);
			switch (token) 
			{
				case ParceOFF::m_off:
				{
					vertexCount = parcel.GetInteger();
					faceCount = parcel.GetInteger();
//					edgeCount = parcel.GetInteger();
					parcel.SkipLine();

					dgVertexAtribute attribute;
					memset (&attribute, 0, sizeof (dgVertexAtribute));
					attribute.m_normal_y = 1.0f;
					//AddAtribute(attribute);
					for (dgInt32 i = 0; i < vertexCount; i ++) {
						//dgBigVector point;
						attribute.m_vertex.m_x = parcel.GetFloat();
						attribute.m_vertex.m_y = parcel.GetFloat();
						attribute.m_vertex.m_z = parcel.GetFloat();
						attribute.m_vertex.m_w = 0.0;
						parcel.SkipLine();
						//AddVertex(point);
						AddPoint(&attribute.m_vertex.m_x, 0);
					}

					BeginFace();
					for (dgInt32 i = 0; i < faceCount; i ++) {
						dgInt32 face[256];
						dgInt64 attrib[256];
						dgInt32 faceVertexCount = parcel.GetInteger();
						for (dgInt32 j = 0; j < faceVertexCount; j ++) {
							face[j] = parcel.GetInteger();
							attrib[j] = face[j];
						}
						parcel.SkipLine();
						AddFace(faceVertexCount, face, attrib);
					}
					EndFace();

					CalculateNormals (3.1416f * 30.0f / 180.0f);
					stillData = false;
					break;
				}
					
				default:;
			}
		}

		fclose (file);
	}
}

dgMeshEffect::dgMeshEffect (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData)
	:dgPolyhedra (allocator) 
{
	dgInt32 faceCount;
	deserialization (userData, &faceCount, sizeof (dgInt32));
	deserialization (userData, &m_pointCount, sizeof (dgInt32));
	deserialization (userData, &m_atribCount, sizeof (dgInt32));
	deserialization (userData, &m_atribCount, sizeof (dgInt32));

	m_maxPointCount = m_pointCount;
	m_maxAtribCount = m_atribCount;

	m_points = (dgBigVector*) GetAllocator()->MallocLow(dgInt32 (m_pointCount * sizeof(dgBigVector)));
	m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_atribCount * sizeof(dgVertexAtribute)));

	deserialization (userData, m_points, m_pointCount * sizeof (dgBigVector));
	deserialization (userData, m_attrib, m_atribCount * sizeof (dgVertexAtribute));

	BeginFace();
	for (dgInt32 i = 0; i < faceCount; i ++) {
		dgInt32 vertexCount;
		dgInt32 face[1024];
		dgInt64 attrib[1024];
		deserialization (userData, &vertexCount, sizeof (dgInt32));
		deserialization (userData, face, vertexCount * sizeof (dgInt32));
		deserialization (userData, attrib, vertexCount * sizeof (dgInt64));
		AddFace (vertexCount, face, attrib);
	}
	EndFace();
}


dgMeshEffect::~dgMeshEffect(void)
{
	GetAllocator()->FreeLow (m_points);
	GetAllocator()->FreeLow (m_attrib);
}


void dgMeshEffect::BeginFace()
{
	dgPolyhedra::BeginFace();
}

void dgMeshEffect::EndFace ()
{
	dgPolyhedra::EndFace();

	for (bool hasVertexCollision = true; hasVertexCollision;) {
		hasVertexCollision = false;

		const dgInt32 currentCount = m_pointCount;
		dgStack<dgInt8> verterCollision (currentCount);
		memset (&verterCollision[0], 0, verterCollision.GetSizeInBytes());

		Iterator iter (*this);
		dgInt32 mark = IncLRU();
		dgList<dgTreeNode*> collisionFound(GetAllocator());
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				if ((edge->m_incidentVertex < currentCount) && (verterCollision[edge->m_incidentVertex] == 0)) {
					verterCollision[edge->m_incidentVertex] = 1;
				} else {
					hasVertexCollision = true;
					collisionFound.Append(iter.GetNode());
				}
				dgEdge* ptr = edge;
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}

		if (hasVertexCollision) {
			dgAssert (Sanity());
			for (dgList<dgTreeNode*>::dgListNode* node = collisionFound.GetFirst(); node; node = node->GetNext()) {
				dgEdge* const edge = &node->GetInfo()->GetInfo();

				// this is a vertex collision
				dgBigVector point (m_points[edge->m_incidentVertex]);
				point.m_w += dgFloat64 (1.0f);
				AddVertex (point);

				dgEdge* ptr = edge;
				do {
					ptr->m_incidentVertex = m_pointCount - 1;

					dgTreeNode* const edgeNode = GetNodeFromInfo (*ptr);
					dgPairKey edgeKey (ptr->m_incidentVertex, ptr->m_twin->m_incidentVertex);
					ReplaceKey (edgeNode, edgeKey.GetVal());

					dgTreeNode* const twinNode = GetNodeFromInfo (*(ptr->m_twin));
					dgPairKey twinKey (ptr->m_twin->m_incidentVertex, ptr->m_incidentVertex);
					ReplaceKey (twinNode, twinKey.GetVal());

					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
			dgAssert (Sanity());
		}
	}
}


void dgMeshEffect::Init()
{
	m_pointCount = 0;
	m_atribCount = 0;
	m_maxPointCount = DG_MESH_EFFECT_INITIAL_VERTEX_SIZE;
	m_maxAtribCount = DG_MESH_EFFECT_INITIAL_VERTEX_SIZE;

	m_points = (dgBigVector*) GetAllocator()->MallocLow(dgInt32 (m_maxPointCount * sizeof(dgBigVector)));
	m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
}

void dgMeshEffect::Trace () const
{
	for (dgInt32 i = 0; i < m_pointCount; i ++ ) {
		dgTrace (("%d-> %f %f %f\n", i, m_points[i].m_x, m_points[i].m_y, m_points[i].m_z));
	}


	dgTree<dgEdge*, dgEdge*>filter(GetAllocator());
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (!filter.Find(edge)) {
			dgEdge* ptr = edge;
			do {
				filter.Insert(edge, ptr);
				dgTrace (("%d ", ptr->m_incidentVertex));
				ptr = ptr->m_next;
			} while (ptr != edge);
			if (edge->m_incidentFace <= 0) {
				dgTrace (("open"));
			}
			dgTrace (("\n"));
		}
	}
	dgTrace (("\n"));
};


void dgMeshEffect::SaveOFF (const char* const fileName) const
{
	FILE* const file = fopen (fileName, "wb");

	fprintf (file, "OFF\n");

	dgInt32 faceCount = 0;
	dgTree<dgEdge*, dgEdge*>filter(GetAllocator());
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const face = &iter.GetNode()->GetInfo();
		if (!filter.Find(face) && (face->m_incidentFace > 0)) {
			faceCount ++;
			dgEdge* edge = face; 
			do {
				filter.Insert(edge, edge);
				edge = edge->m_next;
			} while (edge != face);
		}
	}
	fprintf (file, "%d %d 0\n", m_pointCount, faceCount);

	for (dgInt32 i = 0; i < m_pointCount; i ++) {
		fprintf (file, "%f %f %f\n", m_points[i].m_x, m_points[i].m_y, m_points[i].m_z);
	}

	filter.RemoveAll();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const face = &iter.GetNode()->GetInfo();
		if (!filter.Find(face) && (face->m_incidentFace > 0)) {
			dgInt32 indices[1024];
			dgInt32 vertexCount = 0;
			dgEdge* edge = face; 
			do {
				indices[vertexCount] = edge->m_incidentVertex;
				vertexCount ++;
				filter.Insert(edge, edge);
				edge = edge->m_next;
			} while (edge != face);

			fprintf (file, "%d", vertexCount);
			for (dgInt32 j = 0; j < vertexCount; j ++) {
				fprintf (file, " %d", indices[j]);
			}
			fprintf (file, "\n");
		}
	}
	fclose (file);
}

void dgMeshEffect::Triangulate  ()
{
	dgPolyhedra polygon(GetAllocator());

	dgInt32 mark = IncLRU();
	polygon.BeginFace();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);

		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32	index[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			dgInt32 indexCount = 0;
			do {
				dgInt32 attribIndex = dgInt32 (ptr->m_userData);
				m_attrib[attribIndex].m_vertex.m_w = dgFloat64 (ptr->m_incidentVertex);
				ptr->m_mark = mark;
				index[indexCount] = attribIndex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, index);
		}
	}
	polygon.EndFace();


	dgPolyhedra leftOversOut(GetAllocator());
	polygon.Triangulate(&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &leftOversOut);
	dgAssert (leftOversOut.GetCount() == 0);


	RemoveAll();
	SetLRU (0);

	mark = polygon.IncLRU();
	BeginFace();
	dgPolyhedra::Iterator iter1 (polygon);
	for (iter1.Begin(); iter1; iter1 ++){
		dgEdge* const face = &(*iter1);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
			dgInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			dgInt32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = dgInt32 (m_attrib[ptr->m_incidentVertex].m_vertex.m_w);

				userData[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();

	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) {
			dgInt32 attribIndex = dgInt32 (face->m_userData);
			m_attrib[attribIndex].m_vertex.m_w = m_points[face->m_incidentVertex].m_w;
		}
	}

	RepairTJoints ();
	dgAssert (Sanity ());
}

void dgMeshEffect::ConvertToPolygons ()
{
	dgPolyhedra polygon(GetAllocator());

	dgInt32 mark = IncLRU();
	polygon.BeginFace();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);

		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32	index[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			dgInt32 indexCount = 0;
			do {
				dgInt32 attribIndex = dgInt32 (ptr->m_userData);

				m_attrib[attribIndex].m_vertex.m_w = dgFloat32 (ptr->m_incidentVertex);
				ptr->m_mark = mark;
				index[indexCount] = attribIndex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, index);
		}
	}
	polygon.EndFace();

	dgPolyhedra leftOversOut(GetAllocator());
	polygon.ConvexPartition (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &leftOversOut);
	dgAssert (leftOversOut.GetCount() == 0);

	RemoveAll();
	SetLRU (0);

	mark = polygon.IncLRU();
	BeginFace();
	dgPolyhedra::Iterator iter1 (polygon);
	for (iter1.Begin(); iter1; iter1 ++){
		dgEdge* const face = &(*iter1);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
			dgInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			dgInt32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = dgInt32 (m_attrib[ptr->m_incidentVertex].m_vertex.m_w);
				userData[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();


	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) {
			dgInt32 attribIndex = dgInt32 (face->m_userData);
			m_attrib[attribIndex].m_vertex.m_w = m_points[face->m_incidentVertex].m_w;
		}
	}

	RepairTJoints ();
	dgAssert (Sanity ());
}

void dgMeshEffect::RemoveUnusedVertices(dgInt32* const vertexMapResult)
{
	dgPolyhedra polygon(GetAllocator());
	dgStack<dgInt32>attrbMap(m_atribCount);
	dgStack<dgInt32>vertexMap(m_pointCount);

	dgInt32 savedPointCount = m_pointCount;
	memset(&vertexMap[0], -1, m_pointCount * sizeof (int));
	memset(&attrbMap[0], -1, m_atribCount * sizeof (int));

	int attribCount = 0;
	int vertexCount = 0;

	dgStack<dgBigVector>points (m_pointCount);
	dgStack<dgVertexAtribute>atributes (m_atribCount);

	dgInt32 mark = IncLRU();
	polygon.BeginFace();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32	vertex[DG_MESH_EFFECT_POINT_SPLITED];
			dgInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];
			int indexCount = 0;
			dgEdge* ptr = face;
			do {
				ptr->m_mark = mark;

				int index = ptr->m_incidentVertex;
				if (vertexMap[index] == -1) {
					vertexMap[index] = vertexCount;
					points[vertexCount] = m_points[index];
					vertexCount ++;
				}
				vertex[indexCount] = vertexMap[index];

				index = int (ptr->m_userData);
				if (attrbMap[index] == -1) {
					attrbMap[index] = attribCount;
					atributes[attribCount] = m_attrib[index];
					attribCount ++;
				}
				userData[indexCount] = attrbMap[index];
				indexCount ++;

				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, vertex, userData);
		}
	}
	polygon.EndFace();

	m_pointCount = vertexCount;
	memcpy (&m_points[0].m_x, &points[0].m_x, m_pointCount * sizeof (dgBigVector));
	 
	m_atribCount = attribCount;
	memcpy (&m_attrib[0].m_vertex.m_x, &atributes[0].m_vertex.m_x, m_atribCount * sizeof (dgVertexAtribute));


	RemoveAll();
	SetLRU (0);

	BeginFace();
	dgPolyhedra::Iterator iter1 (polygon);
	for (iter1.Begin(); iter1; iter1 ++){
		dgEdge* const face = &(*iter1);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
			dgInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			dgInt32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = ptr->m_incidentVertex;
				userData[indexCount] = dgInt64 (ptr->m_userData);
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();
	PackVertexArrays ();

	if (vertexMapResult) {
		memcpy (vertexMapResult, &vertexMap[0], savedPointCount * sizeof (dgInt32));
	}
}


void dgMeshEffect::ApplyTransform (const dgMatrix& matrix)
{
	matrix.TransformTriplex(&m_points[0].m_x, sizeof (dgBigVector), &m_points[0].m_x, sizeof (dgBigVector), m_pointCount);
	matrix.TransformTriplex(&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), m_atribCount);

	dgMatrix rotation ((matrix.Inverse4x4()).Transpose4X4());
	for (dgInt32 i = 0; i < m_atribCount; i ++) {
		dgVector n (dgFloat32 (m_attrib[i].m_normal_x), dgFloat32 (m_attrib[i].m_normal_y), dgFloat32 (m_attrib[i].m_normal_z), dgFloat32 (0.0f));
		n = rotation.RotateVector(n);
		dgAssert ((n % n) > dgFloat32 (0.0f));
		n = n.Scale3 (dgRsqrt (n % n));
		m_attrib[i].m_normal_x = n.m_x;
		m_attrib[i].m_normal_y = n.m_y;
		m_attrib[i].m_normal_z = n.m_z;
	}
}

dgMatrix dgMeshEffect::CalculateOOBB (dgBigVector& size) const
{
	dgObb sphere (CalculateSphere (&m_points[0].m_x, sizeof (dgBigVector), NULL));
	size = sphere.m_size;
	size.m_w = 0.0f;

//	dgMatrix permuation (dgGetIdentityMatrix());
//	permuation[0][0] = dgFloat32 (0.0f);
//	permuation[0][1] = dgFloat32 (1.0f);
//	permuation[1][1] = dgFloat32 (0.0f);
//	permuation[1][2] = dgFloat32 (1.0f);
//	permuation[2][2] = dgFloat32 (0.0f);
//	permuation[2][0] = dgFloat32 (1.0f);
//	while ((size.m_x < size.m_y) || (size.m_x < size.m_z)) {
//		sphere = permuation * sphere;
//		size = permuation.UnrotateVector(size);
//	}

	return sphere;
}

void dgMeshEffect::CalculateAABB (dgBigVector& minBox, dgBigVector& maxBox) const
{
	dgBigVector minP ( dgFloat64 (1.0e15f),  dgFloat64 (1.0e15f),  dgFloat64 (1.0e15f), dgFloat64 (0.0f)); 
	dgBigVector maxP (-dgFloat64 (1.0e15f), -dgFloat64 (1.0e15f), -dgFloat64 (1.0e15f), dgFloat64 (0.0f)); 

	dgPolyhedra::Iterator iter (*this);
	const dgBigVector* const points = &m_points[0];
	for (iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		const dgBigVector& p (points[edge->m_incidentVertex]);

		minP.m_x = dgMin (p.m_x, minP.m_x); 
		minP.m_y = dgMin (p.m_y, minP.m_y); 
		minP.m_z = dgMin (p.m_z, minP.m_z); 

		maxP.m_x = dgMax (p.m_x, maxP.m_x); 
		maxP.m_y = dgMax (p.m_y, maxP.m_y); 
		maxP.m_z = dgMax (p.m_z, maxP.m_z); 
	}

	minBox = minP;
	maxBox = maxP;
}


void dgMeshEffect::BeginPolygon ()
{
	m_pointCount = 0;
	m_atribCount = 0;
	RemoveAll();
	BeginFace();
}


void dgMeshEffect::AddAtribute (const dgVertexAtribute& attib)
{
	if (m_atribCount >= m_maxAtribCount) {
		m_maxAtribCount *= 2;
		dgVertexAtribute* const attibArray = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
		memcpy (attibArray, m_attrib, m_atribCount * sizeof(dgVertexAtribute));
		GetAllocator()->FreeLow(m_attrib);
		m_attrib = attibArray;
	}

	m_attrib[m_atribCount] = attib;

	dgBigVector n (attib.m_normal_x, attib.m_normal_y, attib.m_normal_z, dgFloat64 (0.0f));
	dgFloat64 mag2 = n % n ; 
	if (mag2 < dgFloat64 (1.0e-16f)) {
		n.m_x = dgFloat64 (0.0f);
		n.m_y = dgFloat64 (1.0f);
		n.m_z = dgFloat64 (0.0f);
	}
	m_attrib[m_atribCount].m_normal_x = n.m_x;
	m_attrib[m_atribCount].m_normal_y = n.m_y;
	m_attrib[m_atribCount].m_normal_z = n.m_z;

	m_attrib[m_atribCount].m_vertex.m_x = QuantizeCordinade(m_attrib[m_atribCount].m_vertex.m_x);
	m_attrib[m_atribCount].m_vertex.m_y = QuantizeCordinade(m_attrib[m_atribCount].m_vertex.m_y);
	m_attrib[m_atribCount].m_vertex.m_z = QuantizeCordinade(m_attrib[m_atribCount].m_vertex.m_z);
	m_atribCount ++;
}

void dgMeshEffect::AddVertex(const dgBigVector& vertex)
{
	if (m_pointCount >= m_maxPointCount) {
		m_maxPointCount *= 2;
		dgBigVector* const points = (dgBigVector*) GetAllocator()->MallocLow(dgInt32 (m_maxPointCount * sizeof(dgBigVector)));
		memcpy (points, m_points, m_pointCount * sizeof(dgBigVector));
		GetAllocator()->FreeLow(m_points);
		m_points = points;
	}
	
	m_points[m_pointCount].m_x = QuantizeCordinade(vertex[0]);
	m_points[m_pointCount].m_y = QuantizeCordinade(vertex[1]);
	m_points[m_pointCount].m_z = QuantizeCordinade(vertex[2]);
	m_points[m_pointCount].m_w = vertex.m_w;
	m_pointCount ++;
}


void dgMeshEffect::AddPoint(const dgFloat64* vertex, dgInt32 material)
{
	dgVertexAtribute attib;
	AddVertex(dgBigVector (vertex[0], vertex[1], vertex[2], vertex[3]));
	
	attib.m_vertex.m_x = m_points[m_pointCount - 1].m_x;
	attib.m_vertex.m_y = m_points[m_pointCount - 1].m_y;
	attib.m_vertex.m_z = m_points[m_pointCount - 1].m_z;
	attib.m_vertex.m_w = m_points[m_pointCount - 1].m_w;

	attib.m_normal_x = vertex[4];
	attib.m_normal_y = vertex[5];
	attib.m_normal_z = vertex[6];
	attib.m_u0 = vertex[7];
	attib.m_v0 = vertex[8];
	attib.m_u1 = vertex[9];
	attib.m_v1 = vertex[10];
	attib.m_material = material;

	AddAtribute (attib);
}

void dgMeshEffect::PackVertexArrays ()
{
	if (m_maxPointCount > m_pointCount) {
		dgBigVector* const points = (dgBigVector*) GetAllocator()->MallocLow(dgInt32 (m_pointCount * sizeof(dgBigVector)));
		memcpy (points, m_points, m_pointCount * sizeof(dgBigVector));
		GetAllocator()->FreeLow(m_points);
		m_points = points;
		m_maxPointCount = m_pointCount;
	}


	if (m_maxAtribCount > m_atribCount) {
		dgVertexAtribute* const attibArray = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_atribCount * sizeof(dgVertexAtribute)));
		memcpy (attibArray, m_attrib, m_atribCount * sizeof(dgVertexAtribute));
		GetAllocator()->FreeLow(m_attrib);
		m_attrib = attibArray;
		m_maxAtribCount = m_atribCount;
	}
};


void dgMeshEffect::AddPolygon (dgInt32 count, const dgFloat64* const vertexList, dgInt32 strideIndBytes, dgInt32 material)
{
	dgAssert (strideIndBytes >= sizeof (dgBigVector));
	dgInt32 stride = dgInt32 (strideIndBytes / sizeof (dgFloat64));
	
	if (count > 3) {
		dgPolyhedra polygon (GetAllocator());
		dgInt32 indexList[256];
		dgAssert (count < dgInt32 (sizeof (indexList)/sizeof(indexList[0])));
		for (dgInt32 i = 0; i < count; i ++) {
			indexList[i] = i;
		}

		polygon.BeginFace();
		polygon.AddFace(count, indexList, NULL);
		polygon.EndFace();
		polygon.Triangulate(vertexList, strideIndBytes, NULL);

		dgInt32 mark = polygon.IncLRU();
		dgPolyhedra::Iterator iter (polygon);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) {
				dgInt32 i0 = edge->m_incidentVertex;
				dgInt32 i1 = edge->m_next->m_incidentVertex;
				dgInt32 i2 = edge->m_next->m_next->m_incidentVertex;
				edge->m_mark = mark;
				edge->m_next->m_mark = mark;
				edge->m_next->m_next->m_mark = mark;

//				#ifdef _DEBUG
//					dgBigVector p0_ (&vertexList[i0 * stride]);
//					dgBigVector p1_ (&vertexList[i1 * stride]);
//					dgBigVector p2_ (&vertexList[i2 * stride]);
//					dgBigVector e1_ (p1_ - p0_);
//					dgBigVector e2_ (p2_ - p0_);
//					dgBigVector n_ (e1_ * e2_);
//					dgFloat64 mag2_ = n_ % n_;
//					dgAssert (mag2_ > dgFloat32 (DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)); 
//				#endif

				AddPoint(vertexList + i0 * stride, material);
				AddPoint(vertexList + i1 * stride, material);
				AddPoint(vertexList + i2 * stride, material);

				#ifdef _DEBUG
					const dgBigVector& p0 = m_points[m_pointCount - 3];
					const dgBigVector& p1 = m_points[m_pointCount - 2];
					const dgBigVector& p2 = m_points[m_pointCount - 1];
					dgBigVector e1 (p1 - p0);
					dgBigVector e2 (p2 - p0);
					dgBigVector n (e1 * e2);
					dgFloat64 mag3 = n % n;
					dgAssert (mag3 > dgFloat64 (DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV));
				#endif
			}
		}

	} else {

		AddPoint(vertexList, material);
		AddPoint(vertexList + stride, material);
		AddPoint(vertexList + stride + stride, material);

		const dgBigVector& p0 = m_points[m_pointCount - 3];
		const dgBigVector& p1 = m_points[m_pointCount - 2];
		const dgBigVector& p2 = m_points[m_pointCount - 1];
		dgBigVector e1 (p1 - p0);
		dgBigVector e2 (p2 - p0);
		dgBigVector n (e1 * e2);
		dgFloat64 mag3 = n % n;
		if (mag3 < dgFloat64 (DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)) {
			m_pointCount -= 3;
			m_atribCount -= 3;
		}
	}
}

#ifndef _NEWTON_USE_DOUBLE

void dgMeshEffect::AddPolygon (dgInt32 count, const dgFloat32* const vertexList, dgInt32 strideIndBytes, dgInt32 material)
{
	dgVertexAtribute points[256];
	dgAssert (count < dgInt32 (sizeof (points)/sizeof (points[0])));

	dgInt32 stride = strideIndBytes / sizeof (dgFloat32);
	if (stride < 4) {
		for (dgInt32 i = 0; i < count; i ++) {
			points[i].m_vertex.m_x = vertexList[i * stride + 0];
			points[i].m_vertex.m_y = vertexList[i * stride + 1];
			points[i].m_vertex.m_z = vertexList[i * stride + 2];
			points[i].m_vertex.m_w = dgFloat64(0.0f);
			points[i].m_normal_x = dgFloat64(0.0f);
			points[i].m_normal_y = dgFloat64(1.0f);
			points[i].m_normal_z = dgFloat64(0.0f);
			points[i].m_u0 = dgFloat64(0.0f);
			points[i].m_v0 = dgFloat64(0.0f);
			points[i].m_u1 = dgFloat64(0.0f);
			points[i].m_v1 = dgFloat64(0.0f);
			points[i].m_material = dgFloat64(material);
		}
	} else {
		for (dgInt32 i = 0; i < count; i ++) {
			points[i].m_vertex.m_x = vertexList[i * stride + 0];
			points[i].m_vertex.m_y = vertexList[i * stride + 1];
			points[i].m_vertex.m_z = vertexList[i * stride + 2];
			points[i].m_vertex.m_w = vertexList[i * stride + 3];
			points[i].m_normal_x = vertexList[i * stride + 4];
			points[i].m_normal_y = vertexList[i * stride + 5];
			points[i].m_normal_z = vertexList[i * stride + 6];
			points[i].m_u0 = vertexList[i * stride + 7];
			points[i].m_v0 = vertexList[i * stride + 8];
			points[i].m_u1 = vertexList[i * stride + 9];
			points[i].m_v1 = vertexList[i * stride + 10];
			points[i].m_material = dgFloat64(material);
		}
	}
	AddPolygon (count, &points[0].m_vertex.m_x, sizeof (dgVertexAtribute), material);
}
#endif

void dgMeshEffect::EndPolygon (dgFloat64 tol, bool fixTjoint)
{
	dgStack<dgInt32>indexMap(m_pointCount);
	dgStack<dgInt32>attrIndexMap(m_atribCount);

#ifdef _DEBUG
	for (dgInt32 i = 0; i < m_pointCount; i += 3) {
		dgBigVector p0 (m_points[i + 0]);
		dgBigVector p1 (m_points[i + 1]);
		dgBigVector p2 (m_points[i + 2]);
		dgBigVector e1 (p1 - p0);
		dgBigVector e2 (p2 - p0);
		dgBigVector n (e1 * e2);
		dgFloat64 mag2 = n % n;
		dgAssert (mag2 > dgFloat32 (0.0f));
	}
#endif

	dgInt32 triangCount = m_pointCount / 3;
	m_pointCount = dgVertexListToIndexList (&m_points[0].m_x, sizeof (dgBigVector), sizeof (dgBigVector)/sizeof (dgFloat64), m_pointCount, &indexMap[0], tol);
	m_atribCount = dgVertexListToIndexList (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute)/sizeof (dgFloat64), m_atribCount, &attrIndexMap[0], tol);

	for (dgInt32 i = 0; i < triangCount; i ++) {
		dgInt32 index[3];
		dgInt64 userdata[3];

		index[0] = indexMap[i * 3 + 0];
		index[1] = indexMap[i * 3 + 1];
		index[2] = indexMap[i * 3 + 2];


		dgBigVector e1 (m_points[index[1]] - m_points[index[0]]);
		dgBigVector e2 (m_points[index[2]] - m_points[index[0]]);

		dgBigVector n (e1 * e2);
		dgFloat64 mag2 = n % n;
		if (mag2 > dgFloat64 (1.0e-12f)) {
			userdata[0] = attrIndexMap[i * 3 + 0];
			userdata[1] = attrIndexMap[i * 3 + 1];
			userdata[2] = attrIndexMap[i * 3 + 2];
			dgEdge* const edge = AddFace (3, index, userdata);
			if (!edge) {
				dgAssert ((m_pointCount + 3) <= m_maxPointCount);

				m_points[m_pointCount + 0] = m_points[index[0]];
				m_points[m_pointCount + 1] = m_points[index[1]];
				m_points[m_pointCount + 2] = m_points[index[2]];

				index[0] = m_pointCount + 0;
				index[1] = m_pointCount + 1;
				index[2] = m_pointCount + 2;

				m_pointCount += 3;

				#ifdef _DEBUG
					dgEdge* test = AddFace (3, index, userdata);
					dgAssert (test);
				#else 
					AddFace (3, index, userdata);
				#endif
			}
		}
	}
	EndFace();

	if (fixTjoint) {
		RepairTJoints ();
	}

#ifdef _DEBUG
	dgPolyhedra::Iterator iter (*this);	
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) {
			dgBigVector p0 (m_points[face->m_incidentVertex]);
			dgBigVector p1 (m_points[face->m_next->m_incidentVertex]);
			dgBigVector p2 (m_points[face->m_next->m_next->m_incidentVertex]);
			dgBigVector e1 (p1 - p0);
			dgBigVector e2 (p2 - p0);
			dgBigVector n (e1 * e2);
			dgFloat64 mag2 = n % n;
			dgAssert (mag2 >= dgFloat32 (0.0f));
		}
	}
#endif
}


void dgMeshEffect::BuildFromVertexListIndexList(
	dgInt32 faceCount, const dgInt32* const faceIndexCount, const dgInt32* const faceMaterialIndex, 
	const dgFloat32* const vertex, dgInt32 vertexStrideInBytes, const dgInt32* const vertexIndex,
	const dgFloat32* const normal, dgInt32  normalStrideInBytes, const dgInt32* const normalIndex,
	const dgFloat32* const uv0, dgInt32  uv0StrideInBytes, const dgInt32* const uv0Index,
	const dgFloat32* const uv1, dgInt32  uv1StrideInBytes, const dgInt32* const uv1Index)
{
	BeginPolygon ();

	// calculate vertex Count
	dgInt32 acc = 0;
	dgInt32 vertexCount = 0;
	for (dgInt32 j = 0; j < faceCount; j ++) {
		dgInt32 count = faceIndexCount[j];
		for (dgInt32 i = 0; i < count; i ++) {
			vertexCount = dgMax(vertexCount, vertexIndex[acc + i] + 1);
		}
		acc += count;
	}

	dgInt32 layerCountBase = 0;
	dgInt32 vertexStride = dgInt32 (vertexStrideInBytes / sizeof (dgFloat32));
	for (dgInt32 i = 0; i < vertexCount; i ++) {
		dgInt32 index = i * vertexStride;
		dgBigVector v (vertex[index + 0], vertex[index + 1], vertex[index + 2], vertex[index + 3]);
		AddVertex (v);
		layerCountBase += (vertex[index + 3]) > dgFloat32(layerCountBase);
	}


	dgInt32 maxAttribCount = 0;
	for (dgInt32 j = 0; j < faceCount; j ++) {
		maxAttribCount += faceIndexCount[j];
	}
	dgStack<dgInt32>attrIndexMap(maxAttribCount);

	acc = 0;
	dgInt32 currentCount = 0;
	dgInt32 attributeCount = 0;
	dgInt32 attributeCountMarker = 0;
	dgInt32 normalStride = dgInt32 (normalStrideInBytes / sizeof (dgFloat32));
	dgInt32 uv0Stride = dgInt32 (uv0StrideInBytes / sizeof (dgFloat32));
	dgInt32 uv1Stride = dgInt32 (uv1StrideInBytes / sizeof (dgFloat32));
	for (dgInt32 j = 0; j < faceCount; j ++) {
		dgInt32 indexCount = faceIndexCount[j];
		dgInt32 materialIndex = faceMaterialIndex[j];
		for (dgInt32 i = 0; i < indexCount; i ++) {
			dgVertexAtribute point;
			dgInt32 index = vertexIndex[acc + i];
			point.m_vertex = m_points[index];
			
			index = normalIndex[(acc + i)] * normalStride;
			point.m_normal_x =  normal[index + 0];
			point.m_normal_y =  normal[index + 1];
			point.m_normal_z =  normal[index + 2];

			index = uv0Index[(acc + i)] * uv0Stride;
			point.m_u0 = uv0[index + 0];
			point.m_v0 = uv0[index + 1];
			
			index = uv1Index[(acc + i)] * uv1Stride;
			point.m_u1 = uv1[index + 0];
			point.m_v1 = uv1[index + 1];

			point.m_material = materialIndex;
			AddAtribute(point);

			attrIndexMap[attributeCount] = attributeCount;
			attributeCount ++;
		}

		acc += indexCount;
		if (attributeCount >= (attributeCountMarker + 1024 * 256)) {
			dgInt32 count = attributeCount - attributeCountMarker;
			dgInt32 newCount = dgVertexListToIndexList (&m_attrib[currentCount].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof (dgFloat64), count, &attrIndexMap[attributeCountMarker], DG_VERTEXLIST_INDEXLIST_TOL);
			for (dgInt32 i = 0; i < count; i ++) {
				attrIndexMap[attributeCountMarker + i] += currentCount;
			}
			currentCount += newCount;
			m_atribCount = currentCount;
			attributeCountMarker = attributeCount;
		}
	}


	if (attributeCountMarker) {
		dgInt32 count = attributeCount - attributeCountMarker;
		dgInt32 newCount = dgVertexListToIndexList (&m_attrib[currentCount].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof (dgFloat64), count, &attrIndexMap[attributeCountMarker], DG_VERTEXLIST_INDEXLIST_TOL);
		for (dgInt32 i = 0; i < count; i ++) {
			attrIndexMap[attributeCountMarker + i] += currentCount;
		}
		currentCount += newCount;
		m_atribCount = currentCount;
		attributeCountMarker = attributeCount;


		dgStack<dgInt32>indirectAttrIndexMap(m_atribCount);
		m_atribCount = dgVertexListToIndexList (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof (dgFloat64), m_atribCount, &indirectAttrIndexMap[0], DG_VERTEXLIST_INDEXLIST_TOL);

		for (dgInt32 i = 0; i < maxAttribCount; i ++) {
			dgInt32 j = attrIndexMap[i];
			attrIndexMap[i] = indirectAttrIndexMap[j];
		}

	} else {
		m_atribCount = dgVertexListToIndexList (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof (dgFloat64), m_atribCount, &attrIndexMap[0], DG_VERTEXLIST_INDEXLIST_TOL);
	}


	bool hasFaces = true;
	dgStack<dgInt8> faceMark (faceCount);
	memset (&faceMark[0], 1, size_t (faceMark.GetSizeInBytes()));
	
	dgInt32 layerCount = 0;
	while (hasFaces) {
		acc = 0;
		hasFaces = false;
		dgInt32 vertexBank = layerCount * vertexCount;
		for (dgInt32 j = 0; j < faceCount; j ++) {
			int indexCount = faceIndexCount[j];
			if (indexCount > 0) {
				dgInt32 index[256];
				dgInt64 userdata[256];
				dgAssert (indexCount >= 3);
				dgAssert (indexCount < dgInt32 (sizeof (index) / sizeof (index[0])));

				if (faceMark[j]) {
					for (int i = 0; i < indexCount; i ++) {
						index[i] = vertexIndex[acc + i] + vertexBank;
						userdata[i] = attrIndexMap[acc + i];
					}
					dgEdge* const edge = AddFace (indexCount, index, userdata);
					if (edge) {
						faceMark[j] = 0;
					} else {
						// check if the face is not degenerated
						bool degeneratedFace = false;
						for (int i = 0; i < indexCount - 1; i ++) {
							for (int k = i + 1; k < indexCount; k ++) {
								if (index[i] == index[k]) {
									degeneratedFace = true;		
								}
							}
						}
						if (degeneratedFace) {
							faceMark[j] = 0;
						} else {
							hasFaces = true;
						}
					}
				}
				acc += indexCount;
			}
		}
		if (hasFaces) {
			layerCount ++;
			for (int i = 0; i < vertexCount; i ++) {
				int index = i * vertexStride;
				AddVertex (dgBigVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], dgFloat64 (layerCount + layerCountBase)));
			}
		}
	}

	EndFace();
	PackVertexArrays ();
}


dgInt32 dgMeshEffect::GetTotalFaceCount() const
{
	return GetFaceCount();
}

dgInt32 dgMeshEffect::GetTotalIndexCount() const
{
	Iterator iter (*this);
	dgInt32 count = 0;
	dgInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}
		
		dgEdge* ptr = edge;
		do {
			count ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}

void dgMeshEffect::GetFaces (dgInt32* const facesIndex, dgInt32* const materials, void** const faceNodeList) const
{
	Iterator iter (*this);

	dgInt32 faces = 0;
	dgInt32 indexCount = 0;
	dgInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		dgInt32 faceCount = 0;
		dgEdge* ptr = edge;
		do {
//			indexList[indexCount] = dgInt32 (ptr->m_userData);
			faceNodeList[indexCount] = GetNodeFromInfo (*ptr);
			indexCount ++;
			faceCount ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);

		facesIndex[faces] = faceCount;
		materials[faces] = dgFastInt(m_attrib[dgInt32 (edge->m_userData)].m_material);
		faces ++;
	}
}

void* dgMeshEffect::GetFirstVertex () const
{
	Iterator iter (*this);
	iter.Begin();

	dgTreeNode* node = NULL;
	if (iter) {
		dgInt32 mark = IncLRU();
		node = iter.GetNode();

		dgEdge* const edge = &node->GetInfo();
		dgEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);
	}
	return node; 
}

void* dgMeshEffect::GetNextVertex (const void* const vertex) const
{
	dgTreeNode* node = (dgTreeNode*) vertex;
	dgInt32 mark = node->GetInfo().m_mark;

	Iterator iter (*this);
	iter.Set (node);
	for (iter ++; iter; iter ++) {
		dgTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			dgEdge* const edge = &node->GetInfo();
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			return node; 
		}
	}
	return NULL; 
}

dgInt32 dgMeshEffect::GetVertexIndex (const void* const vertex) const
{
	dgTreeNode* const node = (dgTreeNode*) vertex;
	dgEdge* const edge = &node->GetInfo();
	return edge->m_incidentVertex;
}


void* dgMeshEffect::GetFirstPoint () const
{
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgTreeNode* const node = iter.GetNode();
		dgEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node;
		}
	}
	return NULL; 
}

void* dgMeshEffect::GetNextPoint (const void* const point) const
{
	Iterator iter (*this);
	iter.Set ((dgTreeNode*) point);
	for (iter ++; iter; iter ++) {
		dgTreeNode* const node = iter.GetNode();
		dgEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node; 
		}
	}
	return NULL; 
}

dgInt32 dgMeshEffect::GetPointIndex (const void* const point) const
{
	dgTreeNode* const node = (dgTreeNode*) point;
	dgEdge* const edge = &node->GetInfo();
	return int (edge->m_userData);
}

dgInt32 dgMeshEffect::GetVertexIndexFromPoint (const void* const point) const
{
	return GetVertexIndex (point);
}


dgEdge* dgMeshEffect::SpliteFace (dgInt32 v0, dgInt32 v1)
{
	if (!FindEdge(v0, v1)) {
		dgPolyhedra::dgPairKey key (v0, 0);
		dgTreeNode* const node = FindGreaterEqual(key.GetVal());
		if (node) {
			dgEdge* const edge = &node->GetInfo();
			dgEdge* edge0 = edge;
			do {
				if (edge0->m_incidentFace > 0) {
					for (dgEdge* edge1 = edge0->m_next->m_next; edge1 != edge0->m_prev; edge1 = edge1->m_next) {
						if (edge1->m_incidentVertex == v1) {
							return ConnectVertex (edge0, edge1);
						}
					};
				}
				edge0 = edge0->m_twin->m_next;
			} while (edge0 != edge);
		}
	}
	return NULL;
}


void* dgMeshEffect::GetFirstEdge () const
{
	Iterator iter (*this);
	iter.Begin();

	dgTreeNode* node = NULL;
	if (iter) {
		dgInt32 mark = IncLRU();

		node = iter.GetNode();

		dgEdge* const edge = &node->GetInfo();
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}
	return node; 
}

void* dgMeshEffect::GetNextEdge (const void* const edge) const
{
	dgTreeNode* node = (dgTreeNode*) edge;
	dgInt32 mark = node->GetInfo().m_mark;

	Iterator iter (*this);
	iter.Set (node);
	for (iter ++; iter; iter ++) {
		dgTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			node->GetInfo().m_mark = mark;
			node->GetInfo().m_twin->m_mark = mark;
			return node; 
		}
	}
	return NULL; 
}

void dgMeshEffect::GetEdgeIndex (const void* const edge, dgInt32& v0, dgInt32& v1) const
{
	dgTreeNode* node = (dgTreeNode*) edge;
	v0 = node->GetInfo().m_incidentVertex;
	v1 = node->GetInfo().m_twin->m_incidentVertex;
}

//void dgMeshEffect::GetEdgeAttributeIndex (const void* edge, dgInt32& v0, dgInt32& v1) const
//{
//	dgTreeNode* node = (dgTreeNode*) edge;
//	v0 = int (node->GetInfo().m_userData);
//	v1 = int (node->GetInfo().m_twin->m_userData);
//}


void* dgMeshEffect::GetFirstFace () const
{
	Iterator iter (*this);
	iter.Begin();

	dgTreeNode* node = NULL;
	if (iter) {
		dgInt32 mark = IncLRU();
		node = iter.GetNode();

		dgEdge* const edge = &node->GetInfo();
		dgEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}

	return node;
}

void* dgMeshEffect::GetNextFace (const void* const face) const
{
	dgTreeNode* node = (dgTreeNode*) face;
	dgInt32 mark = node->GetInfo().m_mark;

	Iterator iter (*this);
	iter.Set (node);
	for (iter ++; iter; iter ++) {
		dgTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			dgEdge* const edge = &node->GetInfo();
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
			return node; 
		}
	}
	return NULL; 
}


dgInt32 dgMeshEffect::IsFaceOpen (const void* const face) const
{
	dgTreeNode* const node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	return (edge->m_incidentFace > 0) ? 0 : 1;
}

dgInt32 dgMeshEffect::GetFaceMaterial (const void* const face) const
{
	dgTreeNode* const node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	return dgInt32 (m_attrib[edge->m_userData].m_material);
}

void dgMeshEffect::SetFaceMaterial (const void* const face, int mateialID) const
{
	dgTreeNode* const node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	if (edge->m_incidentFace > 0) {
		dgEdge* ptr = edge;
		do {
			dgVertexAtribute* const attrib = &m_attrib[ptr->m_userData];
			attrib->m_material = dgFloat64 (mateialID);
			ptr = ptr->m_next;
		} while (ptr != edge) ;
	}
}


dgInt32 dgMeshEffect::GetFaceIndexCount (const void* const face) const
{
	int count = 0;
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	dgEdge* ptr = edge;
	do {
		count ++;
		ptr = ptr->m_next;
	} while (ptr != edge);
	return count; 
}

void dgMeshEffect::GetFaceIndex (const void* const face, dgInt32* const indices) const
{
	int count = 0;
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	dgEdge* ptr = edge;
	do {
		indices[count] =  ptr->m_incidentVertex;
		count ++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}

void dgMeshEffect::GetFaceAttributeIndex (const void* const face, dgInt32* const indices) const
{
	int count = 0;
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	dgEdge* ptr = edge;
	do {
		indices[count] = int (ptr->m_userData);
		count ++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}


dgBigVector dgMeshEffect::CalculateFaceNormal (const void* const face) const
{
	dgTreeNode* const node = (dgTreeNode*) face;
	dgEdge* const faceEdge = &node->GetInfo();
	dgBigVector normal (FaceNormal (faceEdge, &m_points[0].m_x, sizeof (m_points[0])));
	normal = normal.Scale3 (1.0f / sqrt (normal % normal));
	return normal;
}

/*
dgInt32 GetTotalFaceCount() const;
{
	dgInt32 mark;
	dgInt32 count;
	dgInt32 materialCount;
	dgInt32 materials[256];
	dgInt32 streamIndexMap[256];
	dgIndexArray* array; 

	count = 0;
	materialCount = 0;

	array = (dgIndexArray*) GetAllocator()->MallocLow (4 * sizeof (dgInt32) * GetCount() + sizeof (dgIndexArray) + 2048);
	array->m_indexList = (dgInt32*)&array[1];

	mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);	
	memset(streamIndexMap, 0, sizeof (streamIndexMap));
	for(iter.Begin(); iter; iter ++){

		dgEdge* const edge;
		edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark)) {
			dgEdge* ptr;
			dgInt32 hashValue;
			dgInt32 index0;
			dgInt32 index1;

			ptr = edge;
			ptr->m_mark = mark;
			index0 = dgInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			index1 = dgInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			do {
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = dgInt32 (ptr->m_userData);
				array->m_indexList[count * 4 + 3] = m_attrib[dgInt32 (edge->m_userData)].m_material;
				index1 = dgInt32 (ptr->m_userData);

				hashValue = array->m_indexList[count * 4 + 3] & 0xff;
				streamIndexMap[hashValue] ++;
				materials[hashValue] = array->m_indexList[count * 4 + 3];
				count ++;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
*/




void dgMeshEffect::GetVertexStreams (dgInt32 vetexStrideInByte, dgFloat32* const vertex, 
									 dgInt32 normalStrideInByte, dgFloat32* const normal, 
									 dgInt32 uvStrideInByte0, dgFloat32* const uv0, 
									 dgInt32 uvStrideInByte1, dgFloat32* const uv1)
{
	uvStrideInByte0 /= sizeof (dgFloat32);
	uvStrideInByte1 /= sizeof (dgFloat32);
	vetexStrideInByte /= sizeof (dgFloat32);
	normalStrideInByte /= sizeof (dgFloat32);
	for (dgInt32 i = 0; i < m_atribCount; i ++)	{
		dgInt32 j = i * vetexStrideInByte;
		vertex[j + 0] = dgFloat32 (m_attrib[i].m_vertex.m_x);
		vertex[j + 1] = dgFloat32 (m_attrib[i].m_vertex.m_y);
		vertex[j + 2] = dgFloat32 (m_attrib[i].m_vertex.m_z);

		j = i * normalStrideInByte;
		normal[j + 0] = dgFloat32 (m_attrib[i].m_normal_x);
		normal[j + 1] = dgFloat32 (m_attrib[i].m_normal_y);
		normal[j + 2] = dgFloat32 (m_attrib[i].m_normal_z);

		j = i * uvStrideInByte1;
		uv1[j + 0] = dgFloat32 (m_attrib[i].m_u1);
		uv1[j + 1] = dgFloat32 (m_attrib[i].m_v1);

		j = i * uvStrideInByte0;
		uv0[j + 0] = dgFloat32 (m_attrib[i].m_u0);
		uv0[j + 1] = dgFloat32 (m_attrib[i].m_v0);
	}
}


void dgMeshEffect::GetIndirectVertexStreams(
	dgInt32 vetexStrideInByte, dgFloat64* const vertex, dgInt32* const vertexIndices, dgInt32* const vertexCount,
	dgInt32 normalStrideInByte, dgFloat64* const normal, dgInt32* const normalIndices, dgInt32* const normalCount,
	dgInt32 uvStrideInByte0, dgFloat64* const uv0, dgInt32* const uvIndices0, dgInt32* const uvCount0,
	dgInt32 uvStrideInByte1, dgFloat64* const uv1, dgInt32* const uvIndices1, dgInt32* const uvCount1)
{
/*
	GetVertexStreams (vetexStrideInByte, vertex, normalStrideInByte, normal, uvStrideInByte0, uv0, uvStrideInByte1, uv1);

	*vertexCount = dgVertexListToIndexList(vertex, vetexStrideInByte, vetexStrideInByte, 0, m_atribCount, vertexIndices, dgFloat32 (0.0f));
	*normalCount = dgVertexListToIndexList(normal, normalStrideInByte, normalStrideInByte, 0, m_atribCount, normalIndices, dgFloat32 (0.0f));

	dgTriplex* const tmpUV = (dgTriplex*) GetAllocator()->MallocLow (dgInt32 (sizeof (dgTriplex) * m_atribCount));
	dgInt32 stride = dgInt32 (uvStrideInByte1 /sizeof (dgFloat32));
	for (dgInt32 i = 0; i < m_atribCount; i ++){
		tmpUV[i].m_x = uv1[i * stride + 0];
		tmpUV[i].m_y = uv1[i * stride + 1];
		tmpUV[i].m_z = dgFloat32 (0.0f);
	}

	dgInt32 count = dgVertexListToIndexList(&tmpUV[0].m_x, sizeof (dgTriplex), sizeof (dgTriplex), 0, m_atribCount, uvIndices1, dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < count; i ++){
		uv1[i * stride + 0] = tmpUV[i].m_x;
		uv1[i * stride + 1] = tmpUV[i].m_y;
	}
	*uvCount1 = count;

	stride = dgInt32 (uvStrideInByte0 /sizeof (dgFloat32));
	for (dgInt32 i = 0; i < m_atribCount; i ++){
		tmpUV[i].m_x = uv0[i * stride + 0];
		tmpUV[i].m_y = uv0[i * stride + 1];
		tmpUV[i].m_z = dgFloat32 (0.0f);
	}
	count = dgVertexListToIndexList(&tmpUV[0].m_x, sizeof (dgTriplex), sizeof (dgTriplex), 0, m_atribCount, uvIndices0, dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < count; i ++){
		uv0[i * stride + 0] = tmpUV[i].m_x;
		uv0[i * stride + 1] = tmpUV[i].m_y;
	}
	*uvCount0 = count;

	GetAllocator()->FreeLow (tmpUV);
*/
}

dgMeshEffect::dgIndexArray* dgMeshEffect::MaterialGeometryBegin()
{
	dgInt32 materials[256];
	dgInt32 streamIndexMap[256];

	dgInt32 count = 0;
	dgInt32 materialCount = 0;
	
	dgIndexArray* const array = (dgIndexArray*) GetAllocator()->MallocLow (dgInt32 (4 * sizeof (dgInt32) * GetCount() + sizeof (dgIndexArray) + 2048));
	array->m_indexList = (dgInt32*)&array[1];
	
	dgInt32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);	
	memset(streamIndexMap, 0, sizeof (streamIndexMap));
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark)) {
			dgEdge* ptr = edge;
			ptr->m_mark = mark;
			dgInt32 index0 = dgInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			dgInt32 index1 = dgInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			do {
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = dgInt32 (ptr->m_userData);
				array->m_indexList[count * 4 + 3] = dgInt32 (m_attrib[dgInt32 (edge->m_userData)].m_material);
				index1 = dgInt32 (ptr->m_userData);

				dgInt32 hashValue = array->m_indexList[count * 4 + 3] & 0xff;
				streamIndexMap[hashValue] ++;
				materials[hashValue] = array->m_indexList[count * 4 + 3];
				count ++;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	array->m_indexCount = count;
	array->m_materialCount = materialCount;

	count = 0;
	for (dgInt32 i = 0; i < 256;i ++) {
		if (streamIndexMap[i]) {
			array->m_materials[count] = materials[i];
			array->m_materialsIndexCount[count] = streamIndexMap[i] * 3;
			count ++;
		}
	}

	array->m_materialCount = count;

	return array;
}

void dgMeshEffect::MaterialGeomteryEnd(dgIndexArray* const handle)
{
	GetAllocator()->FreeLow (handle);
}


dgInt32 dgMeshEffect::GetFirstMaterial (dgIndexArray* const handle) const
{
	return GetNextMaterial (handle, -1);
}

dgInt32 dgMeshEffect::GetNextMaterial (dgIndexArray* const handle, dgInt32 materialId) const
{
	materialId ++;
	if(materialId >= handle->m_materialCount) {
		materialId = -1;
	}
	return materialId;
}

void dgMeshEffect::GetMaterialGetIndexStream (dgIndexArray* const handle, dgInt32 materialHandle, dgInt32* const indexArray) const
{

	dgInt32 index = 0;
	dgInt32 textureID = handle->m_materials[materialHandle];
	for (dgInt32 j = 0; j < handle->m_indexCount; j ++) {
		if (handle->m_indexList[j * 4 + 3] == textureID) {
			indexArray[index + 0] = handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = handle->m_indexList[j * 4 + 2];

			index += 3;
		}
	}
}

void dgMeshEffect::GetMaterialGetIndexStreamShort (dgIndexArray* const handle, dgInt32 materialHandle, dgInt16* const indexArray) const
{
	dgInt32 index = 0;
	dgInt32 textureID = handle->m_materials[materialHandle];
	for (dgInt32 j = 0; j < handle->m_indexCount; j ++) {
		if (handle->m_indexList[j * 4 + 3] == textureID) {
			indexArray[index + 0] = (dgInt16)handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = (dgInt16)handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = (dgInt16)handle->m_indexList[j * 4 + 2];
			index += 3;
		}
	}
}

dgCollisionInstance* dgMeshEffect::CreateCollisionTree(dgWorld* const world, dgInt32 shapeID) const
{
	dgCollisionBVH* const collision = new  (GetAllocator()) dgCollisionBVH (world);

	collision->BeginBuild();

	dgInt32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgInt32 count = 0;
			dgVector polygon[256]; 
			dgEdge* ptr = face;
			do {
				polygon[count] = dgVector (m_points[ptr->m_incidentVertex]);
				polygon[count].m_w = dgFloat32 (0.0f);
				count ++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);
			collision->AddFace(count, &polygon[0].m_x, sizeof (dgVector), dgInt32 (m_attrib[face->m_userData].m_material));
		}
	}
	collision->EndBuild(0);

	dgCollisionInstance* const instance = world->CreateInstance(collision, shapeID, dgGetIdentityMatrix());
	collision->Release();
	return instance;
}

dgCollisionInstance* dgMeshEffect::CreateConvexCollision(dgWorld* const world, dgFloat64 tolerance, dgInt32 shapeID, const dgMatrix& srcMatrix) const
{
	dgStack<dgVector> poolPtr (m_pointCount * 2); 
	dgVector* const pool = &poolPtr[0];

	dgBigVector minBox;
	dgBigVector maxBox;
	CalculateAABB (minBox, maxBox);
	dgVector com ((minBox + maxBox).Scale3 (dgFloat32 (0.5f)));

	dgInt32 count = 0;
	dgInt32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const vertex = &(*iter);
		if (vertex->m_mark != mark) {
			dgEdge* ptr = vertex;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != vertex);

			if (count < dgInt32 (poolPtr.GetElementsCount())) {
				const dgBigVector p = m_points[vertex->m_incidentVertex];
				pool[count] = dgVector (p) - com;
				count ++;
			}
		}
	}

	dgMatrix matrix (srcMatrix);
	matrix.m_posit += matrix.RotateVector(com);
	matrix.m_posit.m_w = dgFloat32 (1.0f);

	dgUnsigned32 crc = dgCollisionConvexHull::CalculateSignature (count, &pool[0].m_x, sizeof (dgVector));
	dgCollisionConvexHull* const collision = new (GetAllocator()) dgCollisionConvexHull (GetAllocator(), crc, count, sizeof (dgVector), dgFloat32 (tolerance), &pool[0].m_x);
	if (!collision->GetConvexVertexCount()) {
		collision->Release();
		return NULL;
	}
	dgCollisionInstance* const instance = world->CreateInstance(collision, shapeID, matrix);
	collision->Release();
	return instance;
}


void dgMeshEffect::TransformMesh (const dgMatrix& matrix)
{
	dgMatrix normalMatrix (matrix);
	normalMatrix.m_posit = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f));

	matrix.TransformTriplex (&m_points->m_x, sizeof (dgBigVector), &m_points->m_x, sizeof (dgBigVector), m_pointCount);
	matrix.TransformTriplex (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), m_atribCount);
	normalMatrix.TransformTriplex (&m_attrib[0].m_normal_x, sizeof (dgVertexAtribute), &m_attrib[0].m_normal_x, sizeof (dgVertexAtribute), m_atribCount);
}


dgMeshEffect::dgVertexAtribute dgMeshEffect::InterpolateEdge (dgEdge* const edge, dgFloat64 param) const
{
	dgVertexAtribute attrEdge;
	dgFloat64 t1 = param;
	dgFloat64 t0 = dgFloat64 (1.0f) - t1;
	dgAssert (t1 >= dgFloat64(0.0f));
	dgAssert (t1 <= dgFloat64(1.0f));

	const dgVertexAtribute& attrEdge0 = m_attrib[edge->m_userData];
	const dgVertexAtribute& attrEdge1 = m_attrib[edge->m_next->m_userData];

	attrEdge.m_vertex.m_x = attrEdge0.m_vertex.m_x * t0 + attrEdge1.m_vertex.m_x * t1;
	attrEdge.m_vertex.m_y = attrEdge0.m_vertex.m_y * t0 + attrEdge1.m_vertex.m_y * t1;
	attrEdge.m_vertex.m_z = attrEdge0.m_vertex.m_z * t0 + attrEdge1.m_vertex.m_z * t1;
	attrEdge.m_vertex.m_w = dgFloat32(0.0f);
	attrEdge.m_normal_x = attrEdge0.m_normal_x * t0 +  attrEdge1.m_normal_x * t1; 
	attrEdge.m_normal_y = attrEdge0.m_normal_y * t0 +  attrEdge1.m_normal_y * t1; 
	attrEdge.m_normal_z = attrEdge0.m_normal_z * t0 +  attrEdge1.m_normal_z * t1; 
	attrEdge.m_u0 = attrEdge0.m_u0 * t0 +  attrEdge1.m_u0 * t1;
	attrEdge.m_v0 = attrEdge0.m_v0 * t0 +  attrEdge1.m_v0 * t1;
	attrEdge.m_u1 = attrEdge0.m_u1 * t0 +  attrEdge1.m_u1 * t1;
	attrEdge.m_v1 = attrEdge0.m_v1 * t0 +  attrEdge1.m_v1 * t1;
	attrEdge.m_material = attrEdge0.m_material;
	return attrEdge;
}

bool dgMeshEffect::Sanity () const
{
	#ifdef  _DEBUG
		dgMeshEffect::Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();

			dgAssert (edge->m_twin->m_twin == edge);
			dgAssert (edge->m_next->m_incidentVertex == edge->m_twin->m_incidentVertex);
			dgAssert (edge->m_incidentVertex == edge->m_twin->m_next->m_incidentVertex);

			if (edge->m_incidentFace > 0) {
				dgBigVector p0 (m_points[edge->m_incidentVertex]);
				dgBigVector p1 (m_attrib[edge->m_userData].m_vertex);
				dgBigVector p1p0 (p1 - p0);
				dgFloat64 mag2 (p1p0 % p1p0);
				dgAssert (mag2 < 1.0e-16f);
			}
		}
	#endif
	return true;
}


dgEdge* dgMeshEffect::InsertEdgeVertex (dgEdge* const edge, dgFloat64 param)
{
	dgEdge* const twin = edge->m_twin;
	dgVertexAtribute attrEdge (InterpolateEdge (edge, param));
	dgVertexAtribute attrTwin (InterpolateEdge (twin, dgFloat32 (1.0f) - param));

	attrTwin.m_vertex = attrEdge.m_vertex;
	AddPoint(&attrEdge.m_vertex.m_x, dgFastInt (attrEdge.m_material));
	AddAtribute (attrTwin);

	dgInt32 edgeAttrV0 = dgInt32 (edge->m_userData);
	dgInt32 twinAttrV0 = dgInt32 (twin->m_userData);

	dgEdge* const faceA0 = edge->m_next;
	dgEdge* const faceA1 = edge->m_prev;
	dgEdge* const faceB0 = twin->m_next;
	dgEdge* const faceB1 = twin->m_prev;

//	SpliteEdgeAndTriangulate (m_pointCount - 1, edge);
	SpliteEdge (m_pointCount - 1, edge);

	faceA0->m_prev->m_userData = dgUnsigned64 (m_atribCount - 2);
	faceA1->m_next->m_userData = dgUnsigned64 (edgeAttrV0);

	faceB0->m_prev->m_userData = dgUnsigned64 (m_atribCount - 1);
	faceB1->m_next->m_userData = dgUnsigned64 (twinAttrV0);

	return faceA1->m_next;
}



dgMeshEffect::dgVertexAtribute dgMeshEffect::InterpolateVertex (const dgBigVector& srcPoint, const dgEdge* const face) const
{
	const dgBigVector point (srcPoint);

	dgVertexAtribute attribute;
	memset (&attribute, 0, sizeof (attribute));

//	dgBigVector normal (FaceNormal(face, &m_points[0].m_x, sizeof(dgBigVector)));
//	normal = normal.Scale3 (dgFloat64 (1.0f) / sqrt (normal % normal));
//	attribute.m_vertex = srcPoint;
//	attribute.m_normal_x = normal.m_x;
//	attribute.m_normal_y = normal.m_y;
//	attribute.m_normal_z = normal.m_z;

	dgFloat64 tol = dgFloat32 (1.0e-4f);
	for (dgInt32 i = 0; i < 4; i ++) {
		const dgEdge* ptr = face;
		const dgEdge* const edge0 = ptr;
		dgBigVector q0 (m_points[ptr->m_incidentVertex]);

		ptr = ptr->m_next;
		const dgEdge* edge1 = ptr;
		dgBigVector q1 (m_points[ptr->m_incidentVertex]);

		ptr = ptr->m_next;
		const dgEdge* edge2 = ptr;
		do {
			const dgBigVector q2 (m_points[ptr->m_incidentVertex]);

			dgBigVector p10 (q1 - q0);
			dgBigVector p20 (q2 - q0);

			dgFloat64 dot = p20 % p10;
			dgFloat64 mag1 = p10 % p10;
			dgFloat64 mag2 = p20 % p20;
			dgFloat64 collinear = dot * dot - mag2 * mag1;
			if (fabs (collinear) > dgFloat64 (1.0e-8f)) {
				dgBigVector p_p0 (point - q0);
				dgBigVector p_p1 (point - q1);
				dgBigVector p_p2 (point - q2);

				dgFloat64 alpha1 = p10 % p_p0;
				dgFloat64 alpha2 = p20 % p_p0;
				dgFloat64 alpha3 = p10 % p_p1;
				dgFloat64 alpha4 = p20 % p_p1;
				dgFloat64 alpha5 = p10 % p_p2;
				dgFloat64 alpha6 = p20 % p_p2;

				dgFloat64 vc = alpha1 * alpha4 - alpha3 * alpha2;
				dgFloat64 vb = alpha5 * alpha2 - alpha1 * alpha6;
				dgFloat64 va = alpha3 * alpha6 - alpha5 * alpha4;
				dgFloat64 den = va + vb + vc;
				dgFloat64 minError = den * (-tol);
				dgFloat64 maxError = den * (dgFloat32 (1.0f) + tol);
				if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {
					edge2 = ptr;

					den = dgFloat64 (1.0f) / (va + vb + vc);

					dgFloat64 alpha0 = dgFloat32 (va * den);
					dgFloat64 alpha1 = dgFloat32 (vb * den);
					dgFloat64 alpha2 = dgFloat32 (vc * den);

					const dgVertexAtribute& attr0 = m_attrib[edge0->m_userData];
					const dgVertexAtribute& attr1 = m_attrib[edge1->m_userData];
					const dgVertexAtribute& attr2 = m_attrib[edge2->m_userData];
					dgBigVector normal (attr0.m_normal_x * alpha0 + attr1.m_normal_x * alpha1 + attr2.m_normal_x * alpha2,
										attr0.m_normal_y * alpha0 + attr1.m_normal_y * alpha1 + attr2.m_normal_y * alpha2,
										attr0.m_normal_z * alpha0 + attr1.m_normal_z * alpha1 + attr2.m_normal_z * alpha2, dgFloat32 (0.0f));
					normal = normal.Scale3 (dgFloat64 (1.0f) / sqrt (normal % normal));

		#ifdef _DEBUG
					dgBigVector testPoint (attr0.m_vertex.m_x * alpha0 + attr1.m_vertex.m_x * alpha1 + attr2.m_vertex.m_x * alpha2,
										   attr0.m_vertex.m_y * alpha0 + attr1.m_vertex.m_y * alpha1 + attr2.m_vertex.m_y * alpha2,
										   attr0.m_vertex.m_z * alpha0 + attr1.m_vertex.m_z * alpha1 + attr2.m_vertex.m_z * alpha2, dgFloat32 (0.0f));
					dgAssert (fabs (testPoint.m_x - point.m_x) < dgFloat32 (1.0e-2f));
					dgAssert (fabs (testPoint.m_y - point.m_y) < dgFloat32 (1.0e-2f));
					dgAssert (fabs (testPoint.m_z - point.m_z) < dgFloat32 (1.0e-2f));
		#endif


					attribute.m_vertex.m_x = point.m_x;
					attribute.m_vertex.m_y = point.m_y;
					attribute.m_vertex.m_z = point.m_z;
					attribute.m_vertex.m_w = point.m_w;
					attribute.m_normal_x = normal.m_x;
					attribute.m_normal_y = normal.m_y;
					attribute.m_normal_z = normal.m_z;
					attribute.m_u0 = attr0.m_u0 * alpha0 +  attr1.m_u0 * alpha1 + attr2.m_u0 * alpha2;
					attribute.m_v0 = attr0.m_v0 * alpha0 +  attr1.m_v0 * alpha1 + attr2.m_v0 * alpha2;
					attribute.m_u1 = attr0.m_u1 * alpha0 +  attr1.m_u1 * alpha1 + attr2.m_u1 * alpha2;
					attribute.m_v1 = attr0.m_v1 * alpha0 +  attr1.m_v1 * alpha1 + attr2.m_v1 * alpha2;

					attribute.m_material = attr0.m_material;
					dgAssert (attr0.m_material == attr1.m_material);
					dgAssert (attr0.m_material == attr2.m_material);
					return attribute; 
				}
			}
				
			q1 = q2;
			edge1 = ptr;

			ptr = ptr->m_next;
		} while (ptr != face);
		tol *= dgFloat64 (2.0f);
	}
	// this should never happens
	dgAssert (0);
	return attribute;
}

bool dgMeshEffect::HasOpenEdges () const
{
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace < 0){
			return true;
		}
	}
	return false;
}

dgFloat64 dgMeshEffect::CalculateVolume () const
{
	dgAssert (0);
	return 0;
	/*

	dgPolyhedraMassProperties localData;

	dgInt32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgInt32 count;
		dgEdge* ptr;
		dgEdge* face;
		dgVector points[256];
		
		face = &(*iter);
		if ((face->m_incidentFace > 0) && (face->m_mark != mark)) {
			count = 0;
			ptr = face;
			do {
				points[count] = m_points[ptr->m_incidentVertex];
				count ++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);
			localData.AddCGFace (count, points);
		}
	}

	dgFloat32 volume;
	dgVector p0;
	dgVector p1;
	dgVector com;
	dgVector inertia;
	dgVector crossInertia;
	volume = localData.MassProperties (com, inertia, crossInertia);
	return volume;
*/
}




dgMeshEffect* dgMeshEffect::GetNextLayer (dgInt32 mark)
{
	Iterator iter(*this);
	dgEdge* edge = NULL;
	for (iter.Begin (); iter; iter ++) {
		edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
			break;
		}
	}

	if (!edge) {
		return NULL;
	}

	dgInt32 layer = dgInt32 (m_points[edge->m_incidentVertex].m_w);
	dgPolyhedra polyhedra(GetAllocator());

	polyhedra.BeginFace ();
	for (iter.Begin (); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
			dgInt32 thislayer = dgInt32 (m_points[edge->m_incidentVertex].m_w);
			if (thislayer == layer) {
				dgEdge* ptr = edge;
				dgInt32 count = 0;
				dgInt32 faceIndex[256];
				dgInt64 faceDataIndex[256];
				do {
					ptr->m_mark = mark;
					faceIndex[count] = ptr->m_incidentVertex;
					faceDataIndex[count] = ptr->m_userData;
					count ++;
					dgAssert (count < dgInt32 (sizeof (faceIndex)/ sizeof(faceIndex[0])));
					ptr = ptr->m_next;
				} while (ptr != edge);
				polyhedra.AddFace (count, &faceIndex[0], &faceDataIndex[0]);
			}
		}
	}
	polyhedra.EndFace ();

	dgMeshEffect* solid = NULL;
	if (polyhedra.GetCount()) {
		solid = new (GetAllocator()) dgMeshEffect(polyhedra, *this);
		solid->SetLRU(mark);
	}
	return solid;
}



void dgMeshEffect::MergeFaces (const dgMeshEffect* const source)
{
	dgInt32 mark = source->IncLRU();
	dgPolyhedra::Iterator iter (*source);
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) {
			dgVertexAtribute face[DG_MESH_EFFECT_POINT_SPLITED];

			dgInt32 count = 0;
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				face[count] = source->m_attrib[ptr->m_userData];
				count ++;
				dgAssert (count < dgInt32 (sizeof (face) / sizeof (face[0])));
				ptr = ptr->m_next;
			} while (ptr != edge);
			AddPolygon(count, &face[0].m_vertex.m_x, sizeof (dgVertexAtribute), dgFastInt (face[0].m_material));
		}
	}
}


bool dgMeshEffect::SeparateDuplicateLoops (dgEdge* const face)
{
	for (dgEdge* ptr0 = face; ptr0 != face->m_prev; ptr0 = ptr0->m_next) {
		dgInt32 index = ptr0->m_incidentVertex;

		dgEdge* ptr1 = ptr0->m_next; 
		do {
			if (ptr1->m_incidentVertex == index) {
				dgEdge* const ptr00 = ptr0->m_prev;
				dgEdge* const ptr11 = ptr1->m_prev;

				ptr00->m_next = ptr1;
				ptr1->m_prev = ptr00;

				ptr11->m_next = ptr0;
				ptr0->m_prev = ptr11;

				return true;
			}

			ptr1 = ptr1->m_next;
		} while (ptr1 != face);
	}

	return false;
}



void dgMeshEffect::RepairTJoints ()
{
	dgAssert (Sanity ());

	// delete edge of zero length
	bool dirty = true;
	while (dirty) {
		dgFloat64 tol = 1.0e-5;
		dgFloat64 tol2 = tol * tol;
		dirty = false;
		dgPolyhedra::Iterator iter (*this);
		for (iter.Begin(); iter; ) {
			dgEdge* const edge = &(*iter);
			iter ++;
			const dgBigVector& p0 = m_points[edge->m_incidentVertex];
			const dgBigVector& p1 = m_points[edge->m_twin->m_incidentVertex];
			dgBigVector dist (p1 - p0);
			dgFloat64 mag2 = dist % dist;
			if (mag2 < tol2) {
				bool move = true;
				while (move) {
					move = false;
					dgEdge* ptr = edge->m_twin;
					do {
						if ((&(*iter) == ptr) || (&(*iter) == ptr->m_twin)) {
							move = true;
							iter ++;
						}
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge->m_twin);

					ptr = edge;
					do {
						if ((&(*iter) == ptr) || (&(*iter) == ptr->m_twin)) {
							move = true;
							iter ++;
						}
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);
				}
				
				dgEdge* const collapsedEdge = CollapseEdge(edge);
				if (collapsedEdge) {
					dirty = true;
					dgBigVector q (m_points[collapsedEdge->m_incidentVertex]);
					dgEdge* ptr = collapsedEdge;
					do {
						if (ptr->m_incidentFace > 0) {
							m_attrib[ptr->m_userData].m_vertex = q;
						}
						ptr = ptr->m_twin->m_next;
					} while (ptr != collapsedEdge);
				}
			}
		}
	}
	dgAssert (Sanity ());

	// repair straight open edges
	dgInt32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if ((edge->m_mark) != mark && (edge->m_incidentFace < 0)) {

			while (SeparateDuplicateLoops (edge));
			dgEdge* ptr = edge; 
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	dgAssert (Sanity ());
	DeleteDegenerateFaces(&m_points[0].m_x, sizeof (m_points[0]), dgFloat64 (1.0e-7f));
	dgAssert (Sanity ());

	// delete straight line edges
	dirty = true;
	while (dirty) {
		dgFloat64 tol = 1.0 - 1.0e-8;
		dgFloat64 tol2 = tol * tol;

		dirty = false;
		dgAssert (Sanity ());

		dgPolyhedra::Iterator iter (*this);
		for (iter.Begin(); iter; ) {
			dgEdge* const edge = &(*iter);
			iter ++;

			const dgBigVector& p0 = m_points[edge->m_incidentVertex];
			const dgBigVector& p1 = m_points[edge->m_next->m_incidentVertex];
			const dgBigVector& p2 = m_points[edge->m_next->m_next->m_incidentVertex];

			dgBigVector A (p1 - p0);
			dgBigVector B (p2 - p1);
			dgFloat64 ab = A % B;
			if (ab >= 0.0f) {
				dgFloat64 aa = A % A;
				dgFloat64 bb = B % B;

				dgFloat64 magab2 = ab * ab;
				dgFloat64 magaabb = aa * bb * tol2;
				if (magab2 >= magaabb) {
					if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
						if (edge->m_twin->m_prev == edge->m_next->m_twin) {
							dgEdge* const newEdge = AddHalfEdge(edge->m_incidentVertex, edge->m_next->m_next->m_incidentVertex);
							if (newEdge) {
								dirty = true;
								dgEdge* const newTwin = AddHalfEdge(edge->m_next->m_next->m_incidentVertex, edge->m_incidentVertex);
								dgAssert (newEdge);
								dgAssert (newTwin);

								newEdge->m_twin = newTwin;
								newTwin->m_twin = newEdge;

								newEdge->m_userData = edge->m_userData;
								newTwin->m_userData = edge->m_twin->m_prev->m_userData;

								newEdge->m_incidentFace = edge->m_incidentFace;
								newTwin->m_incidentFace = edge->m_twin->m_incidentFace;

								dgEdge* const nextEdge = edge->m_next;

								nextEdge->m_twin->m_prev->m_next = newTwin;
								newTwin->m_prev = nextEdge->m_twin->m_prev;

								edge->m_twin->m_next->m_prev = newTwin;
								newTwin->m_next = edge->m_twin->m_next;

								nextEdge->m_next->m_prev = newEdge;
								newEdge->m_next = nextEdge->m_next;

								edge->m_prev->m_next = newEdge;
								newEdge->m_prev = edge->m_prev;

								while ((&(*iter) == edge->m_twin) || (&(*iter) == nextEdge) || (&(*iter) == nextEdge->m_twin)) {
									iter ++;
								}

								nextEdge->m_twin->m_prev = nextEdge;
								nextEdge->m_twin->m_next = nextEdge;
								nextEdge->m_prev = nextEdge->m_twin;
								nextEdge->m_next = nextEdge->m_twin;

								edge->m_twin->m_prev = edge;
								edge->m_twin->m_next = edge;
								edge->m_prev = edge->m_twin;
								edge->m_next = edge->m_twin;

								DeleteEdge(edge);
								DeleteEdge(nextEdge);
								//dgAssert (Sanity ());

							} else if (edge->m_next->m_next->m_next == edge) {
								dirty = true;
								dgEdge* const openEdge = edge;
								dgEdge* const nextEdge = openEdge->m_next;
								dgEdge* const deletedEdge = openEdge->m_prev;
								while ((&(*iter) == deletedEdge) || (&(*iter) == deletedEdge->m_twin)) {
									iter ++;
								}

								openEdge->m_userData = deletedEdge->m_twin->m_userData;

								dgBigVector p2p0 (p2 - p0);
								dgFloat64 den = p2p0 % p2p0;
								dgFloat64 param1 = ((p1 - p0) % p2p0) / den;
								dgVertexAtribute attib1 = InterpolateEdge (deletedEdge->m_twin, param1);
								AddAtribute(attib1);
								openEdge->m_next->m_userData = m_atribCount  - 1;

								openEdge->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
								openEdge->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;

								deletedEdge->m_twin->m_prev->m_next = openEdge;
								openEdge->m_prev = deletedEdge->m_twin->m_prev;

								deletedEdge->m_twin->m_next->m_prev = nextEdge;
								nextEdge->m_next = deletedEdge->m_twin->m_next;

								deletedEdge->m_twin->m_next = deletedEdge;
								deletedEdge->m_twin->m_prev = deletedEdge;
								deletedEdge->m_next = deletedEdge->m_twin;
								deletedEdge->m_prev = deletedEdge->m_twin;
								DeleteEdge(deletedEdge);
								//dgAssert (Sanity ());
							}
						}
					} else if (FindEdge(edge->m_incidentVertex, edge->m_next->m_next->m_incidentVertex)) {
						dgEdge* const openEdge = edge;
						dgAssert (openEdge->m_incidentFace <= 0);
						dgEdge* const nextEdge = openEdge->m_next;
						dgEdge* const deletedEdge = openEdge->m_prev;
						if (deletedEdge == openEdge->m_next->m_next) {
							dirty = true;
							while ((&(*iter) == deletedEdge) || (&(*iter) == deletedEdge->m_twin)) {
								iter ++;
							}

							dgAssert (deletedEdge->m_twin->m_incidentFace > 0);
							openEdge->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
							openEdge->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;

							openEdge->m_userData = deletedEdge->m_twin->m_userData;
							dgBigVector p2p0 (p2 - p0);
							dgFloat64 den = p2p0 % p2p0;
							dgFloat64 param1 = ((p1 - p0) % p2p0) / den;
							dgVertexAtribute attib1 = InterpolateEdge (deletedEdge->m_twin, param1);
							attib1.m_vertex = m_points[openEdge->m_next->m_incidentVertex];
							AddAtribute(attib1);
							openEdge->m_next->m_userData = m_atribCount  - 1;
							
							deletedEdge->m_twin->m_prev->m_next = openEdge;
							openEdge->m_prev = deletedEdge->m_twin->m_prev;

							deletedEdge->m_twin->m_next->m_prev = nextEdge;
							nextEdge->m_next = deletedEdge->m_twin->m_next;

							deletedEdge->m_twin->m_next = deletedEdge;
							deletedEdge->m_twin->m_prev = deletedEdge;
							deletedEdge->m_next = deletedEdge->m_twin;
							deletedEdge->m_prev = deletedEdge->m_twin;
							DeleteEdge(deletedEdge);
							//dgAssert (Sanity ());
						}

					} else {

						dgEdge* const openEdge = (edge->m_incidentFace <= 0) ? edge : edge->m_twin;
						dgAssert (openEdge->m_incidentFace <= 0);

						const dgBigVector& p3 = m_points[openEdge->m_next->m_next->m_next->m_incidentVertex];

						dgBigVector A (p3 - p2);
						dgBigVector B (p2 - p1);
						dgFloat64 ab (A % B);
						if (ab >= 0.0) {
							dgFloat64 aa (A % A);
							dgFloat64 bb (B % B);

							dgFloat64 magab2 = ab * ab;
							dgFloat64 magaabb = aa * bb * tol2;
							if (magab2 >= magaabb) {
								if (openEdge->m_next->m_next->m_next->m_next != openEdge) {
									const dgBigVector& p4 = m_points[openEdge->m_prev->m_incidentVertex];
									dgBigVector A (p1 - p0);
									dgBigVector B (p1 - p4);
									dgFloat64 ab (A % B);
									if (ab < 0.0f) {
										dgFloat64 magab2 = ab * ab;
										dgFloat64 magaabb = aa * bb * tol2;
										if (magab2 >= magaabb) {
											dgEdge* const newFace = ConnectVertex (openEdge->m_prev, openEdge->m_next);
											dirty |= newFace ? true : false;
										}
									}
									//dgAssert (Sanity ());
								} else if (openEdge->m_prev->m_twin->m_incidentFace > 0) {
									dirty = true;

									dgEdge* const nextEdge = openEdge->m_next->m_next;
									dgEdge* const deletedEdge = openEdge->m_prev;
									while ((&(*iter) == deletedEdge) || (&(*iter) == deletedEdge->m_twin)) {
										iter ++;
									}

									openEdge->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
									openEdge->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
									openEdge->m_next->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;

									openEdge->m_userData = deletedEdge->m_twin->m_userData;

									dgBigVector p3p0 (p3 - p0);
									dgFloat64 den = p3p0 % p3p0;
									dgFloat64 param1 = ((p1 - p0) % p3p0) / den;
									dgVertexAtribute attib1 = InterpolateEdge (deletedEdge->m_twin, param1);
									attib1.m_vertex = m_points[openEdge->m_next->m_incidentVertex];
									AddAtribute(attib1);
									openEdge->m_next->m_userData = m_atribCount  - 1;

									dgFloat64 param2 = ((p2 - p0) % p3p0) / den;
									dgVertexAtribute attib2 = InterpolateEdge (deletedEdge->m_twin, param2);
									attib2.m_vertex = m_points[openEdge->m_next->m_next->m_incidentVertex];
									AddAtribute(attib2);
									openEdge->m_next->m_next->m_userData = m_atribCount  - 1;

									deletedEdge->m_twin->m_prev->m_next = openEdge;
									openEdge->m_prev = deletedEdge->m_twin->m_prev;

									deletedEdge->m_twin->m_next->m_prev = nextEdge;
									nextEdge->m_next = deletedEdge->m_twin->m_next;
									
									deletedEdge->m_twin->m_next = deletedEdge;
									deletedEdge->m_twin->m_prev = deletedEdge;
									deletedEdge->m_next = deletedEdge->m_twin;
									deletedEdge->m_prev = deletedEdge->m_twin;
									DeleteEdge(deletedEdge);
									//dgAssert (Sanity ());
								}
							}
						}
					}
				}
			}
		}
	}
	dgAssert (Sanity ());

	DeleteDegenerateFaces(&m_points[0].m_x, sizeof (m_points[0]), dgFloat64 (1.0e-7f));
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_incidentFace > 0) {
			dgBigVector p0 (m_points[edge->m_incidentVertex]);
			m_attrib[edge->m_userData].m_vertex.m_x = p0.m_x;
			m_attrib[edge->m_userData].m_vertex.m_y = p0.m_y;
			m_attrib[edge->m_userData].m_vertex.m_z = p0.m_z;
		}
	}
	dgAssert (Sanity ());
}





