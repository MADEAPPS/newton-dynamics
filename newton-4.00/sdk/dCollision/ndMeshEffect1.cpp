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

#include "dCoreStdafx.h"
#include "dSort.h"
#include "dStack.h"
#include "dVector.h"
#include "dMatrix.h"
#include "ndShape.h"
#include "ndMeshEffect.h"
#include "ndShapeInstance.h"
#include "ndShapeConvexHull.h"

#if 0
void ndMeshEffect::dAttibutFormat::CopyFrom(const dAttibutFormat& source)
{
	m_pointChannel.CopyFrom(source.m_pointChannel);
	m_materialChannel.CopyFrom(source.m_materialChannel);
	m_normalChannel.CopyFrom(source.m_normalChannel);
	m_binormalChannel.CopyFrom(source.m_binormalChannel);
	m_colorChannel.CopyFrom(source.m_colorChannel);
	m_uv0Channel.CopyFrom(source.m_uv0Channel);
	m_uv1Channel.CopyFrom(source.m_uv1Channel);
}


ndMeshEffect::dMeshBVH::dgFitnessList::dgFitnessList(dgMemoryAllocator* const allocator)
	:dTree <dgMeshBVHNode*, dgMeshBVHNode*>(allocator)
{
}

dFloat64 ndMeshEffect::dMeshBVH::dgFitnessList::TotalCost() const
{
	dFloat64 cost = dFloat32(0.0f);
	Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dTreeNode* const node = iter.GetNode();
		dgMeshBVHNode* const box = node->GetInfo();
		cost += box->m_area;
	}
	return cost;
}


ndMeshEffect::dMeshBVH::dgMeshBVHNode::dgMeshBVHNode(const ndMeshEffect* const mesh, dEdge* const face, void* const userData)
	:m_area(dFloat32(0.0f))
	, m_face(face)
	, m_userData(userData)
	, m_left(nullptr)
	, m_right(nullptr)
	, m_parent(nullptr)
{
	dBigVector p0(dFloat32(1.0e30f));
	dBigVector p1(dFloat32(-1.0e30f));

	const dBigVector* const points = (dBigVector*)mesh->GetVertexPool();

	dEdge* ptr = m_face;
	do {
		dInt32 i = ptr->m_incidentVertex;
		const dBigVector& p = points[i];
		p0 = p.GetMin(p0);
		p1 = p.GetMax(p1);

		ptr = ptr->m_next;
	} while (ptr != face);

	dVector padding(dFloat32(1.0f / 32.0f));
	SetBox(p0 - padding, p1 + padding);
}

ndMeshEffect::dMeshBVH::dgMeshBVHNode::dgMeshBVHNode(dgMeshBVHNode* const left, dgMeshBVHNode* const right)
	:m_area(dFloat32(0.0f))
	, m_face(nullptr)
	, m_userData(nullptr)
	, m_left(left)
	, m_right(right)
	, m_parent(nullptr)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	//	dVector p0 (dMin (left->m_p0.m_x, right->m_p0.m_x), dMin (left->m_p0.m_y, right->m_p0.m_y), dMin (left->m_p0.m_z, right->m_p0.m_z), dFloat32 (0.0f));
	//	dVector p1 (dMax (left->m_p1.m_x, right->m_p1.m_x), dMax (left->m_p1.m_y, right->m_p1.m_y), dMax (left->m_p1.m_z, right->m_p1.m_z), dFloat32 (0.0f));
	dVector p0(left->m_p0.GetMin(right->m_p0));
	dVector p1(left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}


ndMeshEffect::dMeshBVH::dgMeshBVHNode::~dgMeshBVHNode()
{
	if (m_left) {
		delete m_left;
	}
	if (m_right) {
		delete m_right;
	}
}

void ndMeshEffect::dMeshBVH::dgMeshBVHNode::SetBox(const dVector& p0, const dVector& p1)
{
	m_p0 = p0 & dVector::m_triplexMask;
	m_p1 = p1 & dVector::m_triplexMask;
	dVector size(dVector::m_half * (m_p1 - m_p0));
	dVector size1(size.ShiftTripleLeft());
	dAssert(size1.m_w == dFloat32(0.0f));
	m_area = size.DotProduct(size1).GetScalar();
}

ndMeshEffect::dMeshBVH::dMeshBVH(const ndMeshEffect* const mesh)
	:m_mesh(mesh)
	, m_rootNode(nullptr)
	, m_fitness(m_mesh->GetAllocator())
{
}


ndMeshEffect::dMeshBVH::~dMeshBVH()
{
	Cleanup();
}

ndMeshEffect* ndMeshEffect::CreateFromSerialization(dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData)
{
	return new (allocator) ndMeshEffect(allocator, deserialization, userData);
}

void ndMeshEffect::Serialize(dgSerialize callback, void* const userData) const
{
	dAssert(0);
	/*
		dInt32 faceCount = 0;
		dTree<dEdge*, dEdge*>filter(GetAllocator());
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			dEdge* const face = &iter.GetNode()->GetInfo();
			if (!filter.Find(face) && (face->m_incidentFace > 0)) {
				faceCount ++;
				dEdge* edge = face;
				do {
					filter.Insert(edge, edge);
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		callback (userData, &faceCount, sizeof (dInt32));
		callback (userData, &m_pointCount, sizeof (dInt32));
		callback (userData, &m_atribCount, sizeof (dInt32));
		callback (userData, &m_atribCount, sizeof (dInt32));

		callback (userData, m_points, m_pointCount * sizeof (dBigVector));
		callback (userData, m_attrib, m_atribCount * sizeof (dgVertexAtribute));

		filter.RemoveAll();
		for (iter.Begin(); iter; iter ++) {
			dEdge* const face = &iter.GetNode()->GetInfo();
			if (!filter.Find(face) && (face->m_incidentFace > 0)) {
				dInt32 indices[1024];
				dInt64 attibuteIndex[1024];
				dInt32 vertexCount = 0;
				dEdge* edge = face;
				do {
					indices[vertexCount] = edge->m_incidentVertex;
					attibuteIndex[vertexCount] = edge->m_userData;
					vertexCount ++;
					filter.Insert(edge, edge);
					edge = edge->m_next;
				} while (edge != face);

				callback (userData, &vertexCount, sizeof (dInt32));
				callback (userData, indices, vertexCount * sizeof (dInt32));
				callback (userData, attibuteIndex, vertexCount * sizeof (dInt64));
			}
		}
	*/
}

void ndMeshEffect::dMeshBVH::Build()
{
	dInt32 lru = m_mesh->IncLRU();
	/*
		for (void* faceNode = m_mesh->GetFirstFace (); faceNode; faceNode = m_mesh->GetNextFace(faceNode)) {
			if (!m_mesh->IsFaceOpen(faceNode)) {
				dEdge* const face = &((dTreeNode*)faceNode)->GetInfo();
				if (face->m_mark != mark) {
					AddFaceNode(face, nullptr);
				}
			}
		}
	*/
	ndMeshEffect::Iterator iter(*m_mesh);
	for (iter.Begin(); iter; iter++) {
		dEdge* const face = &iter.GetNode()->GetInfo();
		if (face->m_mark != lru) {
			AddFaceNode(face, nullptr);
		}
	}
	ImproveNodeFitness();
}

void ndMeshEffect::dMeshBVH::Cleanup()
{
	if (m_rootNode) {
		delete m_rootNode;
	}
}

dFloat32 ndMeshEffect::dMeshBVH::CalculateSurfaceArea(dgMeshBVHNode* const node0, dgMeshBVHNode* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = dVector(dMin(node0->m_p0.m_x, node1->m_p0.m_x), dMin(node0->m_p0.m_y, node1->m_p0.m_y), dMin(node0->m_p0.m_z, node1->m_p0.m_z), dFloat32(0.0f));
	maxBox = dVector(dMax(node0->m_p1.m_x, node1->m_p1.m_x), dMax(node0->m_p1.m_y, node1->m_p1.m_y), dMax(node0->m_p1.m_z, node1->m_p1.m_z), dFloat32(0.0f));
	dVector side0((maxBox - minBox) * dVector::m_half);
	dVector side1(side0.ShiftTripleLeft());
	dAssert(side1.m_w == dFloat32(0.0f));
	return side0.DotProduct(side1).GetScalar();
}

void ndMeshEffect::dMeshBVH::ImproveNodeFitness(dgMeshBVHNode* const node)
{
	dAssert(node->m_left);
	dAssert(node->m_right);

	if (node->m_parent) {
		if (node->m_parent->m_left == node) {
			dFloat32 cost0 = node->m_area;

			dVector cost1P0;
			dVector cost1P1;
			dFloat32 cost1 = CalculateSurfaceArea(node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dVector cost2P0;
			dVector cost2P1;
			dFloat32 cost2 = CalculateSurfaceArea(node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					}
					else {
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				else {
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


			}
			else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					}
					else {
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				else {
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
		}
		else {
			dFloat32 cost0 = node->m_area;

			dVector cost1P0;
			dVector cost1P1;
			dFloat32 cost1 = CalculateSurfaceArea(node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dVector cost2P0;
			dVector cost2P1;
			dFloat32 cost2 = CalculateSurfaceArea(node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					}
					else {
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				else {
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

			}
			else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgMeshBVHNode* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					}
					else {
						dAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				else {
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
	//	dAssert (SanityCheck());
}


void ndMeshEffect::dMeshBVH::ImproveNodeFitness()
{
	dFloat64 cost0 = m_fitness.TotalCost();
	dFloat64 cost1 = cost0;
	do {
		cost0 = cost1;
		dgFitnessList::Iterator iter(m_fitness);
		for (iter.Begin(); iter; iter++) {
			dgFitnessList::dTreeNode* const node = iter.GetNode();
			ImproveNodeFitness(node->GetInfo());
		}
		cost1 = m_fitness.TotalCost();
	} while (cost1 < (dFloat32(0.95f)) * cost0);
}

/*
ndMeshEffect::dMeshBVH::dgMeshBVHNode* ndMeshEffect::dMeshBVH::CreateLeafNode (dEdge* const face, void* const userData)
{
	dgMemoryAllocator* const allocator = m_mesh->GetAllocator();
	return new (allocator) dgMeshBVHNode (m_mesh, face, userData);
}
*/

ndMeshEffect::dMeshBVH::dgMeshBVHNode* ndMeshEffect::dMeshBVH::AddFaceNode(dEdge* const face, void* const userData)
{
	dgMemoryAllocator* const allocator = m_mesh->GetAllocator();

	dgMeshBVHNode* const newNode = CreateLeafNode(face, userData);
	if (!m_rootNode) {
		m_rootNode = newNode;
	}
	else {

		dVector p0;
		dVector p1;
		dgMeshBVHNode* sibling = m_rootNode;

		dFloat32 surfaceArea = dMeshBVH::CalculateSurfaceArea(newNode, sibling, p0, p1);
		while (sibling->m_left && sibling->m_right) {

			if (surfaceArea > sibling->m_area) {
				break;
			}

			sibling->SetBox(p0, p1);

			dVector leftP0;
			dVector leftP1;
			dFloat32 leftSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_left, leftP0, leftP1);

			dVector rightP0;
			dVector rightP1;
			dFloat32 rightSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_right, rightP0, rightP1);

			if (leftSurfaceArea < rightSurfaceArea) {
				sibling = sibling->m_left;
				p0 = leftP0;
				p1 = leftP1;
				surfaceArea = leftSurfaceArea;
			}
			else {
				sibling = sibling->m_right;
				p0 = rightP0;
				p1 = rightP1;
				surfaceArea = rightSurfaceArea;
			}
		}

		if (!sibling->m_parent) {
			m_rootNode = new (allocator) dgMeshBVHNode(sibling, newNode);
			m_fitness.Insert(m_rootNode, m_rootNode);
		}
		else {
			dgMeshBVHNode* const parent = sibling->m_parent;
			if (parent->m_left == sibling) {
				dgMeshBVHNode* const node = new (allocator) dgMeshBVHNode(sibling, newNode);
				m_fitness.Insert(node, node);
				parent->m_left = node;
				node->m_parent = parent;
			}
			else {
				dAssert(parent->m_right == sibling);
				dgMeshBVHNode* const node = new (allocator) dgMeshBVHNode(sibling, newNode);
				m_fitness.Insert(node, node);
				parent->m_right = node;
				node->m_parent = parent;
			}
		}
	}

	return newNode;
}


void ndMeshEffect::dMeshBVH::RemoveNode(dgMeshBVHNode* const treeNode)
{
	if (!treeNode->m_parent) {
		delete (m_rootNode);
		m_rootNode = nullptr;
	}
	else if (!treeNode->m_parent->m_parent) {
		dgMeshBVHNode* const root = m_rootNode;
		if (treeNode->m_parent->m_left == treeNode) {
			m_rootNode = treeNode->m_parent->m_right;
			treeNode->m_parent->m_right = nullptr;
		}
		else {
			dAssert(treeNode->m_parent->m_right == treeNode);
			m_rootNode = treeNode->m_parent->m_left;
			treeNode->m_parent->m_left = nullptr;
		}
		m_rootNode->m_parent = nullptr;
		dAssert(m_fitness.Find(root));
		m_fitness.Remove(root);
		delete (root);

	}
	else {
		dgMeshBVHNode* const root = treeNode->m_parent->m_parent;
		if (treeNode->m_parent == root->m_left) {
			if (treeNode->m_parent->m_right == treeNode) {
				root->m_left = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = nullptr;
			}
			else {
				dAssert(treeNode->m_parent->m_left == treeNode);
				root->m_left = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = nullptr;
			}
			root->m_left->m_parent = root;
		}
		else {
			if (treeNode->m_parent->m_right == treeNode) {
				root->m_right = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = nullptr;
			}
			else {
				dAssert(treeNode->m_parent->m_left == treeNode);
				root->m_right = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = nullptr;
			}
			root->m_right->m_parent = root;
		}

		dAssert(m_fitness.Find(treeNode->m_parent));
		m_fitness.Remove(treeNode->m_parent);
		delete (treeNode->m_parent);
	}

	//dAssert (SanityCheck());
}

bool ndMeshEffect::dMeshBVH::SanityCheck() const
{
#ifdef _DEBUG
	dAssert(m_mesh->Sanity());

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

		dInt32 stack = 2;
		stackPool[0] = m_rootNode->m_left;
		stackPool[1] = m_rootNode->m_right;
		while (stack) {
			stack--;
			dgMeshBVHNode* const node = stackPool[stack];

			if ((node->m_parent->m_left != node) && (node->m_parent->m_right != node)) {
				return false;
			}

			if (node->m_left) {
				dAssert(node->m_right);
				stackPool[stack] = node->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));

				stackPool[stack] = node->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));
			}
		}
	}
#endif
	return true;
}

void ndMeshEffect::dMeshBVH::GetOverlapNodes(dList<dgMeshBVHNode*>& overlapNodes, const dBigVector& p0, const dBigVector& p1) const
{
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	dInt32 stack = 1;
	stackPool[0] = m_rootNode;

	dVector l0(p0);
	dVector l1(p1);

	while (stack) {
		stack--;
		dgMeshBVHNode* const me = stackPool[stack];

		if (me && dgOverlapTest(me->m_p0, me->m_p1, l0, l1)) {

			if (!me->m_left) {
				dAssert(!me->m_right);
				overlapNodes.Append(me);
			}
			else {
				dAssert(me->m_left);
				dAssert(me->m_right);
				stackPool[stack] = me->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));
			}
		}
	}
}

/*
dFloat64 ndMeshEffect::dMeshBVH::VertexRayCast (const dBigVector& p0, const dBigVector& p1) const
{
	dAssert (0);

	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	dInt32 stack = 1;
	stackPool[0] = m_rootNode;

	dVector l0(p0);
	dVector l1(p1);
	dBigVector p1p0 (p1 - p0);
	dFloat64 den = p1p0 % p1p0;

	const dBigVector* const points = (dBigVector*) m_mesh->GetVertexPool();
	while (stack) {
		stack --;
		dgMeshBVHNode* const me = stackPool[stack];

		if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
			if (!me->m_left) {
				dAssert (!me->m_right);

				dEdge* ptr = me->m_face;
				do {
					dInt32 index = ptr->m_incidentVertex;
					const dBigVector& q0 = points[index];
					dBigVector q0p0 (q0 - p0);
					dFloat64 alpha = q0p0 % p1p0;
					if ((alpha > (DG_BOOLEAN_ZERO_TOLERANCE * den)) && (alpha < (den - DG_BOOLEAN_ZERO_TOLERANCE))) {
						dBigVector dist (p0 + p1p0.Scale (alpha / den) - q0);
						dFloat64 dist2 = dist % dist;
						if (dist2 < (DG_BOOLEAN_ZERO_TOLERANCE * DG_BOOLEAN_ZERO_TOLERANCE)) {
							return alpha / den;
						}
					}

					ptr = ptr->m_next;
				} while (ptr != me->m_face);
			} else {
				dAssert (me->m_left);
				dAssert (me->m_right);
				stackPool[stack] = me->m_left;
				stack++;
				dAssert (stack < dInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dAssert (stack < dInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
			}
		}
	}
	return 1.2f;
}


bool ndMeshEffect::dMeshBVH::RayRayIntersect (dEdge* const edge, const ndMeshEffect* const otherMesh, dEdge* const otherEdge, dFloat64& param, dFloat64& otherParam) const
{
	dAssert (0);

	dBigVector ray_p0 (m_mesh->m_points[edge->m_incidentVertex]);
	dBigVector ray_p1 (m_mesh->m_points[edge->m_twin->m_incidentVertex]);

	dBigVector ray_q0 (otherMesh->m_points[otherEdge->m_incidentVertex]);
	dBigVector ray_q1 (otherMesh->m_points[otherEdge->m_twin->m_incidentVertex]);

	dBigVector p1p0 (ray_p1 - ray_p0);
	dBigVector q1q0 (ray_q1 - ray_q0);
	dBigVector p0q0 (ray_p0 - ray_q0);

	dFloat64 a = p1p0 % p1p0;        // always >= 0
	dFloat64 c = q1q0 % q1q0;        // always >= 0
	dFloat64 b = p1p0 % q1q0;

	dFloat64 d = (p1p0 % p0q0);
	dFloat64 e = (q1q0 % p0q0);
	dFloat64 den = a * c - b * b;   // always >= 0
	// compute the line parameters of the two closest points
	if (den < DG_BOOLEAN_ZERO_TOLERANCE) {
		// the lines are almost parallel
		return false;
	} else {
		// get the closest points on the infinite lines
		dFloat64 t = b * e - c * d;
		dFloat64 s = a * e - b * d;

		if (t < (DG_BOOLEAN_ZERO_TOLERANCE * den) || (s < (DG_BOOLEAN_ZERO_TOLERANCE * den)) || (t > (den - DG_BOOLEAN_ZERO_TOLERANCE)) ||  (s > (den - DG_BOOLEAN_ZERO_TOLERANCE))) {
			return false;
		}
		//dBigVector normal (p1p0 * q1q0);
		dBigVector r0 (ray_p0 + p1p0.Scale (t / den));
		dBigVector r1 (ray_q0 + q1q0.Scale (s / den));
		dBigVector r1r0 (r1 - r0);
		dFloat64 dist2 = r1r0 % r1r0;
		if (dist2 > (DG_BOOLEAN_ZERO_TOLERANCE * DG_BOOLEAN_ZERO_TOLERANCE)) {
			return false;
		}

		param = t / den;
		otherParam = s / den;
	}
	return true;
}
*/


dFloat64 ndMeshEffect::dMeshBVH::RayFaceIntersect(const dgMeshBVHNode* const faceNode, const dBigVector& p0, const dBigVector& p1, void* const userData) const
{
	dAssert(0);
	return 0;
	/*
		dBigVector normal (m_mesh->FaceNormal(faceNode->m_face, m_mesh->GetVertexPool(), sizeof(dBigVector)));

		dBigVector diff (p1 - p0);

		dFloat64 tOut = 2.0f;
		const dBigVector* const points = (dBigVector*) m_mesh->GetVertexPool();
		dFloat64 dir = normal.DotProduct3(diff);
		if (dir < 0.0f) {
			dEdge* ptr = faceNode->m_face;
			do {
				dInt32 index0 = ptr->m_incidentVertex;
				dInt32 index1 = ptr->m_next->m_incidentVertex;
				dBigVector p0v0 (points[index0] - p0);
				dBigVector p0v1 (points[index1] - p0);
				dFloat64 alpha = p0v0.DotProduct3(diff.CrossProduct(p0v1));
				if (alpha <= 0.0f) {
					return 1.2f;
				}

				ptr = ptr->m_next;
			} while (ptr != faceNode->m_face);

			dInt32 index0 = ptr->m_incidentVertex;
			dBigVector p0v0 (points[index0] - p0);
			tOut = normal.DotProduct3(p0v0);
			dFloat64 dist = normal.DotProduct3(diff);
			tOut = tOut / dist;

		} else if (doubleSidedFaces && (dir > 0.0f)) {
			dEdge* ptr = faceNode->m_face;
			do {
				dInt32 index0 = ptr->m_incidentVertex;
				dInt32 index1 = ptr->m_prev->m_incidentVertex;
				dBigVector p0v0 (points[index0] - p0);
				dBigVector p0v1 (points[index1] - p0);
				dFloat64 alpha = p0v0.DotProduct3(diff.CrossProduct(p0v1));
				if (alpha <= 0.0f) {
					return 1.2f;
				}

				ptr = ptr->m_prev;
			} while (ptr != faceNode->m_face);

			dInt32 index0 = ptr->m_incidentVertex;
			dBigVector p0v0 (points[index0] - p0);
			tOut = normal.DotProduct3(p0v0);
			dFloat64 dist = normal.DotProduct3(diff);
			tOut = tOut / dist;
		}

		if (tOut < 1.e-12f) {
			tOut = 2.0f;
		} else if (tOut > (1.0 - 1.e-12f)) {
			tOut = 2.0f;
		}
		return tOut;
	*/
}


void ndMeshEffect::dMeshBVH::FaceRayCast(const dBigVector& p0, const dBigVector& p1, void* const userData) const
{
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	dInt32 stack = 1;
	dgMeshBVHNode* node = nullptr;

	stackPool[0] = m_rootNode;
	dFloat64 maxParam = dFloat32(1.2f);

	dVector l0(p0);
	dVector l1(p1);
	l0 = l0 & dVector::m_triplexMask;
	l1 = l1 & dVector::m_triplexMask;
	dFastRayTest ray(l0, l1);
	while (stack) {
		stack--;
		dgMeshBVHNode* const me = stackPool[stack];
		if (me && ray.BoxTest(me->m_p0, me->m_p1)) {
			if (!me->m_left) {
				dAssert(!me->m_right);
				dFloat64 param = RayFaceIntersect(me, p0, p1, userData);
				if (param < dFloat64(0.0f)) {
					break;
				}
				if (param < maxParam) {
					node = me;
					maxParam = param;
				}
			}
			else {
				dAssert(me->m_left);
				dAssert(me->m_right);
				stackPool[stack] = me->m_left;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dAssert(stack < dInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));
			}
		}
	}
}

ndMeshEffect::ndMeshEffect(dgMemoryAllocator* const allocator, const dMatrix& planeMatrix, dFloat32 witdth, dFloat32 breadth, dInt32 material, const dMatrix& textureMatrix0, const dMatrix& textureMatrix1)
	:dPolyhedra(allocator)
	, m_points(allocator)
	, m_attrib(allocator)
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	dAssert(0);
	Init();
	/*
		dInt32 index[4];
		dInt64 attrIndex[4];
		dBigVector face[4];

	//	Init();

		face[0] = dBigVector (dFloat32 (0.0f), -witdth, -breadth, dFloat32 (0.0f));
		face[1] = dBigVector (dFloat32 (0.0f),  witdth, -breadth, dFloat32 (0.0f));
		face[2] = dBigVector (dFloat32 (0.0f),  witdth,  breadth, dFloat32 (0.0f));
		face[3] = dBigVector (dFloat32 (0.0f), -witdth,  breadth, dFloat32 (0.0f));

		for (dInt32 i = 0; i < 4; i ++) {
			dBigVector uv0 (textureMatrix0.TransformVector(face[i]));
			dBigVector uv1 (textureMatrix1.TransformVector(face[i]));

			m_points[i] = planeMatrix.TransformVector(face[i]);

			m_attrib[i].m_vertex.m_x = m_points[i].m_x;
			m_attrib[i].m_vertex.m_y = m_points[i].m_y;
			m_attrib[i].m_vertex.m_z = m_points[i].m_z;
			m_attrib[i].m_vertex.m_w = dFloat64 (0.0f);

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
	*/
}



ndMeshEffect::ndMeshEffect(const ndMeshEffect& source)
	:dPolyhedra(source)
	, m_points(source.m_points)
	, m_attrib(source.m_attrib)
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	Init();
}

ndMeshEffect::ndMeshEffect(dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData)
	:dPolyhedra(allocator)
	, m_points(allocator)
	, m_attrib(allocator)
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	Init();
	dAssert(0);
	/*
		dInt32 faceCount;
		deserialization (userData, &faceCount, sizeof (dInt32));
		deserialization (userData, &m_pointCount, sizeof (dInt32));
		deserialization (userData, &m_atribCount, sizeof (dInt32));
		deserialization (userData, &m_atribCount, sizeof (dInt32));

		m_maxPointCount = m_pointCount;
		m_maxAtribCount = m_atribCount;

		m_points = (dBigVector*) GetAllocator()->MallocLow(dInt32 (m_pointCount * sizeof(dBigVector)));
		m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(dInt32 (m_atribCount * sizeof(dgVertexAtribute)));

		deserialization (userData, m_points, m_pointCount * sizeof (dBigVector));
		deserialization (userData, m_attrib, m_atribCount * sizeof (dgVertexAtribute));

		BeginFace();
		for (dInt32 i = 0; i < faceCount; i ++) {
			dInt32 vertexCount;
			dInt32 face[1024];
			dInt64 attrib[1024];
			deserialization (userData, &vertexCount, sizeof (dInt32));
			deserialization (userData, face, vertexCount * sizeof (dInt32));
			deserialization (userData, attrib, vertexCount * sizeof (dInt64));
			AddFace (vertexCount, face, attrib);
		}
		EndFace();
	*/
}


ndMeshEffect::~ndMeshEffect(void)
{
}



void ndMeshEffect::Trace() const
{
	dAssert(0);
	/*
		for (dInt32 i = 0; i < m_pointCount; i ++ ) {
			dTrace (("%d-> %f %f %f\n", i, m_points[i].m_x, m_points[i].m_y, m_points[i].m_z));
		}


		dTree<dEdge*, dEdge*>filter(GetAllocator());
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			dEdge* const edge = &iter.GetNode()->GetInfo();
			if (!filter.Find(edge)) {
				dEdge* ptr = edge;
				do {
					filter.Insert(edge, ptr);
					dTrace (("%d ", ptr->m_incidentVertex));
					ptr = ptr->m_next;
				} while (ptr != edge);
				if (edge->m_incidentFace <= 0) {
					dTrace (("open"));
				}
				dTrace (("\n"));
			}
		}
		dTrace (("\n"));
	*/
};

void ndMeshEffect::FlipWinding()
{
	dInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
	dInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];

	dPolyhedra polyhedra(*this);
	RemoveAll();

	dPolyhedra::BeginFace();
	dInt32 mark = polyhedra.IncLRU();
	dPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) {
		dEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {

			dEdge* ptr = face;
			dInt32 indexCount = 0;
			do {
				index[indexCount] = ptr->m_incidentVertex;
				userData[indexCount] = ptr->m_userData;
				ptr->m_mark = mark;
				indexCount++;
				ptr = ptr->m_prev;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	dPolyhedra::EndFace();
}


void ndMeshEffect::Triangulate()
{
	/*
		dInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
		dInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];
		dPolyhedra polygon(GetAllocator());
		polygon.BeginFace();
		dInt32 mark = IncLRU();
		dPolyhedra::Iterator iter1 (*this);
		for (iter1.Begin(); iter1; iter1 ++) {
			dEdge* const face = &iter1.GetNode()->GetInfo();
			if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
				dEdge* ptr = face;
				dInt32 indexCount = 0;
				do {
					index[indexCount] = dInt32 (ptr->m_userData);
					userData[indexCount] = ptr->m_incidentVertex;
					ptr->m_mark = mark;
					indexCount ++;
					ptr = ptr->m_next;
				} while (ptr != face);
				polygon.AddFace(indexCount, index, userData);
			}
		}
		polygon.EndFace();

		dPolyhedra leftOversOut(GetAllocator());
		polygon.Triangulate(&m_points.m_vertex[0].m_x, sizeof (dBigVector), &leftOversOut);
		dAssert (leftOversOut.GetCount() == 0);

		SetLRU (0);
		RemoveAll();
		BeginFace();
		mark = polygon.IncLRU();
		dPolyhedra::Iterator iter (polygon);
		for (iter.Begin(); iter; iter ++){
			dEdge* const face = &iter.GetNode()->GetInfo();
			if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
				dEdge* ptr = face;
				dInt32 indexCount = 0;
				do {
					ptr->m_mark = mark;
					index[indexCount] = dInt32 (ptr->m_userData);
					userData[indexCount] = ptr->m_incidentVertex;
					indexCount ++;
					ptr = ptr->m_next;
				} while (ptr != face);
				AddFace(indexCount, index, userData);
			}
		}
		EndFace();
	*/
	UnpackPoints();
	dPolyhedra leftOversOut(GetAllocator());
	dPolyhedra::Triangulate(&m_points.m_vertex[0].m_x, sizeof(dBigVector), &leftOversOut);
	dAssert(leftOversOut.GetCount() == 0);

	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dEdge* const edge = &iter.GetNode()->GetInfo();
		edge->m_userData = (edge->m_incidentFace) > 0 ? edge->m_incidentVertex : 0;
	}
	PackPoints(dFloat32(1.0e-24f));

	RepairTJoints();
	dAssert(Sanity());
}



void ndMeshEffect::OptimizePoints()
{

}

void ndMeshEffect::OptimizeAttibutes()
{
	UnpackAttibuteData();
	PackAttibuteData();
}


dInt32 ndMeshEffect::GetTotalFaceCount() const
{
	return GetFaceCount();
}

dInt32 ndMeshEffect::GetTotalIndexCount() const
{
	Iterator iter(*this);
	dInt32 count = 0;
	dInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter++) {
		dEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		dEdge* ptr = edge;
		do {
			count++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}

void ndMeshEffect::GetFaces(dInt32* const facesIndex, dInt32* const materials, void** const faceNodeList) const
{
	Iterator iter(*this);

	dInt32 faces = 0;
	dInt32 indexCount = 0;
	dInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter++) {
		dEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		dInt32 faceCount = 0;
		dEdge* ptr = edge;
		do {
			//			indexList[indexCount] = dInt32 (ptr->m_userData);
			faceNodeList[indexCount] = GetNodeFromInfo(*ptr);
			indexCount++;
			faceCount++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);

		facesIndex[faces] = faceCount;
		//materials[faces] = dgFastInt(m_attrib[dInt32 (edge->m_userData)].m_material);
		materials[faces] = m_attrib.m_materialChannel.m_count ? m_attrib.m_materialChannel[dInt32(edge->m_userData)] : 0;
		faces++;
	}
}

void* ndMeshEffect::GetFirstVertex() const
{
	Iterator iter(*this);
	iter.Begin();

	dTreeNode* node = nullptr;
	if (iter) {
		dInt32 mark = IncLRU();
		node = iter.GetNode();

		dEdge* const edge = &node->GetInfo();
		dEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);
	}
	return node;
}

void* ndMeshEffect::GetNextVertex(const void* const vertex) const
{
	dTreeNode* const node0 = (dTreeNode*)vertex;
	dInt32 mark = node0->GetInfo().m_mark;

	Iterator iter(*this);
	iter.Set(node0);
	for (iter++; iter; iter++) {
		dTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			dEdge* const edge = &node->GetInfo();
			dEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			return node;
		}
	}
	return nullptr;
}

dInt32 ndMeshEffect::GetVertexIndex(const void* const vertex) const
{
	dTreeNode* const node = (dTreeNode*)vertex;
	dEdge* const edge = &node->GetInfo();
	return edge->m_incidentVertex;
}

void* ndMeshEffect::GetFirstPoint() const
{
	Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dTreeNode* const node = iter.GetNode();
		dEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node;
		}
	}
	return nullptr;
}

void* ndMeshEffect::GetNextPoint(const void* const point) const
{
	Iterator iter(*this);
	iter.Set((dTreeNode*)point);
	for (iter++; iter; iter++) {
		dTreeNode* const node = iter.GetNode();
		dEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node;
		}
	}
	return nullptr;
}

dInt32 ndMeshEffect::GetPointIndex(const void* const point) const
{
	dTreeNode* const node = (dTreeNode*)point;
	dEdge* const edge = &node->GetInfo();
	return dInt32(edge->m_userData);
}


dInt32 ndMeshEffect::GetVertexIndexFromPoint(const void* const point) const
{
	return GetVertexIndex(point);
}

dEdge* ndMeshEffect::SpliteFace(dInt32 v0, dInt32 v1)
{
	if (!FindEdge(v0, v1)) {
		dPolyhedra::dgPairKey key(v0, 0);
		dTreeNode* const node = FindGreaterEqual(key.GetVal());
		if (node) {
			dEdge* const edge = &node->GetInfo();
			dEdge* edge0 = edge;
			do {
				if (edge0->m_incidentFace > 0) {
					for (dEdge* edge1 = edge0->m_next->m_next; edge1 != edge0->m_prev; edge1 = edge1->m_next) {
						if (edge1->m_incidentVertex == v1) {
							return ConnectVertex(edge0, edge1);
						}
					};
				}
				edge0 = edge0->m_twin->m_next;
			} while (edge0 != edge);
		}
	}
	return nullptr;
}

const dEdge* ndMeshEffect::GetPolyhedraEdgeFromNode(const void* const edge) const
{
	dTreeNode* const node = (dTreeNode*)edge;
	return &node->GetInfo();
}

void* ndMeshEffect::GetFirstEdge() const
{
	Iterator iter(*this);
	iter.Begin();

	dTreeNode* node = nullptr;
	if (iter) {
		dInt32 mark = IncLRU();

		node = iter.GetNode();

		dEdge* const edge = &node->GetInfo();
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}
	return node;
}

void* ndMeshEffect::GetNextEdge(const void* const edge) const
{
	dTreeNode* const node0 = (dTreeNode*)edge;
	dInt32 mark = node0->GetInfo().m_mark;

	Iterator iter(*this);
	iter.Set(node0);
	for (iter++; iter; iter++) {
		dTreeNode* const node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			node->GetInfo().m_mark = mark;
			node->GetInfo().m_twin->m_mark = mark;
			return node;
		}
	}
	return nullptr;
}


void ndMeshEffect::GetEdgeIndex(const void* const edge, dInt32& v0, dInt32& v1) const
{
	dTreeNode* const node = (dTreeNode*)edge;
	v0 = node->GetInfo().m_incidentVertex;
	v1 = node->GetInfo().m_twin->m_incidentVertex;
}

//void* ndMeshEffect::FindEdge (dInt32 v0, dInt32 v1) const
//{
//	return FindEdgeNode(v0, v1);
//}

//void ndMeshEffect::GetEdgeAttributeIndex (const void* edge, dInt32& v0, dInt32& v1) const
//{
//	dTreeNode* node = (dTreeNode*) edge;
//	v0 = dInt32 (node->GetInfo().m_userData);
//	v1 = dInt32 (node->GetInfo().m_twin->m_userData);
//}


void* ndMeshEffect::GetFirstFace() const
{
	Iterator iter(*this);
	iter.Begin();

	dTreeNode* node = nullptr;
	if (iter) {
		dInt32 mark = IncLRU();
		node = iter.GetNode();

		dEdge* const edge = &node->GetInfo();
		dEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}

	return node;
}

void* ndMeshEffect::GetNextFace(const void* const face) const
{
	dTreeNode* const node0 = (dTreeNode*)face;
	dInt32 mark = node0->GetInfo().m_mark;

	Iterator iter(*this);
	iter.Set(node0);
	for (iter++; iter; iter++) {
		dTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			dEdge* const edge = &node->GetInfo();
			dEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
			return node;
		}
	}
	return nullptr;
}


dInt32 ndMeshEffect::IsFaceOpen(const void* const face) const
{
	dTreeNode* const node = (dTreeNode*)face;
	dEdge* const edge = &node->GetInfo();
	return (edge->m_incidentFace > 0) ? 0 : 1;
}

dInt32 ndMeshEffect::GetFaceMaterial(const void* const face) const
{
	dTreeNode* const node = (dTreeNode*)face;
	dEdge* const edge = &node->GetInfo();
	return dInt32(m_attrib.m_materialChannel.m_count ? m_attrib.m_materialChannel[dInt32(edge->m_userData)] : 0);
}

void ndMeshEffect::SetFaceMaterial(const void* const face, dInt32 mateialID)
{
	if (m_attrib.m_materialChannel.m_count) {
		dTreeNode* const node = (dTreeNode*)face;
		dEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			dEdge* ptr = edge;
			do {
				//dgVertexAtribute* const attrib = &m_attrib[ptr->m_userData];
				//attrib->m_material = dFloat64 (mateialID);
				m_attrib.m_materialChannel[dInt32(edge->m_userData)] = mateialID;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
}

dInt32 ndMeshEffect::GetFaceIndexCount(const void* const face) const
{
	dInt32 count = 0;
	dTreeNode* node = (dTreeNode*)face;
	dEdge* const edge = &node->GetInfo();
	dEdge* ptr = edge;
	do {
		count++;
		ptr = ptr->m_next;
	} while (ptr != edge);
	return count;
}

void ndMeshEffect::GetFaceIndex(const void* const face, dInt32* const indices) const
{
	dInt32 count = 0;
	dTreeNode* node = (dTreeNode*)face;
	dEdge* const edge = &node->GetInfo();
	dEdge* ptr = edge;
	do {
		indices[count] = ptr->m_incidentVertex;
		count++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}

void ndMeshEffect::GetFaceAttributeIndex(const void* const face, dInt32* const indices) const
{
	dInt32 count = 0;
	dTreeNode* node = (dTreeNode*)face;
	dEdge* const edge = &node->GetInfo();
	dEdge* ptr = edge;
	do {
		indices[count] = dInt32(ptr->m_userData);
		count++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}


dBigVector ndMeshEffect::CalculateFaceNormal(const void* const face) const
{
	dTreeNode* const node = (dTreeNode*)face;
	dEdge* const faceEdge = &node->GetInfo();
	dBigVector normal(FaceNormal(faceEdge, &m_points.m_vertex[0].m_x, sizeof(dBigVector)));
	dAssert(normal.m_w == dFloat32(0.0f));
	//normal = normal.Scale (1.0f / sqrt (normal.DotProduct3(normal)));
	normal = normal.Normalize();
	return normal;
}

/*
dInt32 GetTotalFaceCount() const;
{
	dInt32 mark;
	dInt32 count;
	dInt32 materialCount;
	dInt32 materials[256];
	dInt32 streamIndexMap[256];
	ndIndexArray* array;

	count = 0;
	materialCount = 0;

	array = (ndIndexArray*) GetAllocator()->MallocLow (4 * sizeof (dInt32) * GetCount() + sizeof (ndIndexArray) + 2048);
	array->m_indexList = (dInt32*)&array[1];

	mark = IncLRU();
	dPolyhedra::Iterator iter (*this);
	memset(streamIndexMap, 0, sizeof (streamIndexMap));
	for(iter.Begin(); iter; iter ++){

		dEdge* const edge;
		edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark)) {
			dEdge* ptr;
			dInt32 hashValue;
			dInt32 index0;
			dInt32 index1;

			ptr = edge;
			ptr->m_mark = mark;
			index0 = dInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			index1 = dInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			do {
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = dInt32 (ptr->m_userData);
				array->m_indexList[count * 4 + 3] = m_attrib[dInt32 (edge->m_userData)].m_material;
				index1 = dInt32 (ptr->m_userData);

				hashValue = array->m_indexList[count * 4 + 3] & 0xff;
				streamIndexMap[hashValue] ++;
				materials[hashValue] = array->m_indexList[count * 4 + 3];
				count ++;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
}
*/

bool ndMeshEffect::HasNormalChannel() const
{
	return m_attrib.m_normalChannel.m_count != 0;
}

bool ndMeshEffect::HasBinormalChannel() const
{
	return m_attrib.m_binormalChannel.m_count != 0;
}

bool ndMeshEffect::HasUV0Channel() const
{
	return m_attrib.m_uv0Channel.m_count != 0;
}

bool ndMeshEffect::HasUV1Channel() const
{
	return m_attrib.m_uv1Channel.m_count != 0;
}

bool ndMeshEffect::HasVertexColorChannel() const
{
	return m_attrib.m_colorChannel.m_count != 0;
}


/*
void ndMeshEffect::GetWeightBlendChannel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt8* const buffer = (dInt8*)bufferOut;
	for (dInt32 i = 0; i < m_attrib.m_pointChannel.m_count; i++) {
		const dInt32 j = i * strideInByte;
		dFloat32* const ptr = (dFloat32*)&buffer[j];

		const dInt32 index = m_attrib.m_pointChannel[i];
		const dFloat32* const p = &m_points.m_weights[index].m_weightBlends[0];
		ptr[0] = dFloat32(p[0]);
		ptr[1] = dFloat32(p[1]);
		ptr[2] = dFloat32(p[2]);
		ptr[3] = dFloat32(p[3]);
	}
}

void ndMeshEffect::GetWeightIndexChannel(dInt32 strideInByte, dInt32* const bufferOut) const
{
	dInt8* const buffer = (dInt8*)bufferOut;
	for (dInt32 i = 0; i < m_attrib.m_pointChannel.m_count; i++) {
		const dInt32 j = i * strideInByte;
		dInt32* const ptr = (dInt32*)&buffer[j];
		const dInt32 index = m_attrib.m_pointChannel[i];
		const dInt32* const p = &m_points.m_weights[index].m_controlIndex[0];
		ptr[0] = p[0];
		ptr[1] = p[1];
		ptr[2] = p[2];
		ptr[3] = p[3];
	}
}
*/


dgCollisionInstance* ndMeshEffect::CreateCollisionTree(dgWorld* const world, dInt32 shapeID) const
{
	ndShapeStaticBVH* const collision = new  (GetAllocator()) ndShapeStaticBVH(world);

	collision->BeginBuild();

	dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dTreeNode* const faceNode = iter.GetNode();
		//dEdge* const face = &(*iter);
		dEdge* const face = &faceNode->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dInt32 count = 0;
			dVector polygon[256];
			dEdge* ptr = face;
			do {
				//polygon[count] = dVector (m_points[ptr->m_incidentVertex]);
				polygon[count] = GetVertex(ptr->m_incidentVertex);
				count++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);
			//collision->AddFace(count, &polygon[0].m_x, sizeof (dVector), dInt32 (m_attrib[face->m_userData].m_material));
			collision->AddFace(count, &polygon[0].m_x, sizeof(dVector), GetFaceMaterial(faceNode));
		}
	}
	collision->EndBuild(0);

	dgCollisionInstance* const instance = world->CreateInstance(collision, shapeID, dGetIdentityMatrix());
	collision->Release();
	return instance;
}

void ndMeshEffect::TransformMesh(const dMatrix& matrix)
{
	dAssert(0);
	/*
	dMatrix normalMatrix (matrix);
	normalMatrix.m_posit = dVector (dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (1.0f));

	matrix.TransformTriplex (&m_points->m_x, sizeof (dBigVector), &m_points->m_x, sizeof (dBigVector), m_pointCount);
	matrix.TransformTriplex (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), m_atribCount);
	normalMatrix.TransformTriplex (&m_attrib[0].m_normal_x, sizeof (dgVertexAtribute), &m_attrib[0].m_normal_x, sizeof (dgVertexAtribute), m_atribCount);
*/
}



//ndMeshEffect::dgVertexAtribute ndMeshEffect::InterpolateVertex (const dBigVector& srcPoint, const dEdge* const face) const
dInt32 ndMeshEffect::InterpolateVertex(const dBigVector& srcPoint, const dEdge* const face) const
{
	dAssert(0);
	return 0;
	/*
		const dBigVector point (srcPoint);

		dgVertexAtribute attribute;
		memset (&attribute, 0, sizeof (attribute));

	//	dBigVector normal (FaceNormal(face, &m_points[0].m_x, sizeof(dBigVector)));
	//	normal = normal.Scale (dFloat64 (1.0f) / sqrt (normal % normal));
	//	attribute.m_vertex = srcPoint;
	//	attribute.m_normal_x = normal.m_x;
	//	attribute.m_normal_y = normal.m_y;
	//	attribute.m_normal_z = normal.m_z;

		dFloat64 tol = dFloat32 (1.0e-4f);
		for (dInt32 i = 0; i < 4; i ++) {
			const dEdge* ptr = face;
			const dEdge* const edge0 = ptr;
			dBigVector q0 (m_points[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dEdge* edge1 = ptr;
			dBigVector q1 (m_points[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dEdge* edge2 = ptr;
			do {
				const dBigVector q2 (m_points[ptr->m_incidentVertex]);

				dBigVector p10 (q1 - q0);
				dBigVector p20 (q2 - q0);

				dFloat64 dot = p20.DotProduct3(p10);
				dFloat64 mag1 = p10.DotProduct3(p10);
				dFloat64 mag2 = p20.DotProduct3(p20);
				dFloat64 collinear = dot * dot - mag2 * mag1;
				if (fabs (collinear) > dFloat64 (1.0e-8f)) {
					dBigVector p_p0 (point - q0);
					dBigVector p_p1 (point - q1);
					dBigVector p_p2 (point - q2);

					dFloat64 alpha1 = p10.DotProduct3(p_p0);
					dFloat64 alpha2 = p20.DotProduct3(p_p0);
					dFloat64 alpha3 = p10.DotProduct3(p_p1);
					dFloat64 alpha4 = p20.DotProduct3(p_p1);
					dFloat64 alpha5 = p10.DotProduct3(p_p2);
					dFloat64 alpha6 = p20.DotProduct3(p_p2);

					dFloat64 vc = alpha1 * alpha4 - alpha3 * alpha2;
					dFloat64 vb = alpha5 * alpha2 - alpha1 * alpha6;
					dFloat64 va = alpha3 * alpha6 - alpha5 * alpha4;
					dFloat64 den = va + vb + vc;
					dFloat64 minError = den * (-tol);
					dFloat64 maxError = den * (dFloat32 (1.0f) + tol);
					if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {
						edge2 = ptr;

						den = dFloat64 (1.0f) / (va + vb + vc);

						dFloat64 alpha0 = dFloat32 (va * den);
						dFloat64 alpha1 = dFloat32 (vb * den);
						dFloat64 alpha2 = dFloat32 (vc * den);

						const dgVertexAtribute& attr0 = m_attrib[edge0->m_userData];
						const dgVertexAtribute& attr1 = m_attrib[edge1->m_userData];
						const dgVertexAtribute& attr2 = m_attrib[edge2->m_userData];
						dBigVector normal (attr0.m_normal_x * alpha0 + attr1.m_normal_x * alpha1 + attr2.m_normal_x * alpha2,
											attr0.m_normal_y * alpha0 + attr1.m_normal_y * alpha1 + attr2.m_normal_y * alpha2,
											attr0.m_normal_z * alpha0 + attr1.m_normal_z * alpha1 + attr2.m_normal_z * alpha2, dFloat32 (0.0f));
						//normal = normal.Scale (dFloat64 (1.0f) / sqrt (normal.DotProduct3(normal)));
						normal = normal.Normalize();

			#ifdef _DEBUG
						dBigVector testPoint (attr0.m_vertex.m_x * alpha0 + attr1.m_vertex.m_x * alpha1 + attr2.m_vertex.m_x * alpha2,
											   attr0.m_vertex.m_y * alpha0 + attr1.m_vertex.m_y * alpha1 + attr2.m_vertex.m_y * alpha2,
											   attr0.m_vertex.m_z * alpha0 + attr1.m_vertex.m_z * alpha1 + attr2.m_vertex.m_z * alpha2, dFloat32 (0.0f));
						dAssert (fabs (testPoint.m_x - point.m_x) < dFloat32 (1.0e-2f));
						dAssert (fabs (testPoint.m_y - point.m_y) < dFloat32 (1.0e-2f));
						dAssert (fabs (testPoint.m_z - point.m_z) < dFloat32 (1.0e-2f));
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
						dAssert (attr0.m_material == attr1.m_material);
						dAssert (attr0.m_material == attr2.m_material);
						return attribute;
					}
				}

				q1 = q2;
				edge1 = ptr;

				ptr = ptr->m_next;
			} while (ptr != face);
			tol *= dFloat64 (2.0f);
		}
		// this should never happens
		dAssert (0);
		return attribute;
	*/
}


#endif

dInt32 ndMeshEffect::dFormat::CompareVertex(const dSortKey* const ptr0, const dSortKey* const ptr1, void* const context)
{
	const dVertexSortData* const sortContext = (dVertexSortData*)context;
	const dInt32 compIndex = sortContext->m_vertexSortIndex;
	const dChannel<dBigVector, m_point>& points = *sortContext->m_points;
	const dFloat64 x0 = points[ptr0->m_vertexIndex][compIndex];
	const dFloat64 x1 = points[ptr1->m_vertexIndex][compIndex];

	if (x0 < x1)
	{
		return -1;
	}
	else if (x0 > x1)
	{
		return 1;
	}
	return 0;
}

dInt32 ndMeshEffect::dFormat::GetSortIndex(const dChannel<dBigVector, m_point>& points, dFloat64& dist) const
{
	dBigVector xc(dBigVector::m_zero);
	dBigVector x2c(dBigVector::m_zero);
	dBigVector minP(dFloat64(1.0e20f));
	dBigVector maxP(dFloat64(-1.0e20f));
	for (dInt32 i = 0; i < points.GetCount(); i++)
	{
		dBigVector x(points[i]);
		xc += x;
		x2c += x * x;
		minP = minP.GetMin(x);
		maxP = maxP.GetMax(x);
	}

	dBigVector del(maxP - minP);
	dFloat64 minDist = dMin(del.m_x, del.m_y, del.m_z);
	if (minDist < dFloat64(1.0e-3f))
	{
		minDist = dFloat64(1.0e-3f);
	}

	dInt32 firstSortAxis = 0;
	x2c = x2c.Scale(points.GetCount()) - xc * xc;
	if ((x2c.m_y >= x2c.m_x) && (x2c.m_y >= x2c.m_z))
	{
		firstSortAxis = 1;
	}
	else if ((x2c.m_z >= x2c.m_x) && (x2c.m_z >= x2c.m_y))
	{
		firstSortAxis = 2;
	}
	dist = minDist;
	return firstSortAxis;
}

void ndMeshEffect::dPointFormat::CompressData(dInt32* const indexList)
{
	dFloat64 minDist;
	const dInt32 firstSortAxis = GetSortIndex(m_vertex, minDist);
	
	dStack<dFormat::dSortKey> indirectListBuffer(m_vertex.GetCount());
	dFormat::dSortKey* indirectList = &indirectListBuffer[0];
	for (dInt32 i = 0; i < m_vertex.GetCount(); i++) 
	{
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_vertexIndex = i;
		indirectList[i].m_attibuteIndex = -1;
	}
	
	dPointFormat tmpFormat(*this);
	dVertexSortData sortContext;
	sortContext.m_points = &tmpFormat.m_vertex;
	//sortContext.m_points = &tmpFormat;
	sortContext.m_vertexSortIndex = firstSortAxis;
	dSort(indirectList, m_vertex.GetCount(), dFormat::CompareVertex, &sortContext);
	
	const dFloat64 tolerance = dMin(minDist, dFloat64(1.0e-12f));
	const dFloat64 sweptWindow = dFloat64(2.0f) * tolerance + dFloat64(1.0e-10f);
	
	dInt32 newCount = 0;
	for (dInt32 i = 0; i < tmpFormat.m_vertex.GetCount(); i++)
	{
		const dInt32 ii = indirectList[i].m_mask;
		if (ii == -1) 
		{
			const dInt32 i0 = indirectList[i].m_ordinal;
			const dInt32 iii = indirectList[i].m_vertexIndex;
			const dBigVector& p = tmpFormat.m_vertex[iii];
			//const dFloat64 swept = tmpFormat.m_vertex[iii][firstSortAxis] + sweptWindow;
			const dFloat64 swept = p[firstSortAxis] + sweptWindow;
			for (dInt32 j = i + 1; j < tmpFormat.m_vertex.GetCount(); j++)
			{
	
				const dInt32 jj = indirectList[j].m_mask;
				if (jj == -1) 
				{
					const dInt32 j0 = indirectList[j].m_ordinal;
					const dInt32 jjj = indirectList[j].m_vertexIndex;
					const dBigVector& q = tmpFormat.m_vertex[jjj];
					//dFloat64 val = tmpFormat.m_vertex[jjj][firstSortAxis];
					dFloat64 val = q[firstSortAxis];
					if (val >= swept) 
					{
						break;
					}
	
					bool test = true;
					if (iii != jjj) 
					{
						//dBigVector dp(tmpFormat.m_vertex[iii] - tmpFormat.m_vertex[jjj]);
						dBigVector dp(p - q);
						for (dInt32 k = 0; k < 3; k++) 
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}
					if (test && tmpFormat.m_layers.GetCount())
					{
						test &= (tmpFormat.m_layers[i0] == tmpFormat.m_layers[j0]);
					}
					// note, is ok weight duplicate to be ignored.
	
					if (test) 
					{
						indirectList[j].m_mask = newCount;
					}
				}
			}
	
			//indirectList[newCount].m_attibuteIndex = indirectList[i].m_attibuteIndex;
			indirectList[newCount].m_vertexIndex = indirectList[i].m_vertexIndex;
			indirectList[i].m_mask = newCount;
			newCount++;
		}
	}
	
	Clear();
	for (dInt32 i = 0; i < newCount; i++) 
	{
		dAssert(indirectList[i].m_attibuteIndex == -1);
		m_vertex.PushBack(tmpFormat.m_vertex[indirectList[i].m_vertexIndex]);
	}
	
	if (tmpFormat.m_layers.GetCount())
	{
		for (dInt32 i = 0; i < newCount; i++) 
		{
			m_layers.PushBack(tmpFormat.m_layers[indirectList[i].m_vertexIndex]);
		}
	}
	
	for (dInt32 i = 0; i < tmpFormat.m_vertex.GetCount(); i++) 
	{
		dInt32 i1 = indirectList[i].m_ordinal;
		dInt32 index = indirectList[i].m_mask;
		indexList[i1] = index;
	}
}

void ndMeshEffect::dAttibutFormat::CompressData(const dPointFormat& points, dInt32* const indexList)
{
	dFloat64 minDist;
	const dInt32 firstSortAxis = GetSortIndex(points.m_vertex, minDist);

	dStack<dFormat::dSortKey> indirectListBuffer(m_pointChannel.GetCount());
	dFormat::dSortKey* indirectList = &indirectListBuffer[0];
	for (dInt32 i = 0; i < m_pointChannel.GetCount(); i++)
	{
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_attibuteIndex = i;
		indirectList[i].m_vertexIndex = m_pointChannel[i];
	}

	dVertexSortData sortContext;
	sortContext.m_points = &points.m_vertex;
	sortContext.m_vertexSortIndex = firstSortAxis;
	dSort(indirectList, m_pointChannel.GetCount(), dFormat::CompareVertex, &sortContext);
	dAttibutFormat tmpFormat(*this);
	Clear();

	const dFloat64 tolerance = dMin(minDist, dFloat64(1.0e-12f));
	const dFloat64 sweptWindow = dFloat64(2.0f) * tolerance + dFloat64(1.0e-10f);

	dInt32 newCount = 0;
	for (dInt32 i = 0; i < tmpFormat.m_pointChannel.GetCount(); i++)
	{
		const dInt32 ii = indirectList[i].m_mask;
		if (ii == -1)
		{
			const dInt32 i0 = indirectList[i].m_ordinal;
			const dInt32 iii = indirectList[i].m_vertexIndex;
			const dFloat64 swept = points.m_vertex[iii][firstSortAxis] + sweptWindow;
			for (dInt32 j = i + 1; j < tmpFormat.m_pointChannel.GetCount(); j++)
			{
				const dInt32 jj = indirectList[j].m_mask;
				if (jj == -1)
				{
					const dInt32 j0 = indirectList[j].m_ordinal;
					const dInt32 jjj = indirectList[j].m_vertexIndex;;
					dFloat64 val = points.m_vertex[jjj][firstSortAxis];
					if (val >= swept)
					{
						break;
					}

					bool test = true;
					if (iii != jjj)
					{
						dBigVector dp(points.m_vertex[iii] - points.m_vertex[jjj]);
						for (dInt32 k = 0; k < 3; k++)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}
					if (test && points.m_layers.m_isValid)
					{
						test &= (points.m_layers[iii] == points.m_layers[jjj]);
					}

					if (test && tmpFormat.m_normalChannel.m_isValid)
					{
						dVector n0(tmpFormat.m_normalChannel[i0].m_x, tmpFormat.m_normalChannel[i0].m_y, tmpFormat.m_normalChannel[i0].m_z, dFloat32(0.0f));
						dVector n1(tmpFormat.m_normalChannel[j0].m_x, tmpFormat.m_normalChannel[j0].m_y, tmpFormat.m_normalChannel[j0].m_z, dFloat32(0.0f));
						dVector dp(n1 - n0);
						for (dInt32 k = 0; k < 3; k++)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && tmpFormat.m_binormalChannel.m_isValid)
					{
						dVector n0(tmpFormat.m_binormalChannel[i0].m_x, tmpFormat.m_binormalChannel[i0].m_y, tmpFormat.m_binormalChannel[i0].m_z, dFloat32(0.0f));
						dVector n1(tmpFormat.m_binormalChannel[j0].m_x, tmpFormat.m_binormalChannel[j0].m_y, tmpFormat.m_binormalChannel[j0].m_z, dFloat32(0.0f));
						dVector dp(n1 - n0);
						for (dInt32 k = 0; k < 3; k++)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && tmpFormat.m_uv0Channel.m_isValid)
					{
						dVector n0(tmpFormat.m_uv0Channel[i0].m_u, tmpFormat.m_uv0Channel[i0].m_v, dFloat32(0.0f), dFloat32(0.0f));
						dVector n1(tmpFormat.m_uv0Channel[j0].m_u, tmpFormat.m_uv0Channel[j0].m_v, dFloat32(0.0f), dFloat32(0.0f));
						dVector dp(n1 - n0);
						for (dInt32 k = 0; k < 2; k++)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && tmpFormat.m_uv1Channel.m_isValid)
					{
						dVector n0(tmpFormat.m_uv1Channel[i0].m_u, tmpFormat.m_uv1Channel[i0].m_v, dFloat32(0.0f), dFloat32(0.0f));
						dVector n1(tmpFormat.m_uv1Channel[j0].m_u, tmpFormat.m_uv1Channel[j0].m_v, dFloat32(0.0f), dFloat32(0.0f));
						dVector dp(n1 - n0);
						for (dInt32 k = 0; k < 2; k++)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && tmpFormat.m_colorChannel.m_isValid)
					{
						dVector dp(m_colorChannel[i0] - m_colorChannel[j0]);
						for (dInt32 k = 0; k < 3; k++)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && tmpFormat.m_materialChannel.m_isValid)
					{
						test &= (tmpFormat.m_materialChannel[i0] == tmpFormat.m_materialChannel[j0]);
					}

					if (test)
					{
						indirectList[j].m_mask = newCount;
					}
				}
			}

			indirectList[newCount].m_attibuteIndex = indirectList[i].m_attibuteIndex;
			indirectList[newCount].m_vertexIndex = indirectList[i].m_vertexIndex;
			indirectList[i].m_mask = newCount;
			newCount++;
		}
	}

	for (dInt32 i = 0; i < newCount; i++)
	{
		m_pointChannel.PushBack(indirectList[i].m_vertexIndex);
	}

	if (tmpFormat.m_normalChannel.m_isValid)
	{
		for (dInt32 i = 0; i < newCount; i++)
		{
			m_normalChannel.PushBack(tmpFormat.m_normalChannel[indirectList[i].m_attibuteIndex]);
		}
	}

	if (tmpFormat.m_binormalChannel.m_isValid)
	{
		for (dInt32 i = 0; i < newCount; i++)
		{
			m_binormalChannel.PushBack(tmpFormat.m_binormalChannel[indirectList[i].m_attibuteIndex]);
		}
	}

	if (tmpFormat.m_uv0Channel.m_isValid)
	{
		for (dInt32 i = 0; i < newCount; i++)
		{
			m_uv0Channel.PushBack(tmpFormat.m_uv0Channel[indirectList[i].m_attibuteIndex]);
		}
	}

	if (tmpFormat.m_uv1Channel.m_isValid)
	{
		for (dInt32 i = 0; i < newCount; i++)
		{
			m_uv1Channel.PushBack(tmpFormat.m_uv1Channel[indirectList[i].m_attibuteIndex]);
		}
	}

	if (tmpFormat.m_colorChannel.m_isValid)
	{
		for (dInt32 i = 0; i < newCount; i++)
		{
			m_colorChannel.PushBack(tmpFormat.m_colorChannel[indirectList[i].m_attibuteIndex]);
		}
	}

	if (tmpFormat.m_materialChannel.m_isValid)
	{
		for (dInt32 i = 0; i < newCount; i++)
		{
			m_materialChannel.PushBack(tmpFormat.m_materialChannel[indirectList[i].m_attibuteIndex]);
		}
	}

	for (dInt32 i = 0; i < tmpFormat.m_pointChannel.GetCount(); i++)
	{
		dInt32 i1 = indirectList[i].m_ordinal;
		dInt32 index = indirectList[i].m_mask;
		indexList[i1] = index;
	}
}

ndMeshEffect::ndMeshEffect()
	:dPolyhedra()
	, m_name()
	, m_points()
	, m_attrib()
	, m_materials()
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	Init();
}

ndMeshEffect::ndMeshEffect(dPolyhedra& mesh, const ndMeshEffect& source)
	:dPolyhedra(mesh)
	,m_points(source.m_points)
	,m_attrib(source.m_attrib)
	,m_vertexBaseCount(-1)
	,m_constructionIndex(0)
{
	Init();
}


ndMeshEffect::~ndMeshEffect()
{
}

void ndMeshEffect::Init()
{
}

bool ndMeshEffect::Sanity() const
{
	#ifdef  _DEBUG
	ndMeshEffect::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dEdge* const edge = &iter.GetNode()->GetInfo();
		dAssert(edge->m_twin);
		dAssert(edge->m_next);
		dAssert(edge->m_prev);
		dAssert(edge->m_twin->m_twin == edge);
		dAssert(edge->m_next->m_incidentVertex == edge->m_twin->m_incidentVertex);
		dAssert(edge->m_incidentVertex == edge->m_twin->m_next->m_incidentVertex);
	}
	#endif
	return true;
}

void ndMeshEffect::BeginBuildFace()
{
	m_constructionIndex = m_points.m_vertex.GetCount();
}

void ndMeshEffect::AddPoint(dFloat64 x, dFloat64 y, dFloat64 z)
{
	//m_attrib.m_pointChannel.PushBack(m_points.m_vertex.m_count);
	m_attrib.m_pointChannel.PushBack(m_points.m_vertex.GetCount());
	m_points.m_vertex.PushBack(dBigVector(QuantizeCordinade(x), QuantizeCordinade(y), QuantizeCordinade(z), dFloat64(0.0f)));
}

void ndMeshEffect::AddLayer(dInt32 layer)
{
	m_points.m_layers.PushBack(layer);
}

void ndMeshEffect::AddVertexColor(dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
{
	m_attrib.m_colorChannel.PushBack(dVector(x, y, z, w));
}

void ndMeshEffect::AddNormal(dFloat32 x, dFloat32 y, dFloat32 z)
{
	dTriplex n;
	n.m_x = x;
	n.m_y = y;
	n.m_z = z;
	m_attrib.m_normalChannel.PushBack(n);
}

void ndMeshEffect::AddBinormal(dFloat32 x, dFloat32 y, dFloat32 z)
{
	dTriplex n;
	n.m_x = x;
	n.m_y = y;
	n.m_z = z;
	m_attrib.m_binormalChannel.PushBack(n);
}

void ndMeshEffect::AddUV0(dFloat32 u, dFloat32 v)
{
	dAttibutFormat::dgUV uv;
	uv.m_u = u;
	uv.m_v = v;
	m_attrib.m_uv0Channel.PushBack(uv);
}

void ndMeshEffect::AddUV1(dFloat32 u, dFloat32 v)
{
	dAttibutFormat::dgUV uv;
	uv.m_u = u;
	uv.m_v = v;
	m_attrib.m_uv1Channel.PushBack(uv);
}

void ndMeshEffect::AddMaterial(dInt32 materialIndex)
{
	m_attrib.m_materialChannel.PushBack(materialIndex);
}

void ndMeshEffect::EndBuildFace()
{
	dInt32 count = m_points.m_vertex.GetCount() - m_constructionIndex;
	if (count > 3) 
	{
		dInt32 indexList[256];

		dAssert(count < dInt32(sizeof(indexList) / sizeof(indexList[0])));
		dPolyhedra polygon;
		dPointFormat points;
		dAttibutFormat attibutes;

		for (dInt32 i = 0; i < count; i++) 
		{
			indexList[i] = i;
		
			points.m_vertex.PushBack(m_points.m_vertex[m_constructionIndex + i]);
			if (m_points.m_layers.GetCount()) 
			{
				points.m_layers.PushBack(m_points.m_layers[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_materialChannel.GetCount())
			{
				attibutes.m_materialChannel.PushBack(m_attrib.m_materialChannel[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_normalChannel.GetCount())
			{
				attibutes.m_normalChannel.PushBack(m_attrib.m_normalChannel[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_binormalChannel.GetCount())
			{
				attibutes.m_binormalChannel.PushBack(m_attrib.m_binormalChannel[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_binormalChannel.GetCount())
			{
				attibutes.m_colorChannel.PushBack(m_attrib.m_colorChannel[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_uv0Channel.GetCount())
			{
				attibutes.m_uv0Channel.PushBack(m_attrib.m_uv0Channel[m_constructionIndex + i]);
			}
		
			if (attibutes.m_uv1Channel.GetCount())
			{
				attibutes.m_uv1Channel.PushBack(attibutes.m_uv1Channel[m_constructionIndex + i]);
			}
		}
		
		polygon.BeginFace();
		polygon.AddFace(count, indexList, nullptr);
		polygon.EndFace();
		polygon.Triangulate(&points.m_vertex[0].m_x, sizeof(dBigVector), nullptr);
		
		m_points.SetCount(m_constructionIndex);
		m_attrib.SetCount(m_constructionIndex);
		dInt32 mark = polygon.IncLRU();
		dPolyhedra::Iterator iter(polygon);
		for (iter.Begin(); iter; iter++) 
		{
			dEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) 
			{
				dInt32 i0 = edge->m_incidentVertex;
				dInt32 i1 = edge->m_next->m_incidentVertex;
				dInt32 i2 = edge->m_next->m_next->m_incidentVertex;
				edge->m_mark = mark;
				edge->m_next->m_mark = mark;
				edge->m_next->m_next->m_mark = mark;
		
				const dBigVector& p0 = points.m_vertex[i0];
				const dBigVector& p1 = points.m_vertex[i1];
				const dBigVector& p2 = points.m_vertex[i2];
		
				dBigVector e1(p1 - p0);
				dBigVector e2(p2 - p0);
				dBigVector n(e1.CrossProduct(e2));
		
				dAssert(e1.m_w == dFloat32(0.0f));
				dAssert(e2.m_w == dFloat32(0.0f));
				dAssert(n.m_w == dFloat32(0.0f));
		
				dFloat64 mag3 = n.DotProduct(n).GetScalar();
				if (mag3 >= dFloat64(DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)) 
				{
					dInt32 index[] = { i0, i1, i2 };
					for (dInt32 i = 0; i < 3; i++) 
					{
						dInt32 j = index[i];
						AddPoint(points.m_vertex[j].m_x, points.m_vertex[j].m_y, points.m_vertex[j].m_z);
						if (points.m_layers.GetCount())
						{
							AddLayer(points.m_layers[j]);
						}
		
						if (attibutes.m_materialChannel.GetCount()) 
						{
							AddMaterial(attibutes.m_materialChannel[j]);
						}
		
						if (attibutes.m_normalChannel.GetCount())
						{
							AddNormal(attibutes.m_normalChannel[j].m_x, attibutes.m_normalChannel[j].m_y, attibutes.m_normalChannel[j].m_z);
						}
		
						if (attibutes.m_binormalChannel.GetCount())
						{
							AddBinormal(attibutes.m_binormalChannel[j].m_x, attibutes.m_binormalChannel[j].m_y, attibutes.m_binormalChannel[j].m_z);
						}
		
						if (attibutes.m_colorChannel.GetCount())
						{
							AddVertexColor(attibutes.m_colorChannel[j].m_x, attibutes.m_colorChannel[j].m_y, attibutes.m_colorChannel[j].m_z, attibutes.m_colorChannel[j].m_w);
						}
		
						if (attibutes.m_uv0Channel.GetCount())
						{
							AddUV0(attibutes.m_uv0Channel[j].m_u, attibutes.m_uv0Channel[j].m_v);
						}
		
						if (attibutes.m_uv1Channel.GetCount())
						{
							AddUV1(attibutes.m_uv1Channel[j].m_u, attibutes.m_uv1Channel[j].m_v);
						}
					}
				}
			}
		}
	}
	else 
	{
		const dBigVector& p0 = m_points.m_vertex[m_constructionIndex + 0];
		const dBigVector& p1 = m_points.m_vertex[m_constructionIndex + 1];
		const dBigVector& p2 = m_points.m_vertex[m_constructionIndex + 2];

		dBigVector e1(p1 - p0);
		dBigVector e2(p2 - p0);
		dBigVector n(e1.CrossProduct(e2));
		dAssert(e1.m_w == dFloat32(0.0f));
		dAssert(e2.m_w == dFloat32(0.0f));
		dAssert(n.m_w == dFloat32(0.0f));

		dFloat64 mag3 = n.DotProduct(n).GetScalar();
		if (mag3 < dFloat64(DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)) 
		{
			m_attrib.SetCount(m_constructionIndex);
			m_points.SetCount(m_constructionIndex);
		}
	}
}


void ndMeshEffect::BeginBuild()
{
	m_points.Clear();
	m_attrib.Clear();
	RemoveAll();
	BeginFace();
	m_vertexBaseCount = -1;
	m_constructionIndex = 0;
}

void ndMeshEffect::EndBuild(dFloat64 tol, bool fixTjoint)
{
	#ifdef _DEBUG
	for (dInt32 i = 0; i < m_points.m_vertex.GetCount(); i += 3)
	{
		dBigVector p0(m_points.m_vertex[i + 0]);
		dBigVector p1(m_points.m_vertex[i + 1]);
		dBigVector p2(m_points.m_vertex[i + 2]);
		dBigVector e1(p1 - p0);
		dBigVector e2(p2 - p0);
		dBigVector n(e1.CrossProduct(e2));

		dAssert(e1.m_w == dFloat32(0.0f));
		dAssert(e2.m_w == dFloat32(0.0f));
		dAssert(n.m_w == dFloat32(0.0f));
		dFloat64 mag2 = n.DotProduct(n).GetScalar();
		dAssert(mag2 > dFloat32(0.0f));
	}
	#endif

	dInt32 triangCount = m_points.m_vertex.GetCount() / 3;
	const dInt32* const indexList = &m_attrib.m_pointChannel[0];
	for (dInt32 i = 0; i < triangCount; i++) 
	{
		dInt32 index[3];
		dInt64 userdata[3];
	
		index[0] = indexList[i * 3 + 0];
		index[1] = indexList[i * 3 + 1];
		index[2] = indexList[i * 3 + 2];
	
		dBigVector e1(m_points.m_vertex[index[1]] - m_points.m_vertex[index[0]]);
		dBigVector e2(m_points.m_vertex[index[2]] - m_points.m_vertex[index[0]]);
		dBigVector n(e1.CrossProduct(e2));
	
		dAssert(e1.m_w == dFloat32(0.0f));
		dAssert(e2.m_w == dFloat32(0.0f));
		dAssert(n.m_w == dFloat32(0.0f));
		dFloat64 mag2 = n.DotProduct(n).GetScalar();
		if (mag2 > dFloat64(1.0e-12f)) 
		{
			userdata[0] = i * 3 + 0;
			userdata[1] = i * 3 + 1;
			userdata[2] = i * 3 + 2;
			dEdge* const edge = AddFace(3, index, userdata);
			if (!edge) 
			{
				dAssert(0);
				/*
				//dAssert ((m_pointCount + 3) <= m_maxPointCount);
				m_points[m_pointCount + 0] = m_points[index[0]];
				m_points[m_pointCount + 1] = m_points[index[1]];
				m_points[m_pointCount + 2] = m_points[index[2]];
	
				index[0] = m_pointCount + 0;
				index[1] = m_pointCount + 1;
				index[2] = m_pointCount + 2;
	
				m_pointCount += 3;
	
				#ifdef _DEBUG
				dEdge* test = AddFace (3, index, userdata);
				dAssert (test);
				#else
				AddFace (3, index, userdata);
				#endif
				*/
			}
		}
	}
	EndFace();
	
	PackAttibuteData();
	PackPoints(tol);
	
	if (fixTjoint) 
	{
		RepairTJoints();
	}
	
	#ifdef _DEBUG
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) 
		{
			dBigVector p0(m_points.m_vertex[face->m_incidentVertex]);
			dBigVector p1(m_points.m_vertex[face->m_next->m_incidentVertex]);
			dBigVector p2(m_points.m_vertex[face->m_next->m_next->m_incidentVertex]);
			dBigVector e1(p1 - p0);
			dBigVector e2(p2 - p0);
			dBigVector n(e1.CrossProduct(e2));
	
			dAssert(e1.m_w == dFloat32(0.0f));
			dAssert(e2.m_w == dFloat32(0.0f));
			dAssert(n.m_w == dFloat32(0.0f));
			dFloat64 mag2 = n.DotProduct(n).GetScalar();
			dAssert(mag2 >= dFloat32(0.0f));
		}
	}
	#endif
}

void ndMeshEffect::BeginFace()
{
	dPolyhedra::BeginFace();
}

bool ndMeshEffect::EndFace()
{
	dPolyhedra::EndFace();
	bool state = false;
	for (bool hasVertexCollision = true; hasVertexCollision;)
	{
		hasVertexCollision = false;
		const dInt32 currentCount = m_points.m_vertex.GetCount();
		dStack<dInt8> verterCollisionBuffer(currentCount);
		dInt8* const verterCollision = &verterCollisionBuffer[0];
		memset(&verterCollision[0], 0, verterCollisionBuffer.GetSizeInBytes());

		Iterator iter(*this);
		dInt32 mark = IncLRU();
		dList<dTreeNode*> collisionFound;
		for (iter.Begin(); iter; iter++)
		{
			dEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark)
			{
				if ((edge->m_incidentVertex < currentCount) && (verterCollision[edge->m_incidentVertex] == 0))
				{
					verterCollision[edge->m_incidentVertex] = 1;
				}
				else
				{
					hasVertexCollision = true;
					collisionFound.Append(iter.GetNode());
				}
				dEdge* ptr = edge;
				do
				{
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}

		dAssert(!collisionFound.GetFirst() || Sanity());
		for (dList<dTreeNode*>::dListNode* node = collisionFound.GetFirst(); node; node = node->GetNext())
		{
			state = true;
			dEdge* const edge = &node->GetInfo()->GetInfo();

			// this is a vertex collision
			m_points.m_vertex.PushBack(m_points.m_vertex[edge->m_incidentVertex]);
			if (m_points.m_layers.GetCount())
			{
				m_points.m_layers.PushBack(m_points.m_layers[edge->m_incidentVertex]);
			}

			dEdge* ptr = edge;
			do
			{
				if (ptr->m_incidentFace > 0)
				{
					//m_attrib.m_pointChannel[dInt32 (ptr->m_userData)] = m_points.m_vertex.m_count - 1;
					dInt32 index = dInt32(ptr->m_userData);
					m_attrib.m_pointChannel.PushBack(m_points.m_vertex.GetCount() - 1);
					if (m_attrib.m_materialChannel.GetCount())
					{
						m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[index]);
					}
					if (m_attrib.m_normalChannel.GetCount())
					{
						m_attrib.m_normalChannel.PushBack(m_attrib.m_normalChannel[index]);
					}
					if (m_attrib.m_binormalChannel.GetCount())
					{
						m_attrib.m_binormalChannel.PushBack(m_attrib.m_binormalChannel[index]);
					}
					if (m_attrib.m_colorChannel.GetCount())
					{
						m_attrib.m_colorChannel.PushBack(m_attrib.m_colorChannel[index]);
					}
					if (m_attrib.m_uv0Channel.GetCount())
					{
						m_attrib.m_uv0Channel.PushBack(m_attrib.m_uv0Channel[index]);
					}
					if (m_attrib.m_uv1Channel.GetCount())
					{
						m_attrib.m_uv1Channel.PushBack(m_attrib.m_uv1Channel[index]);
					}
					ptr->m_userData = m_attrib.m_pointChannel.GetCount() - 1;
				}

				dTreeNode* const edgeNode = GetNodeFromInfo(*ptr);
				dgPairKey edgeKey(ptr->m_incidentVertex, ptr->m_twin->m_incidentVertex);
				ReplaceKey(edgeNode, edgeKey.GetVal());

				dTreeNode* const twinNode = GetNodeFromInfo(*(ptr->m_twin));
				dgPairKey twinKey(ptr->m_twin->m_incidentVertex, ptr->m_incidentVertex);
				ReplaceKey(twinNode, twinKey.GetVal());

				ptr->m_incidentVertex = m_points.m_vertex.GetCount() - 1;

				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
		dAssert(!collisionFound.GetFirst() || Sanity());
	}

	return !state;
}

void ndMeshEffect::BuildFromIndexList(const dMeshVertexFormat* const format)
{
	BeginBuild();
	dAssert(format->m_vertex.m_data);
	dAssert(format->m_vertex.m_indexList);
	dAssert(format->m_vertex.m_strideInBytes);

	// calculate vertex Count
	dInt32 vertexCount = 0;
	dInt32 maxAttribCount = 0;
	for (dInt32 j = 0; j < format->m_faceCount; j++)
	{
		dInt32 count = format->m_faceIndexCount[j];
		for (dInt32 i = 0; i < count; i++)
		{
			vertexCount = dMax(vertexCount, format->m_vertex.m_indexList[maxAttribCount + i] + 1);
		}
		maxAttribCount += count;
	}
	m_vertexBaseCount = vertexCount;

	dInt32 layerIndex = 0;
	dInt32 vertexStride = dInt32(format->m_vertex.m_strideInBytes / sizeof(dFloat64));
	const dFloat64* const vertex = format->m_vertex.m_data;

	m_points.m_layers.Resize(vertexCount);
	m_points.m_vertex.Resize(vertexCount);
	for (dInt32 i = 0; i < vertexCount; i++)
	{
		dInt32 index = i * vertexStride;
		m_points.m_layers.PushBack(layerIndex);
		dBigVector p(vertex[index + 0], vertex[index + 1], vertex[index + 2], dFloat64(0.0f));
		m_points.m_vertex.PushBack(p);
	}

	bool pendingFaces = true;
	dInt32 layerBase = 0;
	dInt32 attributeCount = 0;

	dInt32 normalStride = dInt32(format->m_normal.m_strideInBytes / sizeof(dFloat32));
	dInt32 binormalStride = dInt32(format->m_binormal.m_strideInBytes / sizeof(dFloat32));
	dInt32 uv0Stride = dInt32(format->m_uv0.m_strideInBytes / sizeof(dFloat32));
	dInt32 uv1Stride = dInt32(format->m_uv1.m_strideInBytes / sizeof(dFloat32));
	dInt32 vertexColorStride = dInt32(format->m_vertexColor.m_strideInBytes / sizeof(dFloat32));

	dStack<dInt8> faceMark(format->m_faceCount);
	memset(&faceMark[0], 0, faceMark.GetSizeInBytes());
	const dInt32* const vertexIndex = format->m_vertex.m_indexList;

	while (pendingFaces)
	{
		dInt32 acc = 0;
		pendingFaces = false;
		dInt32 vertexBank = layerIndex * vertexCount;
		for (dInt32 j = 0; j < format->m_faceCount; j++)
		{
			dInt32 indexCount = format->m_faceIndexCount[j];

			if (indexCount > 0)
			{
				dInt32 index[256];
				dInt64 userdata[256];
				dAssert(indexCount >= 3);
				dAssert(indexCount < dInt32(sizeof(index) / sizeof(index[0])));

				if (!faceMark[j])
				{
					for (dInt32 i = 0; i < indexCount; i++)
					{
						dInt32 k = attributeCount + i;
						userdata[i] = k;
						index[i] = vertexIndex[acc + i] + vertexBank;
					}

					dEdge* const edge = AddFace(indexCount, index, userdata);
					if (edge)
					{
						faceMark[j] = 1;
						for (dInt32 i = 0; i < indexCount; i++)
						{
							m_attrib.m_pointChannel.PushBack(index[i]);
						}

						if (format->m_faceMaterial)
						{
							dInt32 materialIndex = format->m_faceMaterial[j];
							for (dInt32 i = 0; i < indexCount; i++)
							{
								m_attrib.m_materialChannel.PushBack(materialIndex);
							}
						}

						if (format->m_normal.m_data)
						{
							dTriplex normal;
							for (dInt32 i = 0; i < indexCount; i++)
							{
								dInt32 k = attributeCount + i;
								dInt32 m = format->m_normal.m_indexList[k] * normalStride;
								normal.m_x = format->m_normal.m_data[m + 0];
								normal.m_y = format->m_normal.m_data[m + 1];
								normal.m_z = format->m_normal.m_data[m + 2];
								m_attrib.m_normalChannel.PushBack(normal);
							}
						}

						if (format->m_binormal.m_data)
						{
							dTriplex normal;
							for (dInt32 i = 0; i < indexCount; i++)
							{
								dInt32 k = attributeCount + i;
								dInt32 m = format->m_binormal.m_indexList[k] * binormalStride;
								normal.m_x = format->m_binormal.m_data[m + 0];
								normal.m_y = format->m_binormal.m_data[m + 1];
								normal.m_z = format->m_binormal.m_data[m + 2];
								m_attrib.m_binormalChannel.PushBack(normal);
							}
						}

						if (format->m_vertexColor.m_data)
						{
							for (dInt32 i = 0; i < indexCount; i++)
							{
								dInt32 k = attributeCount + i;
								dInt32 m = format->m_vertexColor.m_indexList[k] * vertexColorStride;
								dVector color(format->m_vertexColor.m_data[m + 0], format->m_vertexColor.m_data[m + 1], format->m_vertexColor.m_data[m + 2], format->m_vertexColor.m_data[m + 3]);
								m_attrib.m_colorChannel.PushBack(color);
							}
						}

						if (format->m_uv0.m_data)
						{
							dAttibutFormat::dgUV uv;
							for (dInt32 i = 0; i < indexCount; i++)
							{
								dInt32 k = attributeCount + i;
								dInt32 m = format->m_uv0.m_indexList[k] * uv0Stride;
								uv.m_u = format->m_uv0.m_data[m + 0];
								uv.m_v = format->m_uv0.m_data[m + 1];
								m_attrib.m_uv0Channel.PushBack(uv);
							}
						}

						if (format->m_uv1.m_data)
						{
							dAttibutFormat::dgUV uv;
							for (dInt32 i = 0; i < indexCount; i++)
							{
								dInt32 k = attributeCount + i;
								dInt32 m = format->m_uv1.m_indexList[k] * uv1Stride;
								uv.m_u = format->m_uv1.m_data[m + 0];
								uv.m_v = format->m_uv1.m_data[m + 1];
								m_attrib.m_uv1Channel.PushBack(uv);
							}
						}
						attributeCount += indexCount;
					}
					else
					{
						// check if the face is not degenerated
						bool degeneratedFace = false;
						for (dInt32 i = 0; i < indexCount - 1; i++)
						{
							for (dInt32 k = i + 1; k < indexCount; k++)
							{
								if (index[i] == index[k])
								{
									degeneratedFace = true;
								}
							}
						}
						if (degeneratedFace)
						{
							faceMark[j] = 1;
						}
						else
						{
							pendingFaces = true;
						}
					}
				}
				acc += indexCount;
			}
		}

		if (pendingFaces)
		{
			//dAssert (0);
			layerIndex++;
			layerBase += vertexCount;
			for (dInt32 i = 0; i < vertexCount; i++)
			{
				m_points.m_layers.PushBack(layerIndex);
				dInt32 index = i * vertexStride;
				dBigVector p(vertex[index + 0], vertex[index + 1], vertex[index + 2], vertex[index + 3]);
				m_points.m_vertex.PushBack(p);
			}
		}
	}

	dAssert(m_points.m_vertex.GetCount() == vertexCount * (layerIndex + 1));
	dAssert(m_attrib.m_pointChannel.GetCount() == attributeCount);

	EndFace();
	PackAttibuteData();
}

void ndMeshEffect::PackPoints(dFloat64 tol)
{
	dStack<dInt32>vertexIndexMapBuffer(m_points.m_vertex.GetCount());
	dInt32* const vertexIndexMap = &vertexIndexMapBuffer[0];
	m_points.CompressData(&vertexIndexMap[0]);

	dInt32 index[DG_MESH_EFFECT_POINT_SPLITED];
	dInt64 userData[DG_MESH_EFFECT_POINT_SPLITED];
	dPolyhedra polygon;
	SwapInfo(polygon);
	dAssert(GetCount() == 0);

	BeginFace();
	const dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(polygon);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) 
		{
			dEdge* ptr = edge;
			dInt32 indexCount = 0;
			do 
			{
				ptr->m_mark = mark;
				index[indexCount] = vertexIndexMap[ptr->m_incidentVertex];
				m_attrib.m_pointChannel[dInt32(ptr->m_userData)] = vertexIndexMap[ptr->m_incidentVertex];
				userData[indexCount] = ptr->m_userData;

				indexCount++;
				ptr = ptr->m_next;
			} while (ptr != edge);
			dEdge* const face = AddFace(indexCount, index, userData);
			if (!face) 
			{
				dTrace(("skiping degeneraded face\n"));
				//dAssert (0);
			}
		}
	}
	EndFace();
}

void ndMeshEffect::UnpackPoints()
{
	do 
	{
		dPointFormat points(m_points);
		m_points.Clear();
		for (dInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); i++) 
		{
			dInt32 index = m_attrib.m_pointChannel[i];
			m_points.m_vertex.PushBack(points.m_vertex[index]);
			if (points.m_layers.GetCount()) 
			{
				m_points.m_layers.PushBack(points.m_layers[index]);
			}

			m_attrib.m_pointChannel[i] = i;
		}

		dInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
		dInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];
		dPolyhedra polygon;
		SwapInfo(polygon);
		dAssert(GetCount() == 0);
		BeginFace();
		const dInt32 mark = IncLRU();
		dPolyhedra::Iterator iter(polygon);
		for (iter.Begin(); iter; iter++) 
		{
			dEdge* const face = &(*iter);
			if ((face->m_mark != mark) && (face->m_incidentFace > 0)) 
			{
				dEdge* ptr = face;
				dInt32 indexCount = 0;
				do 
				{
					ptr->m_mark = mark;
					index[indexCount] = dInt32(ptr->m_userData);
					userData[indexCount] = ptr->m_userData;
					indexCount++;
					ptr = ptr->m_next;
				} while (ptr != face);
				AddFace(indexCount, index, userData);
			}
		}
	} while (!EndFace());

	dAssert(m_points.m_vertex.GetCount() == m_attrib.m_pointChannel.GetCount());
	#ifdef _DEBUG
	for (dInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); i++)
	{
		dAssert(m_attrib.m_pointChannel[i] == i);
	}
	#endif
}

void ndMeshEffect::PackAttibuteData()
{
	dStack<dInt32>attrIndexBuffer(m_attrib.m_pointChannel.GetCount());
	dInt32* const attrIndexMap = &attrIndexBuffer[0];
	m_attrib.CompressData(m_points, &attrIndexMap[0]);

	Iterator iter(*this);
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &(*iter);
		if (edge->m_incidentFace > 0)
		{
			edge->m_userData = attrIndexMap[edge->m_userData];
		}
	}

	memset(attrIndexMap, -1, sizeof(dInt32) * m_attrib.m_pointChannel.GetCount());
	dAttibutFormat tmpFormat(m_attrib);
	m_attrib.Clear();

	dInt32 remapIndex = 0;
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &(*iter);
		if (edge->m_incidentFace > 0)
		{
			dInt32 index = dInt32(edge->m_userData);
			if (attrIndexMap[edge->m_userData] == -1)
			{
				attrIndexMap[index] = remapIndex;
				remapIndex++;

				m_attrib.m_pointChannel.PushBack(tmpFormat.m_pointChannel[index]);
				if (tmpFormat.m_materialChannel.m_isValid)
				{
					m_attrib.m_materialChannel.PushBack(tmpFormat.m_materialChannel[index]);
				}
				if (tmpFormat.m_normalChannel.m_isValid)
				{
					m_attrib.m_normalChannel.PushBack(tmpFormat.m_normalChannel[index]);
				}
				if (tmpFormat.m_binormalChannel.m_isValid)
				{
					m_attrib.m_binormalChannel.PushBack(tmpFormat.m_binormalChannel[index]);
				}
				if (tmpFormat.m_uv0Channel.m_isValid)
				{
					m_attrib.m_uv0Channel.PushBack(tmpFormat.m_uv0Channel[index]);
				}
				if (tmpFormat.m_uv1Channel.m_isValid)
				{
					m_attrib.m_uv1Channel.PushBack(tmpFormat.m_uv1Channel[index]);
				}
				if (tmpFormat.m_colorChannel.m_isValid)
				{
					m_attrib.m_colorChannel.PushBack(tmpFormat.m_colorChannel[index]);
				}
			}
			edge->m_userData = attrIndexMap[index];
		}
	}
}

void ndMeshEffect::UnpackAttibuteData()
{
	dAttibutFormat attibutes(m_attrib);
	m_attrib.Clear();

	Iterator iter(*this);
	dInt32 attributeCount = 0;
	const dInt32 lru = IncLRU();
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &iter.GetNode()->GetInfo();
		if ((edge->m_incidentFace > 0) && (edge->m_mark != lru))
		{
			dEdge* ptr = edge;

			ptr = edge;
			do
			{
				ptr->m_mark = lru;
				m_attrib.m_pointChannel.PushBack(ptr->m_incidentVertex);

				if (attibutes.m_materialChannel.m_isValid)
				{
					m_attrib.m_materialChannel.PushBack(attibutes.m_materialChannel[dInt32(ptr->m_userData)]);
				}

				if (attibutes.m_normalChannel.m_isValid)
				{
					m_attrib.m_normalChannel.PushBack(attibutes.m_normalChannel[dInt32(ptr->m_userData)]);
				}

				if (attibutes.m_binormalChannel.m_isValid)
				{
					m_attrib.m_binormalChannel.PushBack(attibutes.m_binormalChannel[dInt32(ptr->m_userData)]);
				}

				if (attibutes.m_colorChannel.m_isValid)
				{
					m_attrib.m_colorChannel.PushBack(attibutes.m_colorChannel[dInt32(ptr->m_userData)]);
				}

				if (attibutes.m_uv0Channel.m_isValid)
				{
					m_attrib.m_uv0Channel.PushBack(attibutes.m_uv0Channel[dInt32(ptr->m_userData)]);
				}

				if (attibutes.m_uv1Channel.m_isValid)
				{
					m_attrib.m_uv1Channel.PushBack(attibutes.m_uv1Channel[dInt32(ptr->m_userData)]);
				}

				ptr->m_userData = attributeCount;
				attributeCount++;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
	dAssert(m_attrib.m_pointChannel.GetCount() == attributeCount);
}


bool ndMeshEffect::SeparateDuplicateLoops(dEdge* const face)
{
	for (dEdge* ptr0 = face; ptr0 != face->m_prev; ptr0 = ptr0->m_next)
	{
		dInt32 index = ptr0->m_incidentVertex;

		dEdge* ptr1 = ptr0->m_next;
		do
		{
			if (ptr1->m_incidentVertex == index)
			{
				dEdge* const ptr00 = ptr0->m_prev;
				dEdge* const ptr11 = ptr1->m_prev;

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

dInt32 ndMeshEffect::AddInterpolatedHalfAttribute(dEdge* const edge, dInt32 midPoint)
{
	dBigVector p0(m_points.m_vertex[edge->m_incidentVertex]);
	dBigVector p2(m_points.m_vertex[edge->m_next->m_incidentVertex]);
	dBigVector p1(m_points.m_vertex[midPoint]);
	dBigVector p2p0(p2 - p0);

	dAssert(p2p0.m_w == dFloat32(0.0f));

	dFloat64 den = p2p0.DotProduct(p2p0).GetScalar();
	dFloat64 param = p2p0.DotProduct(p1 - p0).GetScalar() / den;
	dFloat64 t1 = param;
	dFloat64 t0 = dFloat64(1.0f) - t1;
	dAssert(t1 >= dFloat64(0.0f));
	dAssert(t1 <= dFloat64(1.0f));

	m_attrib.m_pointChannel.PushBack(midPoint);

	if (m_attrib.m_materialChannel.m_isValid)
	{
		m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[dInt32(edge->m_userData)]);
	}
	if (m_attrib.m_normalChannel.m_isValid)
	{
		dTriplex edgeNormal;
		dTriplex edgeNormal0(m_attrib.m_normalChannel[dInt32(edge->m_userData)]);
		dTriplex edgeNormal1(m_attrib.m_normalChannel[dInt32(edge->m_next->m_userData)]);
		edgeNormal.m_x = edgeNormal0.m_x * dFloat32(t0) + edgeNormal1.m_x * dFloat32(t1);
		edgeNormal.m_y = edgeNormal0.m_y * dFloat32(t0) + edgeNormal1.m_y * dFloat32(t1);
		edgeNormal.m_z = edgeNormal0.m_z * dFloat32(t0) + edgeNormal1.m_z * dFloat32(t1);
		m_attrib.m_normalChannel.PushBack(edgeNormal);
	}
	if (m_attrib.m_binormalChannel.m_isValid)
	{
		dAssert(0);
	}

	if (m_attrib.m_uv0Channel.m_isValid)
	{
		dAttibutFormat::dgUV edgeUV;
		dAttibutFormat::dgUV edgeUV0(m_attrib.m_uv0Channel[dInt32(edge->m_userData)]);
		dAttibutFormat::dgUV edgeUV1(m_attrib.m_uv0Channel[dInt32(edge->m_next->m_userData)]);
		edgeUV.m_u = edgeUV0.m_u * dFloat32(t0) + edgeUV1.m_u * dFloat32(t1);
		edgeUV.m_v = edgeUV0.m_v * dFloat32(t0) + edgeUV1.m_v * dFloat32(t1);
		m_attrib.m_uv0Channel.PushBack(edgeUV);
	}

	if (m_attrib.m_uv1Channel.m_isValid)
	{
		dAssert(0);
	}

	if (m_attrib.m_colorChannel.m_isValid)
	{
		dAssert(0);
	}
	return m_attrib.m_pointChannel.GetCount() - 1;
}

void ndMeshEffect::RepairTJoints()
{
	dAssert(Sanity());

	// delete edge of zero length
	bool dirty = true;
	while (dirty)
	{
		dFloat64 tol = dFloat64(1.0e-5f);
		dFloat64 tol2 = tol * tol;
		dirty = false;
		dPolyhedra::Iterator iter(*this);
		for (iter.Begin(); iter; )
		{
			dEdge* const edge = &(*iter);
			iter++;
			const dBigVector& p0 = m_points.m_vertex[edge->m_incidentVertex];
			const dBigVector& p1 = m_points.m_vertex[edge->m_twin->m_incidentVertex];
			dBigVector dist(p1 - p0);
			dAssert(dist.m_w == dFloat32(0.0f));
			dFloat64 mag2 = dist.DotProduct(dist).GetScalar();
			if (mag2 < tol2)
			{
				bool move = true;
				while (move)
				{
					move = false;
					dEdge* ptr = edge->m_twin;
					do
					{
						if ((&(*iter) == ptr) || (&(*iter) == ptr->m_twin))
						{
							move = true;
							iter++;
						}
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge->m_twin);

					ptr = edge;
					do
					{
						if ((&(*iter) == ptr) || (&(*iter) == ptr->m_twin))
						{
							move = true;
							iter++;
						}
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);
				}

				dEdge* const collapsedEdge = CollapseEdge(edge);
				if (collapsedEdge)
				{
					//dAssert (0);
					dTrace(("remember to finish this!!: %s %s line:%d\n", __FILE__, __FUNCTION__, __LINE__));
					dirty = true;
					dBigVector q(m_points.m_vertex[collapsedEdge->m_incidentVertex]);
					dEdge* ptr = collapsedEdge;
					do
					{
						if (ptr->m_incidentFace > 0)
						{
							//dAssert (0);
							//m_attrib[ptr->m_userData].m_vertex = q;
							dTrace(("remember to finish this!!: %s %s line:%d\n", __FILE__, __FUNCTION__, __LINE__));
						}
						ptr = ptr->m_twin->m_next;
					} while (ptr != collapsedEdge);
				}
			}
		}
	}
	dAssert(Sanity());

	// repair straight open edges
	dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &(*iter);
		if ((edge->m_mark) != mark && (edge->m_incidentFace < 0))
		{
			while (SeparateDuplicateLoops(edge));
			dEdge* ptr = edge;
			do
			{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	dAssert(Sanity());
	DeleteDegenerateFaces(&m_points.m_vertex[0].m_x, sizeof(dBigVector), dFloat64(1.0e-7f));
	dAssert(Sanity());

	// delete straight line edges
	dirty = true;
	//	dirty = false;
	while (dirty)
	{
		dFloat64 tol = dFloat64(1.0 - 1.0e-8);
		dFloat64 tol2 = tol * tol;

		dirty = false;
		dAssert(Sanity());

		dPolyhedra::Iterator iter1(*this);
		for (iter1.Begin(); iter1; )
		{
			dEdge* const edge = &(*iter1);
			iter1++;

			const dBigVector& p0 = m_points.m_vertex[edge->m_incidentVertex];
			const dBigVector& p1 = m_points.m_vertex[edge->m_next->m_incidentVertex];
			const dBigVector& p2 = m_points.m_vertex[edge->m_next->m_next->m_incidentVertex];

			dBigVector A(p1 - p0);
			dBigVector B(p2 - p1);
			dAssert(A.m_w == dFloat32(0.0f));
			dAssert(B.m_w == dFloat32(0.0f));
			dFloat64 ab = A.DotProduct(B).GetScalar();
			if (ab >= 0.0f)
			{
				dFloat64 aa = A.DotProduct(A).GetScalar();
				dFloat64 bb = B.DotProduct(B).GetScalar();

				dFloat64 magab2 = ab * ab;
				dFloat64 magaabb = aa * bb * tol2;
				if (magab2 >= magaabb)
				{
					if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0))
					{
						if (edge->m_twin->m_prev == edge->m_next->m_twin)
						{
							dEdge* const newEdge = AddHalfEdge(edge->m_incidentVertex, edge->m_next->m_next->m_incidentVertex);
							if (newEdge)
							{
								dirty = true;
								dEdge* const newTwin = AddHalfEdge(edge->m_next->m_next->m_incidentVertex, edge->m_incidentVertex);
								dAssert(newEdge);
								dAssert(newTwin);

								newEdge->m_twin = newTwin;
								newTwin->m_twin = newEdge;

								newEdge->m_userData = edge->m_userData;
								newTwin->m_userData = edge->m_twin->m_prev->m_userData;

								newEdge->m_incidentFace = edge->m_incidentFace;
								newTwin->m_incidentFace = edge->m_twin->m_incidentFace;

								dEdge* const nextEdge = edge->m_next;

								nextEdge->m_twin->m_prev->m_next = newTwin;
								newTwin->m_prev = nextEdge->m_twin->m_prev;

								edge->m_twin->m_next->m_prev = newTwin;
								newTwin->m_next = edge->m_twin->m_next;

								nextEdge->m_next->m_prev = newEdge;
								newEdge->m_next = nextEdge->m_next;

								edge->m_prev->m_next = newEdge;
								newEdge->m_prev = edge->m_prev;

								while ((&(*iter1) == edge->m_twin) || (&(*iter1) == nextEdge) || (&(*iter1) == nextEdge->m_twin))
								{
									iter1++;
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
								//dAssert (Sanity ());

							}
							else if (edge->m_next->m_next->m_next == edge)
							{
								dAssert(0);
								/*
								dirty = true;
								dEdge* const openEdge = edge;
								dEdge* const nextEdge = openEdge->m_next;
								dEdge* const deletedEdge = openEdge->m_prev;
								while ((&(*iter) == deletedEdge) || (&(*iter) == deletedEdge->m_twin))
								{
									iter ++;
								}

								openEdge->m_userData = deletedEdge->m_twin->m_userData;

								dBigVector p2p0 (p2 - p0);
								dFloat64 den = p2p0.DotProduct3(p2p0);
								dFloat64 param1 = p2p0.DotProduct3(p1 - p0) / den;
								dgVertexAtribute attib1 = AddInterpolateEdgeAttibute (deletedEdge->m_twin, param1);
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
								//dAssert (Sanity ());
								*/
							}
						}
					}
					else if (FindEdge(edge->m_incidentVertex, edge->m_next->m_next->m_incidentVertex))
					{
						dEdge* const openEdge = edge;
						//dAssert (openEdge->m_incidentFace <= 0);
						dTrace(("remember to finish this!!: %s %s line:%d\n", __FILE__, __FUNCTION__, __LINE__));
						dEdge* const nextEdge = openEdge->m_next;
						dEdge* const deletedEdge = openEdge->m_prev;
						if (deletedEdge == openEdge->m_next->m_next)
						{
							dirty = true;
							while ((&(*iter1) == deletedEdge) || (&(*iter1) == deletedEdge->m_twin))
							{
								iter1++;
							}

							//dAssert (deletedEdge->m_twin->m_incidentFace > 0);
							dTrace(("remember to finish this!!: %s %s line:%d\n", __FILE__, __FUNCTION__, __LINE__));
							openEdge->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
							openEdge->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;

							dInt32 attibuteIndex = AddInterpolatedHalfAttribute(deletedEdge->m_twin, nextEdge->m_incidentVertex);
							openEdge->m_next->m_userData = attibuteIndex;
							openEdge->m_userData = deletedEdge->m_twin->m_userData;

							deletedEdge->m_twin->m_prev->m_next = openEdge;
							openEdge->m_prev = deletedEdge->m_twin->m_prev;

							deletedEdge->m_twin->m_next->m_prev = nextEdge;
							nextEdge->m_next = deletedEdge->m_twin->m_next;

							deletedEdge->m_twin->m_next = deletedEdge;
							deletedEdge->m_twin->m_prev = deletedEdge;
							deletedEdge->m_next = deletedEdge->m_twin;
							deletedEdge->m_prev = deletedEdge->m_twin;
							DeleteEdge(deletedEdge);
							dAssert(Sanity());
						}
					}
					else
					{
						dEdge* const openEdge = (edge->m_incidentFace <= 0) ? edge : edge->m_twin;
						dAssert(openEdge->m_incidentFace <= 0);

						const dBigVector& p3 = m_points.m_vertex[openEdge->m_next->m_next->m_next->m_incidentVertex];

						dBigVector A0(p3 - p2);
						dBigVector B0(p2 - p1);
						dAssert(A0.m_w == dFloat32(0.0f));
						dAssert(B0.m_w == dFloat32(0.0f));

						dFloat64 ab0 = A0.DotProduct(B0).GetScalar();
						if (ab0 >= dFloat32(0.0f))
						{
							dFloat64 aa0 = A0.DotProduct(A0).GetScalar();
							dFloat64 bb0 = B0.DotProduct(B0).GetScalar();

							dFloat64 ab0ab0 = ab0 * ab0;
							dFloat64 aa0bb0 = aa0 * bb0 * tol2;
							if (ab0ab0 >= aa0bb0)
							{
								if (openEdge->m_next->m_next->m_next->m_next != openEdge)
								{
									const dBigVector& p4 = m_points.m_vertex[openEdge->m_prev->m_incidentVertex];
									dBigVector A1(p1 - p0);
									dBigVector B1(p1 - p4);
									dAssert(A1.m_w == dFloat32(0.0f));
									dAssert(B1.m_w == dFloat32(0.0f));
									dFloat64 ab1 = A1.DotProduct(B1).GetScalar();
									if (ab1 < dFloat32(0.0f))
									{
										dFloat64 ab1ab1 = ab1 * ab1;
										dFloat64 aa1bb1 = aa0 * bb0 * tol2;
										if (ab1ab1 >= aa1bb1)
										{
											dEdge* const newFace = ConnectVertex(openEdge->m_prev, openEdge->m_next);
											dirty |= newFace ? true : false;
										}
									}
									//dAssert (Sanity ());
								}
								else if (openEdge->m_prev->m_twin->m_incidentFace > 0)
								{
									dirty = true;

									dEdge* const deletedEdge = openEdge->m_prev;
									while ((&(*iter1) == deletedEdge) || (&(*iter1) == deletedEdge->m_twin))
									{
										iter1++;
									}

									openEdge->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
									openEdge->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
									openEdge->m_next->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;

									dInt32 attibuteIndex0 = AddInterpolatedHalfAttribute(deletedEdge->m_twin, openEdge->m_next->m_incidentVertex);
									dInt32 attibuteIndex1 = AddInterpolatedHalfAttribute(deletedEdge->m_twin, openEdge->m_next->m_next->m_incidentVertex);

									openEdge->m_userData = deletedEdge->m_twin->m_userData;
									openEdge->m_next->m_userData = attibuteIndex0;
									openEdge->m_next->m_next->m_userData = attibuteIndex1;

									deletedEdge->m_twin->m_prev->m_next = openEdge;
									openEdge->m_prev = deletedEdge->m_twin->m_prev;

									deletedEdge->m_twin->m_next->m_prev = deletedEdge->m_prev;
									deletedEdge->m_prev->m_next = deletedEdge->m_twin->m_next;

									deletedEdge->m_twin->m_next = deletedEdge;
									deletedEdge->m_twin->m_prev = deletedEdge;
									deletedEdge->m_next = deletedEdge->m_twin;
									deletedEdge->m_prev = deletedEdge->m_twin;
									DeleteEdge(deletedEdge);
									//dAssert (Sanity ());
								}
							}
						}
					}
				}
			}
		}
	}
	dAssert(Sanity());

	DeleteDegenerateFaces(&m_points.m_vertex[0].m_x, sizeof(dBigVector), dFloat64(1.0e-7f));
	/*
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_incidentFace > 0)
		{
			dBigVector p0(m_points[edge->m_incidentVertex]);
			m_attrib[edge->m_userData].m_vertex.m_x = p0.m_x;
			m_attrib[edge->m_userData].m_vertex.m_y = p0.m_y;
			m_attrib[edge->m_userData].m_vertex.m_z = p0.m_z;
		}
	}
	*/

	dAssert(Sanity());
}


void ndMeshEffect::GetVertexChannel64(dInt32 strideInByte, dFloat64* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat64);
	for (dInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		const dInt32 index = m_attrib.m_pointChannel[i];
		bufferOut[j + 0] = m_points.m_vertex[index].m_x;
		bufferOut[j + 1] = m_points.m_vertex[index].m_y;
		bufferOut[j + 2] = m_points.m_vertex[index].m_z;
	}
}

void ndMeshEffect::GetVertexChannel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat32);
	for (dInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		const dInt32 index = m_attrib.m_pointChannel[i];
		const dBigVector& p = m_points.m_vertex[index];
		bufferOut[j + 0] = dFloat32(p.m_x);
		bufferOut[j + 1] = dFloat32(p.m_y);
		bufferOut[j + 2] = dFloat32(p.m_z);
	}
}

void ndMeshEffect::GetNormalChannel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat32);
	for (dInt32 i = 0; i < m_attrib.m_normalChannel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		bufferOut[j + 0] = dFloat32(m_attrib.m_normalChannel[i].m_x);
		bufferOut[j + 1] = dFloat32(m_attrib.m_normalChannel[i].m_y);
		bufferOut[j + 2] = dFloat32(m_attrib.m_normalChannel[i].m_z);
	}
}

void ndMeshEffect::GetBinormalChannel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat32);
	for (dInt32 i = 0; i < m_attrib.m_binormalChannel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		bufferOut[j + 0] = dFloat32(m_attrib.m_binormalChannel[i].m_x);
		bufferOut[j + 1] = dFloat32(m_attrib.m_binormalChannel[i].m_y);
		bufferOut[j + 2] = dFloat32(m_attrib.m_binormalChannel[i].m_z);
	}
}

void ndMeshEffect::GetUV0Channel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat32);
	for (dInt32 i = 0; i < m_attrib.m_uv0Channel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		bufferOut[j + 0] = dFloat32(m_attrib.m_uv0Channel[i].m_u);
		bufferOut[j + 1] = dFloat32(m_attrib.m_uv0Channel[i].m_v);
	}
}

void ndMeshEffect::GetUV1Channel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat32);
	for (dInt32 i = 0; i < m_attrib.m_uv1Channel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		bufferOut[j + 0] = dFloat32(m_attrib.m_uv1Channel[i].m_u);
		bufferOut[j + 1] = dFloat32(m_attrib.m_uv1Channel[i].m_v);
	}
}

void ndMeshEffect::GetVertexColorChannel(dInt32 strideInByte, dFloat32* const bufferOut) const
{
	dInt32 stride = strideInByte / sizeof(dFloat32);
	for (dInt32 i = 0; i < m_attrib.m_colorChannel.GetCount(); i++)
	{
		const dInt32 j = i * stride;
		bufferOut[j + 0] = dFloat32(m_attrib.m_colorChannel[i].m_x);
		bufferOut[j + 1] = dFloat32(m_attrib.m_colorChannel[i].m_y);
		bufferOut[j + 2] = dFloat32(m_attrib.m_colorChannel[i].m_z);
		bufferOut[j + 3] = dFloat32(m_attrib.m_colorChannel[i].m_w);
	}
}

ndIndexArray* ndMeshEffect::MaterialGeometryBegin()
{
	dInt32 materials[256];
	dInt32 streamIndexMap[256];

	dInt32 count = 0;
	dInt32 materialCount = 0;

	ndIndexArray* const array = (ndIndexArray*)dMemory::Malloc(dInt32(4 * sizeof(dInt32) * GetCount() + sizeof(ndIndexArray) + 2048));
	array->m_indexList = (dInt32*)&array[1];

	dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(*this);
	memset(streamIndexMap, 0, sizeof(streamIndexMap));
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
		{
			dEdge* ptr = edge;
			ptr->m_mark = mark;
			dInt32 index0 = dInt32(ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			dInt32 index1 = dInt32(ptr->m_userData);

			ptr = ptr->m_next;
			do
			{
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = dInt32(ptr->m_userData);
				array->m_indexList[count * 4 + 3] = m_attrib.m_materialChannel.m_isValid ? dInt32(m_attrib.m_materialChannel[dInt32(edge->m_userData)]) : 0;
				index1 = dInt32(ptr->m_userData);

				dInt32 hashValue = array->m_indexList[count * 4 + 3] & 0xff;
				streamIndexMap[hashValue] ++;
				materials[hashValue] = array->m_indexList[count * 4 + 3];
				count++;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	array->m_indexCount = count;
	array->m_materialCount = materialCount;

	count = 0;
	for (dInt32 i = 0; i < 256; i++)
	{
		if (streamIndexMap[i])
		{
			array->m_materials[count] = materials[i];
			array->m_materialsIndexCount[count] = streamIndexMap[i] * 3;
			count++;
		}
	}

	array->m_materialCount = count;
	return array;
}

void ndMeshEffect::MaterialGeomteryEnd(ndIndexArray* const handle)
{
	dMemory::Free(handle);
}

dInt32 ndMeshEffect::GetFirstMaterial(ndIndexArray* const handle) const
{
	return GetNextMaterial(handle, -1);
}

dInt32 ndMeshEffect::GetNextMaterial(ndIndexArray* const handle, dInt32 materialId) const
{
	materialId++;
	if (materialId >= handle->m_materialCount)
	{
		materialId = -1;
	}
	return materialId;
}

dInt32 ndMeshEffect::GetMaterialID(ndIndexArray* const handle, dInt32 materialHandle) const
{
	return handle->m_materials[materialHandle];
}

dInt32 ndMeshEffect::GetMaterialIndexCount(ndIndexArray* const handle, dInt32 materialHandle) const
{
	return handle->m_materialsIndexCount[materialHandle];
}

void ndMeshEffect::GetMaterialGetIndexStream(ndIndexArray* const handle, dInt32 materialHandle, dInt32* const indexArray) const
{
	dInt32 index = 0;
	dInt32 textureID = handle->m_materials[materialHandle];
	for (dInt32 j = 0; j < handle->m_indexCount; j++)
	{
		if (handle->m_indexList[j * 4 + 3] == textureID)
		{
			indexArray[index + 0] = handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = handle->m_indexList[j * 4 + 2];
			index += 3;
		}
	}
}

void ndMeshEffect::GetMaterialGetIndexStreamShort(ndIndexArray* const handle, dInt32 materialHandle, dInt16* const indexArray) const
{
	dInt32 index = 0;
	dInt32 textureID = handle->m_materials[materialHandle];
	for (dInt32 j = 0; j < handle->m_indexCount; j++)
	{
		if (handle->m_indexList[j * 4 + 3] == textureID)
		{
			indexArray[index + 0] = (dInt16)handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = (dInt16)handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = (dInt16)handle->m_indexList[j * 4 + 2];
			index += 3;
		}
	}
}

void ndMeshEffect::ApplyTransform(const dMatrix& matrix)
{
	matrix.TransformTriplex(&m_points.m_vertex[0].m_x, sizeof(dBigVector), &m_points.m_vertex[0].m_x, sizeof(dBigVector), m_points.m_vertex.GetCount());

	dMatrix invMatix(matrix.Inverse4x4());
	invMatix.m_posit = dVector::m_wOne;
	dMatrix rotation(invMatix.Transpose4X4());
	for (dInt32 i = 0; i < m_attrib.m_normalChannel.GetCount(); i++)
	{
		dVector n(dFloat32(m_attrib.m_normalChannel[i].m_x), dFloat32(m_attrib.m_normalChannel[i].m_y), dFloat32(m_attrib.m_normalChannel[i].m_z), dFloat32(0.0f));
		n = rotation.RotateVector(n);
		dAssert(n.m_w == dFloat32(0.0f));
		dAssert(n.DotProduct(n).GetScalar() > dFloat32(0.0f));
		n = n.Normalize();
		m_attrib.m_normalChannel[i].m_x = n.m_x;
		m_attrib.m_normalChannel[i].m_y = n.m_y;
		m_attrib.m_normalChannel[i].m_z = n.m_z;
	}

	for (dInt32 i = 0; i < m_attrib.m_binormalChannel.GetCount(); i++)
	{
		dVector n(dFloat32(m_attrib.m_binormalChannel[i].m_x), dFloat32(m_attrib.m_binormalChannel[i].m_y), dFloat32(m_attrib.m_binormalChannel[i].m_z), dFloat32(0.0f));
		n = rotation.RotateVector(n);
		dAssert(n.m_w == dFloat32(0.0f));
		dAssert(n.DotProduct(n).GetScalar() > dFloat32(0.0f));
		n = n.Normalize();
		m_attrib.m_binormalChannel[i].m_x = n.m_x;
		m_attrib.m_binormalChannel[i].m_y = n.m_y;
		m_attrib.m_binormalChannel[i].m_z = n.m_z;
	}
}

void ndMeshEffect::MergeFaces(const ndMeshEffect* const source)
{
	dInt32 mark = source->IncLRU();
	dPolyhedra::Iterator iter(*source);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);
		if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) 
		{
			BeginBuildFace();
			dEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				dInt32 vIndex = ptr->m_incidentVertex;
				dInt32 aIndex = dInt32(ptr->m_userData);
				AddPoint(source->m_points.m_vertex[vIndex].m_x, source->m_points.m_vertex[vIndex].m_y, source->m_points.m_vertex[vIndex].m_z);
				if (source->m_points.m_layers.GetCount()) 
				{
					AddLayer(source->m_points.m_layers[vIndex]);
				}
	
				if (source->m_attrib.m_materialChannel.GetCount()) 
				{
					AddMaterial(source->m_attrib.m_materialChannel[aIndex]);
				}

				if (source->m_attrib.m_colorChannel.GetCount())
				{
					AddVertexColor(source->m_attrib.m_colorChannel[aIndex].m_x, source->m_attrib.m_colorChannel[aIndex].m_y, source->m_attrib.m_colorChannel[aIndex].m_z, source->m_attrib.m_colorChannel[aIndex].m_w);
				}

				if (source->m_attrib.m_normalChannel.GetCount())
				{
					AddNormal(source->m_attrib.m_normalChannel[aIndex].m_x, source->m_attrib.m_normalChannel[aIndex].m_y, source->m_attrib.m_normalChannel[aIndex].m_z);
				}

				if (source->m_attrib.m_binormalChannel.GetCount())
				{
					AddBinormal(source->m_attrib.m_binormalChannel[aIndex].m_x, source->m_attrib.m_binormalChannel[aIndex].m_y, source->m_attrib.m_binormalChannel[aIndex].m_z);
				}

				if (source->m_attrib.m_uv0Channel.GetCount()) 
				{
					AddUV0(source->m_attrib.m_uv0Channel[aIndex].m_u, source->m_attrib.m_uv0Channel[aIndex].m_v);
				}

				if (source->m_attrib.m_uv1Channel.GetCount())
				{
					AddUV1(source->m_attrib.m_uv1Channel[aIndex].m_u, source->m_attrib.m_uv1Channel[aIndex].m_v);
				}
				ptr = ptr->m_next;
			} while (ptr != edge);
			EndBuildFace();
		}
	}
}

// create a convex hull
ndMeshEffect::ndMeshEffect(const dFloat64* const vertexCloud, dInt32 count, dInt32 strideInByte, dFloat64 distTol)
	:dPolyhedra()
	,m_name()
	,m_points()
	,m_attrib()
	,m_materials()
	,m_vertexBaseCount(0)
	,m_constructionIndex(0)
{
	Init();
	if (count >= 4) 
	{
		dConvexHull3d convexHull(vertexCloud, strideInByte, count, distTol);
		if (convexHull.GetCount()) 
		{
			dStack<dInt32> faceCountPool(convexHull.GetCount());
			dStack<dInt32> vertexIndexListPool(convexHull.GetCount() * 3);
	
			dInt32 index = 0;
			dMeshVertexFormat format;
			format.m_faceCount = convexHull.GetCount();
			format.m_faceIndexCount = &faceCountPool[0];
			format.m_vertex.m_indexList = &vertexIndexListPool[0];
			format.m_vertex.m_data = &convexHull.GetVertexPool()[0].m_x;
			format.m_vertex.m_strideInBytes = sizeof(dBigVector);
			for (dConvexHull3d::dListNode* faceNode = convexHull.GetFirst(); faceNode; faceNode = faceNode->GetNext()) 
			{
				dConvexHull3dFace& face = faceNode->GetInfo();
				faceCountPool[index] = 3;
				vertexIndexListPool[index * 3 + 0] = face.m_index[0];
				vertexIndexListPool[index * 3 + 1] = face.m_index[1];
				vertexIndexListPool[index * 3 + 2] = face.m_index[2];
				index++;
			}
			BuildFromIndexList(&format);
			RepairTJoints();
		}
	}
}

ndMeshEffect::ndMeshEffect(const ndShapeInstance& shape)
	:dPolyhedra()
	, m_name()
	, m_points()
	, m_attrib()
	, m_materials()
	, m_vertexBaseCount(0)
	, m_constructionIndex(0)
{
	class ndMeshEffectBuilder : public ndShapeDebugCallback
	{
	public:
		ndMeshEffectBuilder()
			:ndShapeDebugCallback()
			, m_vertex(1024)
			, m_faceMaterial(1024)
			, m_faceIndexCount(1024)
			, m_brush(0)
		{
		}

		void DrawPolygon(dInt32 vertexCount, const dVector* const faceVertex)
		{
			const dFloat64 brush = m_brush;
			m_faceIndexCount.PushBack(vertexCount);
			m_faceMaterial.PushBack(0);
			for (dInt32 i = 0; i < vertexCount; i++)
			{
				const dBigVector point(faceVertex[i].m_x, faceVertex[i].m_y, faceVertex[i].m_z, brush);
				m_vertex.PushBack(point);
			}
		}

		dArray<dBigVector> m_vertex;
		dArray<dInt32> m_faceMaterial;
		dArray<dInt32> m_faceIndexCount;

		dInt32 m_brush;
		dInt32 m_materialIndex;
	};
	ndMeshEffectBuilder builder;

	Init();
	if (((ndShape*)shape.GetShape())->GetAsShapeCompound())
	{
		dAssert(0);
		//	dgCollisionInfo collisionInfo;
		//	collision->GetCollisionInfo(&collisionInfo);
		//
		//	dInt32 brush = 0;
		//	dMatrix matrix(collisionInfo.m_offsetMatrix);
		//	dgCollisionCompound* const compoundCollision = (dgCollisionCompound*)collision->GetChildShape();
		//	for (dTree<dgCollisionCompound::dgNodeBase*, dInt32>::dTreeNode* node = compoundCollision->GetFirstNode(); node; node = compoundCollision->GetNextNode(node)) {
		//		builder.m_brush = brush;
		//		brush++;
		//		dgCollisionInstance* const childShape = compoundCollision->GetCollisionFromNode(node);
		//		childShape->DebugCollision(matrix, (dgCollision::OnDebugCollisionMeshCallback) dgMeshEffectBuilder::GetShapeFromCollision, &builder);
		//	}
		//
	}
	else
	{
		dMatrix matrix(dGetIdentityMatrix());
		shape.DebugShape(matrix, builder);
	}

	dStack<dInt32>indexListBuffer(builder.m_vertex.GetCount());
	dInt32* const indexList = &indexListBuffer[0];
	dVertexListToIndexList(&builder.m_vertex[0].m_x, sizeof(dBigVector), 4, builder.m_vertex.GetCount(), &indexList[0], DG_VERTEXLIST_INDEXLIST_TOL);

	ndMeshEffect::dMeshVertexFormat vertexFormat;
	vertexFormat.m_faceCount = builder.m_faceIndexCount.GetCount();
	vertexFormat.m_faceIndexCount = &builder.m_faceIndexCount[0];
	vertexFormat.m_faceMaterial = &builder.m_faceIndexCount[0];
	vertexFormat.m_vertex.m_data = &builder.m_vertex[0].m_x;
	vertexFormat.m_vertex.m_strideInBytes = sizeof(dBigVector);
	vertexFormat.m_vertex.m_indexList = &indexList[0];

	m_materials.PushBack(dMaterial());
	BuildFromIndexList(&vertexFormat);
	RepairTJoints();
	CalculateNormals(dFloat32(45.0f * dDegreeToRad));
}

dMatrix ndMeshEffect::CalculateOOBB(dBigVector& size) const
{
	dMatrix sphere(CalculateSphere(size, &m_points.m_vertex[0].m_x, sizeof(dBigVector)));
	//size = sphere.m_size;
	//size.m_w = 0.0f;

	//	dMatrix permuation (dGetIdentityMatrix());
	//	permuation[0][0] = dFloat32 (0.0f);
	//	permuation[0][1] = dFloat32 (1.0f);
	//	permuation[1][1] = dFloat32 (0.0f);
	//	permuation[1][2] = dFloat32 (1.0f);
	//	permuation[2][2] = dFloat32 (0.0f);
	//	permuation[2][0] = dFloat32 (1.0f);
	//	while ((size.m_x < size.m_y) || (size.m_x < size.m_z)) {
	//		sphere = permuation * sphere;
	//		size = permuation.UnrotateVector(size);
	//	}

	return sphere;
}

void ndMeshEffect::CalculateAABB(dBigVector& minBox, dBigVector& maxBox) const
{
	dBigVector minP(dFloat64(1.0e15f), dFloat64(1.0e15f), dFloat64(1.0e15f), dFloat64(0.0f));
	dBigVector maxP(-dFloat64(1.0e15f), -dFloat64(1.0e15f), -dFloat64(1.0e15f), dFloat64(0.0f));

	dPolyhedra::Iterator iter(*this);
	const dBigVector* const points = &m_points.m_vertex[0];
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &(*iter);
		const dBigVector& p(points[edge->m_incidentVertex]);

		minP.m_x = dMin(p.m_x, minP.m_x);
		minP.m_y = dMin(p.m_y, minP.m_y);
		minP.m_z = dMin(p.m_z, minP.m_z);

		maxP.m_x = dMax(p.m_x, maxP.m_x);
		maxP.m_y = dMax(p.m_y, maxP.m_y);
		maxP.m_z = dMax(p.m_z, maxP.m_z);
	}

	minBox = minP;
	maxBox = maxP;
}

dFloat64 ndMeshEffect::CalculateVolume() const
{
	dPolyhedraMassProperties localData;

	dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		dVector points[256];

		dEdge* face = &(*iter);
		if ((face->m_incidentFace > 0) && (face->m_mark != mark)) 
		{
			dInt32 count = 0;
			dEdge* ptr = face;
			do 
			{
				points[count] = m_points.m_vertex[ptr->m_incidentVertex];
				count++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);
			localData.AddCGFace(count, points);
		}
	}

	dVector com;
	dVector inertia;
	dVector crossInertia;
	dFloat32 volume = localData.MassProperties(com, inertia, crossInertia);
	return volume;
}

ndMeshEffect* ndMeshEffect::GetNextLayer(dInt32 mark)
{
	Iterator iter(*this);
	dEdge* edge = nullptr;
	for (iter.Begin(); iter; iter++) 
	{
		edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) 
		{
			break;
		}
	}

	if (!edge) 
	{
		return nullptr;
	}

	const dInt32 layer = m_points.m_layers.GetCount() ? m_points.m_layers[edge->m_incidentVertex] : 0;
	dPolyhedra polyhedra;

	polyhedra.BeginFace();
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge1 = &(*iter);
		if ((edge1->m_mark < mark) && (edge1->m_incidentFace > 0)) 
		{
			const dInt32 thislayer = m_points.m_layers.GetCount() ? m_points.m_layers[edge1->m_incidentVertex] : 0;
			if (thislayer == layer) 
			{
				dEdge* ptr = edge1;
				dInt32 count = 0;
				dInt32 faceIndex[256];
				dInt64 faceDataIndex[256];
				do 
				{
					ptr->m_mark = mark;
					faceIndex[count] = ptr->m_incidentVertex;
					faceDataIndex[count] = ptr->m_userData;
					count++;
					dAssert(count < dInt32(sizeof(faceIndex) / sizeof(faceIndex[0])));
					ptr = ptr->m_next;
				} while (ptr != edge1);
				polyhedra.AddFace(count, &faceIndex[0], &faceDataIndex[0]);
			}
		}
	}
	polyhedra.EndFace();

	ndMeshEffect* solid = nullptr;
	if (polyhedra.GetCount()) 
	{
		solid = new ndMeshEffect(polyhedra, *this);
		solid->SetLRU(mark);
	}
	return solid;
}

ndShapeInstance* ndMeshEffect::CreateConvexCollision(dFloat64 tolerance) const
{
	dStack<dVector> poolPtr(m_points.m_vertex.GetCount() * 2);
	dVector* const pool = &poolPtr[0];
	
	dBigVector minBox;
	dBigVector maxBox;
	CalculateAABB(minBox, maxBox);
	//dVector com ((minBox + maxBox).Scale (dFloat32 (0.5f)));
	dVector com((minBox + maxBox) * dVector::m_half);
	
	dInt32 count = 0;
	dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const vertex = &(*iter);
		if (vertex->m_mark != mark) 
		{
			dEdge* ptr = vertex;
			do 
			{
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != vertex);
	
			if (count < dInt32(poolPtr.GetElementsCount())) 
			{
				const dBigVector p(m_points.m_vertex[vertex->m_incidentVertex]);
				pool[count] = dVector(p) - com;
				count++;
			}
		}
	}
	
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit += matrix.RotateVector(com);
	matrix.m_posit.m_w = dFloat32(1.0f);
	
	ndShapeConvexHull* const collision = new ndShapeConvexHull(count, sizeof(dVector), dFloat32(tolerance), &pool[0].m_x);
	if (!collision->GetConvexVertexCount()) 
	{
		collision->Release();
		return nullptr;
	}
	ndShapeInstance* const instance = new ndShapeInstance(collision);
	//collision->Release();
	return instance;
}

void ndMeshEffect::ConvertToPolygons()
{
	UnpackPoints();
	dPolyhedra leftOversOut;
	dPolyhedra::ConvexPartition(&m_points.m_vertex[0].m_x, sizeof(dBigVector), &leftOversOut);
	dAssert(leftOversOut.GetCount() == 0);

	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &iter.GetNode()->GetInfo();
		edge->m_userData = (edge->m_incidentFace) > 0 ? edge->m_incidentVertex : 0;
	}
	PackPoints(dFloat32(1.0e-24f));

	RepairTJoints();
	dAssert(Sanity());
}


bool ndMeshEffect::HasOpenEdges() const
{
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const face = &(*iter);
		if (face->m_incidentFace < 0) 
		{
			return true;
		}
	}
	return false;
}

void ndMeshEffect::RemoveUnusedVertices(dInt32* const vertexMapResult)
{
	dAssert(!vertexMapResult);
	UnpackAttibuteData();
	PackAttibuteData();
	UnpackPoints();
	PackPoints(dFloat32(1.0e-24f));
}

dEdge* ndMeshEffect::InsertEdgeVertex(dEdge* const edge, dFloat64 param)
{
	dEdge* const twin = edge->m_twin;
	AddInterpolatedEdgeAttribute(edge, param);

	dInt32 edgeAttrV0 = dInt32(edge->m_userData);
	dInt32 twinAttrV0 = dInt32(twin->m_userData);

	dEdge* const faceA0 = edge->m_next;
	dEdge* const faceA1 = edge->m_prev;
	dEdge* const faceB0 = twin->m_next;
	dEdge* const faceB1 = twin->m_prev;
	SpliteEdge(m_points.m_vertex.GetCount() - 1, edge);

	faceA0->m_prev->m_userData = dUnsigned64(m_attrib.m_pointChannel.GetCount() - 2);
	faceA1->m_next->m_userData = dUnsigned64(edgeAttrV0);

	faceB0->m_prev->m_userData = dUnsigned64(m_attrib.m_pointChannel.GetCount() - 1);
	faceB1->m_next->m_userData = dUnsigned64(twinAttrV0);
	return faceA1->m_next;
}

void ndMeshEffect::AddInterpolatedEdgeAttribute(dEdge* const edge, dFloat64 param)
{
	dFloat64 t1 = param;
	dFloat64 t0 = dFloat64(1.0f) - t1;
	dAssert(t1 >= dFloat64(0.0f));
	dAssert(t1 <= dFloat64(1.0f));

	const dInt32 vertexIndex = m_points.m_vertex.GetCount();
	m_points.m_vertex.PushBack(m_points.m_vertex[edge->m_incidentVertex].Scale(t0) + m_points.m_vertex[edge->m_next->m_incidentVertex].Scale(t1));
	if (m_points.m_layers.GetCount())
	{
		m_points.m_layers.PushBack(m_points.m_layers[edge->m_incidentVertex]);
	}

	m_attrib.m_pointChannel.PushBack(vertexIndex);
	m_attrib.m_pointChannel.PushBack(vertexIndex);

	if (m_attrib.m_materialChannel.GetCount()) 
	{
		m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[dInt32(edge->m_userData)]);
		m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[dInt32(edge->m_twin->m_userData)]);
	}

	if (m_attrib.m_normalChannel.GetCount())
	{
		dTriplex edgeNormal;
		dTriplex edgeNormal0(m_attrib.m_normalChannel[dInt32(edge->m_userData)]);
		dTriplex edgeNormal1(m_attrib.m_normalChannel[dInt32(edge->m_next->m_userData)]);
		edgeNormal.m_x = edgeNormal0.m_x * dFloat32(t0) + edgeNormal1.m_x * dFloat32(t1);
		edgeNormal.m_y = edgeNormal0.m_y * dFloat32(t0) + edgeNormal1.m_y * dFloat32(t1);
		edgeNormal.m_z = edgeNormal0.m_z * dFloat32(t0) + edgeNormal1.m_z * dFloat32(t1);
		m_attrib.m_normalChannel.PushBack(edgeNormal);

		dTriplex twinNormal;
		dTriplex twinNormal0(m_attrib.m_normalChannel[dInt32(edge->m_twin->m_next->m_userData)]);
		dTriplex twinNormal1(m_attrib.m_normalChannel[dInt32(edge->m_twin->m_userData)]);
		twinNormal.m_x = twinNormal0.m_x * dFloat32(t0) + twinNormal1.m_x * dFloat32(t1);
		twinNormal.m_y = twinNormal0.m_y * dFloat32(t0) + twinNormal1.m_y * dFloat32(t1);
		twinNormal.m_z = twinNormal0.m_z * dFloat32(t0) + twinNormal1.m_z * dFloat32(t1);
		m_attrib.m_normalChannel.PushBack(twinNormal);
	}
	if (m_attrib.m_binormalChannel.GetCount()) 
	{
		dAssert(0);
	}

	if (m_attrib.m_uv0Channel.GetCount()) 
	{
		dAttibutFormat::dgUV edgeUV;
		dAttibutFormat::dgUV edgeUV0(m_attrib.m_uv0Channel[dInt32(edge->m_userData)]);
		dAttibutFormat::dgUV edgeUV1(m_attrib.m_uv0Channel[dInt32(edge->m_next->m_userData)]);
		edgeUV.m_u = edgeUV0.m_u * dFloat32(t0) + edgeUV1.m_u * dFloat32(t1);
		edgeUV.m_v = edgeUV0.m_v * dFloat32(t0) + edgeUV1.m_v * dFloat32(t1);
		m_attrib.m_uv0Channel.PushBack(edgeUV);

		dAttibutFormat::dgUV twinUV;
		dAttibutFormat::dgUV twinUV0(m_attrib.m_uv0Channel[dInt32(edge->m_twin->m_next->m_userData)]);
		dAttibutFormat::dgUV twinUV1(m_attrib.m_uv0Channel[dInt32(edge->m_twin->m_userData)]);
		twinUV.m_u = twinUV0.m_u * dFloat32(t0) + twinUV1.m_u * dFloat32(t1);
		twinUV.m_v = twinUV0.m_v * dFloat32(t0) + twinUV1.m_v * dFloat32(t1);
		m_attrib.m_uv0Channel.PushBack(twinUV);
	}

	if (m_attrib.m_uv1Channel.GetCount())
	{
		dAssert(0);
	}

	if (m_attrib.m_colorChannel.GetCount())
	{
		dAssert(0);
	}
}
