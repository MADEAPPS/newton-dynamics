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
#include "ndSort.h"
#include "ndStack.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndShape.h"
#include "ndMeshEffect.h"
#include "ndShapeInstance.h"
#include "ndShapeConvexHull.h"

#define D_VERTEXLIST_INDEX_LIST_BASH (1024)

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
		dNode* const node = iter.GetNode();
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
				ndInt64 attibuteIndex[1024];
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
				callback (userData, attibuteIndex, vertexCount * sizeof (ndInt64));
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
				dEdge* const face = &((dNode*)faceNode)->GetInfo();
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
			dgFitnessList::dNode* const node = iter.GetNode();
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

void ndMeshEffect::dMeshBVH::GetOverlapNodes(ndList<dgMeshBVHNode*>& overlapNodes, const ndBigVector& p0, const ndBigVector& p1) const
{
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	ndInt32 stack = 1;
	stackPool[0] = m_rootNode;

	ndVector l0(p0);
	ndVector l1(p1);

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
				dAssert(stack < ndInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dAssert(stack < ndInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));
			}
		}
	}
}

/*
ndFloat64 ndMeshEffect::dMeshBVH::VertexRayCast (const ndBigVector& p0, const ndBigVector& p1) const
{
	dAssert (0);

	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	ndInt32 stack = 1;
	stackPool[0] = m_rootNode;

	ndVector l0(p0);
	ndVector l1(p1);
	ndBigVector p1p0 (p1 - p0);
	ndFloat64 den = p1p0 % p1p0;

	const ndBigVector* const points = (ndBigVector*) m_mesh->GetVertexPool();
	while (stack) {
		stack --;
		dgMeshBVHNode* const me = stackPool[stack];

		if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
			if (!me->m_left) {
				dAssert (!me->m_right);

				ndEdge* ptr = me->m_face;
				do {
					ndInt32 index = ptr->m_incidentVertex;
					const ndBigVector& q0 = points[index];
					ndBigVector q0p0 (q0 - p0);
					ndFloat64 alpha = q0p0 % p1p0;
					if ((alpha > (DG_BOOLEAN_ZERO_TOLERANCE * den)) && (alpha < (den - DG_BOOLEAN_ZERO_TOLERANCE))) {
						ndBigVector dist (p0 + p1p0.Scale (alpha / den) - q0);
						ndFloat64 dist2 = dist % dist;
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
				dAssert (stack < ndInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dAssert (stack < ndInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
			}
		}
	}
	return 1.2f;
}


bool ndMeshEffect::dMeshBVH::RayRayIntersect (ndEdge* const edge, const ndMeshEffect* const otherMesh, ndEdge* const otherEdge, ndFloat64& param, ndFloat64& otherParam) const
{
	dAssert (0);

	ndBigVector ray_p0 (m_mesh->m_points[edge->m_incidentVertex]);
	ndBigVector ray_p1 (m_mesh->m_points[edge->m_twin->m_incidentVertex]);

	ndBigVector ray_q0 (otherMesh->m_points[otherEdge->m_incidentVertex]);
	ndBigVector ray_q1 (otherMesh->m_points[otherEdge->m_twin->m_incidentVertex]);

	ndBigVector p1p0 (ray_p1 - ray_p0);
	ndBigVector q1q0 (ray_q1 - ray_q0);
	ndBigVector p0q0 (ray_p0 - ray_q0);

	ndFloat64 a = p1p0 % p1p0;        // always >= 0
	ndFloat64 c = q1q0 % q1q0;        // always >= 0
	ndFloat64 b = p1p0 % q1q0;

	ndFloat64 d = (p1p0 % p0q0);
	ndFloat64 e = (q1q0 % p0q0);
	ndFloat64 den = a * c - b * b;   // always >= 0
	// compute the line parameters of the two closest points
	if (den < DG_BOOLEAN_ZERO_TOLERANCE) {
		// the lines are almost parallel
		return false;
	} else {
		// get the closest points on the infinite lines
		ndFloat64 t = b * e - c * d;
		ndFloat64 s = a * e - b * d;

		if (t < (DG_BOOLEAN_ZERO_TOLERANCE * den) || (s < (DG_BOOLEAN_ZERO_TOLERANCE * den)) || (t > (den - DG_BOOLEAN_ZERO_TOLERANCE)) ||  (s > (den - DG_BOOLEAN_ZERO_TOLERANCE))) {
			return false;
		}
		//ndBigVector normal (p1p0 * q1q0);
		ndBigVector r0 (ray_p0 + p1p0.Scale (t / den));
		ndBigVector r1 (ray_q0 + q1q0.Scale (s / den));
		ndBigVector r1r0 (r1 - r0);
		ndFloat64 dist2 = r1r0 % r1r0;
		if (dist2 > (DG_BOOLEAN_ZERO_TOLERANCE * DG_BOOLEAN_ZERO_TOLERANCE)) {
			return false;
		}

		param = t / den;
		otherParam = s / den;
	}
	return true;
}
*/


ndFloat64 ndMeshEffect::dMeshBVH::RayFaceIntersect(const dgMeshBVHNode* const faceNode, const ndBigVector& p0, const ndBigVector& p1, void* const userData) const
{
	dAssert(0);
	return 0;
	/*
		ndBigVector normal (m_mesh->FaceNormal(faceNode->m_face, m_mesh->GetVertexPool(), sizeof(ndBigVector)));

		ndBigVector diff (p1 - p0);

		ndFloat64 tOut = 2.0f;
		const ndBigVector* const points = (ndBigVector*) m_mesh->GetVertexPool();
		ndFloat64 dir = normal.DotProduct3(diff);
		if (dir < 0.0f) {
			ndEdge* ptr = faceNode->m_face;
			do {
				ndInt32 index0 = ptr->m_incidentVertex;
				ndInt32 index1 = ptr->m_next->m_incidentVertex;
				ndBigVector p0v0 (points[index0] - p0);
				ndBigVector p0v1 (points[index1] - p0);
				ndFloat64 alpha = p0v0.DotProduct3(diff.CrossProduct(p0v1));
				if (alpha <= 0.0f) {
					return 1.2f;
				}

				ptr = ptr->m_next;
			} while (ptr != faceNode->m_face);

			ndInt32 index0 = ptr->m_incidentVertex;
			ndBigVector p0v0 (points[index0] - p0);
			tOut = normal.DotProduct3(p0v0);
			ndFloat64 dist = normal.DotProduct3(diff);
			tOut = tOut / dist;

		} else if (doubleSidedFaces && (dir > 0.0f)) {
			ndEdge* ptr = faceNode->m_face;
			do {
				ndInt32 index0 = ptr->m_incidentVertex;
				ndInt32 index1 = ptr->m_prev->m_incidentVertex;
				ndBigVector p0v0 (points[index0] - p0);
				ndBigVector p0v1 (points[index1] - p0);
				ndFloat64 alpha = p0v0.DotProduct3(diff.CrossProduct(p0v1));
				if (alpha <= 0.0f) {
					return 1.2f;
				}

				ptr = ptr->m_prev;
			} while (ptr != faceNode->m_face);

			ndInt32 index0 = ptr->m_incidentVertex;
			ndBigVector p0v0 (points[index0] - p0);
			tOut = normal.DotProduct3(p0v0);
			ndFloat64 dist = normal.DotProduct3(diff);
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


void ndMeshEffect::dMeshBVH::FaceRayCast(const ndBigVector& p0, const ndBigVector& p1, void* const userData) const
{
	dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];

	ndInt32 stack = 1;
	dgMeshBVHNode* node = nullptr;

	stackPool[0] = m_rootNode;
	ndFloat64 maxParam = ndFloat32(1.2f);

	ndVector l0(p0);
	ndVector l1(p1);
	l0 = l0 & ndVector::m_triplexMask;
	l1 = l1 & ndVector::m_triplexMask;
	ndFastRay ray(l0, l1);
	while (stack) {
		stack--;
		dgMeshBVHNode* const me = stackPool[stack];
		if (me && ray.BoxTest(me->m_p0, me->m_p1)) {
			if (!me->m_left) {
				dAssert(!me->m_right);
				ndFloat64 param = RayFaceIntersect(me, p0, p1, userData);
				if (param < ndFloat64(0.0f)) {
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
				dAssert(stack < ndInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));

				stackPool[stack] = me->m_right;
				stack++;
				dAssert(stack < ndInt32(sizeof(stackPool) / sizeof(dgMeshBVHNode*)));
			}
		}
	}
}

ndMeshEffect::ndMeshEffect(dgMemoryAllocator* const allocator, const ndMatrix& planeMatrix, ndFloat32 witdth, ndFloat32 breadth, ndInt32 material, const ndMatrix& textureMatrix0, const ndMatrix& textureMatrix1)
	:ndPolyhedra(allocator)
	, m_points(allocator)
	, m_attrib(allocator)
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	dAssert(0);
	Init();
	/*
		ndInt32 index[4];
		ndInt64 attrIndex[4];
		ndBigVector face[4];

	//	Init();

		face[0] = ndBigVector (ndFloat32 (0.0f), -witdth, -breadth, ndFloat32 (0.0f));
		face[1] = ndBigVector (ndFloat32 (0.0f),  witdth, -breadth, ndFloat32 (0.0f));
		face[2] = ndBigVector (ndFloat32 (0.0f),  witdth,  breadth, ndFloat32 (0.0f));
		face[3] = ndBigVector (ndFloat32 (0.0f), -witdth,  breadth, ndFloat32 (0.0f));

		for (ndInt32 i = 0; i < 4; i ++) {
			ndBigVector uv0 (textureMatrix0.TransformVector(face[i]));
			ndBigVector uv1 (textureMatrix1.TransformVector(face[i]));

			m_points[i] = planeMatrix.TransformVector(face[i]);

			m_attrib[i].m_vertex.m_x = m_points[i].m_x;
			m_attrib[i].m_vertex.m_y = m_points[i].m_y;
			m_attrib[i].m_vertex.m_z = m_points[i].m_z;
			m_attrib[i].m_vertex.m_w = ndFloat64 (0.0f);

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
	:ndPolyhedra(source)
	, m_points(source.m_points)
	, m_attrib(source.m_attrib)
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	Init();
}

ndMeshEffect::ndMeshEffect(dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData)
	:ndPolyhedra(allocator)
	, m_points(allocator)
	, m_attrib(allocator)
	, m_vertexBaseCount(-1)
	, m_constructionIndex(0)
{
	Init();
	dAssert(0);
	/*
		ndInt32 faceCount;
		deserialization (userData, &faceCount, sizeof (ndInt32));
		deserialization (userData, &m_pointCount, sizeof (ndInt32));
		deserialization (userData, &m_atribCount, sizeof (ndInt32));
		deserialization (userData, &m_atribCount, sizeof (ndInt32));

		m_maxPointCount = m_pointCount;
		m_maxAtribCount = m_atribCount;

		m_points = (ndBigVector*) GetAllocator()->MallocLow(ndInt32 (m_pointCount * sizeof(ndBigVector)));
		m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(ndInt32 (m_atribCount * sizeof(dgVertexAtribute)));

		deserialization (userData, m_points, m_pointCount * sizeof (ndBigVector));
		deserialization (userData, m_attrib, m_atribCount * sizeof (dgVertexAtribute));

		BeginFace();
		for (ndInt32 i = 0; i < faceCount; i ++) {
			ndInt32 vertexCount;
			ndInt32 face[1024];
			ndInt64 attrib[1024];
			deserialization (userData, &vertexCount, sizeof (ndInt32));
			deserialization (userData, face, vertexCount * sizeof (ndInt32));
			deserialization (userData, attrib, vertexCount * sizeof (ndInt64));
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
		for (ndInt32 i = 0; i < m_pointCount; i ++ ) {
			dTrace (("%d-> %f %f %f\n", i, m_points[i].m_x, m_points[i].m_y, m_points[i].m_z));
		}


		ndTree<ndEdge*, ndEdge*>filter(GetAllocator());
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if (!filter.Find(edge)) {
				ndEdge* ptr = edge;
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

void ndMeshEffect::OptimizePoints()
{
}

void ndMeshEffect::OptimizeAttibutes()
{
	UnpackAttibuteData();
	PackAttibuteData();
}


ndInt32 ndMeshEffect::GetTotalFaceCount() const
{
	return GetFaceCount();
}

ndInt32 ndMeshEffect::GetTotalIndexCount() const
{
	Iterator iter(*this);
	ndInt32 count = 0;
	ndInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter++) {
		ndEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		ndEdge* ptr = edge;
		do {
			count++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}

void ndMeshEffect::GetFaces(ndInt32* const facesIndex, ndInt32* const materials, void** const faceNodeList) const
{
	Iterator iter(*this);

	ndInt32 faces = 0;
	ndInt32 indexCount = 0;
	ndInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter++) {
		ndEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		ndInt32 faceCount = 0;
		ndEdge* ptr = edge;
		do {
			//			indexList[indexCount] = ndInt32 (ptr->m_userData);
			faceNodeList[indexCount] = GetNodeFromInfo(*ptr);
			indexCount++;
			faceCount++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);

		facesIndex[faces] = faceCount;
		//materials[faces] = dgFastInt(m_attrib[ndInt32 (edge->m_userData)].m_material);
		materials[faces] = m_attrib.m_materialChannel.m_count ? m_attrib.m_materialChannel[ndInt32(edge->m_userData)] : 0;
		faces++;
	}
}

void* ndMeshEffect::GetFirstVertex() const
{
	Iterator iter(*this);
	iter.Begin();

	ndNode* node = nullptr;
	if (iter) {
		ndInt32 mark = IncLRU();
		node = iter.GetNode();

		ndEdge* const edge = &node->GetInfo();
		ndEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);
	}
	return node;
}

void* ndMeshEffect::GetNextVertex(const void* const vertex) const
{
	ndNode* const node0 = (ndNode*)vertex;
	ndInt32 mark = node0->GetInfo().m_mark;

	Iterator iter(*this);
	iter.Set(node0);
	for (iter++; iter; iter++) {
		ndNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			ndEdge* const edge = &node->GetInfo();
			ndEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			return node;
		}
	}
	return nullptr;
}

ndInt32 ndMeshEffect::GetVertexIndex(const void* const vertex) const
{
	ndNode* const node = (ndNode*)vertex;
	ndEdge* const edge = &node->GetInfo();
	return edge->m_incidentVertex;
}

void* ndMeshEffect::GetFirstPoint() const
{
	Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		ndNode* const node = iter.GetNode();
		ndEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node;
		}
	}
	return nullptr;
}

void* ndMeshEffect::GetNextPoint(const void* const point) const
{
	Iterator iter(*this);
	iter.Set((ndNode*)point);
	for (iter++; iter; iter++) {
		ndNode* const node = iter.GetNode();
		ndEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node;
		}
	}
	return nullptr;
}

ndInt32 ndMeshEffect::GetPointIndex(const void* const point) const
{
	ndNode* const node = (ndNode*)point;
	ndEdge* const edge = &node->GetInfo();
	return ndInt32(edge->m_userData);
}


ndInt32 ndMeshEffect::GetVertexIndexFromPoint(const void* const point) const
{
	return GetVertexIndex(point);
}

ndEdge* ndMeshEffect::SpliteFace(ndInt32 v0, ndInt32 v1)
{
	if (!FindEdge(v0, v1)) {
		ndPolyhedra::ndPairKey key(v0, 0);
		ndNode* const node = FindGreaterEqual(key.GetVal());
		if (node) {
			ndEdge* const edge = &node->GetInfo();
			ndEdge* edge0 = edge;
			do {
				if (edge0->m_incidentFace > 0) {
					for (ndEdge* edge1 = edge0->m_next->m_next; edge1 != edge0->m_prev; edge1 = edge1->m_next) {
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

const ndEdge* ndMeshEffect::GetPolyhedraEdgeFromNode(const void* const edge) const
{
	ndNode* const node = (ndNode*)edge;
	return &node->GetInfo();
}

void* ndMeshEffect::GetFirstEdge() const
{
	Iterator iter(*this);
	iter.Begin();

	ndNode* node = nullptr;
	if (iter) {
		ndInt32 mark = IncLRU();

		node = iter.GetNode();

		ndEdge* const edge = &node->GetInfo();
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}
	return node;
}

void* ndMeshEffect::GetNextEdge(const void* const edge) const
{
	ndNode* const node0 = (ndNode*)edge;
	ndInt32 mark = node0->GetInfo().m_mark;

	Iterator iter(*this);
	iter.Set(node0);
	for (iter++; iter; iter++) {
		ndNode* const node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			node->GetInfo().m_mark = mark;
			node->GetInfo().m_twin->m_mark = mark;
			return node;
		}
	}
	return nullptr;
}


void ndMeshEffect::GetEdgeIndex(const void* const edge, ndInt32& v0, ndInt32& v1) const
{
	ndNode* const node = (ndNode*)edge;
	v0 = node->GetInfo().m_incidentVertex;
	v1 = node->GetInfo().m_twin->m_incidentVertex;
}

//void* ndMeshEffect::FindEdge (ndInt32 v0, ndInt32 v1) const
//{
//	return FindEdgeNode(v0, v1);
//}

//void ndMeshEffect::GetEdgeAttributeIndex (const void* edge, ndInt32& v0, ndInt32& v1) const
//{
//	ndNode* node = (ndNode*) edge;
//	v0 = ndInt32 (node->GetInfo().m_userData);
//	v1 = ndInt32 (node->GetInfo().m_twin->m_userData);
//}


void* ndMeshEffect::GetFirstFace() const
{
	Iterator iter(*this);
	iter.Begin();

	ndNode* node = nullptr;
	if (iter) {
		ndInt32 mark = IncLRU();
		node = iter.GetNode();

		ndEdge* const edge = &node->GetInfo();
		ndEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}

	return node;
}

void* ndMeshEffect::GetNextFace(const void* const face) const
{
	ndNode* const node0 = (ndNode*)face;
	ndInt32 mark = node0->GetInfo().m_mark;

	Iterator iter(*this);
	iter.Set(node0);
	for (iter++; iter; iter++) {
		ndNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			ndEdge* const edge = &node->GetInfo();
			ndEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
			return node;
		}
	}
	return nullptr;
}


ndInt32 ndMeshEffect::IsFaceOpen(const void* const face) const
{
	ndNode* const node = (ndNode*)face;
	ndEdge* const edge = &node->GetInfo();
	return (edge->m_incidentFace > 0) ? 0 : 1;
}

ndInt32 ndMeshEffect::GetFaceMaterial(const void* const face) const
{
	ndNode* const node = (ndNode*)face;
	ndEdge* const edge = &node->GetInfo();
	return ndInt32(m_attrib.m_materialChannel.m_count ? m_attrib.m_materialChannel[ndInt32(edge->m_userData)] : 0);
}

void ndMeshEffect::SetFaceMaterial(const void* const face, ndInt32 mateialID)
{
	if (m_attrib.m_materialChannel.m_count) {
		ndNode* const node = (ndNode*)face;
		ndEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			ndEdge* ptr = edge;
			do {
				//dgVertexAtribute* const attrib = &m_attrib[ptr->m_userData];
				//attrib->m_material = ndFloat64 (mateialID);
				m_attrib.m_materialChannel[ndInt32(edge->m_userData)] = mateialID;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
}

ndInt32 ndMeshEffect::GetFaceIndexCount(const void* const face) const
{
	ndInt32 count = 0;
	ndNode* node = (ndNode*)face;
	ndEdge* const edge = &node->GetInfo();
	ndEdge* ptr = edge;
	do {
		count++;
		ptr = ptr->m_next;
	} while (ptr != edge);
	return count;
}

void ndMeshEffect::GetFaceIndex(const void* const face, ndInt32* const indices) const
{
	ndInt32 count = 0;
	ndNode* node = (ndNode*)face;
	ndEdge* const edge = &node->GetInfo();
	ndEdge* ptr = edge;
	do {
		indices[count] = ptr->m_incidentVertex;
		count++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}

void ndMeshEffect::GetFaceAttributeIndex(const void* const face, ndInt32* const indices) const
{
	ndInt32 count = 0;
	ndNode* node = (ndNode*)face;
	ndEdge* const edge = &node->GetInfo();
	ndEdge* ptr = edge;
	do {
		indices[count] = ndInt32(ptr->m_userData);
		count++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}


ndBigVector ndMeshEffect::CalculateFaceNormal(const void* const face) const
{
	ndNode* const node = (ndNode*)face;
	ndEdge* const faceEdge = &node->GetInfo();
	ndBigVector normal(FaceNormal(faceEdge, &m_points.m_vertex[0].m_x, sizeof(ndBigVector)));
	dAssert(normal.m_w == ndFloat32(0.0f));
	//normal = normal.Scale (1.0f / sqrt (normal.DotProduct3(normal)));
	normal = normal.Normalize();
	return normal;
}

/*
ndInt32 GetTotalFaceCount() const;
{
	ndInt32 mark;
	ndInt32 count;
	ndInt32 materialCount;
	ndInt32 materials[256];
	ndInt32 streamIndexMap[256];
	ndIndexArray* array;

	count = 0;
	materialCount = 0;

	array = (ndIndexArray*) GetAllocator()->MallocLow (4 * sizeof (ndInt32) * GetCount() + sizeof (ndIndexArray) + 2048);
	array->m_indexList = (ndInt32*)&array[1];

	mark = IncLRU();
	ndPolyhedra::Iterator iter (*this);
	memset(streamIndexMap, 0, sizeof (streamIndexMap));
	for(iter.Begin(); iter; iter ++){

		ndEdge* const edge;
		edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark)) {
			ndEdge* ptr;
			ndInt32 hashValue;
			ndInt32 index0;
			ndInt32 index1;

			ptr = edge;
			ptr->m_mark = mark;
			index0 = ndInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			index1 = ndInt32 (ptr->m_userData);

			ptr = ptr->m_next;
			do {
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = ndInt32 (ptr->m_userData);
				array->m_indexList[count * 4 + 3] = m_attrib[ndInt32 (edge->m_userData)].m_material;
				index1 = ndInt32 (ptr->m_userData);

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
void ndMeshEffect::GetWeightBlendChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt8* const buffer = (ndInt8*)bufferOut;
	for (ndInt32 i = 0; i < m_attrib.m_pointChannel.m_count; ++i) {
		const ndInt32 j = i * strideInByte;
		ndFloat32* const ptr = (ndFloat32*)&buffer[j];

		const ndInt32 index = m_attrib.m_pointChannel[i];
		const ndFloat32* const p = &m_points.m_weights[index].m_weightBlends[0];
		ptr[0] = ndFloat32(p[0]);
		ptr[1] = ndFloat32(p[1]);
		ptr[2] = ndFloat32(p[2]);
		ptr[3] = ndFloat32(p[3]);
	}
}

void ndMeshEffect::GetWeightIndexChannel(ndInt32 strideInByte, ndInt32* const bufferOut) const
{
	ndInt8* const buffer = (ndInt8*)bufferOut;
	for (ndInt32 i = 0; i < m_attrib.m_pointChannel.m_count; ++i) {
		const ndInt32 j = i * strideInByte;
		ndInt32* const ptr = (ndInt32*)&buffer[j];
		const ndInt32 index = m_attrib.m_pointChannel[i];
		const ndInt32* const p = &m_points.m_weights[index].m_controlIndex[0];
		ptr[0] = p[0];
		ptr[1] = p[1];
		ptr[2] = p[2];
		ptr[3] = p[3];
	}
}
*/


dgCollisionInstance* ndMeshEffect::CreateCollisionTree(dgWorld* const world, ndInt32 shapeID) const
{
	ndShapeStatic_bvh* const collision = new  (GetAllocator()) ndShapeStatic_bvh(world);

	collision->BeginBuild();

	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		ndNode* const faceNode = iter.GetNode();
		//ndEdge* const face = &(*iter);
		ndEdge* const face = &faceNode->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			ndInt32 count = 0;
			ndVector polygon[256];
			ndEdge* ptr = face;
			do {
				//polygon[count] = ndVector (m_points[ptr->m_incidentVertex]);
				polygon[count] = GetVertex(ptr->m_incidentVertex);
				count++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);
			//collision->AddFace(count, &polygon[0].m_x, sizeof (ndVector), ndInt32 (m_attrib[face->m_userData].m_material));
			collision->AddFace(count, &polygon[0].m_x, sizeof(ndVector), GetFaceMaterial(faceNode));
		}
	}
	collision->EndBuild(0);

	dgCollisionInstance* const instance = world->CreateInstance(collision, shapeID, dGetIdentityMatrix());
	collision->Release();
	return instance;
}

void ndMeshEffect::TransformMesh(const ndMatrix& matrix)
{
	dAssert(0);
	/*
	ndMatrix normalMatrix (matrix);
	normalMatrix.m_posit = ndVector (ndFloat32 (0.0f), ndFloat32 (0.0f), ndFloat32 (0.0f), ndFloat32 (1.0f));

	matrix.TransformTriplex (&m_points->m_x, sizeof (ndBigVector), &m_points->m_x, sizeof (ndBigVector), m_pointCount);
	matrix.TransformTriplex (&m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &m_attrib[0].m_vertex.m_x, sizeof (dgVertexAtribute), m_atribCount);
	normalMatrix.TransformTriplex (&m_attrib[0].m_normal_x, sizeof (dgVertexAtribute), &m_attrib[0].m_normal_x, sizeof (dgVertexAtribute), m_atribCount);
*/
}



//ndMeshEffect::dgVertexAtribute ndMeshEffect::InterpolateVertex (const ndBigVector& srcPoint, const ndEdge* const face) const
ndInt32 ndMeshEffect::InterpolateVertex(const ndBigVector& srcPoint, const ndEdge* const face) const
{
	dAssert(0);
	return 0;
	/*
		const ndBigVector point (srcPoint);

		dgVertexAtribute attribute;
		memset (&attribute, 0, sizeof (attribute));

	//	ndBigVector normal (FaceNormal(face, &m_points[0].m_x, sizeof(ndBigVector)));
	//	normal = normal.Scale (ndFloat64 (1.0f) / sqrt (normal % normal));
	//	attribute.m_vertex = srcPoint;
	//	attribute.m_normal_x = normal.m_x;
	//	attribute.m_normal_y = normal.m_y;
	//	attribute.m_normal_z = normal.m_z;

		ndFloat64 tol = ndFloat32 (1.0e-4f);
		for (ndInt32 i = 0; i < 4; i ++) {
			const ndEdge* ptr = face;
			const ndEdge* const edge0 = ptr;
			ndBigVector q0 (m_points[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const ndEdge* edge1 = ptr;
			ndBigVector q1 (m_points[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const ndEdge* edge2 = ptr;
			do {
				const ndBigVector q2 (m_points[ptr->m_incidentVertex]);

				ndBigVector p10 (q1 - q0);
				ndBigVector p20 (q2 - q0);

				ndFloat64 dot = p20.DotProduct3(p10);
				ndFloat64 mag1 = p10.DotProduct3(p10);
				ndFloat64 mag2 = p20.DotProduct3(p20);
				ndFloat64 collinear = dot * dot - mag2 * mag1;
				if (fabs (collinear) > ndFloat64 (1.0e-8f)) {
					ndBigVector p_p0 (point - q0);
					ndBigVector p_p1 (point - q1);
					ndBigVector p_p2 (point - q2);

					ndFloat64 alpha1 = p10.DotProduct3(p_p0);
					ndFloat64 alpha2 = p20.DotProduct3(p_p0);
					ndFloat64 alpha3 = p10.DotProduct3(p_p1);
					ndFloat64 alpha4 = p20.DotProduct3(p_p1);
					ndFloat64 alpha5 = p10.DotProduct3(p_p2);
					ndFloat64 alpha6 = p20.DotProduct3(p_p2);

					ndFloat64 vc = alpha1 * alpha4 - alpha3 * alpha2;
					ndFloat64 vb = alpha5 * alpha2 - alpha1 * alpha6;
					ndFloat64 va = alpha3 * alpha6 - alpha5 * alpha4;
					ndFloat64 den = va + vb + vc;
					ndFloat64 minError = den * (-tol);
					ndFloat64 maxError = den * (ndFloat32 (1.0f) + tol);
					if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {
						edge2 = ptr;

						den = ndFloat64 (1.0f) / (va + vb + vc);

						ndFloat64 alpha0 = ndFloat32 (va * den);
						ndFloat64 alpha1 = ndFloat32 (vb * den);
						ndFloat64 alpha2 = ndFloat32 (vc * den);

						const dgVertexAtribute& attr0 = m_attrib[edge0->m_userData];
						const dgVertexAtribute& attr1 = m_attrib[edge1->m_userData];
						const dgVertexAtribute& attr2 = m_attrib[edge2->m_userData];
						ndBigVector normal (attr0.m_normal_x * alpha0 + attr1.m_normal_x * alpha1 + attr2.m_normal_x * alpha2,
											attr0.m_normal_y * alpha0 + attr1.m_normal_y * alpha1 + attr2.m_normal_y * alpha2,
											attr0.m_normal_z * alpha0 + attr1.m_normal_z * alpha1 + attr2.m_normal_z * alpha2, ndFloat32 (0.0f));
						//normal = normal.Scale (ndFloat64 (1.0f) / sqrt (normal.DotProduct3(normal)));
						normal = normal.Normalize();

			#ifdef _DEBUG
						ndBigVector testPoint (attr0.m_vertex.m_x * alpha0 + attr1.m_vertex.m_x * alpha1 + attr2.m_vertex.m_x * alpha2,
											   attr0.m_vertex.m_y * alpha0 + attr1.m_vertex.m_y * alpha1 + attr2.m_vertex.m_y * alpha2,
											   attr0.m_vertex.m_z * alpha0 + attr1.m_vertex.m_z * alpha1 + attr2.m_vertex.m_z * alpha2, ndFloat32 (0.0f));
						dAssert (fabs (testPoint.m_x - point.m_x) < ndFloat32 (1.0e-2f));
						dAssert (fabs (testPoint.m_y - point.m_y) < ndFloat32 (1.0e-2f));
						dAssert (fabs (testPoint.m_z - point.m_z) < ndFloat32 (1.0e-2f));
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
			tol *= ndFloat64 (2.0f);
		}
		// this should never happens
		dAssert (0);
		return attribute;
	*/
}


#endif

class ndMeshEffect::dFormat::dSortCluster
{
	public:
	ndBigVector m_sum;
	ndBigVector m_sum2;
	ndInt32 m_start;
	ndInt32 m_count;
};

inline ndInt32 ndMeshEffect::dFormat::CompareVertex(const dSortKey* const ptr0, const dSortKey* const ptr1, void* const context)
{
	const dVertexSortData* const sortContext = (dVertexSortData*)context;
	const ndInt32 compIndex = sortContext->m_vertexSortIndex;
	const dChannel<ndBigVector, m_point>& points = *sortContext->m_points;
	const ndFloat64 x0 = points[ptr0->m_vertexIndex][compIndex];
	const ndFloat64 x1 = points[ptr1->m_vertexIndex][compIndex];

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

void ndMeshEffect::dPointFormat::CompressData(
	dPointFormat& output, ndInt32* const indexList, 
	dSortKey* const remapIndex, const dSortCluster& cluster, ndFloat32 tol)
{
	const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / cluster.m_count));
	const ndBigVector x2c(cluster.m_sum2.Scale(ndFloat32(1.0f) / cluster.m_count) - origin * origin);
	const ndBigVector variance((x2c.GetMax(ndVector::m_zero)).Sqrt());

	ndInt32 firstSortAxis = 0;
	if ((variance.m_y >= variance.m_x) && (variance.m_y >= variance.m_z))
	{
		firstSortAxis = 1;
	}
	else if ((variance.m_z >= variance.m_x) && (variance.m_z >= variance.m_y))
	{
		firstSortAxis = 2;
	}
	
	dVertexSortData sortContext;
	sortContext.m_points = &m_vertex;
	sortContext.m_vertexSortIndex = firstSortAxis;
	class CompareKey
	{
		public:
		ndInt32 Compare(const dSortKey& elementA, const dSortKey& elementB, void* const context) const
		{
			return ndMeshEffect::dFormat::CompareVertex(&elementA, &elementB, context);
		}
	};
	ndSort<dSortKey, CompareKey>(remapIndex, cluster.m_count, &sortContext);

	const ndFloat64 minDist = dMin(dMin(variance.m_x, variance.m_y), variance.m_z);
	const ndFloat64 tolerance = dMax(dMin(minDist, ndFloat64(tol)), ndFloat64(1.0e-8f));
	const ndFloat64 sweptWindow = ndFloat64(2.0f) * tolerance;

	const ndInt32 base = output.m_vertex.GetCount();
	
	ndInt32 newCount = 0;
	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		const ndInt32 ii = remapIndex[i].m_mask;
		if (ii == -1) 
		{
			const ndInt32 i0 = remapIndex[i].m_ordinal;
			const ndInt32 iii = remapIndex[i].m_vertexIndex;
			const ndBigVector& p = m_vertex[iii];
			const ndFloat64 swept = p[firstSortAxis] + sweptWindow;
			for (ndInt32 j = i + 1; j < cluster.m_count; ++j)
			{
				const ndInt32 jj = remapIndex[j].m_mask;
				if (jj == -1) 
				{
					const ndInt32 j0 = remapIndex[j].m_ordinal;
					const ndInt32 jjj = remapIndex[j].m_vertexIndex;
					const ndBigVector& q = m_vertex[jjj];
					ndFloat64 val = q[firstSortAxis];
					if (val >= swept) 
					{
						break;
					}
	
					bool test = true;
					if (iii != jjj) 
					{
						ndBigVector dp(p - q);
						for (ndInt32 k = 0; k < 3; ++k) 
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}
					if (test && m_layers.GetCount())
					{
						test &= (m_layers[i0] == m_layers[j0]);
					}
					// note, is ok weight duplicate to be ignored.
	
					if (test) 
					{
						remapIndex[j].m_mask = newCount + base;
					}
				}
			}
	
			remapIndex[newCount].m_vertexIndex = remapIndex[i].m_vertexIndex;
			remapIndex[i].m_mask = newCount + base;
			newCount++;
		}
	}

	for (ndInt32 i = 0; i < newCount; ++i) 
	{
		dAssert(remapIndex[i].m_attibuteIndex == -1);
		output.m_vertex.PushBack(m_vertex[remapIndex[i].m_vertexIndex]);
	}
	
	if (m_layers.GetCount())
	{
		for (ndInt32 i = 0; i < newCount; ++i) 
		{
			output.m_layers.PushBack(m_layers[remapIndex[i].m_vertexIndex]);
		}
	}
	
	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		ndInt32 i1 = remapIndex[i].m_ordinal;
		ndInt32 index = remapIndex[i].m_mask;
		indexList[i1] = index;
	}
}

void ndMeshEffect::dPointFormat::CompactVertexData(ndInt32* const indexList, ndFloat32 tol)
{
	dPointFormat tmpFormat(*this);
	Clear();
	const ndInt32 vertexCount = tmpFormat.m_vertex.GetCount();

	ndStack<dSortKey> indirectListBuffer(vertexCount);
	dSortKey* const indirectList = &indirectListBuffer[0];

	dSortCluster cluster;
	cluster.m_start = 0;
	cluster.m_count = vertexCount;
	cluster.m_sum = ndBigVector::m_zero;
	cluster.m_sum2 = ndBigVector::m_zero;
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_vertexIndex = i;
		indirectList[i].m_attibuteIndex = -1;

		const ndBigVector x(tmpFormat.m_vertex[i]);
		cluster.m_sum += x;
		cluster.m_sum2 += x * x;
	}

	if (vertexCount > D_VERTEXLIST_INDEX_LIST_BASH)
	{
		dSortCluster spliteStack[128];
		spliteStack[0] = cluster;

		ndInt32 stack = 1;
		while (stack)
		{
			stack--;
			cluster = spliteStack[stack];
			dSortKey* const remapIndex = &indirectList[cluster.m_start];

			const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / cluster.m_count));
			const ndBigVector variance2(cluster.m_sum2.Scale(ndFloat32(1.0f) / cluster.m_count) - origin * origin);

			ndFloat64 maxVariance2 = dMax(dMax(variance2.m_x, variance2.m_y), variance2.m_z);
			if ((cluster.m_count <= D_VERTEXLIST_INDEX_LIST_BASH) || (stack > (sizeof(spliteStack) / sizeof(spliteStack[0]) - 4)) || (maxVariance2 < ndFloat32(4.0f)))
			{
				tmpFormat.CompressData(*this, indexList, remapIndex, cluster, tol);
			}
			else
			{
				ndInt32 firstSortAxis = 0;
				if ((variance2.m_y >= variance2.m_x) && (variance2.m_y >= variance2.m_z))
				{
					firstSortAxis = 1;
				}
				else if ((variance2.m_z >= variance2.m_x) && (variance2.m_z >= variance2.m_y))
				{
					firstSortAxis = 2;
				}
		
				ndBigPlane plane(ndFloat32(0.0f));
				plane[firstSortAxis] = ndFloat32(1.0f);
				plane.m_w = -origin[firstSortAxis];
		
				ndInt32 i0 = 0;
				ndInt32 i1 = cluster.m_count - 1;
				while (i0 < i1)
				{
					ndInt32 index0 = remapIndex[i0].m_vertexIndex;
					ndFloat64 side = plane.Evalue(tmpFormat.m_vertex[index0]);
					while (side <= ndFloat32(0.0f) && (i0 < i1))
					{
						++i0;
						index0 = remapIndex[i0].m_vertexIndex;
						side = plane.Evalue(tmpFormat.m_vertex[index0]);
					};
		
					ndInt32 index1 = remapIndex[i1].m_vertexIndex;
					side = plane.Evalue(tmpFormat.m_vertex[index1]);
					while (side > ndFloat32(0.0f) && (i0 < i1))
					{
						--i1;
						index1 = remapIndex[i1].m_vertexIndex;
						side = plane.Evalue(tmpFormat.m_vertex[index1]);
					}
		
					dAssert(i0 <= i1);
					if (i0 < i1)
					{
						dSwap(remapIndex[i0], remapIndex[i1]);
						++i0;
						--i1;
					}
				}
		
				ndInt32 index0 = remapIndex[i0].m_vertexIndex;
				ndFloat64 side0 = plane.Evalue(tmpFormat.m_vertex[index0]);
				while (side0 <= ndFloat32(0.0f) && (i0 < cluster.m_count))
				{
					++i0;
					index0 = remapIndex[i0].m_vertexIndex;
					side0 = plane.Evalue(tmpFormat.m_vertex[index0]);
				};
		
				#ifdef _DEBUG
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndInt32 index = remapIndex[i].m_vertexIndex;
					ndFloat64 side = plane.Evalue(tmpFormat.m_vertex[index]);
					dAssert(side <= ndFloat32(0.0f));
				}
		
				for (ndInt32 i = i0; i < cluster.m_count; ++i)
				{
					ndInt32 index = remapIndex[i].m_vertexIndex;
					ndFloat64 side = plane.Evalue(tmpFormat.m_vertex[index]);
					dAssert(side > ndFloat32(0.0f));
				}
				#endif
		
				ndBigVector xc(ndBigVector::m_zero);
				ndBigVector x2c(ndBigVector::m_zero);
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndInt32 index = remapIndex[i].m_vertexIndex;
					ndBigVector x(tmpFormat.m_vertex[index]);
					xc += x;
					x2c += x * x;
				}

				dSortCluster cluster_i1(cluster);
				cluster_i1.m_start = cluster.m_start + i0;
				cluster_i1.m_count = cluster.m_count - i0;
				cluster_i1.m_sum -= xc;
				cluster_i1.m_sum2 -= x2c;
				spliteStack[stack] = cluster_i1;
				stack++;
		
				dSortCluster cluster_i0(cluster);
				cluster_i0.m_start = cluster.m_start;
				cluster_i0.m_count = i0;
				cluster_i0.m_sum = xc;
				cluster_i0.m_sum2 = x2c;
				spliteStack[stack] = cluster_i0;
				stack++;
			}
		}
	}
	else
	{
		tmpFormat.CompressData(*this, indexList, indirectList, cluster, tol);
	}
}

void ndMeshEffect::dAttibutFormat::CompressData(
	dAttibutFormat& output,	const dPointFormat& points, ndInt32* const indexList, 
	dSortKey* const remapIndex, const dSortCluster& cluster, ndFloat32 tol)
{
	const ndBigVector origin (cluster.m_sum.Scale(ndFloat32(1.0f) / cluster.m_count));
	const ndBigVector x2c (cluster.m_sum2.Scale(ndFloat32(1.0f) / cluster.m_count) - origin * origin);
	const ndBigVector variance((x2c.GetMax(ndVector::m_zero)).Sqrt());

	ndInt32 firstSortAxis = 0;
	if ((variance.m_y >= variance.m_x) && (variance.m_y >= variance.m_z))
	{
		firstSortAxis = 1;
	}
	else if ((variance.m_z >= variance.m_x) && (variance.m_z >= variance.m_y))
	{
		firstSortAxis = 2;
	}

	dVertexSortData sortContext;
	sortContext.m_points = &points.m_vertex;
	sortContext.m_vertexSortIndex = firstSortAxis;

	class CompareKey
	{
		public:
		ndInt32 Compare(const dSortKey& elementA, const dSortKey& elementB, void* const context) const
		{
			return ndMeshEffect::dFormat::CompareVertex(&elementA, &elementB, context);
		}
	};
	ndSort<dSortKey, CompareKey>(remapIndex, cluster.m_count, &sortContext);

	const ndFloat64 minDist = dMin(dMin(variance.m_x, variance.m_y), variance.m_z);
	const ndFloat64 tolerance = dMax(dMin(minDist, ndFloat64(tol)), ndFloat64(1.0e-8f));
	const ndFloat64 sweptWindow = ndFloat64(2.0f) * tolerance;

	const ndInt32 base = output.m_pointChannel.GetCount();

	ndInt32 newCount = 0;
	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		const ndInt32 ii = remapIndex[i].m_mask;
		if (ii == -1)
		{
			const ndInt32 i0 = remapIndex[i].m_ordinal;
			const ndInt32 iii = remapIndex[i].m_vertexIndex;
			const ndFloat64 swept = points.m_vertex[iii][firstSortAxis] + sweptWindow;
			for (ndInt32 j = i + 1; j < cluster.m_count; ++j)
			{
				const ndInt32 jj = remapIndex[j].m_mask;
				if (jj == -1)
				{
					const ndInt32 j0 = remapIndex[j].m_ordinal;
					const ndInt32 jjj = remapIndex[j].m_vertexIndex;;
					ndFloat64 val = points.m_vertex[jjj][firstSortAxis];
					if (val >= swept)
					{
						break;
					}

					bool test = true;
					if (iii != jjj)
					{
						ndBigVector dp(points.m_vertex[iii] - points.m_vertex[jjj]);
						for (ndInt32 k = 0; k < 3; ++k)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}
					if (test && points.m_layers.m_isValid)
					{
						test &= (points.m_layers[iii] == points.m_layers[jjj]);
					}

					if (test && m_normalChannel.m_isValid)
					{
						ndVector n0(m_normalChannel[i0].m_x, m_normalChannel[i0].m_y, m_normalChannel[i0].m_z, ndFloat32(0.0f));
						ndVector n1(m_normalChannel[j0].m_x, m_normalChannel[j0].m_y, m_normalChannel[j0].m_z, ndFloat32(0.0f));
						ndVector dp(n1 - n0);
						for (ndInt32 k = 0; k < 3; ++k)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && m_binormalChannel.m_isValid)
					{
						ndVector n0(m_binormalChannel[i0].m_x, m_binormalChannel[i0].m_y, m_binormalChannel[i0].m_z, ndFloat32(0.0f));
						ndVector n1(m_binormalChannel[j0].m_x, m_binormalChannel[j0].m_y, m_binormalChannel[j0].m_z, ndFloat32(0.0f));
						ndVector dp(n1 - n0);
						for (ndInt32 k = 0; k < 3; ++k)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && m_uv0Channel.m_isValid)
					{
						ndVector n0(m_uv0Channel[i0].m_u, m_uv0Channel[i0].m_v, ndFloat32(0.0f), ndFloat32(0.0f));
						ndVector n1(m_uv0Channel[j0].m_u, m_uv0Channel[j0].m_v, ndFloat32(0.0f), ndFloat32(0.0f));
						ndVector dp(n1 - n0);
						for (ndInt32 k = 0; k < 2; ++k)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && m_uv1Channel.m_isValid)
					{
						ndVector n0(m_uv1Channel[i0].m_u, m_uv1Channel[i0].m_v, ndFloat32(0.0f), ndFloat32(0.0f));
						ndVector n1(m_uv1Channel[j0].m_u, m_uv1Channel[j0].m_v, ndFloat32(0.0f), ndFloat32(0.0f));
						ndVector dp(n1 - n0);
						for (ndInt32 k = 0; k < 2; ++k)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && m_colorChannel.m_isValid)
					{
						ndVector dp(m_colorChannel[i0] - m_colorChannel[j0]);
						for (ndInt32 k = 0; k < 3; ++k)
						{
							test &= (fabs(dp[k]) <= tolerance);
						}
					}

					if (test && m_materialChannel.m_isValid)
					{
						test &= (m_materialChannel[i0] == m_materialChannel[j0]);
					}

					if (test)
					{
						remapIndex[j].m_mask = newCount + base;
					}
				}
			}

			remapIndex[newCount].m_attibuteIndex = remapIndex[i].m_attibuteIndex;
			remapIndex[newCount].m_vertexIndex = remapIndex[i].m_vertexIndex;
			remapIndex[i].m_mask = newCount + base;
			newCount++;
		}
	}

	for (ndInt32 i = 0; i < newCount; ++i)
	{
		output.m_pointChannel.PushBack(remapIndex[i].m_vertexIndex);
	}

	if (m_normalChannel.m_isValid)
	{
		for (ndInt32 i = 0; i < newCount; ++i)
		{
			output.m_normalChannel.PushBack(m_normalChannel[remapIndex[i].m_attibuteIndex]);
		}
	}

	if (m_binormalChannel.m_isValid)
	{
		for (ndInt32 i = 0; i < newCount; ++i)
		{
			output.m_binormalChannel.PushBack(m_binormalChannel[remapIndex[i].m_attibuteIndex]);
		}
	}

	if (m_uv0Channel.m_isValid)
	{
		for (ndInt32 i = 0; i < newCount; ++i)
		{
			output.m_uv0Channel.PushBack(m_uv0Channel[remapIndex[i].m_attibuteIndex]);
		}
	}

	if (m_uv1Channel.m_isValid)
	{
		for (ndInt32 i = 0; i < newCount; ++i)
		{
			output.m_uv1Channel.PushBack(m_uv1Channel[remapIndex[i].m_attibuteIndex]);
		}
	}

	if (m_colorChannel.m_isValid)
	{
		for (ndInt32 i = 0; i < newCount; ++i)
		{
			output.m_colorChannel.PushBack(m_colorChannel[remapIndex[i].m_attibuteIndex]);
		}
	}

	if (m_materialChannel.m_isValid)
	{
		for (ndInt32 i = 0; i < newCount; ++i)
		{
			output.m_materialChannel.PushBack(m_materialChannel[remapIndex[i].m_attibuteIndex]);
		}
	}

	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		ndInt32 i1 = remapIndex[i].m_ordinal;
		ndInt32 index = remapIndex[i].m_mask;
		indexList[i1] = index;
	}
}

void ndMeshEffect::dAttibutFormat::CompactVertexData(const dPointFormat& points, ndInt32* const indexList, ndFloat32 tol)
{
	dAttibutFormat tmpFormat(*this);
	Clear();

	dSortCluster cluster;
	cluster.m_start = 0;
	cluster.m_sum = ndBigVector::m_zero;
	cluster.m_sum2 = ndBigVector::m_zero;
	cluster.m_count = tmpFormat.m_pointChannel.GetCount();
	ndStack<dSortKey> indirectListBuffer(cluster.m_count);
	dSortKey* const indirectList = &indirectListBuffer[0];

	for (ndInt32 i = 0; i < cluster.m_count; ++i)
	{
		ndInt32 index = tmpFormat.m_pointChannel[i];
		indirectList[i].m_mask = -1;
		indirectList[i].m_ordinal = i;
		indirectList[i].m_attibuteIndex = i;
		indirectList[i].m_vertexIndex = index;

		const ndBigVector x(points.m_vertex[index]);
		cluster.m_sum += x;
		cluster.m_sum2 += x * x;
	}

	if (cluster.m_count > D_VERTEXLIST_INDEX_LIST_BASH)
	{
		dSortCluster spliteStack[128];
		spliteStack[0] = cluster;

		ndInt32 stack = 1;
		while (stack)
		{
			stack--;

			cluster = spliteStack[stack];
			dSortKey* const remapIndex = &indirectList[cluster.m_start];

			const ndBigVector origin(cluster.m_sum.Scale(ndFloat32(1.0f) / cluster.m_count));
			const ndBigVector variance2(cluster.m_sum2.Scale(ndFloat32(1.0f) / cluster.m_count) - origin * origin);
			ndFloat64 maxVariance2 = dMax(dMax(variance2.m_x, variance2.m_y), variance2.m_z);

			if ((cluster.m_count <= D_VERTEXLIST_INDEX_LIST_BASH) || (stack > (sizeof (spliteStack) / sizeof (spliteStack[0]) - 4)) || (maxVariance2 < ndFloat32(4.0f)))
			{
				tmpFormat.CompressData(*this, points, indexList, remapIndex, cluster, tol);
			}
			else
			{
				ndInt32 firstSortAxis = 0;
				if ((variance2.m_y >= variance2.m_x) && (variance2.m_y >= variance2.m_z))
				{
					firstSortAxis = 1;
				}
				else if ((variance2.m_z >= variance2.m_x) && (variance2.m_z >= variance2.m_y))
				{
					firstSortAxis = 2;
				}

				ndBigPlane plane(ndFloat32(0.0f));
				plane[firstSortAxis] = ndFloat32(1.0f);
				plane.m_w = -origin[firstSortAxis];

				ndInt32 i0 = 0;
				ndInt32 i1 = cluster.m_count - 1;
				while (i0 < i1)
				{
					ndInt32 index0 = remapIndex[i0].m_vertexIndex;
					ndFloat64 side = plane.Evalue(points.m_vertex[index0]);
					while (side <= ndFloat32(0.0f) && (i0 < i1))
					{
						++i0;
						index0 = remapIndex[i0].m_vertexIndex;
						side = plane.Evalue(points.m_vertex[index0]);
					};

					ndInt32 index1 = remapIndex[i1].m_vertexIndex;
					side = plane.Evalue(points.m_vertex[index1]);
					while (side > ndFloat32(0.0f) && (i0 < i1))
					{
						--i1;
						index1 = remapIndex[i1].m_vertexIndex;
						side = plane.Evalue(points.m_vertex[index1]);
					}

					dAssert(i0 <= i1);
					if (i0 < i1)
					{
						dSwap(remapIndex[i0], remapIndex[i1]);
						++i0;
						--i1;
					}
				}

				ndInt32 index0 = remapIndex[i0].m_vertexIndex;
				ndFloat64 side0 = plane.Evalue(points.m_vertex[index0]);
				while (side0 <= ndFloat32(0.0f) && (i0 < cluster.m_count))
				{
					++i0;
					index0 = remapIndex[i0].m_vertexIndex;
					side0 = plane.Evalue(points.m_vertex[index0]);
				};

				#ifdef _DEBUG
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndInt32 index = remapIndex[i].m_vertexIndex;
					ndFloat64 side = plane.Evalue(points.m_vertex[index]);
					dAssert(side <= ndFloat32(0.0f));
				}

				for (ndInt32 i = i0; i < cluster.m_count; ++i)
				{
					ndInt32 index = remapIndex[i].m_vertexIndex;
					ndFloat64 side = plane.Evalue(points.m_vertex[index]);
					dAssert(side > ndFloat32(0.0f));
				}
				#endif

				ndBigVector xc(ndBigVector::m_zero);
				ndBigVector x2c(ndBigVector::m_zero);
				ndBigVector maxP(ndFloat64(-1.0e20f));
				for (ndInt32 i = 0; i < i0; ++i)
				{
					ndInt32 index = remapIndex[i].m_vertexIndex;
					const ndBigVector x(points.m_vertex[index]);
					xc += x;
					x2c += x * x;
					maxP = maxP.GetMax(x);
				}

				dSortCluster cluster_i1(cluster);
				cluster_i1.m_start = cluster.m_start + i0;
				cluster_i1.m_count = cluster.m_count - i0;
				cluster_i1.m_sum -= xc;
				cluster_i1.m_sum2 -= x2c;
				spliteStack[stack] = cluster_i1;
				stack++;

				dSortCluster cluster_i0(cluster);
				cluster_i0.m_start = cluster.m_start;
				cluster_i0.m_count = i0;
				cluster_i0.m_sum = xc;
				cluster_i0.m_sum2 = x2c;
				spliteStack[stack] = cluster_i0;
				stack++;
			}
		}
	}
	else
	{
		tmpFormat.CompressData(*this, points, indexList, indirectList, cluster, tol);
	}
}

ndMeshEffect::ndMeshEffect()
	:ndPolyhedra()
	,m_name()
	,m_points()
	,m_attrib()
	,m_clusters()
	,m_materials()
	,m_vertexBaseCount(-1)
	,m_constructionIndex(0)
{
	Init();
}

ndMeshEffect::ndMeshEffect(ndPolyhedra& mesh, const ndMeshEffect& source)
	:ndPolyhedra(mesh)
	,m_points(source.m_points)
	,m_attrib(source.m_attrib)
	,m_clusters()
	,m_materials(source.m_materials)
	,m_vertexBaseCount(-1)
	,m_constructionIndex(0)
{
	if (source.m_clusters.GetCount())
	{
		// remember to copy the clusters
		dAssert(0);
	}
	Init();
}

ndMeshEffect::ndMeshEffect(const ndMeshEffect& source)
	:ndPolyhedra(source)
	,m_points(source.m_points)
	,m_attrib(source.m_attrib)
	,m_clusters()
	,m_materials(source.m_materials)
	,m_vertexBaseCount(-1)
	,m_constructionIndex(0)
{
	if (source.m_clusters.GetCount())
	{
		// remember to copy the clusters
		dAssert(0);
	}
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
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
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

void ndMeshEffect::AddPoint(ndFloat64 x, ndFloat64 y, ndFloat64 z)
{
	m_attrib.m_pointChannel.PushBack(m_points.m_vertex.GetCount());
	m_points.m_vertex.PushBack(ndBigVector(QuantizeCordinade(x), QuantizeCordinade(y), QuantizeCordinade(z), ndFloat64(0.0f)));
}

void ndMeshEffect::AddLayer(ndInt32 layer)
{
	m_points.m_layers.PushBack(layer);
}

void ndMeshEffect::AddVertexColor(ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w)
{
	m_attrib.m_colorChannel.PushBack(ndVector(x, y, z, w));
}

void ndMeshEffect::AddNormal(ndFloat32 x, ndFloat32 y, ndFloat32 z)
{
	ndTriplex n;
	n.m_x = x;
	n.m_y = y;
	n.m_z = z;
	m_attrib.m_normalChannel.PushBack(n);
}

void ndMeshEffect::AddBinormal(ndFloat32 x, ndFloat32 y, ndFloat32 z)
{
	ndTriplex n;
	n.m_x = x;
	n.m_y = y;
	n.m_z = z;
	m_attrib.m_binormalChannel.PushBack(n);
}

void ndMeshEffect::AddUV0(ndFloat32 u, ndFloat32 v)
{
	dAttibutFormat::dgUV uv;
	uv.m_u = u;
	uv.m_v = v;
	m_attrib.m_uv0Channel.PushBack(uv);
}

void ndMeshEffect::AddUV1(ndFloat32 u, ndFloat32 v)
{
	dAttibutFormat::dgUV uv;
	uv.m_u = u;
	uv.m_v = v;
	m_attrib.m_uv1Channel.PushBack(uv);
}

void ndMeshEffect::AddMaterial(ndInt32 materialIndex)
{
	m_attrib.m_materialChannel.PushBack(materialIndex);
}

void ndMeshEffect::EndBuildFace()
{
	ndInt32 count = m_points.m_vertex.GetCount() - m_constructionIndex;
	if (count > 3) 
	{
		ndInt32 indexList[256];

		dAssert(count < ndInt32(sizeof(indexList) / sizeof(indexList[0])));
		ndPolyhedra polygon;
		dPointFormat points;
		dAttibutFormat attibutes;

		for (ndInt32 i = 0; i < count; ++i) 
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
		
			if (m_attrib.m_colorChannel.GetCount())
			{
				attibutes.m_colorChannel.PushBack(m_attrib.m_colorChannel[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_uv0Channel.GetCount())
			{
				attibutes.m_uv0Channel.PushBack(m_attrib.m_uv0Channel[m_constructionIndex + i]);
			}
		
			if (m_attrib.m_uv1Channel.GetCount())
			{
				attibutes.m_uv1Channel.PushBack(m_attrib.m_uv1Channel[m_constructionIndex + i]);
			}
		}
		
		polygon.BeginFace();
		polygon.AddFace(count, indexList, nullptr);
		polygon.EndFace();
		polygon.Triangulate(&points.m_vertex[0].m_x, sizeof(ndBigVector), nullptr);
		
		ndInt32 mark = polygon.IncLRU();
		ndPolyhedra::Iterator iter(polygon);
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) 
			{
				ndInt32 i0 = edge->m_incidentVertex;
				ndInt32 i1 = edge->m_next->m_incidentVertex;
				ndInt32 i2 = edge->m_next->m_next->m_incidentVertex;
				edge->m_mark = mark;
				edge->m_next->m_mark = mark;
				edge->m_next->m_next->m_mark = mark;
		
				const ndBigVector& p0 = points.m_vertex[i0];
				const ndBigVector& p1 = points.m_vertex[i1];
				const ndBigVector& p2 = points.m_vertex[i2];
		
				ndBigVector e1(p1 - p0);
				ndBigVector e2(p2 - p0);
				ndBigVector n(e1.CrossProduct(e2));
		
				dAssert(e1.m_w == ndFloat32(0.0f));
				dAssert(e2.m_w == ndFloat32(0.0f));
				dAssert(n.m_w == ndFloat32(0.0f));
		
				ndFloat64 mag3 = n.DotProduct(n).GetScalar();
				if (mag3 >= ndFloat64(DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)) 
				{
					ndInt32 index[] = { i0, i1, i2 };
					for (ndInt32 i = 0; i < 3; ++i) 
					{
						ndInt32 j = index[i];
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
		const ndBigVector& p0 = m_points.m_vertex[m_constructionIndex + 0];
		const ndBigVector& p1 = m_points.m_vertex[m_constructionIndex + 1];
		const ndBigVector& p2 = m_points.m_vertex[m_constructionIndex + 2];

		ndBigVector e1(p1 - p0);
		ndBigVector e2(p2 - p0);
		ndBigVector n(e1.CrossProduct(e2));
		dAssert(e1.m_w == ndFloat32(0.0f));
		dAssert(e2.m_w == ndFloat32(0.0f));
		dAssert(n.m_w == ndFloat32(0.0f));

		ndFloat64 mag3 = n.DotProduct(n).GetScalar();
		if (mag3 < ndFloat64(DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)) 
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

void ndMeshEffect::EndBuild(bool fixTjoint)
{
	#ifdef _DEBUG
	for (ndInt32 i = 0; i < m_points.m_vertex.GetCount(); i += 3)
	{
		ndBigVector p0(m_points.m_vertex[i + 0]);
		ndBigVector p1(m_points.m_vertex[i + 1]);
		ndBigVector p2(m_points.m_vertex[i + 2]);
		ndBigVector e1(p1 - p0);
		ndBigVector e2(p2 - p0);
		ndBigVector n(e1.CrossProduct(e2));

		dAssert(e1.m_w == ndFloat32(0.0f));
		dAssert(e2.m_w == ndFloat32(0.0f));
		dAssert(n.m_w == ndFloat32(0.0f));
		ndFloat64 mag2 = n.DotProduct(n).GetScalar();
		dAssert(mag2 > ndFloat32(0.0f));
	}
	#endif

	ndInt32 triangCount = m_points.m_vertex.GetCount() / 3;
	const ndInt32* const indexList = &m_attrib.m_pointChannel[0];
	for (ndInt32 i = 0; i < triangCount; ++i) 
	{
		ndInt32 index[3];
		ndInt64 userdata[3];
	
		index[0] = indexList[i * 3 + 0];
		index[1] = indexList[i * 3 + 1];
		index[2] = indexList[i * 3 + 2];
	
		ndBigVector e1(m_points.m_vertex[index[1]] - m_points.m_vertex[index[0]]);
		ndBigVector e2(m_points.m_vertex[index[2]] - m_points.m_vertex[index[0]]);
		ndBigVector n(e1.CrossProduct(e2));
	
		dAssert(e1.m_w == ndFloat32(0.0f));
		dAssert(e2.m_w == ndFloat32(0.0f));
		dAssert(n.m_w == ndFloat32(0.0f));
		ndFloat64 mag2 = n.DotProduct(n).GetScalar();
		if (mag2 > ndFloat64(1.0e-12f)) 
		{
			userdata[0] = i * 3 + 0;
			userdata[1] = i * 3 + 1;
			userdata[2] = i * 3 + 2;
			ndEdge* const edge = AddFace(3, index, userdata);
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
				ndEdge* test = AddFace (3, index, userdata);
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
	PackPoints();
	
	if (fixTjoint) 
	{
		RepairTJoints();
	}
	
	#ifdef _DEBUG
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) 
		{
			ndBigVector p0(m_points.m_vertex[face->m_incidentVertex]);
			ndBigVector p1(m_points.m_vertex[face->m_next->m_incidentVertex]);
			ndBigVector p2(m_points.m_vertex[face->m_next->m_next->m_incidentVertex]);
			ndBigVector e1(p1 - p0);
			ndBigVector e2(p2 - p0);
			ndBigVector n(e1.CrossProduct(e2));
	
			dAssert(e1.m_w == ndFloat32(0.0f));
			dAssert(e2.m_w == ndFloat32(0.0f));
			dAssert(n.m_w == ndFloat32(0.0f));
			ndFloat64 mag2 = n.DotProduct(n).GetScalar();
			dAssert(mag2 >= ndFloat32(0.0f));
		}
	}
	#endif
}

void ndMeshEffect::BeginFace()
{
	ndPolyhedra::BeginFace();
}

bool ndMeshEffect::EndFace()
{
	ndPolyhedra::EndFace();
	bool state = false;
	for (bool hasVertexCollision = true; hasVertexCollision;)
	{
		hasVertexCollision = false;
		const ndInt32 currentCount = m_points.m_vertex.GetCount();
		ndStack<ndInt8> verterCollisionBuffer(currentCount);
		ndInt8* const verterCollision = &verterCollisionBuffer[0];
		memset(&verterCollision[0], 0, verterCollisionBuffer.GetSizeInBytes());

		Iterator iter(*this);
		ndInt32 mark = IncLRU();
		ndList<ndNode*> collisionFound;
		for (iter.Begin(); iter; iter++)
		{
			ndEdge* const edge = &iter.GetNode()->GetInfo();
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
				ndEdge* ptr = edge;
				do
				{
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}

		dAssert(!collisionFound.GetFirst() || Sanity());
		for (ndList<ndNode*>::ndNode* node = collisionFound.GetFirst(); node; node = node->GetNext())
		{
			state = true;
			ndEdge* const edge = &node->GetInfo()->GetInfo();

			// this is a vertex collision
			m_points.m_vertex.PushBack(m_points.m_vertex[edge->m_incidentVertex]);
			if (m_points.m_layers.GetCount())
			{
				m_points.m_layers.PushBack(m_points.m_layers[edge->m_incidentVertex]);
			}

			ndEdge* ptr = edge;
			do
			{
				if (ptr->m_incidentFace > 0)
				{
					//m_attrib.m_pointChannel[ndInt32 (ptr->m_userData)] = m_points.m_vertex.m_count - 1;
					ndInt32 index = ndInt32(ptr->m_userData);
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

				ndNode* const edgeNode = GetNodeFromInfo(*ptr);
				ndPairKey edgeKey(ptr->m_incidentVertex, ptr->m_twin->m_incidentVertex);
				ReplaceKey(edgeNode, edgeKey.GetVal());

				ndNode* const twinNode = GetNodeFromInfo(*(ptr->m_twin));
				ndPairKey twinKey(ptr->m_twin->m_incidentVertex, ptr->m_incidentVertex);
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
	ndInt32 vertexCount = 0;
	ndInt32 maxAttribCount = 0;
	for (ndInt32 j = 0; j < format->m_faceCount; ++j)
	{
		ndInt32 count = format->m_faceIndexCount[j];
		for (ndInt32 i = 0; i < count; ++i)
		{
			vertexCount = dMax(vertexCount, format->m_vertex.m_indexList[maxAttribCount + i] + 1);
		}
		maxAttribCount += count;
	}
	m_vertexBaseCount = vertexCount;

	ndInt32 layerIndex = 0;
	ndInt32 vertexStride = ndInt32(format->m_vertex.m_strideInBytes / sizeof(ndFloat64));
	const ndFloat64* const vertex = format->m_vertex.m_data;

	m_points.m_layers.Resize(vertexCount);
	m_points.m_vertex.Resize(vertexCount);
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		ndInt32 index = i * vertexStride;
		m_points.m_layers.PushBack(layerIndex);
		ndBigVector p(vertex[index + 0], vertex[index + 1], vertex[index + 2], ndFloat64(0.0f));
		m_points.m_vertex.PushBack(p);
	}

	bool pendingFaces = true;
	ndInt32 layerBase = 0;
	ndInt32 attributeCount = 0;

	ndInt32 normalStride = ndInt32(format->m_normal.m_strideInBytes / sizeof(ndFloat32));
	ndInt32 binormalStride = ndInt32(format->m_binormal.m_strideInBytes / sizeof(ndFloat32));
	ndInt32 uv0Stride = ndInt32(format->m_uv0.m_strideInBytes / sizeof(ndFloat32));
	ndInt32 uv1Stride = ndInt32(format->m_uv1.m_strideInBytes / sizeof(ndFloat32));
	ndInt32 vertexColorStride = ndInt32(format->m_vertexColor.m_strideInBytes / sizeof(ndFloat32));

	ndStack<ndInt8> faceMark(format->m_faceCount);
	memset(&faceMark[0], 0, faceMark.GetSizeInBytes());
	const ndInt32* const vertexIndex = format->m_vertex.m_indexList;

	while (pendingFaces)
	{
		ndInt32 acc = 0;
		pendingFaces = false;
		ndInt32 vertexBank = layerIndex * vertexCount;
		for (ndInt32 j = 0; j < format->m_faceCount; ++j)
		{
			ndInt32 indexCount = format->m_faceIndexCount[j];

			if (indexCount > 0)
			{
				ndInt32 index[256];
				ndInt64 userdata[256];
				dAssert(indexCount >= 3);
				dAssert(indexCount < ndInt32(sizeof(index) / sizeof(index[0])));

				if (!faceMark[j])
				{
					for (ndInt32 i = 0; i < indexCount; ++i)
					{
						ndInt32 k = attributeCount + i;
						userdata[i] = k;
						index[i] = vertexIndex[acc + i] + vertexBank;
					}

					ndEdge* const edge = AddFace(indexCount, index, userdata);
					if (edge)
					{
						faceMark[j] = 1;
						for (ndInt32 i = 0; i < indexCount; ++i)
						{
							m_attrib.m_pointChannel.PushBack(index[i]);
						}

						if (format->m_faceMaterial)
						{
							ndInt32 materialIndex = format->m_faceMaterial[j];
							for (ndInt32 i = 0; i < indexCount; ++i)
							{
								m_attrib.m_materialChannel.PushBack(materialIndex);
							}
						}

						if (format->m_normal.m_data)
						{
							ndTriplex normal;
							for (ndInt32 i = 0; i < indexCount; ++i)
							{
								ndInt32 k = attributeCount + i;
								ndInt32 m = format->m_normal.m_indexList[k] * normalStride;
								normal.m_x = format->m_normal.m_data[m + 0];
								normal.m_y = format->m_normal.m_data[m + 1];
								normal.m_z = format->m_normal.m_data[m + 2];
								m_attrib.m_normalChannel.PushBack(normal);
							}
						}

						if (format->m_binormal.m_data)
						{
							ndTriplex normal;
							for (ndInt32 i = 0; i < indexCount; ++i)
							{
								ndInt32 k = attributeCount + i;
								ndInt32 m = format->m_binormal.m_indexList[k] * binormalStride;
								normal.m_x = format->m_binormal.m_data[m + 0];
								normal.m_y = format->m_binormal.m_data[m + 1];
								normal.m_z = format->m_binormal.m_data[m + 2];
								m_attrib.m_binormalChannel.PushBack(normal);
							}
						}

						if (format->m_vertexColor.m_data)
						{
							for (ndInt32 i = 0; i < indexCount; ++i)
							{
								ndInt32 k = attributeCount + i;
								ndInt32 m = format->m_vertexColor.m_indexList[k] * vertexColorStride;
								ndVector color(format->m_vertexColor.m_data[m + 0], format->m_vertexColor.m_data[m + 1], format->m_vertexColor.m_data[m + 2], format->m_vertexColor.m_data[m + 3]);
								m_attrib.m_colorChannel.PushBack(color);
							}
						}

						if (format->m_uv0.m_data)
						{
							dAttibutFormat::dgUV uv;
							for (ndInt32 i = 0; i < indexCount; ++i)
							{
								ndInt32 k = attributeCount + i;
								ndInt32 m = format->m_uv0.m_indexList[k] * uv0Stride;
								uv.m_u = format->m_uv0.m_data[m + 0];
								uv.m_v = format->m_uv0.m_data[m + 1];
								m_attrib.m_uv0Channel.PushBack(uv);
							}
						}

						if (format->m_uv1.m_data)
						{
							dAttibutFormat::dgUV uv;
							for (ndInt32 i = 0; i < indexCount; ++i)
							{
								ndInt32 k = attributeCount + i;
								ndInt32 m = format->m_uv1.m_indexList[k] * uv1Stride;
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
						for (ndInt32 i = 0; i < indexCount - 1; ++i)
						{
							for (ndInt32 k = i + 1; k < indexCount; ++k)
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
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				m_points.m_layers.PushBack(layerIndex);
				ndInt32 index = i * vertexStride;
				ndBigVector p(vertex[index + 0], vertex[index + 1], vertex[index + 2], vertex[index + 3]);
				m_points.m_vertex.PushBack(p);
			}
		}
	}

	dAssert(m_points.m_vertex.GetCount() == vertexCount * (layerIndex + 1));
	dAssert(m_attrib.m_pointChannel.GetCount() == attributeCount);

	EndFace();
	PackAttibuteData();
}

void ndMeshEffect::PackPoints()
{
	ndStack<ndInt32>vertexIndexMapBuffer(m_points.m_vertex.GetCount());
	ndInt32* const vertexIndexMap = &vertexIndexMapBuffer[0];
	m_points.CompactVertexData(&vertexIndexMap[0], ndFloat32 (1.0e-6f));

	ndInt32 index[DG_MESH_EFFECT_POINT_SPLITED];
	ndInt64 userData[DG_MESH_EFFECT_POINT_SPLITED];
	ndPolyhedra polygon;
	SwapInfo(polygon);
	dAssert(GetCount() == 0);

	BeginFace();
	const ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(polygon);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) 
		{
			ndEdge* ptr = edge;
			ndInt32 indexCount = 0;
			do 
			{
				ptr->m_mark = mark;
				index[indexCount] = vertexIndexMap[ptr->m_incidentVertex];
				m_attrib.m_pointChannel[ndInt32(ptr->m_userData)] = vertexIndexMap[ptr->m_incidentVertex];
				userData[indexCount] = ptr->m_userData;

				indexCount++;
				ptr = ptr->m_next;
			} while (ptr != edge);
			ndEdge* const face = AddFace(indexCount, index, userData);
			
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
		for (ndInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); ++i) 
		{
			ndInt32 index = m_attrib.m_pointChannel[i];
			m_points.m_vertex.PushBack(points.m_vertex[index]);
			if (points.m_layers.GetCount()) 
			{
				m_points.m_layers.PushBack(points.m_layers[index]);
			}

			m_attrib.m_pointChannel[i] = i;
		}

		ndInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
		ndInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];
		ndPolyhedra polygon;
		SwapInfo(polygon);
		dAssert(GetCount() == 0);
		BeginFace();
		const ndInt32 mark = IncLRU();
		ndPolyhedra::Iterator iter(polygon);
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const face = &(*iter);
			if ((face->m_mark != mark) && (face->m_incidentFace > 0)) 
			{
				ndEdge* ptr = face;
				ndInt32 indexCount = 0;
				do 
				{
					ptr->m_mark = mark;
					index[indexCount] = ndInt32(ptr->m_userData);
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
	for (ndInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); ++i)
	{
		dAssert(m_attrib.m_pointChannel[i] == i);
	}
	#endif
}

void ndMeshEffect::PackAttibuteData()
{
	ndStack<ndInt32>attrIndexBuffer(m_attrib.m_pointChannel.GetCount());
	ndInt32* const attrIndexMap = &attrIndexBuffer[0];
	m_attrib.CompactVertexData(m_points, &attrIndexMap[0], ndFloat32(1.0e-6f));

	Iterator iter(*this);
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if (edge->m_incidentFace > 0)
		{
			edge->m_userData = attrIndexMap[edge->m_userData];
		}
	}

	memset(attrIndexMap, -1, sizeof(ndInt32) * m_attrib.m_pointChannel.GetCount());
	dAttibutFormat tmpFormat(m_attrib);
	m_attrib.Clear();

	ndInt32 remapIndex = 0;
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if (edge->m_incidentFace > 0)
		{
			ndInt32 index = ndInt32(edge->m_userData);
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
	ndInt32 attributeCount = 0;
	const ndInt32 lru = IncLRU();
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
		if ((edge->m_incidentFace > 0) && (edge->m_mark != lru))
		{
			ndEdge* ptr = edge;

			ptr = edge;
			do
			{
				ptr->m_mark = lru;
				m_attrib.m_pointChannel.PushBack(ptr->m_incidentVertex);

				if (attibutes.m_materialChannel.m_isValid)
				{
					m_attrib.m_materialChannel.PushBack(attibutes.m_materialChannel[ndInt32(ptr->m_userData)]);
				}

				if (attibutes.m_normalChannel.m_isValid)
				{
					m_attrib.m_normalChannel.PushBack(attibutes.m_normalChannel[ndInt32(ptr->m_userData)]);
				}

				if (attibutes.m_binormalChannel.m_isValid)
				{
					m_attrib.m_binormalChannel.PushBack(attibutes.m_binormalChannel[ndInt32(ptr->m_userData)]);
				}

				if (attibutes.m_colorChannel.m_isValid)
				{
					m_attrib.m_colorChannel.PushBack(attibutes.m_colorChannel[ndInt32(ptr->m_userData)]);
				}

				if (attibutes.m_uv0Channel.m_isValid)
				{
					m_attrib.m_uv0Channel.PushBack(attibutes.m_uv0Channel[ndInt32(ptr->m_userData)]);
				}

				if (attibutes.m_uv1Channel.m_isValid)
				{
					m_attrib.m_uv1Channel.PushBack(attibutes.m_uv1Channel[ndInt32(ptr->m_userData)]);
				}

				ptr->m_userData = attributeCount;
				attributeCount++;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
	dAssert(m_attrib.m_pointChannel.GetCount() == attributeCount);
}

bool ndMeshEffect::SeparateDuplicateLoops(ndEdge* const face)
{
	for (ndEdge* ptr0 = face; ptr0 != face->m_prev; ptr0 = ptr0->m_next)
	{
		ndInt32 index = ptr0->m_incidentVertex;

		ndEdge* ptr1 = ptr0->m_next;
		do
		{
			if (ptr1->m_incidentVertex == index)
			{
				ndEdge* const ptr00 = ptr0->m_prev;
				ndEdge* const ptr11 = ptr1->m_prev;

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

ndInt32 ndMeshEffect::AddInterpolatedHalfAttribute(ndEdge* const edge, ndInt32 midPoint)
{
	ndBigVector p0(m_points.m_vertex[edge->m_incidentVertex]);
	ndBigVector p2(m_points.m_vertex[edge->m_next->m_incidentVertex]);
	ndBigVector p1(m_points.m_vertex[midPoint]);
	ndBigVector p2p0(p2 - p0);

	dAssert(p2p0.m_w == ndFloat32(0.0f));

	ndFloat64 den = p2p0.DotProduct(p2p0).GetScalar();
	ndFloat64 param = p2p0.DotProduct(p1 - p0).GetScalar() / den;
	ndFloat64 t1 = param;
	ndFloat64 t0 = ndFloat64(1.0f) - t1;
	dAssert(t1 >= ndFloat64(0.0f));
	dAssert(t1 <= ndFloat64(1.0f));

	m_attrib.m_pointChannel.PushBack(midPoint);

	if (m_attrib.m_materialChannel.m_isValid)
	{
		m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[ndInt32(edge->m_userData)]);
	}
	if (m_attrib.m_normalChannel.m_isValid)
	{
		ndTriplex edgeNormal;
		ndTriplex edgeNormal0(m_attrib.m_normalChannel[ndInt32(edge->m_userData)]);
		ndTriplex edgeNormal1(m_attrib.m_normalChannel[ndInt32(edge->m_next->m_userData)]);
		edgeNormal.m_x = edgeNormal0.m_x * ndFloat32(t0) + edgeNormal1.m_x * ndFloat32(t1);
		edgeNormal.m_y = edgeNormal0.m_y * ndFloat32(t0) + edgeNormal1.m_y * ndFloat32(t1);
		edgeNormal.m_z = edgeNormal0.m_z * ndFloat32(t0) + edgeNormal1.m_z * ndFloat32(t1);
		m_attrib.m_normalChannel.PushBack(edgeNormal);
	}
	if (m_attrib.m_binormalChannel.m_isValid)
	{
		dAssert(0);
	}

	if (m_attrib.m_uv0Channel.m_isValid)
	{
		dAttibutFormat::dgUV edgeUV;
		dAttibutFormat::dgUV edgeUV0(m_attrib.m_uv0Channel[ndInt32(edge->m_userData)]);
		dAttibutFormat::dgUV edgeUV1(m_attrib.m_uv0Channel[ndInt32(edge->m_next->m_userData)]);
		edgeUV.m_u = edgeUV0.m_u * ndFloat32(t0) + edgeUV1.m_u * ndFloat32(t1);
		edgeUV.m_v = edgeUV0.m_v * ndFloat32(t0) + edgeUV1.m_v * ndFloat32(t1);
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
		ndFloat64 tol = ndFloat64(1.0e-5f);
		ndFloat64 tol2 = tol * tol;
		dirty = false;
		ndPolyhedra::Iterator iter(*this);
		for (iter.Begin(); iter; )
		{
			ndEdge* const edge = &(*iter);
			iter++;
			const ndBigVector& p0 = m_points.m_vertex[edge->m_incidentVertex];
			const ndBigVector& p1 = m_points.m_vertex[edge->m_twin->m_incidentVertex];
			ndBigVector dist(p1 - p0);
			dAssert(dist.m_w == ndFloat32(0.0f));
			ndFloat64 mag2 = dist.DotProduct(dist).GetScalar();
			if (mag2 < tol2)
			{
				bool move = true;
				while (move)
				{
					move = false;
					ndEdge* ptr = edge->m_twin;
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

				ndEdge* const collapsedEdge = CollapseEdge(edge);
				if (collapsedEdge)
				{
					//dAssert (0);
					dTrace(("remember to finish this!!: %s %s line:%d\n", __FILE__, __FUNCTION__, __LINE__));
					dirty = true;
					ndBigVector q(m_points.m_vertex[collapsedEdge->m_incidentVertex]);
					ndEdge* ptr = collapsedEdge;
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
	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark) != mark && (edge->m_incidentFace < 0))
		{
			while (SeparateDuplicateLoops(edge));
			ndEdge* ptr = edge;
			do
			{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	dAssert(Sanity());
	DeleteDegenerateFaces(&m_points.m_vertex[0].m_x, sizeof(ndBigVector), ndFloat64(1.0e-7f));
	dAssert(Sanity());

	// delete straight line edges
	dirty = true;
	//	dirty = false;
	while (dirty)
	{
		ndFloat64 tol = ndFloat64(1.0 - 1.0e-8);
		ndFloat64 tol2 = tol * tol;

		dirty = false;
		dAssert(Sanity());

		ndPolyhedra::Iterator iter1(*this);
		for (iter1.Begin(); iter1; )
		{
			ndEdge* const edge = &(*iter1);
			iter1++;

			const ndBigVector& p0 = m_points.m_vertex[edge->m_incidentVertex];
			const ndBigVector& p1 = m_points.m_vertex[edge->m_next->m_incidentVertex];
			const ndBigVector& p2 = m_points.m_vertex[edge->m_next->m_next->m_incidentVertex];

			ndBigVector A(p1 - p0);
			ndBigVector B(p2 - p1);
			dAssert(A.m_w == ndFloat32(0.0f));
			dAssert(B.m_w == ndFloat32(0.0f));
			ndFloat64 ab = A.DotProduct(B).GetScalar();
			if (ab >= 0.0f)
			{
				ndFloat64 aa = A.DotProduct(A).GetScalar();
				ndFloat64 bb = B.DotProduct(B).GetScalar();

				ndFloat64 magab2 = ab * ab;
				ndFloat64 magaabb = aa * bb * tol2;
				if (magab2 >= magaabb)
				{
					if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0))
					{
						if (edge->m_twin->m_prev == edge->m_next->m_twin)
						{
							ndEdge* const newEdge = AddHalfEdge(edge->m_incidentVertex, edge->m_next->m_next->m_incidentVertex);
							if (newEdge)
							{
								dirty = true;
								ndEdge* const newTwin = AddHalfEdge(edge->m_next->m_next->m_incidentVertex, edge->m_incidentVertex);
								dAssert(newEdge);
								dAssert(newTwin);

								newEdge->m_twin = newTwin;
								newTwin->m_twin = newEdge;

								newEdge->m_userData = edge->m_userData;
								newTwin->m_userData = edge->m_twin->m_prev->m_userData;

								newEdge->m_incidentFace = edge->m_incidentFace;
								newTwin->m_incidentFace = edge->m_twin->m_incidentFace;

								ndEdge* const nextEdge = edge->m_next;

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
								ndEdge* const openEdge = edge;
								ndEdge* const nextEdge = openEdge->m_next;
								ndEdge* const deletedEdge = openEdge->m_prev;
								while ((&(*iter) == deletedEdge) || (&(*iter) == deletedEdge->m_twin))
								{
									iter ++;
								}

								openEdge->m_userData = deletedEdge->m_twin->m_userData;

								ndBigVector p2p0 (p2 - p0);
								ndFloat64 den = p2p0.DotProduct3(p2p0);
								ndFloat64 param1 = p2p0.DotProduct3(p1 - p0) / den;
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
						ndEdge* const openEdge = edge;
						//dAssert (openEdge->m_incidentFace <= 0);
						dTrace(("remember to finish this!!: %s %s line:%d\n", __FILE__, __FUNCTION__, __LINE__));
						ndEdge* const nextEdge = openEdge->m_next;
						ndEdge* const deletedEdge = openEdge->m_prev;
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

							ndInt32 attibuteIndex = AddInterpolatedHalfAttribute(deletedEdge->m_twin, nextEdge->m_incidentVertex);
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
						ndEdge* const openEdge = (edge->m_incidentFace <= 0) ? edge : edge->m_twin;
						dAssert(openEdge->m_incidentFace <= 0);

						const ndBigVector& p3 = m_points.m_vertex[openEdge->m_next->m_next->m_next->m_incidentVertex];

						ndBigVector A0(p3 - p2);
						ndBigVector B0(p2 - p1);
						dAssert(A0.m_w == ndFloat32(0.0f));
						dAssert(B0.m_w == ndFloat32(0.0f));

						ndFloat64 ab0 = A0.DotProduct(B0).GetScalar();
						if (ab0 >= ndFloat32(0.0f))
						{
							ndFloat64 aa0 = A0.DotProduct(A0).GetScalar();
							ndFloat64 bb0 = B0.DotProduct(B0).GetScalar();

							ndFloat64 ab0ab0 = ab0 * ab0;
							ndFloat64 aa0bb0 = aa0 * bb0 * tol2;
							if (ab0ab0 >= aa0bb0)
							{
								if (openEdge->m_next->m_next->m_next->m_next != openEdge)
								{
									const ndBigVector& p4 = m_points.m_vertex[openEdge->m_prev->m_incidentVertex];
									ndBigVector A1(p1 - p0);
									ndBigVector B1(p1 - p4);
									dAssert(A1.m_w == ndFloat32(0.0f));
									dAssert(B1.m_w == ndFloat32(0.0f));
									ndFloat64 ab1 = A1.DotProduct(B1).GetScalar();
									if (ab1 < ndFloat32(0.0f))
									{
										ndFloat64 ab1ab1 = ab1 * ab1;
										ndFloat64 aa1bb1 = aa0 * bb0 * tol2;
										if (ab1ab1 >= aa1bb1)
										{
											ndEdge* const newFace = ConnectVertex(openEdge->m_prev, openEdge->m_next);
											dirty |= newFace ? true : false;
										}
									}
									//dAssert (Sanity ());
								}
								else if (openEdge->m_prev->m_twin->m_incidentFace > 0)
								{
									dirty = true;

									ndEdge* const deletedEdge = openEdge->m_prev;
									while ((&(*iter1) == deletedEdge) || (&(*iter1) == deletedEdge->m_twin))
									{
										iter1++;
									}

									openEdge->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
									openEdge->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;
									openEdge->m_next->m_next->m_incidentFace = deletedEdge->m_twin->m_incidentFace;

									ndInt32 attibuteIndex0 = AddInterpolatedHalfAttribute(deletedEdge->m_twin, openEdge->m_next->m_incidentVertex);
									ndInt32 attibuteIndex1 = AddInterpolatedHalfAttribute(deletedEdge->m_twin, openEdge->m_next->m_next->m_incidentVertex);

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

	DeleteDegenerateFaces(&m_points.m_vertex[0].m_x, sizeof(ndBigVector), ndFloat64(1.0e-7f));
	/*
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_incidentFace > 0)
		{
			ndBigVector p0(m_points[edge->m_incidentVertex]);
			m_attrib[edge->m_userData].m_vertex.m_x = p0.m_x;
			m_attrib[edge->m_userData].m_vertex.m_y = p0.m_y;
			m_attrib[edge->m_userData].m_vertex.m_z = p0.m_z;
		}
	}
	*/

	dAssert(Sanity());
}

void ndMeshEffect::GetVertexIndexChannel(ndInt32* const bufferOut) const
{
	for (ndInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); ++i)
	{
		const ndInt32 index = m_attrib.m_pointChannel[i];
		bufferOut[i] = index;
	}
}

void ndMeshEffect::GetVertexChannel64(ndInt32 strideInByte, ndFloat64* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat64);
	for (ndInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		const ndInt32 index = m_attrib.m_pointChannel[i];
		bufferOut[j + 0] = m_points.m_vertex[index].m_x;
		bufferOut[j + 1] = m_points.m_vertex[index].m_y;
		bufferOut[j + 2] = m_points.m_vertex[index].m_z;
	}
}

void ndMeshEffect::GetVertexChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat32);
	for (ndInt32 i = 0; i < m_attrib.m_pointChannel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		const ndInt32 index = m_attrib.m_pointChannel[i];
		const ndBigVector& p = m_points.m_vertex[index];
		bufferOut[j + 0] = ndFloat32(p.m_x);
		bufferOut[j + 1] = ndFloat32(p.m_y);
		bufferOut[j + 2] = ndFloat32(p.m_z);
	}
}

void ndMeshEffect::GetNormalChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat32);
	for (ndInt32 i = 0; i < m_attrib.m_normalChannel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		bufferOut[j + 0] = ndFloat32(m_attrib.m_normalChannel[i].m_x);
		bufferOut[j + 1] = ndFloat32(m_attrib.m_normalChannel[i].m_y);
		bufferOut[j + 2] = ndFloat32(m_attrib.m_normalChannel[i].m_z);
	}
}

void ndMeshEffect::GetBinormalChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat32);
	for (ndInt32 i = 0; i < m_attrib.m_binormalChannel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		bufferOut[j + 0] = ndFloat32(m_attrib.m_binormalChannel[i].m_x);
		bufferOut[j + 1] = ndFloat32(m_attrib.m_binormalChannel[i].m_y);
		bufferOut[j + 2] = ndFloat32(m_attrib.m_binormalChannel[i].m_z);
	}
}

void ndMeshEffect::GetUV0Channel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat32);
	for (ndInt32 i = 0; i < m_attrib.m_uv0Channel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		bufferOut[j + 0] = ndFloat32(m_attrib.m_uv0Channel[i].m_u);
		bufferOut[j + 1] = ndFloat32(m_attrib.m_uv0Channel[i].m_v);
	}
}

void ndMeshEffect::GetUV1Channel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat32);
	for (ndInt32 i = 0; i < m_attrib.m_uv1Channel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		bufferOut[j + 0] = ndFloat32(m_attrib.m_uv1Channel[i].m_u);
		bufferOut[j + 1] = ndFloat32(m_attrib.m_uv1Channel[i].m_v);
	}
}

void ndMeshEffect::GetVertexColorChannel(ndInt32 strideInByte, ndFloat32* const bufferOut) const
{
	ndInt32 stride = strideInByte / sizeof(ndFloat32);
	for (ndInt32 i = 0; i < m_attrib.m_colorChannel.GetCount(); ++i)
	{
		const ndInt32 j = i * stride;
		bufferOut[j + 0] = ndFloat32(m_attrib.m_colorChannel[i].m_x);
		bufferOut[j + 1] = ndFloat32(m_attrib.m_colorChannel[i].m_y);
		bufferOut[j + 2] = ndFloat32(m_attrib.m_colorChannel[i].m_z);
		bufferOut[j + 3] = ndFloat32(m_attrib.m_colorChannel[i].m_w);
	}
}

ndIndexArray* ndMeshEffect::MaterialGeometryBegin()
{
	ndInt32 materials[256];
	ndInt32 streamIndexMap[256];

	ndInt32 count = 0;
	ndInt32 materialCount = 0;

	ndIndexArray* const array = (ndIndexArray*)ndMemory::Malloc(ndInt32(4 * sizeof(ndInt32) * GetCount() + sizeof(ndIndexArray) + 2048));
	array->m_indexList = (ndInt32*)&array[1];

	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	memset(streamIndexMap, 0, sizeof(streamIndexMap));
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
		{
			ndEdge* ptr = edge;
			ptr->m_mark = mark;
			ndInt32 index0 = ndInt32(ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			ndInt32 index1 = ndInt32(ptr->m_userData);

			ptr = ptr->m_next;
			do
			{
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = ndInt32(ptr->m_userData);
				array->m_indexList[count * 4 + 3] = m_attrib.m_materialChannel.m_isValid ? ndInt32(m_attrib.m_materialChannel[ndInt32(edge->m_userData)]) : 0;
				index1 = ndInt32(ptr->m_userData);

				ndInt32 hashValue = array->m_indexList[count * 4 + 3] & 0xff;
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
	for (ndInt32 i = 0; i < 256; ++i)
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

void ndMeshEffect::MaterialGeometryEnd(ndIndexArray* const handle)
{
	ndMemory::Free(handle);
}

ndInt32 ndMeshEffect::GetFirstMaterial(ndIndexArray* const handle) const
{
	return GetNextMaterial(handle, -1);
}

ndInt32 ndMeshEffect::GetNextMaterial(ndIndexArray* const handle, ndInt32 materialId) const
{
	materialId++;
	if (materialId >= handle->m_materialCount)
	{
		materialId = -1;
	}
	return materialId;
}

ndInt32 ndMeshEffect::GetMaterialID(ndIndexArray* const handle, ndInt32 materialHandle) const
{
	return handle->m_materials[materialHandle];
}

ndInt32 ndMeshEffect::GetMaterialIndexCount(ndIndexArray* const handle, ndInt32 materialHandle) const
{
	return handle->m_materialsIndexCount[materialHandle];
}

void ndMeshEffect::GetMaterialGetIndexStream(ndIndexArray* const handle, ndInt32 materialHandle, ndInt32* const indexArray) const
{
	ndInt32 index = 0;
	ndInt32 textureID = handle->m_materials[materialHandle];
	for (ndInt32 j = 0; j < handle->m_indexCount; ++j)
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

void ndMeshEffect::GetMaterialGetIndexStreamShort(ndIndexArray* const handle, ndInt32 materialHandle, ndInt16* const indexArray) const
{
	ndInt32 index = 0;
	ndInt32 textureID = handle->m_materials[materialHandle];
	for (ndInt32 j = 0; j < handle->m_indexCount; ++j)
	{
		if (handle->m_indexList[j * 4 + 3] == textureID)
		{
			indexArray[index + 0] = (ndInt16)handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = (ndInt16)handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = (ndInt16)handle->m_indexList[j * 4 + 2];
			index += 3;
		}
	}
}

void ndMeshEffect::ApplyTransform(const ndMatrix& matrix)
{
	matrix.TransformTriplex(&m_points.m_vertex[0].m_x, sizeof(ndBigVector), &m_points.m_vertex[0].m_x, sizeof(ndBigVector), m_points.m_vertex.GetCount());

	ndMatrix invMatix(matrix.Inverse4x4());
	invMatix.m_posit = ndVector::m_wOne;
	ndMatrix rotation(invMatix.Transpose4X4());
	for (ndInt32 i = 0; i < m_attrib.m_normalChannel.GetCount(); ++i)
	{
		ndVector n(ndFloat32(m_attrib.m_normalChannel[i].m_x), ndFloat32(m_attrib.m_normalChannel[i].m_y), ndFloat32(m_attrib.m_normalChannel[i].m_z), ndFloat32(0.0f));
		n = rotation.RotateVector(n);
		dAssert(n.m_w == ndFloat32(0.0f));
		dAssert(n.DotProduct(n).GetScalar() > ndFloat32(0.0f));
		n = n.Normalize();
		m_attrib.m_normalChannel[i].m_x = n.m_x;
		m_attrib.m_normalChannel[i].m_y = n.m_y;
		m_attrib.m_normalChannel[i].m_z = n.m_z;
	}

	for (ndInt32 i = 0; i < m_attrib.m_binormalChannel.GetCount(); ++i)
	{
		ndVector n(ndFloat32(m_attrib.m_binormalChannel[i].m_x), ndFloat32(m_attrib.m_binormalChannel[i].m_y), ndFloat32(m_attrib.m_binormalChannel[i].m_z), ndFloat32(0.0f));
		n = rotation.RotateVector(n);
		dAssert(n.m_w == ndFloat32(0.0f));
		dAssert(n.DotProduct(n).GetScalar() > ndFloat32(0.0f));
		n = n.Normalize();
		m_attrib.m_binormalChannel[i].m_x = n.m_x;
		m_attrib.m_binormalChannel[i].m_y = n.m_y;
		m_attrib.m_binormalChannel[i].m_z = n.m_z;
	}
}

void ndMeshEffect::MergeFaces(const ndMeshEffect* const source)
{
	ndInt32 mark = source->IncLRU();
	ndPolyhedra::Iterator iter(*source);
	ndInt32 maxMatIndex = 0;
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const face = &(*iter);
		if ((face->m_incidentFace > 0) && (face->m_mark < mark)) 
		{
			BeginBuildFace();
			ndEdge* ptr = face;
			do {
				ptr->m_mark = mark;
				ndInt32 vIndex = ptr->m_incidentVertex;
				ndInt32 aIndex = ndInt32(ptr->m_userData);
				AddPoint(source->m_points.m_vertex[vIndex].m_x, source->m_points.m_vertex[vIndex].m_y, source->m_points.m_vertex[vIndex].m_z);
				if (source->m_points.m_layers.GetCount()) 
				{
					AddLayer(source->m_points.m_layers[vIndex]);
				}
	
				if (source->m_attrib.m_materialChannel.GetCount()) 
				{
					ndInt32 matIndex = source->m_attrib.m_materialChannel[aIndex];
					maxMatIndex = dMax(matIndex, maxMatIndex);
					AddMaterial(matIndex);
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
			} while (ptr != face);
			EndBuildFace();
		}
	}

	for (ndInt32 i = m_materials.GetCount(); i <= maxMatIndex; ++i)
	{
		m_materials.PushBack(source->m_materials[i]);
	}
}

// create a convex hull
ndMeshEffect::ndMeshEffect(const ndFloat64* const vertexCloud, ndInt32 count, ndInt32 strideInByte, ndFloat64 distTol)
	:ndPolyhedra()
	,m_name()
	,m_points()
	,m_attrib()
	,m_clusters()
	,m_materials()
	,m_vertexBaseCount(0)
	,m_constructionIndex(0)
{
	Init();
	if (count >= 4) 
	{
		ndConvexHull3d convexHull(vertexCloud, strideInByte, count, distTol);
		if (convexHull.GetCount()) 
		{
			ndStack<ndInt32> faceCountPool(convexHull.GetCount());
			ndStack<ndInt32> vertexIndexListPool(convexHull.GetCount() * 3);
	
			ndInt32 index = 0;
			dMeshVertexFormat format;
			format.m_faceCount = convexHull.GetCount();
			format.m_faceIndexCount = &faceCountPool[0];
			format.m_vertex.m_indexList = &vertexIndexListPool[0];
			format.m_vertex.m_data = &convexHull.GetVertexPool()[0].m_x;
			format.m_vertex.m_strideInBytes = sizeof(ndBigVector);
			for (ndConvexHull3d::ndNode* faceNode = convexHull.GetFirst(); faceNode; faceNode = faceNode->GetNext()) 
			{
				ndConvexHull3dFace& face = faceNode->GetInfo();
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
	:ndPolyhedra()
	,m_name()
	,m_points()
	,m_attrib()
	,m_clusters()
	,m_materials()
	,m_vertexBaseCount(0)
	,m_constructionIndex(0)
{
	class ndMeshEffectBuilder : public ndShapeDebugNotify
	{
		public:
		ndMeshEffectBuilder()
			:ndShapeDebugNotify()
			, m_vertex(1024)
			, m_faceMaterial(1024)
			, m_faceIndexCount(1024)
			, m_brush(0)
		{
		}

		void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			const ndFloat64 brush = m_brush;
			m_faceIndexCount.PushBack(vertexCount);
			m_faceMaterial.PushBack(0);
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				const ndBigVector point(faceVertex[i].m_x, faceVertex[i].m_y, faceVertex[i].m_z, brush);
				m_vertex.PushBack(point);
			}
		}

		ndArray<ndBigVector> m_vertex;
		ndArray<ndInt32> m_faceMaterial;
		ndArray<ndInt32> m_faceIndexCount;

		ndInt32 m_brush;
		ndInt32 m_materialIndex;
	};
	ndMeshEffectBuilder builder;

	Init();
	if (((ndShape*)shape.GetShape())->GetAsShapeCompound())
	{
		dAssert(0);
		//	dgCollisionInfo collisionInfo;
		//	collision->GetCollisionInfo(&collisionInfo);
		//
		//	ndInt32 brush = 0;
		//	ndMatrix matrix(collisionInfo.m_offsetMatrix);
		//	dgCollisionCompound* const compoundCollision = (dgCollisionCompound*)collision->GetChildShape();
		//	for (ndTree<dgCollisionCompound::dgNodeBase*, ndInt32>::ndNode* node = compoundCollision->GetFirstNode(); node; node = compoundCollision->GetNextNode(node)) {
		//		builder.m_brush = brush;
		//		brush++;
		//		dgCollisionInstance* const childShape = compoundCollision->GetCollisionFromNode(node);
		//		childShape->DebugCollision(matrix, (dgCollision::OnDebugCollisionMeshCallback) dgMeshEffectBuilder::GetShapeFromCollision, &builder);
		//	}
		//
	}
	else
	{
		ndMatrix matrix(dGetIdentityMatrix());
		shape.DebugShape(matrix, builder);
	}

	ndStack<ndInt32>indexListBuffer(builder.m_vertex.GetCount());
	ndInt32* const indexList = &indexListBuffer[0];
	dVertexListToIndexList(&builder.m_vertex[0].m_x, sizeof(ndBigVector), 4, builder.m_vertex.GetCount(), &indexList[0], DG_VERTEXLIST_INDEXLIST_TOL);

	ndMeshEffect::dMeshVertexFormat vertexFormat;
	vertexFormat.m_faceCount = builder.m_faceIndexCount.GetCount();
	vertexFormat.m_faceIndexCount = &builder.m_faceIndexCount[0];
	vertexFormat.m_faceMaterial = &builder.m_faceIndexCount[0];
	vertexFormat.m_vertex.m_data = &builder.m_vertex[0].m_x;
	vertexFormat.m_vertex.m_strideInBytes = sizeof(ndBigVector);
	vertexFormat.m_vertex.m_indexList = &indexList[0];

	m_materials.PushBack(dMaterial());
	BuildFromIndexList(&vertexFormat);
	RepairTJoints();
	CalculateNormals(ndFloat32(45.0f * ndDegreeToRad));
}

ndMatrix ndMeshEffect::CalculateOOBB(ndBigVector& size) const
{
	ndMatrix sphere(CalculateSphere(size, &m_points.m_vertex[0].m_x, sizeof(ndBigVector)));
	//size = sphere.m_size;
	//size.m_w = 0.0f;

	//	ndMatrix permuation (dGetIdentityMatrix());
	//	permuation[0][0] = ndFloat32 (0.0f);
	//	permuation[0][1] = ndFloat32 (1.0f);
	//	permuation[1][1] = ndFloat32 (0.0f);
	//	permuation[1][2] = ndFloat32 (1.0f);
	//	permuation[2][2] = ndFloat32 (0.0f);
	//	permuation[2][0] = ndFloat32 (1.0f);
	//	while ((size.m_x < size.m_y) || (size.m_x < size.m_z)) {
	//		sphere = permuation * sphere;
	//		size = permuation.UnrotateVector(size);
	//	}

	return sphere;
}

void ndMeshEffect::CalculateAABB(ndBigVector& minBox, ndBigVector& maxBox) const
{
	ndBigVector minP(ndFloat64(1.0e15f), ndFloat64(1.0e15f), ndFloat64(1.0e15f), ndFloat64(0.0f));
	ndBigVector maxP(-ndFloat64(1.0e15f), -ndFloat64(1.0e15f), -ndFloat64(1.0e15f), ndFloat64(0.0f));

	ndPolyhedra::Iterator iter(*this);
	const ndBigVector* const points = &m_points.m_vertex[0];
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		const ndBigVector& p(points[edge->m_incidentVertex]);

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

ndFloat64 ndMeshEffect::CalculateVolume() const
{
	ndPolyhedraMassProperties localData;

	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndVector points[256];

		ndEdge* face = &(*iter);
		if ((face->m_incidentFace > 0) && (face->m_mark != mark)) 
		{
			ndInt32 count = 0;
			ndEdge* ptr = face;
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

	ndVector com;
	ndVector inertia;
	ndVector crossInertia;
	ndFloat32 volume = localData.MassProperties(com, inertia, crossInertia);
	return volume;
}

ndMeshEffect* ndMeshEffect::GetNextLayer(ndInt32 mark)
{
	Iterator iter(*this);
	ndEdge* edge = nullptr;
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

	const ndInt32 layer = m_points.m_layers.GetCount() ? m_points.m_layers[edge->m_incidentVertex] : 0;
	ndPolyhedra polyhedra;

	polyhedra.BeginFace();
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge1 = &(*iter);
		if ((edge1->m_mark < mark) && (edge1->m_incidentFace > 0)) 
		{
			const ndInt32 thislayer = m_points.m_layers.GetCount() ? m_points.m_layers[edge1->m_incidentVertex] : 0;
			if (thislayer == layer) 
			{
				ndEdge* ptr = edge1;
				ndInt32 count = 0;
				ndInt32 faceIndex[256];
				ndInt64 faceDataIndex[256];
				do 
				{
					ptr->m_mark = mark;
					faceIndex[count] = ptr->m_incidentVertex;
					faceDataIndex[count] = ptr->m_userData;
					count++;
					dAssert(count < ndInt32(sizeof(faceIndex) / sizeof(faceIndex[0])));
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

ndShapeInstance* ndMeshEffect::CreateConvexCollision(ndFloat64 tolerance) const
{
	ndStack<ndVector> poolPtr(m_points.m_vertex.GetCount() * 2);
	ndVector* const pool = &poolPtr[0];
	
	ndBigVector minBox;
	ndBigVector maxBox;
	CalculateAABB(minBox, maxBox);
	ndVector com((minBox + maxBox) * ndVector::m_half);
	
	ndInt32 count = 0;
	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const vertex = &(*iter);
		if (vertex->m_mark != mark) 
		{
			ndEdge* ptr = vertex;
			do 
			{
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != vertex);
	
			if (count < ndInt32(poolPtr.GetElementsCount())) 
			{
				const ndBigVector p(m_points.m_vertex[vertex->m_incidentVertex]);
				pool[count] = ndVector(p) - com;
				count++;
			}
		}
	}
	
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit += matrix.RotateVector(com);
	matrix.m_posit.m_w = ndFloat32(1.0f);
	
	ndShapeConvexHull* const collision = new ndShapeConvexHull(count, sizeof(ndVector), ndFloat32(tolerance), &pool[0].m_x);
	if (!collision->GetConvexVertexCount()) 
	{
		delete collision;
		return nullptr;
	}
	ndShapeInstance* const instance = new ndShapeInstance(collision);
	instance->SetLocalMatrix(matrix);
	return instance;
}

void ndMeshEffect::ConvertToPolygons()
{
	UnpackPoints();
	ndPolyhedra leftOversOut;
	ndPolyhedra::ConvexPartition(&m_points.m_vertex[0].m_x, sizeof(ndBigVector), &leftOversOut);
	dAssert(leftOversOut.GetCount() == 0);

	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
		edge->m_userData = (edge->m_incidentFace) > 0 ? edge->m_incidentVertex : 0;
	}
	PackPoints();

	RepairTJoints();
	dAssert(Sanity());
}

void ndMeshEffect::Triangulate()
{
/*	
	ndInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
	ndInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];
	ndPolyhedra polygon(GetAllocator());
	polygon.BeginFace();
	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter1(*this);
	for (iter1.Begin(); iter1; iter1++) {
		ndEdge* const face = &iter1.GetNode()->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			ndEdge* ptr = face;
			ndInt32 indexCount = 0;
			do {
				index[indexCount] = ndInt32(ptr->m_userData);
				userData[indexCount] = ptr->m_incidentVertex;
				ptr->m_mark = mark;
				indexCount++;
				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, index, userData);
		}
	}
	polygon.EndFace();

	ndPolyhedra leftOversOut(GetAllocator());
	polygon.Triangulate(&m_points.m_vertex[0].m_x, sizeof(ndBigVector), &leftOversOut);
	dAssert(leftOversOut.GetCount() == 0);

	SetLRU(0);
	RemoveAll();
	BeginFace();
	mark = polygon.IncLRU();
	ndPolyhedra::Iterator iter(polygon);
	for (iter.Begin(); iter; iter++) {
		ndEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			ndEdge* ptr = face;
			ndInt32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = ndInt32(ptr->m_userData);
				userData[indexCount] = ptr->m_incidentVertex;
				indexCount++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();
*/	

	UnpackPoints();
	ndPolyhedra leftOversOut;
	ndPolyhedra::Triangulate(&m_points.m_vertex[0].m_x, sizeof(ndBigVector), &leftOversOut);
	dAssert(leftOversOut.GetCount() == 0);

	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
		edge->m_userData = (edge->m_incidentFace) > 0 ? edge->m_incidentVertex : 0;
	}
	PackPoints();

	RepairTJoints();
	dAssert(Sanity());
}

bool ndMeshEffect::HasOpenEdges() const
{
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const face = &(*iter);
		if (face->m_incidentFace < 0) 
		{
			return true;
		}
	}
	return false;
}

void ndMeshEffect::RemoveUnusedVertices(ndInt32* const)
{
	//dAssert(!vertexMapResult);
	UnpackAttibuteData();
	PackAttibuteData();
	UnpackPoints();
	PackPoints();
}

ndEdge* ndMeshEffect::InsertEdgeVertex(ndEdge* const edge, ndFloat64 param)
{
	ndEdge* const twin = edge->m_twin;
	AddInterpolatedEdgeAttribute(edge, param);

	ndInt32 edgeAttrV0 = ndInt32(edge->m_userData);
	ndInt32 twinAttrV0 = ndInt32(twin->m_userData);

	ndEdge* const faceA0 = edge->m_next;
	ndEdge* const faceA1 = edge->m_prev;
	ndEdge* const faceB0 = twin->m_next;
	ndEdge* const faceB1 = twin->m_prev;
	SpliteEdge(m_points.m_vertex.GetCount() - 1, edge);

	faceA0->m_prev->m_userData = ndUnsigned64(m_attrib.m_pointChannel.GetCount() - 2);
	faceA1->m_next->m_userData = ndUnsigned64(edgeAttrV0);

	faceB0->m_prev->m_userData = ndUnsigned64(m_attrib.m_pointChannel.GetCount() - 1);
	faceB1->m_next->m_userData = ndUnsigned64(twinAttrV0);
	return faceA1->m_next;
}

void ndMeshEffect::AddInterpolatedEdgeAttribute(ndEdge* const edge, ndFloat64 param)
{
	ndFloat64 t1 = param;
	ndFloat64 t0 = ndFloat64(1.0f) - t1;
	dAssert(t1 >= ndFloat64(0.0f));
	dAssert(t1 <= ndFloat64(1.0f));

	const ndInt32 vertexIndex = m_points.m_vertex.GetCount();
	m_points.m_vertex.PushBack(m_points.m_vertex[edge->m_incidentVertex].Scale(t0) + m_points.m_vertex[edge->m_next->m_incidentVertex].Scale(t1));
	if (m_points.m_layers.GetCount())
	{
		m_points.m_layers.PushBack(m_points.m_layers[edge->m_incidentVertex]);
	}

	m_attrib.m_pointChannel.PushBack(vertexIndex);
	m_attrib.m_pointChannel.PushBack(vertexIndex);

	if (m_attrib.m_materialChannel.GetCount()) 
	{
		m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[ndInt32(edge->m_userData)]);
		m_attrib.m_materialChannel.PushBack(m_attrib.m_materialChannel[ndInt32(edge->m_twin->m_userData)]);
	}

	if (m_attrib.m_normalChannel.GetCount())
	{
		ndTriplex edgeNormal;
		ndTriplex edgeNormal0(m_attrib.m_normalChannel[ndInt32(edge->m_userData)]);
		ndTriplex edgeNormal1(m_attrib.m_normalChannel[ndInt32(edge->m_next->m_userData)]);
		edgeNormal.m_x = edgeNormal0.m_x * ndFloat32(t0) + edgeNormal1.m_x * ndFloat32(t1);
		edgeNormal.m_y = edgeNormal0.m_y * ndFloat32(t0) + edgeNormal1.m_y * ndFloat32(t1);
		edgeNormal.m_z = edgeNormal0.m_z * ndFloat32(t0) + edgeNormal1.m_z * ndFloat32(t1);
		m_attrib.m_normalChannel.PushBack(edgeNormal);

		ndTriplex twinNormal;
		ndTriplex twinNormal0(m_attrib.m_normalChannel[ndInt32(edge->m_twin->m_next->m_userData)]);
		ndTriplex twinNormal1(m_attrib.m_normalChannel[ndInt32(edge->m_twin->m_userData)]);
		twinNormal.m_x = twinNormal0.m_x * ndFloat32(t0) + twinNormal1.m_x * ndFloat32(t1);
		twinNormal.m_y = twinNormal0.m_y * ndFloat32(t0) + twinNormal1.m_y * ndFloat32(t1);
		twinNormal.m_z = twinNormal0.m_z * ndFloat32(t0) + twinNormal1.m_z * ndFloat32(t1);
		m_attrib.m_normalChannel.PushBack(twinNormal);
	}
	if (m_attrib.m_binormalChannel.GetCount()) 
	{
		dAssert(0);
	}

	if (m_attrib.m_uv0Channel.GetCount()) 
	{
		dAttibutFormat::dgUV edgeUV;
		dAttibutFormat::dgUV edgeUV0(m_attrib.m_uv0Channel[ndInt32(edge->m_userData)]);
		dAttibutFormat::dgUV edgeUV1(m_attrib.m_uv0Channel[ndInt32(edge->m_next->m_userData)]);
		edgeUV.m_u = edgeUV0.m_u * ndFloat32(t0) + edgeUV1.m_u * ndFloat32(t1);
		edgeUV.m_v = edgeUV0.m_v * ndFloat32(t0) + edgeUV1.m_v * ndFloat32(t1);
		m_attrib.m_uv0Channel.PushBack(edgeUV);

		dAttibutFormat::dgUV twinUV;
		dAttibutFormat::dgUV twinUV0(m_attrib.m_uv0Channel[ndInt32(edge->m_twin->m_next->m_userData)]);
		dAttibutFormat::dgUV twinUV1(m_attrib.m_uv0Channel[ndInt32(edge->m_twin->m_userData)]);
		twinUV.m_u = twinUV0.m_u * ndFloat32(t0) + twinUV1.m_u * ndFloat32(t1);
		twinUV.m_v = twinUV0.m_v * ndFloat32(t0) + twinUV1.m_v * ndFloat32(t1);
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

void ndMeshEffect::FlipWinding()
{
	ndInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
	ndInt64	userData[DG_MESH_EFFECT_POINT_SPLITED];

	ndPolyhedra polyhedra(*this);
	RemoveAll();

	ndPolyhedra::BeginFace();
	ndInt32 mark = polyhedra.IncLRU();
	ndPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) 
		{
			ndEdge* ptr = face;
			ndInt32 indexCount = 0;
			do 
			{
				index[indexCount] = ptr->m_incidentVertex;
				userData[indexCount] = ptr->m_userData;
				ptr->m_mark = mark;
				indexCount++;
				ptr = ptr->m_prev;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	ndPolyhedra::EndFace();
}

/*
void dgMeshEffect::GetWeightBlendChannel(dgInt32 strideInByte, dgFloat32* const bufferOut) const
{
	dgInt8* const buffer = (dgInt8*)bufferOut;
	for (dgInt32 i = 0; i < m_attrib.m_pointChannel.m_count; ++i) {
		const dgInt32 j = i * strideInByte;
		dgFloat32* const ptr = (dgFloat32*)&buffer[j];

		const dgInt32 index = m_attrib.m_pointChannel[i];
		const dgFloat32* const p = &m_points.m_weights[index].m_weightBlends[0];
		ptr[0] = dgFloat32(p[0]);
		ptr[1] = dgFloat32(p[1]);
		ptr[2] = dgFloat32(p[2]);
		ptr[3] = dgFloat32(p[3]);
	}
}

void dgMeshEffect::GetWeightIndexChannel(dgInt32 strideInByte, dgInt32* const bufferOut) const
{
	dgInt8* const buffer = (dgInt8*)bufferOut;
	for (dgInt32 i = 0; i < m_attrib.m_pointChannel.m_count; ++i) {
		const dgInt32 j = i * strideInByte;
		dgInt32* const ptr = (dgInt32*)&buffer[j];
		const dgInt32 index = m_attrib.m_pointChannel[i];
		const dgInt32* const p = &m_points.m_weights[index].m_controlIndex[0];
		ptr[0] = p[0];
		ptr[1] = p[1];
		ptr[2] = p[2];
		ptr[3] = p[3];
	}
}
*/

ndMeshEffect::dVertexCluster* ndMeshEffect::FindCluster(const char* const name) const
{
	ndTree<dVertexCluster, const ndString>::ndNode* const node = m_clusters.Find(name);
	if (node)
	{
		return &node->GetInfo();
	}
	return nullptr;
}

ndMeshEffect::dVertexCluster* ndMeshEffect::CreateCluster(const char* const name)
{
	ndTree<dVertexCluster, const ndString>::ndNode* node = m_clusters.Find(name);
	if (!node)
	{
		node = m_clusters.Insert(name);
	}
	return &node->GetInfo();
}