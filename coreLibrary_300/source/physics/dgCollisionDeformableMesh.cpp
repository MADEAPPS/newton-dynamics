/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgWorld.h"
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgDynamicBody.h"
#include "dgCollisionBVH.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableMesh.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


class dgCollisionDeformableMesh::dgSpringMassSolver
{
	public:
	dgSpringMassSolver(dgCollisionDeformableMesh* const me, const dgJacobianPair* const matrix, const dgFloat32* const invDiagonal, dgFloat32* const b, dgFloat32 unitInvMass)
		:m_unitInvMass(unitInvMass)
		,m_matrix(matrix)
		,m_invDiagonal(invDiagonal)
		,m_edgeCount(me->m_edgeCount)
		,m_massesCount(me->m_massesCount)
	{
		dgFloat32* const x = me->m_lambda;
		m_edgeList = me->m_edgeList;
		m_massScaler = me->m_unitMassScaler;
		
		m_buffer = dgAlloca(dgVector, m_massesCount);
		dgFloat32* const r0 = dgAlloca(dgFloat32, m_edgeCount);
		dgFloat32* const p0 = dgAlloca(dgFloat32, m_edgeCount);
		dgFloat32* const MinvR0 = dgAlloca(dgFloat32, m_edgeCount);
		dgFloat32* const matrixP0 = dgAlloca(dgFloat32, m_edgeCount);

		MatrixTimeVector(matrixP0, x);
		Sub(r0, b, matrixP0);
		InversePrecoditionerTimeVector(p0, r0);

		dgInt32 iter = 0;
		dgFloat32 num = DotProduct(r0, p0);
		dgFloat32 error2 = num;

		const dgInt32 maxPasses = 8;
		const dgFloat32 tolerance = dgFloat32(1.0e-2f);
		for (dgInt32 j = 0; (j < maxPasses) && (error2 > tolerance); j++) {

			MatrixTimeVector(matrixP0, p0);
			dgFloat32 den = DotProduct(p0, matrixP0);

			dgAssert(fabs(den) > dgFloat32(0.0f));
			dgFloat32 alpha = num / den;

			ScaleAdd(x, x, alpha, p0);
			ScaleAdd(r0, r0, -alpha, matrixP0);
			InversePrecoditionerTimeVector(MinvR0, r0);

			dgFloat32 num1 = DotProduct(r0, MinvR0);
			dgFloat32 beta = num1 / num;
			ScaleAdd(p0, MinvR0, beta, p0);
			num = DotProduct(r0, MinvR0);
			iter++;
			error2 = num;
		}
	}

	private:
	DG_INLINE void MatrixTimeVector(dgFloat32* const out, const dgFloat32* const v)
	{
		memset(m_buffer, 0, m_massesCount * sizeof(dgVector));
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			const dgInt32 j0 = m_edgeList[i].m_v0;
			const dgInt32 j1 = m_edgeList[i].m_v1;
			dgAssert(j0 < m_massesCount);
			dgAssert(j1 < m_massesCount);
			dgVector lamda(v[i]);
			const dgJacobianPair* const jacobian = &m_matrix[i];
			m_buffer[j0] += jacobian->m_j01.CompProduct4(lamda);
			m_buffer[j1] += jacobian->m_j10.CompProduct4(lamda);
		}

		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			const dgInt32 j0 = m_edgeList[i].m_v0;
			const dgInt32 j1 = m_edgeList[i].m_v1;
			dgAssert(j0 < m_massesCount);
			dgAssert(j1 < m_massesCount);
			const dgJacobianPair* const jacobian = &m_matrix[i];
			dgVector invMass0(m_unitInvMass.Scale4(m_massScaler[j0]));
			dgVector invMass1(m_unitInvMass.Scale4(m_massScaler[j1]));
			dgVector accel(jacobian->m_j01.CompProduct4(invMass0).CompProduct4(m_buffer[j0]) + jacobian->m_j10.CompProduct4(invMass1).CompProduct4(m_buffer[j1]));
			out[i] = (accel.AddHorizontal()).GetScalar();
		}
	}

	DG_INLINE void Sub(dgFloat32* const a, const dgFloat32* const b, const dgFloat32* const c) const
	{
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			a[i] = b[i] - c[i];
		}
	}

	DG_INLINE void InversePrecoditionerTimeVector(dgFloat32* const out, const dgFloat32* const v) const
	{
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			out[i] = v[i] * m_invDiagonal[i];
		}
	}

	DG_INLINE dgFloat32 DotProduct(const dgFloat32* const b, const dgFloat32* const c) const
	{
		dgFloat32 product = dgFloat64(0.0f);
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			product += b[i] * c[i];
		}
		return product;
	}

	void ScaleAdd(dgFloat32* const a, const dgFloat32* const b, dgFloat32 scale, const dgFloat32* const c) const
	{
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			a[i] = b[i] + scale * c[i];
		}
	}


	dgVector m_unitInvMass;
	dgVector* m_buffer;
	dgFloat32* m_massScaler;
	const dgEdge* m_edgeList;
	const dgJacobianPair* m_matrix;
	const dgFloat32* m_invDiagonal;
	dgInt32 m_edgeCount;
	dgInt32 m_massesCount;
};

/*
#define DG_DEFORMABLE_STACK_DEPTH	256
#define DG_DEFORMABLE_PADDING				 (dgFloat32 (4.0f))
#define DG_DEFORMABLE_INV_PADDING			 (dgFloat32 (1.0f) / DG_DEFORMABLE_PADDING)

#define DG_DEFORMABLE_DEFAULT_STIFFNESS		 (dgFloat32 (0.3f))
#define DG_DEFORMABLE_DEFAULT_PLASTICITY	 (dgFloat32 (0.3f))
#define DG_DEFORMABLE_PLANE_DISTANCE_TOL	 (dgFloat32 (1.0e-4f))
#define DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS (dgFloat32 (5.0e-2f))


dgCollisionDeformableMesh::dgParticle::dgParticle (dgInt32 particlesCount)
	:m_count(particlesCount)
{
	m_posit = (dgVector*) dgMallocStack (m_count * sizeof (dgVector));
	m_veloc = (dgVector*) dgMallocStack (m_count * sizeof (dgVector));
	m_unitMass = (dgFloat32*) dgMallocStack (m_count * sizeof (dgFloat32));
}


dgCollisionDeformableMesh::dgParticle::dgParticle(const dgParticle& source)
	:m_count(source.m_count)
{
	m_posit = (dgVector*) dgMallocStack (m_count * sizeof (dgVector));
	m_veloc = (dgVector*) dgMallocStack (m_count * sizeof (dgVector));
	m_unitMass = (dgFloat32*) dgMallocStack (m_count * sizeof (dgFloat32));

	memcpy (m_unitMass, source.m_unitMass, m_count * sizeof (dgFloat32));
	memcpy (m_posit, source.m_posit, m_count * sizeof (dgVector));
	memcpy (m_veloc, source.m_veloc, m_count * sizeof (dgVector));
}


dgCollisionDeformableMesh::dgParticle::dgParticle (dgWorld* const world, dgDeserialize deserialization, void* const userData)
{
	dgAssert (0);
}

dgCollisionDeformableMesh::dgParticle::~dgParticle()
{
	if (m_unitMass) {
		dgFree (m_posit);
		dgFree (m_veloc);
		dgFree (m_unitMass);
	}
}

class dgCollisionDeformableMesh::dgDeformableNode
{
	public:
	dgDeformableNode ()
	{
	}

	~dgDeformableNode ()
	{
		if (m_left) {
			delete m_left;
		}
		if (m_right) {
			delete m_right;
		}
	}

	void TriangleBox (const dgVector* const position, const dgInt32* const faceIndices, dgVector& minP, dgVector& maxP) const
	{
		minP = position[faceIndices[0]]; 
		maxP = position[faceIndices[0]]; 
		for (dgInt32 i = 1; i < 3; i ++) {
			dgInt32 index = faceIndices[i];
			const dgVector& p  = position[index];

			minP.m_x = dgMin (p.m_x, minP.m_x); 
			minP.m_y = dgMin (p.m_y, minP.m_y); 
			minP.m_z = dgMin (p.m_z, minP.m_z); 

			maxP.m_x = dgMax (p.m_x, maxP.m_x); 
			maxP.m_y = dgMax (p.m_y, maxP.m_y); 
			maxP.m_z = dgMax (p.m_z, maxP.m_z); 
		}
	}

	void CalculateBox (const dgVector* const position, const dgInt32* const faceIndices) 
	{
		dgVector p0;
		dgVector p1;
		TriangleBox (position, faceIndices, p0, p1);

		p0 = p0.CompProduct3(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));
		p1 = p1.CompProduct3(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));

		m_minBox.m_x = dgFloor (p0.m_x) * DG_DEFORMABLE_INV_PADDING; 
		m_minBox.m_y = dgFloor (p0.m_y) * DG_DEFORMABLE_INV_PADDING;  
		m_minBox.m_z = dgFloor (p0.m_z) * DG_DEFORMABLE_INV_PADDING;  
		m_minBox.m_w = dgFloat32 (0.0f);

		m_maxBox.m_x = dgFloor (p1.m_x + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING;  
		m_maxBox.m_y = dgFloor (p1.m_y + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING;  
		m_maxBox.m_z = dgFloor (p1.m_z + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING;  
		m_maxBox.m_w = dgFloat32 (0.0f);

		dgVector side0 (m_maxBox - m_minBox);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		m_surfaceArea = side0.DotProduct3(side1);
	}

	dgInt32 UpdateBox (const dgVector* const position, const dgInt32* const faceIndices)
	{
		dgVector p0;
		dgVector p1;
		TriangleBox (position, faceIndices, p0, p1);

		p0 = p0.CompProduct3(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));
		p1 = p1.CompProduct3(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));

		dgVector minP (dgFloor (p0.m_x) * DG_DEFORMABLE_INV_PADDING, dgFloor (p0.m_y) * DG_DEFORMABLE_INV_PADDING, dgFloor (p0.m_z) * DG_DEFORMABLE_INV_PADDING, dgFloat32(0.0f));  
		dgVector maxP (dgFloor (p1.m_x + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING,  
					   dgFloor (p1.m_y + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING,  
					   dgFloor (p1.m_z + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING, dgFloat32(0.0f));

		dgInt32 state = dgCompareBox (minP, maxP, m_minBox, m_maxBox);
		if (state) {
			m_minBox = minP;
			m_maxBox = maxP;
			dgVector side0 (m_maxBox - m_minBox);
			dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
			m_surfaceArea = side0.DotProduct3(side1);
		}
		return state;
	}

	dgVector m_minBox;
	dgVector m_maxBox;
	dgInt32 m_indexStart;
	dgFloat32 m_surfaceArea;
	dgDeformableNode* m_left;
	dgDeformableNode* m_right;
	dgDeformableNode* m_parent;
};
*/

						   
dgCollisionDeformableMesh::dgCollisionDeformableMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionConvex (world, deserialization, userData, revisionNumber)
/*
	,m_particles (world, deserialization, userData) 
	,m_visualSegments(world->GetAllocator())
	,m_skinThickness (DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS)
	,m_nodesCount(0)
	,m_trianglesCount(0)
	,m_visualVertexCount(0)
	,m_world (world)
	,m_myBody(NULL)
	,m_indexList(NULL)
	,m_faceNormals(NULL)
	,m_rootNode(NULL)
	,m_nodesMemory(NULL)
	,m_visualVertexData(NULL) 
	,m_isdoubleSided(false)
*/
{
	dgAssert (0);
//	dgCollisionDeformableMeshList& softBodyList = *m_world;
//	softBodyList.Insert (this, this);

/*
	m_rtti |= dgCollisionDeformableMesh_RTTI;
	dgAABBPolygonSoup::Deserialize (deserialization, userData);

	dgVector p0; 
	dgVector p1; 
	GetAABB (p0, p1);
	SetCollisionBBox(p0, p1);
*/
}





#if 0
dgBody* dgCollisionDeformableMesh::GetBody() const
{
	dgAssert(0);
	return NULL;
//	return m_myBody;
}


dgFloat32 dgCollisionDeformableMesh::CalculateSurfaceArea (const dgDeformableNode* const node0, const dgDeformableNode* const node1, dgVector& minBox, dgVector& maxBox) const
{
	dgAssert(0);
	return 0;
/*
	minBox = dgVector (dgMin (node0->m_minBox.m_x, node1->m_minBox.m_x), dgMin (node0->m_minBox.m_y, node1->m_minBox.m_y), dgMin (node0->m_minBox.m_z, node1->m_minBox.m_z), dgFloat32 (0.0f));
	maxBox = dgVector (dgMax (node0->m_maxBox.m_x, node1->m_maxBox.m_x), dgMax (node0->m_maxBox.m_y, node1->m_maxBox.m_y), dgMax (node0->m_maxBox.m_z, node1->m_maxBox.m_z), dgFloat32 (0.0f));		
	dgVector side0 (maxBox - minBox);
	dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
	return side0.DotProduct3(side1);
*/
}


dgCollisionDeformableMesh::dgDeformableNode* dgCollisionDeformableMesh::BuildTopDown (dgInt32 count, dgDeformableNode* const children, dgDeformableNode* const parent)
{
	dgAssert(0);
	return NULL;
/*
	dgDeformableNode* root = NULL;				
	if (count == 1) {
		root = children;
		root->m_left = NULL;
		root->m_right = NULL;
		root->m_parent = parent;
	} else if (count == 2) {
		root = &m_nodesMemory[m_nodesCount];
		m_nodesCount ++;
		root->m_indexStart = -1;
		root->m_parent = parent;
		root->m_left = BuildTopDown (1, children, root);
		root->m_right = BuildTopDown (1, &children[1], root);
		root->m_surfaceArea = CalculateSurfaceArea (root->m_left, root->m_right, root->m_minBox, root->m_maxBox);
	} else {

		dgVector median (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector varian (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < count; i ++) {
			const dgDeformableNode* const node = &children[i];
			dgVector p ((node->m_minBox + node->m_maxBox).Scale3 (0.5f));
			median += p;
			varian += p.CompProduct3 (p);
		}

		varian = varian.Scale3 (dgFloat32 (count)) - median.CompProduct3(median);

		dgInt32 index = 0;
		dgFloat32 maxVarian = dgFloat32 (-1.0e10f);
		for (dgInt32 i = 0; i < 3; i ++) {
			if (varian[i] > maxVarian) {
				index = i;
				maxVarian = varian[i];
			}
		}

		dgVector center = median.Scale3 (dgFloat32 (1.0f) / dgFloat32 (count));
		dgFloat32 test = center[index];

		dgInt32 i0 = 0;
		dgInt32 i1 = count - 1;
		do {    
			for (; i0 <= i1; i0 ++) {
				const dgDeformableNode* const node = &children[i0];
				dgFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dgFloat32 (0.5f);
				if (val > test) {
					break;
				}
			}

			for (; i1 >= i0; i1 --) {
				const dgDeformableNode* const node = &children[i1];
				dgFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dgFloat32 (0.5f);
				if (val < test) {
					break;
				}
			}

			if (i0 < i1)	{
				dgSwap(children[i0], children[i1]);
				i0++; 
				i1--;
			}

		} while (i0 <= i1);

		if (i0 > 0){
			i0 --;
		}
		if ((i0 + 1) >= count) {
			i0 = count - 2;
		}

		dgInt32 spliteCount = i0 + 1;

		root = &m_nodesMemory[m_nodesCount];
		m_nodesCount ++;
		root->m_indexStart = -1;
		root->m_parent = parent;
		root->m_left = BuildTopDown (spliteCount, children, root);
		root->m_right = BuildTopDown (count - spliteCount, &children[spliteCount], root);
		root->m_surfaceArea = CalculateSurfaceArea (root->m_left, root->m_right, root->m_minBox, root->m_maxBox);
	}

	return root;
*/
}


void dgCollisionDeformableMesh::ImproveNodeFitness (dgDeformableNode* const node)
{
	dgAssert(0);
/*
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (!node->m_parent) {
		node->m_surfaceArea = CalculateSurfaceArea (node->m_left, node->m_right, node->m_minBox, node->m_maxBox);
	} else {
		if (node->m_parent->m_left == node) {
			dgFloat32 cost0 = CalculateSurfaceArea (node->m_left, node->m_right, node->m_minBox, node->m_maxBox);
			node->m_surfaceArea = cost0;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgDeformableNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 
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
				parent->m_minBox = cost1P0;
				parent->m_maxBox = cost1P1;		
				parent->m_surfaceArea = cost1;


			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgDeformableNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 

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

				parent->m_minBox = cost2P0;
				parent->m_maxBox = cost2P1;		
				parent->m_surfaceArea = cost2;
			}
		} else {
			dgFloat32 cost0 = CalculateSurfaceArea (node->m_left, node->m_right, node->m_minBox, node->m_maxBox);
			node->m_surfaceArea = cost0;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);


			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgDeformableNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 
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

				parent->m_minBox = cost1P0;
				parent->m_maxBox = cost1P1;		
				parent->m_surfaceArea = cost1;

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgDeformableNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 
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

				parent->m_minBox = cost2P0;
				parent->m_maxBox = cost2P1;		
				parent->m_surfaceArea = cost2;
			}
		}
	}

	dgAssert (!m_rootNode->m_parent);
*/
}


bool dgCollisionDeformableMesh::SanityCheck () const
{
	dgAssert(0);
/*
	for (dgInt32 i = 0; i < m_nodesCount; i ++)	{		
		dgDeformableNode* const node = &m_nodesMemory[i];
		if (node->m_indexStart >= 0) {
			if (node->m_left) {
				return false;
			}
			if (node->m_right) {
				return false;
			}
			if (node->m_parent) {
				if ((node->m_parent->m_left != node) &&  (node->m_parent->m_right != node)) {
					return false;
				}
			}
		} else {
			if (node->m_left->m_parent != node) {
				return false;
			}
			if (node->m_right->m_parent != node) {
				return false;
			}

			if (!node->m_parent) {
				if (node != m_rootNode) {
					return false;
				}
			} else {
				if ((node->m_parent->m_left != node) && (node->m_parent->m_right != node)) {
					return false;
				}
			}
		}
	}
*/
	return true;
}


void dgCollisionDeformableMesh::ImproveTotalFitness()
{
	dgAssert(0);
/*
	dgInt32 count = m_nodesCount - m_trianglesCount; 
	dgInt32 maxPasses = 2 * dgExp2 (count) + 1;

	dgDeformableNode* const nodes = &m_nodesMemory[m_trianglesCount];
	dgFloat64 newCost = dgFloat32 (1.0e20f);
	dgFloat64 prevCost = newCost;
	do {
		prevCost = newCost;
		for (dgInt32 i = 0; i < count; i ++) {
			dgDeformableNode* const node = &nodes[i];
			ImproveNodeFitness (node);
		}

		newCost	= dgFloat32 (0.0f);
		for (dgInt32 i = 0; i < count; i ++) {
			const dgDeformableNode* const node = &nodes[i];
			newCost += node->m_surfaceArea;
		}

		maxPasses --;
	} while (maxPasses && (newCost < (prevCost * dgFloat32 (0.9f))));

	dgAssert (SanityCheck());
*/
}


void dgCollisionDeformableMesh::SetSkinThickness (dgFloat32 skinThickness)
{
	dgAssert(0);
/*
	m_skinThickness = dgAbsf (skinThickness);
	if (m_skinThickness < DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS) {
		m_skinThickness = DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS;
	}
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
*/
}


dgInt32 dgCollisionDeformableMesh::GetVisualPointsCount() const
{
	dgAssert(0);
	return 0;
//	return m_visualVertexCount * (m_isdoubleSided ? 2 : 1);
}


void dgCollisionDeformableMesh::UpdateVisualNormals()
{
	dgAssert(0);
/*
	for (dgInt32 i = 0; i < m_trianglesCount; i ++)	{
		dgInt32 i0 = m_indexList[i * 3];
		dgInt32 i1 = m_indexList[i * 3 + 1];
		dgInt32 i2 = m_indexList[i * 3 + 2];
		dgVector e0 (m_particles.m_posit[i1] - m_particles.m_posit[i0]);
		dgVector e1 (m_particles.m_posit[i2] - m_particles.m_posit[i0]);
		dgVector n = e0.CrossProduct3(e1);
		n = n.Scale3(dgRsqrt (n.DotProduct3(n)));
		m_faceNormals[i] = n;
	} 

	dgAssert (m_visualVertexData);
	for (dgInt32 i = 0; i < m_visualVertexCount; i ++)	{
		m_visualVertexData[i].m_normals[0] = dgFloat32 (0.0f);
		m_visualVertexData[i].m_normals[1] = dgFloat32 (0.0f);
		m_visualVertexData[i].m_normals[2] = dgFloat32 (0.0f);
	}

	for (dgList<dgMeshSegment>::dgListNode* node = m_visualSegments.GetFirst(); node; node = node->GetNext() ) {
		const dgMeshSegment& segment = node->GetInfo();

		for (dgInt32 i = 0; i < segment.m_indexCount; i ++) {
			dgInt32 index = segment.m_indexList[i];
			dgInt32 faceIndexNormal = i / 3;
			m_visualVertexData[index].m_normals[0] += m_faceNormals[faceIndexNormal].m_x;
			m_visualVertexData[index].m_normals[1] += m_faceNormals[faceIndexNormal].m_y;
			m_visualVertexData[index].m_normals[2] += m_faceNormals[faceIndexNormal].m_z;
		}
	}

	for (dgInt32 i = 0; i < m_visualVertexCount; i ++)	{
		dgVector n (m_visualVertexData[i].m_normals[0], m_visualVertexData[i].m_normals[1], m_visualVertexData[i].m_normals[2],  dgFloat32 (0.0f));
		n = n.Scale3(dgRsqrt (n.DotProduct3(n)));
		m_visualVertexData[i].m_normals[0] = n.m_x;
		m_visualVertexData[i].m_normals[1] = n.m_y;
		m_visualVertexData[i].m_normals[2] = n.m_z;
	}
*/
}


void dgCollisionDeformableMesh::GetVisualVertexData(dgInt32 vertexStrideInByte, dgFloat32* const vertex, dgInt32 normalStrideInByte, dgFloat32* const normals, dgInt32 uvStrideInByte0, dgFloat32* const uv0)
{
	dgAssert(0);
/*
	dgInt32 vertexStride = vertexStrideInByte / sizeof (dgFloat32); 
	dgInt32 normalStride = normalStrideInByte / sizeof (dgFloat32);  
	dgInt32 uvStride0 = uvStrideInByte0 / sizeof (dgFloat32); 

//	const dgMatrix& matrix = m_myBody->GetMatrix();
	const dgVector* const posit = m_particles.m_posit;

	if (m_isdoubleSided) {
		for (dgInt32 i = 0; i < m_visualVertexCount; i ++) {
			dgInt32 index = m_visualVertexData[i].m_vertexIndex;
//			dgVector p (matrix.UntransformVector (m_particles.m_posit[index]));
//			dgVector n (matrix.UnrotateVector(dgVector (&m_visualVertexData[i].m_normals[0])));

			vertex[i * vertexStride + 0] = posit[index].m_x;
			vertex[i * vertexStride + 1] = posit[index].m_y;
			vertex[i * vertexStride + 2] = posit[index].m_z;

			normals[i * normalStride + 0] = m_visualVertexData[i].m_normals[0];
			normals[i * normalStride + 1] = m_visualVertexData[i].m_normals[1];
			normals[i * normalStride + 2] = m_visualVertexData[i].m_normals[2];

			uv0[i * uvStride0 + 0] = m_visualVertexData[i].m_uv0[0];
			uv0[i * uvStride0 + 1] = m_visualVertexData[i].m_uv0[1];

			dgInt32 j = i + m_visualVertexCount;
			vertex[j * vertexStride + 0] = posit[index].m_x;
			vertex[j * vertexStride + 1] = posit[index].m_y;
			vertex[j * vertexStride + 2] = posit[index].m_z;

			normals[j * normalStride + 0] = -m_visualVertexData[i].m_normals[0];
			normals[j * normalStride + 1] = -m_visualVertexData[i].m_normals[1];
			normals[j * normalStride + 2] = -m_visualVertexData[i].m_normals[2];

			uv0[j * uvStride0 + 0] = m_visualVertexData[i].m_uv0[0];
			uv0[j * uvStride0 + 1] = m_visualVertexData[i].m_uv0[1];
		}
	} else {
		for (dgInt32 i = 0; i < m_visualVertexCount; i ++) {
			dgInt32 index = m_visualVertexData[i].m_vertexIndex;
			//dgVector p (matrix.UntransformVector (m_particles.m_posit[index]));
			//dgVector n (matrix.UnrotateVector(dgVector (&m_visualVertexData[i].m_normals[0])));

			vertex[i * vertexStride + 0] = posit[index].m_x;
			vertex[i * vertexStride + 1] = posit[index].m_y;
			vertex[i * vertexStride + 2] = posit[index].m_z;

			normals[i * normalStride + 0] = m_visualVertexData[i].m_normals[0];
			normals[i * normalStride + 1] = m_visualVertexData[i].m_normals[1];
			normals[i * normalStride + 2] = m_visualVertexData[i].m_normals[2];

			uv0[i * uvStride0 + 0] = m_visualVertexData[i].m_uv0[0];
			uv0[i * uvStride0 + 1] = m_visualVertexData[i].m_uv0[1];
		}
	}
*/
}


void* dgCollisionDeformableMesh::GetFirtVisualSegment() const
{
	dgAssert(0);
	return NULL;
//	return m_visualSegments.GetFirst();
}

void* dgCollisionDeformableMesh::GetNextVisualSegment(void* const segment) const
{
	dgAssert(0);
	return NULL;
//	return ((dgList<dgMeshSegment>::dgListNode*) segment)->GetNext();
}


dgInt32 dgCollisionDeformableMesh::GetSegmentMaterial (void* const segment) const
{
	dgAssert(0);
	return NULL;
//	const dgMeshSegment& info = ((dgList<dgMeshSegment>::dgListNode*) segment)->GetInfo();
//	return info.m_material;
}

dgInt32 dgCollisionDeformableMesh::GetSegmentIndexCount (void* const segment) const
{
	dgAssert(0);
	return NULL;

//	const dgMeshSegment& info = ((dgList<dgMeshSegment>::dgListNode*) segment)->GetInfo();
//	return info.m_indexCount * (m_isdoubleSided ? 2 : 1);
}

const dgInt32* dgCollisionDeformableMesh::GetSegmentIndexList (void* const segment) const
{
	dgAssert(0);
	return NULL;
//	const dgMeshSegment& info = ((dgList<dgMeshSegment>::dgListNode*) segment)->GetInfo();
//	return info.m_indexList;
}


dgInt32 dgCollisionDeformableMesh::CalculateContacts (dgBroadPhase::dgPair* const pair, dgCollisionParamProxy& proxy)
{
//dgAssert (0);
return 0;
/*

	if (m_rootNode) {
		dgAssert (IsType (dgCollision::dgCollisionDeformableMesh_RTTI));

		if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
			CalculateContactsToCollisionTree (pair, proxy);

//		if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
//			contactCount = CalculateContactsToSingle (pair, proxy);
//		} else if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
//			contactCount = CalculateContactsToCompound (pair, proxy);
//		} else if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
//			contactCount = CalculateContactsToCollisionTree (pair, proxy);
//		} else if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionHeightField_RTTI)) {
//			contactCount = CalculateContactsToHeightField (pair, proxy);
//		} else {
//			dgAssert (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionUserMesh_RTTI));
//			contactCount = CalculateContactsBruteForce (pair, proxy);
//		}
		} else {
			dgAssert (0);
		}
	}
	pair->m_contactCount = 0;
	return 0;
*/
}

dgInt32 dgCollisionDeformableMesh::GetParticleCount() const
{
	dgAssert(0);
	return NULL;
//	return m_particles.m_count;
}

dgVector dgCollisionDeformableMesh::GetParticlePosition(dgInt32 index) const
{
	dgAssert(0);
	return dgVector(0.0f);
//	return m_particles.m_posit[index];
}

void dgCollisionDeformableMesh::CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	dgAssert(0);
/*
    dgVector origin (matrix.TransformVector(m_boxOrigin));
    dgVector size (matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));

    p0 = (origin - size) & dgVector::m_triplexMask;
    p1 = (origin + size) & dgVector::m_triplexMask;
*/
}

dgFloat32 dgCollisionDeformableMesh::CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const
{
	dgAssert(0);
	return NULL;
/*
	dgCollision::OnDebugCollisionMeshCallback saveCallback = m_onDebugDisplay;
	m_onDebugDisplay = NULL;

	dgFloat32 volume = dgCollisionConvex::CalculateMassProperties (offset, inertia, crossInertia, centerOfMass);

	m_onDebugDisplay = saveCallback;
	return volume;
*/
}

void dgCollisionDeformableMesh::SetOnDebugDisplay (dgCollision::OnDebugCollisionMeshCallback debugDisplay)
{
	dgAssert(0);
//	m_onDebugDisplay = debugDisplay;
}

void dgCollisionDeformableMesh::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgAssert(0);
/*
	const dgVector* const particlePosit = m_particles.m_posit;
	for (dgInt32 i = 0; i < m_trianglesCount; i ++ ) {
		dgTriplex points[3];
		for (dgInt32 j = 0; j < 3; j ++) {
			dgInt32 index = m_indexList[i * 3 + j];
			dgVector p (matrix.TransformVector(particlePosit[index]));
			points[j].m_x = p.m_x;
			points[j].m_y = p.m_y;
			points[j].m_z = p.m_z;
		}
		callback (userData, 3, &points[0].m_x, 0);
	}
*/
}
#endif

dgInt32 dgCollisionDeformableMesh::CalculateSignature() const
{
	dgAssert(0);
	return 0;
}

void dgCollisionDeformableMesh::SetCollisionBBox(const dgVector& p0, const dgVector& p1)
{
	dgAssert(0);
}

void dgCollisionDeformableMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert(0);
}


dgCollisionDeformableMesh::~dgCollisionDeformableMesh(void)
{
	dgFree(m_posit);
	dgFree(m_veloc);
	dgFree(m_lambda);
	dgFree(m_edgeList);
	dgFree(m_indexMap);
	dgFree(m_restlength);
	dgFree(m_externalforce);
	dgFree(m_internalforce);
	dgFree(m_unitMassScaler);
}

dgCollisionDeformableMesh::dgCollisionDeformableMesh(dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID)
	:dgCollisionConvex(mesh->GetAllocator(), 0, collsionID)
	,m_posit(NULL)
	,m_veloc(NULL)
	,m_edgeList(NULL)
	,m_lambda(NULL)
	,m_unitMassScaler(NULL)
	,m_indexMap(NULL)
	,m_restlength(NULL)
	,m_externalforce(NULL)
	,m_internalforce(NULL)
	,m_edgeCount(0)
	,m_massesCount(0)
	,m_vertexCount(mesh->GetVertexCount())

	/*
	,m_particles (mesh->GetVertexCount ())
	,m_visualSegments(mesh->GetAllocator())
	,m_skinThickness(DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS)
	,m_nodesCount(0)
	,m_trianglesCount(0)
	,m_visualVertexCount(0)
	,m_world (world)
	,m_myBody(NULL)
	,m_indexList(NULL)
	,m_faceNormals(NULL)
	,m_rootNode(NULL)
	,m_nodesMemory(NULL)
	,m_visualVertexData(NULL)
	,m_onDebugDisplay(NULL)
	,m_isdoubleSided(false)
	*/
{
	m_rtti |= dgCollisionDeformableMesh_RTTI;
/*
	dgDeformableBodiesUpdate& softBodyList = *m_world;
	softBodyList.AddShape(this);

	dgMeshEffect meshCopy(*mesh);
	meshCopy.Triangulate();

	m_trianglesCount = meshCopy.GetTotalFaceCount();
	m_nodesMemory = (dgDeformableNode*)dgMallocStack((m_trianglesCount * 2 - 1) * sizeof(dgDeformableNode));
	m_indexList = (dgInt32*)dgMallocStack(3 * m_trianglesCount * sizeof(dgInt32));
	m_faceNormals = (dgVector*)dgMallocStack(m_trianglesCount * sizeof(dgVector));

	dgInt32 stride = meshCopy.GetVertexStrideInByte() / sizeof(dgFloat64);
	dgFloat64* const vertex = meshCopy.GetVertexPool();

	for (dgInt32 i = 0; i < m_particles.m_count; i++) {
		m_particles.m_unitMass[i] = dgFloat32(1.0f);
		m_particles.m_veloc[i] = dgVector(dgFloat32(0.0f));
		m_particles.m_posit[i] = dgVector(&vertex[i * stride]) & dgVector::m_triplexMask;
	}

	dgInt32 indexCount = meshCopy.GetTotalIndexCount();
	dgStack<dgInt32> faceArray(m_trianglesCount);
	dgStack<dgInt32> materials(m_trianglesCount);
	dgStack<void*>indexArray(indexCount);
	meshCopy.GetFaces(&faceArray[0], &materials[0], &indexArray[0]);
	for (dgInt32 i = 0; i < m_trianglesCount; i++) {
		dgInt32 count = faceArray[i];
		dgAssert(faceArray[i]);
		for (dgInt32 j = 0; j < count; j++) {
			dgInt32 k = meshCopy.GetVertexIndex(indexArray[i * 3 + j]);
			m_indexList[i * 3 + j] = k;
		}

		//dgTrace (("%d %d %d\n", m_indexList[i * 3 + 0], m_indexList[i * 3 + 1], m_indexList[i * 3 + 2]));

		dgDeformableNode& node = m_nodesMemory[i];
		node.m_left = NULL;
		node.m_right = NULL;
		node.m_parent = NULL;
		node.m_indexStart = i * 3;
		node.CalculateBox(m_particles.m_posit, &m_indexList[i * 3]);
	}

	m_nodesCount = m_trianglesCount;
	m_rootNode = BuildTopDown(m_nodesCount, m_nodesMemory, NULL);

	ImproveTotalFitness();
	SetCollisionBBox(m_rootNode->m_minBox, m_rootNode->m_maxBox);

	// create visual vertex data
	m_visualVertexCount = meshCopy.GetPropertiesCount();
	m_visualVertexData = (dgVisualVertexData*)dgMallocStack(m_visualVertexCount * sizeof(dgVisualVertexData));

	for (dgInt32 i = 0; i < m_visualVertexCount; i++) {
		dgMeshEffect::dgVertexAtribute& attribute = meshCopy.GetAttribute(i);
		m_visualVertexData[i].m_uv0[0] = dgFloat32(attribute.m_u0);
		m_visualVertexData[i].m_uv0[1] = dgFloat32(attribute.m_v0);
	}

	for (void* point = meshCopy.GetFirstPoint(); point; point = meshCopy.GetNextPoint(point)) {
		dgInt32 pointIndex = meshCopy.GetPointIndex(point);
		dgInt32 vertexIndex = meshCopy.GetVertexIndexFromPoint(point);
		m_visualVertexData[pointIndex].m_vertexIndex = vertexIndex;
	}

	for (dgInt32 i = 0; i < m_trianglesCount; i++) {
		dgInt32 mat = materials[i];
		if (mat != -1) {
			dgInt32 count = 0;
			for (dgInt32 j = i; j < m_trianglesCount; j++) {
				dgInt32 mat1 = materials[j];
				if (mat == mat1) {
					materials[j] = -1;
					count++;
				}
			}

			dgMeshSegment& segment = m_visualSegments.Append()->GetInfo();
			segment.m_material = mat;
			segment.m_indexCount = count * 3;
			segment.m_indexList = (dgInt32*)dgMallocStack(2 * segment.m_indexCount * sizeof(dgInt32));

			dgInt32 index0 = 0;
			dgInt32 index1 = m_trianglesCount * 3;
			for (dgInt32 j = i; j < m_trianglesCount; j++) {
				if (materials[j] == -1) {
					dgInt32 m0 = meshCopy.GetPointIndex(indexArray[j * 3 + 0]);
					dgInt32 m1 = meshCopy.GetPointIndex(indexArray[j * 3 + 1]);
					dgInt32 m2 = meshCopy.GetPointIndex(indexArray[j * 3 + 2]);

					segment.m_indexList[index0 + 0] = dgInt16(m0);
					segment.m_indexList[index0 + 1] = dgInt16(m1);
					segment.m_indexList[index0 + 2] = dgInt16(m2);
					index0 += 3;

					segment.m_indexList[index1 + 0] = dgInt16(m0);
					segment.m_indexList[index1 + 1] = dgInt16(m2);
					segment.m_indexList[index1 + 2] = dgInt16(m1);
					index1 += 3;
				}
			}
		}
	}
	//	SetVolumeAndCG ();
*/
	dgStack<dgInt32> indexList(m_vertexCount);
	dgStack<dgVector> positBuff(m_vertexCount);
	dgVector* const posit = &positBuff[0];
	const dgFloat64* const meshVertex = mesh->GetVertexPool();
	const dgInt32 stride = mesh->GetVertexStrideInByte() / sizeof (dgFloat64);
	for (dgInt32 i = 0; i < m_vertexCount; i++) {
		posit[i] = dgVector(dgFloat32(meshVertex[i * stride + 0]), dgFloat32(meshVertex[i * stride + 1]), dgFloat32(meshVertex[i * stride + 2]), dgFloat32(0.0f));
	}
	m_massesCount = dgVertexListToIndexList(&posit[0].m_x, sizeof(dgVector), sizeof(dgVector), 0, m_vertexCount, &indexList[0], dgFloat32(1.0e-8f));

	m_indexMap = (dgInt16*)dgMallocStack(sizeof(dgInt16) * m_vertexCount);
	m_unitMassScaler = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_massesCount);
	m_posit = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	m_veloc = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	m_externalforce = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	m_internalforce = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		com += posit[i];
		m_unitMassScaler[i] = dgFloat32 (1.0f);
		m_internalforce[i] = dgVector(dgFloat32(0.0f));
		m_externalforce[i] = dgVector(dgFloat32(0.0f));
	}
// for now use a fix size box
	m_boxSize = dgVector (dgFloat32 (1.0f), dgFloat32(1.0f), dgFloat32(1.0f), dgFloat32(0.0f));
	m_boxOrigin = com.CompProduct4(dgFloat32(1.0f) / m_massesCount);

	for (dgInt32 i = 0; i < m_massesCount; i++) {
		m_veloc[i] = dgVector(0.0f);
		m_posit[i] = posit[i] - m_boxOrigin;
	}

	for (dgInt32 i = 0; i < m_vertexCount; i++) {
		m_indexMap[i] = dgInt16 (indexList[i]);
	}

	dgPolyhedra polyhedra(GetAllocator());
	polyhedra.BeginFace();
	for (void* facePtr = mesh->GetFirstFace(); facePtr; facePtr = mesh->GetNextEdge(facePtr)) {
		if (!mesh->IsFaceOpen(facePtr)) {
			dgInt32 indices[256];
			dgAssert(mesh->GetFaceIndexCount(facePtr) == 3);
			mesh->GetFaceIndex(facePtr, indices);
			indices[0] = m_indexMap[indices[0]];
			indices[1] = m_indexMap[indices[1]];
			indices[2] = m_indexMap[indices[2]];
			polyhedra.AddFace(3, indices);
		}
	}
	polyhedra.EndFace();
	m_edgeCount = polyhedra.GetCount() / 2;
	m_edgeList = (dgEdge*)dgMallocStack(sizeof(dgEdge) * m_edgeCount);
	m_lambda = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_edgeCount);
	m_restlength = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_edgeCount);

	dgInt32 edgeCount = 0;
	dgInt32 lru = polyhedra.IncLRU();
	dgPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) {
		::dgEdge& edge = iter.GetNode()->GetInfo();
		if (edge.m_mark != lru) {
			edge.m_mark = lru;
			edge.m_twin->m_mark = lru;
			m_edgeList[edgeCount].m_v0 = dgInt16(edge.m_incidentVertex);
			m_edgeList[edgeCount].m_v1 = dgInt16(edge.m_twin->m_incidentVertex);
			dgVector dp(m_posit[m_edgeList[edgeCount].m_v0] - m_posit[m_edgeList[edgeCount].m_v1]);
			m_restlength[edgeCount] = dgSqrt(dp.DotProduct3(dp));
			m_lambda[edgeCount] = dgFloat32(0.0f);
			edgeCount++;
		}
	}
}

dgCollisionDeformableMesh::dgCollisionDeformableMesh(const dgCollisionDeformableMesh& source)
	:dgCollisionConvex(source)
	,m_posit((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_massesCount))
	,m_veloc ((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_massesCount))
	,m_edgeList((dgEdge*)dgMallocStack(sizeof(dgEdge) * source.m_edgeCount))
	,m_lambda((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_edgeCount))
	,m_unitMassScaler((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_massesCount))
	,m_indexMap((dgInt16*)dgMallocStack(sizeof(dgInt16) * source.m_vertexCount))
	,m_restlength((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_edgeCount))
	,m_externalforce((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_vertexCount))
	,m_internalforce((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_vertexCount))
	,m_edgeCount(source.m_edgeCount)
	,m_massesCount(source.m_massesCount)
	,m_vertexCount(source.m_vertexCount)
{
	m_rtti = source.m_rtti;
	memcpy(m_veloc, source.m_veloc, m_massesCount * sizeof(dgVector));
	memcpy(m_posit, source.m_posit, m_massesCount * sizeof(dgVector));
	memcpy(m_unitMassScaler, source.m_unitMassScaler, m_massesCount * sizeof(dgFloat32));
	memcpy(m_lambda, source.m_lambda, m_edgeCount * sizeof(dgFloat32));
	memcpy(m_edgeList, source.m_edgeList, m_edgeCount * sizeof(dgEdge));
	memcpy(m_indexMap, source.m_indexMap, m_vertexCount * sizeof(dgInt16));
	memcpy(m_restlength, source.m_restlength, m_edgeCount * sizeof(dgFloat32));
	memcpy(m_externalforce, source.m_externalforce, m_massesCount * sizeof(dgVector));
	memcpy(m_internalforce, source.m_internalforce, m_massesCount * sizeof(dgVector));
}

dgMatrix dgCollisionDeformableMesh::CalculateInertiaAndCenterOfMass(const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const
{
	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		com = matrix.RotateVector(m_posit[i].CompProduct4 (localScale));
	}
	dgVector den (dgFloat32(1.0f / m_massesCount));
	dgMatrix inertia(dgGetIdentityMatrix());
	inertia.m_posit = com.CompProduct4(den);
	inertia.m_posit.m_w = dgFloat32(1.0f);

	return inertia;
}

void dgCollisionDeformableMesh::CalcAABB(const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	dgVector origin(matrix.TransformVector(m_boxOrigin));
	dgVector size(matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));
	p0 = (origin - size) & dgVector::m_triplexMask;
	p1 = (origin + size) & dgVector::m_triplexMask;
}

void dgCollisionDeformableMesh::ApplyExternalForces(dgDynamicBody* const body, dgFloat32 timestep)
{
	dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));
	const dgMatrix& matrix = body->GetCollision()->GetGlobalMatrix();

	dgVector den (dgFloat32(1.0f / m_massesCount));
	dgVector unitMass (body->m_mass.m_w * den.GetScalar());
	dgVector comSum(0.0f);
	dgVector comVeloc(0.0f);
	dgVector angularMomentum(0.0f);

	dgVector origin(matrix.TransformVector(m_boxOrigin));

	dgVector xxSum(dgFloat32(0.0f));
	dgVector xxSum2(dgFloat32(0.0f));
	dgVector xySum(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		xxSum += m_posit[i];
		comVeloc += m_veloc[i];
		xxSum2 += m_posit[i].CompProduct4(m_posit[i]);
		xySum += m_posit[i].CompProduct4(m_posit[i].ShiftTripleRight());
		angularMomentum += m_posit[i].CrossProduct3(m_veloc[i]);
	}
	dgVector yySum(xxSum.ShiftTripleRight());
	dgVector com (xxSum.CompProduct4(den) + origin);
	dgVector pxx0(origin - com);
	dgVector pxy0(pxx0.ShiftTripleRight());
	dgVector Ixx(unitMass.CompProduct4(xxSum2 + xxSum.CompProduct4(pxx0.CompProduct4(dgVector::m_two))) + pxx0.CompProduct4(pxx0.Scale4(body->m_mass.m_w)));
	dgVector Ixy(unitMass.CompProduct4(xySum  + xxSum.CompProduct4(pxy0) + yySum.CompProduct4(pxx0)) + pxx0.CompProduct4(pxy0.Scale4(body->m_mass.m_w)));
	comVeloc = comVeloc.CompProduct4(den);

	dgMatrix inertia(dgGetIdentityMatrix());
	inertia[0][0] = Ixx[1] + Ixx[2];
	inertia[1][1] = Ixx[0] + Ixx[2];
	inertia[2][2] = Ixx[0] + Ixx[1];
	
	inertia[0][1] = -Ixy[0];
	inertia[1][0] = -Ixy[0];
	inertia[0][2] = -Ixy[1];
	inertia[2][0] = -Ixy[1];
	inertia[1][2] = -Ixy[2];
	inertia[2][1] = -Ixy[2];
	body->m_invWorldInertiaMatrix = inertia.Symetric3by3Inverse();

	comVeloc += body->m_accel.Scale4(timestep);
	body->m_veloc = comVeloc;
	body->m_accel = body->m_externalForce.Scale4(body->m_invMass.m_w);

	angularMomentum = unitMass.CompProduct4(angularMomentum + pxx0.CrossProduct3(comVeloc)) + body->m_externalTorque.Scale4(timestep);
	body->m_alpha = body->m_invWorldInertiaMatrix.UnrotateVector(body->m_externalTorque);
	body->m_omega = body->m_invWorldInertiaMatrix.RotateVector(angularMomentum);

	dgVector damp(dgFloat32(1.0f));
	if (body->m_linearDampOn) {
		const dgFloat32 tau = dgFloat32(1.0f) / (dgFloat32(60.0f) * timestep);
		damp = dgVector (dgPow(dgFloat32(1.0f) - body->m_dampCoef.m_w, tau));
	}

	dgVector timestepV(timestep);
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		dgVector deltaVeloc(body->m_veloc + body->m_omega.CrossProduct3(m_posit[i] + pxx0) - m_veloc[i]);
		m_veloc[i] = m_veloc[i].CompProduct4 (damp) + deltaVeloc;
		m_posit[i] += (m_veloc[i] - comVeloc).CompProduct4(timestepV);
	}
}

void dgCollisionDeformableMesh::CollideMasses(dgDynamicBody* const myBody, dgBody* const otherBody)
{
}


void dgCollisionDeformableMesh::ApplyInternalForces(dgDynamicBody* const body, dgFloat32 timestep)
{
	dgFloat32* const b = dgAlloca(dgFloat32, m_edgeCount);
	dgJacobianPair* const jacobians = dgAlloca(dgJacobianPair, m_edgeCount);
	dgFloat32* const invDiagonal = dgAlloca(dgFloat32, m_edgeCount);

	//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	dgVector ks (dgFloat32 (100.0f));
	dgVector kd (dgFloat32(10.0f));
	dgVector kst (ks.GetScalar() * timestep);
	dgFloat32 den = dgFloat32(1.0f) / (dgFloat32(1.0f) + kd.GetScalar() * timestep + kst.GetScalar() * timestep);
	dgFloat32 unitInvMass = m_massesCount * body->m_invMass.m_w;
	for (dgInt32 i = 0; i < m_edgeCount; i++) {
		const dgInt32 j0 = m_edgeList[i].m_v0;
		const dgInt32 j1 = m_edgeList[i].m_v1;
		const dgVector& p0 = m_posit[j0];
		const dgVector& p1 = m_posit[j1];
		const dgVector& v0 = m_veloc[j0];
		const dgVector& v1 = m_veloc[j1];

		dgVector dir(p1 - p0);
		dir = dir.Scale4(dgFloat32 (1.0f )/ dgSqrt ((dir.DotProduct4(dir)).GetScalar()));
		jacobians[i].m_j01 = dir;
		jacobians[i].m_j10 = dir.CompProduct4(dgVector::m_negOne);
		dgAssert(jacobians[i].m_j01.m_w == dgFloat32(0.0f));
		dgAssert(jacobians[i].m_j10.m_w == dgFloat32(0.0f));

		dgVector invMass0(m_unitMassScaler[j0] * unitInvMass);
		dgVector invMass1(m_unitMassScaler[j1] * unitInvMass);
		dgVector diag = invMass0.CompProduct4(jacobians[i].m_j01.CompProduct4(jacobians[i].m_j01)) +
						invMass1.CompProduct4(jacobians[i].m_j10.CompProduct4(jacobians[i].m_j10));
		invDiagonal[i] = dgFloat32(1.0f / (diag.AddHorizontal()).GetScalar());

		dgVector a (ks.CompProduct4(jacobians[i].m_j01.CompProduct4(p0) + jacobians[i].m_j10.CompProduct4(p1)) +
					kd.CompProduct4(jacobians[i].m_j01.CompProduct4(v0) + jacobians[i].m_j10.CompProduct4(v1)) +
					kst.CompProduct4(jacobians[i].m_j01.CompProduct4(v0) + jacobians[i].m_j10.CompProduct4(v1)));
		dgFloat32 as = -(a.AddHorizontal()).GetScalar() - ks.GetScalar() * m_restlength[i];
		b[i] = den * as;
	}

	
	dgSpringMassSolver solver(this, jacobians, invDiagonal, b, unitInvMass);

}

