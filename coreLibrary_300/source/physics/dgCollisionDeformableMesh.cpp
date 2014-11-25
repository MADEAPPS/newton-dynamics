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
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgCollisionBVH.h"
#include "dgDeformableBody.h"
#include "dgDeformableContact.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableMesh.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

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
		m_surfaceArea = side0 % side1;
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
			m_surfaceArea = side0 % side1;
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



dgCollisionDeformableMesh::dgCollisionDeformableMesh (const dgCollisionDeformableMesh& source)
	:dgCollisionConvex (source.GetAllocator(), source.m_signature, source.m_collisionId)
	,m_basePosit(dgFloat32 (0.0f))
	,m_particles (source.m_particles)
	,m_visualSegments(source.m_allocator)	
	,m_skinThickness(source.m_skinThickness)
	,m_nodesCount(source.m_nodesCount)
	,m_trianglesCount(source.m_trianglesCount)
	,m_visualVertexCount(source.m_visualVertexCount)
	,m_world (source.m_world)
	,m_myBody(NULL)
	,m_indexList(NULL)
	,m_faceNormals(NULL)
	,m_rootNode(NULL)
	,m_nodesMemory(NULL)
	,m_visualVertexData(NULL) 
	,m_onDebugDisplay(source.m_onDebugDisplay)
	,m_isdoubleSided(source.m_isdoubleSided)
{
	m_rtti = source.m_rtti;

	dgDeformableBodiesUpdate& softBodyList = *m_world;
	softBodyList.AddShape (this);

	m_indexList = (dgInt32*) dgMallocStack (3 * m_trianglesCount * sizeof (dgInt32));
	m_faceNormals = (dgVector*) dgMallocStack (m_trianglesCount * sizeof (dgVector));
	m_nodesMemory = (dgDeformableNode*) dgMallocStack(m_nodesCount * sizeof (dgDeformableNode));

	memcpy (m_indexList, source.m_indexList, 3 * m_trianglesCount * sizeof (dgInt32));
	memcpy (m_faceNormals, source.m_faceNormals, m_trianglesCount * sizeof (dgVector));
	memcpy (m_nodesMemory, source.m_nodesMemory, m_nodesCount * sizeof (dgDeformableNode));

	dgInt32 index = dgInt32 (source.m_rootNode - source.m_nodesMemory);
	m_rootNode = &m_nodesMemory[index];
	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		dgDeformableNode* const node = &m_nodesMemory[i];
		if (node->m_parent) {
			dgInt32 index = dgInt32(node->m_parent - source.m_nodesMemory);
			node->m_parent = &m_nodesMemory[index];
		}

		if (node->m_left) {
			dgInt32 index = dgInt32 (node->m_left - source.m_nodesMemory);
			node->m_left = &m_nodesMemory[index];
		}

		if (node->m_right) {
			dgInt32 index = dgInt32 (node->m_right - source.m_nodesMemory);
			node->m_right = &m_nodesMemory[index];
		}
	}

	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);

	m_visualVertexData = (dgVisualVertexData*) dgMallocStack (m_visualVertexCount * sizeof (dgVisualVertexData));
	memcpy (m_visualVertexData, source.m_visualVertexData, m_visualVertexCount * sizeof (dgVisualVertexData));

	for (dgList<dgMeshSegment>::dgListNode* node = source.m_visualSegments.GetFirst(); node; node = node->GetNext() ) {
		dgMeshSegment& srcSegment = node->GetInfo();
		dgMeshSegment& segment = m_visualSegments.Append()->GetInfo();
		segment.m_material = srcSegment.m_material;
		segment.m_indexCount = srcSegment.m_indexCount;
		segment.m_indexList = (dgInt32*) dgMallocStack (2 * segment.m_indexCount * sizeof (dgInt32));
		memcpy (segment.m_indexList, srcSegment.m_indexList, 2 * segment.m_indexCount * sizeof (dgInt32));
	}
}

						   
dgCollisionDeformableMesh::dgCollisionDeformableMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
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


dgCollisionDeformableMesh::dgCollisionDeformableMesh(dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID)
	:dgCollisionConvex (mesh->GetAllocator(), 0, collsionID)
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
{
	m_rtti |= dgCollisionDeformableMesh_RTTI;

	dgDeformableBodiesUpdate& softBodyList = *m_world;
	softBodyList.AddShape (this);

	dgMeshEffect meshCopy (*mesh);
	meshCopy.Triangulate();

	m_trianglesCount = meshCopy.GetTotalFaceCount (); 
	m_nodesMemory = (dgDeformableNode*) dgMallocStack((m_trianglesCount * 2 - 1) * sizeof (dgDeformableNode));
	m_indexList = (dgInt32*) dgMallocStack (3 * m_trianglesCount * sizeof (dgInt32));
	m_faceNormals = (dgVector*) dgMallocStack (m_trianglesCount * sizeof (dgVector));

	dgInt32 stride = meshCopy.GetVertexStrideInByte() / sizeof (dgFloat64);  
	dgFloat64* const vertex = meshCopy.GetVertexPool();  

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_particles.m_unitMass[i] = dgFloat32 (1.0f);
		m_particles.m_veloc[i] = dgVector (dgFloat32 (0.0f));
		m_particles.m_posit[i] = dgVector (&vertex[i * stride]) & dgVector::m_triplexMask;
	}

	dgInt32 indexCount = meshCopy.GetTotalIndexCount (); 
	dgStack<dgInt32> faceArray (m_trianglesCount);
	dgStack<dgInt32> materials (m_trianglesCount);
	dgStack<void*>indexArray (indexCount);
	meshCopy.GetFaces (&faceArray[0], &materials[0], &indexArray[0]);
	for (dgInt32 i = 0; i < m_trianglesCount; i ++) {
		dgInt32 count = faceArray[i];
		dgAssert (faceArray[i]);
		for (dgInt32 j = 0; j < count; j ++) {
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
	m_rootNode = BuildTopDown (m_nodesCount, m_nodesMemory, NULL);

	ImproveTotalFitness();
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);

	// create visual vertex data
	m_visualVertexCount = meshCopy.GetPropertiesCount();
	m_visualVertexData = (dgVisualVertexData*) dgMallocStack(m_visualVertexCount * sizeof (dgVisualVertexData));

	for (dgInt32 i = 0; i < m_visualVertexCount; i ++) {
		dgMeshEffect::dgVertexAtribute& attribute = meshCopy.GetAttribute (i);
		m_visualVertexData[i].m_uv0[0] = dgFloat32 (attribute.m_u0);
		m_visualVertexData[i].m_uv0[1] = dgFloat32 (attribute.m_v0);
	}

	for (void* point = meshCopy.GetFirstPoint(); point; point = meshCopy.GetNextPoint(point)) {
		dgInt32 pointIndex = meshCopy.GetPointIndex (point);
		dgInt32 vertexIndex = meshCopy.GetVertexIndexFromPoint (point);
		m_visualVertexData[pointIndex].m_vertexIndex = vertexIndex;
	}

	for (dgInt32 i = 0; i < m_trianglesCount; i ++) {
		dgInt32 mat = materials[i];
		if (mat != -1) {
			dgInt32 count = 0;
			for (dgInt32 j = i; j < m_trianglesCount; j ++) {
				dgInt32 mat1 = materials[j];
				if (mat == mat1) {
					materials[j] = -1;
					count ++;
				}
			}

			dgMeshSegment& segment = m_visualSegments.Append()->GetInfo();
			segment.m_material = mat;
			segment.m_indexCount = count * 3;
			segment.m_indexList = (dgInt32*) dgMallocStack( 2 * segment.m_indexCount * sizeof (dgInt32));

			dgInt32 index0 = 0;
			dgInt32 index1 = m_trianglesCount * 3;
			for (dgInt32 j = i; j < m_trianglesCount; j ++) {
				if (materials[j] == -1) {
					dgInt32 m0 = meshCopy.GetPointIndex(indexArray[j * 3 + 0]);
					dgInt32 m1 = meshCopy.GetPointIndex(indexArray[j * 3 + 1]);
					dgInt32 m2 = meshCopy.GetPointIndex(indexArray[j * 3 + 2]);

					segment.m_indexList[index0 + 0] = dgInt16 (m0);
					segment.m_indexList[index0 + 1] = dgInt16 (m1);
					segment.m_indexList[index0 + 2] = dgInt16 (m2);
					index0 += 3;

					segment.m_indexList[index1 + 0] = dgInt16 (m0);
					segment.m_indexList[index1 + 1] = dgInt16 (m2);
					segment.m_indexList[index1 + 2] = dgInt16 (m1);
					index1 += 3;
				}
			}
		}
	}
}


dgCollisionDeformableMesh::~dgCollisionDeformableMesh(void)
{
	if (m_indexList) {
		dgFree (m_indexList);
		dgFree (m_faceNormals);
	}
	if (m_nodesMemory) {
		dgFree (m_nodesMemory);
	}
	if (m_visualVertexData) {
		dgFree (m_visualVertexData);
	}

	dgDeformableBodiesUpdate& softBodyList = *m_world;
	softBodyList.RemoveShape (this);
}

dgBody* dgCollisionDeformableMesh::GetBody() const
{
	return m_myBody;
}

void dgCollisionDeformableMesh::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (p0.m_x <= p1.m_x);
	dgAssert (p0.m_y <= p1.m_y);
	dgAssert (p0.m_z <= p1.m_z);

	m_boxSize = (p1 - p0).Scale3 (dgFloat32 (0.5f)); 
	m_boxOrigin = (p1 + p0).Scale3 (dgFloat32 (0.5f)); 

	dgFloat32 padding = m_skinThickness + DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS;
	m_boxSize += dgVector (padding, padding, padding, dgFloat32 (0.0f));
}


dgFloat32 dgCollisionDeformableMesh::CalculateSurfaceArea (const dgDeformableNode* const node0, const dgDeformableNode* const node1, dgVector& minBox, dgVector& maxBox) const
{
	minBox = dgVector (dgMin (node0->m_minBox.m_x, node1->m_minBox.m_x), dgMin (node0->m_minBox.m_y, node1->m_minBox.m_y), dgMin (node0->m_minBox.m_z, node1->m_minBox.m_z), dgFloat32 (0.0f));
	maxBox = dgVector (dgMax (node0->m_maxBox.m_x, node1->m_maxBox.m_x), dgMax (node0->m_maxBox.m_y, node1->m_maxBox.m_y), dgMax (node0->m_maxBox.m_z, node1->m_maxBox.m_z), dgFloat32 (0.0f));		
	dgVector side0 (maxBox - minBox);
	dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
	return side0 % side1;
}


dgCollisionDeformableMesh::dgDeformableNode* dgCollisionDeformableMesh::BuildTopDown (dgInt32 count, dgDeformableNode* const children, dgDeformableNode* const parent)
{
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
}


void dgCollisionDeformableMesh::ImproveNodeFitness (dgDeformableNode* const node)
{
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
}


bool dgCollisionDeformableMesh::SanityCheck () const
{
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

	return true;
}


void dgCollisionDeformableMesh::ImproveTotalFitness()
{
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
}


void dgCollisionDeformableMesh::SetSkinThickness (dgFloat32 skinThickness)
{
	m_skinThickness = dgAbsf (skinThickness);
	if (m_skinThickness < DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS) {
		m_skinThickness = DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS;
	}
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
}


dgInt32 dgCollisionDeformableMesh::GetVisualPointsCount() const
{
	return m_visualVertexCount * (m_isdoubleSided ? 2 : 1);
}


void dgCollisionDeformableMesh::UpdateVisualNormals()
{
	for (dgInt32 i = 0; i < m_trianglesCount; i ++)	{
		dgInt32 i0 = m_indexList[i * 3];
		dgInt32 i1 = m_indexList[i * 3 + 1];
		dgInt32 i2 = m_indexList[i * 3 + 2];
		dgVector e0 (m_particles.m_posit[i1] - m_particles.m_posit[i0]);
		dgVector e1 (m_particles.m_posit[i2] - m_particles.m_posit[i0]);
		dgVector n = e0 * e1;
		n = n.Scale3(dgRsqrt (n % n));
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
		n = n.Scale3(dgRsqrt (n % n));
		m_visualVertexData[i].m_normals[0] = n.m_x;
		m_visualVertexData[i].m_normals[1] = n.m_y;
		m_visualVertexData[i].m_normals[2] = n.m_z;
	}
}


void dgCollisionDeformableMesh::GetVisualVertexData(dgInt32 vertexStrideInByte, dgFloat32* const vertex, dgInt32 normalStrideInByte, dgFloat32* const normals, dgInt32 uvStrideInByte0, dgFloat32* const uv0)
{
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
}


void* dgCollisionDeformableMesh::GetFirtVisualSegment() const
{
	return m_visualSegments.GetFirst();
}

void* dgCollisionDeformableMesh::GetNextVisualSegment(void* const segment) const
{
	return ((dgList<dgMeshSegment>::dgListNode*) segment)->GetNext();
}


dgInt32 dgCollisionDeformableMesh::GetSegmentMaterial (void* const segment) const
{
	const dgMeshSegment& info = ((dgList<dgMeshSegment>::dgListNode*) segment)->GetInfo();
	return info.m_material;
}

dgInt32 dgCollisionDeformableMesh::GetSegmentIndexCount (void* const segment) const
{
	const dgMeshSegment& info = ((dgList<dgMeshSegment>::dgListNode*) segment)->GetInfo();
	return info.m_indexCount * (m_isdoubleSided ? 2 : 1);
}

const dgInt32* dgCollisionDeformableMesh::GetSegmentIndexList (void* const segment) const
{
	const dgMeshSegment& info = ((dgList<dgMeshSegment>::dgListNode*) segment)->GetInfo();
	return info.m_indexList;
}


dgInt32 dgCollisionDeformableMesh::CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy)
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
	return m_particles.m_count;
}

dgVector dgCollisionDeformableMesh::GetParticlePosition(dgInt32 index) const
{
	return m_particles.m_posit[index];
}

void dgCollisionDeformableMesh::CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
    dgVector origin (matrix.TransformVector(m_boxOrigin));
    dgVector size (matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));

    p0 = (origin - size) & dgVector::m_triplexMask;
    p1 = (origin + size) & dgVector::m_triplexMask;
}

dgFloat32 dgCollisionDeformableMesh::CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const
{
	dgCollision::OnDebugCollisionMeshCallback saveCallback = m_onDebugDisplay;
	m_onDebugDisplay = NULL;

	dgFloat32 volume = dgCollisionConvex::CalculateMassProperties (offset, inertia, crossInertia, centerOfMass);

	m_onDebugDisplay = saveCallback;
	return volume;
}

void dgCollisionDeformableMesh::SetOnDebugDisplay (dgCollision::OnDebugCollisionMeshCallback debugDisplay)
{
	m_onDebugDisplay = debugDisplay;
}

void dgCollisionDeformableMesh::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
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
}
