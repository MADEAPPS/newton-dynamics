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
#include "dgCollisionDeformableSolidMesh.h"


/*
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


dgCollisionDeformableSolidMesh::dgParticle::dgParticle (dgInt32 particlesCount)
	:m_com (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)) 
	,m_count(particlesCount)
{
	m_mass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_count);
	m_invMass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_count);
	m_position = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_deltaPosition = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_shapePosition = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_instantVelocity = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_internalVelocity = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	

	memset (m_mass, 0, sizeof (dgFloat32) * m_count);
	memset (m_invMass, 0, sizeof (dgFloat32) * m_count);
	
	memset (m_position, 0, sizeof (dgVector) * m_count);
	memset (m_deltaPosition, 0, sizeof (dgVector) * m_count);
	memset (m_shapePosition, 0, sizeof (dgVector) * m_count);
	memset (m_instantVelocity, 0, sizeof (dgVector) * m_count);
}

dgCollisionDeformableSolidMesh::dgParticle::dgParticle(const dgParticle& source)
	:m_com (source.m_com) 
	,m_count(source.m_count)
{
	m_mass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_count);
	m_invMass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_count);
	m_position = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_deltaPosition = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_shapePosition = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_instantVelocity = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);
	m_internalVelocity = (dgVector*) dgMallocStack (sizeof (dgVector) * m_count);

	memcpy (m_mass, source.m_mass, sizeof (dgFloat32) * m_count);
	memcpy (m_invMass, source.m_invMass, sizeof (dgFloat32) * m_count);
	
	memcpy (m_position, source.m_position, sizeof (dgVector) * m_count);
	memcpy (m_shapePosition, source.m_shapePosition, sizeof (dgVector) * m_count);
	memcpy (m_deltaPosition, source.m_deltaPosition, sizeof (dgVector) * m_count);
	memcpy (m_instantVelocity, source.m_instantVelocity, sizeof (dgVector) * m_count);
	memcpy (m_internalVelocity, source.m_internalVelocity, sizeof (dgVector) * m_count);
}

dgCollisionDeformableSolidMesh::dgParticle::dgParticle (dgWorld* const world, dgDeserialize deserialization, void* const userData)
{
	dgAssert (0);
}

dgCollisionDeformableSolidMesh::dgParticle::~dgParticle()
{
	dgFree (m_mass);
	dgFree (m_invMass);
	dgFree (m_position);
	dgFree (m_deltaPosition);
	dgFree (m_shapePosition);
	dgFree (m_instantVelocity);
	dgFree (m_internalVelocity);
}
*/

class dgCollisionDeformableSolidMesh::dgDeformationRegion
{
	public:

	void Init (dgCollisionDeformableSolidMesh* const me, dgInt32 count, dgInt16* const indexList)
	{
		dgAssert (!m_indices);
		m_count = count;
		m_indices = (dgInt16*) dgMallocStack(sizeof (dgInt16) * m_count);
		for (dgInt32 i = 0; i < m_count; i ++) {
			m_indices[i] = indexList[i];
		}
		Update(me);
	}

	void Update (dgCollisionDeformableSolidMesh* const me)
	{
		m_com = dgVector (dgFloat32 (0.0f));
		m_com0 = dgVector (dgFloat32 (0.0f));
		m_AqqInv = dgGetIdentityMatrix();
		m_rotSeed = dgGetIdentityMatrix();

		
		const dgVector* const shapePosit = me->m_shapePosition;
		const dgFloat32* const unitMass = me->m_particles.m_unitMass;

		m_totalMass = dgFloat32 (0.0f);
		dgVector accSum (dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			//dgFloat32 mass = particles.m_mass[index];
			dgFloat32 mass = unitMass[index];
			m_totalMass += mass;
			accSum += shapePosit[index].Scale4 (mass);
		}

		if (m_totalMass > dgFloat32 (0.0f)) {
			m_com0 = accSum.Scale4 (dgFloat32 (1.0f) / m_totalMass);

			dgMatrix sumQiQi (dgGetZeroMatrix());
			for (dgInt32 i = 0; i < m_count; i ++) {
				dgInt32 index = m_indices[i];

				dgVector qi (shapePosit[index] - m_com0);
				dgFloat32 mass = unitMass[index];
				for (dgInt32 j = 0; j < 3; j ++) {
					for (dgInt32 k = 0; k < 3; k ++) {
						sumQiQi[j][k] += mass * qi[j] * qi[k];
					}
				}
			}
			m_AqqInv = sumQiQi.Symetric3by3Inverse();
		}
	}

	void Copy (const dgDeformationRegion& src)
	{
		dgAssert (!m_indices);
		m_count = src.m_count;
		m_indices = (dgInt16*) dgMallocStack(sizeof (dgInt16) * m_count);
		memcpy (m_indices, src.m_indices, sizeof (dgInt16) * m_count);
		m_com0 = src.m_com0;
	}

	void CleanUP()
	{
		dgFree(m_indices);
	}


	void UpdateVelocities(const dgCollisionDeformableSolidMesh::dgParticle& particles, dgFloat32 timestep, dgFloat32 stiffness)
	{
/*
		const dgFloat32* const mass = particles.m_mass;
		const dgVector* const posit0 = particles.m_shapePosition;
		const dgVector* const posit1 = particles.m_position;

		dgVector com = dgVector (dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			com += posit1[index].Scale (mass[index]);
		}
		m_com = com.Scale (dgFloat32 (1.0f) / m_totalMass);

		dgMatrix sumQiPi (dgGetZeroMatrix());
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			dgVector pi (posit1[index] - m_com);
			dgVector qi (posit0[index] - m_com0);
			dgFloat32 massI = mass[index];
			for (dgInt32 j = 0; j < 3; j ++) {
				for (dgInt32 k = 0; k < 3; k ++) {
					sumQiPi[j][k] += massI * qi[j] * pi[k];
				}
			}
		}
		sumQiPi [3][3] = dgFloat32 (1.0f);

		dgMatrix S (sumQiPi * sumQiPi.Transpose4X4());
		dgVector eigenValues;

		S.EigenVectors (eigenValues, m_rotSeed);
		m_rotSeed = S;
#if 0
		#ifdef _DEBUG
			dgMatrix P0 (sumQiPi * sumQiPi.Transpose4X4());
			dgMatrix D (dgGetIdentityMatrix());
			D[0][0] = eigenValues[0];
			D[1][1] = eigenValues[1];
			D[2][2] = eigenValues[2];
			dgMatrix P1 (S.Transpose4X4() * D * S);

			dgMatrix xx (P1 * P0.Symetric3by3Inverse());
			dgAssert (dgAbsf (xx[0][0] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[1][1] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[2][2] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[0][1]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[0][2]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[1][0]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[1][2]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[2][0]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[2][1]) < dgFloat32 (1.0e-3f));
		#endif
#endif
		eigenValues.m_x = dgSqrt (eigenValues.m_x);
		eigenValues.m_y = dgSqrt (eigenValues.m_y);
		eigenValues.m_z = dgSqrt (eigenValues.m_z);

		dgMatrix m (S);
		m.m_front = m.m_front.Scale (eigenValues.m_x);
		m.m_up    = m.m_up.Scale (eigenValues.m_y);
		m.m_right = m.m_right.Scale (eigenValues.m_z);
		S = S.Transpose4X4() * m;
		m_rot = S.Symetric3by3Inverse() * sumQiPi;
		dgMatrix A (m_AqqInv * sumQiPi);

m_rot = dgGetIdentityMatrix();


		dgFloat32 invTimeScale = stiffness / timestep; 
		dgVector* const velocity = particles.m_internalVelocity;
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			dgVector qi (posit0[index] - m_com0);
			dgVector gi (m_rot.UnrotateVector(qi) + m_com);
			velocity[index] += (gi - posit1[index]).Scale3 (invTimeScale);
		}
*/
	}

	dgVector m_com;
	dgVector m_com0;
	dgMatrix m_rot; 
	dgMatrix m_AqqInv; 
	dgMatrix m_rotSeed; 
	
	dgFloat32 m_totalMass;
	dgInt32 m_count;
	dgInt16* m_indices;
	dgVector* m_targetPostions;
};

#if 0
class dgCollisionDeformableSolidMesh::dgDeformableNode
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

	void TriangleBox (const dgVector* const position, const dgInt16* const faceIndices, dgVector& minP, dgVector& maxP) const
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

	void CalculateBox (const dgVector* const position, const dgInt16* const faceIndices) 
	{
		dgVector p0;
		dgVector p1;
		TriangleBox (position, faceIndices, p0, p1);

		p0 = p0.CompProduct(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));
		p1 = p1.CompProduct(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));

		m_minBox.m_x = dgFloor (p0.m_x) * DG_DEFORMABLE_INV_PADDING; 
		m_minBox.m_y = dgFloor (p0.m_y) * DG_DEFORMABLE_INV_PADDING;  
		m_minBox.m_z = dgFloor (p0.m_z) * DG_DEFORMABLE_INV_PADDING;  

		m_maxBox.m_x = dgFloor (p1.m_x + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING;  
		m_maxBox.m_y = dgFloor (p1.m_y + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING;  
		m_maxBox.m_z = dgFloor (p1.m_z + dgFloat32 (1.0f)) * DG_DEFORMABLE_INV_PADDING;  

		dgVector side0 (m_maxBox - m_minBox);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		m_surfaceArea = side0 % side1;
	}

	dgInt32 UpdateBox (const dgVector* const position, const dgInt16* const faceIndices)
	{
		dgVector p0;
		dgVector p1;
		TriangleBox (position, faceIndices, p0, p1);

		p0 = p0.CompProduct(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));
		p1 = p1.CompProduct(dgVector (DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, DG_DEFORMABLE_PADDING, dgFloat32 (0.0f)));

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

int xxx;
	dgVector m_minBox;
	dgVector m_maxBox;
	dgInt32 m_indexStart;
	dgFloat32 m_surfaceArea;
	dgDeformableNode* m_left;
	dgDeformableNode* m_right;
	dgDeformableNode* m_parent;
};


void dgCollisionDeformableSolidMesh::SetSkinThickness (dgFloat32 skinThickness)
{
	m_skinThickness = dgAbsf (skinThickness);
	if (m_skinThickness < DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS) {
		m_skinThickness = DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS;
	}
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
}

void dgCollisionDeformableSolidMesh::SetStiffness (dgFloat32 stiffness)
{
	m_stiffness = dgAbsf (stiffness);
	if (m_stiffness < DG_DEFORMABLE_DEFAULT_STIFFNESS) {
		m_stiffness = DG_DEFORMABLE_DEFAULT_STIFFNESS;
	}
}

void dgCollisionDeformableSolidMesh::SetPlasticity (dgFloat32 plasticity)
{
	m_plasticity = dgAbsf (plasticity);
	if (m_plasticity < DG_DEFORMABLE_DEFAULT_PLASTICITY) {
		m_plasticity = DG_DEFORMABLE_DEFAULT_PLASTICITY;
	}
}


void dgCollisionDeformableSolidMesh::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (p0.m_x <= p1.m_x);
	dgAssert (p0.m_y <= p1.m_y);
	dgAssert (p0.m_z <= p1.m_z);

	m_boxSize = (p1 - p0).Scale (dgFloat32 (0.5f)); 
	m_boxOrigin = (p1 + p0).Scale (dgFloat32 (0.5f)); 
	m_boxOrigin-= m_particles.m_com;


	dgFloat32 padding = m_skinThickness + DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS;
	m_boxSize += dgVector (padding, padding, padding, dgFloat32 (0.0f));
}


/*
void dgCollisionDeformableSolidMesh::GetVertexListIndexList (const dgVector& p0, const dgVector& p1, dgGetVertexListIndexList &data) const
{
	dgAssert (0);

	if (m_faceInAabb) {
		return m_faceInAabb (m_userData, &p0[0], &p1[0], (const dgFloat32**) &data.m_veterxArray, &data.m_vertexCount, &data.m_vertexStrideInBytes,
							 data.m_indexList, data.m_maxIndexCount, data.m_userDataList);

	} else {
		data.m_triangleCount = 0;
	}
}
*/



dgFloat32 dgCollisionDeformableSolidMesh::RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
{
dgAssert (0);
return 0;
/*
	dgFloat32 t;
	dgFloat32 param;
	if (PREFILTER_RAYCAST (preFilter, body, this, userData)) {
		return dgFloat32 (1.2f);
	}

	param = dgFloat32 (1.2f);
	if (m_rayHitCallback) {
		dgCollisionMeshRayHitDesc data;
		data.m_localP0 = localP0;
		data.m_localP1 = localP1;
		data.m_userData = m_userData;
		data.m_altenateUserData = userData;
		if (body) {
			data.m_matrix = body->m_collisionWorldMatrix;
		}

		t = m_rayHitCallback (data);
		if ((t < dgFloat32 (1.0f)) && (t > dgFloat32 (0.0f))) {
			param = t;
			contactOut.m_normal = data.m_normal;
			contactOut.m_userId = data.m_userId;
		} 
	}
	return param;
*/
}



void dgCollisionDeformableSolidMesh::GetCollidingFaces (dgPolygonMeshDesc* const data) const
{
	dgAssert (0);
/*
	data->m_faceCount = 0;
	if (m_collideCallback) {
		data->m_me = this;
		data->m_userData = m_userData;
		m_collideCallback (*data);
	}
*/
}



dgFloat32 dgCollisionDeformableSolidMesh::CalculateSurfaceArea (const dgDeformableNode* const node0, const dgDeformableNode* const node1, dgVector& minBox, dgVector& maxBox) const
{
	minBox = dgVector (dgMin (node0->m_minBox.m_x, node1->m_minBox.m_x), dgMin (node0->m_minBox.m_y, node1->m_minBox.m_y), dgMin (node0->m_minBox.m_z, node1->m_minBox.m_z), dgFloat32 (0.0f));
	maxBox = dgVector (dgMax (node0->m_maxBox.m_x, node1->m_maxBox.m_x), dgMax (node0->m_maxBox.m_y, node1->m_maxBox.m_y), dgMax (node0->m_maxBox.m_z, node1->m_maxBox.m_z), dgFloat32 (0.0f));		
	dgVector side0 (maxBox - minBox);
	dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
	return side0 % side1;
}


dgCollisionDeformableSolidMesh::dgDeformableNode* dgCollisionDeformableSolidMesh::BuildTopDown (dgInt32 count, dgDeformableNode* const children, dgDeformableNode* const parent)
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
			dgVector p ((node->m_minBox + node->m_maxBox).Scale (0.5f));
			median += p;
			varian += p.CompProduct (p);
		}

		varian = varian.Scale (dgFloat32 (count)) - median.CompProduct(median);

		dgInt32 index = 0;
		dgFloat32 maxVarian = dgFloat32 (-1.0e10f);
		for (dgInt32 i = 0; i < 3; i ++) {
			if (varian[i] > maxVarian) {
				index = i;
				maxVarian = varian[i];
			}
		}

		dgVector center = median.Scale (dgFloat32 (1.0f) / dgFloat32 (count));
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

bool dgCollisionDeformableSolidMesh::SanityCheck () const
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

void dgCollisionDeformableSolidMesh::ImproveNodeFitness (dgDeformableNode* const node)
{
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (!node->m_parent) {
		node->m_surfaceArea = CalculateSurfaceArea (node->m_left, node->m_right, node->m_minBox, node->m_maxBox);
	} else {
		if (node->m_parent->m_left == node) {
			//dgFloat32 cost0 = node->m_surfaceArea;
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
			//dgFloat32 cost0 = node->m_surfaceArea;
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



void dgCollisionDeformableSolidMesh::ImproveTotalFitness()
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



void dgCollisionDeformableSolidMesh::DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const
{
dgAssert (0);
/*
	dgMatrix matrix (GetLocalMatrix() * matrixPtr);
	for (dgInt32 i = 0; i < m_trianglesCount; i ++ ) {
		dgTriplex points[3];
		for (dgInt32 j = 0; j < 3; j ++) {
			dgInt32 index = m_indexList[i * 3 + j];
			dgVector p (matrix.TransformVector(m_particles.m_position[index]));
			points[j].m_x = p.m_x;
			points[j].m_y = p.m_y;
			points[j].m_z = p.m_z;
		}
		callback (userData, 3, &points[0].m_x, 0);
	}
*/
}


void dgCollisionDeformableSolidMesh::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);
	
	dgCollisionInfo::dgDeformableMeshData& data = info->m_deformableMesh;
	data.m_vertexCount = m_particles.m_count;
	data.m_vertexStrideInBytes = sizeof (dgVector);
	data.m_triangleCount = m_trianglesCount;
	data.m_indexList = (dgUnsigned16*) m_indexList;
	data.m_vertexList = &m_particles.m_shapePosition->m_x;
}

/*
void dgCollisionDeformableSolidMesh::CalculateInertia (dgVector& inertiaOut, dgVector& originOut) const
{
	dgVector sum (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector sum2 (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		sum += m_particles.m_shapePosition[i];
		sum2 += m_particles.m_shapePosition[i].CompProduct(m_particles.m_shapePosition[i]);
	}

	originOut = sum.Scale (dgFloat32 (1.0f)/m_particles.m_count);
	inertiaOut = sum2.Scale (dgFloat32 (1.0f)/m_particles.m_count) - originOut.CompProduct(originOut);
}
*/




void dgCollisionDeformableSolidMesh::SetParticlesMasses (dgFloat32 totalMass)
{
	if (totalMass < DG_INFINITE_MASS) {
		dgFloat32 mass = totalMass / m_particles.m_count;
		dgFloat32 invMass = dgFloat32 (1.0f) / mass;

		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			m_particles.m_mass[i] = mass;
			m_particles.m_unitMass[i] = invMass;
		}

		for (dgInt32 i = 0; i < m_regionsCount; i ++) {
			m_regions[i].Update(m_particles);
		}
	}
}

void dgCollisionDeformableSolidMesh::SetParticlesVelocities (const dgVector& velocity)
{
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_particles.m_instantVelocity[i] = velocity;
	}
}


void dgCollisionDeformableSolidMesh::UpdateCollision ()
{
	// reorganize the collision structure
	dgVector* const positions = m_particles.m_position;
	bool update = false;
	for (dgInt32 i = 0; i < m_trianglesCount; i ++) {
		dgDeformableNode* const node = &m_nodesMemory[i];

		if (node->UpdateBox (positions, &m_indexList[node->m_indexStart])) {
			for (dgDeformableNode* parent = node->m_parent; parent; parent = parent->m_parent) {
				dgVector minBox;
				dgVector maxBox;
				dgFloat32 area = CalculateSurfaceArea (parent->m_left, parent->m_right, minBox, maxBox);
				if (!dgCompareBox (minBox, maxBox, parent->m_minBox, parent->m_maxBox)) {
					break;
				}
				update = true;
				parent->m_minBox = minBox;
				parent->m_maxBox = maxBox;
				parent->m_surfaceArea = area;
			}
		}
	}
	if (update) {
		ImproveTotalFitness	();
	}
}


void dgCollisionDeformableSolidMesh::SetMatrix (const dgMatrix& matrix)
{
	dgVector* const positions = m_particles.m_position;
	dgVector* const deltaPositions = m_particles.m_deltaPosition;

	dgMatrix matrix1 (matrix);
	matrix1.m_posit -= matrix1.RotateVector (m_particles.m_com);

	m_particles.m_com = matrix.m_posit;

	matrix1.TransformTriplex(&positions[0].m_x, sizeof (dgVector), &positions[0].m_x, sizeof (dgVector), m_particles.m_count);
	memset (deltaPositions, 0, sizeof (dgVector) * m_particles.m_count);

	UpdateCollision ();
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
}



dgInt32 dgCollisionDeformableSolidMesh::CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy)
{
dgAssert (0);
return 0;
/*

	if (m_rootNode) {
		dgAssert (IsType (dgCollision::dgCollisionDeformableSolidMesh_RTTI));

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




void dgCollisionDeformableSolidMesh::CalculateContactsToCollisionTree (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy)
{
	dgAssert (0);
	/*
	dgBody* const myBody = pair->m_body0;
	dgBody* const treeBody = pair->m_body1;

	dgAssert (pair->m_body0->m_collision == this);
	dgCollisionBVH* const treeCollision = (dgCollisionBVH*)treeBody->m_collision;

	dgMatrix matrix (myBody->m_collisionWorldMatrix * treeBody->m_collisionWorldMatrix.Inverse());
	dgPolygonMeshDesc data;
	CalcAABB (matrix, data.m_boxP0, data.m_boxP1);

	data.m_vertex = NULL;
	data.m_threadNumber = proxy.m_threadIndex;
	data.m_faceCount = 0;
	data.m_faceIndexCount = 0;
	data.m_vertexStrideInBytes = 0;

	data.m_faceMaxSize = NULL;
	data.m_userAttribute = NULL;
	data.m_faceVertexIndex = NULL;
	data.m_faceNormalIndex = NULL;
	data.m_faceAdjencentEdgeNormal = NULL;
	data.m_userData = treeCollision->GetUserData();
	data.m_objCollision = collision;
	data.m_objBody = myBody;
	data.m_polySoupBody = treeBody;


static int xxx;
xxx ++;

	treeCollision->GetCollidingFaces (&data);
	if (data.m_faceCount) {
		//dgFloat32* const faceSize = data.m_faceMaxSize; 
		//dgInt32* const idArray = (dgInt32*)data.m_userAttribute; 


		dgFloat32 timestep = proxy.m_timestep;
		//dgFloat32 invTimeStep = dgFloat32 (1.0f) / timestep;
		//dgVector externImpulseDistance ((myBody->m_accel.Scale (myBody->m_invMass.m_w * timestep) + myBody->m_veloc).Scale (timestep));
		//dgFloat32 gravityDistance = dgAbsf (dgSqrt (externImpulseDistance % externImpulseDistance));

		dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;
		dgInt32 thread = data.m_threadNumber;
		dgCollisionMesh::dgCollisionConvexPolygon* const polygon = treeCollision->m_polygon[thread];

		polygon->m_vertex = data.m_vertex;
		polygon->m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

//		dgMatrix matInv (matrix.Inverse());
		const dgMatrix& worldMatrix = treeBody->m_collisionWorldMatrix;

		dgInt32 indexCount = 0;
		for (dgInt32 i = 0; i < data.m_faceCount; i ++) {

//dgTrace (("%d ", xxx));
//dgTrace (("(%f %f %f) ", m_particles.m_position1[0].m_y, m_particles.m_instantVelocity[0].m_y, m_particles.m_internalVelocity[0].m_y));
//dgTrace (("(%f %f %f) ", m_particles.m_position1[4].m_y, m_particles.m_instantVelocity[4].m_y, m_particles.m_internalVelocity[4].m_y));
//dgTrace (("\n"));

			polygon->SetFaceVertex (data.m_faceIndexCount[i], &indexArray[indexCount]);
			dgPlane plane (polygon->m_normal, -(polygon->m_normal % polygon->m_localPoly[0]));
			plane.m_w -= m_skinThickness;
			plane = worldMatrix.TransformPlane(plane);
			worldMatrix.TransformTriplex (&polygon->m_localPoly[0].m_x, sizeof (polygon->m_localPoly[0]), &polygon->m_localPoly[0].m_x, sizeof (polygon->m_localPoly[0]), polygon->m_count); 

			dgPlane scaledPlane(plane);
			scaledPlane.m_w *= dgFloat32 (2.0f);
			dgVector planeSupport (dgAbsf(scaledPlane.m_x), dgAbsf(scaledPlane.m_y), dgAbsf(scaledPlane.m_z), dgAbsf(0.0f));

			dgDeformableNode* stackPool[DG_DEFORMABLE_STACK_DEPTH];
#ifdef _DEBUG
			dgList<dgDeformableNode*> collidingList (GetAllocator());
			for (dgInt32 j = 0; j < m_trianglesCount; j ++) {
				dgDeformableNode* const node = &m_nodesMemory[j];
				dgVector size (node->m_maxBox - node->m_minBox) ;
				dgVector origin (node->m_maxBox + node->m_minBox);
				dgFloat32 support = planeSupport % size;
				dgFloat32 dist = scaledPlane.Evalue(origin);
				dgFloat32 maxDist = dist + support;
				dgFloat32 minDist = dist - support;
				if (minDist * maxDist <= dgFloat32 (0.0f)) {
					collidingList.Append(node);
				}
			}
#endif


			dgInt32 stack = 1;
			stackPool[0] = m_rootNode;
			while (stack) {
				stack --;
				dgDeformableNode* const node = stackPool[stack];

				dgVector size (node->m_maxBox - node->m_minBox) ;
				dgVector origin (node->m_maxBox + node->m_minBox);
				dgFloat32 support = planeSupport % size;
				dgFloat32 dist = scaledPlane.Evalue(origin);
				dgFloat32 maxDist = dist + support;
				dgFloat32 minDist = dist - support;
				maxDist = 1;
				minDist = -1;
				if ((maxDist * minDist) <= dgFloat32 (0.0f)) {
					if(node->m_indexStart >= 0) {
#ifdef _DEBUG	
						collidingList.Remove(node);
#endif
						CalculatePolygonContacts (node, plane, polygon, timestep);
					}

					if (node->m_left) {
						stackPool[stack] = node->m_left;
						stack ++;
					}
					if (node->m_right) {
						stackPool[stack] = node->m_right;
						stack ++;
					}
				}
			}
			indexCount += data.m_faceIndexCount[i];

#ifdef _DEBUG	
			if (collidingList.GetCount()) {
				dgAssert (0);
			}
#endif		
		}
	}
*/
}


void dgCollisionDeformableSolidMesh::CalculatePolygonContacts (dgDeformableNode* const node, const dgPlane& plane, dgCollisionConvexPolygon* const polygonShape, dgFloat32 timestep)
{
static int xxx;
xxx ++;

	dgInt32 polyVertCount = polygonShape->m_count;
	dgVector* const positions = m_particles.m_position;
	dgVector* const velocity = m_particles.m_instantVelocity;
	dgVector* const deltaPosition = m_particles.m_deltaPosition;
	const dgVector* const polygon = polygonShape->m_localPoly;

	if (plane.m_y < 0.0f)
		xxx *=1;

	for (dgInt32 i = 0; i < 3; i ++) {
		dgInt32 index = m_indexList[node->m_indexStart + i];
		dgVector& p0 = positions[index];
		dgFloat32 side0 = plane.Evalue(p0);
		if (side0 <= dgFloat32 (0.0f)) {
			const dgVector& delta = deltaPosition[index];
			dgVector p1 (p0 - delta);
			dgFloat32 side1 = plane.Evalue(p1);
			if (side1 >= dgFloat32 (0.0f)) {

				bool inside = true;
				dgInt32 j0 = polyVertCount - 1;
				for(dgInt32 j1 = 0; j1 < polyVertCount; j1 ++) {
					dgVector e0 (polygon[j1] - polygon[j0]);
					dgVector e1 (p0 - polygon[j0]);
					dgVector e2 (p1 - polygon[j0]);
					dgFloat32 volume = (e0 * e1) % e2;
					if (volume < dgFloat32 (0.0f)) {
						inside = false;
						break;
					}
					j0 = j1;
				}

				if (inside) {
					dgFloat32 den =  delta % plane;
					dgAssert (dgAbsf(den) > dgFloat32 (1.0e-6f));
					p0 -= delta.Scale (dgFloat32 (1.001f) * side0 / den);

//					dgFloat32 reflexVeloc = veloc % plane;
//					dgVector bounceVeloc (plane.Scale (reflexVeloc));
//					dgVector tangentVeloc (veloc - bounceVeloc);
//					float restitution = dgFloat32 (0.0f);
//					veloc = veloc - bounceVeloc.Scale (dgFloat32 (1.0f) + restitution) ;
					velocity[index] = dgVector (dgFloat32 (0.0f));
					//float keneticFriction = dgFloat32 (0.5f);
				}
			}
		}
	}
}


#endif





						   
dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionDeformableMesh (world, deserialization, userData)
//	,m_particles (world, deserialization, userData) 
//	,m_trianglesCount(0)
//	,m_stiffness(DG_DEFORMABLE_DEFAULT_STIFFNESS)
//	,m_plasticity(DG_DEFORMABLE_DEFAULT_PLASTICITY)
//	,m_skinThickness(DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS)
//	,m_indexList(NULL)
//	,m_faceNormals(NULL)
//	,m_rootNode(NULL)
//	,m_nodesMemory(NULL)
//	,m_visualSegments(world->GetAllocator())
{
	dgAssert (0);
/*
	m_rtti |= dgCollisionDeformableSolidMesh_RTTI;
	dgAABBPolygonSoup::Deserialize (deserialization, userData);

	dgVector p0; 
	dgVector p1; 
	GetAABB (p0, p1);
	SetCollisionBBox(p0, p1);
*/
}


dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgMeshEffect* const mesh)
	:dgCollisionDeformableMesh (world, mesh, m_deformableSolidMesh)
	,m_shapePosition(NULL)
	,m_regions(NULL)
	,m_regionsCount(0)
//	,m_trianglesCount(0)
//	,m_nodesCount(0)
//	,m_stiffness(DG_DEFORMABLE_DEFAULT_STIFFNESS)
//	,m_plasticity(DG_DEFORMABLE_DEFAULT_PLASTICITY)
//	,m_skinThickness(DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS)
//	,m_indexList(NULL)
//	,m_faceNormals(NULL)
//	,m_rootNode(NULL)
//	,m_nodesMemory(NULL)
//	,
//	,m_visualVertexCount(0)
//	,m_visualVertexData(NULL)
//	,m_visualSegments(allocator)
{
	m_rtti |= dgCollisionDeformableSolidMesh_RTTI;
	mesh->Triangulate();

	m_regionsCount = 1;
//	m_trianglesCount = mesh->GetTotalFaceCount (); 
//	m_nodesMemory = (dgDeformableNode*) dgMallocStack(sizeof (dgDeformableNode) * (m_trianglesCount * 2 - 1));
//	m_indexList = (dgInt16*) dgMallocStack (sizeof (dgInt16) * m_trianglesCount * 3);
//	m_faceNormals = (dgVector*) dgMallocStack (sizeof (dgVector) * m_trianglesCount);
	m_shapePosition = (dgVector*) dgMallocStack (sizeof (dgVector) * m_particles.m_count);
	m_regions = (dgDeformationRegion*) dgMallocStack (sizeof (dgDeformationRegion) * m_regionsCount);
	memset (m_regions, 0, sizeof (dgDeformationRegion) * m_regionsCount);

//	dgInt32 stride = mesh->GetVertexStrideInByte() / sizeof (dgFloat64);  
//	dgFloat64* const vertex = mesh->GetVertexPool();  
//	dgVector delta (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
//	dgBigVector com (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_shapePosition[i] = m_particles.m_posit[i];
//		m_particles.m_position[i] = m_particles.m_shapePosition[i];
//		m_particles.m_deltaPosition[i] = delta;
//		if (m_particles.m_position[i].m_y < 0.0f){
//			//m_particles.m_position1[i].m_y = 0.0f;
//		}
//		com += dgBigVector (m_particles.m_position[i]);
	}
//	com = com.Scale(dgFloat32 (1.0f / m_particles.m_count));
//	m_particles.m_com = com;
	CreateRegions();
/*
	dgInt32 indexCount = mesh->GetTotalIndexCount (); 
	//	dgInt32* const faceArray = (dgInt32*) dgMallocStack(m_trianglesCount * sizeof (dgInt32));
	//	dgInt32* const materialIndexArray = (dgInt32*) dgMallocStack(m_trianglesCount * sizeof (dgInt32));
	//	void** const indexArray = (void**) dgMallocStack(indexCount * sizeof (void*));
	dgStack<dgInt32> faceArray (m_trianglesCount);
	dgStack<dgInt32> materials (m_trianglesCount);
	dgStack<void*>indexArray (indexCount);
	mesh->GetFaces (&faceArray[0], &materials[0], &indexArray[0]);

	for (dgInt32 i = 0; i < m_trianglesCount; i ++) {
		dgInt32 count = faceArray[i];
		dgAssert (count == 3);
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 k = mesh->GetVertexIndex(indexArray[i * 3 + j]);
			m_indexList[i * 3 + j] = dgInt16 (k);
		}
		dgDeformableNode& node = m_nodesMemory[i];
		node.m_left = NULL;
		node.m_right = NULL;
		node.m_parent = NULL;
		node.m_indexStart = i * 3;
		node.CalculateBox(m_particles.m_position, &m_indexList[i * 3]);
	}

	m_nodesCount = m_trianglesCount;
	m_rootNode = BuildTopDown (m_nodesCount, m_nodesMemory, NULL);

	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		m_nodesMemory[i].xxx = i;
	}


	ImproveTotalFitness();
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
*/
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh (const dgCollisionDeformableSolidMesh& source)
	:dgCollisionDeformableMesh (source)
//	,m_particles (source.m_particles)
	,m_regionsCount(source.m_regionsCount)
//	,m_trianglesCount(source.m_trianglesCount)
//	,m_nodesCount(source.m_nodesCount)
//	,m_stiffness(source.m_stiffness)
//	,m_plasticity(source.m_plasticity)
//	,m_skinThickness(source.m_skinThickness)
//	,m_visualVertexCount(source.m_visualVertexCount)
//	,m_visualSegments (source.GetAllocator())
{
	m_rtti |= dgCollisionCompoundBreakable_RTTI;

//	m_indexList = (dgInt16*) dgMallocStack (sizeof (dgInt16) * m_trianglesCount * 3);
//	m_faceNormals = (dgVector*) dgMallocStack (sizeof (dgVector) * m_trianglesCount);
//	m_nodesMemory = (dgDeformableNode*) dgMallocStack(sizeof (dgDeformableNode) * m_nodesCount);
//	m_regions = (dgDeformationRegion*) dgMallocStack (sizeof (dgDeformationRegion) * m_regionsCount);

	m_shapePosition = (dgVector*) dgMallocStack (sizeof (dgVector) * m_particles.m_count);
	m_regions = (dgDeformationRegion*) dgMallocStack (sizeof (dgDeformationRegion) * m_regionsCount);
	memset (m_regions, 0, sizeof (dgDeformationRegion) * m_regionsCount);

	memcpy (m_shapePosition, source.m_shapePosition, sizeof (dgVector) * m_particles.m_count);
//	memcpy (m_indexList, source.m_indexList, sizeof (dgInt16) * m_trianglesCount * 3);
//	memcpy (m_faceNormals, source.m_faceNormals, sizeof (dgVector) * m_trianglesCount);
//	memcpy (m_nodesMemory, source.m_nodesMemory, sizeof (dgDeformableNode) * m_nodesCount);
	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		m_regions[i].Copy (source.m_regions[i]);
	}

/*
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

	m_visualVertexData = (dgVisualVertexData*) dgMallocStack(sizeof (dgVisualVertexData) * m_visualVertexCount);
	memcpy (m_visualVertexData, source.m_visualVertexData, sizeof (dgVisualVertexData) * m_visualVertexCount);

	for (dgList<dgMeshSegment>::dgListNode* node = source.m_visualSegments.GetFirst(); node; node = node->GetNext() ) {
		dgMeshSegment& srcSegment = node->GetInfo();
		dgMeshSegment& segment = m_visualSegments.Append()->GetInfo();
		segment.m_material = srcSegment.m_material;
		segment.m_indexCount = srcSegment.m_indexCount;
		segment.m_indexList = (dgInt16*) dgMallocStack(sizeof (dgInt16) * segment.m_indexCount);
		memcpy (segment.m_indexList, srcSegment.m_indexList, sizeof (dgInt16) * segment.m_indexCount);
	}
*/
}


dgCollisionDeformableSolidMesh::~dgCollisionDeformableSolidMesh(void)
{
	if (m_shapePosition) {
		dgFree (m_shapePosition);
	}

	if (m_regions) {
		for (dgInt32 i = 0; i < m_regionsCount; i ++) {
			m_regions[i].CleanUP();
		}
		dgFree (m_regions);
	}
}


void dgCollisionDeformableSolidMesh::CreateRegions()
{
	// for now only one region
	dgStack<dgInt16> indexList(m_particles.m_count);
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		indexList[i] = dgInt16(i);
	}

	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		m_regions[i].Init(this, m_particles.m_count, &indexList[0]);
	}
}




void dgCollisionDeformableSolidMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
/*
	SerializeLow(callback, userData);
	dgAABBPolygonSoup::Serialize ((dgSerialize) callback, userData);
*/
}

dgInt32 dgCollisionDeformableSolidMesh::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}





void dgCollisionDeformableSolidMesh::EndConfiguration ()
{
	dgAssert(0);
}

void dgCollisionDeformableSolidMesh::ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body)
{
	dgAssert(0);
}




//void dgCollisionDeformableSolidMesh::ApplyExternalAndInternalForces (dgDeformableBody* const myBody, dgFloat32 timestep, dgInt32 threadIndex)
void dgCollisionDeformableSolidMesh::IntegrateParticles (dgFloat32 timestep)
{
	dgAssert (m_myBody);
/*
	dgVector* const posit = m_particles.m_posit;
	dgVector* const veloc = m_particles.m_veloc;
	dgFloat32* const unitMass = m_particles.m_unitMass;

	dgVector gravityStep (gravity.Scale4 (timestep));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		veloc[i] = (posit[i] - m_shapePosition[i]).Scale4 (invTimeStep);
		m_shapePosition[i] = posit[i];
		veloc[i] += gravityStep.Scale4 (unitMass[i]);
		posit[i] += veloc[i].Scale4 (timestep);
	}
*/
	dgBody* const body = GetBody();
	dgFloat32 invMass = body->GetInvMass().m_w;
	dgVector velocyStep (body->GetForce().Scale4(invMass * timestep));

	dgVector* const veloc = m_particles.m_veloc;
	dgFloat32* const unitMass = m_particles.m_unitMass;
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		veloc[i] += velocyStep.Scale4 (unitMass[i]);
	}


}

void dgCollisionDeformableSolidMesh::ResolvePositionsConstraints (dgFloat32 timestep)
{
	dgAssert (m_myBody);
	dgBody* const body = GetBody();
/*
	// force are applied immediately to each particle
	dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector* const positions = m_particles.m_position;
	dgVector* const deltaPositions = m_particles.m_deltaPosition;
	dgVector* const instantVelocity = m_particles.m_instantVelocity;
	dgVector* const internalVelocity = m_particles.m_internalVelocity;

	// integrate particles external forces and current velocity
	dgVector extenalVelocityImpulse (myBody->m_accel.Scale3 (myBody->m_invMass.m_w * timestep));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		internalVelocity[i] = zero;
		instantVelocity[i] += extenalVelocityImpulse;
		deltaPositions[i] = instantVelocity[i].Scale3 (timestep);
		positions[i] += deltaPositions[i];
	}

	// apply particle velocity contribution by each particle regions
	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		m_regions[i].UpdateVelocities(m_particles, timestep, m_stiffness);
	}

	// integrate each particle by the deformation velocity, also calculate the new com
	dgFloat32 dampCoef = 0.0f;
	dgVector com (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		instantVelocity[i] += internalVelocity[i].Scale3 (dampCoef);
		dgVector step (internalVelocity[i].Scale3 (timestep));
		deltaPositions[i] += step;
		positions[i] += step;
		com += positions[i];
	}

	// center the particles around the new geometrics center of mass
	dgVector oldCom (m_particles.m_com);
	m_particles.m_com = com.Scale (dgFloat32 (1.0f) / m_particles.m_count); 

	// calculate the new body average velocity
	myBody->m_veloc = (m_particles.m_com - oldCom).Scale (dgFloat32 (1.0f) / timestep);
	myBody->m_globalCentreOfMass = m_particles.m_com; 
	myBody->m_matrix.m_posit = m_particles.m_com; 

	//myBody->UpdateMatrix (timestep, threadIndex);
	if (myBody->m_matrixUpdate) {
		myBody->m_matrixUpdate (*myBody, myBody->m_matrix, threadIndex);
	}

	// the collision changed shape, need to update spatial structure 
	UpdateCollision ();

	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
*/
}

