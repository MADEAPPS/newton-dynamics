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
#include "dgCollisionDeformableClothPatch.h"


class dgCollisionDeformableClothPatch::dgClothLink
{
	public:
	dgFloat32 m_restLengh;
	dgInt16 m_particle_0;
	dgInt16 m_particle_1;
	dgInt16 m_materialIndex;
};

#if 0
void dgCollisionDeformableClothPatch::SetSkinThickness (dgFloat32 skinThickness)
{
	m_skinThickness = dgAbsf (skinThickness);
	if (m_skinThickness < DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS) {
		m_skinThickness = DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS;
	}
	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
}

void dgCollisionDeformableClothPatch::SetStiffness (dgFloat32 stiffness)
{
	m_stiffness = dgAbsf (stiffness);
	if (m_stiffness < DG_DEFORMABLE_DEFAULT_STIFFNESS) {
		m_stiffness = DG_DEFORMABLE_DEFAULT_STIFFNESS;
	}
}

void dgCollisionDeformableClothPatch::SetPlasticity (dgFloat32 plasticity)
{
	m_plasticity = dgAbsf (plasticity);
	if (m_plasticity < DG_DEFORMABLE_DEFAULT_PLASTICITY) {
		m_plasticity = DG_DEFORMABLE_DEFAULT_PLASTICITY;
	}
}


void dgCollisionDeformableClothPatch::DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const
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


void dgCollisionDeformableClothPatch::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);
	
	dgCollisionInfo::dgDeformableMeshData& data = info->m_deformableMesh;
	data.m_vertexCount = m_particles.m_count;
	data.m_vertexStrideInBytes = sizeof (dgVector);
	data.m_triangleCount = m_trianglesCount;
	data.m_indexList = (dgUnsigned16*) m_indexList;
	data.m_vertexList = &m_particles.m_shapePosition->m_x;
}


void dgCollisionDeformableClothPatch::SetParticlesMasses (dgFloat32 totalMass)
{
	if (totalMass < DG_INFINITE_MASS) {
		dgFloat32 mass = totalMass / m_particles.m_count;
		dgFloat32 invMass = dgFloat32 (1.0f) / mass;

		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			m_particles.m_mass[i] = mass;
			m_particles.m_invMass[i] = invMass;
		}

		for (dgInt32 i = 0; i < m_regionsCount; i ++) {
			m_regions[i].Update(m_particles);
		}
	}
}

void dgCollisionDeformableClothPatch::SetParticlesVelocities (const dgVector& velocity)
{
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_particles.m_instantVelocity[i] = velocity;
	}
}


void dgCollisionDeformableClothPatch::UpdateCollision ()
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


void dgCollisionDeformableClothPatch::SetMatrix (const dgMatrix& matrix)
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

void dgCollisionDeformableClothPatch::ApplyExternalAndInternalForces (dgDeformableBody* const myBody, dgFloat32 timestep, dgInt32 threadIndex)
{
//	sleep (100);

	// force are applied immediately to each particle
	dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector* const positions = m_particles.m_position;
	dgVector* const deltaPositions = m_particles.m_deltaPosition;
	dgVector* const instantVelocity = m_particles.m_instantVelocity;
	dgVector* const internalVelocity = m_particles.m_internalVelocity;

	// integrate particles external forces and current velocity
	dgVector extenalVelocityImpulse (myBody->m_accel.Scale (myBody->m_invMass.m_w * timestep));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		internalVelocity[i] = zero;
		instantVelocity[i] += extenalVelocityImpulse;
		deltaPositions[i] = instantVelocity[i].Scale (timestep);
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
		instantVelocity[i] += internalVelocity[i].Scale (dampCoef);
		dgVector step (internalVelocity[i].Scale (timestep));
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


//	if (xxxx >= 70)
//		xxxx *=1;
//	dgTrace (("%d ", xxxx));
//	dgVector xxx (myBody->m_collisionWorldMatrix.TransformVector(m_particles.m_position1[0]));
//	dgTrace (("(%f %f %f) ", xxx.m_y, m_particles.m_instantVelocity[0].m_y, m_particles.m_internalVelocity[0].m_y));
//	//dgTrace (("(%f %f %f) ", m_particles.m_position1[0].m_y, m_particles.m_instantVelocity[0].m_y, m_particles.m_internalVelocity[0].m_y));
//	//dgTrace (("(%f %f %f) ", m_particles.m_position1[4].m_y, m_particles.m_instantVelocity[4].m_y, m_particles.m_internalVelocity[4].m_y));
//	dgTrace (("\n"));

}


dgInt32 dgCollisionDeformableClothPatch::CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy, dgInt32 useSimd)
{
dgAssert (0);
return 0;
/*

	if (m_rootNode) {
		dgAssert (IsType (dgCollision::dgCollisionDeformableClothPatch_RTTI));

		if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
			CalculateContactsToCollisionTree (pair, proxy, useSimd);

//		if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionConvexShape_RTTI)) {
//			contactCount = CalculateContactsToSingle (pair, proxy, useSimd);
//		} else if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
//			contactCount = CalculateContactsToCompound (pair, proxy, useSimd);
//		} else if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionBVH_RTTI)) {
//			contactCount = CalculateContactsToCollisionTree (pair, proxy, useSimd);
//		} else if (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionHeightField_RTTI)) {
//			contactCount = CalculateContactsToHeightField (pair, proxy, useSimd);
//		} else {
//			dgAssert (pair->m_body1->m_collision->IsType (dgCollision::dgCollisionUserMesh_RTTI));
//			contactCount = CalculateContactsBruteForce (pair, proxy, useSimd);
//		}
		} else {
			dgAssert (0);
		}
	}
	pair->m_contactCount = 0;
	return 0;
*/
}




void dgCollisionDeformableClothPatch::CalculateContactsToCollisionTree (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy, dgInt32 useSimd)
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


void dgCollisionDeformableClothPatch::CalculatePolygonContacts (dgDeformableNode* const node, const dgPlane& plane, dgCollisionConvexPolygon* const polygonShape, dgFloat32 timestep)
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
					velocity[index] = dgVector (0.0f, 0.0f, 0.0f, 0.0f);
					//float keneticFriction = dgFloat32 (0.5f);
				}
			}
		}
	}
}
#endif



						   






void dgCollisionDeformableClothPatch::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
/*
	SerializeLow(callback, userData);
	dgAABBPolygonSoup::Serialize ((dgSerialize) callback, userData);
*/
}

dgInt32 dgCollisionDeformableClothPatch::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}

dgCollisionDeformableClothPatch::dgCollisionDeformableClothPatch (const dgCollisionDeformableClothPatch& source)
	:dgCollisionDeformableMesh (source)
	,m_linksCount (source.m_linksCount)
{
	m_rtti = source.m_rtti;
	m_isdoubleSided = source.m_isdoubleSided;

	memcpy (&m_materials, &source.m_materials, sizeof (source.m_materials));

	m_links = (dgClothLink*) dgMallocStack (m_linksCount * sizeof (dgClothLink));
	memcpy (m_links, source.m_links, m_linksCount * sizeof (dgClothLink));
}


dgCollisionDeformableClothPatch::dgCollisionDeformableClothPatch (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionDeformableMesh (world, deserialization, userData)
{
	dgAssert (0);
}


dgCollisionDeformableClothPatch::dgCollisionDeformableClothPatch(dgWorld* const world, dgMeshEffect* const mesh, const dgClothPatchMaterial& structuralMaterial, const dgClothPatchMaterial& bendMaterial)
	:dgCollisionDeformableMesh (world, mesh, m_deformableClothPatch)
	,m_linksCount (0)
	,m_links(NULL)
{
	m_isdoubleSided = true;
	m_rtti |= dgCollisionDeformableClothPatch_RTTI;

	m_materials[1] = bendMaterial;
	m_materials[0] = structuralMaterial;
	
	//create structural connectivity 
	dgPolyhedra conectivity (GetAllocator());
	conectivity.BeginFace();
	for (dgInt32 i = 0; i < m_trianglesCount; i ++) {
		conectivity.AddFace (m_indexList[i * 3 + 0], m_indexList[i * 3 + 1], m_indexList[i * 3 + 2]);
	}
	conectivity.EndFace();

	// add bending edges
	dgInt32 mark = conectivity.IncLRU();
	dgPolyhedra::Iterator iter (conectivity);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark < mark) {
			dgEdge* const twin = edge->m_twin;
			edge->m_mark = mark;
			twin->m_mark = mark;
			if ((edge->m_incidentFace > 0) && (twin->m_incidentFace > 0)) {
				dgEdge* const bendEdge = conectivity.AddHalfEdge (edge->m_prev->m_incidentVertex, twin->m_prev->m_incidentVertex);
				dgEdge* const bendTwin = conectivity.AddHalfEdge (twin->m_prev->m_incidentVertex, edge->m_prev->m_incidentVertex);
				dgAssert (bendEdge);
				dgAssert (bendTwin);

				bendEdge->m_mark = mark;
				bendTwin->m_mark = mark;

				bendEdge->m_twin = bendTwin;
				bendTwin->m_twin = bendEdge;
				
				bendEdge->m_userData = 1;
				bendTwin->m_userData = 1;
			}
		}
	}

	m_linksCount = conectivity.GetEdgeCount() / 2;
	m_links = (dgClothLink*) dgMallocStack (m_linksCount * sizeof (dgClothLink));

	dgInt32 index = 0;
	mark = conectivity.IncLRU();

	const dgVector* const posit = m_particles.m_posit;
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark < mark){
			dgEdge* const twin = edge->m_twin;
			edge->m_mark = mark;
			twin->m_mark = mark;

			dgVector dist (posit[edge->m_incidentVertex] - posit[twin->m_incidentVertex]);
			dgFloat32 dist2 = dist % dist;
			if (dist2 > 1.0e-5f) {
				dgClothLink* const link = &m_links[index];
				link->m_restLengh = dgSqrt (dist2);
				link->m_particle_0 = dgInt16 (edge->m_incidentVertex);
				link->m_particle_1 = dgInt16 (twin->m_incidentVertex);
				link->m_materialIndex = dgInt16 (edge->m_userData);
				index ++;
			}
		}
	}
}

dgCollisionDeformableClothPatch::~dgCollisionDeformableClothPatch(void)
{
	if (m_links) {
		dgFree (m_links);
	}
}



void dgCollisionDeformableClothPatch::CalculateInternalForces (dgFloat32 timestep)
{
	dgFloat32* const mass = m_particles.m_mass;
	dgFloat32* const invMass = m_particles.m_invMass;
	dgVector* const posit = m_particles.m_posit;
	dgVector* const veloc = m_particles.m_veloc;
	dgVector* const force = m_particles.m_force;

	// apply gravity force
	dgFloat32 massScale = 0.01f;
	dgVector gravity (0.0f, -9.8f, 0.0f, 0.0f);
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		force[i] = gravity.Scale (mass[i] * massScale);
	}

	// calculate inernal forces
	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		dgClothLink& link = m_links[i];

		const dgClothPatchMaterial& material = m_materials[link.m_materialIndex];
		dgInt32 index0 = link.m_particle_0;
		dgInt32 index1 = link.m_particle_1;

		dgVector relPosit (posit[index0] - posit[index1]);
		dgVector relVeloc (veloc[index0] - veloc[index1]);

		dgFloat32 dirMag2 = relPosit % relPosit;
		dgAssert (dirMag2 > dgFloat32 (0.0f));
		
		dgFloat32 invMag = dgRsqrt (dirMag2);
		dgFloat32 mag = dirMag2 * invMag;

		dgVector dir (relPosit.Scale (invMag));

		dgFloat32 x = mag - link.m_restLengh;
		dgFloat32 s = relVeloc % dir;

		dgFloat32 ksd = timestep * material.m_stiffness;
		dgFloat32 num = material.m_stiffness * x + material.m_damper * s + ksd * s;
		dgFloat32 den = dgFloat32 (1.0f) + timestep * material.m_damper + timestep * ksd;

		dgAssert (den > 0.0f);
		dgFloat32 accel = - num / den;
		force[index0] += dir.Scale (accel * invMass[index0]);
		force[index1] -= dir.Scale (accel * invMass[index1]);
	}
}


void dgCollisionDeformableClothPatch::IntegrateVelocities (dgFloat32 timestep)
{
	dgFloat32* const invMass = m_particles.m_invMass;
	dgVector* const posit = m_particles.m_posit;
	dgVector* const veloc = m_particles.m_veloc;
	dgVector* const force = m_particles.m_force;

dgFloat32 massScale = 1.0f / 0.01f;
	dgFloat32 invMassTime = massScale * timestep;
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		veloc[i] += force[i].Scale (invMass[i] * invMassTime);
		posit[i] += veloc[i].Scale (timestep);
		
	}
}
