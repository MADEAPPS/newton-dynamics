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
	dgFloat32 m_mass0_influence;
	dgFloat32 m_mass1_influence;
	dgInt16 m_particle_0;
	dgInt16 m_particle_1;
	dgInt16 m_materialIndex;
};

class dgCollisionDeformableClothPatch::dgSoftBodyEdge: public dgConvexSimplexEdge
{
	public:
	dgClothLink* m_link;
};


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
	,m_graphCount (source.m_graphCount)
{
	_ASSERTE (0);
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
	,m_graphCount (0)
	,m_links(NULL)
	,m_graph(NULL)
	,m_particleToEddgeMap(NULL)
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
//				dgEdge* const bendEdge = conectivity.AddHalfEdge (edge->m_prev->m_incidentVertex, twin->m_prev->m_incidentVertex);
//				dgEdge* const bendTwin = conectivity.AddHalfEdge (twin->m_prev->m_incidentVertex, edge->m_prev->m_incidentVertex);
//				dgAssert (bendEdge);
//				dgAssert (bendTwin);
//				bendEdge->m_twin = bendTwin;
//				bendTwin->m_twin = bendEdge;


				dgEdge* const bendEdge = conectivity.ConnectVertex (edge->m_prev, twin->m_prev);
				dgAssert (bendEdge);
				dgEdge* const bendTwin = bendEdge->m_twin;

				bendEdge->m_incidentFace = -1;
				bendTwin->m_incidentFace = -1;

				bendEdge->m_mark = mark;
				bendTwin->m_mark = mark;
			
				bendEdge->m_userData = 1;
				bendTwin->m_userData = 1;
			}
		}
	}


	dgInt32 index = 0;
	mark = conectivity.IncLRU();
	m_linksCount = conectivity.GetEdgeCount() / 2;
	m_links = (dgClothLink*) dgMallocStack (m_linksCount * sizeof (dgClothLink));

	const dgVector* const posit = m_particles.m_posit;
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark < mark){
			dgEdge* const twin = edge->m_twin;
			edge->m_mark = mark;
			twin->m_mark = mark;

			dgVector dist (posit[edge->m_incidentVertex] - posit[twin->m_incidentVertex]);
			dgFloat32 dist2 = dist % dist;
			if (dist2 > 1.0e-6f) {
				dgClothLink* const link = &m_links[index];
				link->m_restLengh = dgSqrt (dist2);
				link->m_particle_0 = dgInt16 (edge->m_incidentVertex);
				link->m_particle_1 = dgInt16 (twin->m_incidentVertex);
				link->m_mass0_influence = dgFloat32 (0.5f);
				link->m_mass1_influence = dgFloat32 (0.5f);
				link->m_materialIndex = dgInt16 (edge->m_userData);

				edge->m_userData = index;
				edge->m_twin->m_userData = index;
				index ++;
			} else {
				edge->m_userData = 0xffffffff;
				edge->m_twin->m_userData = 0xffffffff;
			}
		}
	}


	// enumerate the edges
	mark = conectivity.IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) {
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr->m_userData = (dgInt64 (m_graphCount) << 32) + ptr->m_userData;
				m_graphCount ++;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge) ;
		}
	} 

	m_graph = (dgSoftBodyEdge*) m_allocator->Malloc (m_graphCount * dgInt32 (sizeof (dgSoftBodyEdge)));
	m_particleToEddgeMap = (dgSoftBodyEdge**) m_allocator->Malloc (m_particles.m_count * dgInt32 (sizeof (dgSoftBodyEdge*)));
	memset (m_particleToEddgeMap, 0, m_particles.m_count * dgInt32 (sizeof (dgSoftBodyEdge*)));

	mark = conectivity.IncLRU();;
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) {
			dgEdge *ptr = edge;
			do {
				ptr->m_mark = mark;
				dgSoftBodyEdge* const simplexPtr = &m_graph[ptr->m_userData >> 32];

				simplexPtr->m_vertex = ptr->m_incidentVertex;
				simplexPtr->m_next = &m_graph[ptr->m_next->m_userData >> 32];
				simplexPtr->m_prev = &m_graph[ptr->m_prev->m_userData >> 32];
				simplexPtr->m_twin = &m_graph[ptr->m_twin->m_userData >> 32];

				dgInt32 linkIndex = ptr->m_userData & 0xffffffff;
				simplexPtr->m_link = (linkIndex >= 0) ? &m_links[linkIndex] : NULL;

				if (!m_particleToEddgeMap[simplexPtr->m_vertex] && (ptr->m_incidentFace > 0)) {
					m_particleToEddgeMap[simplexPtr->m_vertex] = simplexPtr;
				}

				ptr = ptr->m_twin->m_next;
			} while (ptr != edge) ;
		}
	} 
}

dgCollisionDeformableClothPatch::~dgCollisionDeformableClothPatch(void)
{
	if (m_links) {
		dgFree (m_links);
		dgFree (m_graph);
		dgFree (m_particleToEddgeMap);
	}
}

void dgCollisionDeformableClothPatch::ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body)
{
	m_particles.m_unitMass[particleIndex] = dgFloat32 (0.0f);
	m_particles.m_posit[particleIndex].m_x = posit.m_x;
	m_particles.m_posit[particleIndex].m_y = posit.m_y;
	m_particles.m_posit[particleIndex].m_z = posit.m_z;

	dgSoftBodyEdge* edge = m_particleToEddgeMap[particleIndex];
	do {
		dgClothLink* const link = edge->m_link;		
		if (link) {
			dgInt32 i0 = link->m_particle_0;
			dgInt32 i1 = link->m_particle_1;
			dgAssert ((i0 == particleIndex) || (i1 == particleIndex));

			if ((m_particles.m_unitMass[i0] == dgFloat32 (0.0f)) && (m_particles.m_unitMass[i1] == dgFloat32 (0.0f))) {
				link->m_mass0_influence = dgFloat32 (0.0f);
				link->m_mass1_influence = dgFloat32 (0.0f);
			} else if (m_particles.m_unitMass[i0] == dgFloat32 (0.0f)){
				link->m_mass0_influence = dgFloat32 (0.0f);
				link->m_mass1_influence = dgFloat32 (1.0f);
			} else {
				link->m_mass0_influence = dgFloat32 (1.0f);
				link->m_mass1_influence = dgFloat32 (0.0f);
			}
		}
		edge = (dgSoftBodyEdge*)edge->m_twin->m_next;
	} while (edge != m_particleToEddgeMap[particleIndex]);

}


void dgCollisionDeformableClothPatch::CalculateInternalForces (dgFloat32 timestep)
{
	dgVector* const posit = m_particles.m_posit;
	dgVector* const veloc = m_particles.m_veloc;
	dgVector* const force = m_particles.m_force;
	dgFloat32* const unitMass = m_particles.m_unitMass;

//timestep /= 4.0f;
	// apply gravity force
	dgFloat32 particleMass = 0.01f;
	dgVector gravity (0.0f, -9.8f, 0.0f, 0.0f);
	//dgVector gravity (0.0f, -1.0f, 0.0f, 0.0f);
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		force[i] = gravity.Scale (unitMass[i] * particleMass);
	}

//static int xxx;
//dgTrace (("%d\n", xxx));
//xxx ++;
//Sleep (100);
//if (xxx >50)
//xxx *=1;

	// calculate internal forces
	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		dgClothLink& link = m_links[i];

		const dgClothPatchMaterial& material = m_materials[link.m_materialIndex];
		dgInt32 index0 = link.m_particle_0;
		dgInt32 index1 = link.m_particle_1;

		dgVector relPosit (posit[index1] - posit[index0]);
		dgVector relVeloc (veloc[index1] - veloc[index0]);

		dgFloat32 dirMag2 = relPosit % relPosit;
		dgAssert (dirMag2 > dgFloat32 (0.0f));
		
		dgFloat32 invMag = dgRsqrt (dirMag2);
		dgVector dir (relPosit.Scale (invMag));

		dgFloat32 mag = dirMag2 * invMag;
		dgFloat32 x = mag - link.m_restLengh;
		dgFloat32 s = relVeloc % dir;
		dgFloat32 ksd = timestep * material.m_stiffness;
		dgFloat32 num = material.m_stiffness * x + material.m_damper * s + ksd * s;
		dgFloat32 den = dgFloat32 (1.0f) + timestep * material.m_damper + timestep * ksd;
		dgAssert (den > 0.0f);
		dgFloat32 linkForce = particleMass * num / den;

		force[index0] += dir.Scale (linkForce * unitMass[index0]);
		force[index1] -= dir.Scale (linkForce * unitMass[index1]);
	}
}



void dgCollisionDeformableClothPatch::IntegrateVelocities (dgFloat32 timestep)
{
//return;
	
//timestep /= 4.0f;
	dgVector* const posit = m_particles.m_posit;
	dgVector* const veloc = m_particles.m_veloc;
	dgVector* const force = m_particles.m_force;
//	dgFloat32* const unitMass = m_particles.m_unitMass;

dgFloat32 invMassScale = 1.0f / 0.01f;
	dgFloat32 invMassTime = invMassScale * timestep;

	// later change this to a verlet integration
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		veloc[i] += force[i].Scale (invMassTime);
		posit[i] += veloc[i].Scale (timestep);
	}

}
