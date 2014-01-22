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

class dgCollisionDeformableClothPatch::dgEdgePair
{
	public:
	dgEdge* m_edge;
	dgEdge* m_twin;
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
	,m_posit(NULL)
	,m_linkOrder(NULL)
	,m_links(NULL)
	,m_graph(NULL)
	,m_particleToEddgeMap(NULL)
{
	m_rtti = source.m_rtti;
	m_isdoubleSided = source.m_isdoubleSided;

	memcpy (&m_materials, &source.m_materials, sizeof (source.m_materials));

	m_linkOrder = (dgInt32*) dgMallocStack (m_linksCount * sizeof (dgInt32));
	memcpy (m_linkOrder, source.m_linkOrder, m_linksCount * sizeof (dgInt32));

	m_links = (dgClothLink*) dgMallocStack (m_linksCount * sizeof (dgClothLink));
	memcpy (m_links, source.m_links, m_linksCount * sizeof (dgClothLink));

	m_graph = (dgSoftBodyEdge*) m_allocator->Malloc (m_graphCount * dgInt32 (sizeof (dgSoftBodyEdge)));
	for (dgInt32 i = 0; i < m_graphCount; i ++) {
		dgSoftBodyEdge* const dst = &m_graph[i];
		dgSoftBodyEdge* const src = &source.m_graph[i];
		
		dst->m_vertex = src->m_vertex;
		dst->m_twin = &m_graph[(dgSoftBodyEdge*)src->m_twin - source.m_graph];
		dst->m_next = &m_graph[(dgSoftBodyEdge*)src->m_next - source.m_graph];
		dst->m_prev = &m_graph[(dgSoftBodyEdge*)src->m_prev - source.m_graph];
		dst->m_link = &m_links[src->m_link - source.m_links];
	}

	m_posit = (dgVector*) dgMallocStack (m_particles.m_count * sizeof (dgVector));
	memcpy (m_posit, source.m_posit, m_particles.m_count * sizeof (dgVector));

	m_particleToEddgeMap = (dgSoftBodyEdge**) m_allocator->Malloc (m_particles.m_count * dgInt32 (sizeof (dgSoftBodyEdge*)));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_particleToEddgeMap[i] = &m_graph[source.m_graph - source.m_particleToEddgeMap[i]];
	}
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
	,m_posit(NULL)
	,m_linkOrder(NULL)
	,m_links(NULL)
	,m_graph(NULL)
	,m_particleToEddgeMap(NULL)
{
	m_isdoubleSided = true;
	m_rtti |= dgCollisionDeformableClothPatch_RTTI;

	m_materials[1] = bendMaterial;
	m_materials[0] = structuralMaterial;

	m_posit = (dgVector*) dgMallocStack (m_particles.m_count * sizeof (dgVector));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_posit[i] = m_particles.m_posit[i];
	}
	
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

	dgList<dgEdgePair> bendPairs (GetAllocator());
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark < mark) {
			dgEdge* const twin = edge->m_twin;
			edge->m_mark = mark;
			twin->m_mark = mark;
			if ((edge->m_incidentFace > 0) && (twin->m_incidentFace > 0)) {
//				dgEdgePair& pair = bendPairs.Append()->GetInfo();
//				pair.m_edge = edge->m_prev;
//				pair.m_twin = twin->m_prev;
			}
		}
	}

	for (dgList<dgEdgePair>::dgListNode* node = bendPairs.GetFirst(); node; node = node->GetNext()) {
		dgEdgePair& pair = node->GetInfo();

		dgEdge* const bendEdge = conectivity.ConnectVertex (pair.m_edge, pair.m_twin);
		dgAssert (bendEdge);
		dgEdge* const bendTwin = bendEdge->m_twin;

		bendEdge->m_incidentFace = -1;
		bendTwin->m_incidentFace = -1;

		bendEdge->m_mark = mark;
		bendTwin->m_mark = mark;

		bendEdge->m_userData = 1;
		bendTwin->m_userData = 1;
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

	m_linkOrder = (dgInt32*) dgMallocStack (m_linksCount * sizeof (dgInt32));
	for (dgInt32 i = 0; i <  m_linksCount; i ++) {
		m_linkOrder[i] = i; 
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
		dgFree (m_posit);
		dgFree (m_links);
		dgFree (m_graph);
		dgFree (m_linkOrder);
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


void dgCollisionDeformableClothPatch::EndConfiguration ()
{
	dgStack<dgInt8> markPool (m_particles.m_count);
	dgInt8* const mark = &markPool[0];
	memset (mark, 0, markPool.GetSizeInBytes());

	dgList<dgSoftBodyEdge*> queue (GetAllocator());
	const dgFloat32* mass = m_particles.m_unitMass;
	for (dgInt32 i = 0; i < m_graphCount; i ++) {
		dgSoftBodyEdge* const edge = &m_graph[i];
		if ((mass[edge->m_vertex] == dgFloat32 (0.0f)) && !mark[edge->m_vertex]) {
			mark[edge->m_vertex] = 1;
			queue.Append (edge);
		}
	}

	dgInt32 index = 0;
	while (queue.GetCount()) {
		dgSoftBodyEdge* const edge = queue.GetFirst()->GetInfo();
		queue.Remove(queue.GetFirst());

		if (mark[edge->m_vertex] < 2) {
			mark[edge->m_vertex] = 2;
			dgSoftBodyEdge* ptr = edge;
			do {
				if (mark[ptr->m_twin->m_vertex] < 2) {
					m_linkOrder[index] = dgInt32 (ptr->m_link - m_links);
					index ++;
					queue.Append ((dgSoftBodyEdge*) ptr->m_twin);
				}
				ptr = (dgSoftBodyEdge*) ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}
}

void dgCollisionDeformableClothPatch::SetMatrix(const dgMatrix& matrix)
{
    dgAssert (0);
}

void dgCollisionDeformableClothPatch::SetMass (dgFloat32 mass)
{
	dgAssert(0);
}


void dgCollisionDeformableClothPatch::CreateClusters (dgInt32 count, dgFloat32 overlaringWidth)
{
	dgAssert(0);
}

void dgCollisionDeformableClothPatch::ApplyExternalForces (dgFloat32 timestep)
{
	dgVector* const posit = m_particles.m_posit;
	dgVector* const veloc = m_particles.m_veloc;
	dgFloat32* const unitMass = m_particles.m_unitMass;

	//timestep /= 4.0f;
	// apply gravity force
	// I am restricting all particle masses to be either infinite of constant value
	// this allow me to express then by a value that can be either 0.0 (infinite mass) or 1.0 (constant mass)

//	dgFloat32 particleMass = 0.01f;
	dgFloat32 invTimeStep = dgFloat32 (1.0f) / timestep;
	dgVector gravity (0.0f, -9.8f, 0.0f, 0.0f);
	//dgVector gravity (0.0f, -1.0f, 0.0f, 0.0f);
	dgVector gravityStep (gravity.Scale4 (timestep));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		veloc[i] = (posit[i] - m_posit[i]).Scale4 (invTimeStep);
		m_posit[i] = posit[i];
		veloc[i] += gravityStep.Scale4 (unitMass[i]);
		posit[i] += veloc[i].Scale4 (timestep);
	}
/*
	// calculate internal forces
	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		dgClothLink* const link = &m_links[m_linkOrder[i]];

		//no materials for now
		//const dgClothPatchMaterial& material = m_materials[link.m_materialIndex];

		// when iteration count is more than one, these can be precomputed ouside the loop;
		dgInt32 index0 = link->m_particle_0;
		dgInt32 index1 = link->m_particle_1;
		dgVector relPosit (posit[index1] - posit[index0]);
		dgFloat32 invMag2 = dgFloat32 (1.0f) / (relPosit % relPosit);

		// this is the velocity projection code 
		dgVector relVeloc (veloc[index0] - veloc[index1]);
		relVeloc = relPosit.Scale3 ((relVeloc % relPosit) * invMag2);

		veloc[index0] -= relVeloc.Scale3 (link->m_mass0_influence);
		veloc[index1] += relVeloc.Scale3 (link->m_mass1_influence);
	}
*/
}

//using paper, this is more stable than spring mass  
//http://www.pagines.ma1.upc.edu/~susin/files/AdvancedCharacterPhysics.pdf
void dgCollisionDeformableClothPatch::ResolvePositionsConstraints (dgFloat32 timestep)
{
	dgVector* const posit = m_particles.m_posit;
//	dgFloat32* const unitMass = m_particles.m_unitMass;

	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		dgClothLink* const link = &m_links[m_linkOrder[i]];

		// here these are not constant, need to do in each iteration
		// remenbet that mass == invMass (when boteh are 1.0
		dgInt32 index0 = link->m_particle_0;
		dgInt32 index1 = link->m_particle_1;
		dgVector relPosit (posit[index0] - posit[index1]);
		dgFloat32 x2 = relPosit % relPosit; 
#if 1
		dgFloat32 x = dgSqrt(x2);
#else 
		// using two turns of Taylor expansion for sqrt (x) with inital guess equa restlength the x = dgSqrt(x2) ~= (restLengh * restLengh + x2) / (2.0f * restLengh)
		dgFloat32 x = (link->m_restLengh * link->m_restLengh + x2) / (2.0f * link->m_restLengh);
#endif
		dgFloat32 error = (x - link->m_restLengh) / x;
		
		posit[index0] -= relPosit.Scale4 (error * link->m_mass0_influence);
		posit[index1] += relPosit.Scale4 (error * link->m_mass1_influence);
	}
}
