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
#include "dgContact.h"
#include "dgCollisionCone.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


dgInt32 dgCollisionCone::m_shapeRefCount = 0;
dgConvexSimplexEdge dgCollisionCone::m_edgeArray[DG_CONE_SEGMENTS * 4];

dgCollisionCone::dgCollisionCone(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 radius, dgFloat32 height)
	:dgCollisionConvex(allocator, signature, m_coneCollision)
{
	Init (radius, height);
}

dgCollisionCone::dgCollisionCone(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y);
}


dgCollisionCone::~dgCollisionCone()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}

void dgCollisionCone::Init (dgFloat32 radius, dgFloat32 height)
{
	m_rtti |= dgCollisionCone_RTTI;
	m_radius = dgAbsf (radius);
	//m_sinAngle = m_radius / dgSqrt (height * height + m_radius * m_radius);
	m_sinAngle = m_radius * dgRsqrt (height * height + m_radius * m_radius);

	m_height = dgAbsf (height * dgFloat32 (0.5f));

	m_amp = dgFloat32 (0.5f) * m_radius / m_height;

	dgFloat32 angle = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < DG_CONE_SEGMENTS; i ++) {
		dgFloat32 z = dgSin (angle) * m_radius;
		dgFloat32 y = dgCos (angle) * m_radius;
		m_vertex[i] = dgVector (- m_height, y, z, dgFloat32 (0.0f));

		angle += dgPI2 / DG_CONE_SEGMENTS;
	}
	m_vertex[DG_CONE_SEGMENTS] = dgVector (m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	m_edgeCount = DG_CONE_SEGMENTS * 4;
	m_vertexCount = DG_CONE_SEGMENTS + 1;
	dgCollisionConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) {
		dgPolyhedra polyhedra(m_allocator);
		dgInt32 wireframe[DG_CONE_SEGMENTS];

		dgInt32 j = DG_CONE_SEGMENTS - 1;
		polyhedra.BeginFace ();
		for (dgInt32 i = 0; i < DG_CONE_SEGMENTS; i ++) { 
			wireframe[0] = j;
			wireframe[1] = i;
			wireframe[2] = DG_CONE_SEGMENTS;
			j = i;
			polyhedra.AddFace (3, wireframe);
		}

		for (dgInt32 i = 0; i < DG_CONE_SEGMENTS; i ++) { 
			wireframe[i] = DG_CONE_SEGMENTS - 1 - i;
		}
		polyhedra.AddFace (DG_CONE_SEGMENTS, wireframe);
		polyhedra.EndFace ();

		dgAssert (SanityCheck (polyhedra));

		dgUnsigned64 i = 0;
		dgPolyhedra::Iterator iter (polyhedra);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			edge->m_userData = i;
			i ++;
		}

		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);

			dgConvexSimplexEdge* const ptr = &m_edgeArray[edge->m_userData];

			ptr->m_vertex = edge->m_incidentVertex;
			ptr->m_next = &m_edgeArray[edge->m_next->m_userData];
			ptr->m_prev = &m_edgeArray[edge->m_prev->m_userData];
			ptr->m_twin = &m_edgeArray[edge->m_twin->m_userData];
		}
	}

	m_shapeRefCount ++;
	dgCollisionConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG ();
}

dgInt32 dgCollisionCone::CalculateSignature (dgFloat32 radius, dgFloat32 height)
{
	dgUnsigned32 buffer[3];

	memset (buffer, 0, sizeof (buffer));
	buffer[0] = m_coneCollision;
	buffer[1] = dgCollision::Quantize (radius);
	buffer[2] = dgCollision::Quantize (height);
	return dgInt32 (dgCollision::Quantize(buffer, sizeof (buffer)));
}

dgInt32 dgCollisionCone::CalculateSignature () const
{
	return CalculateSignature (m_radius, m_height);
}


void dgCollisionCone::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	#define NUMBER_OF_DEBUG_SEGMENTS  40
	dgTriplex pool[NUMBER_OF_DEBUG_SEGMENTS + 1];
	dgTriplex face[NUMBER_OF_DEBUG_SEGMENTS];

	dgFloat32 angle = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i ++) {
		dgFloat32 z = dgSin (angle) * m_radius;
		dgFloat32 y = dgCos (angle) * m_radius;
		pool[i].m_x = - m_height;
		pool[i].m_y = y;
		pool[i].m_z = z;
		angle += dgPI2 / dgFloat32 (NUMBER_OF_DEBUG_SEGMENTS);
	}
	
	pool[NUMBER_OF_DEBUG_SEGMENTS].m_x = m_height;
	pool[NUMBER_OF_DEBUG_SEGMENTS].m_y = dgFloat32 (0.0f);
	pool[NUMBER_OF_DEBUG_SEGMENTS].m_z = dgFloat32 (0.0f);

	matrix.TransformTriplex (&pool[0].m_x, sizeof (dgTriplex), &pool[0].m_x, sizeof (dgTriplex), NUMBER_OF_DEBUG_SEGMENTS + 1);
	dgInt32 j = NUMBER_OF_DEBUG_SEGMENTS - 1;
	for (dgInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i ++) { 
		face[0] = pool[j];
		face[1] = pool[i];
		face[2] = pool[NUMBER_OF_DEBUG_SEGMENTS];
		j = i;
		callback (userData, 3, &face[0].m_x, 0);
	}

	for (dgInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i ++) { 
		face[i] = pool[NUMBER_OF_DEBUG_SEGMENTS - 1 - i];
	}
	callback (userData, NUMBER_OF_DEBUG_SEGMENTS, &face[0].m_x, 0);
}




void dgCollisionCone::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}


dgVector dgCollisionCone::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	if (dir.m_x > m_sinAngle) {
		return dgVector (m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));       
	} 

	dgFloat32 y0 = m_radius;
	dgFloat32 z0 = dgFloat32 (0.0f);
	dgFloat32 mag2 = dir.m_y * dir.m_y + dir.m_z * dir.m_z;
	if (mag2 > dgFloat32 (1.0e-12f)) {
		mag2 = dgRsqrt(mag2);
		y0 = dir.m_y * m_radius * mag2;
		z0 = dir.m_z * m_radius * mag2;
	}
	return dgVector (-m_height, y0, z0, dgFloat32 (0.0f));       
}

void dgCollisionCone::MassProperties ()
{
	m_centerOfMass = dgVector (-dgFloat32 (1.0f/2.0f) * m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	m_crossInertia = dgVector (dgFloat32 (0.0f));

	dgFloat32 volume = dgFloat32 (3.14159f * 2.0f / 3.0f) * m_radius * m_radius * m_height; 
	dgFloat32 inertiaxx = dgFloat32 (3.0f / 10.0f) * m_radius * m_radius;
	dgFloat32 inertiayyzz = (dgFloat32 (3.0f / 20.0f) * m_radius * m_radius + dgFloat32 (3.0f / 20.0f) * m_height * m_height);

//	dgCollisionConvex::MassProperties ();
	m_inertia[0] = inertiaxx;
	m_inertia[1] = inertiayyzz;
	m_inertia[2] = inertiayyzz;
	m_centerOfMass.m_w = volume;
}


dgInt32 dgCollisionCone::CalculatePlaneIntersection (const dgVector& normal, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgInt32 count;
	if (dgAbsf (normal.m_x) < dgFloat32 (0.999f)) { 
		dgFloat32 magInv = dgRsqrt (normal.m_y * normal.m_y + normal.m_z * normal.m_z);
		dgFloat32 cosAng = normal.m_y * magInv;
		dgFloat32 sinAng = normal.m_z * magInv;
		dgAssert (dgAbsf (normal.m_z * cosAng - normal.m_y * sinAng) < dgFloat32 (1.0e-4f));
		dgVector normal1 (normal.m_x, normal.m_y * cosAng + normal.m_z * sinAng, dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector origin1 (origin.m_x, origin.m_y * cosAng + origin.m_z * sinAng, 
									  origin.m_z * cosAng - origin.m_y * sinAng, dgFloat32 (0.0f));
		count = dgCollisionConvex::CalculatePlaneIntersection (normal1, origin1, contactsOut, normalSign);
		for (dgInt32 i = 0; i < count; i ++) {
			dgFloat32 y = contactsOut[i].m_y;
			dgFloat32 z = contactsOut[i].m_z;
			contactsOut[i].m_y = y * cosAng - z * sinAng; 
			contactsOut[i].m_z = z * cosAng + y * sinAng;
		}

	} else {
		count = dgCollisionConvex::CalculatePlaneIntersection (normal, origin, contactsOut, normalSign);
	}
	return count;
}



void dgCollisionCone::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_cone.m_r = m_radius;
	info->m_cone.m_height = m_height * dgFloat32 (2.0f);
}

void dgCollisionCone::Serialize(dgSerialize callback, void* const userData) const
{
	dgVector size (m_radius, m_height * dgFloat32 (2.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	SerializeLow(callback, userData);
	callback (userData, &size, sizeof (dgVector));
}
