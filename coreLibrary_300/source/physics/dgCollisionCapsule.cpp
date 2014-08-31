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
#include "dgCollisionCapsule.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// around 89.5 degress
#define DG_CAPSULE_PERPENDICULAR_NORMAL		dgFloat32 (1.0e-2f)

dgInt32 dgCollisionCapsule::m_shapeRefCount = 0;
dgConvexSimplexEdge dgCollisionCapsule::m_edgeArray[DG_CAPSULE_SEGMENTS * (6 + 8 * (DG_CAP_SEGMENTS - 1))];

dgCollisionCapsule::dgCollisionCapsule(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 radius, dgFloat32 height)
	:dgCollisionConvex(allocator, signature, m_capsuleCollision)
{
	Init (radius, height);
}

dgCollisionCapsule::dgCollisionCapsule(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y);
}


dgCollisionCapsule::~dgCollisionCapsule()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}

void dgCollisionCapsule::Init (dgFloat32 radius, dgFloat32 height)
{
	m_rtti |= dgCollisionCapsule_RTTI;

	dgInt32 i0 = 0;
	dgInt32 i1 = DG_CAPSULE_SEGMENTS * DG_CAP_SEGMENTS * 2;

	m_radius = dgAbsf (radius);
	m_height = dgAbsf (height * dgFloat32 (0.5f));
	for (dgInt32 j = 0; j < DG_CAP_SEGMENTS; j ++) {
		dgFloat32 angle = dgFloat32 (0.0f);
		dgFloat32 x = (DG_CAP_SEGMENTS - j - 1) * m_radius / DG_CAP_SEGMENTS;
		dgFloat32 r = dgSqrt (m_radius * m_radius - x * x);
		i1 -= DG_CAPSULE_SEGMENTS;
		for (dgInt32 i = 0; i < DG_CAPSULE_SEGMENTS; i ++) {
			dgFloat32 z = dgSin (angle) * r;
			dgFloat32 y = dgCos (angle) * r;
			m_vertex[i0] = dgVector (- (m_height + x), y, z, dgFloat32 (0.0f));
			m_vertex[i1] = dgVector (  (m_height + x), y, z, dgFloat32 (0.0f));
			i0 ++;
			i1 ++;
			angle += dgPI2 / DG_CAPSULE_SEGMENTS;
		}
		i1 -= DG_CAPSULE_SEGMENTS;
	}

	m_vertexCount = DG_CAPSULE_SEGMENTS * DG_CAP_SEGMENTS * 2;
	m_edgeCount = DG_CAPSULE_SEGMENTS * (6 + 8 * (DG_CAP_SEGMENTS - 1));
	dgCollisionConvex::m_vertex = m_vertex;


	if (!m_shapeRefCount) {
		dgPolyhedra polyhedra(m_allocator);
		dgInt32 wireframe[DG_CAPSULE_SEGMENTS + 10];

		i1 = 0;
		i0 = DG_CAPSULE_SEGMENTS - 1;
		polyhedra.BeginFace ();
		for (dgInt32 j = 0; j < DG_CAP_SEGMENTS * 2 - 1; j ++) {
			for (dgInt32 i = 0; i < DG_CAPSULE_SEGMENTS; i ++) { 
				wireframe[0] = i0;
				wireframe[1] = i1;
				wireframe[2] = i1 + DG_CAPSULE_SEGMENTS;
				wireframe[3] = i0 + DG_CAPSULE_SEGMENTS;
				i0 = i1;
				i1 ++;
				polyhedra.AddFace (4, wireframe);
			}
			i0 = i1 + DG_CAPSULE_SEGMENTS - 1;
		}

		for (dgInt32 i = 0; i < DG_CAPSULE_SEGMENTS; i ++) { 
			wireframe[i] = DG_CAPSULE_SEGMENTS - 1 - i;
		}
		polyhedra.AddFace (DG_CAPSULE_SEGMENTS, wireframe);

		for (dgInt32 i = 0; i < DG_CAPSULE_SEGMENTS; i ++) { 
			wireframe[i] = i + DG_CAPSULE_SEGMENTS * (DG_CAP_SEGMENTS * 2 - 1);
		}
		polyhedra.AddFace (DG_CAPSULE_SEGMENTS, wireframe);
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


void dgCollisionCapsule::CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
//	dgVector radius (m_radius);
//	dgVector q0 (matrix[0].Scale4 (m_height));
	dgVector q0 (matrix[3] + matrix[0].Scale4 (m_height));
	dgVector q1 (matrix[3] - matrix[0].Scale4 (m_height));
	dgVector size (matrix.m_front.Abs().Scale4(m_radius) + matrix.m_up.Abs().Scale4(m_radius) + matrix.m_right.Abs().Scale4(m_radius));

//	dgVector q1 (q0.CompProduct4(dgVector::m_negOne));
//	p0 = (matrix[3] - radius + q0.GetMin(q1)) & dgVector::m_triplexMask;
//	p1 = (matrix[3] + radius + q0.GetMax(q1)) & dgVector::m_triplexMask;
	p0 = (q0.GetMin(q1) - size) & dgVector::m_triplexMask;
	p1 = (q0.GetMax(q1) + size) & dgVector::m_triplexMask;
}


dgInt32 dgCollisionCapsule::CalculateSignature (dgFloat32 radius, dgFloat32 height)
{
	dgUnsigned32 buffer[3];

	buffer[0] = m_capsuleCollision;
	buffer[1] = Quantize (radius);
	buffer[2] = Quantize (height);
	return Quantize(buffer, sizeof (buffer));
}

dgInt32 dgCollisionCapsule::CalculateSignature () const
{
	return CalculateSignature (m_radius, m_height);
}

void dgCollisionCapsule::TesselateTriangle (dgInt32 level, const dgVector& p0, const dgVector& p1, const dgVector& p2, dgInt32& count, dgVector* ouput) const
{
	if (level) {
		dgAssert (dgAbsf (p0 % p0 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p1 % p1 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p2 % p2 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgVector p01 (p0 + p1);
		dgVector p12 (p1 + p2);
		dgVector p20 (p2 + p0);

		p01 = p01.Scale3 (dgRsqrt(p01 % p01));
		p12 = p12.Scale3 (dgRsqrt(p12 % p12));
		p20 = p20.Scale3 (dgRsqrt(p20 % p20));

		dgAssert (dgAbsf (p01 % p01 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p12 % p12 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p20 % p20 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));

		TesselateTriangle (level - 1, p0,  p01, p20, count, ouput);
		TesselateTriangle (level - 1, p1,  p12, p01, count, ouput);
		TesselateTriangle (level - 1, p2,  p20, p12, count, ouput);
		TesselateTriangle (level - 1, p01, p12, p20, count, ouput);

	} else {
		ouput[count + 0] = p0.Scale3 (m_radius);
		ouput[count + 1] = p1.Scale3 (m_radius);
		ouput[count + 2] = p2.Scale3 (m_radius);
		count += 3;
	}
}


void dgCollisionCapsule::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	#define POWER 2
	dgVector tmpVectex[512];

	dgVector p0 ( dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	dgVector p1 (-dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	dgVector p2 ( dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	dgVector p3 ( dgFloat32 (0.0f),-dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector p4 ( dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f));
	dgVector p5 ( dgFloat32 (0.0f), dgFloat32 (0.0f),-dgFloat32 (1.0f), dgFloat32 (0.0f));

	dgInt32 count = 0;
	TesselateTriangle (POWER, p0, p2, p4, count, tmpVectex);
	TesselateTriangle (POWER, p0, p4, p3, count, tmpVectex);
	TesselateTriangle (POWER, p0, p3, p5, count, tmpVectex);
	TesselateTriangle (POWER, p0, p5, p2, count, tmpVectex);

	TesselateTriangle (POWER, p1, p4, p2, count, tmpVectex);
	TesselateTriangle (POWER, p1, p3, p4, count, tmpVectex);
	TesselateTriangle (POWER, p1, p5, p3, count, tmpVectex);
	TesselateTriangle (POWER, p1, p2, p5, count, tmpVectex);

	for (dgInt32 i = 0; i < count; i += 3) {
		int positive = 0;
		for (int j = 0; j < 3; j ++) {
			if (tmpVectex[i + j].m_x > dgFloat32 (0.0f)) {
				positive ++;
			}
		}

		if (positive) {
			dgVector face[4]; 	
			face[0] = tmpVectex[i + 0];
			face[1] = tmpVectex[i + 1];
			face[2] = tmpVectex[i + 2];
			face[0].m_x += m_height;
			face[1].m_x += m_height;
			face[2].m_x += m_height;
			matrix.TransformTriplex (&face[0].m_x, sizeof (dgTriplex), &face[0].m_x, sizeof (dgVector), 3);			
			callback (userData, 3, &face[0].m_x, 0);
		} else {
			dgVector face[4]; 	
			face[0] = tmpVectex[i + 0];
			face[1] = tmpVectex[i + 1];
			face[2] = tmpVectex[i + 2];
			face[0].m_x -= m_height;
			face[1].m_x -= m_height;
			face[2].m_x -= m_height;
			matrix.TransformTriplex (&face[0].m_x, sizeof (dgTriplex), &face[0].m_x, sizeof (dgVector), 3);			
			callback (userData, 3, &face[0].m_x, 0);
		}
		if (positive == 1) {
			dgVector p0 (tmpVectex[i + 0]);
			dgVector p1 (tmpVectex[i + 1]);
			if ((tmpVectex[i + 1].m_x == dgFloat32 (0.0f)) && (tmpVectex[i + 2].m_x == dgFloat32 (0.0f))) {
				p0 = tmpVectex[i + 1];
				p1 = tmpVectex[i + 2];
			} else if ((tmpVectex[i + 2].m_x == dgFloat32 (0.0f)) && (tmpVectex[i + 0].m_x == dgFloat32 (0.0f))){
				p0 = tmpVectex[i + 2];
				p1 = tmpVectex[i + 0];
			}

			dgVector face[4]; 	
			face[0] = p1;
			face[1] = p0;
			face[2] = p0;
			face[3] = p1;
			face[0].m_x += m_height;
			face[1].m_x += m_height;
			face[2].m_x -= m_height;
			face[3].m_x -= m_height;
			matrix.TransformTriplex (&face[0].m_x, sizeof (dgTriplex), &face[0].m_x, sizeof (dgVector), 4);			
			callback (userData, 4, &face[0].m_x, 0);
		}
	}
}


void dgCollisionCapsule::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}


dgVector dgCollisionCapsule::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	dgVector p0(dir.Scale4 (m_radius));
	dgVector p1(p0);
	dgVector height (m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	p0 += height;
	p1 -= height;
	dgVector mask (p1.DotProduct4(dir) > p0.DotProduct4(dir));
	return (p1 & mask) | p0.AndNot(mask);
}


dgFloat32 dgCollisionCapsule::RayCast (const dgVector& q0, const dgVector& q1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	dgVector origin0 ( m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector origin1 (-m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgFloat32 t0 = dgRayCastSphere (q0, q1, origin0, m_radius);
	dgFloat32 t1 = dgRayCastSphere (q0, q1, origin1, m_radius);

	if ((t0 < maxT) && (t1 < maxT)) {
		if (t0 < t1) {
			dgVector q (q0 + (q1 - q0).Scale4 (t0));
			dgVector n (q - origin0); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
			//contactOut.m_userId = SetUserDataID();
			return t0;
		} else {
			dgVector q (q0 + (q1 - q0).Scale4 (t1));
			dgVector n (q - origin1); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
			//contactOut.m_userId = SetUserDataID();
			return t1;
		}
//	} else if (t0 < dgFloat32 (1.2f)) {
	} else if (t0 < maxT) {
		dgVector q (q0 + (q1 - q0).Scale4 (t0));
		if (q.m_x >= m_height) {
			dgVector n (q - origin0); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
			//contactOut.m_userId = SetUserDataID();
			return t0;
		}

//	} else if (t1 < dgFloat32 (1.2f)) {
	} else if (t1 < maxT) {
		dgVector q (q0 + (q1 - q0).Scale4 (t1));
		if (q.m_x <= -m_height) {
			dgVector n (q - origin1); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
			//contactOut.m_userId = SetUserDataID();
			return t1;
		}
	}

	dgVector p0(q0);
	dgVector p1(q1);
	p0.m_x = dgFloat32 (0.0f);
	p1.m_x = dgFloat32 (0.0f);
	t0 = dgRayCastSphere (p0, p1, dgVector (dgFloat32 (0.0f)), m_radius);
//	if (t0 < dgFloat32 (1.0f)) {
	if (t0 < maxT) {
		dgVector q (q0 + (q1 - q0).Scale3 (t0));
		if ((q.m_x <= m_height) && (q.m_x >= -m_height)) {
			dgVector n (p0 + (p1 - p0).Scale4 (t0));
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
			//contactOut.m_userId = SetUserDataID();
			return t0;
		}
	}
	
	return dgFloat32 (1.2f);
}


void dgCollisionCapsule::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_capsule.m_radio = m_radius;
	info->m_capsule.m_height = dgFloat32 (2.0f) * m_height;
}


void dgCollisionCapsule::Serialize(dgSerialize callback, void* const userData) const
{
	dgVector size (m_radius, dgFloat32 (2.0f) * m_height, dgFloat32 (0.0f), dgFloat32 (0.0f));

	SerializeLow(callback, userData);
	callback (userData, &size, sizeof (dgVector));
}


void dgCollisionCapsule::MassProperties ()
{
/*
	m_centerOfMass = dgVector (dgFloat32 (0.0f));
	m_crossInertia = dgVector (dgFloat32 (0.0f));

	dgFloat32 cylVolume = dgFloat32 (3.14159f * 2.0f) * m_radius * m_radius * m_height;
	dgFloat32 sphVolume = dgFloat32 (3.14159f * 4.0f / 3.0f) * m_radius * m_radius * m_radius; 

	dgFloat32 cylInertiaxx = (dgFloat32 (0.5f) * m_radius * m_radius);
	dgFloat32 sphInertiaxx = (dgFloat32 (2.0f / 5.0f) * m_radius * m_radius);

	dgFloat32 cylInertiayyzz = (dgFloat32 (0.25f) * m_radius *  m_radius + dgFloat32 (1.0f / 3.0f) * m_height * m_height);
	dgFloat32 sphInertiayyzz = sphInertiaxx + m_height * m_height;

	dgFloat32 volume = cylVolume + sphVolume;
	dgFloat32 inertiaxx = cylInertiaxx + sphInertiaxx * dgFloat32 (0.5f);
	dgFloat32 inertiayyzz = cylInertiayyzz + sphInertiayyzz * dgFloat32 (0.5f);

	m_inertia[0] = inertiaxx;
	m_inertia[1] = inertiayyzz;
	m_inertia[2] = inertiayyzz;
	m_centerOfMass.m_w = volume;
*/
	dgCollisionConvex::MassProperties ();
	m_centerOfMass = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), m_centerOfMass.m_w);
	m_crossInertia = dgVector (dgFloat32 (0.0f));
}


dgVector dgCollisionCapsule::ConvexConicSupporVertex (const dgVector& dir) const
{
	return dgVector ((dir.m_x >= dgFloat32 (0.0f)) ? m_height : - m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
}

dgVector dgCollisionCapsule::ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const
{
	dgVector p (SupportVertex(dir, NULL));
	if (dgAbsf(dir.m_x) < dgFloat32 (1.0e-3f)) {
		p.m_x = point.m_x;
	}
	return p;
}


dgInt32 dgCollisionCapsule::CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const
{
//	if (dgAbsf (normal.m_x) <= dgFloat32 (5.0e-2f)) {
//		return CalculateContactsGeneric (point, normal, proxy, contactsOut);
//	} else if (point.m_x > m_height) {
//		return CalculateSphereConicContacts (m_height, normal, point, contactsOut);
//	} else if (point.m_x < -m_height) {
//		return CalculateSphereConicContacts (-m_height, normal, point, contactsOut);
//	}
	if (dgAbsf (normal.m_x) > DG_CAPSULE_PERPENDICULAR_NORMAL) {
		contactsOut[0] = SupportVertex (normal.Scale3 (dgFloat32 (-1.0f)), NULL);
		return 1;
	}
	return CalculateContactsGeneric (point, normal, proxy, contactsOut);
}



dgInt32 dgCollisionCapsule::CalculatePlaneIntersection (const dgVector& normal, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgInt32 count = 0;
	if (dgAbsf (normal.m_x) > DG_CAPSULE_PERPENDICULAR_NORMAL) {
		contactsOut[0] = SupportVertex (normal, NULL);
		count = 1;
	} else {
		dgVector p0 (m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector dir0 (p0 - origin);
		dgFloat32 dist0 = dir0 % normal;
		dgVector p1 (-m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector dir1 (p1 - origin);
		dgFloat32 dist1 = dir1 % normal;
		contactsOut[0] = p0 - normal.Scale3 (dist0);
		contactsOut[1] = p1 - normal.Scale3 (dist1);
		count = 2;
	}
	return count;
}

