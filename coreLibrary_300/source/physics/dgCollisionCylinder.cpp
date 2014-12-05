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
#include "dgCollisionCylinder.h"
#include "dgCollisionConvexPolygon.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define DG_CYLINDER_SKIN_PADDING dgFloat32 (0.05f)
//#define DG_CYLINDER_SKIN_PADDING (DG_RESTING_CONTACT_PENETRATION * dgFloat32 (0.25f))

dgInt32 dgCollisionCylinder::m_shapeRefCount = 0;
dgConvexSimplexEdge dgCollisionCylinder::m_edgeArray[DG_CYLINDER_SEGMENTS * 2 * 3];

dgCollisionCylinder::dgCollisionCylinder(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 radius, dgFloat32 height)
	:dgCollisionConvex(allocator, signature, m_cylinderCollision)
{
	Init (radius, height);
}


dgCollisionCylinder::dgCollisionCylinder(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y);
}

void dgCollisionCylinder::Init (dgFloat32 radius, dgFloat32 height)
{
	m_rtti |= dgCollisionCylinder_RTTI;
	m_radius = dgAbsf (radius);
	m_height = dgAbsf (height * dgFloat32 (0.5f));
	m_skinthickness = dgMin (dgMin (m_radius, m_height) * dgFloat32 (0.005f), dgFloat32 (1.0f / 64.0f));

	dgFloat32 angle = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < DG_CYLINDER_SEGMENTS; i ++) {
		dgFloat32 z = dgSin (angle) * m_radius;
		dgFloat32 y = dgCos (angle) * m_radius;
		m_vertex[i                       ] = dgVector (- m_height, y, z, dgFloat32 (0.0f));
		m_vertex[i + DG_CYLINDER_SEGMENTS] = dgVector (  m_height, y, z, dgFloat32 (0.0f));
		angle += dgPI2 / DG_CYLINDER_SEGMENTS;
	}

	m_edgeCount = DG_CYLINDER_SEGMENTS * 6;
	m_vertexCount = DG_CYLINDER_SEGMENTS * 2;
	dgCollisionConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) {
		dgPolyhedra polyhedra(m_allocator);
		dgInt32 wireframe[DG_CYLINDER_SEGMENTS];

		dgInt32 j = DG_CYLINDER_SEGMENTS - 1;
		polyhedra.BeginFace ();
		for (dgInt32 i = 0; i < DG_CYLINDER_SEGMENTS; i ++) { 
			wireframe[0] = j;
			wireframe[1] = i;
			wireframe[2] = i + DG_CYLINDER_SEGMENTS;
			wireframe[3] = j + DG_CYLINDER_SEGMENTS;
			j = i;
			polyhedra.AddFace (4, wireframe);
		}

		for (dgInt32 i = 0; i < DG_CYLINDER_SEGMENTS; i ++) { 
			wireframe[i] = DG_CYLINDER_SEGMENTS - 1 - i;
		}
		polyhedra.AddFace (DG_CYLINDER_SEGMENTS, wireframe);

		for (dgInt32 i = 0; i < DG_CYLINDER_SEGMENTS; i ++) { 
			wireframe[i] = i + DG_CYLINDER_SEGMENTS;
		}
		polyhedra.AddFace (DG_CYLINDER_SEGMENTS, wireframe);
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

	m_profile[0] = dgVector ( m_height,  m_radius, 0.0f, 0.0f);
	m_profile[1] = dgVector (-m_height,  m_radius, 0.0f, 0.0f);
	m_profile[2] = dgVector (-m_height, -m_radius, 0.0f, 0.0f);
	m_profile[3] = dgVector ( m_height, -m_radius, 0.0f, 0.0f);

	m_shapeRefCount ++;
	dgCollisionConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG ();
}

dgCollisionCylinder::~dgCollisionCylinder()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}

dgInt32 dgCollisionCylinder::CalculateSignature (dgFloat32 radius, dgFloat32 height)
{
	dgUnsigned32 buffer[3];

	buffer[0] = m_cylinderCollision;
	buffer[1] = Quantize (radius);
	buffer[2] = Quantize (height);
	return Quantize(buffer, sizeof (buffer));
}

dgInt32 dgCollisionCylinder::CalculateSignature () const
{
	return CalculateSignature (m_radius, m_height);
}


void dgCollisionCylinder::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (0);
}


void dgCollisionCylinder::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTriplex pool[24 * 2];

	dgFloat32 angle = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < 24; i ++) {
		dgFloat32 z = dgSin (angle) * m_radius;
		dgFloat32 y = dgCos (angle) * m_radius;
		pool[i].m_x = - m_height;
		pool[i].m_y = y;
		pool[i].m_z = z;
		pool[i + 24].m_x = m_height;
		pool[i + 24].m_y = y;
		pool[i + 24].m_z = z;
		angle += dgPI2 / dgFloat32 (24.0f);
	}

	matrix.TransformTriplex (&pool[0].m_x, sizeof (dgTriplex), &pool[0].m_x, sizeof (dgTriplex), 24 * 2);

	dgTriplex face[24];

	dgInt32 j = 24 - 1;
	for (dgInt32 i = 0; i < 24; i ++) { 
		face[0] = pool[j];
		face[1] = pool[i];
		face[2] = pool[i + 24];
		face[3] = pool[j + 24];
		j = i;
		callback (userData, 4, &face[0].m_x, 0);
	}

	for (dgInt32 i = 0; i < 24; i ++) { 
		face[i] = pool[24 - 1 - i];
	}
	callback (userData, 24, &face[0].m_x, 0);
		
	for (dgInt32 i = 0; i < 24; i ++) { 
		face[i] = pool[i + 24];
	}
	callback (userData, 24, &face[0].m_x, 0);
}



dgVector dgCollisionCylinder::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf (dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	dgFloat32 y0 = m_radius;
	dgFloat32 z0 = dgFloat32 (0.0f);
	dgFloat32 mag2 = dir.m_y * dir.m_y + dir.m_z * dir.m_z;
	if (mag2 > dgFloat32 (1.0e-12f)) {
		mag2 = dgRsqrt(mag2);
		y0 = dir.m_y * m_radius * mag2;
		z0 = dir.m_z * m_radius * mag2;
	}
	return dgVector (dir.m_x >= dgFloat32 (0.0f) ? m_height : - m_height, y0, z0, dgFloat32 (0.0f));       
}



void dgCollisionCylinder::MassProperties ()
{
	m_centerOfMass = dgVector (dgFloat32 (0.0f));
	m_crossInertia = dgVector (dgFloat32 (0.0f));

	dgFloat32 volume = dgFloat32 (3.14159f * 2.0f) * m_radius * m_radius * m_height; 	
	dgFloat32 inertiaxx   = dgFloat32 (0.5f) * m_radius *  m_radius;
	dgFloat32 inertiayyzz = (dgFloat32 (0.25f) * m_radius *  m_radius + dgFloat32 (1.0f / 3.0f) * m_height * m_height);

	//dgCollisionConvex::MassProperties ();
	m_inertia[0] = inertiaxx;
	m_inertia[1] = inertiayyzz;
	m_inertia[2] = inertiayyzz;
	m_centerOfMass.m_w = volume;
}

dgFloat32 dgCollisionCylinder::GetSkinThickness () const
{
	return m_skinthickness;
}

dgVector dgCollisionCylinder::ConvexConicSupporVertex (const dgVector& dir) const
{
	dgAssert (dgAbsf ((dir % dir - dgFloat32 (1.0f))) < dgFloat32 (1.0e-3f));

	//dgFloat32 radius = (m_radius > DG_CYLINDER_SKIN_PADDING) ? m_radius - DG_CYLINDER_SKIN_PADDING : m_radius;
	//dgFloat32 height = (m_height > DG_CYLINDER_SKIN_PADDING) ? m_height - DG_CYLINDER_SKIN_PADDING : m_height;
	const dgFloat32 radius = m_radius - m_skinthickness;
	const dgFloat32 height = m_height - m_skinthickness;

	dgFloat32 y0 = radius;
	dgFloat32 z0 = dgFloat32 (0.0f);
	dgFloat32 mag2 = dir.m_y * dir.m_y + dir.m_z * dir.m_z;
	if (mag2 > dgFloat32 (1.0e-12f)) {
		mag2 = dgRsqrt(mag2);
		y0 = dir.m_y * radius * mag2;
		z0 = dir.m_z * radius * mag2;
	}
	return dgVector (dir.m_x >= dgFloat32 (0.0f) ? height : - height, y0, z0, dgFloat32 (0.0f));       
}

dgVector dgCollisionCylinder::ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const
{
	dgVector p (SupportVertex(dir, NULL));
	if (dgAbsf(dir.m_x) > dgFloat32 (0.9997f)) {
		p.m_y = point.m_y;
		p.m_z = point.m_z;
	} else if (dgAbsf(dir.m_x) < dgFloat32 (1.0e-3f)) {
		p.m_x = point.m_x;
	}
	return p;
}



dgInt32 dgCollisionCylinder::CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const
{
	dgAssert (dgAbsf (normal % normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	return CalculateContactsGeneric (point, normal, proxy, contactsOut);
}




dgFloat32 dgCollisionCylinder::RayCast (const dgVector& q0, const dgVector& q1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	dgFloat32 t = maxT;
	dgVector p0 (q0);
	p0.m_x = dgFloat32 (0.0f);
	const dgFloat32 radius2 = m_radius * m_radius;
	const dgFloat32 c = (p0 % p0) - radius2;
	if (c > dgFloat32 (0.0f)) {
		dgVector dp (q1 - q0);
		dp.m_x = dgFloat32 (0.0f);
		dgFloat32 a = dp % dp;
		const dgFloat32 b = dgFloat32 (2.0f) * (p0 % dp);

		dgFloat32 desc = b * b - dgFloat32 (4.0f) * a * c;
		if (desc > dgFloat32 (1.0e-8f)) {
			desc = dgSqrt (desc);
			a = dgFloat32 (1.0f) / (dgFloat32 (2.0f) * a);
			dgFloat32 t1 = dgMin ((- b + desc) * a, (- b - desc) * a);
			//if ((t1 < dgFloat32 (1.0f)) && (t1 >= dgFloat32 (0.0f))) {
			if ((t1 < maxT) && (t1 >= dgFloat32 (0.0f))) {
				dgVector dq (q1 - q0);
				dgVector contact (q0 + dq.Scale3 (t1));
				if (contact.m_x > m_height) {
					if (q1.m_x < m_height) {
						t1 = (m_height - q0.m_x) / (q1.m_x - q0.m_x);
						dgFloat32 y = q0.m_y + (q1.m_y - q0.m_y) * t1;
						dgFloat32 z = q0.m_z + (q1.m_z - q0.m_z) * t1;
						if ((y * y + z * z - radius2) < dgFloat32 (0.0f)) {
							t = t1;
							contactOut.m_normal = dgVector (dgFloat32 (dgFloat32 (1.0f)), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
							//contactOut.m_userId = SetUserDataID();
						}
					}
				} else if (contact.m_x < -m_height) {
					if (q1.m_x > -m_height) {
						t1 = (-m_height - q0.m_x) / (q1.m_x - q0.m_x);
						dgFloat32 y = q0.m_y + (q1.m_y - q0.m_y) * t1;
						dgFloat32 z = q0.m_z + (q1.m_z - q0.m_z) * t1;
						if ((y * y + z * z - radius2) < dgFloat32 (0.0f)) {
							t = t1;
							contactOut.m_normal = dgVector (-dgFloat32 (dgFloat32 (1.0f)), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
							//contactOut.m_userId = SetUserDataID();
						}
					}
				} else if (t1 >=  dgFloat32 (0.0f)) {
					t = t1;
					dgVector n (contact);
					n.m_x = dgFloat32 (0.0f);
					contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
					//contactOut.m_userId = SetUserDataID();
				}
			}
		}
	} else {
		if (q0.m_x > m_height) {
			if (q1.m_x < m_height) {
				dgFloat32 t1 = (m_height - q0.m_x) / (q1.m_x - q0.m_x);
				dgFloat32 y = q0.m_y + (q1.m_y - q0.m_y) * t1;
				dgFloat32 z = q0.m_z + (q1.m_z - q0.m_z) * t1;
				if ((y * y + z * z - radius2) < dgFloat32 (0.0f)) {
					t = t1;
					contactOut.m_normal = dgVector (dgFloat32 (dgFloat32 (1.0f)), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
					//contactOut.m_userId = SetUserDataID();
				}
			}
		} else if (q0.m_x < -m_height) {
			if (q1.m_x > -m_height) {
				dgFloat32 t1 = (-m_height - q0.m_x) / (q1.m_x - q0.m_x);
				dgFloat32 y = q0.m_y + (q1.m_y - q0.m_y) * t1;
				dgFloat32 z = q0.m_z + (q1.m_z - q0.m_z) * t1;
				if ((y * y + z * z - radius2) < dgFloat32 (0.0f)) {
					t = t1;
					contactOut.m_normal = dgVector (-dgFloat32 (dgFloat32 (1.0f)), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
					//contactOut.m_userId = SetUserDataID();
				}
			}
		}
	}
	return t;
}


void dgCollisionCylinder::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_cylinder.m_radius = m_radius;
	info->m_cylinder.m_height = m_height * dgFloat32 (2.0f);
}


void dgCollisionCylinder::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);

	dgVector size (dgAbsf (m_radius), m_height * dgFloat32 (2.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	callback (userData, &size, sizeof (dgVector));
}



dgInt32 dgCollisionCylinder::CalculatePlaneIntersection (const dgVector& normal, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgInt32 count;
	if (dgAbsf (normal.m_x) < dgFloat32 (0.999f)) { 
/*
		dgFloat32 magInv = dgRsqrt (normal.m_y * normal.m_y + normal.m_z * normal.m_z);
		dgFloat32 cosAng = normal.m_y * magInv;
		dgFloat32 sinAng = normal.m_z * magInv;

		dgAssert (dgAbsf (normal.m_z * cosAng - normal.m_y * sinAng) < dgFloat32 (1.0e-4f));
		dgVector normal1 (normal.m_x, normal.m_y * cosAng + normal.m_z * sinAng, dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector origin1 (origin.m_x, origin.m_y * cosAng + origin.m_z * sinAng, origin.m_z * cosAng - origin.m_y * sinAng, dgFloat32 (0.0f));

		count = dgCollisionConvex::CalculatePlaneIntersection (normal1, origin1, contactsOut, normalSign);
		for (dgInt32 i = 0; i < count; i ++) {
			dgFloat32 y = contactsOut[i].m_y;
			dgFloat32 z = contactsOut[i].m_z;
			contactsOut[i].m_y = y * cosAng - z * sinAng; 
			contactsOut[i].m_z = z * cosAng + y * sinAng;
		}
*/

		if (dgAbsf (normal.m_x) > dgFloat32 (0.15f)) {
			contactsOut[0] = SupportVertex(normal , NULL);
			contactsOut[1] = SupportVertex(normal.CompProduct4(dgVector::m_negOne) , NULL);
			dgVector test0 ((contactsOut[0] - origin).DotProduct4(normal).Abs());
			dgVector test1 ((contactsOut[1] - origin).DotProduct4(normal).Abs()); 
			if ((test0 > test1).GetSignMask()) {
				contactsOut[0] = contactsOut[1];
			}
			count = 1;

		}  else {
			dgFloat32 magInv = dgRsqrt (normal.m_y * normal.m_y + normal.m_z * normal.m_z);
			dgFloat32 cosAng = normal.m_y * magInv;
			dgFloat32 sinAng = normal.m_z * magInv;

			dgAssert (dgAbsf (normal.m_z * cosAng - normal.m_y * sinAng) < dgFloat32 (1.0e-4f));
			dgVector normal1 (normal.m_x, normal.m_y * cosAng + normal.m_z * sinAng, dgFloat32 (0.0f), dgFloat32 (0.0f));
			dgVector origin1 (origin.m_x, origin.m_y * cosAng + origin.m_z * sinAng, origin.m_z * cosAng - origin.m_y * sinAng, dgFloat32 (0.0f));

			count = 0;
			int i0 = 3;
			dgVector test0 ((m_profile[i0] - origin1).DotProduct4(normal1));
			for (int i = 0; (i < 4) && (count < 2); i ++) {
				dgVector test1 ((m_profile[i] - origin1).DotProduct4(normal1));
				dgVector acrossPlane (test0.CompProduct4(test1));
				if (acrossPlane.m_x < 0.0f) {
					dgVector step (m_profile[i] - m_profile[i0]);
					contactsOut[count] = m_profile[i0] - step.Scale4(test0.m_x/(step.DotProduct4(normal1).m_x));
					count ++;
				}
				i0 = i;
				test0 = test1;
			}

			for (dgInt32 i = 0; i < count; i ++) {
				dgFloat32 y = contactsOut[i].m_y;
				dgFloat32 z = contactsOut[i].m_z;
				contactsOut[i].m_y = y * cosAng - z * sinAng; 
				contactsOut[i].m_z = z * cosAng + y * sinAng;
			}
		}

	} else {
		count = dgCollisionConvex::CalculatePlaneIntersection (normal, origin, contactsOut, normalSign);
	}
	return count;
}
