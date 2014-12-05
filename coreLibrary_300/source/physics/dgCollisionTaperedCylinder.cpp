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
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionTaperedCylinder.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define DG_TAPED_CYLINDER_SKIN_PADDING dgFloat32 (0.05f)
//#define DG_TAPED_CYLINDER_SKIN_PADDING (DG_RESTING_CONTACT_PENETRATION * 0.25f)

dgInt32 dgCollisionTaperedCylinder::m_shapeRefCount = 0;
dgConvexSimplexEdge dgCollisionTaperedCylinder::m_edgeArray[DG_CYLINDER_SEGMENTS * 2 * 3];

dgCollisionTaperedCylinder::dgCollisionTaperedCylinder(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height)
	:dgCollisionConvex(allocator, signature, m_taperedCylinderCollision)
{
	Init (radio0, radio1, height);
}


dgCollisionTaperedCylinder::dgCollisionTaperedCylinder(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y, size.m_z);
}

void dgCollisionTaperedCylinder::Init (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height)
{
	m_rtti |= dgCollisionTaperedCylinder_RTTI;
	m_radio0 = dgAbsf (radio0);
	m_radio1 = dgAbsf (radio1);
	m_height = dgAbsf (height * dgFloat32 (0.5f));
	m_skinthickness = dgMin (dgMin (m_radio0, m_radio1, m_height) * dgFloat32 (0.005f), dgFloat32 (1.0f / 64.0f));

	m_dirVector.m_x = radio1 - radio0;
	m_dirVector.m_y = m_height * dgFloat32 (2.0f);
	m_dirVector.m_z = dgFloat32 (0.0f);
	m_dirVector.m_w = dgFloat32 (0.0f);
	m_dirVector = m_dirVector.Scale3(dgRsqrt(m_dirVector % m_dirVector));

	dgFloat32 angle = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < DG_CYLINDER_SEGMENTS; i ++) {
		dgFloat32 sinAngle = dgSin (angle);
		dgFloat32 cosAngle = dgCos (angle);
		m_vertex[i                       ] = dgVector (- m_height, m_radio1 * cosAngle, m_radio1 * sinAngle, dgFloat32 (0.0f));
		m_vertex[i + DG_CYLINDER_SEGMENTS] = dgVector (  m_height, m_radio0 * cosAngle, m_radio0 * sinAngle,  dgFloat32 (0.0f));
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

	m_shapeRefCount ++;
	dgCollisionConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG ();
}

dgCollisionTaperedCylinder::~dgCollisionTaperedCylinder()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}

dgInt32 dgCollisionTaperedCylinder::CalculateSignature (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height)
{
	dgUnsigned32 buffer[4];

	buffer[0] = m_taperedCylinderCollision;
	buffer[1] = Quantize (radio0);
	buffer[2] = Quantize (radio1);
	buffer[3] = Quantize (height);
	return Quantize(buffer, sizeof (buffer));
}

dgInt32 dgCollisionTaperedCylinder::CalculateSignature () const
{
	return CalculateSignature (m_radio0, m_radio1, m_height);
}


void dgCollisionTaperedCylinder::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (0);
}


void dgCollisionTaperedCylinder::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTriplex pool[24 * 2];

	dgFloat32 angle = dgFloat32 (0.0f);
	for (dgInt32 i = 0; i < 24; i ++) {
		dgFloat32 z = dgSin (angle);
		dgFloat32 y = dgCos (angle);
		pool[i].m_x = - m_height;
		pool[i].m_y = y * m_radio1;
		pool[i].m_z = z * m_radio1;
		pool[i + 24].m_x = m_height;
		pool[i + 24].m_y = y * m_radio0;
		pool[i + 24].m_z = z * m_radio0;
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



dgVector dgCollisionTaperedCylinder::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf ((dir % dir - dgFloat32 (1.0f))) < dgFloat32 (1.0e-3f));
	dgFloat32 y0 = m_radio0;
	dgFloat32 z0 = dgFloat32 (0.0f);
	dgFloat32 y1 = m_radio1;
	dgFloat32 z1 = dgFloat32 (0.0f);
	dgFloat32 mag2 = dir.m_y * dir.m_y + dir.m_z * dir.m_z;
	if (mag2 > dgFloat32 (1.0e-12f)) {
		mag2 = dgRsqrt(mag2);
		y0 = dir.m_y * m_radio0 * mag2;
		z0 = dir.m_z * m_radio0 * mag2;
		y1 = dir.m_y * m_radio1 * mag2;
		z1 = dir.m_z * m_radio1 * mag2;
	}
	dgVector p0 ( m_height, y0, z0, dgFloat32 (0.0f));
	dgVector p1 (-m_height, y1, z1, dgFloat32 (0.0f));

	dgFloat32 dist0 = dir % p0;
	dgFloat32 dist1 = dir % p1;
	if (dist1 >= dist0) {
		p0 = p1;
	}
	return p0;
}

dgFloat32 dgCollisionTaperedCylinder::GetSkinThickness () const
{
	return m_skinthickness;
}


dgVector dgCollisionTaperedCylinder::ConvexConicSupporVertex (const dgVector& dir) const
{
	dgAssert (dgAbsf ((dir % dir - dgFloat32 (1.0f))) < dgFloat32 (1.0e-3f));

	//dgFloat32 height = (m_height > DG_TAPED_CYLINDER_SKIN_PADDING) ? m_height - DG_TAPED_CYLINDER_SKIN_PADDING : m_height;
	//dgFloat32 radio0 = (m_radio0 > DG_TAPED_CYLINDER_SKIN_PADDING) ? m_radio0 - DG_TAPED_CYLINDER_SKIN_PADDING : m_radio0;
	//dgFloat32 radio1 = (m_radio1 > DG_TAPED_CYLINDER_SKIN_PADDING) ? m_radio1 - DG_TAPED_CYLINDER_SKIN_PADDING : m_radio1;
	const dgFloat32 height = m_height - m_skinthickness;
	const dgFloat32 radio0 = m_radio0 - m_skinthickness;
	const dgFloat32 radio1 = m_radio1 - m_skinthickness;

	dgAssert (dgAbsf ((dir % dir - dgFloat32 (1.0f))) < dgFloat32 (1.0e-3f));
	dgFloat32 y0 = radio0;
	dgFloat32 z0 = dgFloat32 (0.0f);
	dgFloat32 y1 = radio1;
	dgFloat32 z1 = dgFloat32 (0.0f);
	dgFloat32 mag2 = dir.m_y * dir.m_y + dir.m_z * dir.m_z;
	if (mag2 > dgFloat32 (1.0e-12f)) {
		mag2 = dgRsqrt(mag2);
		y0 = dir.m_y * radio0 * mag2;
		z0 = dir.m_z * radio0 * mag2;
		y1 = dir.m_y * radio1 * mag2;
		z1 = dir.m_z * radio1 * mag2;
	}
	dgVector p0 ( height, y0, z0, dgFloat32 (0.0f));
	dgVector p1 (-height, y1, z1, dgFloat32 (0.0f));

	dgFloat32 dist0 = dir % p0;
	dgFloat32 dist1 = dir % p1;
	if (dist1 >= dist0) {
		p0 = p1;
	}
	return p0;
}

dgVector dgCollisionTaperedCylinder::ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const
{
	dgVector p (SupportVertex(dir, NULL));
	if (dgAbsf(dir.m_x) > dgFloat32 (0.9997f)) {
		p.m_y = point.m_y;
		p.m_z = point.m_z;
	} else {
		dgVector n (dir.m_x, dgSqrt (dir.m_y * dir.m_y + dir.m_z * dir.m_z), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgFloat32 project = n % m_dirVector;
		if (project > dgFloat32 (0.9998f)) {
			p.m_x = point.m_x;	
		}
	}
	return p;

}


dgInt32 dgCollisionTaperedCylinder::CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const
{
	dgAssert (dgAbsf (normal % normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	return CalculateContactsGeneric (point, normal, proxy, contactsOut);
}


dgFloat32 dgCollisionTaperedCylinder::CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const
{
	dgFloat32 volume = dgCollisionConvex::CalculateMassProperties (offset, inertia, crossInertia, centerOfMass);

/*
	centerOfMass = GetLocalMatrix().m_posit;
	volume = dgFloat32 (3.14159f * 2.0f) * m_radius * m_radius * m_height; 

	inertaxx   = dgFloat32 (0.5f) * m_radius *  m_radius * volume;
	inertayyzz = (dgFloat32 (0.25f) * m_radius *  m_radius + dgFloat32 (1.0f / 3.0f) * m_height * m_height) * volume;

	dgMatrix inertiaTensor (dgGetIdentityMatrix());

	inertiaTensor[0][0] = inertaxx;
	inertiaTensor[1][1] = inertayyzz;
	inertiaTensor[2][2] = inertayyzz;

	inertiaTensor = GetLocalMatrix().Inverse() * inertiaTensor * GetLocalMatrix();

	crossInertia.m_x = inertiaTensor[1][2] - volume * centerOfMass.m_y * centerOfMass.m_z;
	crossInertia.m_y = inertiaTensor[0][2] - volume * centerOfMass.m_z * centerOfMass.m_x;
	crossInertia.m_z = inertiaTensor[0][1] - volume * centerOfMass.m_x * centerOfMass.m_y;

	dgVector central (centerOfMass.CompProduct(centerOfMass));
	inertia.m_x = inertiaTensor[0][0] + volume * (central.m_y + central.m_z);
	inertia.m_y = inertiaTensor[1][1] + volume * (central.m_z + central.m_x);
	inertia.m_z = inertiaTensor[2][2] + volume * (central.m_x + central.m_y);

	centerOfMass = centerOfMass.Scale3 (volume);
*/
	return volume;
}


dgInt32 dgCollisionTaperedCylinder::CalculatePlaneIntersection (const dgVector& normal, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
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


void dgCollisionTaperedCylinder::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_taperedCylinder.m_radio0 = m_radio0;
	info->m_taperedCylinder.m_radio1 = m_radio1;
	info->m_cylinder.m_height = m_height * dgFloat32 (2.0f);
}


void dgCollisionTaperedCylinder::Serialize(dgSerialize callback, void* const userData) const
{
	dgVector size (m_radio0, m_radio1, m_height * dgFloat32 (2.0f), dgFloat32 (0.0f));
	SerializeLow(callback, userData);
	callback (userData, &size, sizeof (dgVector));
}

