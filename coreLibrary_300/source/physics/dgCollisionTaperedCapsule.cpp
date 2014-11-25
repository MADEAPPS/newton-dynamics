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
#include "dgCollisionTaperedCapsule.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



dgInt32 dgCollisionTaperedCapsule::m_shapeRefCount = 0;
dgConvexSimplexEdge dgCollisionTaperedCapsule::m_edgeArray[DG_CAPSULE_SEGMENTS * (6 + 8 * (DG_CAP_SEGMENTS - 1))];

dgCollisionTaperedCapsule::dgCollisionTaperedCapsule(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height)
	:dgCollisionConvex(allocator, signature, m_taperedCapsuleCollision)
{
	Init (radio0, radio1, height);
}

dgCollisionTaperedCapsule::dgCollisionTaperedCapsule(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y, size.m_z);
}


dgCollisionTaperedCapsule::~dgCollisionTaperedCapsule()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}

void dgCollisionTaperedCapsule::Init (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height)
{
	m_rtti |= dgCollisionTaperedCapsule_RTTI;

	m_radio0 = dgAbsf (radio0);
	m_radio1 = dgAbsf (radio1);
	m_height = dgAbsf (height * 0.5f);

	m_clip1 = dgFloat32 (0.0f);
	m_clip0 = dgFloat32 (0.0f);
	dgFloat32 angle0 = dgFloat32 (-3.141592f / 2.0f);
	dgFloat32 angle1 = dgFloat32 ( 3.141592f / 2.0f);
	do {
		dgFloat32 angle = (angle1 + angle0) * dgFloat32 (0.5f);
		dgVector dir (dgSin (angle), dgCos (angle), dgFloat32 (0.0f), dgFloat32 (0.0f));

		dgVector p0(dir.Scale3 (m_radio0));
		dgVector p1(dir.Scale3 (m_radio1));
		p0.m_x += m_height;
		p1.m_x -= m_height;
		dgFloat32 dir0 = p0 % dir;
		dgFloat32 dir1 = p1 % dir;
		if (dir0 > dir1) {
			angle1 = angle;
			m_clip0 = p0.m_x - m_height;
		} else {
			angle0 = angle;
			m_clip1 = p1.m_x + m_height;
		}
	} while ((angle1 - angle0) > dgFloat32 (0.001f * 3.141592f/180.0f));

	dgFloat32 angle = (angle1 + angle0) * dgFloat32 (0.5f);
	m_sideNormal = dgVector (dgSin (angle), dgCos (angle), dgFloat32 (0.0f), dgFloat32 (0.0f));

	m_clipRadio0 = dgSqrt (m_radio0 * m_radio0 - m_clip0 * m_clip0);
	m_clipRadio1 = dgSqrt (m_radio1 * m_radio1 - m_clip1 * m_clip1);

	dgInt32 i1 = 0;
	dgInt32 i0 = DG_CAPSULE_SEGMENTS * DG_CAP_SEGMENTS * 2;

	dgFloat32 dx0 = (m_clip0 - m_radio0) / DG_CAP_SEGMENTS;
	dgFloat32 x0 =  m_radio0 + dx0;
	
	dgFloat32 dx1 = (m_clip1 + m_radio1) / DG_CAP_SEGMENTS;
	dgFloat32 x1 =  -m_radio1 + dx1;
	for (dgInt32 j = 0; j < DG_CAP_SEGMENTS; j ++) {
		dgFloat32 angle = dgFloat32 (0.0f);
		dgFloat32 r0 = dgSqrt (m_radio0 * m_radio0 - x0 * x0);
		dgFloat32 r1 = dgSqrt (m_radio1 * m_radio1 - x1 * x1);

		i0 -= DG_CAPSULE_SEGMENTS;
		for (dgInt32 i = 0; i < DG_CAPSULE_SEGMENTS; i ++) {
			dgFloat32 z = dgSin (angle);
			dgFloat32 y = dgCos (angle);
			m_vertex[i0] = dgVector ( m_height + x0, y * r0, z * r0, dgFloat32 (0.0f));
			m_vertex[i1] = dgVector (-m_height + x1, y * r1, z * r1, dgFloat32 (0.0f));
			i0 ++;
			i1 ++;
			angle += dgPI2 / DG_CAPSULE_SEGMENTS;
		}
		x0 += dx0;
		x1 += dx1;
		i0 -= DG_CAPSULE_SEGMENTS;
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


dgInt32 dgCollisionTaperedCapsule::CalculateSignature (dgFloat32 radio0, dgFloat32 radio1, dgFloat32 height)
{
	dgUnsigned32 buffer[4];

	buffer[0] = m_taperedCapsuleCollision;
	buffer[1] = Quantize (radio0);
	buffer[2] = Quantize (radio1);
	buffer[3] = Quantize (height);
	return Quantize(buffer, sizeof (buffer));
}

dgInt32 dgCollisionTaperedCapsule::CalculateSignature () const
{
	return CalculateSignature (m_radio0, m_radio1, m_height);
}

void dgCollisionTaperedCapsule::TesselateTriangle (dgInt32 level, const dgVector& p0, const dgVector& p1, const dgVector& p2, dgInt32& count, dgVector* ouput) const
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
		ouput[count + 0] = p0.Scale3 (m_radio0);
		ouput[count + 1] = p1.Scale3 (m_radio0);
		ouput[count + 2] = p2.Scale3 (m_radio0);
		count += 3;
	}
}



void dgCollisionTaperedCapsule::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	#define POWER 2
	dgVector tmpVectex[1024 * 2];

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

	dgFloat32 scale = m_radio1 / m_radio0;
	dgVector edgeP0(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector edgeP1(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	for (dgInt32 i = 0; i < count; i += 3) {
		dgVector face0[4]; 	
		dgVector face1[4]; 	
		dgInt32 n0 = 0;
		dgInt32 n1 = 0;
		dgVector p0 (tmpVectex[i + 2]);
		for (dgInt32 j = 0; j < 3; j ++) {
			dgVector p1 (tmpVectex[i + j]);
			if (p1.m_x > m_clip0) {
				if (p0.m_x < m_clip0) {
					dgFloat32 t = (m_clip0 - p0.m_x) / (p1.m_x - p0.m_x);
					edgeP0 = p0 + (p1 - p0).Scale3 (t);
					edgeP0.m_x = m_clip0;

					face0[n0] = edgeP0;
					face0[n0].m_x += m_height;
					n0 ++;

					face1[n1] = edgeP0.Scale3 (scale);
					face1[n1].m_x -= m_height;
					n1 ++;
				}
				face0[n0] = p1;
				face0[n0].m_x += m_height;
				n0 ++;
			} else {
				if (p0.m_x > m_clip0) {
					dgFloat32 t = (m_clip0 - p0.m_x) / (p1.m_x - p0.m_x);
					edgeP1 = p0 + (p1 - p0).Scale3 (t);
					edgeP1.m_x = m_clip0;

					face0[n0] = edgeP1;
					face0[n0].m_x += m_height;
					n0 ++;

					face1[n1] = edgeP1.Scale3 (scale);
					face1[n1].m_x -= m_height;
					n1 ++;
				}
				face1[n1] = p1.Scale3 (scale);;
				face1[n1].m_x -= m_height;
				
				n1 ++;
			}
			p0 = p1;
		}

		dgTriplex face[4];
		if (n0) {
			matrix.TransformTriplex (&face[0].m_x, sizeof (dgTriplex), &face0[0].m_x, sizeof (dgVector), n0);			
			callback (userData, n0, &face[0].m_x, 0);
		}
		if (n1) {
			matrix.TransformTriplex (&face[0].m_x, sizeof (dgTriplex), &face1[0].m_x, sizeof (dgVector), n1);
			callback (userData, n1, &face[0].m_x, 0);
		}
		if (n0 && n1) {
			face0[0] = edgeP0;
			face0[1] = edgeP1;
			face0[2] = edgeP1.Scale3 (scale);
			face0[3] = edgeP0.Scale3 (scale);
			face0[0].m_x += m_height;
			face0[1].m_x += m_height;
			face0[2].m_x -= m_height;
			face0[3].m_x -= m_height;

			dgPlane plane (face0[0], face0[1], face0[2]);
			if (plane.m_w >= dgFloat32 (0.0f)) {
				dgAssert (0);
			}

			matrix.TransformTriplex (&face[0].m_x, sizeof (dgTriplex), &face0[0].m_x, sizeof (dgVector), 4);
			callback (userData, 4, &face[0].m_x, 0);
		}
	}
}


void dgCollisionTaperedCapsule::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}


dgVector dgCollisionTaperedCapsule::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	dgVector p0(dir.Scale3 (m_radio0));
	dgVector p1(dir.Scale3 (m_radio1));
	p0.m_x += m_height;
	p1.m_x -= m_height;
	dgFloat32 dir0 = p0 % dir;
	dgFloat32 dir1 = p1 % dir;
	if (dir1 > dir0) {
		p0 = p1;
	}
	return p0;
}



dgFloat32 dgCollisionTaperedCapsule::RayCast (const dgVector& q0, const dgVector& q1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	dgVector origin0 ( m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector origin1 (-m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgFloat32 t0 = dgRayCastSphere (q0, q1, origin0, m_radio0);
	dgFloat32 t1 = dgRayCastSphere (q0, q1, origin1, m_radio1);
//	if ((t0 < dgFloat32 (1.2f)) && (t1 < dgFloat32 (1.2f))) {
	if ((t0 < maxT) && (t1 < maxT)) {
		if (t0 < t1) {
			dgVector q (q0 + (q1 - q0).Scale4 (t0));
			dgVector n (q - origin0); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
//			contactOut.m_userId = SetUserDataID();
			return t0;
		} else {
			dgVector q (q0 + (q1 - q0).Scale4 (t1));
			dgVector n (q - origin1); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
//			contactOut.m_userId = SetUserDataID();
			return t1;
		}
//	} else if (t0 < dgFloat32 (1.2f)) {
	} else if (t0 < maxT) {
		dgVector q (q0 + (q1 - q0).Scale4 (t0));
		if (q.m_x >= (m_height + m_clip0)) {
			dgVector n (q - origin0); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
//			contactOut.m_userId = SetUserDataID();
			return t0;
		}

//	} else if (t1 < dgFloat32 (1.2f)) {
	} else if (t1 < maxT) {
		dgVector q (q0 + (q1 - q0).Scale4 (t1));
		if (q.m_x <= (-m_height + m_clip1)) {
			dgVector n (q - origin1); 
			dgAssert (n.m_w == dgFloat32 (0.0f));
			//contactOut.m_normal = n.Scale3 (dgRsqrt (n % n));
			contactOut.m_normal = n.CompProduct4(n.DotProduct4(n).InvSqrt());
//			contactOut.m_userId = SetUserDataID();
			return t1;
		}
	}
	return dgCollisionConvex::RayCast (q0, q1, maxT, contactOut, NULL, NULL, NULL);

}



dgFloat32 dgCollisionTaperedCapsule::CalculateMassProperties (const dgMatrix& offset, dgVector& inertia, dgVector& crossInertia, dgVector& centerOfMass) const
{
	dgFloat32 volume = dgCollisionConvex::CalculateMassProperties (offset, inertia, crossInertia, centerOfMass);
/*
	centerOfMass = GetLocalMatrix().m_posit;
	dgFloat32 cylVolume = dgFloat32 (3.14159f * 2.0f) * m_radio0 * m_radio0 * m_height;
	dgFloat32 sphVolume = dgFloat32 (3.14159f * 4.0f / 3.0f) * m_radio0 * m_radio0 * m_radio0; 

	dgFloat32 cylInertiaxx = (dgFloat32 (0.5f) * m_radio0 * m_radio0) * cylVolume;
	dgFloat32 sphInertiaxx = (dgFloat32 (2.0f / 5.0f) * m_radio0 * m_radio0) * sphVolume;

	dgFloat32 cylInertiayyzz = (dgFloat32 (0.25f) * m_radio0 *  m_radio0 + dgFloat32 (1.0f / 3.0f) * m_height * m_height) * cylVolume;
	dgFloat32 sphInertiayyzz = sphInertiaxx + m_height * m_height * sphVolume;

	dgFloat32 volume = cylVolume + sphVolume;
	dgFloat32 inertiaxx = cylInertiaxx + sphInertiaxx;
	dgFloat32 inertiayyzz = cylInertiayyzz + sphInertiayyzz;

	dgMatrix inertiaTensor (dgGetIdentityMatrix());

	inertiaTensor[0][0] = inertiaxx;
	inertiaTensor[1][1] = inertiayyzz;
	inertiaTensor[2][2] = inertiayyzz;

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




void dgCollisionTaperedCapsule::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_taperedCapsule.m_radio0 = m_radio0;
	info->m_taperedCapsule.m_radio1 = m_radio1;
	info->m_capsule.m_height = dgFloat32 (2.0f) * m_height;
}


void dgCollisionTaperedCapsule::Serialize(dgSerialize callback, void* const userData) const
{
	dgVector size (m_radio0, m_radio1, dgFloat32 (2.0f) * m_height, dgFloat32 (0.0f));

	SerializeLow(callback, userData);
	callback (userData, &size, sizeof (dgVector));
}


dgVector dgCollisionTaperedCapsule::ConvexConicSupporVertex (const dgVector& dir) const
{
	dgVector p0(dir.Scale3 (m_radio0));
	dgVector p1(dir.Scale3 (m_radio1));
	p0.m_x += m_height;
	p1.m_x -= m_height;
	dgFloat32 dir0 = p0 % dir;
	dgFloat32 dir1 = p1 % dir;
	return dgVector ((dir0 >= dir1) ? m_height : - m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
}

dgVector dgCollisionTaperedCapsule::ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const
{
	dgAssert (dgAbsf (dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	dgVector p (SupportVertex(dir, NULL));

	dgVector n (dir.m_x, dgSqrt (dir.m_y * dir.m_y + dir.m_z * dir.m_z), dgFloat32 (0.0), dgFloat32 (0.0f));
	dgAssert (dgAbsf (n % n - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	
	dgFloat32 project = m_sideNormal % n;
	if (project > dgFloat32 (0.9998f)) {
		dgFloat32 t = dgFloat32 (0.5f) * (point.m_x + m_height) / m_height;
		dgFloat32 r = m_radio1 + (m_radio0 - m_radio1) * t;
		p = dir.Scale3 (r);
		p.m_x += point.m_x;
	}
	return p;
}

dgInt32 dgCollisionTaperedCapsule::CalculateSphereConicContacts (dgFloat32 posit, dgFloat32 radius, const dgVector& normal, const dgVector& point, dgVector* const contact) const
{
	dgVector r (posit, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgFloat32 t = normal % (r - point);
	contact[0] = r - normal.Scale3 (t);
	return 1;
}


dgInt32 dgCollisionTaperedCapsule::CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const
{
	dgVector n (-normal.m_x, -dgSqrt (normal.m_y * normal.m_y + normal.m_z * normal.m_z), dgFloat32 (0.0), dgFloat32 (0.0f));
	dgFloat32 project = m_sideNormal % n;
	if (project > dgFloat32 (0.9998f)) {
		return CalculateContactsGeneric (point, normal, proxy, contactsOut);
	} else if (point.m_x > (m_height + m_clip0)){
		return CalculateSphereConicContacts ( m_height, m_radio0, normal, point, contactsOut);
	} else if (point.m_x < (-m_height + m_clip1)) {
		return CalculateSphereConicContacts (-m_height, m_radio1, normal, point, contactsOut);
	}
	return CalculateContactsGeneric (point, normal, proxy, contactsOut);
}



dgInt32 dgCollisionTaperedCapsule::CalculatePlaneIntersection (const dgVector& normal, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgInt32 count = 0;
	dgVector p0 (m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector dir0 (p0 - origin);
	dgFloat32 dist0 = dir0 % normal;
	if ((dist0 * dist0) < (m_radio0 * m_radio0)) {
		contactsOut[count] = p0 - normal.Scale3 (dist0);
		count ++;
	}

	dgVector p1 (-m_height, dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector dir1 (p1 - origin);
	dgFloat32 dist1 = dir1 % normal;
	if ((dist1 * dist1) < (m_radio1 * m_radio1)) {
		contactsOut[count] = p1 - normal.Scale3 (dist1);
		count ++;
	}
	return count;
}

