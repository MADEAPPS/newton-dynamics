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
#include "dgCollisionChamferCylinder.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



dgInt32 dgCollisionChamferCylinder::m_shapeRefCount = 0;
dgVector dgCollisionChamferCylinder::m_yzMask (0, 0xffffffff, 0xffffffff, 0);
dgVector dgCollisionChamferCylinder::m_shapesDirs[DG_MAX_CHAMFERCYLINDER_DIR_COUNT];
dgConvexSimplexEdge dgCollisionChamferCylinder::m_edgeArray[(4 * DG_CHAMFERCYLINDER_SLICES + 2)* DG_CHAMFERCYLINDER_BRAKES];

dgCollisionChamferCylinder::dgCollisionChamferCylinder(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 radius, dgFloat32 height)
	:dgCollisionConvex(allocator, signature, m_chamferCylinderCollision)
{
	Init (radius, height);
}

dgCollisionChamferCylinder::dgCollisionChamferCylinder(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y);
}


dgCollisionChamferCylinder::~dgCollisionChamferCylinder()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}


void dgCollisionChamferCylinder::Init (dgFloat32 radius, dgFloat32 height)
{
	m_rtti |= dgCollisionChamferCylinder_RTTI;
	m_radius = dgAbsf (radius);
	m_height = dgAbsf (height * dgFloat32 (0.5f));

	dgFloat32 sliceAngle = dgFloat32 (0.0f);
	dgFloat32 sliceStep = dgPI  / DG_CHAMFERCYLINDER_SLICES; 
	dgFloat32 breakStep = dgPI2 / DG_CHAMFERCYLINDER_BRAKES;

	dgMatrix rot (dgPitchMatrix (breakStep));	
	dgInt32 index = 0;
	for (dgInt32 j = 0; j <= DG_CHAMFERCYLINDER_SLICES; j ++) {
		dgVector p0 (-m_height * dgCos(sliceAngle), dgFloat32 (0.0f), m_radius + m_height * dgSin(sliceAngle), dgFloat32 (0.0f));
		sliceAngle += sliceStep;
		for (dgInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) {
			m_vertex[index] = p0;
			index ++;
			p0 = rot.UnrotateVector (p0);
		}
	}

	m_edgeCount = (4 * DG_CHAMFERCYLINDER_SLICES + 2)* DG_CHAMFERCYLINDER_BRAKES;
	m_vertexCount = DG_CHAMFERCYLINDER_BRAKES * (DG_CHAMFERCYLINDER_SLICES + 1);
	dgCollisionConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) {
		dgPolyhedra polyhedra(m_allocator);
		dgInt32 wireframe[DG_CHAMFERCYLINDER_SLICES + 10];

		for (dgInt32 i = 0; i < DG_MAX_CHAMFERCYLINDER_DIR_COUNT; i ++) {
			dgMatrix matrix (dgPitchMatrix (dgFloat32 (dgPI2 * i) / DG_MAX_CHAMFERCYLINDER_DIR_COUNT));
			m_shapesDirs[i] = matrix.RotateVector (dgVector (dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));
		}


		dgInt32 index = 0;
		for (dgInt32 j = 0; j < DG_CHAMFERCYLINDER_SLICES; j ++) {
			dgInt32 index0 = index + DG_CHAMFERCYLINDER_BRAKES - 1;
			for (dgInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) {
				wireframe[0] = index;
				wireframe[1] = index0;
				wireframe[2] = index0 + DG_CHAMFERCYLINDER_BRAKES;
				wireframe[3] = index + DG_CHAMFERCYLINDER_BRAKES;

				index0 = index;
				index ++;
				polyhedra.AddFace (4, wireframe);
			}
		}

		for (dgInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) { 
			wireframe[i] = i;
		}
		polyhedra.AddFace (DG_CHAMFERCYLINDER_BRAKES, wireframe);

		for (dgInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) { 
			wireframe[i] = DG_CHAMFERCYLINDER_BRAKES * (DG_CHAMFERCYLINDER_SLICES + 1) - i - 1;
		}
		polyhedra.AddFace (DG_CHAMFERCYLINDER_BRAKES, wireframe);
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


void dgCollisionChamferCylinder::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgInt32 slices = 12;
	dgInt32 brakes = 24;
	dgFloat32 sliceAngle = dgFloat32 (0.0f);
	dgFloat32 sliceStep = dgPI  / slices; 
	dgFloat32 breakStep = dgPI2 / brakes;

	dgTriplex pool[24 * (12 + 1)];

	dgMatrix rot (dgPitchMatrix (breakStep));	
	dgInt32 index = 0;
	for (dgInt32 j = 0; j <= slices; j ++) {
		dgVector p0 (-m_height * dgCos(sliceAngle), dgFloat32 (0.0f), m_radius + m_height * dgSin(sliceAngle), dgFloat32 (0.0f));
		sliceAngle += sliceStep;
		for (dgInt32 i = 0; i < brakes; i ++) {
			pool[index].m_x = p0.m_x;
			pool[index].m_y = p0.m_y;
			pool[index].m_z = p0.m_z;
			index ++;
			p0 = rot.UnrotateVector (p0);
		}
	}

	//dgMatrix matrix (GetLocalMatrix() * matrixPtr);
	matrix.TransformTriplex (&pool[0].m_x, sizeof (dgTriplex), &pool[0].m_x, sizeof (dgTriplex), 24 * (12 + 1));

	dgTriplex face[32];

	index = 0;
	for (dgInt32 j = 0; j < slices; j ++) {
		dgInt32 index0 = index + brakes - 1;
		for (dgInt32 i = 0; i < brakes; i ++) {
			face[0] = pool[index];
			face[1] = pool[index0];
			face[2] = pool[index0 + brakes];
			face[3] = pool[index + brakes];
			index0 = index;
			index ++;
			callback (userData, 4, &face[0].m_x, 0);
		}
	}

	for (dgInt32 i = 0; i < brakes; i ++) { 
		face[i] = pool[i];
	}
	callback (userData, 24, &face[0].m_x, 0);

	for (dgInt32 i = 0; i < brakes; i ++) { 
		face[i] = pool[brakes * (slices + 1) - i - 1];
	}
	callback (userData, 24, &face[0].m_x, 0);
}


dgInt32 dgCollisionChamferCylinder::CalculateSignature (dgFloat32 radius, dgFloat32 height)
{
	dgUnsigned32 buffer[3];

	memset (buffer, 0, sizeof (buffer));
	buffer[0] = m_chamferCylinderCollision;
	buffer[1] = dgCollision::Quantize (radius);
	buffer[2] = dgCollision::Quantize (height);
	return dgInt32 (dgCollision::Quantize(buffer, sizeof (buffer)));
}

dgInt32 dgCollisionChamferCylinder::CalculateSignature () const
{
	return CalculateSignature (m_radius, m_height);
}


void dgCollisionChamferCylinder::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}


void dgCollisionChamferCylinder::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_chamferCylinder.m_r = m_radius;
	info->m_chamferCylinder.m_height = m_height * dgFloat32 (2.0f);
}

void dgCollisionChamferCylinder::Serialize(dgSerialize callback, void* const userData) const
{
	dgVector size (m_radius, m_height * dgFloat32 (2.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	SerializeLow(callback, userData);
	callback (userData, &size, sizeof (dgVector));
}



dgFloat32 dgCollisionChamferCylinder::RayCast (const dgVector& q0, const dgVector& q1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	if (q0.m_x > m_height) {
		if (q1.m_x < m_height) {
			dgFloat32 t1 = (m_height - q0.m_x) / (q1.m_x - q0.m_x);
			dgFloat32 y = q0.m_y + (q1.m_y - q0.m_y) * t1;
			dgFloat32 z = q0.m_z + (q1.m_z - q0.m_z) * t1;
			if ((y * y + z * z) < m_radius * m_radius) {
				contactOut.m_normal = dgVector (dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				//contactOut.m_userId = SetUserDataID();
				return t1; 
			}
		}
	}

	if (q0.m_x < -m_height) {
		if (q1.m_x > -m_height) {
			dgFloat32 t1 = (-m_height - q0.m_x) / (q1.m_x - q0.m_x);
			dgFloat32 y = q0.m_y + (q1.m_y - q0.m_y) * t1;
			dgFloat32 z = q0.m_z + (q1.m_z - q0.m_z) * t1;
			if ((y * y + z * z) < m_radius * m_radius) {
				contactOut.m_normal = dgVector (dgFloat32 (-1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				//contactOut.m_userId = SetUserDataID();
				return t1; 
			}
		}
	}

	dgVector dq ((q1 - q0) & dgVector::m_triplexMask);
	
	// avoid NaN as a result of a division by zero
	if ((dq % dq) <= 0.0f) {
		return dgFloat32(1.2f);
	}

	//dgVector dir (dq.Scale3 (dgRsqrt(dq % dq)));
	dgVector dir (dq.CompProduct4 (dq.InvMagSqrt()));
	if (dgAbsf (dir.m_x) > 0.9999f) {
		return dgCollisionConvex::RayCast (q0, q1, maxT, contactOut, NULL, NULL, NULL);
	}

	dgVector p0 (q0);
	dgVector p1 (q1);
	dgVector dp (dq);

	p0.m_x = dgFloat32 (0.0f);
	p1.m_x = dgFloat32 (0.0f);
	dp.m_x = dgFloat32 (0.0f);

	dgFloat32 a = dp % dp;
	//dgFloat32 b = dgFloat32 (2.0f) * (p0 % dp);
	dgFloat32 b = dgFloat32 (2.0f) * (p0.DotProduct4(dp).GetScalar());
	dgFloat32 r = m_radius + m_height;
	dgFloat32 c = p0 % p0 - r * r;
	dgFloat32 desc = b * b - dgFloat32 (4.0f) * a * c;
	if (desc >= 0.0f) {
		desc = dgSqrt (desc);
		dgFloat32 den = dgFloat32 (0.5f) / a;
		dgFloat32 s0 = (-b + desc) * den;
		dgFloat32 s1 = (-b - desc) * den;

		dgVector origin0 (p0 + dp.Scale4 (s0));
		dgVector origin1 (p0 + dp.Scale4 (s1));
		dgFloat32 s = m_radius / (m_radius + m_height);
		origin0 = origin0.Scale4 (s);
		origin1 = origin1.Scale4 (s);
		dgFloat32 t0 = dgRayCastSphere (q0, q1, origin0, m_height);
		dgFloat32 t1 = dgRayCastSphere (q0, q1, origin1, m_height);
		if (t0 < t1) {
			if ((t0 >= 0.0f) && (t0 <= 1.0f)) {
				contactOut.m_normal = q0 + dq.Scale4 (t0) - origin0;
				dgAssert (contactOut.m_normal.m_w == dgFloat32 (0.0f));
				//contactOut.m_normal = contactOut.m_normal.Scale3 (dgRsqrt (contactOut.m_normal % contactOut.m_normal));
				contactOut.m_normal = contactOut.m_normal.CompProduct4(contactOut.m_normal.DotProduct4(contactOut.m_normal).InvSqrt());
				//contactOut.m_userId = SetUserDataID();
				return t0; 
			}
		} else {
			if ((t1 >= 0.0f) && (t1 <= 1.0f)) {
				contactOut.m_normal = q0 + dq.Scale4 (t1) - origin1;
				dgAssert (contactOut.m_normal.m_w == dgFloat32 (0.0f));
				//contactOut.m_normal = contactOut.m_normal.Scale3 (dgRsqrt (contactOut.m_normal % contactOut.m_normal));
				contactOut.m_normal = contactOut.m_normal.CompProduct4(contactOut.m_normal.DotProduct4(contactOut.m_normal).InvSqrt());
				//contactOut.m_userId = SetUserDataID();
				return t1; 
			}
		}
	}

	return dgFloat32 (1.2f);
}


dgVector dgCollisionChamferCylinder::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	dgFloat32 x = dir.GetScalar();
	if (dgAbsf (x) > dgFloat32 (0.9999f)) {
		dgFloat32 x0 = (x >= dgFloat32 (0.0f)) ? m_height : -m_height;
		return dgVector (x0, dgFloat32 (0.0f), m_radius, dgFloat32 (0.0f)); 
	}

//	dgVector sideDir (dgFloat32 (0.0f), dir.m_y, dir.m_z, dgFloat32 (0.0f));
	dgVector sideDir (m_yzMask & dir);
//	sideDir = sideDir.Scale3 (m_radius * dgRsqrt (sideDir % sideDir + dgFloat32 (1.0e-18f)));
	sideDir = sideDir.CompProduct4(sideDir.InvMagSqrt());
//	return sideDir + dir.Scale3 (m_height);
	return sideDir.Scale4(m_radius) + dir.Scale4 (m_height);
}


dgVector dgCollisionChamferCylinder::ConvexConicSupporVertex (const dgVector& dir) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	dgFloat32 x = dir.GetScalar();
	//if (dgAbsf (dir.m_x) > dgFloat32 (0.99995f)) {
	if (dgAbsf (x) > dgFloat32 (0.99995f)) {
		return dgVector (dgFloat32 (0.0f), m_radius, dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	}

	//dgVector sideDir (dgFloat32 (0.0f), dir.m_y, dir.m_z, dgFloat32 (0.0f));
	dgVector sideDir (m_yzMask & dir);
	dgAssert ((sideDir % sideDir) > dgFloat32 (0.0f));
	//return sideDir.Scale3 (m_radius * dgRsqrt (sideDir % sideDir));
	return sideDir.CompProduct4(sideDir.InvMagSqrt()).Scale4(m_radius);
}

dgVector dgCollisionChamferCylinder::ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const
{
	dgAssert (dir.m_w == 0.0f);
	return point + dir.Scale4(m_height);
}


dgInt32 dgCollisionChamferCylinder::CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const
{
	dgFloat32 disc2 = point.m_y * point.m_y + point.m_z * point.m_z;
	if (disc2 < m_radius * m_radius) {
		dgVector cylinderNormal ((point.m_x >= dgFloat32 (0.0)) ? dgFloat32 (-1.0f) : dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		return CalculateContactsGeneric (point, cylinderNormal, proxy, contactsOut);
	} else {

		dgVector r (dgFloat32 (0.0f), point.m_y, point.m_z, dgFloat32 (0.0f));
		dgAssert ((r % r) > dgFloat32 (0.0f));
//		r = r.Scale3(m_radius * dgRsqrt (r % r));
		r = r.CompProduct4(r.InvMagSqrt()).Scale4(m_radius);
		//dgFloat32 t = normal % (r - point);
		dgVector t (normal.DotProduct4(r - point));
		//contactsOut[0] = r - normal.Scale3 (t);
		contactsOut[0] = r - normal.CompProduct4(t);
		return 1;
	}
}




dgInt32 dgCollisionChamferCylinder::CalculatePlaneIntersection (const dgVector& normal, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgInt32 count = 0;
	if (dgAbsf (normal.m_x) < dgFloat32 (0.999f)) { 
		count = 1;
		contactsOut[0] = SupportVertex (normal, NULL);
	} else {
		count = dgCollisionConvex::CalculatePlaneIntersection (normal, origin, contactsOut, normalSign);
	}
	return count;
}
