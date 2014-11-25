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
#include "dgCollisionSphere.h"
#include "dgCollisionConvexPolygon.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


#define DG_SPHERE_EDGE_COUNT 96

dgInt32 dgCollisionSphere::m_shapeRefCount = 0;
dgVector dgCollisionSphere::m_unitSphere[DG_SPHERE_VERTEX_COUNT];
dgConvexSimplexEdge dgCollisionSphere::m_edgeArray[DG_SPHERE_EDGE_COUNT];


dgCollisionSphere::dgCollisionSphere(dgMemoryAllocator* const allocator, dgUnsigned32 signature, dgFloat32 radii)
	:dgCollisionConvex(allocator, signature, m_sphereCollision) 
{
	Init (radii, allocator);
}

dgCollisionSphere::dgCollisionSphere(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgFloat32 radios;
	deserialization (userData, &radios, sizeof (radios));
	Init (radios, world->GetAllocator());
}

dgCollisionSphere::~dgCollisionSphere()
{
	m_shapeRefCount --;
	dgAssert (m_shapeRefCount >= 0);

	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}


void dgCollisionSphere::Init (dgFloat32 radius, dgMemoryAllocator* allocator)
{
	m_rtti |= dgCollisionSphere_RTTI;
	m_radius = radius;

	m_edgeCount = DG_SPHERE_EDGE_COUNT;
	m_vertexCount = DG_SPHERE_VERTEX_COUNT;
	dgCollisionConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) {

		dgInt32 indexList[256];
		dgVector tmpVectex[256];

		dgVector p0 ( dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
		dgVector p1 (-dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
		dgVector p2 ( dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
		dgVector p3 ( dgFloat32 (0.0f),-dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector p4 ( dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f));
		dgVector p5 ( dgFloat32 (0.0f), dgFloat32 (0.0f),-dgFloat32 (1.0f), dgFloat32 (0.0f));

		dgInt32 i = 1;
		dgInt32 count = 0;
		TesselateTriangle (i, p4, p0, p2, count, tmpVectex);
		TesselateTriangle (i, p4, p2, p1, count, tmpVectex);
		TesselateTriangle (i, p4, p1, p3, count, tmpVectex);
		TesselateTriangle (i, p4, p3, p0, count, tmpVectex);
		TesselateTriangle (i, p5, p2, p0, count, tmpVectex);
		TesselateTriangle (i, p5, p1, p2, count, tmpVectex);
		TesselateTriangle (i, p5, p3, p1, count, tmpVectex);
		TesselateTriangle (i, p5, p0, p3, count, tmpVectex);

		//dgAssert (count == EDGE_COUNT);
		dgInt32 vertexCount = dgVertexListToIndexList (&tmpVectex[0].m_x, sizeof (dgVector), 3 * sizeof (dgFloat32), 0, count, indexList, 0.001f); 

		dgAssert (vertexCount == DG_SPHERE_VERTEX_COUNT);
		for (dgInt32 i = 0; i < vertexCount; i ++) {
			m_unitSphere[i] = tmpVectex[i];
		}
		dgPolyhedra polyhedra(m_allocator);

		polyhedra.BeginFace();
		for (dgInt32 i = 0; i < count; i += 3) {
#ifdef _DEBUG
			dgEdge* const edge = polyhedra.AddFace (indexList[i],  indexList[i + 1], indexList[i + 2]);
			dgAssert (edge);
#else 
			polyhedra.AddFace (indexList[i],  indexList[i + 1], indexList[i + 2]);
#endif
		}
		polyhedra.EndFace();

		dgUnsigned64 i1 = 0;
		dgPolyhedra::Iterator iter (polyhedra);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			edge->m_userData = i1;
			i1 ++;
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

	for (dgInt32 i = 0; i < DG_SPHERE_VERTEX_COUNT; i ++) {
		m_vertex[i] = m_unitSphere[i].Scale4 (m_radius);
	}

	m_shapeRefCount ++;
	dgCollisionConvex::m_simplex = m_edgeArray;
	SetVolumeAndCG ();
}


dgVector dgCollisionSphere::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	dgAssert (dir.m_w == 0.0f);
	return dir.Scale4 (m_radius);
}



void dgCollisionSphere::TesselateTriangle (dgInt32 level, const dgVector& p0, const dgVector& p1, const dgVector& p2, dgInt32& count, dgVector* ouput) const
{
	if (level) {
		dgAssert (dgAbsf (p0 % p0 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p1 % p1 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p2 % p2 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgVector p01 (p0 + p1);
		dgVector p12 (p1 + p2);
		dgVector p20 (p2 + p0);

		p01 = p01.CompProduct4(p01.InvMagSqrt());
		p12 = p12.CompProduct4(p12.InvMagSqrt());
		p20 = p20.CompProduct4(p20.InvMagSqrt());

		dgAssert (dgAbsf (p01 % p01 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p12 % p12 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
		dgAssert (dgAbsf (p20 % p20 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));

		TesselateTriangle (level - 1, p0,  p01, p20, count, ouput);
		TesselateTriangle (level - 1, p1,  p12, p01, count, ouput);
		TesselateTriangle (level - 1, p2,  p20, p12, count, ouput);
		TesselateTriangle (level - 1, p01, p12, p20, count, ouput);

	} else {
		ouput[count ++] = p0;
		ouput[count ++] = p1;
		ouput[count ++] = p2;
	}
}

void dgCollisionSphere::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}

dgInt32 dgCollisionSphere::CalculateSignature (dgFloat32 radius)
{
	dgUnsigned32 buffer[2];
	radius = dgAbsf (radius);

	buffer[0] = m_sphereCollision;
	buffer[1] = Quantize (radius);
	return Quantize(buffer, sizeof (buffer));
}

dgInt32 dgCollisionSphere::CalculateSignature () const
{
	return CalculateSignature(m_radius);
}

void dgCollisionSphere::CalcAABB (const dgMatrix& matrix, dgVector &p0, dgVector &p1) const
{
	dgVector size (matrix.m_front.Abs().Scale4(m_radius) + matrix.m_up.Abs().Scale4(m_radius) + matrix.m_right.Abs().Scale4(m_radius));
	p0 = (matrix[3] - size) & dgVector::m_triplexMask;
	p1 = (matrix[3] + size) & dgVector::m_triplexMask;
}


dgInt32 dgCollisionSphere::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgAssert (normal.m_w == 0.0f);
	dgAssert ((normal % normal) > dgFloat32 (0.999f));
	//contactsOut[0] = normal.Scale3 (normal % point);
	contactsOut[0] = normal.CompProduct4 (normal.DotProduct4(point));
	return 1;
}



void dgCollisionSphere::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTriplex pool[1024 * 2];
	dgVector tmpVectex[1024 * 2];

	dgVector p0 ( dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	dgVector p1 (-dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	dgVector p2 ( dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
	dgVector p3 ( dgFloat32 (0.0f),-dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector p4 ( dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f));
	dgVector p5 ( dgFloat32 (0.0f), dgFloat32 (0.0f),-dgFloat32 (1.0f), dgFloat32 (0.0f));

	dgInt32 i = 3;
	dgInt32 count = 0;
	TesselateTriangle (i, p4, p0, p2, count, tmpVectex);
	TesselateTriangle (i, p4, p2, p1, count, tmpVectex);
	TesselateTriangle (i, p4, p1, p3, count, tmpVectex);
	TesselateTriangle (i, p4, p3, p0, count, tmpVectex);
	TesselateTriangle (i, p5, p2, p0, count, tmpVectex);
	TesselateTriangle (i, p5, p1, p2, count, tmpVectex);
	TesselateTriangle (i, p5, p3, p1, count, tmpVectex);
	TesselateTriangle (i, p5, p0, p3, count, tmpVectex);

	for (dgInt32 i = 0; i < count; i ++) {
		tmpVectex[i] = tmpVectex[i].Scale4 (m_radius);
	}

	//dgMatrix matrix (GetLocalMatrix() * matrixPtr);
	matrix.TransformTriplex (&pool[0].m_x, sizeof (dgTriplex), &tmpVectex[0].m_x, sizeof (dgVector), count);
	for (dgInt32 i = 0; i < count; i += 3) {
		callback (userData, 3, &pool[i].m_x, 0);
	}
}

dgFloat32 dgCollisionPoint::GetVolume () const
{
	dgAssert (0);
	return dgFloat32 (0.0f); 
}


dgVector dgCollisionPoint::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	return dgVector (dgFloat32 (0.0f)); 
}


dgFloat32 dgCollisionSphere::RayCast (const dgVector& p0, const dgVector& p1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	dgFloat32 t = dgRayCastSphere (p0, p1, dgVector (dgFloat32 (0.0f)), m_radius);
	if (t < maxT) {
		dgVector contact (p0 + (p1 - p0).Scale3 (t));
		contactOut.m_normal = contact.Scale3 (dgRsqrt (contact % contact));
		//contactOut.m_userId = SetUserDataID();
	}
	return t;
}


void dgCollisionSphere::MassProperties () 
{
	m_centerOfMass = dgVector (dgFloat32 (0.0f));
	m_crossInertia = dgVector (dgFloat32 (0.0f));
	dgFloat32 volume = dgFloat32 (4.0f * 3.141592f / 3.0f) * m_radius *  m_radius * m_radius; 
	dgFloat32 II = dgFloat32 (2.0f / 5.0f) * m_radius *  m_radius;

//dgCollisionConvex::MassProperties ();
	m_inertia = dgVector  (II, II, II, dgFloat32 (0.0f));
	m_centerOfMass.m_w = volume;
}


void dgCollisionSphere::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);
	info->m_sphere.m_radius = m_radius;
}

void dgCollisionSphere::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	callback (userData, &m_radius, sizeof (m_radius));
}


dgVector dgCollisionSphere::ConvexConicSupporVertex (const dgVector& dir) const 
{
	return dgVector (dgFloat32 (0.0f));
}

dgVector dgCollisionSphere::ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const
{
	return SupportVertex(dir, NULL);
}

dgInt32 dgCollisionSphere::CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const
{
	dgAssert (normal.m_w == 0.0f);
	//contactsOut[0] = normal.Scale3(normal % point);
	contactsOut[0] = normal.CompProduct4 (normal.DotProduct4(point));
	return 1;
}

