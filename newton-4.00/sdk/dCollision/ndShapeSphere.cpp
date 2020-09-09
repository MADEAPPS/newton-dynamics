/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeSphere.h"
#include "ndContactSolver.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define D_SPHERE_EDGE_COUNT 96

dInt32 ndShapeSphere::m_shapeRefCount = 0;
dVector ndShapeSphere::m_unitSphere[D_SPHERE_VERTEX_COUNT];
ndShapeConvex::ndConvexSimplexEdge ndShapeSphere::m_edgeArray[D_SPHERE_EDGE_COUNT];

#if 0



ndShapeSphere::ndShapeSphere(dgMemoryAllocator* const allocator, dgUnsigned32 signature, dFloat32 radii)
	:dgCollisionConvex(allocator, signature, m_sphereCollision) 
{
	Init (radii, allocator);
}

ndShapeSphere::ndShapeSphere(dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber)
	:dgCollisionConvex (world, deserialization, userData, revisionNumber)
{
	dFloat32 radios;
	deserialization (userData, &radios, sizeof (radios));
	Init (radios, world->GetAllocator());
}


void ndShapeSphere::SetCollisionBBox (const dVector& p0__, const dVector& p1__)
{
	dAssert (0);
}

dInt32 ndShapeSphere::CalculateSignature (dFloat32 radius)
{
	dgUnsigned32 buffer[2];
	radius = dAbs (radius);

	buffer[0] = m_sphereCollision;
	buffer[1] = Quantize (radius);
	return Quantize(buffer, sizeof (buffer));
}

dInt32 ndShapeSphere::CalculateSignature () const
{
	return CalculateSignature(m_radius);
}



void ndShapeSphere::DebugCollision (const dMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTriplex pool[1024 * 2];
	dVector tmpVectex[1024 * 2];

	dVector p0 ( dFloat32 (1.0f), dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f)); 
	dVector p1 (-dFloat32 (1.0f), dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f)); 
	dVector p2 ( dFloat32 (0.0f), dFloat32 (1.0f), dFloat32 (0.0f), dFloat32 (0.0f)); 
	dVector p3 ( dFloat32 (0.0f),-dFloat32 (1.0f), dFloat32 (0.0f), dFloat32 (0.0f));
	dVector p4 ( dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (1.0f), dFloat32 (0.0f));
	dVector p5 ( dFloat32 (0.0f), dFloat32 (0.0f),-dFloat32 (1.0f), dFloat32 (0.0f));

	dInt32 index = 3;
	dInt32 count = 0;
	TesselateTriangle (index, p4, p0, p2, count, tmpVectex);
	TesselateTriangle (index, p4, p2, p1, count, tmpVectex);
	TesselateTriangle (index, p4, p1, p3, count, tmpVectex);
	TesselateTriangle (index, p4, p3, p0, count, tmpVectex);
	TesselateTriangle (index, p5, p2, p0, count, tmpVectex);
	TesselateTriangle (index, p5, p1, p2, count, tmpVectex);
	TesselateTriangle (index, p5, p3, p1, count, tmpVectex);
	TesselateTriangle (index, p5, p0, p3, count, tmpVectex);

	for (dInt32 i = 0; i < count; i ++) {
		tmpVectex[i] = tmpVectex[i].Scale (m_radius);
	}

	//dMatrix matrix (GetLocalMatrix() * matrixPtr);
	matrix.TransformTriplex (&pool[0].m_x, sizeof (dgTriplex), &tmpVectex[0].m_x, sizeof (dVector), count);
	for (dInt32 i = 0; i < count; i += 3) {
		callback (userData, 3, &pool[i].m_x, 0);
	}
}

dFloat32 dgCollisionPoint::GetVolume () const
{
	dAssert (0);
	return dFloat32 (0.0f); 
}

dVector dgCollisionPoint::SupportVertex (const dVector& dir, dInt32* const vertexIndex) const
{
	return dVector (dFloat32 (0.0f)); 
}

dVector dgCollisionPoint::SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const
{
	return dVector (dFloat32 (0.0f)); 
}

void ndShapeSphere::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);
	info->m_sphere.m_radius = m_radius;
}

void ndShapeSphere::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	callback (userData, &m_radius, sizeof (m_radius));
}

void ndShapeSphere::CalculateImplicitContacts(dInt32 count, dgContactPoint* const contactPoints) const
{
	for (dInt32 i = 0; i < count; i++) {
		dVector normal(contactPoints[i].m_point & dVector::m_triplexMask);
		dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(0.0f));
		normal = normal.Normalize();
		contactPoints[i].m_normal = normal * dVector::m_negOne;
		contactPoints[i].m_point = normal.Scale(m_radius);
	}
}

#endif

ndShapeSphere::ndShapeSphere(dFloat32 radius)
	:ndShapeConvex(m_sphereCollision)
{
	Init(radius);
}

ndShapeSphere::~ndShapeSphere()
{
	m_shapeRefCount--;
	dAssert(m_shapeRefCount >= 0);

	ndShapeConvex::m_simplex = nullptr;
	ndShapeConvex::m_vertex = nullptr;
}

void ndShapeSphere::TesselateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dInt32& count, dVector* const ouput) const
{
	if (level) 
	{
		dAssert(dAbs(p0.DotProduct(p0).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p1.DotProduct(p1).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p2.DotProduct(p2).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dVector p01(p0 + p1);
		dVector p12(p1 + p2);
		dVector p20(p2 + p0);

		p01 = p01.Normalize();
		p12 = p12.Normalize();
		p20 = p20.Normalize();

		dAssert(dAbs(p01.DotProduct(p01).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p12.DotProduct(p12).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p20.DotProduct(p20).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));

		TesselateTriangle(level - 1, p0, p01, p20, count, ouput);
		TesselateTriangle(level - 1, p1, p12, p01, count, ouput);
		TesselateTriangle(level - 1, p2, p20, p12, count, ouput);
		TesselateTriangle(level - 1, p01, p12, p20, count, ouput);
	}
	else 
	{
		ouput[count++] = p0;
		ouput[count++] = p1;
		ouput[count++] = p2;
	}
}

void ndShapeSphere::Init(dFloat32 radius)
{
	//m_rtti |= ndShapeSphere_RTTI;
	m_radius = dMax(dAbs(radius), D_MIN_CONVEX_SHAPE_SIZE);
	
	m_edgeCount = D_SPHERE_EDGE_COUNT;
	m_vertexCount = D_SPHERE_VERTEX_COUNT;
	ndShapeConvex::m_vertex = m_vertex;
	
	if (!m_shapeRefCount) 
	{
		dVector p0(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p1(-dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p2(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p3(dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p4(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
		dVector p5(dFloat32(0.0f), dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f));

		dVector tmpVectex[256];
		dInt32 indexList[256];
		dInt32 index = 1;
		dInt32 count = 0;
		TesselateTriangle(index, p4, p0, p2, count, tmpVectex);
		TesselateTriangle(index, p4, p2, p1, count, tmpVectex);
		TesselateTriangle(index, p4, p1, p3, count, tmpVectex);
		TesselateTriangle(index, p4, p3, p0, count, tmpVectex);
		TesselateTriangle(index, p5, p2, p0, count, tmpVectex);
		TesselateTriangle(index, p5, p1, p2, count, tmpVectex);
		TesselateTriangle(index, p5, p3, p1, count, tmpVectex);
		TesselateTriangle(index, p5, p0, p3, count, tmpVectex);

		dInt32 vertexCount = dVertexListToIndexList(&tmpVectex[0].m_x, sizeof(dVector), 3 * sizeof(dFloat32), 0, count, indexList, 0.001f);

		dAssert(vertexCount == D_SPHERE_VERTEX_COUNT);
		for (dInt32 i = 0; i < vertexCount; i++) 
		{
			m_unitSphere[i] = tmpVectex[i];
		}
		
		dPolyhedra polyhedra;
		
		polyhedra.BeginFace();
		for (dInt32 i = 0; i < count; i += 3) 
		{
			#ifdef _DEBUG
			dEdge* const edge = polyhedra.AddFace(indexList[i], indexList[i + 1], indexList[i + 2]);
			dAssert(edge);
			#else 
			polyhedra.AddFace(indexList[i], indexList[i + 1], indexList[i + 2]);
			#endif
		}
		polyhedra.EndFace();
		
		dUnsigned64 i1 = 0;
		dPolyhedra::Iterator iter(polyhedra);
		for (iter.Begin(); iter; iter++) 
		{
			dEdge* const edge = &(*iter);
			edge->m_userData = i1;
			i1++;
		}
		
		for (iter.Begin(); iter; iter++) 
		{
			dEdge* const edge = &(*iter);
		
			ndConvexSimplexEdge* const ptr = &m_edgeArray[edge->m_userData];
		
			ptr->m_vertex = edge->m_incidentVertex;
			ptr->m_next = &m_edgeArray[edge->m_next->m_userData];
			ptr->m_prev = &m_edgeArray[edge->m_prev->m_userData];
			ptr->m_twin = &m_edgeArray[edge->m_twin->m_userData];
		}
	}
	
	for (dInt32 i = 0; i < D_SPHERE_VERTEX_COUNT; i++) 
	{
		m_vertex[i] = m_unitSphere[i].Scale(m_radius);
	}
	
	m_shapeRefCount++;
	ndShapeConvex::m_simplex = m_edgeArray;
	SetVolumeAndCG();
}

void ndShapeSphere::MassProperties()
{
	m_centerOfMass = dVector::m_zero;
	m_crossInertia = dVector::m_zero;
	dFloat32 volume = dFloat32(4.0f * dPi / 3.0f) * m_radius *  m_radius * m_radius;
	dFloat32 II = dFloat32(2.0f / 5.0f) * m_radius *  m_radius;
	m_inertia = dVector(II, II, II, dFloat32(0.0f));
	m_centerOfMass.m_w = volume;
}

void ndShapeSphere::CalcAABB(const dMatrix& matrix, dVector &p0, dVector &p1) const
{
	dVector size(matrix.m_front.Abs().Scale(m_radius) + matrix.m_up.Abs().Scale(m_radius) + matrix.m_right.Abs().Scale(m_radius));
	p0 = (matrix[3] - size) & dVector::m_triplexMask;
	p1 = (matrix[3] + size) & dVector::m_triplexMask;
}

dVector ndShapeSphere::SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const
{
	return dir.Scale(m_radius - D_PENETRATION_TOL);
}

dVector ndShapeSphere::SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const
{
	return dVector::m_zero;
}

dVector ndShapeSphere::SupportVertex(const dVector& dir, dInt32* const vertexIndex) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	dAssert(dir.m_w == 0.0f);
	return dir.Scale(m_radius);
}

dInt32 ndShapeSphere::CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const
{
	dAssert(normal.m_w == 0.0f);
	dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(0.999f));
	contactsOut[0] = normal * normal.DotProduct(point);
	return 1;
}

dFloat32 ndShapeSphere::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	dAssert(0);
	return 0;
	//dFloat32 t = dgRayCastSphere(p0, p1, dVector(dFloat32(0.0f)), m_radius);
	//if (t < maxT) {
	//	dVector contact(p0 + (p1 - p0).Scale(t));
	//	dAssert(contact.m_w == dFloat32(0.0f));
	//	//contactOut.m_normal = contact.Scale (dgRsqrt (contact.DotProduct(contact).GetScalar()));
	//	contactOut.m_normal = contact.Normalize();
	//	//contactOut.m_userId = SetUserDataID();
	//}
	//return t;
}
