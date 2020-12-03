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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeCapsule.h"
#include "ndContactSolver.h"

#define DG_CAPSULE_SEGMENTS		10
#define DG_CAPSULE_CAP_SEGMENTS	12


#if 0
#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgContact.h"
#include "dgCollisionCapsule.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ndShapeCapsule::ndShapeCapsule(dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber)
	:dgCollisionConvex (world, deserialization, userData, revisionNumber)
{
	dVector size;
	deserialization(userData, &size, sizeof (dVector));
	Init (size.m_x, size.m_y, size.m_z * dFloat32 (2.0f));
}


ndShapeCapsule::~ndShapeCapsule()
{
}

dInt32 ndShapeCapsule::CalculateSignature (dFloat32 radio0, dFloat32 radio1, dFloat32 height)
{
	dgUnsigned32 buffer[4];

	buffer[0] = m_capsuleCollision;
	buffer[1] = Quantize (radio0);
	buffer[2] = Quantize (radio1);
	buffer[3] = Quantize (height);
	return Quantize(buffer, sizeof (buffer));
}

dInt32 ndShapeCapsule::CalculateSignature () const
{
	return CalculateSignature (m_radio0, m_radio1, m_height);
}

void ndShapeCapsule::SetCollisionBBox (const dVector& p0__, const dVector& p1__)
{
	dAssert (0);
}

void ndShapeCapsule::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	dVector size(m_radio0, m_radio1, m_height, dFloat32 (0.0f));
	callback (userData, &size, sizeof (dVector));
}



void ndShapeCapsule::CalculateImplicitContacts(dInt32 count, dgContactPoint* const contactPoints) const
{
	for (dInt32 i = 0; i < count; i++) {
		contactPoints[i].m_point = contactPoints[i].m_point * m_transform;
		if (contactPoints[i].m_point.m_x <= m_p0.m_x) {
			dVector normal(contactPoints[i].m_point);
			normal.m_x += m_height;
			dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(0.0f));
			normal = normal.Normalize();
			contactPoints[i].m_normal = normal * dVector::m_negOne;
			contactPoints[i].m_point = normal.Scale(m_radio0);
			contactPoints[i].m_point.m_x -= m_height;
		} else if (contactPoints[i].m_point.m_x >= m_p1.m_x) {
			dVector normal(contactPoints[i].m_point);
			normal.m_x -= m_height;
			dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(0.0f));
			normal = normal.Normalize();
			contactPoints[i].m_normal = normal * dVector::m_negOne;
			contactPoints[i].m_point = normal.Scale(m_radio1);
			contactPoints[i].m_point.m_x += m_height;
		} else {
			dVector normal(contactPoints[i].m_point);
			normal.m_x = dFloat32(0.0f);
			normal.m_w = dFloat32(0.0f);
			dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(0.0f));
			normal = normal.Normalize();

			dFloat32 h = m_p0.m_y + dFloat32(0.5f) * (m_p1.m_y - m_p0.m_y) * contactPoints[i].m_point.m_x / m_height;
			dVector point(normal.Scale(h));
			contactPoints[i].m_point.m_y = point.m_y;
			contactPoints[i].m_point.m_z = point.m_z;

			contactPoints[i].m_normal.m_x = -m_normal.m_x;
			contactPoints[i].m_normal.m_y = -m_normal.m_y * normal.m_y;
			contactPoints[i].m_normal.m_z = -m_normal.m_y * normal.m_z;
		}
		contactPoints[i].m_point = contactPoints[i].m_point * m_transform;
		contactPoints[i].m_normal = contactPoints[i].m_normal * m_transform;
	}
}

#endif

ndShapeCapsule::ndShapeCapsule(dFloat32 radius0, dFloat32 radius1, dFloat32 height)
	:ndShapeConvex(m_capsuleCollision)
{
	Init(radius0, radius1, height);
}

ndShapeCapsule::ndShapeCapsule(const nd::TiXmlNode* const xmlNode)
	: ndShapeConvex(m_capsuleCollision)
{
	dFloat32 radius0 = xmlGetFloat(xmlNode, "radius0");
	dFloat32 radius1 = xmlGetFloat(xmlNode, "radius1");
	dFloat32 height = xmlGetFloat(xmlNode, "height");
	Init(radius0, radius1, height);
}

void ndShapeCapsule::Init(dFloat32 radio0, dFloat32 radio1, dFloat32 height)
{
	radio0 = dMax(dAbs(radio0), D_MIN_CONVEX_SHAPE_SIZE);
	radio1 = dMax(dAbs(radio1), D_MIN_CONVEX_SHAPE_SIZE);
	height = dMax(dAbs(height), D_MIN_CONVEX_SHAPE_SIZE);

	m_transform = dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f));
	if (radio0 > radio1) 
	{
		m_transform.m_x = dFloat32(-1.0f);
		m_transform.m_y = dFloat32(-1.0f);
		dSwap(radio0, radio1);
	}

	m_radius0 = radio0;
	m_radius1 = radio1;
	m_height = height * dFloat32(0.5f);

	m_p0 = dVector(-m_height, m_radius0, dFloat32(0.0f), dFloat32(0.0f));
	m_p1 = dVector( m_height, m_radius1, dFloat32(0.0f), dFloat32(0.0f));
	m_normal = dVector(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector side(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));

	for (int i = 0; i < 16; i++) 
	{
		dVector p1p0(m_p1 - m_p0);
		m_normal = side.CrossProduct(p1p0).Normalize();
		dVector support0(m_normal.Scale(m_radius0));
		dVector support1(m_normal.Scale(m_radius1));
		support0.m_x -= m_height;
		support1.m_x += m_height;
		dFloat32 distance0 = support0.DotProduct(m_normal).GetScalar();
		dFloat32 distance1 = support1.DotProduct(m_normal).GetScalar();

		if (distance1 > distance0) 
		{
			m_p1 = support1;
		}
		else if (distance1 < distance0) 
		{
			m_p0 = support0;
		}
		else 
		{
			i = 1000;
		}
	}

	dVector tempVertex[4 * DG_CAPSULE_CAP_SEGMENTS * DG_CAPSULE_SEGMENTS + 100];
	dInt32 index = 0;
	dInt32 dx0 = dInt32(dFloor(DG_CAPSULE_SEGMENTS * ((m_p0.m_x + m_height + m_radius0) / m_radius0)) + dFloat32(1.0f));
	dFloat32 step = m_radius0 / DG_CAPSULE_SEGMENTS;
	dFloat32 x0 = m_p0.m_x - step * dx0;
	for (dInt32 j = 0; j < dx0; j++) 
	{
		x0 += step;
		dFloat32 x = x0 + m_height;
		dFloat32 arg = dMax(m_radius0 * m_radius0 - x * x, dFloat32(1.0e-3f));
		dFloat32 r0 = dSqrt(arg);

		dFloat32 angle = dFloat32(0.0f);
		for (dInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
		{
			dFloat32 z = dSin(angle);
			dFloat32 y = dCos(angle);
			tempVertex[index] = dVector(x0, y * r0, z * r0, dFloat32(0.0f));
			index++;
			angle += (dFloat32(2.0f) * dPi) / DG_CAPSULE_CAP_SEGMENTS;
			dAssert(index < sizeof(tempVertex) / sizeof(tempVertex[0]));
		}
	}

	dFloat32 x1 = m_p1.m_x;
	dInt32 dx1 = dInt32(dFloor(DG_CAPSULE_SEGMENTS * ((m_height + m_radius1 - m_p1.m_x) / m_radius1)) + dFloat32(1.0f));
	step = m_radius1 / DG_CAPSULE_SEGMENTS;
	for (dInt32 j = 0; j < dx1; j++) 
	{
		dFloat32 x = x1 - m_height;
		dFloat32 arg = dMax(m_radius1 * m_radius1 - x * x, dFloat32(1.0e-3f));
		dFloat32 r1 = dSqrt(arg);
		dFloat32 angle = dFloat32(0.0f);
		for (dInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
		{
			dFloat32 z = dSin(angle);
			dFloat32 y = dCos(angle);
			tempVertex[index] = dVector(x1, y * r1, z * r1, dFloat32(0.0f));
			index++;
			angle += (dFloat32(2.0f) * dPi) / DG_CAPSULE_CAP_SEGMENTS;
			dAssert(index < sizeof(tempVertex) / sizeof(tempVertex[0]));
		}
		x1 += step;
	}

	m_vertexCount = dInt16(index);
	ndShapeConvex::m_vertex = (dVector*)dMemory::Malloc(dInt32(m_vertexCount * sizeof(dVector)));
	memcpy(ndShapeConvex::m_vertex, tempVertex, m_vertexCount * sizeof(dVector));

	dPolyhedra polyhedra;
	polyhedra.BeginFace();

	dInt32 wireframe[DG_CAPSULE_SEGMENTS + 10];

	dInt32 i1 = 0;
	dInt32 i0 = DG_CAPSULE_CAP_SEGMENTS - 1;
	const dInt32 n = index / DG_CAPSULE_CAP_SEGMENTS - 1;
	for (dInt32 j = 0; j < n; j++) 
	{
		for (dInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
		{
			wireframe[0] = i0;
			wireframe[1] = i1;
			wireframe[2] = i1 + DG_CAPSULE_CAP_SEGMENTS;
			wireframe[3] = i0 + DG_CAPSULE_CAP_SEGMENTS;
			i0 = i1;
			i1++;
			polyhedra.AddFace(4, wireframe);
		}
		i0 = i1 + DG_CAPSULE_CAP_SEGMENTS - 1;
	}

	for (dInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
	{
		wireframe[i] = DG_CAPSULE_CAP_SEGMENTS - i - 1;
	}
	polyhedra.AddFace(DG_CAPSULE_CAP_SEGMENTS, wireframe);

	for (dInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
	{
		wireframe[i] = index - DG_CAPSULE_CAP_SEGMENTS + i;
	}
	polyhedra.AddFace(DG_CAPSULE_CAP_SEGMENTS, wireframe);
	polyhedra.EndFace();

	dAssert(SanityCheck(polyhedra));

	m_edgeCount = dInt16(polyhedra.GetEdgeCount());
	m_simplex = (ndConvexSimplexEdge*)dMemory::Malloc(dInt32(m_edgeCount * sizeof(ndConvexSimplexEdge)));

	dUnsigned64 i = 0;
	dPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);
		edge->m_userData = i;
		i++;
	}

	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);

		ndConvexSimplexEdge* const ptr = &m_simplex[edge->m_userData];

		ptr->m_vertex = edge->m_incidentVertex;
		ptr->m_next = &m_simplex[edge->m_next->m_userData];
		ptr->m_prev = &m_simplex[edge->m_prev->m_userData];
		ptr->m_twin = &m_simplex[edge->m_twin->m_userData];
	}
	SetVolumeAndCG();
}

ndShapeInfo ndShapeCapsule::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());

	info.m_capsule.m_radio0 = m_radius0;
	info.m_capsule.m_radio1 = m_radius1;
	info.m_capsule.m_height = dFloat32(2.0f) * m_height;

	if (m_transform.m_x < dFloat32(0.0f)) 
	{
		dSwap(info.m_capsule.m_radio0, info.m_capsule.m_radio1);
	}
	return info;
}

void ndShapeCapsule::TesselateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dInt32& count, dVector* ouput) const
{
	if (level) 
	{
		dAssert(dAbs(p0.DotProduct(p0).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p1.DotProduct(p1).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p2.DotProduct(p2).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dVector p01(p0 + p1);
		dVector p12(p1 + p2);
		dVector p20(p2 + p0);

		p01 = p01.Scale(dRsqrt(p01.DotProduct(p01).GetScalar()));
		p12 = p12.Scale(dRsqrt(p12.DotProduct(p12).GetScalar()));
		p20 = p20.Scale(dRsqrt(p20.DotProduct(p20).GetScalar()));

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
		ouput[count + 0] = p0.Scale(m_radius0);
		ouput[count + 1] = p1.Scale(m_radius0);
		ouput[count + 2] = p2.Scale(m_radius0);
		count += 3;
	}
}

void ndShapeCapsule::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	if (m_radius0 == m_radius1) 
	{
		#define POWER 2
		dVector tmpVectex[512];

		dVector p0(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p1(-dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p2(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p3(dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p4(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
		dVector p5(dFloat32(0.0f), dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f));

		dInt32 count = 0;
		TesselateTriangle(POWER, p0, p2, p4, count, tmpVectex);
		TesselateTriangle(POWER, p0, p4, p3, count, tmpVectex);
		TesselateTriangle(POWER, p0, p3, p5, count, tmpVectex);
		TesselateTriangle(POWER, p0, p5, p2, count, tmpVectex);

		TesselateTriangle(POWER, p1, p4, p2, count, tmpVectex);
		TesselateTriangle(POWER, p1, p3, p4, count, tmpVectex);
		TesselateTriangle(POWER, p1, p5, p3, count, tmpVectex);
		TesselateTriangle(POWER, p1, p2, p5, count, tmpVectex);

		for (dInt32 i = 0; i < count; i += 3) 
		{
			dInt32 positive = 0;
			for (dInt32 j = 0; j < 3; j++) 
			{
				if (tmpVectex[i + j].m_x > dFloat32(0.0f)) 
				{
					positive++;
				}
			}

			if (positive) 
			{
				dVector face[4];
				face[0] = tmpVectex[i + 0];
				face[1] = tmpVectex[i + 1];
				face[2] = tmpVectex[i + 2];
				face[0].m_x += m_height;
				face[1].m_x += m_height;
				face[2].m_x += m_height;
				matrix.TransformTriplex(&face[0].m_x, sizeof(dVector), &face[0].m_x, sizeof(dVector), 3);

				debugCallback.DrawPolygon(3, face);
				//callback(userData, 3, &face[0].m_x, 0);
			}
			else 
			{
				dVector face[4];
				face[0] = tmpVectex[i + 0];
				face[1] = tmpVectex[i + 1];
				face[2] = tmpVectex[i + 2];
				face[0].m_x -= m_height;
				face[1].m_x -= m_height;
				face[2].m_x -= m_height;
				matrix.TransformTriplex(&face[0].m_x, sizeof(dVector), &face[0].m_x, sizeof(dVector), 3);
				//callback(userData, 3, &face[0].m_x, 0);
				debugCallback.DrawPolygon(3, face);
			}
			if (positive == 1) 
			{
				dVector q0(tmpVectex[i + 0]);
				dVector q1(tmpVectex[i + 1]);
				if ((tmpVectex[i + 1].m_x == dFloat32(0.0f)) && (tmpVectex[i + 2].m_x == dFloat32(0.0f))) 
				{
					q0 = tmpVectex[i + 1];
					q1 = tmpVectex[i + 2];
				}
				else if ((tmpVectex[i + 2].m_x == dFloat32(0.0f)) && (tmpVectex[i + 0].m_x == dFloat32(0.0f))) 
				{
					q0 = tmpVectex[i + 2];
					q1 = tmpVectex[i + 0];
				}

				dVector face[4];
				face[0] = q1;
				face[1] = q0;
				face[2] = q0;
				face[3] = q1;
				face[0].m_x += m_height;
				face[1].m_x += m_height;
				face[2].m_x -= m_height;
				face[3].m_x -= m_height;
				matrix.TransformTriplex(&face[0].m_x, sizeof(dVector), &face[0].m_x, sizeof(dVector), 4);
				//callback(userData, 4, &face[0].m_x, 0);
				debugCallback.DrawPolygon(4, face);
			}
		}
	}
	else 
	{
		dAssert(0);
		dMatrix transform(matrix);
		transform[0] = transform[0].Scale(m_transform.m_x);
		transform[1] = transform[1].Scale(m_transform.m_y);
		transform[2] = transform[2].Scale(m_transform.m_z);
		//ndShapeConvex::DebugCollision(transform, callback, userData);
		ndShapeConvex::DebugShape(transform, debugCallback);
	}
}

//dVector ndShapeCapsule::SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const
dVector ndShapeCapsule::SupportVertexSpecialProjectPoint(const dVector& testPoint, const dVector& direction) const
{
	dVector dir(direction * m_transform);
	dVector point(testPoint * m_transform);
	point += dir.Scale(m_radius0 - D_PENETRATION_TOL);
	return m_transform * point;
}


dVector ndShapeCapsule::SupportVertex(const dVector& direction, dInt32* const vertexIndex) const
{
	dVector dir(direction * m_transform);
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	dVector p0(dir.Scale(m_radius0));
	dVector p1(dir.Scale(m_radius1));
	p0.m_x -= m_height;
	p1.m_x += m_height;
	dFloat32 dir0 = p0.DotProduct(dir).GetScalar();
	dFloat32 dir1 = p1.DotProduct(dir).GetScalar();
	if (dir1 > dir0) 
	{
		p0 = p1;
	}
	return p0 * m_transform;
}

dVector ndShapeCapsule::SupportVertexSpecial(const dVector& direction, dFloat32 skinThickness, dInt32* const vertexIndex) const
{
	dVector dir(direction * m_transform);
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	dVector p0(dVector::m_zero);
	dVector p1(dir.Scale(m_radius1 - m_radius0));
	p0.m_x -= m_height;
	p1.m_x += m_height;
	dFloat32 dir0 = p0.DotProduct(dir).GetScalar();
	dFloat32 dir1 = p1.DotProduct(dir).GetScalar();
	if (dir1 > dir0) 
	{
		p0 = p1;
	}
	return p0 * m_transform;
}

dFloat32 ndShapeCapsule::RayCast(ndRayCastNotify& callback, const dVector& r0, const dVector& r1, const ndBody* const body, ndContactPoint& contactOut) const
{
	dVector q0(r0 * m_transform);
	dVector q1(r1 * m_transform);

	dVector origin0(-m_height, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector origin1(m_height, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	dFloat32 t0 = dRayCastSphere(q0, q1, origin0, m_radius0);
	dFloat32 t1 = dRayCastSphere(q0, q1, origin1, m_radius1);
	if ((t0 < dFloat32(1.0f)) && (t1 < dFloat32 (1.0f)))
	{
		if (t0 < t1) 
		{
			dVector q(q0 + (q1 - q0).Scale(t0));
			dVector n(q - origin0);
			dAssert(n.m_w == dFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t0;
		}
		else 
		{
			dVector q(q0 + (q1 - q0).Scale(t1));
			dVector n(q - origin1);
			dAssert(n.m_w == dFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t1;
		}
	}
	else if (t1 < dFloat32(1.0f))
	{
		dVector q(q0 + (q1 - q0).Scale(t1));
		if (q.m_x >= m_p1.m_x) 
		{
			dVector n(q - origin1);
			dAssert(n.m_w == dFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t1;
		}
	}
	else if (t0 < dFloat32(1.0f))
	{
		dVector q(q0 + (q1 - q0).Scale(t0));
		if (q.m_x <= m_p0.m_x) 
		{
			dVector n(q - origin0);
			dAssert(n.m_w == dFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t0;
		}
	}

	dFloat32 ret = ndShapeConvex::RayCast(callback, q0, q1, body, contactOut);
	if (ret <= dFloat32(1.0f)) 
	{
		contactOut.m_normal = m_transform * contactOut.m_normal;
	}
	return ret;
}

dInt32 ndShapeCapsule::CalculatePlaneIntersection(const dVector& direction, const dVector& point, dVector* const contactsOut) const
{
	dVector normal(direction * m_transform);
	dVector origin(point * m_transform);

	dInt32 count = 0;
	dVector p0(-m_height, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector dir0(p0 - origin);
	dFloat32 dist0 = dir0.DotProduct(normal).GetScalar();
	if ((dist0 * dist0 - dFloat32(5.0e-5f)) < (m_radius0 * m_radius0)) 
	{
		contactsOut[count] = m_transform * (p0 - normal.Scale(dist0));
		count++;
	}

	dVector p1(m_height, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector dir1(p1 - origin);
	dFloat32 dist1 = dir1.DotProduct(normal).GetScalar();
	if ((dist1 * dist1 - dFloat32(5.0e-5f)) < (m_radius1 * m_radius1)) 
	{
		contactsOut[count] = m_transform * (p1 - normal.Scale(dist1));
		count++;
	}
	return count;
}

void ndShapeCapsule::CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	//ndShapeConvex::CalcAABB(matrix, p0, p1);

	dVector size0(m_radius0);
	dVector size1(m_radius1);
	dVector q0(matrix.m_posit - matrix.m_front.Scale(m_height));
	dVector q1(matrix.m_posit + matrix.m_front.Scale(m_height));

	dVector min_q0(q0 - size0);
	dVector min_q1(q1 - size1);

	dVector max_q0(q0 + size1);
	dVector max_q1(q1 + size1);

	p0 = min_q0.GetMin(min_q1) & dVector::m_triplexMask;
	p1 = max_q0.GetMax(max_q1) & dVector::m_triplexMask;
}

void ndShapeCapsule::Save( nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid ) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement("ndShapeCapsule");
	xmlNode->LinkEndChild(paramNode);

	paramNode->SetAttribute("nodeId", nodeid);

	xmlSaveParam(paramNode, "radius0", m_radius0);
	xmlSaveParam(paramNode, "radius1", m_radius0);
	xmlSaveParam(paramNode, "height", m_height * dFloat32 (2.0f));
}