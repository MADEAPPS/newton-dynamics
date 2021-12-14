/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeCapsule.h"
#include "ndContactSolver.h"

#define DG_CAPSULE_SEGMENTS		10
#define DG_CAPSULE_CAP_SEGMENTS	12

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeCapsule)

ndShapeCapsule::ndShapeCapsule(ndFloat32 radius0, ndFloat32 radius1, ndFloat32 height)
	:ndShapeConvex(m_capsule)
{
	Init(radius0, radius1, height);
}

ndShapeCapsule::ndShapeCapsule(const ndLoadSaveBase::dLoadDescriptor& desc)
	:ndShapeConvex(m_capsule)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	ndFloat32 radius0 = xmlGetFloat(xmlNode, "radius0");
	ndFloat32 radius1 = xmlGetFloat(xmlNode, "radius1");
	ndFloat32 height = xmlGetFloat(xmlNode, "height");
	Init(radius0, radius1, height);
}

void ndShapeCapsule::Init(ndFloat32 radio0, ndFloat32 radio1, ndFloat32 height)
{
	radio0 = dMax(dAbs(radio0), D_MIN_CONVEX_SHAPE_SIZE);
	radio1 = dMax(dAbs(radio1), D_MIN_CONVEX_SHAPE_SIZE);
	height = dMax(dAbs(height), D_MIN_CONVEX_SHAPE_SIZE);

	m_transform = ndVector(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	if (radio0 > radio1) 
	{
		m_transform.m_x = ndFloat32(-1.0f);
		m_transform.m_y = ndFloat32(-1.0f);
		dSwap(radio0, radio1);
	}

	m_radius0 = radio0;
	m_radius1 = radio1;
	m_height = height * ndFloat32(0.5f);

	m_p0 = ndVector(-m_height, m_radius0, ndFloat32(0.0f), ndFloat32(0.0f));
	m_p1 = ndVector( m_height, m_radius1, ndFloat32(0.0f), ndFloat32(0.0f));
	m_normal = ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector side(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));

	for (ndInt32 i = 0; i < 16; i++) 
	{
		ndVector p1p0(m_p1 - m_p0);
		m_normal = side.CrossProduct(p1p0).Normalize();
		ndVector support0(m_normal.Scale(m_radius0));
		ndVector support1(m_normal.Scale(m_radius1));
		support0.m_x -= m_height;
		support1.m_x += m_height;
		ndFloat32 distance0 = support0.DotProduct(m_normal).GetScalar();
		ndFloat32 distance1 = support1.DotProduct(m_normal).GetScalar();

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

	ndVector tempVertex[4 * DG_CAPSULE_CAP_SEGMENTS * DG_CAPSULE_SEGMENTS + 100];
	ndInt32 index = 0;
	ndInt32 dx0 = ndInt32(ndFloor(DG_CAPSULE_SEGMENTS * ((m_p0.m_x + m_height + m_radius0) / m_radius0)) + ndFloat32(1.0f));
	ndFloat32 step = m_radius0 / DG_CAPSULE_SEGMENTS;
	ndFloat32 x0 = m_p0.m_x - step * dx0;
	for (ndInt32 j = 0; j < dx0; j++) 
	{
		x0 += step;
		ndFloat32 x = x0 + m_height;
		ndFloat32 arg = dMax(m_radius0 * m_radius0 - x * x, ndFloat32(1.0e-3f));
		ndFloat32 r0 = ndSqrt(arg);

		ndFloat32 angle = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
		{
			ndFloat32 z = ndSin(angle);
			ndFloat32 y = ndCos(angle);
			tempVertex[index] = ndVector(x0, y * r0, z * r0, ndFloat32(0.0f));
			index++;
			angle += (ndFloat32(2.0f) * ndPi) / DG_CAPSULE_CAP_SEGMENTS;
			dAssert(index < ndInt32 (sizeof(tempVertex) / sizeof(tempVertex[0])));
		}
	}

	ndFloat32 x1 = m_p1.m_x;
	ndInt32 dx1 = ndInt32(ndFloor(DG_CAPSULE_SEGMENTS * ((m_height + m_radius1 - m_p1.m_x) / m_radius1)) + ndFloat32(1.0f));
	step = m_radius1 / DG_CAPSULE_SEGMENTS;
	for (ndInt32 j = 0; j < dx1; j++) 
	{
		ndFloat32 x = x1 - m_height;
		ndFloat32 arg = dMax(m_radius1 * m_radius1 - x * x, ndFloat32(1.0e-3f));
		ndFloat32 r1 = ndSqrt(arg);
		ndFloat32 angle = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
		{
			ndFloat32 z = ndSin(angle);
			ndFloat32 y = ndCos(angle);
			tempVertex[index] = ndVector(x1, y * r1, z * r1, ndFloat32(0.0f));
			index++;
			angle += (ndFloat32(2.0f) * ndPi) / DG_CAPSULE_CAP_SEGMENTS;
			dAssert(index < ndInt32 (sizeof(tempVertex) / sizeof(tempVertex[0])));
		}
		x1 += step;
	}

	m_vertexCount = ndInt16(index);
	ndShapeConvex::m_vertex = (ndVector*)ndMemory::Malloc(ndInt32(m_vertexCount * sizeof(ndVector)));
	memcpy(ndShapeConvex::m_vertex, tempVertex, m_vertexCount * sizeof(ndVector));

	ndPolyhedra polyhedra;
	polyhedra.BeginFace();

	ndInt32 wireframe[DG_CAPSULE_SEGMENTS + 10];

	ndInt32 i1 = 0;
	ndInt32 i0 = DG_CAPSULE_CAP_SEGMENTS - 1;
	const ndInt32 n = index / DG_CAPSULE_CAP_SEGMENTS - 1;
	for (ndInt32 j = 0; j < n; j++) 
	{
		for (ndInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
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

	for (ndInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
	{
		wireframe[i] = DG_CAPSULE_CAP_SEGMENTS - i - 1;
	}
	polyhedra.AddFace(DG_CAPSULE_CAP_SEGMENTS, wireframe);

	for (ndInt32 i = 0; i < DG_CAPSULE_CAP_SEGMENTS; i++) 
	{
		wireframe[i] = index - DG_CAPSULE_CAP_SEGMENTS + i;
	}
	polyhedra.AddFace(DG_CAPSULE_CAP_SEGMENTS, wireframe);
	polyhedra.EndFace();

	dAssert(SanityCheck(polyhedra));

	m_edgeCount = ndInt16(polyhedra.GetEdgeCount());
	m_simplex = (ndConvexSimplexEdge*)ndMemory::Malloc(ndInt32(m_edgeCount * sizeof(ndConvexSimplexEdge)));

	ndUnsigned64 i = 0;
	ndPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		edge->m_userData = i;
		i++;
	}

	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);

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
	info.m_capsule.m_height = ndFloat32(2.0f) * m_height;

	if (m_transform.m_x < ndFloat32(0.0f)) 
	{
		dSwap(info.m_capsule.m_radio0, info.m_capsule.m_radio1);
	}
	return info;
}

void ndShapeCapsule::TesselateTriangle(ndInt32 level, const ndVector& p0, const ndVector& p1, const ndVector& p2, ndInt32& count, ndVector* ouput) const
{
	if (level) 
	{
		dAssert(dAbs(p0.DotProduct(p0).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p1.DotProduct(p1).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p2.DotProduct(p2).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		ndVector p01(p0 + p1);
		ndVector p12(p1 + p2);
		ndVector p20(p2 + p0);

		p01 = p01.Scale(ndRsqrt(p01.DotProduct(p01).GetScalar()));
		p12 = p12.Scale(ndRsqrt(p12.DotProduct(p12).GetScalar()));
		p20 = p20.Scale(ndRsqrt(p20.DotProduct(p20).GetScalar()));

		dAssert(dAbs(p01.DotProduct(p01).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p12.DotProduct(p12).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p20.DotProduct(p20).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));

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

void ndShapeCapsule::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	if (m_radius0 == m_radius1) 
	{
		#define POWER 2
		ndVector tmpVectex[512];

		ndVector p0(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p1(-ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p2(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p3(ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p4(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
		ndVector p5(ndFloat32(0.0f), ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f));

		ndInt32 count = 0;
		TesselateTriangle(POWER, p0, p2, p4, count, tmpVectex);
		TesselateTriangle(POWER, p0, p4, p3, count, tmpVectex);
		TesselateTriangle(POWER, p0, p3, p5, count, tmpVectex);
		TesselateTriangle(POWER, p0, p5, p2, count, tmpVectex);

		TesselateTriangle(POWER, p1, p4, p2, count, tmpVectex);
		TesselateTriangle(POWER, p1, p3, p4, count, tmpVectex);
		TesselateTriangle(POWER, p1, p5, p3, count, tmpVectex);
		TesselateTriangle(POWER, p1, p2, p5, count, tmpVectex);

		ndVector face[4];
		ndShapeDebugNotify::ndEdgeType edgeType[4];
		memset(edgeType, ndShapeDebugNotify::m_shared, sizeof(edgeType));

		for (ndInt32 i = 0; i < count; i += 3) 
		{
			ndInt32 positive = 0;
			for (ndInt32 j = 0; j < 3; j++) 
			{
				if (tmpVectex[i + j].m_x > ndFloat32(0.0f)) 
				{
					positive++;
				}
			}

			if (positive) 
			{
				face[0] = tmpVectex[i + 0];
				face[1] = tmpVectex[i + 1];
				face[2] = tmpVectex[i + 2];
				face[0].m_x += m_height;
				face[1].m_x += m_height;
				face[2].m_x += m_height;
				matrix.TransformTriplex(&face[0].m_x, sizeof(ndVector), &face[0].m_x, sizeof(ndVector), 3);

				debugCallback.DrawPolygon(3, face, edgeType);
			}
			else 
			{
				
				face[0] = tmpVectex[i + 0];
				face[1] = tmpVectex[i + 1];
				face[2] = tmpVectex[i + 2];
				face[0].m_x -= m_height;
				face[1].m_x -= m_height;
				face[2].m_x -= m_height;
				matrix.TransformTriplex(&face[0].m_x, sizeof(ndVector), &face[0].m_x, sizeof(ndVector), 3);

				debugCallback.DrawPolygon(3, face, edgeType);
			}
			if (positive == 1) 
			{
				ndVector q0(tmpVectex[i + 0]);
				ndVector q1(tmpVectex[i + 1]);
				if ((tmpVectex[i + 1].m_x == ndFloat32(0.0f)) && (tmpVectex[i + 2].m_x == ndFloat32(0.0f))) 
				{
					q0 = tmpVectex[i + 1];
					q1 = tmpVectex[i + 2];
				}
				else if ((tmpVectex[i + 2].m_x == ndFloat32(0.0f)) && (tmpVectex[i + 0].m_x == ndFloat32(0.0f))) 
				{
					q0 = tmpVectex[i + 2];
					q1 = tmpVectex[i + 0];
				}

				face[0] = q1;
				face[1] = q0;
				face[2] = q0;
				face[3] = q1;
				face[0].m_x += m_height;
				face[1].m_x += m_height;
				face[2].m_x -= m_height;
				face[3].m_x -= m_height;
				matrix.TransformTriplex(&face[0].m_x, sizeof(ndVector), &face[0].m_x, sizeof(ndVector), 4);

				debugCallback.DrawPolygon(4, face, edgeType);
			}
		}
	}
	else 
	{
		ndMatrix transform(matrix);
		transform[0] = transform[0].Scale(m_transform.m_x);
		transform[1] = transform[1].Scale(m_transform.m_y);
		transform[2] = transform[2].Scale(m_transform.m_z);
		ndShapeConvex::DebugShape(transform, debugCallback);
	}
}

ndVector ndShapeCapsule::SupportVertexSpecialProjectPoint(const ndVector& testPoint, const ndVector& direction) const
{
	ndVector dir(direction * m_transform);
	ndVector point(testPoint * m_transform);
	point += dir.Scale(m_radius0 - D_PENETRATION_TOL);
	return m_transform * point;
}


ndVector ndShapeCapsule::SupportVertex(const ndVector& direction, ndInt32* const) const
{
	ndVector dir(direction * m_transform);
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));

	ndVector p0(dir.Scale(m_radius0));
	ndVector p1(dir.Scale(m_radius1));
	p0.m_x -= m_height;
	p1.m_x += m_height;
	ndFloat32 dir0 = p0.DotProduct(dir).GetScalar();
	ndFloat32 dir1 = p1.DotProduct(dir).GetScalar();
	if (dir1 > dir0) 
	{
		p0 = p1;
	}
	return p0 * m_transform;
}

ndVector ndShapeCapsule::SupportVertexSpecial(const ndVector& direction, ndFloat32, ndInt32* const) const
{
	ndVector dir(direction * m_transform);
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));

	ndVector p0(ndVector::m_zero);
	ndVector p1(dir.Scale(m_radius1 - m_radius0));
	p0.m_x -= m_height;
	p1.m_x += m_height;
	ndFloat32 dir0 = p0.DotProduct(dir).GetScalar();
	ndFloat32 dir1 = p1.DotProduct(dir).GetScalar();
	if (dir1 > dir0) 
	{
		p0 = p1;
	}
	return p0 * m_transform;
}

ndFloat32 ndShapeCapsule::RayCast(ndRayCastNotify& callback, const ndVector& r0, const ndVector& r1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	ndVector q0(r0 * m_transform);
	ndVector q1(r1 * m_transform);

	ndVector origin0(-m_height, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector origin1(m_height, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndFloat32 t0 = dRayCastSphere(q0, q1, origin0, m_radius0);
	ndFloat32 t1 = dRayCastSphere(q0, q1, origin1, m_radius1);
	if ((t0 < ndFloat32(1.0f)) && (t1 < ndFloat32 (1.0f)))
	{
		if (t0 < t1) 
		{
			ndVector q(q0 + (q1 - q0).Scale(t0));
			ndVector n(q - origin0);
			dAssert(n.m_w == ndFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t0;
		}
		else 
		{
			ndVector q(q0 + (q1 - q0).Scale(t1));
			ndVector n(q - origin1);
			dAssert(n.m_w == ndFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t1;
		}
	}
	else if (t1 < ndFloat32(1.0f))
	{
		ndVector q(q0 + (q1 - q0).Scale(t1));
		if (q.m_x >= m_p1.m_x) 
		{
			ndVector n(q - origin1);
			dAssert(n.m_w == ndFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t1;
		}
	}
	else if (t0 < ndFloat32(1.0f))
	{
		ndVector q(q0 + (q1 - q0).Scale(t0));
		if (q.m_x <= m_p0.m_x) 
		{
			ndVector n(q - origin0);
			dAssert(n.m_w == ndFloat32(0.0f));
			//contactOut.m_normal = m_transform * n * n.DotProduct(n).InvSqrt();
			contactOut.m_normal = m_transform * n.Normalize();
			return t0;
		}
	}

	ndFloat32 ret = ndShapeConvex::RayCast(callback, q0, q1, maxT, body, contactOut);
	if (ret <= ndFloat32(1.0f)) 
	{
		contactOut.m_normal = m_transform * contactOut.m_normal;
	}
	return ret;
}

ndInt32 ndShapeCapsule::CalculatePlaneIntersection(const ndVector& direction, const ndVector& point, ndVector* const contactsOut) const
{
	ndVector normal(direction * m_transform);
	ndVector origin(point * m_transform);

	ndInt32 count = 0;
	ndVector p0(-m_height, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector dir0(p0 - origin);
	ndFloat32 dist0 = dir0.DotProduct(normal).GetScalar();
	if ((dist0 * dist0 - ndFloat32(5.0e-5f)) < (m_radius0 * m_radius0)) 
	{
		contactsOut[count] = m_transform * (p0 - normal.Scale(dist0));
		count++;
	}

	ndVector p1(m_height, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector dir1(p1 - origin);
	ndFloat32 dist1 = dir1.DotProduct(normal).GetScalar();
	if ((dist1 * dist1 - ndFloat32(5.0e-5f)) < (m_radius1 * m_radius1)) 
	{
		contactsOut[count] = m_transform * (p1 - normal.Scale(dist1));
		count++;
	}
	return count;
}

void ndShapeCapsule::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	ndVector size0(m_radius0);
	ndVector size1(m_radius1);
	ndVector q0(matrix.m_posit - matrix.m_front.Scale(m_height));
	ndVector q1(matrix.m_posit + matrix.m_front.Scale(m_height));

	ndVector min_q0(q0 - size0);
	ndVector min_q1(q1 - size1);

	ndVector max_q0(q0 + size1);
	ndVector max_q1(q1 + size1);

	p0 = min_q0.GetMin(min_q1) & ndVector::m_triplexMask;
	p1 = max_q0.GetMax(max_q1) & ndVector::m_triplexMask;
}

void ndShapeCapsule::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeConvex::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "radius0", m_radius0);
	xmlSaveParam(childNode, "radius1", m_radius0);
	xmlSaveParam(childNode, "height", m_height * ndFloat32 (2.0f));


//__classLoader__.CreateClass(childNode);

}