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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeCylinder.h"
#include "ndContactSolver.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeCylinder)

dInt32 ndShapeCylinder::m_shapeRefCount = 0;
ndShapeConvex::ndConvexSimplexEdge ndShapeCylinder::m_edgeArray[D_TAPED_CYLINDER_SEGMENTS * 2 * 3];

ndShapeCylinder::ndShapeCylinder(dFloat32 radius0, dFloat32 radius1, dFloat32 height)
	:ndShapeConvex(m_cylinder)
{
	Init(radius0, radius1, height);
}

ndShapeCylinder::ndShapeCylinder(const dLoadSaveBase::dLoadDescriptor& desc)
	: ndShapeConvex(m_cylinder)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	dFloat32 radius0 = xmlGetFloat(xmlNode, "radius0");
	dFloat32 radius1 = xmlGetFloat(xmlNode, "radius1");
	dFloat32 height = xmlGetFloat(xmlNode, "height");
	Init(radius0, radius1, height);
}

ndShapeCylinder::~ndShapeCylinder()
{
	m_shapeRefCount--;
	dAssert(m_shapeRefCount >= 0);
	ndShapeConvex::m_vertex = nullptr;
	ndShapeConvex::m_simplex = nullptr;
}

void ndShapeCylinder::Init(dFloat32 radio0, dFloat32 radio1, dFloat32 height)
{
	m_radius0 = dMax(dAbs(radio0), D_MIN_CONVEX_SHAPE_SIZE);
	m_radius1 = dMax(dAbs(radio1), D_MIN_CONVEX_SHAPE_SIZE);
	m_height = dMax(dAbs(height) * dFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);

	dFloat32 angle = dFloat32(0.0f);
	const dInt32 offset0 = 0;
	const dInt32 offset1 = D_TAPED_CYLINDER_SEGMENTS;
	for (dInt32 i = 0; i < D_TAPED_CYLINDER_SEGMENTS; i++) 
	{
		dFloat32 sinAngle = dSin(angle);
		dFloat32 cosAngle = dCos(angle);
		m_vertex[i + offset0] = dVector(-m_height, m_radius0 * cosAngle, m_radius0 * sinAngle, dFloat32(0.0f));
		m_vertex[i + offset1] = dVector( m_height, m_radius1 * cosAngle, m_radius1 * sinAngle, dFloat32(0.0f));
		angle += dFloat32 (2.0f) * dPi / D_TAPED_CYLINDER_SEGMENTS;
	}

	m_edgeCount = D_TAPED_CYLINDER_SEGMENTS * 6;
	m_vertexCount = D_TAPED_CYLINDER_SEGMENTS * 2;
	ndShapeConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) 
	{
		dPolyhedra polyhedra;
		dInt32 wireframe[D_TAPED_CYLINDER_SEGMENTS];

		dInt32 j = D_TAPED_CYLINDER_SEGMENTS - 1;
		polyhedra.BeginFace();
		for (dInt32 i = 0; i < D_TAPED_CYLINDER_SEGMENTS; i++) 
		{
			wireframe[0] = j;
			wireframe[1] = i;
			wireframe[2] = i + D_TAPED_CYLINDER_SEGMENTS;
			wireframe[3] = j + D_TAPED_CYLINDER_SEGMENTS;
			j = i;
			polyhedra.AddFace(4, wireframe);
		}

		for (dInt32 i = 0; i < D_TAPED_CYLINDER_SEGMENTS; i++) 
		{
			wireframe[i] = D_TAPED_CYLINDER_SEGMENTS - 1 - i;
		}
		polyhedra.AddFace(D_TAPED_CYLINDER_SEGMENTS, wireframe);

		for (dInt32 i = 0; i < D_TAPED_CYLINDER_SEGMENTS; i++) 
		{
			wireframe[i] = i + D_TAPED_CYLINDER_SEGMENTS;
		}
		polyhedra.AddFace(D_TAPED_CYLINDER_SEGMENTS, wireframe);
		polyhedra.EndFace();

		dAssert(SanityCheck(polyhedra));

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

			ndConvexSimplexEdge* const ptr = &m_edgeArray[edge->m_userData];
			ptr->m_vertex = edge->m_incidentVertex;
			ptr->m_next = &m_edgeArray[edge->m_next->m_userData];
			ptr->m_prev = &m_edgeArray[edge->m_prev->m_userData];
			ptr->m_twin = &m_edgeArray[edge->m_twin->m_userData];
		}
	}

	m_profile[0] = dVector(m_height, m_radius1, dFloat32(0.0f), dFloat32(0.0f));
	m_profile[1] = dVector(-m_height, m_radius0, dFloat32(0.0f), dFloat32(0.0f));
	m_profile[2] = dVector(-m_height, -m_radius0, dFloat32(0.0f), dFloat32(0.0f));
	m_profile[3] = dVector(m_height, -m_radius1, dFloat32(0.0f), dFloat32(0.0f));

	m_shapeRefCount++;
	ndShapeConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG();
}

ndShapeInfo ndShapeCylinder::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());

	info.m_cylinder.m_radio0 = m_radius0;
	info.m_cylinder.m_radio1 = m_radius1;
	info.m_cylinder.m_height = dFloat32(2.0f) * m_height;
	return info;
}

void ndShapeCylinder::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	#define NUMBER_OF_DEBUG_SEGMENTS 24
	dVector face[NUMBER_OF_DEBUG_SEGMENTS];
	dVector pool[NUMBER_OF_DEBUG_SEGMENTS * 2];
	ndShapeDebugCallback::ndEdgeType edgeType[NUMBER_OF_DEBUG_SEGMENTS];
	memset(edgeType, ndShapeDebugCallback::m_shared, sizeof(edgeType));

	dFloat32 angle = dFloat32(0.0f);
	for (dInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i++) 
	{
		dFloat32 z = dSin(angle);
		dFloat32 y = dCos(angle);
		pool[i].m_x = -m_height;
		pool[i].m_y = y * m_radius0;
		pool[i].m_z = z * m_radius0;
		pool[i].m_w = dFloat32 (0.0f);
		pool[i + NUMBER_OF_DEBUG_SEGMENTS].m_x = m_height;
		pool[i + NUMBER_OF_DEBUG_SEGMENTS].m_y = y * m_radius1;
		pool[i + NUMBER_OF_DEBUG_SEGMENTS].m_z = z * m_radius1;
		pool[i + NUMBER_OF_DEBUG_SEGMENTS].m_w = dFloat32(0.0f);
		angle += dFloat32 (2.0) * dPi / dFloat32(NUMBER_OF_DEBUG_SEGMENTS);
	}

	matrix.TransformTriplex(&pool[0].m_x, sizeof(dVector), &pool[0].m_x, sizeof(dVector), NUMBER_OF_DEBUG_SEGMENTS * 2);

	dInt32 j = NUMBER_OF_DEBUG_SEGMENTS - 1;
	for (dInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i++) 
	{
		face[0] = pool[j];
		face[1] = pool[i];
		face[2] = pool[i + NUMBER_OF_DEBUG_SEGMENTS];
		face[3] = pool[j + NUMBER_OF_DEBUG_SEGMENTS];
		j = i;
		debugCallback.DrawPolygon(4, face, edgeType);
	}

	for (dInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i++) 
	{
		face[i] = pool[NUMBER_OF_DEBUG_SEGMENTS - 1 - i];
	}
	debugCallback.DrawPolygon(NUMBER_OF_DEBUG_SEGMENTS, face, edgeType);

	for (dInt32 i = 0; i < NUMBER_OF_DEBUG_SEGMENTS; i++) 
	{
		face[i] = pool[i + NUMBER_OF_DEBUG_SEGMENTS];
	}
	debugCallback.DrawPolygon(NUMBER_OF_DEBUG_SEGMENTS, face, edgeType);
}

dVector ndShapeCylinder::SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	return point + dir.Scale(D_PENETRATION_TOL);
}

dVector ndShapeCylinder::SupportVertex(const dVector& dir, dInt32* const) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	if (dir.m_x < dFloat32(-0.9999f)) 
	{
		return dVector(-m_height, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	}
	else if (dir.m_x > dFloat32(0.9999f)) 
	{
		return dVector(m_height, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	}

	dVector dir_yz(dir);
	dir_yz.m_x = dFloat32(0.0f);
	dAssert(dir_yz.DotProduct(dir_yz).GetScalar() > dFloat32(0.0f));
	//dFloat32 mag2 = dir_yz.DotProduct(dir_yz).GetScalar();
	//dAssert(mag2 > dFloat32(0.0f));
	//dir_yz = dir_yz.Scale(dFloat32(1.0f) / dSqrt(mag2));
	dir_yz = dir_yz.Normalize();
	dVector p0(dir_yz.Scale(m_radius0));
	dVector p1(dir_yz.Scale(m_radius1));

	p0.m_x = -m_height;
	p1.m_x = m_height;

	dFloat32 dist0 = dir.DotProduct(p0).GetScalar();
	dFloat32 dist1 = dir.DotProduct(p1).GetScalar();

	if (dist1 >= dist0) 
	{
		p0 = p1;
	}
	return p0;
}

dVector ndShapeCylinder::SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	const dFloat32 thickness = D_PENETRATION_TOL + skinThickness;
	if (dir.m_x < dFloat32(-0.9999f)) 
	{
		return dVector(-(m_height - thickness), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	}
	else if (dir.m_x > dFloat32(0.9999f)) 
	{
		return dVector(m_height - thickness, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	}

	dVector dir_yz(dir);
	dir_yz.m_x = dFloat32(0.0f);
	//dFloat32 mag2 = dir_yz.DotProduct(dir_yz).GetScalar();
	//dAssert(mag2 > dFloat32(0.0f));
	//dir_yz = dir_yz.Scale(dFloat32(1.0f) / dgSqrt(mag2));
	dir_yz = dir_yz.Normalize();
	dVector p0(dir_yz.Scale(m_radius0 - thickness));
	dVector p1(dir_yz.Scale(m_radius1 - thickness));

	p0.m_x = -(m_height - thickness);
	p1.m_x = m_height - thickness;

	dFloat32 dist0 = dir.DotProduct(p0).GetScalar();
	dFloat32 dist1 = dir.DotProduct(p1).GetScalar();

	if (dist1 >= dist0) 
	{
		p0 = p1;
	}
	return p0;
}

dFloat32 ndShapeCylinder::RayCast(ndRayCastNotify& callback, const dVector& r0, const dVector& r1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	return ndShapeConvex::RayCast(callback, r0, r1, maxT, body, contactOut);
}

dInt32 ndShapeCylinder::CalculatePlaneIntersection(const dVector& normal, const dVector& origin, dVector* const contactsOut) const
{
	dInt32 count = 0;
	const dFloat32 inclination = dFloat32(0.9998f);
	if (normal.m_x < dFloat32(-0.995f)) 
	{
		if (normal.m_x < -inclination) 
		{
			dMatrix matrix(normal);
			matrix.m_posit.m_x = origin.m_x;
			count = BuildCylinderCapPoly(m_radius0, matrix, contactsOut);
			//count = RectifyConvexSlice(n, normal, contactsOut);
		}
		else 
		{
			dFloat32 magInv = dRsqrt(normal.m_y * normal.m_y + normal.m_z * normal.m_z);
			dFloat32 cosAng = normal.m_y * magInv;
			dFloat32 sinAng = normal.m_z * magInv;

			dAssert(dAbs(normal.m_z * cosAng - normal.m_y * sinAng) < dFloat32(1.0e-4f));
			dVector normal1(normal.m_x, normal.m_y * cosAng + normal.m_z * sinAng, dFloat32(0.0f), dFloat32(0.0f));
			dVector origin1(origin.m_x, origin.m_y * cosAng + origin.m_z * sinAng, origin.m_z * cosAng - origin.m_y * sinAng, dFloat32(0.0f));

			count = ndShapeConvex::CalculatePlaneIntersection(normal1, origin1, contactsOut);
			if (count > 6) 
			{
				dInt32 dy = 2 * 6;
				dInt32 dx = 2 * count;
				dInt32 acc = dy - count;
				dInt32 index = 0;
				for (dInt32 i = 0; i < count; i++) 
				{
					if (acc > 0) 
					{
						contactsOut[index] = contactsOut[i];
						index++;
						acc -= dx;
					}
					acc += dy;
				}
				count = index;
			}

			for (dInt32 i = 0; i < count; i++) 
			{
				dFloat32 y = contactsOut[i].m_y;
				dFloat32 z = contactsOut[i].m_z;
				contactsOut[i].m_y = y * cosAng - z * sinAng;
				contactsOut[i].m_z = z * cosAng + y * sinAng;
			}
		}
	}
	else if (normal.m_x > dFloat32(0.995f)) 
	{
		if (normal.m_x > inclination) 
		{
			dMatrix matrix(normal);
			matrix.m_posit.m_x = origin.m_x;
			count = BuildCylinderCapPoly(m_radius1, matrix, contactsOut);
			//count = RectifyConvexSlice(n, normal, contactsOut);
		}
		else 
		{
			dFloat32 magInv = dRsqrt(normal.m_y * normal.m_y + normal.m_z * normal.m_z);
			dFloat32 cosAng = normal.m_y * magInv;
			dFloat32 sinAng = normal.m_z * magInv;

			dAssert(dAbs(normal.m_z * cosAng - normal.m_y * sinAng) < dFloat32(1.0e-4f));
			dVector normal1(normal.m_x, normal.m_y * cosAng + normal.m_z * sinAng, dFloat32(0.0f), dFloat32(0.0f));
			dVector origin1(origin.m_x, origin.m_y * cosAng + origin.m_z * sinAng, origin.m_z * cosAng - origin.m_y * sinAng, dFloat32(0.0f));

			count = ndShapeConvex::CalculatePlaneIntersection(normal1, origin1, contactsOut);
			if (count > 6) 
			{
				dInt32 dy = 2 * 6;
				dInt32 dx = 2 * count;
				dInt32 acc = dy - count;
				dInt32 index = 0;
				for (dInt32 i = 0; i < count; i++) 
				{
					if (acc > 0) 
					{
						contactsOut[index] = contactsOut[i];
						index++;
						acc -= dx;
					}
					acc += dy;
				}
				count = index;
			}

			for (dInt32 i = 0; i < count; i++) 
			{
				dFloat32 y = contactsOut[i].m_y;
				dFloat32 z = contactsOut[i].m_z;
				contactsOut[i].m_y = y * cosAng - z * sinAng;
				contactsOut[i].m_z = z * cosAng + y * sinAng;
			}
		}
	}
	else 
	{
		dFloat32 magInv = dRsqrt(normal.m_y * normal.m_y + normal.m_z * normal.m_z);
		dFloat32 cosAng = normal.m_y * magInv;
		dFloat32 sinAng = normal.m_z * magInv;

		dAssert(dAbs(normal.m_z * cosAng - normal.m_y * sinAng) < dFloat32(1.0e-4f));
		dVector normal1(normal.m_x, normal.m_y * cosAng + normal.m_z * sinAng, dFloat32(0.0f), dFloat32(0.0f));
		dVector origin1(origin.m_x, origin.m_y * cosAng + origin.m_z * sinAng, origin.m_z * cosAng - origin.m_y * sinAng, dFloat32(0.0f));

		count = 0;
		dInt32 i0 = 3;
		dVector test0((m_profile[i0] - origin1).DotProduct(normal1));
		for (dInt32 i = 0; (i < 4) && (count < 2); i++) 
		{
			dVector test1((m_profile[i] - origin1).DotProduct(normal1));
			dVector acrossPlane(test0 * test1);
			if (acrossPlane.m_x < 0.0f) 
			{
				dVector step(m_profile[i] - m_profile[i0]);
				contactsOut[count] = m_profile[i0] - step.Scale(test0.m_x / (step.DotProduct(normal1).m_x));
				count++;
			}
			i0 = i;
			test0 = test1;
		}

		for (dInt32 i = 0; i < count; i++) 
		{
			dFloat32 y = contactsOut[i].m_y;
			dFloat32 z = contactsOut[i].m_z;
			contactsOut[i].m_y = y * cosAng - z * sinAng;
			contactsOut[i].m_z = z * cosAng + y * sinAng;
		}
	}
	return count;
}

void ndShapeCylinder::CalculateAabb(const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	ndShapeConvex::CalculateAabb(matrix, p0, p1);
}

void ndShapeCylinder::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeConvex::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "radius0", m_radius0);
	xmlSaveParam(childNode, "radius1", m_radius1);
	xmlSaveParam(childNode, "height", m_height * dFloat32 (2.0f));
}