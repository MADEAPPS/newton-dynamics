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
#include "ndShapeSphere.h"
#include "ndContactSolver.h"


#define D_SPHERE_EDGE_COUNT 96

dInt32 ndShapeSphere::m_shapeRefCount = 0;
dVector ndShapeSphere::m_unitSphere[D_SPHERE_VERTEX_COUNT];
ndShapeConvex::ndConvexSimplexEdge ndShapeSphere::m_edgeArray[D_SPHERE_EDGE_COUNT];


ndShapeSphere::ndShapeSphere(dFloat32 radius)
	:ndShapeConvex(m_sphereCollision)
{
	Init(radius);
}

ndShapeSphere::ndShapeSphere(const nd::TiXmlNode* const xmlNode)
	: ndShapeConvex(m_sphereCollision)
{
	dFloat32 radius = xmlGetFloat(xmlNode, "radius");
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
	//dMatrix transp(matrix.Transpose4X4());
	////dVector size(matrix.m_front.Abs().Scale(m_radius) + matrix.m_up.Abs().Scale(m_radius) + matrix.m_right.Abs().Scale(m_radius));
	//dVector size(transp.m_front.Abs() + transp.m_up.Abs() + transp.m_right.Abs());
	//size = size.Scale(m_radius);
	dVector size(m_radius);
	p0 = (matrix[3] - size) & dVector::m_triplexMask;
	p1 = (matrix[3] + size) & dVector::m_triplexMask;
}

dVector ndShapeSphere::SupportVertexSpecialProjectPoint(const dVector&, const dVector& dir) const
{
	return dir.Scale(m_radius - D_PENETRATION_TOL);
}

dVector ndShapeSphere::SupportVertexSpecial(const dVector&, dFloat32, dInt32* const) const
{
	return dVector::m_zero;
}

dVector ndShapeSphere::SupportVertex(const dVector& dir, dInt32* const) const
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

dFloat32 ndShapeSphere::RayCast(ndRayCastNotify&, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const
{
	dFloat32 t = dRayCastSphere(localP0, localP1, dVector::m_zero, m_radius);
	if (t < maxT) 
	{
		dVector contact(localP0 + (localP1 - localP0).Scale(t));
		dAssert(contact.m_w == dFloat32(0.0f));
		//contactOut.m_normal = contact.Scale (dgRsqrt (contact.DotProduct(contact).GetScalar()));
		contactOut.m_normal = contact.Normalize();
		//contactOut.m_userId = SetUserDataID();
	}
	return t;
}

ndShapeInfo ndShapeSphere::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());
	info.m_sphere.m_radius = m_radius;
	return info;
}

void ndShapeSphere::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	dVector tmpVectex[1024 * 2];
	ndShapeDebugCallback::ndEdgeType edgeType[1024 * 2];

	dVector p0(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector p1(-dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector p2(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector p3(dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
	dVector p4(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
	dVector p5(dFloat32(0.0f), dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f));

	dInt32 index = 3;
	dInt32 count = 0;
	TesselateTriangle(index, p4, p0, p2, count, tmpVectex);
	TesselateTriangle(index, p4, p2, p1, count, tmpVectex);
	TesselateTriangle(index, p4, p1, p3, count, tmpVectex);
	TesselateTriangle(index, p4, p3, p0, count, tmpVectex);
	TesselateTriangle(index, p5, p2, p0, count, tmpVectex);
	TesselateTriangle(index, p5, p1, p2, count, tmpVectex);
	TesselateTriangle(index, p5, p3, p1, count, tmpVectex);
	TesselateTriangle(index, p5, p0, p3, count, tmpVectex);

	for (dInt32 i = 0; i < count; i++) 
	{
		edgeType[i] = ndShapeDebugCallback::m_shared;
		tmpVectex[i] = matrix.TransformVector(tmpVectex[i].Scale(m_radius)) & dVector::m_triplexMask;
	}

	for (dInt32 i = 0; i < count; i += 3) 
	{
		debugCallback.DrawPolygon(3, &tmpVectex[i], &edgeType[i]);
	}
}

D_COLLISION_API void ndShapeSphere::Save( nd::TiXmlElement* const xmlNode, const char* const, dInt32 nodeid ) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement("ndShapeSphere");
	xmlNode->LinkEndChild(paramNode);

	paramNode->SetAttribute("nodeId", nodeid);

	xmlSaveParam(paramNode, "radius", m_radius);
}