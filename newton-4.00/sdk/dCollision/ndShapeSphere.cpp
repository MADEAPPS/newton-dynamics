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
#include "ndShapeSphere.h"
#include "ndContactSolver.h"

#define D_SPHERE_EDGE_COUNT 96
D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeSphere)

ndInt32 ndShapeSphere::m_shapeRefCount = 0;
ndVector ndShapeSphere::m_unitSphere[D_SPHERE_VERTEX_COUNT];
ndShapeConvex::ndConvexSimplexEdge ndShapeSphere::m_edgeArray[D_SPHERE_EDGE_COUNT];

ndShapeSphere::ndShapeSphere(ndFloat32 radius)
	:ndShapeConvex(m_sphere)
{
	Init(radius);
}

ndShapeSphere::ndShapeSphere(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndShapeConvex(m_sphere)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	ndFloat32 radius = xmlGetFloat(xmlNode, "radius");
	Init(radius);
}

ndShapeSphere::~ndShapeSphere()
{
	m_shapeRefCount--;
	dAssert(m_shapeRefCount >= 0);

	ndShapeConvex::m_simplex = nullptr;
	ndShapeConvex::m_vertex = nullptr;
}

void ndShapeSphere::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeConvex::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "radius", m_radius);
}

void ndShapeSphere::TesselateTriangle(ndInt32 level, const ndVector& p0, const ndVector& p1, const ndVector& p2, ndInt32& count, ndVector* const ouput) const
{
	if (level) 
	{
		dAssert(dAbs(p0.DotProduct(p0).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p1.DotProduct(p1).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p2.DotProduct(p2).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		ndVector p01(p0 + p1);
		ndVector p12(p1 + p2);
		ndVector p20(p2 + p0);

		p01 = p01.Normalize();
		p12 = p12.Normalize();
		p20 = p20.Normalize();

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
		ouput[count++] = p0;
		ouput[count++] = p1;
		ouput[count++] = p2;
	}
}

void ndShapeSphere::Init(ndFloat32 radius)
{
	m_radius = dMax(dAbs(radius), D_MIN_CONVEX_SHAPE_SIZE);
	
	m_edgeCount = D_SPHERE_EDGE_COUNT;
	m_vertexCount = D_SPHERE_VERTEX_COUNT;
	ndShapeConvex::m_vertex = m_vertex;
	
	if (!m_shapeRefCount) 
	{
		ndVector p0(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p1(-ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p2(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p3(ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p4(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
		ndVector p5(ndFloat32(0.0f), ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f));

		ndVector tmpVectex[256];
		ndInt32 indexList[256];
		ndInt32 index = 1;
		ndInt32 count = 0;
		TesselateTriangle(index, p4, p0, p2, count, tmpVectex);
		TesselateTriangle(index, p4, p2, p1, count, tmpVectex);
		TesselateTriangle(index, p4, p1, p3, count, tmpVectex);
		TesselateTriangle(index, p4, p3, p0, count, tmpVectex);
		TesselateTriangle(index, p5, p2, p0, count, tmpVectex);
		TesselateTriangle(index, p5, p1, p2, count, tmpVectex);
		TesselateTriangle(index, p5, p3, p1, count, tmpVectex);
		TesselateTriangle(index, p5, p0, p3, count, tmpVectex);

		ndInt32 vertexCount = dVertexListToIndexList(&tmpVectex[0].m_x, sizeof(ndVector), 3, count, indexList, 0.001f);

		dAssert(vertexCount == D_SPHERE_VERTEX_COUNT);
		for (ndInt32 i = 0; i < vertexCount; i++) 
		{
			m_unitSphere[i] = tmpVectex[i];
		}
		
		ndPolyhedra polyhedra;
		
		polyhedra.BeginFace();
		for (ndInt32 i = 0; i < count; i += 3) 
		{
			#ifdef _DEBUG
			ndEdge* const edge = polyhedra.AddFace(indexList[i], indexList[i + 1], indexList[i + 2]);
			dAssert(edge);
			#else 
			polyhedra.AddFace(indexList[i], indexList[i + 1], indexList[i + 2]);
			#endif
		}
		polyhedra.EndFace();
		
		ndUnsigned64 i1 = 0;
		ndPolyhedra::Iterator iter(polyhedra);
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &(*iter);
			edge->m_userData = i1;
			i1++;
		}
		
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &(*iter);
		
			ndConvexSimplexEdge* const ptr = &m_edgeArray[edge->m_userData];
		
			ptr->m_vertex = edge->m_incidentVertex;
			ptr->m_next = &m_edgeArray[edge->m_next->m_userData];
			ptr->m_prev = &m_edgeArray[edge->m_prev->m_userData];
			ptr->m_twin = &m_edgeArray[edge->m_twin->m_userData];
		}
	}
	
	for (ndInt32 i = 0; i < D_SPHERE_VERTEX_COUNT; i++) 
	{
		m_vertex[i] = m_unitSphere[i].Scale(m_radius);
	}
	
	m_shapeRefCount++;
	ndShapeConvex::m_simplex = m_edgeArray;
	SetVolumeAndCG();
}

void ndShapeSphere::MassProperties()
{
	m_centerOfMass = ndVector::m_zero;
	m_crossInertia = ndVector::m_zero;
	ndFloat32 volume = ndFloat32(4.0f * ndPi / 3.0f) * m_radius *  m_radius * m_radius;
	ndFloat32 II = ndFloat32(2.0f / 5.0f) * m_radius *  m_radius;
	m_inertia = ndVector(II, II, II, ndFloat32(0.0f));
	m_centerOfMass.m_w = volume;
}

void ndShapeSphere::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	//ndMatrix transp(matrix.Transpose4X4());
	////ndVector size(matrix.m_front.Abs().Scale(m_radius) + matrix.m_up.Abs().Scale(m_radius) + matrix.m_right.Abs().Scale(m_radius));
	//ndVector size(transp.m_front.Abs() + transp.m_up.Abs() + transp.m_right.Abs());
	//size = size.Scale(m_radius);
	ndVector size(m_radius);
	p0 = (matrix[3] - size) & ndVector::m_triplexMask;
	p1 = (matrix[3] + size) & ndVector::m_triplexMask;
}

ndVector ndShapeSphere::SupportVertexSpecialProjectPoint(const ndVector&, const ndVector& dir) const
{
	return dir.Scale(m_radius - D_PENETRATION_TOL);
}

ndVector ndShapeSphere::SupportVertexSpecial(const ndVector&, ndFloat32, ndInt32* const) const
{
	return ndVector::m_zero;
}

ndVector ndShapeSphere::SupportVertex(const ndVector& dir, ndInt32* const) const
{
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
	dAssert(dir.m_w == 0.0f);
	return dir.Scale(m_radius);
}

ndInt32 ndShapeSphere::CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const
{
	dAssert(normal.m_w == 0.0f);
	dAssert(normal.DotProduct(normal).GetScalar() > ndFloat32(0.999f));
	contactsOut[0] = normal * normal.DotProduct(point);
	return 1;
}

ndFloat32 ndShapeSphere::RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const
{
	ndFloat32 t = dRayCastSphere(localP0, localP1, ndVector::m_zero, m_radius);
	if (t < maxT) 
	{
		ndVector contact(localP0 + (localP1 - localP0).Scale(t));
		dAssert(contact.m_w == ndFloat32(0.0f));
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

void ndShapeSphere::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndVector tmpVectex[1024 * 2];
	ndShapeDebugNotify::ndEdgeType edgeType[1024 * 2];

	ndVector p0(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector p1(-ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector p2(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector p3(ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector p4(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	ndVector p5(ndFloat32(0.0f), ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f));

	ndInt32 index = 3;
	ndInt32 count = 0;
	TesselateTriangle(index, p4, p0, p2, count, tmpVectex);
	TesselateTriangle(index, p4, p2, p1, count, tmpVectex);
	TesselateTriangle(index, p4, p1, p3, count, tmpVectex);
	TesselateTriangle(index, p4, p3, p0, count, tmpVectex);
	TesselateTriangle(index, p5, p2, p0, count, tmpVectex);
	TesselateTriangle(index, p5, p1, p2, count, tmpVectex);
	TesselateTriangle(index, p5, p3, p1, count, tmpVectex);
	TesselateTriangle(index, p5, p0, p3, count, tmpVectex);

	for (ndInt32 i = 0; i < count; i++) 
	{
		edgeType[i] = ndShapeDebugNotify::m_shared;
		tmpVectex[i] = matrix.TransformVector(tmpVectex[i].Scale(m_radius)) & ndVector::m_triplexMask;
	}

	for (ndInt32 i = 0; i < count; i += 3) 
	{
		debugCallback.DrawPolygon(3, &tmpVectex[i], &edgeType[i]);
	}
}

