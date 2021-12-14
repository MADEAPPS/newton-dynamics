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
#include "ndShapeBox.h"
#include "ndContactSolver.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeBox)

ndInt32 ndShapeBox::m_initSimplex = 0;
ndShapeConvex::ndConvexSimplexEdge ndShapeBox::m_edgeArray[24];
ndShapeConvex::ndConvexSimplexEdge* ndShapeBox::m_edgeEdgeMap[12];
ndShapeConvex::ndConvexSimplexEdge* ndShapeBox::m_vertexToEdgeMap[8];
ndInt32 ndShapeBox::m_faces[][4] =
{
	{0, 1, 3, 2},
	{0, 4, 5, 1},
	{1, 5, 7, 3},
	{0, 2, 6, 4},
	{2, 3, 7, 6},
	{4, 6, 7, 5},
};

ndVector ndShapeBox::m_indexMark(ndFloat32(1.0f), ndFloat32(2.0f), ndFloat32(4.0f), ndFloat32(0.0f));
ndVector ndShapeBox::m_penetrationTol(D_PENETRATION_TOL, D_PENETRATION_TOL, D_PENETRATION_TOL, ndFloat32(0.0f));

ndShapeBox::ndShapeBox(ndFloat32 size_x, ndFloat32 size_y, ndFloat32 size_z)
	:ndShapeConvex(m_box)
{
	Init(size_x, size_y, size_z);
}

ndShapeBox::ndShapeBox(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndShapeConvex(m_box)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	ndFloat32 size_x = xmlGetFloat(xmlNode, "size_x");
	ndFloat32 size_y = xmlGetFloat(xmlNode, "size_y");
	ndFloat32 size_z = xmlGetFloat(xmlNode, "size_z");
	Init(size_x, size_y, size_z);
}

ndShapeBox::~ndShapeBox()
{
	ndShapeConvex::m_simplex = nullptr;
	ndShapeConvex::m_vertex = nullptr;
}

void ndShapeBox::Init(ndFloat32 size_x, ndFloat32 size_y, ndFloat32 size_z)
{
	m_size[0].m_x = dMax(dAbs(size_x) * ndFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);
	m_size[0].m_y = dMax(dAbs(size_y) * ndFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);
	m_size[0].m_z = dMax(dAbs(size_z) * ndFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);
	m_size[0].m_w = ndFloat32(0.0f);

	m_size[1].m_x = -m_size[0].m_x;
	m_size[1].m_y = -m_size[0].m_y;
	m_size[1].m_z = -m_size[0].m_z;
	m_size[1].m_w = ndFloat32(0.0f);

	m_edgeCount = 24;
	m_vertexCount = 8;

	m_vertex[0] = ndVector(m_size[0].m_x, m_size[0].m_y, m_size[0].m_z, ndFloat32(0.0f));
	m_vertex[1] = ndVector(-m_size[0].m_x, m_size[0].m_y, m_size[0].m_z, ndFloat32(0.0f));
	m_vertex[2] = ndVector(m_size[0].m_x, -m_size[0].m_y, m_size[0].m_z, ndFloat32(0.0f));
	m_vertex[3] = ndVector(-m_size[0].m_x, -m_size[0].m_y, m_size[0].m_z, ndFloat32(0.0f));

	m_vertex[4] = ndVector(m_size[0].m_x, m_size[0].m_y, -m_size[0].m_z, ndFloat32(0.0f));
	m_vertex[5] = ndVector(-m_size[0].m_x, m_size[0].m_y, -m_size[0].m_z, ndFloat32(0.0f));
	m_vertex[6] = ndVector(m_size[0].m_x, -m_size[0].m_y, -m_size[0].m_z, ndFloat32(0.0f));
	m_vertex[7] = ndVector(-m_size[0].m_x, -m_size[0].m_y, -m_size[0].m_z, ndFloat32(0.0f));

	ndShapeConvex::m_vertex = m_vertex;
	ndShapeConvex::m_simplex = m_edgeArray;
	
	if (!m_initSimplex) 
	{
		ndPolyhedra polyhedra;
		polyhedra.BeginFace();
		for (ndInt32 i = 0; i < 6; i++) 
		{
			polyhedra.AddFace(4, &m_faces[i][0]);
		}
		polyhedra.EndFace();
	
		ndInt32 index = 0;
		ndInt32 mark = polyhedra.IncLRU();;
		ndPolyhedra::Iterator iter(polyhedra);
		for (iter.Begin(); iter; iter++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				ndEdge* ptr = edge;
				do {
					ptr->m_mark = mark;
					ptr->m_userData = index;
					index++;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}
		dAssert(index == 24);
	
		polyhedra.IncLRU();
		mark = polyhedra.IncLRU();
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			ndEdge *ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ndConvexSimplexEdge* const simplexPtr = &m_simplex[ptr->m_userData];
				simplexPtr->m_vertex = ptr->m_incidentVertex;
				simplexPtr->m_next = &m_simplex[ptr->m_next->m_userData];
				simplexPtr->m_prev = &m_simplex[ptr->m_prev->m_userData];
				simplexPtr->m_twin = &m_simplex[ptr->m_twin->m_userData];
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			m_vertexToEdgeMap[edge->m_incidentVertex] = &m_simplex[edge->m_userData];
		}
	
		ndInt32 count = 0;
		mark = polyhedra.IncLRU();
		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) 
			{
				edge->m_mark = mark;
				edge->m_twin->m_mark = mark;
				m_edgeEdgeMap[count] = &m_simplex[edge->m_userData];
				count++;
				dAssert(count <= 12);
			}
		}
	
		m_initSimplex = 1;
	}

	SetVolumeAndCG();
}

void ndShapeBox::MassProperties()
{
	m_centerOfMass = ndVector::m_zero;
	m_crossInertia = ndVector::m_zero;
	ndFloat32 volume = ndFloat32(8.0f) * m_size[0].m_x * m_size[0].m_y * m_size[0].m_z;
	m_inertia = ndVector(ndFloat32(1.0f / 3.0f) * (m_size[0].m_y * m_size[0].m_y + m_size[0].m_z * m_size[0].m_z),
						ndFloat32(1.0f / 3.0f) * (m_size[0].m_x * m_size[0].m_x + m_size[0].m_z * m_size[0].m_z),
						ndFloat32(1.0f / 3.0f) * (m_size[0].m_x * m_size[0].m_x + m_size[0].m_y * m_size[0].m_y),
						ndFloat32(0.0f));
	m_centerOfMass.m_w = volume;
}

void ndShapeBox::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	ndVector size(matrix[0].Abs().Scale(m_size[0].m_x) + matrix[1].Abs().Scale(m_size[0].m_y) + matrix[2].Abs().Scale(m_size[0].m_z));
	p0 = (matrix[3] - size) & ndVector::m_triplexMask;
	p1 = (matrix[3] + size) & ndVector::m_triplexMask;
}

ndVector ndShapeBox::SupportVertex(const ndVector& dir0, ndInt32* const vertexIndex) const
{
	ndVector mask0(dir0.Abs() > m_flushZero);
	ndVector dir(dir0 & mask0);

	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
	dAssert(dir.m_w == ndFloat32(0.0f));
	ndVector mask(dir < ndVector::m_zero);
	if (vertexIndex) 
	{
		ndVector index(m_indexMark * (mask & ndVector::m_one));
		index = (index.AddHorizontal()).GetInt();
		*vertexIndex = ndInt32(index.m_ix);
	}
	return m_size[0].Select(m_size[1], mask);
}

ndVector ndShapeBox::SupportVertexSpecial(const ndVector& dir0, ndFloat32, ndInt32* const vertexIndex) const
{
	ndVector mask0(dir0.Abs() > m_flushZero);
	ndVector dir(dir0 & mask0);
	
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
	dAssert(dir.m_w == ndFloat32(0.0f));
	ndVector mask(dir < ndVector::m_zero);
	if (vertexIndex) 
	{
		ndVector index(m_indexMark * (mask & ndVector::m_one));
		index = (index.AddHorizontal()).GetInt();
		*vertexIndex = ndInt32(index.m_ix);
	}
	
	ndVector size0(m_size[0] - m_penetrationTol);
	ndVector size1(m_size[1] + m_penetrationTol);
	return size0.Select(size1, mask);
}

ndVector ndShapeBox::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir0) const
{
	ndVector mask0(dir0.Abs() > m_flushZero);
	ndVector dir(dir0 & mask0);
	dAssert(dAbs((dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f))) < ndFloat32(1.0e-3f));
	return point + dir.Scale(D_PENETRATION_TOL);
}

ndFloat32 ndShapeBox::RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32, const ndBody* const, ndContactPoint& contactOut) const
{
	dAssert(localP0.m_w == localP1.m_w);

	ndInt32 index = 0;
	ndFloat32 signDir = ndFloat32(0.0f);
	ndFloat32 tmin = ndFloat32(0.0f);
	ndFloat32 tmax = ndFloat32(1.0f);
	for (ndInt32 i = 0; i < 3; i++) 
	{
		ndFloat32 dp = localP1[i] - localP0[i];
		if (dAbs(dp) < ndFloat32(1.0e-8f)) 
		{
			if (localP0[i] <= m_size[1][i] || localP0[i] >= m_size[0][i]) 
			{
				return ndFloat32(1.2f);
			}
		}
		else 
		{
			dp = ndFloat32(1.0f) / dp;
			ndFloat32 t1 = (m_size[1][i] - localP0[i]) * dp;
			ndFloat32 t2 = (m_size[0][i] - localP0[i]) * dp;

			ndFloat32 sign = ndFloat32(-1.0f);
			if (t1 > t2) 
			{
				sign = 1;
				dSwap(t1, t2);
			}
			if (t1 > tmin) 
			{
				signDir = sign;
				index = i;
				tmin = t1;
			}
			if (t2 < tmax) 
			{
				tmax = t2;
			}
			if (tmin > tmax) 
			{
				return ndFloat32(1.2f);
			}
		}
	}

	if (tmin > ndFloat32(0.0f)) 
	{
		dAssert(tmin <= 1.0f);
		contactOut.m_normal = ndVector::m_zero;
		contactOut.m_normal[index] = signDir;
		//contactOut.m_userId = SetUserDataID();
	}
	else 
	{
		tmin = ndFloat32(1.2f);
	}
	return tmin;
}

const ndShapeConvex::ndConvexSimplexEdge** ndShapeBox::GetVertexToEdgeMapping() const
{
	return (const ndConvexSimplexEdge**)&m_vertexToEdgeMap[0];
}

ndInt32 ndShapeBox::CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const
{
	ndVector support[4];
	ndInt32 featureCount = 3;

	const ndConvexSimplexEdge** const vertToEdgeMapping = GetVertexToEdgeMapping();
	if (vertToEdgeMapping) 
	{
		ndInt32 edgeIndex;
		support[0] = SupportVertex(normal, &edgeIndex);

		ndFloat32 dist = normal.DotProduct(support[0] - point).GetScalar();
		if (dist <= D_PENETRATION_TOL) 
		{
			ndVector normalAlgin(normal.Abs());
			if (!((normalAlgin.m_x > ndFloat32(0.9999f)) || (normalAlgin.m_y > ndFloat32(0.9999f)) || (normalAlgin.m_z > ndFloat32(0.9999f)))) 
			{
				// 0.25 degrees
				const ndFloat32 tiltAngle = ndFloat32(0.005f);
				const ndFloat32 tiltAngle2 = tiltAngle * tiltAngle;
				ndPlane testPlane(normal, -(normal.DotProduct(support[0]).GetScalar()));

				featureCount = 1;
				const ndConvexSimplexEdge* const edge = vertToEdgeMapping[edgeIndex];
				const ndConvexSimplexEdge* ptr = edge;
				do 
				{
					const ndVector& p = m_vertex[ptr->m_twin->m_vertex];
					ndFloat32 test1 = testPlane.Evalue(p);
					ndVector dist1(p - support[0]);
					ndFloat32 angle2 = test1 * test1 / (dist1.DotProduct(dist1).GetScalar());
					if (angle2 < tiltAngle2) 
					{
						support[featureCount] = p;
						featureCount++;
					}
					ptr = ptr->m_twin->m_next;
				} while ((ptr != edge) && (featureCount < 3));
			}
		}
	}

	ndInt32 count = 0;
	switch (featureCount)
	{
		case 1:
		{
			contactsOut[0] = support[0] - normal * normal.DotProduct(support[0] - point);
			count = 1;
			break;
		}

		case 2:
		{
			contactsOut[0] = support[0] - normal * normal.DotProduct(support[0] - point);
			contactsOut[1] = support[1] - normal * normal.DotProduct(support[1] - point);
			count = 2;
			break;
		}

		default:
		{
			ndFloat32 test[8];
			dAssert(normal.m_w == ndFloat32(0.0f));
			ndPlane plane(normal, -(normal.DotProduct(point).GetScalar()));
			for (ndInt32 i = 0; i < 8; i++) 
			{
				dAssert(m_vertex[i].m_w == ndFloat32(0.0f));
				test[i] = plane.DotProduct(m_vertex[i] | ndVector::m_wOne).m_x;
			}

			ndConvexSimplexEdge* edge = nullptr;
			for (ndInt32 i = 0; i < ndInt32(sizeof(m_edgeEdgeMap) / sizeof(m_edgeEdgeMap[0])); i++) 
			{
				ndConvexSimplexEdge* const ptr = m_edgeEdgeMap[i];
				ndFloat32 side0 = test[ptr->m_vertex];
				ndFloat32 side1 = test[ptr->m_twin->m_vertex];
				if ((side0 * side1) < ndFloat32(0.0f)) 
				{
					edge = ptr;
					break;
				}
			}

			if (edge) 
			{
				if (test[edge->m_vertex] < ndFloat32(0.0f)) 
				{
					edge = edge->m_twin;
				}
				dAssert(test[edge->m_vertex] > ndFloat32(0.0f));

				ndConvexSimplexEdge* ptr = edge;
				ndConvexSimplexEdge* firstEdge = nullptr;
				ndFloat32 side0 = test[edge->m_vertex];
				do 
				{
					dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == ndFloat32(0.0f));
					ndFloat32 side1 = test[ptr->m_twin->m_vertex];
					if (side1 < side0) 
					{
						if (side1 < ndFloat32(0.0f)) 
						{
							firstEdge = ptr;
							break;
						}

						side0 = side1;
						edge = ptr->m_twin;
						ptr = edge;
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);

				if (firstEdge) 
				{
					edge = firstEdge;
					ptr = edge;
					do 
					{
						ndVector dp(m_vertex[ptr->m_twin->m_vertex] - m_vertex[ptr->m_vertex]);
						ndFloat32 t = plane.DotProduct(dp).m_x;
						if (t >= ndFloat32(-1.e-24f)) 
						{
							t = ndFloat32(0.0f);
						}
						else 
						{
							t = test[ptr->m_vertex] / t;
							if (t > ndFloat32(0.0f)) 
							{
								t = ndFloat32(0.0f);
							}
							if (t < ndFloat32(-1.0f)) 
							{
								t = ndFloat32(-1.0f);
							}
						}

						dAssert(t <= ndFloat32(0.01f));
						dAssert(t >= ndFloat32(-1.05f));
						contactsOut[count] = m_vertex[ptr->m_vertex] - dp.Scale(t);
						count++;

						ndConvexSimplexEdge* ptr1 = ptr->m_next;
						for (; ptr1 != ptr; ptr1 = ptr1->m_next) 
						{
							ndInt32 index0 = ptr1->m_twin->m_vertex;
							if (test[index0] >= ndFloat32(0.0f)) 
							{
								dAssert(test[ptr1->m_vertex] <= ndFloat32(0.0f));
								break;
							}
						}
						dAssert(ptr != ptr1);
						ptr = ptr1->m_twin;

					} while ((ptr != edge) && (count < 8));
				}
			}
		}
	}

	if (count > 2) 
	{
		count = RectifyConvexSlice(count, normal, contactsOut);
	}
	return count;
}

ndShapeInfo ndShapeBox::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());
	info.m_box.m_x = m_size[0].m_x * ndFloat32(2.0f);
	info.m_box.m_y = m_size[0].m_y * ndFloat32(2.0f);
	info.m_box.m_z = m_size[0].m_z * ndFloat32(2.0f);
	return info;
}

D_COLLISION_API void ndShapeBox::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeConvex::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "size_x", m_size[0][0] * ndFloat32(2.0f));
	xmlSaveParam(childNode, "size_y", m_size[0][1] * ndFloat32(2.0f));
	xmlSaveParam(childNode, "size_z", m_size[0][2] * ndFloat32(2.0f));
}