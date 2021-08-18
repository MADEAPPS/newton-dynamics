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
#include "ndShapeBox.h"
#include "ndContactSolver.h"

dInt32 ndShapeBox::m_initSimplex = 0;
ndShapeConvex::ndConvexSimplexEdge ndShapeBox::m_edgeArray[24];
ndShapeConvex::ndConvexSimplexEdge* ndShapeBox::m_edgeEdgeMap[12];
ndShapeConvex::ndConvexSimplexEdge* ndShapeBox::m_vertexToEdgeMap[8];
dInt32 ndShapeBox::m_faces[][4] =
{
	{0, 1, 3, 2},
	{0, 4, 5, 1},
	{1, 5, 7, 3},
	{0, 2, 6, 4},
	{2, 3, 7, 6},
	{4, 6, 7, 5},
};

dVector ndShapeBox::m_indexMark(dFloat32(1.0f), dFloat32(2.0f), dFloat32(4.0f), dFloat32(0.0f));
dVector ndShapeBox::m_penetrationTol(D_PENETRATION_TOL, D_PENETRATION_TOL, D_PENETRATION_TOL, dFloat32(0.0f));

ndShapeBox::ndShapeBox(dFloat32 size_x, dFloat32 size_y, dFloat32 size_z)
	:ndShapeConvex(m_box)
{
	Init(size_x, size_y, size_z);
}

ndShapeBox::ndShapeBox(const nd::TiXmlNode* const xmlNode)
	:ndShapeConvex(m_box)
{
	dFloat32 size_x = xmlGetFloat(xmlNode, "size_x");
	dFloat32 size_y = xmlGetFloat(xmlNode, "size_y");
	dFloat32 size_z = xmlGetFloat(xmlNode, "size_z");
	Init(size_x, size_y, size_z);
}

ndShapeBox::~ndShapeBox()
{
	ndShapeConvex::m_simplex = nullptr;
	ndShapeConvex::m_vertex = nullptr;
}

void ndShapeBox::Init(dFloat32 size_x, dFloat32 size_y, dFloat32 size_z)
{
	m_size[0].m_x = dMax(dAbs(size_x) * dFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);
	m_size[0].m_y = dMax(dAbs(size_y) * dFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);
	m_size[0].m_z = dMax(dAbs(size_z) * dFloat32(0.5f), D_MIN_CONVEX_SHAPE_SIZE);
	m_size[0].m_w = dFloat32(0.0f);

	m_size[1].m_x = -m_size[0].m_x;
	m_size[1].m_y = -m_size[0].m_y;
	m_size[1].m_z = -m_size[0].m_z;
	m_size[1].m_w = dFloat32(0.0f);

	m_edgeCount = 24;
	m_vertexCount = 8;

	m_vertex[0] = dVector(m_size[0].m_x, m_size[0].m_y, m_size[0].m_z, dFloat32(0.0f));
	m_vertex[1] = dVector(-m_size[0].m_x, m_size[0].m_y, m_size[0].m_z, dFloat32(0.0f));
	m_vertex[2] = dVector(m_size[0].m_x, -m_size[0].m_y, m_size[0].m_z, dFloat32(0.0f));
	m_vertex[3] = dVector(-m_size[0].m_x, -m_size[0].m_y, m_size[0].m_z, dFloat32(0.0f));

	m_vertex[4] = dVector(m_size[0].m_x, m_size[0].m_y, -m_size[0].m_z, dFloat32(0.0f));
	m_vertex[5] = dVector(-m_size[0].m_x, m_size[0].m_y, -m_size[0].m_z, dFloat32(0.0f));
	m_vertex[6] = dVector(m_size[0].m_x, -m_size[0].m_y, -m_size[0].m_z, dFloat32(0.0f));
	m_vertex[7] = dVector(-m_size[0].m_x, -m_size[0].m_y, -m_size[0].m_z, dFloat32(0.0f));

	ndShapeConvex::m_vertex = m_vertex;
	ndShapeConvex::m_simplex = m_edgeArray;
	
	if (!m_initSimplex) 
	{
		dPolyhedra polyhedra;
		polyhedra.BeginFace();
		for (dInt32 i = 0; i < 6; i++) 
		{
			polyhedra.AddFace(4, &m_faces[i][0]);
		}
		polyhedra.EndFace();
	
		dInt32 index = 0;
		dInt32 mark = polyhedra.IncLRU();;
		dPolyhedra::Iterator iter(polyhedra);
		for (iter.Begin(); iter; iter++) {
			dEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				dEdge* ptr = edge;
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
			dEdge* const edge = &iter.GetNode()->GetInfo();
			dEdge *ptr = edge;
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
			dEdge* const edge = &iter.GetNode()->GetInfo();
			m_vertexToEdgeMap[edge->m_incidentVertex] = &m_simplex[edge->m_userData];
		}
	
		dInt32 count = 0;
		mark = polyhedra.IncLRU();
		for (iter.Begin(); iter; iter++) 
		{
			dEdge* const edge = &iter.GetNode()->GetInfo();
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
	m_centerOfMass = dVector::m_zero;
	m_crossInertia = dVector::m_zero;
	dFloat32 volume = dFloat32(8.0f) * m_size[0].m_x * m_size[0].m_y * m_size[0].m_z;
	m_inertia = dVector(dFloat32(1.0f / 3.0f) * (m_size[0].m_y * m_size[0].m_y + m_size[0].m_z * m_size[0].m_z),
						dFloat32(1.0f / 3.0f) * (m_size[0].m_x * m_size[0].m_x + m_size[0].m_z * m_size[0].m_z),
						dFloat32(1.0f / 3.0f) * (m_size[0].m_x * m_size[0].m_x + m_size[0].m_y * m_size[0].m_y),
						dFloat32(0.0f));
	m_centerOfMass.m_w = volume;
}

void ndShapeBox::CalculateAabb(const dMatrix& matrix, dVector &p0, dVector &p1) const
{
	dVector size(matrix[0].Abs().Scale(m_size[0].m_x) + matrix[1].Abs().Scale(m_size[0].m_y) + matrix[2].Abs().Scale(m_size[0].m_z));
	p0 = (matrix[3] - size) & dVector::m_triplexMask;
	p1 = (matrix[3] + size) & dVector::m_triplexMask;
}

dVector ndShapeBox::SupportVertex(const dVector& dir0, dInt32* const vertexIndex) const
{
	dVector mask0(dir0.Abs() > m_flushZero);
	dVector dir(dir0 & mask0);

	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	dAssert(dir.m_w == dFloat32(0.0f));
	dVector mask(dir < dVector::m_zero);
	if (vertexIndex) 
	{
		dVector index(m_indexMark * (mask & dVector::m_one));
		index = (index.AddHorizontal()).GetInt();
		*vertexIndex = dInt32(index.m_ix);
	}
	return m_size[0].Select(m_size[1], mask);
}

dVector ndShapeBox::SupportVertexSpecial(const dVector& dir0, dFloat32, dInt32* const vertexIndex) const
{
	dVector mask0(dir0.Abs() > m_flushZero);
	dVector dir(dir0 & mask0);
	
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	dAssert(dir.m_w == dFloat32(0.0f));
	dVector mask(dir < dVector::m_zero);
	if (vertexIndex) 
	{
		dVector index(m_indexMark * (mask & dVector::m_one));
		index = (index.AddHorizontal()).GetInt();
		*vertexIndex = dInt32(index.m_ix);
	}
	
	dVector size0(m_size[0] - m_penetrationTol);
	dVector size1(m_size[1] + m_penetrationTol);
	return size0.Select(size1, mask);
}

dVector ndShapeBox::SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir0) const
{
	dVector mask0(dir0.Abs() > m_flushZero);
	dVector dir(dir0 & mask0);
	dAssert(dAbs((dir.DotProduct(dir).GetScalar() - dFloat32(1.0f))) < dFloat32(1.0e-3f));
	return point + dir.Scale(D_PENETRATION_TOL);
}

dFloat32 ndShapeBox::RayCast(ndRayCastNotify&, const dVector& localP0, const dVector& localP1, dFloat32, const ndBody* const, ndContactPoint& contactOut) const
{
	dAssert(localP0.m_w == localP1.m_w);

	dInt32 index = 0;
	dFloat32 signDir = dFloat32(0.0f);
	dFloat32 tmin = dFloat32(0.0f);
	dFloat32 tmax = dFloat32(1.0f);
	for (dInt32 i = 0; i < 3; i++) 
	{
		dFloat32 dp = localP1[i] - localP0[i];
		if (dAbs(dp) < dFloat32(1.0e-8f)) 
		{
			if (localP0[i] <= m_size[1][i] || localP0[i] >= m_size[0][i]) 
			{
				return dFloat32(1.2f);
			}
		}
		else 
		{
			dp = dFloat32(1.0f) / dp;
			dFloat32 t1 = (m_size[1][i] - localP0[i]) * dp;
			dFloat32 t2 = (m_size[0][i] - localP0[i]) * dp;

			dFloat32 sign = dFloat32(-1.0f);
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
				return dFloat32(1.2f);
			}
		}
	}

	if (tmin > dFloat32(0.0f)) 
	{
		dAssert(tmin <= 1.0f);
		contactOut.m_normal = dVector::m_zero;
		contactOut.m_normal[index] = signDir;
		//contactOut.m_userId = SetUserDataID();
	}
	else 
	{
		tmin = dFloat32(1.2f);
	}
	return tmin;
}

const ndShapeConvex::ndConvexSimplexEdge** ndShapeBox::GetVertexToEdgeMapping() const
{
	return (const ndConvexSimplexEdge**)&m_vertexToEdgeMap[0];
}

dInt32 ndShapeBox::CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const
{
	dVector support[4];
	dInt32 featureCount = 3;

	const ndConvexSimplexEdge** const vertToEdgeMapping = GetVertexToEdgeMapping();
	if (vertToEdgeMapping) 
	{
		dInt32 edgeIndex;
		support[0] = SupportVertex(normal, &edgeIndex);

		dFloat32 dist = normal.DotProduct(support[0] - point).GetScalar();
		if (dist <= D_PENETRATION_TOL) 
		{
			dVector normalAlgin(normal.Abs());
			if (!((normalAlgin.m_x > dFloat32(0.9999f)) || (normalAlgin.m_y > dFloat32(0.9999f)) || (normalAlgin.m_z > dFloat32(0.9999f)))) 
			{
				// 0.25 degrees
				const dFloat32 tiltAngle = dFloat32(0.005f);
				const dFloat32 tiltAngle2 = tiltAngle * tiltAngle;
				dPlane testPlane(normal, -(normal.DotProduct(support[0]).GetScalar()));

				featureCount = 1;
				const ndConvexSimplexEdge* const edge = vertToEdgeMapping[edgeIndex];
				const ndConvexSimplexEdge* ptr = edge;
				do 
				{
					const dVector& p = m_vertex[ptr->m_twin->m_vertex];
					dFloat32 test1 = testPlane.Evalue(p);
					dVector dist1(p - support[0]);
					dFloat32 angle2 = test1 * test1 / (dist1.DotProduct(dist1).GetScalar());
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

	dInt32 count = 0;
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
			dFloat32 test[8];
			dAssert(normal.m_w == dFloat32(0.0f));
			dPlane plane(normal, -(normal.DotProduct(point).GetScalar()));
			for (dInt32 i = 0; i < 8; i++) 
			{
				dAssert(m_vertex[i].m_w == dFloat32(0.0f));
				test[i] = plane.DotProduct(m_vertex[i] | dVector::m_wOne).m_x;
			}

			ndConvexSimplexEdge* edge = nullptr;
			for (dInt32 i = 0; i < dInt32(sizeof(m_edgeEdgeMap) / sizeof(m_edgeEdgeMap[0])); i++) 
			{
				ndConvexSimplexEdge* const ptr = m_edgeEdgeMap[i];
				dFloat32 side0 = test[ptr->m_vertex];
				dFloat32 side1 = test[ptr->m_twin->m_vertex];
				if ((side0 * side1) < dFloat32(0.0f)) 
				{
					edge = ptr;
					break;
				}
			}

			if (edge) 
			{
				if (test[edge->m_vertex] < dFloat32(0.0f)) 
				{
					edge = edge->m_twin;
				}
				dAssert(test[edge->m_vertex] > dFloat32(0.0f));

				ndConvexSimplexEdge* ptr = edge;
				ndConvexSimplexEdge* firstEdge = nullptr;
				dFloat32 side0 = test[edge->m_vertex];
				do 
				{
					dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == dFloat32(0.0f));
					dFloat32 side1 = test[ptr->m_twin->m_vertex];
					if (side1 < side0) 
					{
						if (side1 < dFloat32(0.0f)) 
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
						dVector dp(m_vertex[ptr->m_twin->m_vertex] - m_vertex[ptr->m_vertex]);
						dFloat32 t = plane.DotProduct(dp).m_x;
						if (t >= dFloat32(-1.e-24f)) 
						{
							t = dFloat32(0.0f);
						}
						else 
						{
							t = test[ptr->m_vertex] / t;
							if (t > dFloat32(0.0f)) 
							{
								t = dFloat32(0.0f);
							}
							if (t < dFloat32(-1.0f)) 
							{
								t = dFloat32(-1.0f);
							}
						}

						dAssert(t <= dFloat32(0.01f));
						dAssert(t >= dFloat32(-1.05f));
						contactsOut[count] = m_vertex[ptr->m_vertex] - dp.Scale(t);
						count++;

						ndConvexSimplexEdge* ptr1 = ptr->m_next;
						for (; ptr1 != ptr; ptr1 = ptr1->m_next) 
						{
							dInt32 index0 = ptr1->m_twin->m_vertex;
							if (test[index0] >= dFloat32(0.0f)) 
							{
								dAssert(test[ptr1->m_vertex] <= dFloat32(0.0f));
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
	info.m_box.m_x = m_size[0].m_x * dFloat32(2.0f);
	info.m_box.m_y = m_size[0].m_y * dFloat32(2.0f);
	info.m_box.m_z = m_size[0].m_z * dFloat32(2.0f);
	return info;
}

D_COLLISION_API void ndShapeBox::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeHash) const
{
	nd::TiXmlElement* const shapeNode = new nd::TiXmlElement("className");
	rootNode->LinkEndChild(shapeNode);
	shapeNode->SetAttribute(ClassName(), nodeHash);
	ndShapeConvex::Save(shapeNode, assetPath, nodeHash);

	xmlSaveParam(shapeNode, "size_x", m_size[0][0] * dFloat32(2.0f));
	xmlSaveParam(shapeNode, "size_y", m_size[0][1] * dFloat32(2.0f));
	xmlSaveParam(shapeNode, "size_z", m_size[0][2] * dFloat32(2.0f));
}