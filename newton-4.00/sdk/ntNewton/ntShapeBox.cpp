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

#include "ntStdafx.h"
#include "ntContact.h"
#include "ntShapeBox.h"

dInt32 ntShapeBox::m_initSimplex = 0;
ntShapeConvex::dConvexSimplexEdge ntShapeBox::m_edgeArray[24];
ntShapeConvex::dConvexSimplexEdge* ntShapeBox::m_edgeEdgeMap[12];
ntShapeConvex::dConvexSimplexEdge* ntShapeBox::m_vertexToEdgeMap[8];
dInt32 ntShapeBox::m_faces[][4] =
{
	{0, 1, 3, 2},
	{0, 4, 5, 1},
	{1, 5, 7, 3},
	{0, 2, 6, 4},
	{2, 3, 7, 6},
	{4, 6, 7, 5},
};

dVector ntShapeBox::m_indexMark(dFloat32(1.0f), dFloat32(2.0f), dFloat32(4.0f), dFloat32(0.0f));

ntShapeBox::ntShapeBox(dFloat32 size_x, dFloat32 size_y, dFloat32 size_z)
	:ntShapeConvex(m_boxCollision)
{
	Init(size_x, size_y, size_z);
}

ntShapeBox::~ntShapeBox()
{
	ntShapeConvex::m_simplex = NULL;
	ntShapeConvex::m_vertex = NULL;
}

void ntShapeBox::Init(dFloat32 size_x, dFloat32 size_y, dFloat32 size_z)
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

	ntShapeConvex::m_vertex = m_vertex;
	ntShapeConvex::m_simplex = m_edgeArray;
	
	if (!m_initSimplex) 
	{
		dPolyhedra polyhedra;
		polyhedra.BeginFace();
		for (dInt32 i = 0; i < 6; i++) 
		{
			polyhedra.AddFace(4, &m_faces[i][0]);
		}
		polyhedra.EndFace();
	
		int index = 0;
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
				dConvexSimplexEdge* const simplexPtr = &m_simplex[ptr->m_userData];
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

void ntShapeBox::MassProperties()
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

void ntShapeBox::CalcAABB(const dMatrix& matrix, dVector &p0, dVector &p1) const
{
	dVector size(matrix[0].Abs().Scale(m_size[0].m_x) + matrix[1].Abs().Scale(m_size[0].m_y) + matrix[2].Abs().Scale(m_size[0].m_z));
	p0 = (matrix[3] - size) & dVector::m_triplexMask;
	p1 = (matrix[3] + size) & dVector::m_triplexMask;
}

dVector ntShapeBox::SupportVertex(const dVector& dir0, dInt32* const vertexIndex) const
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

dFloat32 ntShapeBox::RayCast(ntRayCastCallback& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ntBody* const body, ntContactPoint& contactOut) const
{
	dAssert(localP0.m_w == dFloat32(0.0f));
	dAssert(localP1.m_w == dFloat32(0.0f));

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
		contactOut.m_normal = dVector(dFloat32(0.0f));
		contactOut.m_normal[index] = signDir;
		//contactOut.m_userId = SetUserDataID();
	}
	else 
	{
		tmin = dFloat32(1.2f);
	}
	return tmin;
}
