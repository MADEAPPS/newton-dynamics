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
#include "dgCollisionBox.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgConvexSimplexEdge dgCollisionBox::m_edgeArray[24] = 
{
	{&m_edgeArray[3], &m_edgeArray[12], &m_edgeArray[5], 1}, 
	{&m_edgeArray[6], &m_edgeArray[3], &m_edgeArray[7], 2}, 
	{&m_edgeArray[12], &m_edgeArray[6], &m_edgeArray[14], 7}, 
	{&m_edgeArray[0], &m_edgeArray[9], &m_edgeArray[1], 0}, 
	{&m_edgeArray[9], &m_edgeArray[15], &m_edgeArray[11], 3}, 
	{&m_edgeArray[15], &m_edgeArray[0], &m_edgeArray[16], 6}, 
	{&m_edgeArray[1], &m_edgeArray[18], &m_edgeArray[2], 0}, 
	{&m_edgeArray[10], &m_edgeArray[1], &m_edgeArray[9], 3}, 
	{&m_edgeArray[18], &m_edgeArray[10], &m_edgeArray[20], 5}, 
	{&m_edgeArray[4], &m_edgeArray[7], &m_edgeArray[3], 1}, 
	{&m_edgeArray[7], &m_edgeArray[21], &m_edgeArray[8], 2}, 
	{&m_edgeArray[21], &m_edgeArray[4], &m_edgeArray[22], 4}, 
	{&m_edgeArray[2], &m_edgeArray[16], &m_edgeArray[0], 0}, 
	{&m_edgeArray[16], &m_edgeArray[19], &m_edgeArray[17], 6}, 
	{&m_edgeArray[19], &m_edgeArray[2], &m_edgeArray[18], 5}, 
	{&m_edgeArray[5], &m_edgeArray[22], &m_edgeArray[4], 1}, 
	{&m_edgeArray[13], &m_edgeArray[5], &m_edgeArray[12], 7}, 
	{&m_edgeArray[22], &m_edgeArray[13], &m_edgeArray[23], 4}, 
	{&m_edgeArray[8], &m_edgeArray[14], &m_edgeArray[6], 2}, 
	{&m_edgeArray[14], &m_edgeArray[23], &m_edgeArray[13], 7}, 
	{&m_edgeArray[23], &m_edgeArray[8], &m_edgeArray[21], 4}, 
	{&m_edgeArray[11], &m_edgeArray[20], &m_edgeArray[10], 3}, 
	{&m_edgeArray[17], &m_edgeArray[11], &m_edgeArray[15], 6}, 
	{&m_edgeArray[20], &m_edgeArray[17], &m_edgeArray[19], 5}, 
};

dgConvexSimplexEdge* dgCollisionBox::m_vertexToEdgeMap[8] = {&m_edgeArray[3], &m_edgeArray[0], &m_edgeArray[1], &m_edgeArray[4], &m_edgeArray[11], &m_edgeArray[8], &m_edgeArray[5], &m_edgeArray[2]};

dgCollisionBox::dgCollisionBox(dgMemoryAllocator* allocator, dgUnsigned32 signature, dgFloat32 size_x, dgFloat32 size_y, dgFloat32 size_z)
	:dgCollisionConvex(allocator, signature, m_boxCollision)
{
	Init (size_x, size_y, size_z);
}

dgCollisionBox::dgCollisionBox(dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData)
{
	dgVector size;
	deserialization (userData, &size, sizeof (dgVector));
	Init (size.m_x, size.m_y, size.m_z);
}

void dgCollisionBox::Init (dgFloat32 size_x, dgFloat32 size_y, dgFloat32 size_z)
{
	m_rtti |= dgCollisionBox_RTTI;
	m_size[0].m_x = dgAbsf (size_x) * dgFloat32 (0.5f);
	m_size[0].m_y = dgAbsf (size_y) * dgFloat32 (0.5f);
	m_size[0].m_z = dgAbsf (size_z) * dgFloat32 (0.5f);
	m_size[0].m_w = dgFloat32 (0.0f);

	m_size[1].m_x = - m_size[0].m_x;
	m_size[1].m_y = - m_size[0].m_y;
	m_size[1].m_z = - m_size[0].m_z;
	m_size[1].m_w = dgFloat32 (0.0f);

	m_edgeCount = 24;
	m_vertexCount = 8;

	m_vertex[0]	= dgVector ( m_size[0].m_x,  m_size[0].m_y,  m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[1]	= dgVector (-m_size[0].m_x,  m_size[0].m_y,  m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[2]	= dgVector ( m_size[0].m_x, -m_size[0].m_y,  m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[3]	= dgVector (-m_size[0].m_x, -m_size[0].m_y,  m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[4]	= dgVector (-m_size[0].m_x, -m_size[0].m_y, -m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[5]	= dgVector ( m_size[0].m_x, -m_size[0].m_y, -m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[6]	= dgVector (-m_size[0].m_x,  m_size[0].m_y, -m_size[0].m_z, dgFloat32 (1.0f));
	m_vertex[7]	= dgVector ( m_size[0].m_x,  m_size[0].m_y, -m_size[0].m_z, dgFloat32 (1.0f));

	dgCollisionConvex::m_vertex = m_vertex;
	dgCollisionConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG ();
}

dgCollisionBox::~dgCollisionBox()
{
	dgCollisionConvex::m_simplex = NULL;
	dgCollisionConvex::m_vertex = NULL;
}


void dgCollisionBox::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}

dgInt32 dgCollisionBox::CalculateSignature (dgFloat32 dx, dgFloat32 dy, dgFloat32 dz)
{
	dgUnsigned32 buffer[4];

	dx = dgAbsf (dx);
	dy = dgAbsf (dy);
	dz = dgAbsf (dz);
	buffer[0] = m_boxCollision;
	buffer[1] = Quantize (dx * dgFloat32 (0.5f));
	buffer[2] = Quantize (dy * dgFloat32 (0.5f));
	buffer[3] = Quantize (dz * dgFloat32 (0.5f));
	return Quantize(buffer, sizeof (buffer));
}


dgInt32 dgCollisionBox::CalculateSignature () const
{
	return CalculateSignature(m_size[0].m_x, m_size[0].m_y, m_size[0].m_z);
}


dgVector dgCollisionBox::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	// according to Intel doc, with latest possessors this is better, because read after write are very, very expensive
	dgVector mask (dir < dgVector (dgFloat32 (0.0f)));
//	return dgVector (dir.m_x < dgFloat32 (0.0f) ? m_size[1].m_x : m_size[0].m_x, 
//					 dir.m_y < dgFloat32 (0.0f) ? m_size[1].m_y : m_size[0].m_y, 
//					 dir.m_z < dgFloat32 (0.0f) ? m_size[1].m_z : m_size[0].m_z, dgFloat32 (0.0f));
	return (m_size[1] & mask) + m_size[0].AndNot(mask);
}



void dgCollisionBox::CalcAABB (const dgMatrix &matrix, dgVector &p0, dgVector &p1) const
{
//	dgFloat32 x = m_size[0].m_x * dgAbsf(matrix[0][0]) + m_size[0].m_y * dgAbsf(matrix[1][0]) + m_size[0].m_z * dgAbsf(matrix[2][0]);  
//	dgFloat32 y = m_size[0].m_x * dgAbsf(matrix[0][1]) + m_size[0].m_y * dgAbsf(matrix[1][1]) + m_size[0].m_z * dgAbsf(matrix[2][1]);  
//	dgFloat32 z = m_size[0].m_x * dgAbsf(matrix[0][2]) + m_size[0].m_y * dgAbsf(matrix[1][2]) + m_size[0].m_z * dgAbsf(matrix[2][2]);  

	dgVector size (matrix[0].Abs().CompProduct4(dgVector(m_size[0].m_x)) + matrix[1].Abs().CompProduct4(dgVector(m_size[0].m_y)) + matrix[2].Abs().CompProduct4(dgVector(m_size[0].m_z)));

	p0 = (matrix[3] - size) & dgVector::m_triplexMask;
	p1 = (matrix[3] + size) & dgVector::m_triplexMask;
}



dgFloat32 dgCollisionBox::RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
{
	dgAssert (localP0.m_w == dgFloat32 (0.0f));
	dgAssert (localP1.m_w == dgFloat32 (0.0f));

	dgInt32 index = 0;
	dgFloat32 signDir = dgFloat32 (0.0f);
	dgFloat32 tmin = dgFloat32 (0.0f);
	dgFloat32 tmax = dgFloat32 (1.0f);
	for (dgInt32 i = 0; i < 3; i++) {
		dgFloat32 dp = localP1[i] - localP0[i];
		if (dgAbsf (dp) < dgFloat32 (1.0e-8f)) {
			if (localP0[i] <= m_size[1][i] || localP0[i] >= m_size[0][i]) {
				return dgFloat32 (1.2f);
			}
		} else {
			dp = dgFloat32 (1.0f) / dp; 
			dgFloat32 t1 = (m_size[1][i] - localP0[i]) * dp;
			dgFloat32 t2 = (m_size[0][i] - localP0[i]) * dp;

			dgFloat32 sign = dgFloat32 (-1.0f);
			if (t1 > t2) {
				sign = 1;
				dgSwap(t1, t2);
			}
			if (t1 > tmin) {
				signDir = sign;
				index = i;
				tmin = t1;
			}
			if (t2 < tmax) {
				tmax = t2;
			}
			if (tmin > tmax) {
				return dgFloat32 (1.2f);
			}
		}
	}



	if (tmin > dgFloat32 (0.0f)) {
		dgAssert (tmin < 1.0f);
		contactOut.m_normal = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		contactOut.m_normal[index] = signDir;
		//contactOut.m_userId = SetUserDataID();
	} else {
		tmin = dgFloat32 (1.2f);
	}
	return tmin;

}


void dgCollisionBox::MassProperties ()
{
	m_centerOfMass = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	m_crossInertia = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgFloat32 volume = dgFloat32 (8.0f) * m_size[0].m_x * m_size[0].m_y * m_size[0].m_z; 
	m_inertia = dgVector (dgFloat32 (1.0f / 3.0f) * (m_size[0].m_y * m_size[0].m_y + m_size[0].m_z * m_size[0].m_z),
						  dgFloat32 (1.0f / 3.0f) * (m_size[0].m_x * m_size[0].m_x + m_size[0].m_z * m_size[0].m_z),
						  dgFloat32 (1.0f / 3.0f) * (m_size[0].m_x * m_size[0].m_x + m_size[0].m_y * m_size[0].m_y),
						  dgFloat32 (0.0f));
	m_centerOfMass.m_w = volume;
}


dgInt32 dgCollisionBox::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const
{
	dgFloat32 test[8];
	dgPlane plane (normal, - (normal % point));
	for (dgInt32 i = 0; i < 8; i ++) {
		//test[i] = plane.Evalue (m_vertex[i]);
		test[i] = plane.DotProduct4 (m_vertex[i]).m_x;
	}


	dgConvexSimplexEdge* edge = NULL;
	for (dgInt32 i = 0; i < 8; i ++) {
		dgConvexSimplexEdge* const ptr = m_vertexToEdgeMap[i];
		dgFloat32 side0 = test[ptr->m_vertex];
		dgFloat32 side1 = test[ptr->m_twin->m_vertex];
		if ((side0 * side1) < dgFloat32 (0.0f)) {
			edge = ptr;
			break;
		}
	}

	if (!edge) {
		for (dgInt32 i = 0; i < 8; i ++) {
			dgConvexSimplexEdge* const ptr = m_vertexToEdgeMap[i];
			dgFloat32 side0 = test[ptr->m_vertex];
			dgFloat32 side1 = test[ptr->m_twin->m_vertex];
			if ((side0 * side1) <= dgFloat32 (0.0f)) {
				edge = ptr;
				break;
			}
		}
	}


	dgInt32 count = 0;
	if (edge) {
		if (test[edge->m_vertex] < dgFloat32 (0.0f)) {
			edge = edge->m_twin;
		}
		dgAssert (test[edge->m_vertex] > dgFloat32 (0.0f));

		dgConvexSimplexEdge* ptr = edge;
		dgConvexSimplexEdge* firstEdge = NULL;
		dgFloat32 side0 = test[edge->m_vertex];
		do {
			dgAssert (m_vertex[ptr->m_twin->m_vertex].m_w == dgFloat32 (1.0f));
			dgFloat32 side1 = test[ptr->m_twin->m_vertex];
			if (side1 < side0) {
				if (side1 < dgFloat32 (0.0f)) {
					firstEdge = ptr;
					break;
				}

				side0 = side1;
				edge = ptr->m_twin;
				ptr = edge;
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);

		if (firstEdge) {
			edge = firstEdge;
			ptr = edge;
			do {
				dgVector dp (m_vertex[ptr->m_twin->m_vertex] - m_vertex[ptr->m_vertex]);

				dgFloat32 t = plane % dp;
				if (t >= dgFloat32 (-1.e-24f)) {
					t = dgFloat32 (0.0f);
				} else {
					t = test[ptr->m_vertex] / t;
					if (t > dgFloat32 (0.0f)) {
						t = dgFloat32 (0.0f);
					}
					if (t < dgFloat32 (-1.0f)) {
						t = dgFloat32 (-1.0f);
					}
				}

				dgAssert (t <= dgFloat32 (0.01f));
				dgAssert (t >= dgFloat32 (-1.05f));
				contactsOut[count] = m_vertex[ptr->m_vertex] - dp.Scale3 (t);
				count ++;

				dgConvexSimplexEdge* ptr1 = ptr->m_next;
				for (; ptr1 != ptr; ptr1 = ptr1->m_next) {
					dgInt32 index0 = ptr1->m_twin->m_vertex;
					if (test[index0] >= dgFloat32 (0.0f)) {
						dgAssert (test[ptr1->m_vertex] <= dgFloat32 (0.0f));
						break;
					}
				}
				dgAssert (ptr != ptr1);
				ptr = ptr1->m_twin;

			} while ((ptr != edge) && (count < 8));
		}
	}

	if (count > 1) {
		count = RectifyConvexSlice (count, normal, contactsOut);
	}
	return count;
}

void dgCollisionBox::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_box.m_x = m_size[0].m_x * dgFloat32 (2.0f);
	info->m_box.m_y = m_size[0].m_y * dgFloat32 (2.0f);
	info->m_box.m_z = m_size[0].m_z * dgFloat32 (2.0f);
}

void dgCollisionBox::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	dgVector size (m_size[0].Scale3 (dgFloat32 (2.0f)));
	callback (userData, &size, sizeof (dgVector));
}

