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

	dgSimd tmp;
	dgSimd::Transpose4x4 (m_vertex_sse[0], m_vertex_sse[1], m_vertex_sse[2], tmp, 
						 (dgSimd&)m_vertex[0], (dgSimd&)m_vertex[1], (dgSimd&)m_vertex[2], (dgSimd&)m_vertex[3]);
	dgSimd::Transpose4x4 (m_vertex_sse[3], m_vertex_sse[4], m_vertex_sse[5], tmp, 
						 (dgSimd&)m_vertex[4], (dgSimd&)m_vertex[5], (dgSimd&)m_vertex[6], (dgSimd&)m_vertex[7]);

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

#if 0
	dgFloatSign* const ptr =  (dgFloatSign*) &dir; 

	dgInt32 x = -(ptr[0].m_integer.m_iVal >> 31);
	dgInt32 y = -(ptr[1].m_integer.m_iVal >> 31);
	dgInt32 z = -(ptr[2].m_integer.m_iVal >> 31);
	return dgVector (m_size[x].m_x, m_size[y].m_y, m_size[z].m_z, dgFloat32 (0.0f));

#else
	// according to Intel doc, with latest possessors this is better, because read after write are very, very expensive
	return dgVector (dir.m_x < dgFloat32 (0.0f) ? m_size[1].m_x : m_size[0].m_x, 
					 dir.m_y < dgFloat32 (0.0f) ? m_size[1].m_y : m_size[0].m_y, 
					 dir.m_z < dgFloat32 (0.0f) ? m_size[1].m_z : m_size[0].m_z, dgFloat32 (0.0f));
#endif

}


dgVector dgCollisionBox::SupportVertexSimd (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgSimd test ((dgSimd&)dir > dgSimd(dgFloat32 (0.0f)));
	return ((dgSimd&)m_size[0] & test) | ((dgSimd&)m_size[1]).AndNot(test); 
}


void dgCollisionBox::CalcAABB (const dgMatrix &matrix, dgVector &p0, dgVector &p1) const
{
	dgFloat32 x = m_size[0].m_x * dgAbsf(matrix[0][0]) + m_size[0].m_y * dgAbsf(matrix[1][0]) + m_size[0].m_z * dgAbsf(matrix[2][0]);  
	dgFloat32 y = m_size[0].m_x * dgAbsf(matrix[0][1]) + m_size[0].m_y * dgAbsf(matrix[1][1]) + m_size[0].m_z * dgAbsf(matrix[2][1]);  
	dgFloat32 z = m_size[0].m_x * dgAbsf(matrix[0][2]) + m_size[0].m_y * dgAbsf(matrix[1][2]) + m_size[0].m_z * dgAbsf(matrix[2][2]);  

	p0.m_x = matrix[3][0] - x;
	p1.m_x = matrix[3][0] + x;

	p0.m_y = matrix[3][1] - y;
	p1.m_y = matrix[3][1] + y;

	p0.m_z = matrix[3][2] - z;
	p1.m_z = matrix[3][2] + z;

	p0.m_w = dgFloat32 (0.0f);
	p1.m_w = dgFloat32 (0.0f);
}


void dgCollisionBox::CalcAABBSimd (const dgMatrix &matrix, dgVector &p0, dgVector &p1) const
{
	dgSimd size (((dgSimd&)matrix[0] & dgSimd::m_signMask) * (dgSimd&) m_size_x + 
				   ((dgSimd&)matrix[1] & dgSimd::m_signMask) * (dgSimd&) m_size_y + 
				   ((dgSimd&)matrix[2] & dgSimd::m_signMask) * (dgSimd&) m_size_z); 

	(dgSimd&)p0 = ((dgSimd&)matrix[3] - size) & dgSimd::m_triplexMask;
	(dgSimd&)p1 = ((dgSimd&)matrix[3] + size) & dgSimd::m_triplexMask;
}


dgFloat32 dgCollisionBox::RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
{
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

dgFloat32 dgCollisionBox::RayCastSimd (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
{
	return RayCast (localP0, localP1, contactOut, body, userData);
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
		test[i] = plane.Evalue (m_vertex[i]);
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
				contactsOut[count] = m_vertex[ptr->m_vertex] - dp.Scale (t);
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


dgInt32 dgCollisionBox::CalculatePlaneIntersectionSimd (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const
{
	dgAssert (0);
	return CalculatePlaneIntersection (normal, point, contactsOut);
/*
	dgInt32 count = 0;
	dgSimd plane (((dgSimd&)normal & dgSimd::m_triplexMask));
	plane = plane - plane.DotProduct((dgSimd&)point).AndNot(dgSimd::m_triplexMask);

	dgSimd plane_a;
	dgSimd plane_b;
	dgSimd plane_c;
	dgSimd plane_d;
	dgSimd side[2];

	dgSimd::Transpose4x4 (plane_a, plane_b, plane_c, plane_d, plane, plane, plane, plane);
	
	side[0] = plane_a * m_vertex_sse[0] + plane_b * m_vertex_sse[1] + plane_c * m_vertex_sse[2] + plane_d;
	side[1] = plane_a * m_vertex_sse[3] + plane_b * m_vertex_sse[4] + plane_c * m_vertex_sse[5] + plane_d;
	
	dgSimd zero (dgFloat32 (0.0f));
	dgSimd huge (dgFloat32 (1.0e20f));
	dgSimd negOne (dgFloat32 (-1.0f));
	
	dgSimd sideTest (side[0] > zero);
	dgSimd positiveSide0 ((side[0] & sideTest) | huge.AndNot(sideTest));
	dgSimd index0 ((dgSimd::m_index_0123 & sideTest) | negOne.AndNot(sideTest));

	sideTest = side[1] > zero;
	dgSimd positiveSide1 ((side[1] & sideTest) | huge.AndNot(sideTest));
	dgSimd index1 ((dgSimd::m_index_4567 & sideTest) | negOne.AndNot(sideTest));

	sideTest = positiveSide0 < positiveSide1;
	positiveSide0 = positiveSide0.GetMin(positiveSide1);
	index0 = (index0 & sideTest) | index1.AndNot(sideTest);

	positiveSide1 = positiveSide0.MoveHigh(positiveSide0);
	sideTest = positiveSide0 < positiveSide1;
	positiveSide0 = positiveSide0.GetMin(positiveSide1);
	index0 = (index0 & sideTest) | index0.MoveHigh(index0).AndNot(sideTest);

	positiveSide1 = positiveSide0.PackLow(positiveSide0);
	sideTest = positiveSide0 < positiveSide1.MoveHigh(positiveSide1);
	index1 = index0.PackLow(index0);
	dgInt32 index = ((index0 & sideTest) | (index1.MoveHigh(index1).AndNot(sideTest))).GetInt();

	if (index >= 0) {
		dgConvexSimplexEdge* edge = m_supportVertexStarQuadrant[index];

		const dgFloat32* const test = (dgFloat32*) &side[0];
		dgAssert (test[edge->m_vertex] > dgFloat32 (0.0f));

		dgConvexSimplexEdge* ptr = edge;
		dgConvexSimplexEdge* firstEdge = NULL;

		dgSimd side0 (test[edge->m_vertex]);
		do {
			dgAssert (m_vertex[ptr->m_twin->m_vertex].m_w == dgFloat32 (1.0f));
			dgSimd side1 (test[ptr->m_twin->m_vertex]);
			if ((side1 < side0).GetSignMask()) {
				if ((side1 < zero).GetSignMask()) {
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
			const dgSimd* const vertexArray = (dgSimd*) m_vertex ;
			do {

				dgInt32 index0 = ptr->m_vertex;
				dgInt32 index1 = ptr->m_twin->m_vertex;
				dgAssert (m_vertex[index0].m_w == dgFloat32 (1.0f));
				dgAssert (m_vertex[index1].m_w == dgFloat32 (1.0f));

				//dgSimd p1p0 (dgSimd&)m_vertex[index1] - (dgSimd&)m_vertex[index0]);
				dgSimd p1p0 (vertexArray[index1] - vertexArray[index0]);
				
				dgSimd dot (plane.DotProduct(p1p0));
				
				// GCC do no like dgSimd den (dgSimd(test[index0]) / dot);
				dgFloat32 a = test[index0];
				dgSimd den (dgSimd(a) / dot);
				den = zero.GetMin (negOne.GetMax (den));

				//dgAssert (den.m_type.m128_f32[0] <= dgFloat32 (0.0f));
				//dgAssert (den.m_type.m128_f32[0] >= dgFloat32 (-1.0f));
				//(dgSimd&)contactsOut[count] = ((dgSimd&)m_vertex[index0] & dgSimd::m_triplexMask) - p1p0 * den;
				(dgSimd&)contactsOut[count] = (vertexArray[index0] & dgSimd::m_triplexMask) - p1p0 * den;
				count ++;

				dgConvexSimplexEdge* ptr1 = ptr->m_next;
				for (; ptr1 != ptr; ptr1 = ptr1->m_next) {
					dgInt32 index0 = ptr1->m_twin->m_vertex;
					if ((dgSimd (test[index0]) >= zero).GetSignMask()) {
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
*/
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
	dgVector size (m_size[0].Scale (dgFloat32 (2.0f)));
	callback (userData, &size, sizeof (dgVector));
}

/*
dgInt32 dgWorld::CalculateBoxToBoxContacts (dgBody* box1, dgBody* box2, dgContactPoint* const contactOut) const
{
	dgAssert (0);
	return 0;

	dgInt32 i;
	dgInt32 k;
	dgInt32 count1;
	dgInt32 count2;
	dgFloat32 d1;
	dgFloat32 d2;
	dgFloat32 min;
	dgFloat32 dist;
	dgFloat32 test;
	dgFloat32 minDist;
	dgPlane plane; 
	dgVector shape1[16];
	dgVector shape2[16];
	dgCollisionBox* collision1;
	dgCollisionBox* collision2;

	dgAssert (box1->m_collision->IsType (m_boxType));
	dgAssert (box2->m_collision->IsType (m_boxType));

	const dgMatrix& matrix1 = box1->m_collisionWorldMatrix;
	const dgMatrix& matrix2 = box2->m_collisionWorldMatrix;

	collision1 = (dgCollisionBox*) box1->m_collision;
	collision2 = (dgCollisionBox*) box2->m_collision;

	const dgVector& size1 = collision1->m_size;
	const dgVector& size2 = collision2->m_size;

	minDist = dgFloat32 (-1.0e10f);

	dgMatrix mat12 (matrix1 * matrix2.Inverse ());
	for (i = 0; i < 3; i ++) {
		min = dgAbsf (mat12[0][i]) * size1[0] + dgAbsf (mat12[1][i]) * size1[1] + dgAbsf (mat12[2][i]) * size1[2];
		dist = dgAbsf (mat12[3][i]) - size2[i] - min;
		if (dist > (-DG_RESTING_CONTACT_PENETRATION)) {
			return 0;
		}
		if (dist > minDist) {
			minDist = dist;
			plane[0] = dgFloat32 (0.0f);
			plane[1] = dgFloat32 (0.0f);
			plane[2] = dgFloat32 (0.0f);
			plane[3] = - (size2[i] + dist * dgFloat32 (0.5f));

			plane[i] = dgFloat32 (1.0f);
			test = plane[3] + mat12[3][i] + min; 
			if (test < dgFloat32 (0.0f)) {
				plane[i] = dgFloat32 (-1.0f);
			}
			plane = matrix2.TransformPlane (plane);
		}
	}

	//	dgMatrix mat21 (matrix2 * matrix1.Inverse ());
	dgMatrix mat21 (mat12.Inverse ());
	for (i = 0; i < 3; i ++) {
		min = dgAbsf (mat21[0][i]) * size2[0] + dgAbsf (mat21[1][i]) * size2[1] + dgAbsf (mat21[2][i]) * size2[2];
		dist = dgAbsf (mat21[3][i]) - size1[i] - min;
		if (dist > (-DG_RESTING_CONTACT_PENETRATION)) {
			return 0;
		}
		if (dist > minDist) {
			minDist = dist;
			plane[0] = dgFloat32 (0.0f);
			plane[1] = dgFloat32 (0.0f);
			plane[2] = dgFloat32 (0.0f);
			plane[3] = - (size1[i] + dist * dgFloat32 (0.5f));

			plane[i] = dgFloat32 (1.0f);

			test = plane[3] + mat21[3][i] + min; 
			if (test < dgFloat32 (0.0f)) {
				plane[i] = dgFloat32 (-1.0f);
			}
			plane = matrix1.TransformPlane (plane).Scale (dgFloat32 (-1.0f));
		}
	}

	for (k = 0; k < 3; k ++) {
		for (i = 0; i < 3; i ++) { 
			dgVector normal (matrix1[k] * matrix2[i]);
			test = (normal % normal) ;
			if (test > dgFloat32(1.0e-6f)) {
				normal = normal.Scale (dgRsqrt (test));
				d2 = size2[0] * dgAbsf (matrix2[0] % normal) + size2[1] * dgAbsf (matrix2[1] % normal) + size2[2] * dgAbsf (matrix2[2] % normal);
				d1 = size1[0] * dgAbsf (matrix1[0] % normal) + size1[1] * dgAbsf (matrix1[1] % normal) + size1[2] * dgAbsf (matrix1[2] % normal);

				dgVector q (matrix2[3] - normal.Scale (d2));
				dgVector p (matrix1[3] + normal.Scale (d1));
				dist = (q - p) % normal;
				if (dist > (-DG_RESTING_CONTACT_PENETRATION)) {
					return 0;
				}

				dgVector q1 (matrix2[3] + normal.Scale (d2));
				dgVector p1 (matrix1[3] - normal.Scale (d1));
				test = (p1 - q1) % normal;
				if (test > (-DG_RESTING_CONTACT_PENETRATION)) {
					return 0;
				}

				if (test > dist) {
					dist = test;
					p = p1;
					q = q1;
				}

				if (dist > minDist) {
					minDist = dist;
					plane[0] = normal[0];
					plane[1] = normal[1];
					plane[2] = normal[2];
					plane[3] = - dgFloat32 (0.5f) * ((q + p) % normal);

					test = plane.Evalue (matrix1[3]); 
					if (test < dgFloat32 (0.0f)) {
						plane.m_x *= dgFloat32 (-1.0f);
						plane.m_y *= dgFloat32 (-1.0f);
						plane.m_z *= dgFloat32 (-1.0f);
						plane.m_w *= dgFloat32 (-1.0f);
					}
				}
			}
		}
	}

	dgPlane plane1 (matrix1.UntransformPlane (plane));	
	count1 = collision1->CalculatePlaneIntersection (plane1, shape1);
	if (!count1) {
		dgVector p1 (collision1->SupportVertex (plane1.Scale (dgFloat32 (-1.0f))));
		dgPlane plane (plane1, - (plane1 % p1) - DG_ROBUST_PLANE_CLIP);
		count1 = collision1->CalculatePlaneIntersection (plane, shape1);
		if (count1) {
			dgVector err (plane1.Scale (plane1.Evalue (shape1[0])));
			for (i = 0; i < count1; i ++) {
				shape1[i] -= err;
			}
		}
	}
	if (count1 == 0) {
		return 0;
	}

	dgPlane plane2 (matrix2.UntransformPlane (plane));	
	count2 = collision2->CalculatePlaneIntersection (plane2, shape2);
	if (!count2) {
		dgVector p2 (collision2->SupportVertex (plane2));
		dgPlane plane (plane2, DG_ROBUST_PLANE_CLIP - (plane2 % p2));
		count2 = collision2->CalculatePlaneIntersection (plane, shape2);
		if (count2) {
			dgVector err (plane2.Scale (plane2.Evalue (shape2[0])));
			for (i = 0; i < count2; i ++) {
				shape2[i] -= err;
			}
		}
	}

	if (count2 == 0) {
		return 0;
	}

	dgAssert (count1 <= 6);
	dgAssert (count2 <= 6);
	matrix1.TransformTriplex (shape1, sizeof (dgVector), shape1, sizeof (dgVector), count1);
	matrix2.TransformTriplex (shape2, sizeof (dgVector), shape2, sizeof (dgVector), count2);

	minDist = (dgAbsf (minDist) - DG_IMPULSIVE_CONTACT_PENETRATION);
	if (minDist < dgFloat32 (0.0f)) {
		minDist = dgFloat32 (0.0f);
	}
	k = dgContactSolver::CalculateConvexShapeIntersection (plane, 0, minDist, count1, shape1, count2, shape2, contactOut);
	return k;
}
*/


