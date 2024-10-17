/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
#include "ndContactSolver.h"
#include "ndShapeChamferCylinder.h"

ndInt32 ndShapeChamferCylinder::m_shapeRefCount = 0;
ndVector ndShapeChamferCylinder::m_yzMask (0, 0xffffffff, 0xffffffff, 0);
ndVector ndShapeChamferCylinder::m_shapesDirs[DG_MAX_CHAMFERCYLINDER_DIR_COUNT];
ndShapeConvex::ndConvexSimplexEdge ndShapeChamferCylinder::m_edgeArray[(4 * DG_CHAMFERCYLINDER_SLICES + 2)* DG_CHAMFERCYLINDER_BRAKES];

ndShapeChamferCylinder::ndShapeChamferCylinder(ndFloat32 radius, ndFloat32 height)
	:ndShapeConvex(m_chamferCylinder)
{
	Init (radius, height);
}

ndShapeChamferCylinder::~ndShapeChamferCylinder()
{
	m_shapeRefCount --;
	ndAssert (m_shapeRefCount >= 0);

	ndShapeConvex::m_simplex = nullptr;
	ndShapeConvex::m_vertex = nullptr;
}

void ndShapeChamferCylinder::Init (ndFloat32 radius, ndFloat32 height)
{
	m_radius = ndMax (ndAbs (radius), D_MIN_CONVEX_SHAPE_SIZE);
	m_height = ndMax (ndAbs (height * ndFloat32 (0.5f)), D_MIN_CONVEX_SHAPE_SIZE);

	ndFloat32 sliceAngle = ndFloat32 (0.0f);
	ndFloat32 sliceStep = ndPi  / DG_CHAMFERCYLINDER_SLICES; 
	ndFloat32 breakStep = ndFloat32 (2.0f) * ndPi / DG_CHAMFERCYLINDER_BRAKES;

	ndMatrix rot (ndPitchMatrix (breakStep));	
	ndInt32 index = 0;
	for (ndInt32 j = 0; j <= DG_CHAMFERCYLINDER_SLICES; ++j) 
	{
		ndVector p0 (-m_height * ndCos(sliceAngle), ndFloat32 (0.0f), m_radius + m_height * ndSin(sliceAngle), ndFloat32 (0.0f));
		sliceAngle += sliceStep;
		for (ndInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; ++i) 
		{
			m_vertex[index] = p0;
			index ++;
			p0 = rot.UnrotateVector (p0);
		}
	}

	m_edgeCount = (4 * DG_CHAMFERCYLINDER_SLICES + 2)* DG_CHAMFERCYLINDER_BRAKES;
	m_vertexCount = DG_CHAMFERCYLINDER_BRAKES * (DG_CHAMFERCYLINDER_SLICES + 1);
	ndShapeConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) 
	{
		ndPolyhedra polyhedra;
		ndInt32 wireframe[DG_CHAMFERCYLINDER_SLICES + 10];

		ndVector locus(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		for (ndInt32 i = 0; i < DG_MAX_CHAMFERCYLINDER_DIR_COUNT; ++i) 
		{
			ndMatrix matrix (ndPitchMatrix (ndFloat32 (2.0f) * ndPi * ndFloat32 (i) / DG_MAX_CHAMFERCYLINDER_DIR_COUNT));
			m_shapesDirs[i] = matrix.RotateVector (locus);
		}

		ndInt32 index0 = 0;
		for (ndInt32 j = 0; j < DG_CHAMFERCYLINDER_SLICES; ++j) 
		{
			ndInt32 index1 = index0 + DG_CHAMFERCYLINDER_BRAKES - 1;
			for (ndInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; ++i) 
			{
				wireframe[0] = index0;
				wireframe[1] = index1;
				wireframe[2] = index1 + DG_CHAMFERCYLINDER_BRAKES;
				wireframe[3] = index0 + DG_CHAMFERCYLINDER_BRAKES;

				index1 = index0;
				index0 ++;
				polyhedra.AddFace (4, wireframe);
			}
		}

		for (ndInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; ++i) 
		{ 
			wireframe[i] = i;
		}
		polyhedra.AddFace (DG_CHAMFERCYLINDER_BRAKES, wireframe);

		for (ndInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; ++i) 
		{ 
			wireframe[i] = DG_CHAMFERCYLINDER_BRAKES * (DG_CHAMFERCYLINDER_SLICES + 1) - i - 1;
		}
		polyhedra.AddFace (DG_CHAMFERCYLINDER_BRAKES, wireframe);
		polyhedra.EndFace ();

		ndAssert (SanityCheck (polyhedra));

		ndUnsigned64 i = 0;
		ndPolyhedra::Iterator iter (polyhedra);
		for (iter.Begin(); iter; iter ++) 
		{
			ndEdge* const edge = &(*iter);
			edge->m_userData = i;
			i ++;
		}

		for (iter.Begin(); iter; iter ++) 
		{
			ndEdge* const edge = &(*iter);

			ndConvexSimplexEdge* const ptr = &m_edgeArray[edge->m_userData];
			ptr->m_vertex = edge->m_incidentVertex;
			ptr->m_next = &m_edgeArray[edge->m_next->m_userData];
			ptr->m_prev = &m_edgeArray[edge->m_prev->m_userData];
			ptr->m_twin = &m_edgeArray[edge->m_twin->m_userData];
		}
	}

	m_shapeRefCount ++;
	ndShapeConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG ();
}

ndShapeInfo ndShapeChamferCylinder::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());

	info.m_chamferCylinder.m_r = m_radius;
	info.m_chamferCylinder.m_height = m_height * ndFloat32(2.0f);
	return info;
}

void ndShapeChamferCylinder::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	ndShapeConvex::CalculateAabb(matrix, p0, p1);

//	ndAssert(0);
	//ndVector size0(m_radius0);
	//ndVector size1(m_radius1);
	//ndVector q0(matrix.m_posit - matrix.m_front.Scale(m_height));
	//ndVector q1(matrix.m_posit + matrix.m_front.Scale(m_height));
	//
	//ndVector min_q0(q0 - size0);
	//ndVector min_q1(q1 - size1);
	//
	//ndVector max_q0(q0 + size1);
	//ndVector max_q1(q1 + size1);
	//
	//p0 = min_q0.GetMin(min_q1) & ndVector::m_triplexMask;
	//p1 = max_q0.GetMax(max_q1) & ndVector::m_triplexMask;
}

void ndShapeChamferCylinder::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndInt32 slices = 12;
	ndInt32 brakes = 24;
	ndFloat32 sliceAngle = ndFloat32(0.0f);
	ndFloat32 sliceStep = ndPi / (ndFloat32)slices;
	ndFloat32 breakStep = ndFloat32(2.0f) * ndPi / (ndFloat32)brakes;
	ndVector pool[24 * (12 + 1)];

	ndMatrix rot(ndPitchMatrix(breakStep));
	ndInt32 index = 0;
	for (ndInt32 j = 0; j <= slices; ++j)
	{
		ndVector p0(-m_height * ndCos(sliceAngle), ndFloat32(0.0f), m_radius + m_height * ndSin(sliceAngle), ndFloat32(0.0f));
		sliceAngle += sliceStep;
		for (ndInt32 i = 0; i < brakes; ++i)
		{
			pool[index] = p0;
			p0 = rot.UnrotateVector(p0);
			index++;
		}
	}

	matrix.TransformTriplex(&pool[0].m_x, sizeof(ndVector), &pool[0].m_x, sizeof(ndVector), 24 * (12 + 1));

	ndVector face[32];
	ndShapeDebugNotify::ndEdgeType edgeType[32];
	memset(edgeType, ndShapeDebugNotify::m_shared, sizeof(edgeType));

	index = 0;
	for (ndInt32 j = 0; j < slices; ++j)
	{
		ndInt32 index0 = index + brakes - 1;
		for (ndInt32 i = 0; i < brakes; ++i)
		{
			face[0] = pool[index];
			face[1] = pool[index0];
			face[2] = pool[index0 + brakes];
			face[3] = pool[index + brakes];
			index0 = index;
			index++;
			debugCallback.DrawPolygon(4, face, edgeType);
		}
	}

	for (ndInt32 i = 0; i < brakes; ++i)
	{
		face[i] = pool[i];
	}
	debugCallback.DrawPolygon(24, face, edgeType);

	for (ndInt32 i = 0; i < brakes; ++i)
	{
		face[i] = pool[brakes * (slices + 1) - i - 1];
	}
	debugCallback.DrawPolygon(24, face, edgeType);
}

ndVector ndShapeChamferCylinder::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const
{
	ndAssert(dir.m_w == 0.0f);
	return point + dir.Scale(m_height - D_PENETRATION_TOL);
}

ndVector ndShapeChamferCylinder::SupportVertex(const ndVector& dir) const
{
	ndAssert(dir.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));

	ndFloat32 x = dir.GetScalar();
	if (ndAbs(x) > ndFloat32(0.9999f)) 
	{
		return ndVector(ndSign(x) * m_height, m_radius, ndFloat32(0.0f), ndFloat32(0.0f));
	}

	ndVector sideDir(m_yzMask & dir);
	sideDir = sideDir.Normalize();
	return sideDir.Scale(m_radius) + dir.Scale(m_height);
}

ndVector ndShapeChamferCylinder::SupportVertexSpecial(const ndVector& dir, ndFloat32) const
{
	ndAssert(dir.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));

	ndFloat32 x = dir.GetScalar();
	if (ndAbs(x) > ndFloat32(0.99995f)) 
	{
		return ndVector(ndFloat32(0.0f), m_radius, ndFloat32(0.0f), ndFloat32(0.0f));
	}

	ndVector sideDir(m_yzMask & dir);
	ndAssert(sideDir.DotProduct(sideDir).GetScalar() > ndFloat32(0.0f));
	return sideDir.Normalize().Scale(m_radius);
}

ndFloat32 ndShapeChamferCylinder::RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	if (localP0.m_x > m_height) 
	{
		if (localP1.m_x < m_height) 
		{
			ndFloat32 t1 = (m_height - localP0.m_x) / (localP1.m_x - localP0.m_x);
			ndFloat32 y = localP0.m_y + (localP1.m_y - localP0.m_y) * t1;
			ndFloat32 z = localP0.m_z + (localP1.m_z - localP0.m_z) * t1;
			if ((y * y + z * z) < m_radius * m_radius) 
			{
				contactOut.m_normal = ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
				return t1;
			}
		}
	}

	if (localP0.m_x < -m_height) 
	{
		if (localP1.m_x > -m_height) 
		{
			ndFloat32 t1 = (-m_height - localP0.m_x) / (localP1.m_x - localP0.m_x);
			ndFloat32 y = localP0.m_y + (localP1.m_y - localP0.m_y) * t1;
			ndFloat32 z = localP0.m_z + (localP1.m_z - localP0.m_z) * t1;
			if ((y * y + z * z) < m_radius * m_radius) 
			{
				contactOut.m_normal = ndVector(ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
				return t1;
			}
		}
	}

	ndVector dq((localP1 - localP0) & ndVector::m_triplexMask);

	// avoid NaN as a result of a division by zero
	if (dq.DotProduct(dq).GetScalar() <= 0.0f) 
	{
		return ndFloat32(1.2f);
	}

	//ndVector dir(dq * dq.InvMagSqrt());
	ndVector dir(dq.Normalize());
	if (ndAbs(dir.m_x) > 0.9999f) 
	{
		//return ndShapeConvex::RayCast(localP0, localP1, maxT, contactOut, body, nullptr, nullptr);
		return ndShapeConvex::RayCast(callback, localP0, localP1, maxT, body, contactOut);
	}

	ndVector p0(localP0 & ndVector::m_triplexMask);
	ndVector p1(localP1 & ndVector::m_triplexMask);

	p0.m_x = ndFloat32(0.0f);
	p1.m_x = ndFloat32(0.0f);

	ndVector dp(p1 - p0);
	ndFloat32 a = dp.DotProduct(dp).GetScalar();
	ndFloat32 b = ndFloat32(2.0f) * dp.DotProduct(p0).GetScalar();
	ndFloat32 c = p0.DotProduct(p0).GetScalar() - m_radius * m_radius;

	ndFloat32 disc = b * b - ndFloat32(4.0f) * a * c;
	if (disc >= ndFloat32(0.0f)) 
	{
		disc = ndSqrt(disc);
		ndVector origin0(p0 + dp.Scale((-b + disc) / (ndFloat32(2.0f) * a)));
		ndVector origin1(p0 + dp.Scale((-b - disc) / (ndFloat32(2.0f) * a)));
		ndFloat32 t0 = ndRayCastSphere(localP0, localP1, origin0, m_height);
		ndFloat32 t1 = ndRayCastSphere(localP0, localP1, origin1, m_height);
		if (t1 < t0) 
		{
			t0 = t1;
			origin0 = origin1;
		}

		if ((t0 >= 0.0f) && (t0 <= 1.0f)) 
		{
			contactOut.m_normal = localP0 + dq.Scale(t0) - origin0;
			ndAssert(contactOut.m_normal.m_w == ndFloat32(0.0f));

			//contactOut.m_normal = contactOut.m_normal * contactOut.m_normal.DotProduct(contactOut.m_normal).InvSqrt();
			contactOut.m_normal = contactOut.m_normal.Normalize();
			return t0;
		}
	}
	else 
	{
		ndVector origin0(ndPointToRayDistance(ndVector::m_zero, p0, p1));
		origin0 = origin0.Scale(m_radius / ndSqrt(origin0.DotProduct(origin0).GetScalar()));
		ndFloat32 t0 = ndRayCastSphere(localP0, localP1, origin0, m_height);
		if ((t0 >= 0.0f) && (t0 <= 1.0f)) 
		{
			contactOut.m_normal = localP0 + dq.Scale(t0) - origin0;
			ndAssert(contactOut.m_normal.m_w == ndFloat32(0.0f));

			//contactOut.m_normal = contactOut.m_normal * contactOut.m_normal.DotProduct(contactOut.m_normal).InvSqrt();
			contactOut.m_normal = contactOut.m_normal.Normalize();
			return t0;
		}
	}
	return ndFloat32(1.2f);
}

ndInt32 ndShapeChamferCylinder::CalculatePlaneIntersection(const ndVector& normal, const ndVector& origin, ndVector* const contactsOut) const
{
	ndInt32 count = 0;
	const ndFloat32 inclination = ndFloat32(0.9999f);
	//if (normal.m_x < -inclination) 
	//{
	//	ndMatrix matrix(ndGramSchmidtMatrix(normal));
	//	ndFloat32 x = ndSqrt(ndMax(m_height * m_height - origin.m_x * origin.m_x, ndFloat32(0.0f)));
	//	matrix.m_posit.m_x = origin.m_x;
	//	count = BuildCylinderCapPoly(m_radius + x, matrix, contactsOut);
	//	//count = RectifyConvexSlice(n, normal, contactsOut);
	//}
	//else if (normal.m_x > inclination) 
	//{
	//	ndMatrix matrix(ndGramSchmidtMatrix(normal));
	//	ndFloat32 x = ndSqrt(ndMax(m_height * m_height - origin.m_x * origin.m_x, ndFloat32(0.0f)));
	//	matrix.m_posit.m_x = origin.m_x;
	//	count = BuildCylinderCapPoly(m_radius + x, matrix, contactsOut);
	//	//count = RectifyConvexSlice(n, normal, contactsOut);
	//}
	if (ndAbs(normal.m_x) > inclination)
	{
		ndMatrix matrix(ndGramSchmidtMatrix(normal));
		ndFloat32 x = ndSqrt(ndMax(m_height * m_height - origin.m_x * origin.m_x, ndFloat32(0.0f)));
		matrix.m_posit.m_x = origin.m_x;
		count = BuildCylinderCapPoly(m_radius + x, matrix, contactsOut);
	}
	else 
	{
		count = 1;
		contactsOut[0] = SupportVertex(normal);
	}
	return count;
}

ndUnsigned64 ndShapeChamferCylinder::GetHash(ndUnsigned64 hash) const
{
	ndShapeInfo info(GetShapeInfo());
	return info.GetHash(hash);
}