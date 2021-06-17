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
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndShapeConvexPolygon.h"

#define D_CONVEX_POLYGON_SKIRT_LENGTH dFloat32 (0.025f)

ndShapeConvexPolygon::ndShapeConvexPolygon ()
	:ndShapeConvex(m_polygonCollision)
	,m_count(0)
	,m_paddedCount(0)
	,m_stride(0)
	,m_faceNormalIndex(0)
	,m_faceClipSize(0) 
	,m_vertex(nullptr)
	,m_vertexIndex(nullptr)
	,m_adjacentFaceEdgeNormalIndex(nullptr)
{
}

ndShapeConvexPolygon::~ndShapeConvexPolygon ()
{
}

dInt32 ndShapeConvexPolygon::CalculatePlaneIntersection (const dVector& normalIn, const dVector& origin, dVector* const contactsOut) const
{
	dAssert (normalIn.m_w == dFloat32 (0.0f));
	dVector normal(normalIn);
	dInt32 count = 0;
	dFloat32 maxDist = dFloat32 (1.0f);
	dFloat32 projectFactor = m_normal.DotProduct(normal).GetScalar();
	if (projectFactor < dFloat32 (0.0f)) 
	{
		projectFactor *= dFloat32 (-1.0f);
		normal = normal * dVector::m_negOne;
	}

	if (projectFactor > dFloat32 (0.9999f)) 
	{
		for (dInt32 i = 0; i < m_count; i ++) 
		{
			contactsOut[count] = m_localPoly[i];
			count ++;
		}

		#ifdef _DEBUG
			dInt32 j = count - 1;
			for (dInt32 i = 0; i < count; i ++) 
			{
				dVector error (contactsOut[i] - contactsOut[j]);
				dAssert (error.m_w == dFloat32 (0.0f));
				dAssert (error.DotProduct(error).GetScalar() > dFloat32 (1.0e-20f));
				j = i;
			}
		#endif

	} 
	else if (projectFactor > dFloat32 (0.1736f)) 
	{
		maxDist = dFloat32 (0.0f);
		dPlane plane (normal, - normal.DotProduct(origin).GetScalar());

		dVector p0 (m_localPoly[m_count - 1]);
		dFloat32 side0 = plane.Evalue (p0);
		for (dInt32 i = 0; i < m_count; i ++) 
		{
			dVector p1 (m_localPoly[i]);
			dFloat32 side1 = plane.Evalue (p1);

			if (side0 > dFloat32 (0.0f)) 
			{
				maxDist = dMax (maxDist, side0);
				contactsOut[count] = p0 - normal.Scale (side0);
				count ++;
				if (count > 1) 
				{
					dVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dAssert (edgeSegment.m_w == dFloat32 (0.0f));
					dFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < dFloat32 (1.0e-8f)) {
						count --;
					}
				}

				if (side1 <= dFloat32 (0.0f)) 
				{
					dVector dp (p1 - p0);
					dFloat32 t = normal.DotProduct(dp).GetScalar();
					dAssert (dAbs (t) >= dFloat32 (0.0f));
					if (dAbs (t) < dFloat32 (1.0e-8f)) 
					{
						t = dSign(t) * dFloat32 (1.0e-8f);	
					}
					contactsOut[count] = p0 - dp.Scale (side0 / t);
					count ++;
					if (count > 1) 
					{
						dVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
						dAssert (edgeSegment.m_w == dFloat32 (0.0f));
						dFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
						if (error < dFloat32 (1.0e-8f)) 
						{
							count --;
						}
					}
				} 
			} 
			else if (side1 > dFloat32 (0.0f)) 
			{
				dVector dp (p1 - p0);
				dFloat32 t = normal.DotProduct(dp).GetScalar();
				dAssert (dAbs (t) >= dFloat32 (0.0f));
				if (dAbs (t) < dFloat32 (1.0e-8f)) 
				{
					t = dSign(t) * dFloat32 (1.0e-8f);	
				}
				contactsOut[count] = p0 - dp.Scale (side0 / t);
				count ++;
				if (count > 1) 
				{
					dVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dAssert (edgeSegment.m_w == dFloat32 (0.0f));
					dFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < dFloat32 (1.0e-8f)) 
					{
						count --;
					}
				}
			}

			side0 = side1;
			p0 = p1;
		}
	} 
	else 
	{
		maxDist = dFloat32 (1.0e10f);
		dPlane plane (normal, - normal.DotProduct(origin).GetScalar());

		dVector p0 (m_localPoly[m_count - 1]);
		dFloat32 side0 = plane.Evalue (p0);
		for (dInt32 i = 0; i < m_count; i ++) 
		{
			dVector p1 (m_localPoly[i]);
			dFloat32 side1 = plane.Evalue (p1);

			if ((side0 * side1) < dFloat32 (0.0f)) 
			{
				dVector dp (p1 - p0);
				dFloat32 t = normal.DotProduct(dp).GetScalar();
				dAssert (dAbs (t) >= dFloat32 (0.0f));
				if (dAbs (t) < dFloat32 (1.0e-8f)) 
				{
					t = dSign(t) * dFloat32 (1.0e-8f);	
				}
				contactsOut[count] = p0 - dp.Scale (side0 / t);
				count ++;
				if (count > 1) 
				{
					dVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dAssert (edgeSegment.m_w == dFloat32 (0.0f));
					dFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < dFloat32 (1.0e-8f)) 
					{
						count --;
					}
				}
			}
			side0 = side1;
			p0 = p1;
		}
	}

	if (count > 1) 
	{
		if (maxDist < dFloat32 (1.0e-3f)) 
		{
			dVector maxPoint (contactsOut[0]);
			dVector minPoint (contactsOut[0]);
			dVector lineDir (m_normal.CrossProduct(normal));

			dAssert (lineDir.m_w == dFloat32 (0.0f));
			dFloat32 proj = contactsOut[0].DotProduct(lineDir).GetScalar();
			dFloat32 maxProjection = proj;
			dFloat32 minProjection = proj;
			for (dInt32 i = 1; i < count; i ++) 
			{
				proj = contactsOut[i].DotProduct(lineDir).GetScalar();
				if (proj > maxProjection) 
				{
					maxProjection = proj;
					maxPoint = contactsOut[i];
				}
				if (proj < minProjection) 
				{
					minProjection = proj;
					minPoint = contactsOut[i];
				}
			}	

			contactsOut[0] = maxPoint;
			contactsOut[1] = minPoint;
			count = 2;
		}

		dVector error (contactsOut[count - 1] - contactsOut[0]);
		dAssert (error.m_w == dFloat32 (0.0f));
		if (error.DotProduct(error).GetScalar() < dFloat32 (1.0e-8f)) 
		{
			count --;
		}
	}

	#ifdef _DEBUG
		if (count > 1) 
		{
			dInt32 j = count - 1;
			for (dInt32 i = 0; i < count; i ++) 
			{
				dVector error (contactsOut[i] - contactsOut[j]);
				dAssert (error.m_w == dFloat32 (0.0f));
				dAssert (error.DotProduct(error).GetScalar() > dFloat32 (1.0e-20f));
				j = i;
			}

			if (count >= 3) 
			{
				dVector n (dFloat32 (0.0f));
				dVector e0 (contactsOut[1] - contactsOut[0]);
				for (dInt32 i = 2; i < count; i ++) 
				{
					dVector e1 (contactsOut[i] - contactsOut[0]);
					n += e0.CrossProduct(e1);
					e0 = e1;
				} 
				dAssert (n.m_w == dFloat32 (0.0f));
				n = n.Normalize();
				dAssert (n.DotProduct(normal).GetScalar() > dFloat32 (0.9f));
			}
		}
	#endif
	return count;
}

dVector ndShapeConvexPolygon::SupportVertex(const dVector& dir, dInt32* const) const
{
	dAssert(dAbs(dir.m_w) == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - 1.0f) < dFloat32(1.0e-2f));

	dInt32 index = 0;
	dFloat32 val = m_localPoly[0].DotProduct(dir).GetScalar();
	for (dInt32 i = 1; i < m_paddedCount; i++) 
	{
		dFloat32 val1 = m_localPoly[i].DotProduct(dir).GetScalar();
		if (val1 > val) 
		{
			val = val1;
			index = i;
		}
	}
	//dAssert(vertexIndex == nullptr);
	return m_localPoly[index];
}

dVector ndShapeConvexPolygon::CalculateGlobalNormal(const ndShapeInstance* const parentMesh, const dVector& localNormal) const
{
	const dVector& invScale = parentMesh->GetInvScale();
	const dMatrix& globalMatrix = parentMesh->m_globalMatrix;
	const dMatrix& aligmentMatrix = parentMesh->m_aligmentMatrix;

	dVector normal(aligmentMatrix.RotateVector(localNormal));
	normal = normal * invScale;
	dAssert(normal.m_w == dFloat32(0.0f));
	normal = normal.Normalize();
	return globalMatrix.RotateVector(normal);
}

bool ndShapeConvexPolygon::BeamClipping(const dVector& origin, dFloat32 dist, const ndShapeInstance* const parentMesh)
{
	dPlane planes[4];
	dVector points[128];

	dgClippedFaceEdge clippedFace[2 * sizeof(m_localPoly) / sizeof(m_localPoly[0]) + 8];

	dVector dir(m_localPoly[1] - m_localPoly[0]);
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dir.DotProduct(dir).GetScalar() > dFloat32(1.0e-8f));
	dir = dir.Normalize();

	dFloat32 distH = origin.DotProduct(dir).GetScalar();
	planes[0] = dPlane(dir, dist - distH);
	planes[2] = dPlane(dir * dVector::m_negOne, dist + distH);

	dir = m_normal.CrossProduct(dir);
	dFloat32 distV = origin.DotProduct(dir).GetScalar();
	planes[1] = dPlane(dir, dist - distV);
	planes[3] = dPlane(dir * dVector::m_negOne, dist + distV);

	for (dInt32 i = 0; i < m_count; i++) 
	{
		dInt32 j = i << 1;
		dAssert(j < dInt32 (sizeof(clippedFace) / sizeof(clippedFace[0])));

		points[i] = m_localPoly[i];

		clippedFace[j + 0].m_twin = &clippedFace[j + 1];
		clippedFace[j + 0].m_next = &clippedFace[j + 2];
		clippedFace[j + 0].m_incidentVertex = i;
		clippedFace[j + 0].m_incidentNormal = m_adjacentFaceEdgeNormalIndex[i] & (~D_CONCAVE_EDGE_MASK);

		clippedFace[j + 1].m_twin = &clippedFace[j + 0];
		clippedFace[j + 1].m_next = &clippedFace[j - 2];
		clippedFace[j + 1].m_incidentVertex = i + 1;
		clippedFace[j + 1].m_incidentNormal = -1;
	}

	clippedFace[1].m_next = &clippedFace[m_count * 2 - 2 + 1];
	dAssert((m_count * 2 - 2) >= 0);
	clippedFace[m_count * 2 - 2].m_next = &clippedFace[0];
	clippedFace[m_count * 2 - 2 + 1].m_incidentVertex = 0;

	const dFloat32 tol = dFloat32(1.0e-5f);
	dInt32 edgeCount = m_count * 2;
	dInt32 indexCount = m_count;
	dgClippedFaceEdge* first = &clippedFace[0];
	for (dInt32 i = 0; i < 4; i++) 
	{
		const dPlane& plane = planes[i];

		dInt32 conectCount = 0;
		dgClippedFaceEdge* connect[2];
		dgClippedFaceEdge* ptr = first;
		dgClippedFaceEdge* newFirst = first;
		dFloat32 test0 = plane.Evalue(points[ptr->m_incidentVertex]);
		do 
		{
			dFloat32 test1 = plane.Evalue(points[ptr->m_next->m_incidentVertex]);

			if (test0 > tol) 
			{
				if (test1 <= -tol) 
				{
					const dVector& p0 = points[ptr->m_incidentVertex];
					const dVector& p1 = points[ptr->m_next->m_incidentVertex];
					dVector dp(p1 - p0);
					points[indexCount] = p0 - dp.Scale(test0 / dp.DotProduct(plane).GetScalar());

					dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
					newEdge->m_twin = newEdge + 1;
					newEdge->m_twin->m_twin = newEdge;

					newEdge->m_twin->m_incidentNormal = ptr->m_incidentNormal;
					newEdge->m_incidentNormal = ptr->m_incidentNormal;

					newEdge->m_incidentVertex = indexCount;
					newEdge->m_twin->m_incidentVertex = ptr->m_next->m_incidentVertex;
					ptr->m_twin->m_incidentVertex = indexCount;

					newEdge->m_next = ptr->m_next;
					ptr->m_next->m_twin->m_next = newEdge->m_twin;
					newEdge->m_twin->m_next = ptr->m_twin;
					ptr->m_next = newEdge;

					connect[conectCount] = ptr;
					conectCount++;
					indexCount++;
					edgeCount += 2;
					ptr = newEdge;
					dAssert(indexCount < dInt32 (sizeof(points) / sizeof(points[0])));
				}
			}
			else 
			{
				if ((test1 > tol) && (test0 * test1) < dFloat32(0.0f)) 
				{
					newFirst = ptr->m_next;

					const dVector& p0 = points[ptr->m_incidentVertex];
					const dVector& p1 = points[ptr->m_next->m_incidentVertex];
					dVector dp(p1 - p0);
					points[indexCount] = p0 - dp.Scale(test0 / dp.DotProduct(plane).GetScalar());

					dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
					newEdge->m_twin = newEdge + 1;
					newEdge->m_twin->m_twin = newEdge;

					newEdge->m_twin->m_incidentNormal = ptr->m_incidentNormal;
					newEdge->m_incidentNormal = ptr->m_incidentNormal;

					newEdge->m_incidentVertex = indexCount;
					newEdge->m_twin->m_incidentVertex = ptr->m_next->m_incidentVertex;
					ptr->m_twin->m_incidentVertex = indexCount;

					newEdge->m_next = ptr->m_next;
					ptr->m_next->m_twin->m_next = newEdge->m_twin;
					newEdge->m_twin->m_next = ptr->m_twin;
					ptr->m_next = newEdge;

					connect[conectCount] = ptr;
					conectCount++;
					indexCount++;
					edgeCount += 2;

					ptr = newEdge;
					dAssert(indexCount < dInt32 (sizeof(points) / sizeof(points[0])));
				}
			}

			test0 = test1;
			ptr = ptr->m_next;
		} while (ptr != first);

		if (conectCount > 1) 
		{
			first = newFirst;
			dAssert(conectCount == 2);

			dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
			newEdge->m_twin = newEdge + 1;
			newEdge->m_twin->m_twin = newEdge;

			newEdge->m_incidentNormal = m_faceNormalIndex;
			newEdge->m_incidentVertex = connect[0]->m_next->m_incidentVertex;
			newEdge->m_twin->m_next = connect[0]->m_next;
			connect[0]->m_next = newEdge;

			newEdge->m_twin->m_incidentNormal = m_faceNormalIndex;
			newEdge->m_twin->m_incidentVertex = connect[1]->m_next->m_incidentVertex;
			newEdge->m_next = connect[1]->m_next;
			connect[1]->m_next = newEdge->m_twin;

			edgeCount += 2;
		}
	}

	dgClippedFaceEdge* ptr = first;
	do 
	{
		dVector dist1(points[ptr->m_next->m_incidentVertex] - points[ptr->m_incidentVertex]);
		dAssert(dist1.m_w == dFloat32(0.0f));
		dFloat32 error = dist1.DotProduct(dist1).GetScalar();
		if (error < dFloat32(1.0e-6f)) 
		{
			ptr->m_next = ptr->m_next->m_next;
			first = ptr;
		}
		ptr = ptr->m_next;
	} while (ptr != first);

	dInt32 count = 0;
	m_adjacentFaceEdgeNormalIndex = &m_clippEdgeNormal[0];
	do 
	{
		m_clippEdgeNormal[count] = ptr->m_incidentNormal;
		m_localPoly[count] = points[ptr->m_incidentVertex];
		count++;
		ptr = ptr->m_next;
		dAssert(m_count < D_CONVEX_POLYGON_MAX_VERTEX_COUNT);
	} while (ptr != first);

	m_count = count;

	if (m_count >= 3) 
	{
		dInt32 i0 = m_count - 1;
		for (dInt32 i = 0; i < m_count; i++) 
		{
			const dVector faceEddge(m_localPoly[i] - m_localPoly[i0]);
			dAssert(faceEddge.m_w == dFloat32(0.0f));
			dAssert(faceEddge.DotProduct(faceEddge).GetScalar() > dFloat32(0.0f));
			const dVector edge (faceEddge.Normalize());
			const dInt32 adjacentNormalIndex = m_adjacentFaceEdgeNormalIndex[i0];
			const dVector localAdjacentNormal(&m_vertex[adjacentNormalIndex * m_stride]);
			const dVector adjacentNormal(CalculateGlobalNormal(parentMesh, localAdjacentNormal & dVector::m_triplexMask));
			const dVector edgeSkirt(edge.CrossProduct(adjacentNormal).Scale(D_CONVEX_POLYGON_SKIRT_LENGTH));

			m_localPoly[count + 0] = m_localPoly[i] + edgeSkirt;
			m_localPoly[count + 1] = m_localPoly[i0] + edgeSkirt;
			count += 2;
			i0 = i;
		}
		m_paddedCount = count;
	}

	return (m_count >= 3);
}

dInt32 ndShapeConvexPolygon::CalculateContactToConvexHullDescrete(const ndShapeInstance* const parentMesh, ndContactSolver& proxy)
{
	dInt32 count = 0;
	dAssert(proxy.m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(proxy.m_instance1.GetShape()->GetAsShapeAsConvexPolygon());
	dAssert(proxy.m_instance1.GetGlobalMatrix().TestIdentity());
	dAssert(this == proxy.m_instance1.GetShape());
	dAssert(m_count);
	dAssert(m_count < dInt32(sizeof(m_localPoly) / sizeof(m_localPoly[0])));

	const dMatrix& hullMatrix = proxy.m_instance0.m_globalMatrix;
	ndContact* const contactJoint = proxy.m_contact;
	const ndShapeInstance* const hull = &proxy.m_instance0;

	dAssert(m_normal.m_w == dFloat32(0.0f));
	const dVector obbOrigin(hullMatrix.TransformVector(proxy.m_instance0.GetShape()->GetObbOrigin()));
	const dFloat32 shapeSide = m_normal.DotProduct(obbOrigin - m_localPoly[0]).GetScalar();
	if (shapeSide < dFloat32(0.0f))
	{
		//dgTrace(("normal face away (%f %f %f)\n", m_normal[0], m_normal[1], m_normal[2]));
		return 0;
	}

	dVector normalInHull(hullMatrix.UnrotateVector(m_normal));
	dVector pointInHull(hull->SupportVertex(normalInHull.Scale(dFloat32(-1.0f))));
	dVector p0(hullMatrix.TransformVector(pointInHull));

	dFloat32 penetration = m_normal.DotProduct(m_localPoly[0] - p0).GetScalar() + proxy.m_skinThickness;

	if (penetration < -(D_PENETRATION_TOL * dFloat32(5.0f)))
	{
		return 0;
	}

	dVector p1(hullMatrix.TransformVector(hull->SupportVertex(normalInHull)));
	contactJoint->m_separationDistance = dFloat32(0.0f);
	dFloat32 distance = m_normal.DotProduct(m_localPoly[0] - p1).GetScalar();
	if (distance >= dFloat32(0.0f))
	{
		return 0;
	}

	dVector boxSize;
	dVector boxOrigin;
	hull->CalculateObb(boxOrigin, boxSize);
	dAssert(boxOrigin.m_w == dFloat32(0.0f));
	boxOrigin += dVector::m_wOne;

	bool inside = true;
	dInt32 i0 = m_count - 1;
	for (dInt32 i = 0; i < m_count; i++)
	{
		dVector e(m_localPoly[i] - m_localPoly[i0]);
		dVector edgeBoundaryNormal(m_normal.CrossProduct(e));
		dAssert(edgeBoundaryNormal.m_w == dFloat32(0.0f));
		dPlane plane(edgeBoundaryNormal, -m_localPoly[i0].DotProduct(edgeBoundaryNormal).GetScalar());
		plane = hullMatrix.UntransformPlane(plane);

		dFloat32 supportDist = boxSize.DotProduct(plane.Abs()).GetScalar();
		dFloat32 centerDist = plane.DotProduct(boxOrigin).GetScalar();

		if ((centerDist + supportDist) < dFloat32(0.0f))
		{
			return 0;
		}

		if ((centerDist - supportDist) < dFloat32(0.0f))
		{
			inside = false;
			break;
		}
		i0 = i;
	}

	dFloat32 convexSphapeUmbra = hull->GetUmbraClipSize();
	if (m_faceClipSize > convexSphapeUmbra)
	{
		dVector boxP0;
		dVector boxP1;
		hull->CalculateAABB(hullMatrix, boxP0, boxP1);
		dVector origin(dVector::m_half * (boxP1 + boxP1));

		if (!BeamClipping(origin, convexSphapeUmbra, parentMesh))
		{
			return 0;
		}
		m_faceClipSize = hull->GetShape()->GetBoxMaxRadius();
	}

	const dInt64 hullId = hull->GetUserDataID();
	if (inside & !proxy.m_intersectionTestOnly)
	{
		penetration = dMax(dFloat32(0.0f), penetration);
		dAssert(penetration >= dFloat32(0.0f));
		dVector contactPoints[128];
		dVector point(pointInHull + normalInHull.Scale(penetration - D_PENETRATION_TOL));

		count = hull->CalculatePlaneIntersection(normalInHull.Scale(dFloat32(-1.0f)), point, contactPoints);
		dVector step(normalInHull.Scale((proxy.m_skinThickness - penetration) * dFloat32(0.5f)));

		ndContactPoint* const contactsOut = proxy.m_contactBuffer;
		for (dInt32 i = 0; i < count; i++)
		{
			contactsOut[i].m_point = hullMatrix.TransformVector(contactPoints[i] + step);
			contactsOut[i].m_normal = m_normal;
			contactsOut[i].m_shapeId0 = hullId;
			contactsOut[i].m_shapeId1 = m_faceId;
			contactsOut[i].m_penetration = penetration;
		}
	}
	else
	{
		m_vertexCount = dUnsigned16(m_count);
		count = proxy.CalculateConvexToConvexContacts();
		dAssert(proxy.m_intersectionTestOnly || (count >= 0));
		if (count >= 1)
		{
			ndContactPoint* const contactsOut = proxy.m_contactBuffer;
			for (dInt32 i = 0; i < count; i++)
			{
				contactsOut[i].m_shapeId0 = hullId;
				contactsOut[i].m_shapeId1 = m_faceId;
			}
		}
	}

	return count;
}