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
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"
#include "ndShapeConvexPolygon.h"

//#define D_CONVEX_POLYGON_SKIRT_LENGTH dFloat32 (0.025f)
#define D_CONVEX_POLYGON_SKIRT_LENGTH dFloat32 (0.25f)

ndShapeConvexPolygon::ndShapeConvexPolygon ()
	:ndShapeConvex(m_polygonCollision)
	,m_faceClipSize(0)
	,m_count(0)
	,m_paddedCount(0)
	,m_stride(0)
	,m_faceNormalIndex(0)
	,m_vertex(nullptr)
	,m_vertexIndex(nullptr)
	,m_adjacentFaceEdgeNormalIndex(nullptr)
{
}

ndShapeConvexPolygon::~ndShapeConvexPolygon ()
{
}

dInt32 ndShapeConvexPolygon::Release() const
{
	dInt32 count = m_refCount.fetch_add(-1);
	return count;
}

dInt32 ndShapeConvexPolygon::CalculatePlaneIntersection (const ndVector& normalIn, const ndVector& origin, ndVector* const contactsOut) const
{
	dAssert (normalIn.m_w == dFloat32 (0.0f));
	ndVector normal(normalIn);
	dInt32 count = 0;
	dFloat32 maxDist = dFloat32 (1.0f);
	dFloat32 projectFactor = m_normal.DotProduct(normal).GetScalar();
	if (projectFactor < dFloat32 (0.0f)) 
	{
		projectFactor *= dFloat32 (-1.0f);
		normal = normal * ndVector::m_negOne;
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
				ndVector error (contactsOut[i] - contactsOut[j]);
				dAssert (error.m_w == dFloat32 (0.0f));
				dAssert (error.DotProduct(error).GetScalar() > dFloat32 (1.0e-20f));
				j = i;
			}
		#endif

	} 
	else if (projectFactor > dFloat32 (0.1736f)) 
	{
		maxDist = dFloat32 (0.0f);
		ndPlane plane (normal, - normal.DotProduct(origin).GetScalar());

		ndVector p0 (m_localPoly[m_count - 1]);
		dFloat32 side0 = plane.Evalue (p0);
		for (dInt32 i = 0; i < m_count; i ++) 
		{
			ndVector p1 (m_localPoly[i]);
			dFloat32 side1 = plane.Evalue (p1);

			if (side0 > dFloat32 (0.0f)) 
			{
				maxDist = dMax (maxDist, side0);
				contactsOut[count] = p0 - normal.Scale (side0);
				count ++;
				if (count > 1) 
				{
					ndVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dAssert (edgeSegment.m_w == dFloat32 (0.0f));
					dFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < dFloat32 (1.0e-8f)) {
						count --;
					}
				}

				if (side1 <= dFloat32 (0.0f)) 
				{
					ndVector dp (p1 - p0);
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
						ndVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
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
				ndVector dp (p1 - p0);
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
					ndVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
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
		ndPlane plane (normal, - normal.DotProduct(origin).GetScalar());

		ndVector p0 (m_localPoly[m_count - 1]);
		dFloat32 side0 = plane.Evalue (p0);
		for (dInt32 i = 0; i < m_count; i ++) 
		{
			ndVector p1 (m_localPoly[i]);
			dFloat32 side1 = plane.Evalue (p1);

			if ((side0 * side1) < dFloat32 (0.0f)) 
			{
				ndVector dp (p1 - p0);
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
					ndVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
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
			ndVector maxPoint (contactsOut[0]);
			ndVector minPoint (contactsOut[0]);
			ndVector lineDir (m_normal.CrossProduct(normal));

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

		ndVector error (contactsOut[count - 1] - contactsOut[0]);
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
				ndVector error (contactsOut[i] - contactsOut[j]);
				dAssert (error.m_w == dFloat32 (0.0f));
				dAssert (error.DotProduct(error).GetScalar() > dFloat32 (1.0e-20f));
				j = i;
			}

			if (count >= 3) 
			{
				ndVector n (dFloat32 (0.0f));
				ndVector e0 (contactsOut[1] - contactsOut[0]);
				for (dInt32 i = 2; i < count; i ++) 
				{
					ndVector e1 (contactsOut[i] - contactsOut[0]);
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

ndVector ndShapeConvexPolygon::SupportVertex(const ndVector& dir, dInt32* const) const
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

ndVector ndShapeConvexPolygon::CalculateGlobalNormal(const ndShapeInstance* const parentMesh, const ndVector& localNormal) const
{
	const ndVector& invScale = parentMesh->GetInvScale();
	const ndMatrix& globalMatrix = parentMesh->m_globalMatrix;
	const ndMatrix& aligmentMatrix = parentMesh->m_aligmentMatrix;

	ndVector normal(aligmentMatrix.RotateVector(localNormal));
	normal = normal * invScale;
	dAssert(normal.m_w == dFloat32(0.0f));
	normal = normal.Normalize();
	return globalMatrix.RotateVector(normal);
}

bool ndShapeConvexPolygon::BeamClipping(const ndVector& origin, dFloat32 dist, const ndShapeInstance* const parentMesh)
{
	ndPlane planes[4];
	ndVector points[128];

	dgClippedFaceEdge clippedFace[2 * sizeof(m_localPoly) / sizeof(m_localPoly[0]) + 8];

	ndVector dir(m_localPoly[1] - m_localPoly[0]);
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dir.DotProduct(dir).GetScalar() > dFloat32(1.0e-8f));
	dir = dir.Normalize();

	dFloat32 distH = origin.DotProduct(dir).GetScalar();
	planes[0] = ndPlane(dir, dist - distH);
	planes[2] = ndPlane(dir * ndVector::m_negOne, dist + distH);

	dir = m_normal.CrossProduct(dir);
	dFloat32 distV = origin.DotProduct(dir).GetScalar();
	planes[1] = ndPlane(dir, dist - distV);
	planes[3] = ndPlane(dir * ndVector::m_negOne, dist + distV);

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
		const ndPlane& plane = planes[i];

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
					const ndVector& p0 = points[ptr->m_incidentVertex];
					const ndVector& p1 = points[ptr->m_next->m_incidentVertex];
					ndVector dp(p1 - p0);
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

					const ndVector& p0 = points[ptr->m_incidentVertex];
					const ndVector& p1 = points[ptr->m_next->m_incidentVertex];
					ndVector dp(p1 - p0);
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
		ndVector dist1(points[ptr->m_next->m_incidentVertex] - points[ptr->m_incidentVertex]);
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
		const ndVector skirt(D_CONVEX_POLYGON_SKIRT_LENGTH);
		dInt32 i0 = m_count - 1;
		for (dInt32 i = 0; i < m_count; i++) 
		{
			const ndVector faceEddge(m_localPoly[i] - m_localPoly[i0]);
			dAssert(faceEddge.m_w == dFloat32(0.0f));
			dAssert(faceEddge.DotProduct(faceEddge).GetScalar() > dFloat32(0.0f));
			const ndVector edge (faceEddge.Normalize());
			const dInt32 adjacentNormalIndex = m_adjacentFaceEdgeNormalIndex[i0];
			const ndVector localAdjacentNormal(&m_vertex[adjacentNormalIndex * m_stride]);
			const ndVector adjacentNormal(CalculateGlobalNormal(parentMesh, localAdjacentNormal & ndVector::m_triplexMask));
			dAssert(edge.DotProduct(adjacentNormal).GetScalar() < dFloat32(5.0e-2f));
			const ndVector edgeSkirt(edge.CrossProduct(adjacentNormal) * skirt);

			m_localPoly[count + 0] = m_localPoly[i] + edgeSkirt;
			m_localPoly[count + 1] = m_localPoly[i0] + edgeSkirt;
			count += 2;
			i0 = i;
		}
		m_paddedCount = count;
	}

	return (m_count >= 3);
}

dInt32 ndShapeConvexPolygon::CalculateContactToConvexHullDescrete(const ndShapeInstance* const parentMesh, ndContactSolver& contactSolver)
{
	dAssert(contactSolver.m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(contactSolver.m_instance1.GetShape()->GetAsShapeAsConvexPolygon());
	dAssert(contactSolver.m_instance1.GetGlobalMatrix().TestIdentity());
	dAssert(this == contactSolver.m_instance1.GetShape());
	dAssert(m_count);
	dAssert(m_count < dInt32(sizeof(m_localPoly) / sizeof(m_localPoly[0])));

	const ndMatrix& hullMatrix = contactSolver.m_instance0.m_globalMatrix;
	const ndShapeInstance* const hull = &contactSolver.m_instance0;

	dAssert(m_normal.m_w == dFloat32(0.0f));
	const ndVector obbOrigin(hullMatrix.TransformVector(contactSolver.m_instance0.GetShape()->GetObbOrigin()));
	const dFloat32 shapeSide = m_normal.DotProduct(obbOrigin - m_localPoly[0]).GetScalar();
	if (shapeSide < dFloat32(0.0f))
	{
		//dgTrace(("normal face away (%f %f %f)\n", m_normal[0], m_normal[1], m_normal[2]));
		return 0;
	}

	const ndVector normalInHull(hullMatrix.UnrotateVector(m_normal));
	const ndVector pointInHull(hull->SupportVertex(normalInHull.Scale(dFloat32(-1.0f))));
	const ndVector p0(hullMatrix.TransformVector(pointInHull));

	dFloat32 penetration = m_normal.DotProduct(m_localPoly[0] - p0).GetScalar() + contactSolver.m_skinThickness;
	if (penetration < -(D_PENETRATION_TOL * dFloat32(5.0f)))
	{
		contactSolver.m_separatingVector = m_normal;
		contactSolver.m_closestPoint0 = p0;
		contactSolver.m_closestPoint1 = p0 + m_normal.Scale(penetration);
		contactSolver.m_separationDistance = -penetration;
		return 0;
	}

	const ndVector p1(hullMatrix.TransformVector(hull->SupportVertex(normalInHull)));

	//contactJoint->m_separationDistance = dFloat32(0.0f);
	dFloat32 distance = m_normal.DotProduct(m_localPoly[0] - p1).GetScalar();
	if (distance >= dFloat32(0.0f))
	{
		//contactSolver.m_separationDistance = dFloat32 (0.0f);
		return 0;
	}

	ndVector boxSize;
	ndVector boxOrigin;
	hull->CalculateObb(boxOrigin, boxSize);
	dAssert(boxOrigin.m_w == dFloat32(0.0f));
	boxOrigin += ndVector::m_wOne;

	bool inside = true;
	dInt32 i0 = m_count - 1;
	for (dInt32 i = 0; i < m_count; i++)
	{
		ndVector e(m_localPoly[i] - m_localPoly[i0]);
		ndVector edgeBoundaryNormal(m_normal.CrossProduct(e));
		dAssert(edgeBoundaryNormal.m_w == dFloat32(0.0f));
		ndPlane plane(edgeBoundaryNormal, -m_localPoly[i0].DotProduct(edgeBoundaryNormal).GetScalar());
		plane = hullMatrix.UntransformPlane(plane);

		dFloat32 supportDist = boxSize.DotProduct(plane.Abs()).GetScalar();
		dFloat32 centerDist = plane.DotProduct(boxOrigin).GetScalar();

		if ((centerDist + supportDist) < dFloat32(0.0f))
		{
			//contactSolver.m_separationDistance = dFloat32(0.0f);
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
		ndVector boxP0;
		ndVector boxP1;
		hull->CalculateAabb(hullMatrix, boxP0, boxP1);
		ndVector origin(ndVector::m_half * (boxP1 + boxP1));

		if (!BeamClipping(origin, convexSphapeUmbra, parentMesh))
		{
			return 0;
		}
		m_faceClipSize = hull->GetShape()->GetBoxMaxRadius();
	}

	dInt32 count = 0;
	const dInt64 hullId = hull->GetUserDataID();
	if (inside & !contactSolver.m_intersectionTestOnly)
	{
		contactSolver.m_separationDistance = -penetration;
		contactSolver.m_separatingVector = m_normal;
		contactSolver.m_closestPoint0 = p0;
		contactSolver.m_closestPoint1 = p0 + m_normal.Scale(penetration);

		penetration = dMax(dFloat32(0.0f), penetration);
		dAssert(penetration >= dFloat32(0.0f));
		ndVector contactPoints[128];
		ndVector point(pointInHull + normalInHull.Scale(penetration - D_PENETRATION_TOL));

		count = hull->CalculatePlaneIntersection(normalInHull.Scale(dFloat32(-1.0f)), point, contactPoints);
		ndVector step(normalInHull.Scale((contactSolver.m_skinThickness - penetration) * dFloat32(0.5f)));

		ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
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
		count = contactSolver.ConvexToConvexContactsDiscrete();
		dAssert(contactSolver.m_intersectionTestOnly || (count >= 0));
		if (count >= 1)
		{
			ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
			for (dInt32 i = 0; i < count; i++)
			{
				contactsOut[i].m_shapeId0 = hullId;
				contactsOut[i].m_shapeId1 = m_faceId;
			}
		}
	}

	return count;
}

dInt32 ndShapeConvexPolygon::CalculateContactToConvexHullContinue(const ndShapeInstance* const parentMesh, ndContactSolver& contactSolver)
{
	dAssert(contactSolver.m_instance0.GetShape()->GetAsShapeConvex());
	dAssert(contactSolver.m_instance1.GetShape()->GetAsShapeAsConvexPolygon());
	dAssert(contactSolver.m_instance1.GetGlobalMatrix().TestIdentity());
	dAssert(this == contactSolver.m_instance1.GetShape());
	dAssert(m_count);
	dAssert(m_count < dInt32(sizeof(m_localPoly) / sizeof(m_localPoly[0])));
	dAssert(contactSolver.m_instance1.GetGlobalMatrix().TestIdentity());

	const ndBodyKinematic* const body0 = contactSolver.m_contact->m_body0;
	const ndBodyKinematic* const body1 = contactSolver.m_contact->m_body1;

	ndVector relativeVelocity(body0->GetVelocity() - body1->GetVelocity());
	dAssert(relativeVelocity.m_w == dFloat32(0.0f));
	if (relativeVelocity.DotProduct(relativeVelocity).GetScalar() < dFloat32(1.0e-4f)) 
	{
		return 0;
	}

	dFloat32 den = m_normal.DotProduct(relativeVelocity).GetScalar();
	if (den > dFloat32(-1.0e-10f)) 
	{
		return 0;
	}

	ndMatrix polygonMatrix;
	ndVector right(m_localPoly[1] - m_localPoly[0]);
	polygonMatrix[0] = right.Normalize();
	polygonMatrix[1] = m_normal;
	polygonMatrix[2] = polygonMatrix[0].CrossProduct(m_normal);
	polygonMatrix[3] = m_localPoly[0];
	polygonMatrix[3].m_w = dFloat32(1.0f);
	dAssert(polygonMatrix.TestOrthogonal());
	
	ndVector polyBoxP0(dFloat32(1.0e15f));
	ndVector polyBoxP1(dFloat32(-1.0e15f));
	for (dInt32 i = 0; i < m_count; i++) 
	{
		const ndVector point(polygonMatrix.UntransformVector(m_localPoly[i]));
		polyBoxP0 = polyBoxP0.GetMin(point);
		polyBoxP1 = polyBoxP1.GetMax(point);
	}
	
	ndVector hullBoxP0;
	ndVector hullBoxP1;
	ndMatrix hullMatrix(contactSolver.m_instance0.m_globalMatrix * polygonMatrix.Inverse());
	contactSolver.m_instance0.CalculateAabb(hullMatrix, hullBoxP0, hullBoxP1);
	ndVector minBox(polyBoxP0 - hullBoxP1);
	ndVector maxBox(polyBoxP1 - hullBoxP0);
	
	ndVector relStep(relativeVelocity.Scale(dMax(contactSolver.m_timestep, dFloat32(1.0e-12f))));
	ndFastRay ray(ndVector::m_zero, polygonMatrix.UnrotateVector(relStep));
	dFloat32 distance = ray.BoxIntersect(minBox, maxBox);
	
	dFloat32 relStepSpeed = m_normal.DotProduct(relStep).GetScalar();
	dInt32 count = 0;
	if ((distance < dFloat32(1.0f)) && (dAbs(relStepSpeed) > dFloat32(1.0e-12f))) 
	{
		bool inside = false;
		dAssert(m_normal.DotProduct(relStep).GetScalar() == relStepSpeed);
		dFloat32 invSpeed = dFloat32(1.0f) / relStepSpeed;
		ndVector sphOrigin(polygonMatrix.TransformVector((hullBoxP1 + hullBoxP0) * ndVector::m_half));
		ndVector pointInPlane(sphOrigin - relStep.Scale(m_normal.DotProduct(sphOrigin - m_localPoly[0]).GetScalar() * invSpeed));
	
		ndVector sphRadius(ndVector::m_half * (hullBoxP1 - hullBoxP0));
		dFloat32 radius = dSqrt(sphRadius.DotProduct(sphRadius).GetScalar());
		ndVector planeMinkStep(m_normal.Scale(radius));
		sphOrigin -= planeMinkStep;
		dAssert(m_normal.DotProduct(relStep).GetScalar() == relStepSpeed);
		ndVector supportPoint(sphOrigin - relStep.Scale(m_normal.DotProduct(sphOrigin - m_localPoly[0]).GetScalar() * invSpeed));
	
		supportPoint -= pointInPlane;
		dAssert(supportPoint.m_w == dFloat32(0.0f));
		radius = dMax(dSqrt(supportPoint.DotProduct(supportPoint).GetScalar()), radius);
	
		inside = true;
		dInt32 i0 = m_count - 1;
		for (dInt32 i = 0; i < m_count; i++) 
		{
			const ndVector e(m_localPoly[i] - m_localPoly[i0]);
			const ndVector n((e.CrossProduct(m_normal) & ndVector::m_triplexMask).Normalize());
			dFloat32 dist1 = n.DotProduct(pointInPlane - m_localPoly[i0]).GetScalar();
	
			if (dist1 > radius) 
			{
				return 0;
			}
			inside &= (dist1 <= dFloat32(0.0f));
			i0 = i;
		}
	
		dFloat32 convexSphapeUmbra = dMax(contactSolver.m_instance0.GetUmbraClipSize(), radius);
		if (m_faceClipSize > convexSphapeUmbra) 
		{
			BeamClipping(pointInPlane, convexSphapeUmbra, parentMesh);
			m_faceClipSize = contactSolver.m_instance0.GetShape()->GetBoxMaxRadius();
		}
	
		const dInt64 hullId = contactSolver.m_instance0.GetUserDataID();
		if (inside & !contactSolver.m_intersectionTestOnly) 
		{
			const ndMatrix& matrixInstance0 = contactSolver.m_instance0.m_globalMatrix;
			const ndVector normalInHull(matrixInstance0.UnrotateVector(m_normal.Scale(dFloat32(-1.0f))));
			ndVector pointInHull(contactSolver.m_instance0.SupportVertex(normalInHull));
			const ndVector p0(matrixInstance0.TransformVector(pointInHull));
	
			dFloat32 timetoImpact = dFloat32(0.0f);
			dAssert(m_normal.m_w == dFloat32(0.0f));
			dFloat32 penetration = m_normal.DotProduct(m_localPoly[0] - p0).GetScalar() + contactSolver.m_skinThickness;
			if (penetration < dFloat32(0.0f)) 
			{
				timetoImpact = penetration / relativeVelocity.DotProduct(m_normal).GetScalar();
				dAssert(timetoImpact >= dFloat32(0.0f));
			}
	
			if (timetoImpact <= contactSolver.m_timestep) 
			{
				ndVector contactPoints[64];
				contactSolver.m_timestep = timetoImpact;
				contactSolver.m_separatingVector = m_normal;
				contactSolver.m_closestPoint0 = p0;
				contactSolver.m_closestPoint1 = p0 + m_normal.Scale(penetration);
	
				if (!contactSolver.m_intersectionTestOnly) 
				{
					pointInHull -= normalInHull.Scale(D_PENETRATION_TOL);
					count = contactSolver.m_instance0.CalculatePlaneIntersection(normalInHull, pointInHull, contactPoints);
	
					ndVector step(relativeVelocity.Scale(timetoImpact));
					penetration = dMax(penetration, dFloat32(0.0f));
					ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
					for (dInt32 i = 0; i < count; i++) 
					{
						contactsOut[i].m_point = matrixInstance0.TransformVector(contactPoints[i]) + step;
						contactsOut[i].m_normal = m_normal;
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
						contactsOut[i].m_penetration = penetration;
					}
				}
			}
		}
		else 
		{
			m_vertexCount = dUnsigned16(m_count);
			count = contactSolver.ConvexToConvexContactsContinue();
			if (count >= 1) 
			{
				ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
				for (dInt32 i = 0; i < count; i++) 
				{
					contactsOut[i].m_shapeId0 = hullId;
					contactsOut[i].m_shapeId1 = m_faceId;
				}
			}
		}
	}
	return count;
}