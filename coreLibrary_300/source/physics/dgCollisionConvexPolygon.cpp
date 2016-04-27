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
#include "dgContactSolver.h"
#include "dgCollisionInstance.h"
#include "dgCollisionConvexPolygon.h"



#define DG_CONVEX_POLYGON_CRC 0x12341234

dgCollisionConvexPolygon::dgCollisionConvexPolygon (dgMemoryAllocator* const allocator)
	:dgCollisionConvex (allocator, DG_CONVEX_POLYGON_CRC, m_polygonCollision)
	,m_count(0)
	,m_paddedCount(0)
	,m_stride(0)
	,m_faceNormalIndex(0)
	,m_closestFeatureType(-1)
	,m_faceClipSize(0) 
	,m_vertex(NULL)
	,m_vertexIndex(NULL)
	,m_adjacentFaceEdgeNormalIndex(NULL)
{
	m_rtti |= dgCollisionConvexPolygon_RTTI;
}

dgCollisionConvexPolygon::~dgCollisionConvexPolygon ()
{

}


dgInt32 dgCollisionConvexPolygon::CalculateSignature () const
{
	return DG_CONVEX_POLYGON_CRC;
}

void dgCollisionConvexPolygon::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}

void dgCollisionConvexPolygon::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
}


dgFloat32 dgCollisionConvexPolygon::RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* userData, OnRayPrecastAction preFilter) const
{
	dgAssert (0);
	return dgFloat32 (1.2f);
}


dgFloat32 dgCollisionConvexPolygon::GetVolume () const
{
	dgAssert (0);
	return dgFloat32 (0.0f); 
}

dgFloat32 dgCollisionConvexPolygon::GetBoxMinRadius () const
{
	return m_faceClipSize;
}

dgFloat32 dgCollisionConvexPolygon::GetBoxMaxRadius () const
{
	//return m_faceClipSize;  
	return GetBoxMinRadius ();
}


dgVector dgCollisionConvexPolygon::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf (dir % dir - 1.0f) < dgFloat32 (1.0e-2f));
	dgInt32 index = 0;
	dgFloat32 val = m_localPoly[0] % dir;
	for (dgInt32 i = 1; i < m_count; i ++) {
		dgFloat32 val1 = m_localPoly[i] % dir;
		if (val1 > val) {
			val = val1; 
			index = i;
		}
	}

	dgAssert (vertexIndex);
	*vertexIndex = index;
	return m_localPoly[index];
}


void dgCollisionConvexPolygon::BeamClipping (const dgVector& origin, dgFloat32 dist)
{
	dgPlane planes[4];
	dgVector points[sizeof (m_localPoly) / sizeof (m_localPoly[0]) + 8];

	dgClippedFaceEdge clippedFace [2 * sizeof (m_localPoly) / sizeof (m_localPoly[0]) + 8];

	dgVector dir (m_localPoly[1] - m_localPoly[0]);
	dgAssert (dir.m_w == dgFloat32 (0.0f));
	dgAssert ((dir % dir) > dgFloat32 (1.0e-8f));
	dir = dir.CompProduct4 (dir.InvMagSqrt());

	dgFloat32 distH = origin.DotProduct4(dir).GetScalar();
	planes[0] = dgPlane (dir, dist - distH);
	planes[2] = dgPlane (dir.CompProduct4 (dgVector::m_negOne), dist + distH);

	dir = m_normal * dir;
	dgFloat32 distV = origin.DotProduct4(dir).GetScalar();
	planes[1] = dgPlane (dir, dist - distV);
	planes[3] = dgPlane (dir.CompProduct4 (dgVector::m_negOne), dist + distV);

	for (dgInt32 i = 0; i < m_count; i ++) {
		dgInt32 j = i << 1;
		dgAssert (j < sizeof (clippedFace) / sizeof (clippedFace[0]));

		points[i] = m_localPoly[i];

		clippedFace[j + 0].m_twin = &clippedFace[j + 1];
		clippedFace[j + 0].m_next = &clippedFace[j + 2];
		clippedFace[j + 0].m_incidentVertex = i;
		clippedFace[j + 0].m_incidentNormal = m_adjacentFaceEdgeNormalIndex[i];

		clippedFace[j + 1].m_twin = &clippedFace[j + 0];
		clippedFace[j + 1].m_next = &clippedFace[j - 2];
		clippedFace[j + 1].m_incidentVertex = i + 1;
		clippedFace[j + 1].m_incidentNormal = -1;
	}

	clippedFace[1].m_next = &clippedFace[m_count * 2 - 2 + 1];
	dgAssert ((m_count * 2 - 2) >= 0);
	clippedFace[m_count * 2 - 2].m_next = &clippedFace[0];
	clippedFace[m_count * 2 - 2 + 1].m_incidentVertex = 0;

	const dgFloat32 tol = dgFloat32 (1.0e-5f);
	dgInt32 edgeCount = m_count * 2;
	dgInt32 indexCount = m_count;
	dgClippedFaceEdge* first = &clippedFace[0];
	for (dgInt32 i = 0; i < 4; i ++) {
		const dgPlane& plane = planes[i];

		dgInt32 conectCount = 0;
		dgClippedFaceEdge* connect[2];
		dgClippedFaceEdge* ptr = first;
		dgClippedFaceEdge* newFirst = first;
		dgFloat32 test0 = plane.Evalue(points[ptr->m_incidentVertex]);
		do {
			dgFloat32 test1 = plane.Evalue(points[ptr->m_next->m_incidentVertex]);

			if (test0 > tol) {
				if (test1 <= - tol) {
					const dgVector& p0 = points[ptr->m_incidentVertex];
					const dgVector& p1 = points[ptr->m_next->m_incidentVertex];
					dgVector dp (p1 - p0); 
					points[indexCount] = p0 - dp.Scale4 (test0 / dp.DotProduct4(plane).GetScalar());

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
					conectCount ++;
					indexCount ++;
					edgeCount += 2;
					ptr = newEdge;
				}
			} else {
				if (test1 > tol) {
					newFirst = ptr->m_next;

					const dgVector& p0 = points[ptr->m_incidentVertex];
					const dgVector& p1 = points[ptr->m_next->m_incidentVertex];
					dgVector dp (p1 - p0); 
					points[indexCount] = p0 - dp.Scale4 (test0 / dp.DotProduct4(plane).GetScalar());

					dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
					newEdge->m_twin = newEdge + 1;
					newEdge->m_twin->m_twin = newEdge;

					newEdge->m_twin->m_incidentNormal = ptr->m_incidentNormal;;
					newEdge->m_incidentNormal = ptr->m_incidentNormal;

					newEdge->m_incidentVertex = indexCount;
					newEdge->m_twin->m_incidentVertex = ptr->m_next->m_incidentVertex;
					ptr->m_twin->m_incidentVertex = indexCount;

					newEdge->m_next = ptr->m_next;
					ptr->m_next->m_twin->m_next = newEdge->m_twin;
					newEdge->m_twin->m_next = ptr->m_twin;
					ptr->m_next = newEdge;

					connect[conectCount] = ptr;
					conectCount ++;
					indexCount ++;
					edgeCount += 2;

					ptr = newEdge;
				}
			}


			test0 = test1;
			ptr = ptr->m_next;
		} while (ptr != first);

		if(conectCount > 1) {
			first = newFirst;
			dgAssert (conectCount == 2);

			dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
			newEdge->m_twin = newEdge + 1;
			newEdge->m_twin->m_twin = newEdge;

			newEdge->m_incidentNormal = m_faceNormalIndex;;
			newEdge->m_incidentVertex = connect[0]->m_next->m_incidentVertex;
			newEdge->m_twin->m_next = connect[0]->m_next;
			connect[0]->m_next = newEdge;

			newEdge->m_twin->m_incidentNormal = m_faceNormalIndex;;
			newEdge->m_twin->m_incidentVertex = connect[1]->m_next->m_incidentVertex;
			newEdge->m_next = connect[1]->m_next;
			connect[1]->m_next = newEdge->m_twin;

			edgeCount += 2;
		}
	}

	
	dgClippedFaceEdge* ptr = first;
	do {
		dgVector dist (points[ptr->m_next->m_incidentVertex] - points[ptr->m_incidentVertex]);
		dgFloat32 error = dist % dist;
		if (error < dgFloat32 (1.0e-6f)) {
			ptr->m_next = ptr->m_next->m_next;
			first = ptr;
		}
		ptr = ptr->m_next;
	} while (ptr != first);

	dgInt32 count = 0;
	m_adjacentFaceEdgeNormalIndex = &m_clippEdgeNormal[0];
	do {
		m_clippEdgeNormal[count] = ptr->m_incidentNormal;
		m_localPoly[count] = points[ptr->m_incidentVertex];
		count ++;
		ptr = ptr->m_next;
	} while (ptr != first);

	m_count = count;
}


void dgCollisionConvexPolygon::SetFeatureHit (dgInt32 featureCount, const dgInt32* const index)
{
	dgInt32 copy[3];
	copy[0] = index[0];
	copy[1] = index[1];
	copy[2] = index[2];
	switch (featureCount)
	{
		case 3:
		{
			if ((copy[0] != copy[1]) && (copy[0] != copy[2]) && (copy[1] != copy[2])) {
				m_closestFeatureType = 3;
				m_closestFeatureStartIndex = 0;
				break;
			} else {
				if (copy[0] == copy[1]) {
					copy[0] = copy[2];
				}
			}
			featureCount = 2;
		}

		case 2:
		{
			if (copy[0] != copy[1]) {
				dgInt32 n1 = (copy[0] + 1) % m_count;
				if (n1 == copy[1]) {
					m_closestFeatureType = 2;
					m_closestFeatureStartIndex = copy[0];
				} else {
					n1 = (copy[1] + 1) % m_count;
					if (n1 == copy[0]) {
						m_closestFeatureType = 2;
						m_closestFeatureStartIndex = copy[1];
					} else {
						m_closestFeatureType = 3;
					}
				}

				break;
			} 
		}

		case 1:
		{
			m_closestFeatureType = 1;
			m_closestFeatureStartIndex = copy[0];
			break;
		}
	}
}

dgInt32 dgCollisionConvexPolygon::CalculatePlaneIntersection (const dgVector& normalIn, const dgVector& origin, dgVector* const contactsOut) const
{
	dgVector normal(normalIn);
	dgInt32 count = 0;
	dgFloat32 maxDist = dgFloat32 (1.0f);
	dgFloat32 projectFactor = m_normal % normal;
	if (projectFactor < dgFloat32 (0.0f)) {
		projectFactor *= dgFloat32 (-1.0f);
		normal = normal.Scale3 (dgFloat32 (-1.0f));
	}

	if (projectFactor > dgFloat32 (0.9999f)) {
		for (dgInt32 i = 0; i < m_count; i ++) {
			contactsOut[count] = m_localPoly[i];
			count ++;
		}

		#ifdef _DEBUG
			dgInt32 j = count - 1;
			for (dgInt32 i = 0; i < count; i ++) {
				dgVector error (contactsOut[i] - contactsOut[j]);
				dgAssert ((error % error) > dgFloat32 (1.0e-20f));
				j = i;
			}
		#endif

	} else if (projectFactor > dgFloat32 (0.1736f)) {
		maxDist = dgFloat32 (0.0f);
		dgPlane plane (normal, - (normal % origin));

		dgVector p0 (m_localPoly[m_count - 1]);
		dgFloat32 side0 = plane.Evalue (p0);
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgVector p1 (m_localPoly[i]);
			dgFloat32 side1 = plane.Evalue (p1);

			if (side0 > dgFloat32 (0.0f)) {
				maxDist = dgMax (maxDist, side0);
				contactsOut[count] = p0 - plane.Scale3 (side0);
				count ++;
				if (count > 1) {
					dgVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dgFloat32 error = edgeSegment % edgeSegment;
					if (error < dgFloat32 (1.0e-8f)) {
						count --;
					}
				}

				if (side1 <= dgFloat32 (0.0f)) {
					dgVector dp (p1 - p0);
					dgFloat32 t = plane % dp;
					dgAssert (dgAbsf (t) >= dgFloat32 (0.0f));
					if (dgAbsf (t) < dgFloat32 (1.0e-8f)) {
						t = dgSign(t) * dgFloat32 (1.0e-8f);	
					}
					contactsOut[count] = p0 - dp.Scale3 (side0 / t);
					count ++;
					if (count > 1) {
						dgVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
						dgFloat32 error = edgeSegment % edgeSegment;
						if (error < dgFloat32 (1.0e-8f)) {
							count --;
						}
					}
				} 
			} else if (side1 > dgFloat32 (0.0f)) {
				dgVector dp (p1 - p0);
				dgFloat32 t = plane % dp;
				dgAssert (dgAbsf (t) >= dgFloat32 (0.0f));
				if (dgAbsf (t) < dgFloat32 (1.0e-8f)) {
					t = dgSign(t) * dgFloat32 (1.0e-8f);	
				}
				contactsOut[count] = p0 - dp.Scale3 (side0 / t);
				count ++;
				if (count > 1) {
					dgVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dgFloat32 error = edgeSegment % edgeSegment;
					if (error < dgFloat32 (1.0e-8f)) {
						count --;
					}
				}
			}

			side0 = side1;
			p0 = p1;
		}
	} else {
		maxDist = dgFloat32 (1.0e10f);
		dgPlane plane (normal, - (normal % origin));

		dgVector p0 (m_localPoly[m_count - 1]);
		dgFloat32 side0 = plane.Evalue (p0);
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgVector p1 (m_localPoly[i]);
			dgFloat32 side1 = plane.Evalue (p1);

			if ((side0 * side1) < dgFloat32 (0.0f)) {
				dgVector dp (p1 - p0);
				dgFloat32 t = plane % dp;
				dgAssert (dgAbsf (t) >= dgFloat32 (0.0f));
				if (dgAbsf (t) < dgFloat32 (1.0e-8f)) {
					t = dgSign(t) * dgFloat32 (1.0e-8f);	
				}
				contactsOut[count] = p0 - dp.Scale3 (side0 / t);
				count ++;
				if (count > 1) {
					dgVector edgeSegment (contactsOut[count - 1] - contactsOut[count - 2]);
					dgFloat32 error = edgeSegment % edgeSegment;
					if (error < dgFloat32 (1.0e-8f)) {
						count --;
					}
				}
			}
			side0 = side1;
			p0 = p1;
		}
	}


	if (count > 1) {
		if (maxDist < dgFloat32 (1.0e-3f)) {
			dgVector maxPoint (contactsOut[0]);
			dgVector minPoint (contactsOut[0]);
			dgVector lineDir (m_normal * normal);

			dgFloat32 proj = contactsOut[0] % lineDir;
			dgFloat32 maxProjection = proj;
			dgFloat32 minProjection = proj;
			for (dgInt32 i = 1; i < count; i ++) {
				proj = contactsOut[i] % lineDir;
				if (proj > maxProjection) {
					maxProjection = proj;
					maxPoint = contactsOut[i];
				}
				if (proj < minProjection) {
					minProjection = proj;
					minPoint = contactsOut[i];
				}
			}	

			contactsOut[0] = maxPoint;
			contactsOut[1] = minPoint;
			count = 2;
		}


		dgVector error (contactsOut[count - 1] - contactsOut[0]);
		if ((error % error) < dgFloat32 (1.0e-8f)) {
			count --;
		}
	}

	#ifdef _DEBUG
		if (count > 1) {
			dgInt32 j = count - 1;
			for (dgInt32 i = 0; i < count; i ++) {
				dgVector error (contactsOut[i] - contactsOut[j]);
				dgAssert ((error % error) > dgFloat32 (1.0e-20f));
				j = i;
			}

			if (count >= 3) {
				dgVector n (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				dgVector e0 (contactsOut[1] - contactsOut[0]);
				for (dgInt32 i = 2; i < count; i ++) {
					dgVector e1 (contactsOut[i] - contactsOut[0]);
					n += e0 * e1;
					e0 = e1;
				} 
				n = n.Scale3 (dgRsqrt(n % n));
				dgFloat32 val = n % normal;
				dgAssert (val > dgFloat32 (0.9f));
			}
		}
	#endif
	return count;
}


dgVector dgCollisionConvexPolygon::CalculateGlobalNormal (const dgCollisionInstance* const parentMesh, const dgVector& localNormal) const
{
	const dgVector& invScale = parentMesh->GetInvScale();
	const dgMatrix& globalMatrix = parentMesh->m_globalMatrix;
	const dgMatrix& aligmentMatrix = parentMesh->m_aligmentMatrix;

	dgVector normal (aligmentMatrix.RotateVector(localNormal));
	normal = normal.CompProduct4(invScale);
	dgAssert(normal.m_w == dgFloat32(0.0f));
	normal = normal.CompProduct4(normal.DotProduct4(normal).InvSqrt());
	dgAssert (dgAbsf(normal % normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
	return globalMatrix.RotateVector(normal);
}

dgInt32 dgCollisionConvexPolygon::CalculateContactToConvexHullDescrete(const dgWorld* const world, const dgCollisionInstance* const parentMesh, dgCollisionParamProxy& proxy)
{
	dgInt32 count = 0;

	dgAssert(proxy.m_instance0->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert(proxy.m_instance1->IsType(dgCollision::dgCollisionConvexPolygon_RTTI));
	dgAssert (proxy.m_instance1->GetGlobalMatrix().TestIdentity());

	const dgCollisionInstance* const polygonInstance = proxy.m_instance1;
	dgAssert(this == polygonInstance->GetChildShape());
	dgAssert(m_count);
	dgAssert(m_count < dgInt32(sizeof (m_localPoly) / sizeof (m_localPoly[0])));

	const dgMatrix& hullMatrix = proxy.m_instance0->m_globalMatrix;
	dgContact* const contactJoint = proxy.m_contactJoint;
	const dgCollisionInstance* const hull = proxy.m_instance0;

	dgVector normalInHull(hullMatrix.UnrotateVector(m_normal));
	dgVector pointInHull(hull->SupportVertex(normalInHull.Scale4(dgFloat32(-1.0f)), NULL));
	dgVector p0(hullMatrix.TransformVector(pointInHull));

	dgFloat32 penetration = (m_localPoly[0] - p0) % m_normal + proxy.m_skinThickness;
	if (penetration < dgFloat32(0.0f)) {
		return 0;
	}

	dgVector p1(hullMatrix.TransformVector(hull->SupportVertex(normalInHull, NULL)));
	contactJoint->m_closestDistance = dgFloat32(0.0f);
	dgFloat32 distance = (m_localPoly[0] - p1) % m_normal;
	if (distance >= dgFloat32(0.0f)) {
		return 0;
	}

	dgVector boxSize (hull->GetBoxSize() & dgVector::m_triplexMask);
	dgVector boxOrigin ((hull->GetBoxOrigin() & dgVector::m_triplexMask) + dgVector::m_wOne);

	bool inside = true;
	dgInt32 i0 = m_count - 1;
	for (dgInt32 i = 0; i < m_count; i++) {
		dgVector e(m_localPoly[i] - m_localPoly[i0]);
		dgVector edgeBoundaryNormal(m_normal * e);
		dgPlane plane(edgeBoundaryNormal, - m_localPoly[i0].DotProduct4 (edgeBoundaryNormal).GetScalar());
		plane = hullMatrix.TransformPlane(plane);

		dgFloat32 supportDist = boxSize.DotProduct4 (plane.Abs()).GetScalar();
		dgFloat32 centerDist = plane.DotProduct4 (boxOrigin).GetScalar();

		if ((centerDist + supportDist) < dgFloat32(0.0f)) {
			return 0;
		}

		if ((centerDist - supportDist) < dgFloat32(0.0f)) {
			inside = false;
			break;
		}
		i0 = i;
	}

//inside = false;
	dgFloat32 convexSphapeUmbra = hull->GetUmbraClipSize();
	if (m_faceClipSize > convexSphapeUmbra) {
		BeamClipping(dgVector(dgFloat32(0.0f)), convexSphapeUmbra);
		m_faceClipSize = hull->m_childShape->GetBoxMaxRadius();
	}

	const dgInt32 hullId = hull->GetUserDataID();
	if (inside & !proxy.m_intersectionTestOnly) {
		dgAssert(penetration >= dgFloat32(0.0f));
		dgVector contactPoints[64];

		dgAssert(penetration >= 0.0f);
		dgVector point(pointInHull + normalInHull.Scale4(penetration + DG_ROBUST_PLANE_CLIP));

		count = hull->CalculatePlaneIntersection(normalInHull.Scale4(dgFloat32(-1.0f)), point, contactPoints);
		dgVector step(normalInHull.Scale4((proxy.m_skinThickness - penetration) * dgFloat32(0.5f)));

		dgContactPoint* const contactsOut = proxy.m_contacts;
		dgAssert(contactsOut);
		for (dgInt32 i = 0; i < count; i++) {
			contactsOut[i].m_point = hullMatrix.TransformVector(contactPoints[i] + step);
			contactsOut[i].m_normal = m_normal;
			contactsOut[i].m_shapeId0 = hullId;
			contactsOut[i].m_shapeId1 = m_faceId;
			contactsOut[i].m_penetration = penetration;
		}
	} else {
		m_vertexCount = dgUnsigned16 (m_count);
		count = world->CalculateConvexToConvexContacts(proxy);
		dgAssert(proxy.m_intersectionTestOnly || (count >= 0));

		if (count >= 1) {
			dgContactPoint* const contactsOut = proxy.m_contacts;
			if (m_closestFeatureType == 3) {
				for (dgInt32 i = 0; i < count; i++) {
					//contactsOut[i].m_userId = m_faceId;
					contactsOut[i].m_shapeId0 = hullId;
					contactsOut[i].m_shapeId1 = m_faceId;
				}
			} else {
				dgVector normal (contactsOut[0].m_normal);

				if (normal.DotProduct4(m_normal).GetScalar() < dgFloat32(0.9995f)) {
					dgInt32 index = m_adjacentFaceEdgeNormalIndex[m_closestFeatureStartIndex];
					dgVector adjacentNormal (CalculateGlobalNormal (parentMesh, dgVector(&m_vertex[index * m_stride])));
					if ((m_normal.DotProduct4(adjacentNormal).GetScalar() > dgFloat32(0.9995f))) {
						normal = adjacentNormal;
					} else {
						dgVector dir0(adjacentNormal * m_normal);
						dgVector dir1(adjacentNormal * normal);
						dgFloat32 projection = dir0.DotProduct4(dir1).GetScalar();
						if (projection <= dgFloat32(0.0f)) {
							normal = adjacentNormal;
						}
					}
					normal = polygonInstance->m_globalMatrix.RotateVector(normal);

					for (dgInt32 i = 0; i < count; i++) {
						contactsOut[i].m_normal = normal;
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
					}
				} else {
					for (dgInt32 i = 0; i < count; i++) {
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
					}
				}
			}
		}
	}
	return count;
}

dgInt32 dgCollisionConvexPolygon::CalculateContactToConvexHullContinue(const dgWorld* const world, const dgCollisionInstance* const parentMesh, dgCollisionParamProxy& proxy)
{
	dgAssert(proxy.m_instance0->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert(proxy.m_instance1->IsType(dgCollision::dgCollisionConvexPolygon_RTTI));

	dgAssert(this == proxy.m_instance1->GetChildShape());
	dgAssert(m_count);
	dgAssert(m_count < dgInt32(sizeof (m_localPoly) / sizeof (m_localPoly[0])));

	const dgBody* const body0 = proxy.m_body0;
	const dgBody* const body1 = proxy.m_body1;

	dgAssert (proxy.m_instance1->GetGlobalMatrix().TestIdentity());

	dgVector relativeVelocity (body0->m_veloc - body1->m_veloc);
	if (m_normal.DotProduct4(relativeVelocity).GetScalar() >= 0.0f) {
		return 0;
	}
	dgFloat32 den = dgFloat32 (1.0f) / (relativeVelocity % m_normal);
	if (den > dgFloat32 (1.0e-5f)) {
		// this can actually happens
		dgAssert(0);
		return 0;
	}

	dgContact* const contactJoint = proxy.m_contactJoint;
	contactJoint->m_closestDistance = dgFloat32(1.0e10f);

	dgMatrix polygonMatrix;
	dgVector right (m_localPoly[1] - m_localPoly[0]);
	polygonMatrix[0] = right.CompProduct4(right.InvMagSqrt());
	polygonMatrix[1] = m_normal;
	polygonMatrix[2] = polygonMatrix[0] * m_normal;
	polygonMatrix[3] = dgVector::m_wOne;
	dgAssert (polygonMatrix.TestOrthogonal());

	dgVector polyBoxP0(dgFloat32(1.0e15f));
	dgVector polyBoxP1(dgFloat32(-1.0e15f));
	for (dgInt32 i = 0; i < m_count; i++) {
		dgVector point (polygonMatrix.UnrotateVector(m_localPoly[i]));
		polyBoxP0 = polyBoxP0.GetMin(point);
		polyBoxP1 = polyBoxP1.GetMax(point);
	}

	dgVector hullBoxP0;
	dgVector hullBoxP1;
	dgMatrix hullMatrix (polygonMatrix * proxy.m_instance0->m_globalMatrix);
	proxy.m_instance0->CalcAABB(hullMatrix, hullBoxP0, hullBoxP1);
	dgVector minBox(polyBoxP0 - hullBoxP1);
	dgVector maxBox(polyBoxP1 - hullBoxP0);
	dgVector veloc (polygonMatrix.UnrotateVector (relativeVelocity));
	dgFastRayTest ray(dgVector(dgFloat32(0.0f)), veloc);
 	dgFloat32 distance = ray.BoxIntersect(minBox, maxBox);

	dgInt32 count = 0;
	if (distance < dgFloat32(1.0f)) {
		bool inside = false;

		dgVector boxSize((hullBoxP1 - hullBoxP0).CompProduct4(dgVector::m_half));
		dgVector sphereMag2 (boxSize.DotProduct4(boxSize));
		boxSize = sphereMag2.Sqrt();

		dgVector pointInPlane (polygonMatrix.RotateVector(hullBoxP1 + hullBoxP0).CompProduct4(dgVector::m_half));
		dgFloat32 distToPlane = (m_localPoly[0] - pointInPlane) % m_normal;

		dgFloat32 timeToPlane0 = (distToPlane + boxSize.GetScalar()) * den;
		dgFloat32 timeToPlane1 = (distToPlane - boxSize.GetScalar()) * den;

		dgVector boxOrigin0 (pointInPlane + relativeVelocity.Scale4(timeToPlane0));
		dgVector boxOrigin1 (pointInPlane + relativeVelocity.Scale4(timeToPlane1));
		dgVector boxOrigin ((boxOrigin0 + boxOrigin1).CompProduct4(dgVector::m_half)); 
		dgVector boxProjectSize (((boxOrigin0 - boxOrigin1).CompProduct4(dgVector::m_half))); 
		sphereMag2 = boxProjectSize.DotProduct4(boxProjectSize);
		boxSize = sphereMag2.Sqrt();

		dgAssert (boxOrigin.m_w == 0.0f);
		boxOrigin = boxOrigin | dgVector::m_wOne;
		
		if (!proxy.m_intersectionTestOnly) {
			inside = true;
			dgInt32 i0 = m_count - 1;

			for (dgInt32 i = 0; i < m_count; i++) {
				dgVector e(m_localPoly[i] - m_localPoly[i0]);
				dgVector n(m_normal * e & dgVector::m_triplexMask);
				dgFloat32 param = dgSqrt (sphereMag2.GetScalar() / (n.DotProduct4(n)).GetScalar());
				dgPlane plane(n, -(m_localPoly[i0] % n));

				dgVector p0 (boxOrigin + n.Scale4 (param));
				dgVector p1 (boxOrigin - n.Scale4 (param));

				dgFloat32 size0 = (plane.DotProduct4 (p0)).GetScalar();
				dgFloat32 size1 = (plane.DotProduct4 (p1)).GetScalar();

				if ((size0 < 0.0f) && (size1 < 0.0f)) {
					return 0;
				}

				if ((size0 * size1) < 0.0f) {
					inside = false;
					break;
				}
				i0 = i;
			}
		}

		dgFloat32 convexSphapeUmbra = dgMax (proxy.m_instance0->GetUmbraClipSize(), boxSize.GetScalar());
		if (m_faceClipSize > convexSphapeUmbra) {
			BeamClipping(boxOrigin, convexSphapeUmbra);
			m_faceClipSize = proxy.m_instance0->m_childShape->GetBoxMaxRadius();
		}

		const dgInt32 hullId = proxy.m_instance0->GetUserDataID();
		if (inside & !proxy.m_intersectionTestOnly) {
			const dgMatrix& matrixInstance0 = proxy.m_instance0->m_globalMatrix;
			dgVector normalInHull(matrixInstance0.UnrotateVector(m_normal.Scale4(dgFloat32(-1.0f))));
			dgVector pointInHull(proxy.m_instance0->SupportVertex(normalInHull, NULL));
			dgVector p0 (matrixInstance0.TransformVector(pointInHull));

			dgFloat32 timetoImpact = dgFloat32(0.0f);
			dgFloat32 penetration = (m_localPoly[0] - p0) % m_normal + proxy.m_skinThickness;
			if (penetration < dgFloat32(0.0f)) {
				timetoImpact = penetration / (relativeVelocity % m_normal);
				dgAssert(timetoImpact >= dgFloat32(0.0f));
			}

			if (timetoImpact <= proxy.m_timestep) {
				dgVector contactPoints[64];
				contactJoint->m_closestDistance = penetration;
				proxy.m_timestep = timetoImpact;
				proxy.m_normal = m_normal;
				proxy.m_closestPointBody0 = p0;
				proxy.m_closestPointBody1 = p0 + m_normal.Scale4(penetration);

				if (!proxy.m_intersectionTestOnly) {
					pointInHull -= normalInHull.Scale4 (DG_ROBUST_PLANE_CLIP);
					count = proxy.m_instance0->CalculatePlaneIntersection(normalInHull, pointInHull, contactPoints);

					dgVector step(relativeVelocity.Scale4(timetoImpact));
					penetration = dgMax(penetration, dgFloat32(0.0f));
					dgContactPoint* const contactsOut = proxy.m_contacts;
					for (dgInt32 i = 0; i < count; i++) {
						contactsOut[i].m_point = matrixInstance0.TransformVector(contactPoints[i]) + step;
						contactsOut[i].m_normal = m_normal;
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
						contactsOut[i].m_penetration = penetration;
					}
				}
			}
		} else {
			m_vertexCount = dgUnsigned16 (m_count);
			count = world->CalculateConvexToConvexContacts(proxy);
			if (count >= 1) {
				dgContactPoint* const contactsOut = proxy.m_contacts;
				for (dgInt32 i = 0; i < count; i++) {
					contactsOut[i].m_shapeId0 = hullId;
					contactsOut[i].m_shapeId1 = m_faceId;
				}
			}
		}
	}

	return count;
}
