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

	dgFloat32 distH = origin.DotProduct4(dir).m_x;
	planes[0] = dgPlane (dir, dist - distH);
	planes[2] = dgPlane (dir.Scale4 (dgFloat32 (-1.0f)), dist + distH);

	dir = m_normal * dir;
	dgFloat32 distV = origin.DotProduct4(dir).m_x;
	planes[1] = dgPlane (dir, dist - distV);
	planes[3] = dgPlane (dir.Scale4 (dgFloat32 (-1.0f)), dist + distV);

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

			if (test0 > dgFloat32 (0.0f)) {
				if (test1 <= dgFloat32 (0.0f)) {
					const dgVector& p0 = points[ptr->m_incidentVertex];
					const dgVector& p1 = points[ptr->m_next->m_incidentVertex];
					dgVector dp (p1 - p0); 
					points[indexCount] = p0 - dp.Scale3 (test0  / (dp % plane));

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
				if (test1 > dgFloat32 (0.0f)) {
					newFirst = ptr->m_next;

					const dgVector& p0 = points[ptr->m_incidentVertex];
					const dgVector& p1 = points[ptr->m_next->m_incidentVertex];
					dgVector dp (p1 - p0); 
					points[indexCount] = p0 - dp.Scale3 (test0  / (dp % plane));

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

		if(conectCount) {
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
	dgInt32 isConvexCap = 0;
	m_adjacentFaceEdgeNormalIndex = &m_clippEdgeNormal[0];
	do {
		m_clippEdgeNormal[count] = ptr->m_incidentNormal;
		isConvexCap |= (ptr->m_incidentNormal - m_faceNormalIndex);
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

dgInt32 dgCollisionConvexPolygon::CalculatePlaneIntersection (const dgVector& normalIn, const dgVector& origin, dgVector* const contactsOut, dgFloat32 normalSign) const
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
				contactsOut[count] = p0 - plane.Scale3 (side0);
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




dgInt32 dgCollisionConvexPolygon::CalculateContactToConvexHullContinue (dgCollisionParamProxy& proxy, const dgVector& polyInstanceScale, const dgVector& polyInstanceInvScale)
{
	dgAssert (proxy.m_referenceCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionConvexPolygon_RTTI));

	const dgCollisionInstance* const hull = proxy.m_referenceCollision;

	dgAssert (this == proxy.m_floatingCollision->GetChildShape());
	dgAssert (m_count);
	dgAssert (m_count < dgInt32 (sizeof (m_localPoly) / sizeof (m_localPoly[0])));

	const dgBody* const floatingBody = proxy.m_floatingBody;
	const dgBody* const referenceBody = proxy.m_referenceBody;

	dgContact* const contactJoint = proxy.m_contactJoint;
	contactJoint->m_closestDistance = dgFloat32 (1.0e10f);

	m_normal = m_normal.CompProduct4(polyInstanceInvScale);
	dgAssert (m_normal.m_w == dgFloat32 (0.0f));
	m_normal = m_normal.CompProduct4(m_normal.DotProduct4(m_normal).InvSqrt());
	const dgVector savedFaceNormal (m_normal);

	for (dgInt32 i = 0; i < m_count; i ++) {
		m_localPoly[i] = polyInstanceScale.CompProduct4(dgVector (&m_vertex[m_vertexIndex[i] * m_stride]));
		dgAssert (m_localPoly[i].m_w == dgFloat32 (0.0f));
	}

	dgVector hullOrigin (proxy.m_matrix.UntransformVector(dgVector (dgFloat32 (0.0f))));
	hullOrigin = (hullOrigin - m_normal.CompProduct4(m_normal.DotProduct4(hullOrigin - m_localPoly[0]))) | dgVector::m_wOne;

	dgMatrix polygonMatrix;
	polygonMatrix[0] = m_localPoly[1] - m_localPoly[0];
	polygonMatrix[0] = polygonMatrix[0].CompProduct4 (polygonMatrix[0].InvMagSqrt());
	polygonMatrix[1] = m_normal;
	polygonMatrix[2] = polygonMatrix[0] * m_normal;
	polygonMatrix[3] = hullOrigin;
	dgAssert (polygonMatrix.TestOrthogonal());

	dgMatrix savedProxyMatrix (proxy.m_matrix);
	proxy.m_matrix = polygonMatrix * proxy.m_matrix;

	dgVector floatingVeloc (floatingBody->m_veloc);
	dgVector referenceVeloc (referenceBody->m_veloc);
	const dgMatrix& hullMatrix = hull->GetGlobalMatrix();
	dgVector hullRelativeVeloc (hullMatrix.UnrotateVector(referenceVeloc - floatingVeloc));
	dgVector polyRelativeVeloc (proxy.m_matrix.UnrotateVector (hullRelativeVeloc));

	dgVector polyBoxP0 (dgFloat32 ( 1.0e15f));
	dgVector polyBoxP1 (dgFloat32 (-1.0e15f));
	m_normal = polygonMatrix.UnrotateVector(m_normal);

	if (m_normal.DotProduct4(polyRelativeVeloc).m_x >= 0.0f) {
		proxy.m_matrix = savedProxyMatrix;
		return 0;
	}
	for (dgInt32 i = 0; i < m_count; i ++) {
		m_localPoly[i] = polygonMatrix.UntransformVector(m_localPoly[i]);
		dgAssert (m_localPoly[i].m_w == dgFloat32 (0.0f));
		polyBoxP0 = polyBoxP0.GetMin (m_localPoly[i]);
		polyBoxP1 = polyBoxP1.GetMax (m_localPoly[i]);
	}
	dgInt32 count = 0;


	dgVector hullBoxP0;
	dgVector hullBoxP1;
	hull->CalcAABB (proxy.m_matrix.Inverse(), hullBoxP0, hullBoxP1);
	dgVector minBox (polyBoxP0 - hullBoxP1);
	dgVector maxBox (polyBoxP1 - hullBoxP0);
	dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), polyRelativeVeloc);
	dgFloat32 distance = ray.BoxIntersect(minBox, maxBox);

	if (distance < dgFloat32 (1.0f)) {

		dgVector boxSize ((hullBoxP1 - hullBoxP0).Scale4 (dgFloat32 (0.5f)));
//		dgVector boxOrigin ((hullBoxP1 + hullBoxP0).Scale4 (dgFloat32 (0.5f)));
//		boxOrigin += polyRelativeVeloc.Scale4 (distance);

		dgVector normalInHull (proxy.m_matrix.RotateVector (m_normal.Scale4 (dgFloat32 (-1.0f))));
		dgVector pointInHull (hull->SupportVertex (normalInHull, NULL));
		dgVector pointInPlane (proxy.m_matrix.UntransformVector (pointInHull));
		dgFloat32 distToPlane = (m_localPoly[0] - pointInPlane) % m_normal;
		dgFloat32 timeToPlane = distToPlane / (polyRelativeVeloc % m_normal);
		dgVector boxOrigin (pointInPlane + polyRelativeVeloc.Scale4(timeToPlane));

		bool inside = true;
		dgInt32 i0 = m_count - 1;
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgVector e (m_localPoly[i] - m_localPoly[i0]);
			dgVector n (m_normal * e);
			dgPlane plane (n, - (m_localPoly[i0] % n));

			dgVector supportDist (plane.Abs().DotProduct4 (boxSize)); 
			dgFloat32 centerDist = plane.Evalue(boxOrigin);

			if ((centerDist + supportDist.m_x) < dgFloat32 (0.0f)) {
				proxy.m_matrix = savedProxyMatrix;
				return 0;
			}

			if ((centerDist - supportDist.m_x) < dgFloat32 (0.0f)) {
				inside = false;
			}
			i0 = i;
		}

// for the time being for the minkousky contact calculation
inside = false;
		const dgInt32 hullId = hull->GetUserDataID();
		if (inside) {
			dgVector normalInHull (proxy.m_matrix.RotateVector (m_normal.Scale4 (dgFloat32 (-1.0f))));
			dgVector pointInHull (hull->SupportVertex (normalInHull, NULL));
			dgVector p0 (proxy.m_matrix.UntransformVector (pointInHull));

			dgFloat32 timetoImpact = dgFloat32 (0.0f);
			//dgFloat32 closestDistance = dgFloat32 (0.0f);
dgAssert (0);			
//			dgFloat32 penetration = (m_localPoly[0] - p0) % m_normal + proxy.m_skinThickness + DG_IMPULSIVE_CONTACT_PENETRATION;
			dgFloat32 penetration = (m_localPoly[0] - p0) % m_normal + proxy.m_skinThickness;
			if (penetration < dgFloat32 (0.0f)) {
				timetoImpact = penetration / (polyRelativeVeloc % m_normal);
				dgAssert (timetoImpact >= dgFloat32 (0.0f));
//				closestDistance = -penetration;
			}

			if (timetoImpact <= proxy.m_timestep) {
				dgVector pointsContacts[64];

				contactJoint->m_closestDistance = penetration;
dgAssert (0);
//				dgVector point (pointInHull - normalInHull.Scale4(DG_IMPULSIVE_CONTACT_PENETRATION));
				dgVector point (pointInHull);

				count = hull->CalculatePlaneIntersection (normalInHull, point, pointsContacts, 1.0f);
dgAssert (0);
//				dgVector step (hullRelativeVeloc.Scale3 (timetoImpact) + normalInHull.Scale4(DG_IMPULSIVE_CONTACT_PENETRATION));
				dgVector step (hullRelativeVeloc.Scale3 (timetoImpact));

				penetration = dgMax (penetration, dgFloat32 (0.0f));
				const dgMatrix& worldMatrix = hull->m_globalMatrix;
				dgContactPoint* const contactsOut = proxy.m_contacts;
				dgVector globalNormal (worldMatrix.RotateVector(normalInHull));
				for (dgInt32 i = 0; i < count; i ++) {
					contactsOut[i].m_point = worldMatrix.TransformVector (pointsContacts[i] + step);
					contactsOut[i].m_normal = globalNormal;
					contactsOut[i].m_shapeId0 = hullId;
					contactsOut[i].m_shapeId1 = m_faceId;
					contactsOut[i].m_penetration = penetration;
				}
			}
		} else {
			dgFloat32 convexSphapeUmbra = hull->GetUmbraClipSize ();
			if (m_faceClipSize > convexSphapeUmbra) {
				BeamClipping (boxOrigin, convexSphapeUmbra);
				m_faceClipSize = hull->m_childShape->GetBoxMaxRadius();
			}

			dgCollisionConvex* const convexShape = (dgCollisionConvex*) hull->m_childShape;
			count = convexShape->CalculateConvexCastContacts (proxy);

//			dgAssert (proxy.m_intersectionTestOnly || (count >= 0));
			if (count >= 1) {
				dgContactPoint* const contactsOut = proxy.m_contacts;
#if 0
				if (m_closestFeatureType == 3) {
					for (dgInt32 i = 0; i < count; i ++) {
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
					}
				} else {
					dgVector normal (polygonInstance->m_globalMatrix.UnrotateVector(contactsOut[0].m_normal)); 
					if ((normal % savedFaceNormal) < dgFloat32 (0.995f)) {
						dgInt32 index = m_adjacentFaceEdgeNormalIndex[m_closestFeatureStartIndex];
						dgVector n (&m_vertex[index * m_stride]);
						dgVector dir0 (n * savedFaceNormal);
						dgVector dir1 (n * normal);
						dgFloat32 projection = dir0 % dir1;
						if (projection <= dgFloat32 (0.0f)) {
							normal = n;
						}
						normal = polygonInstance->m_globalMatrix.RotateVector(normal);
						for (dgInt32 i = 0; i < count; i ++) {
							contactsOut[i].m_normal = normal;
							//contactsOut[i].m_userId = m_faceId;
							contactsOut[i].m_shapeId0 = hullId;
							contactsOut[i].m_shapeId1 = m_faceId;
						}
					} else {
						for (dgInt32 i = 0; i < count; i ++) {
							//contactsOut[i].m_userId = m_faceId;
							contactsOut[i].m_shapeId0 = hullId;
							contactsOut[i].m_shapeId1 = m_faceId;
						}
					}
				}
#endif

				for (dgInt32 i = 0; i < count; i ++) {
					contactsOut[i].m_shapeId0 = hullId;
					contactsOut[i].m_shapeId1 = m_faceId;
				}
			}
		}
	}

	proxy.m_matrix = savedProxyMatrix;
	return count;
}




dgInt32 dgCollisionConvexPolygon::CalculateContactToConvexHullDescrete(dgCollisionParamProxy& proxy, const dgVector& polyInstanceScale, const dgVector& polyInstanceInvScale)
{
	dgAssert(proxy.m_referenceCollision->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert(proxy.m_floatingCollision->IsType(dgCollision::dgCollisionConvexPolygon_RTTI));

	const dgCollisionInstance* const polygonInstance = proxy.m_floatingCollision;
	dgAssert(this == polygonInstance->GetChildShape());
	dgAssert(m_count);
	dgAssert(m_count < dgInt32(sizeof (m_localPoly) / sizeof (m_localPoly[0])));

	dgInt32 count = 0;

	m_normal = m_normal.CompProduct4(polyInstanceInvScale);
	dgAssert(m_normal.m_w == dgFloat32(0.0f));
	m_normal = m_normal.CompProduct4(m_normal.DotProduct4(m_normal).InvSqrt());
	dgVector savedFaceNormal(m_normal);
	for (dgInt32 i = 0; i < m_count; i++) {
		m_localPoly[i] = polyInstanceScale.CompProduct4(dgVector(&m_vertex[m_vertexIndex[i] * m_stride]));
		dgAssert(m_localPoly[i].m_w == dgFloat32(0.0f));
	}

	dgVector hullOrigin(proxy.m_matrix.UntransformVector(dgVector(dgFloat32(0.0f))));

	dgMatrix polygonMatrix;
	polygonMatrix[0] = m_localPoly[1] - m_localPoly[0];
	polygonMatrix[0] = polygonMatrix[0].CompProduct4(polygonMatrix[0].DotProduct4(polygonMatrix[0]).InvSqrt());
	polygonMatrix[1] = m_normal;
	polygonMatrix[2] = polygonMatrix[0] * m_normal;
	polygonMatrix[3] = (hullOrigin - m_normal.CompProduct4(m_normal.DotProduct4(hullOrigin - m_localPoly[0]))) | dgVector::m_wOne;

	dgAssert(polygonMatrix.TestOrthogonal());

	m_normal = polygonMatrix.UnrotateVector(m_normal);
	for (dgInt32 i = 0; i < m_count; i++) {
		m_localPoly[i] = polygonMatrix.UntransformVector(m_localPoly[i]);
		dgAssert(m_localPoly[i].m_w == dgFloat32(0.0f));
	}
	dgMatrix savedProxyMatrix(proxy.m_matrix);
	proxy.m_matrix = polygonMatrix * proxy.m_matrix;

	dgContact* const contactJoint = proxy.m_contactJoint;
	const dgCollisionInstance* const hull = proxy.m_referenceCollision;

	dgVector normalInHull(proxy.m_matrix.RotateVector(m_normal));
	dgVector pointInHull(hull->SupportVertex(normalInHull.Scale4(dgFloat32(-1.0f)), NULL));
	dgVector p0(proxy.m_matrix.UntransformVector(pointInHull));
	dgVector p1(proxy.m_matrix.UntransformVector(hull->SupportVertex(normalInHull, NULL)));

	dgFloat32 penetration = (m_localPoly[0] - p0) % m_normal + proxy.m_skinThickness;
	if (penetration < dgFloat32(0.0f)) {
		contactJoint->m_closestDistance = -penetration;
		proxy.m_matrix = savedProxyMatrix;
		return 0;
	}

	contactJoint->m_closestDistance = dgFloat32(0.0f);
	dgFloat32 distance = (m_localPoly[0] - p1) % m_normal;
	if (distance >= dgFloat32(0.0f)) {
		proxy.m_matrix = savedProxyMatrix;
		return 0;
	}

	const dgVector& boxSize = hull->GetBoxSize();
	const dgVector& boxOrigin = hull->GetBoxOrigin();

	bool inside = true;
	dgInt32 i0 = m_count - 1;
	for (dgInt32 i = 0; i < m_count; i++) {

		dgVector e(m_localPoly[i] - m_localPoly[i0]);
		dgVector n(m_normal * e);
		dgPlane plane(n, -(m_localPoly[i0] % n));
		plane = proxy.m_matrix.TransformPlane(plane);

		dgFloat32 supportDist = dgAbsf(plane.m_x) * boxSize.m_x + dgAbsf(plane.m_y) * boxSize.m_y + dgAbsf(plane.m_z) * boxSize.m_z;
		dgFloat32 centerDist = plane.Evalue(boxOrigin);

		if ((centerDist + supportDist) < dgFloat32(0.0f)) {
			proxy.m_matrix = savedProxyMatrix;
			return 0;
		}

		if ((centerDist - supportDist) < dgFloat32(0.0f)) {
			inside = false;
		}
		i0 = i;
	}

	const dgInt32 hullId = hull->GetUserDataID();
	if (inside & !proxy.m_intersectionTestOnly) {
		dgAssert(penetration >= dgFloat32(0.0f));
		dgVector pointsContacts[64];

		dgAssert(penetration >= 0.0f);
		dgVector point(pointInHull + normalInHull.Scale4(penetration));

		count = hull->CalculatePlaneIntersection(normalInHull.Scale4(dgFloat32(-1.0f)), point, pointsContacts, 1.0f);
		dgVector step(normalInHull.Scale4((proxy.m_skinThickness - penetration) * dgFloat32(0.5f)));

		const dgMatrix& worldMatrix = hull->m_globalMatrix;
		dgContactPoint* const contactsOut = proxy.m_contacts;
		dgAssert(contactsOut);
		dgVector globalNormal(worldMatrix.RotateVector(normalInHull));
		for (dgInt32 i = 0; i < count; i++) {
			contactsOut[i].m_point = worldMatrix.TransformVector(pointsContacts[i] + step);
			contactsOut[i].m_normal = globalNormal;
			contactsOut[i].m_shapeId0 = hullId;
			contactsOut[i].m_shapeId1 = m_faceId;
			contactsOut[i].m_penetration = penetration;
		}
	} else {
		dgFloat32 convexSphapeUmbra = hull->GetUmbraClipSize();
		if (m_faceClipSize > convexSphapeUmbra) {
			//const dgBody* const refBody = proxy.m_referenceBody;
			//dgVector origin (polygonInstance->m_globalMatrix.UntransformVector((refBody->m_minAABB + refBody->m_maxAABB).Scale3 (dgFloat32 (0.5f))));
			//dgVector origin ((refBody->m_minAABB + refBody->m_maxAABB).Scale4 (dgFloat32 (0.5f)));
			//origin = proxy.m_matrix.UntransformVector(hull->GetGlobalMatrix().UntransformVector(origin));
			BeamClipping(dgVector(dgFloat32(0.0f)), convexSphapeUmbra);
			m_faceClipSize = hull->m_childShape->GetBoxMaxRadius();
		}

		dgCollisionConvex* const convexShape = (dgCollisionConvex*)hull->m_childShape;
		count = convexShape->CalculateConvexToConvexContact(proxy);
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
				dgVector normal(polygonInstance->m_globalMatrix.UnrotateVector(contactsOut[0].m_normal));
				if ((normal % savedFaceNormal) < dgFloat32(0.9995f)) {
					dgInt32 index = m_adjacentFaceEdgeNormalIndex[m_closestFeatureStartIndex];
					dgVector n(&m_vertex[index * m_stride]);
					if ((savedFaceNormal.DotProduct4(n).GetScalar() > dgFloat32(0.9995f))) {
						normal = n;
					} else {
						dgVector dir0(n * savedFaceNormal);
						dgVector dir1(n * normal);
						dgFloat32 projection = dir0 % dir1;
						if (projection <= dgFloat32(0.0f)) {
							normal = n;
						}
					}
					normal = polygonInstance->m_globalMatrix.RotateVector(normal);

					for (dgInt32 i = 0; i < count; i++) {
						contactsOut[i].m_normal = normal;
						//contactsOut[i].m_userId = m_faceId;
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
					}
				} else {
					for (dgInt32 i = 0; i < count; i++) {
						//contactsOut[i].m_userId = m_faceId;
						contactsOut[i].m_shapeId0 = hullId;
						contactsOut[i].m_shapeId1 = m_faceId;
					}
				}
			}
		}
	}

	proxy.m_matrix = savedProxyMatrix;
	return count;
}
