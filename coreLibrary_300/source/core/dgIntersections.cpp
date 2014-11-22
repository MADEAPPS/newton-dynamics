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

#include "dgStdafx.h"
#include "dgMemory.h"
#include "dgVector.h"
#include "dgPlane.h"
#include "dgGoogol.h"
#include "dgIntersections.h"

#define USE_FLOAT_VERSION


#define DG_RAY_TOL_ERROR (dgFloat32 (-1.0e-3f))
#define DG_RAY_TOL_ADAPTIVE_ERROR (dgFloat32 (1.0e-1f))



dgFloat32 dgFastRayTest::PolygonIntersectFallback (const dgVector& normal, dgFloat32 maxT, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const
{
	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	dgBigVector v0 (&polygon[indexArray[indexCount - 1] * stride]);
	dgBigVector m_p0_ (m_p0);
	dgBigVector p0v0_ (v0 - m_p0_);
	dgBigVector normal_ (normal);
	dgBigVector diff_ (m_diff);
	dgFloat64 tOut = normal_ % p0v0_;
	dgFloat64 dist = normal_ % diff_;
	if (tOut >= dist * maxT) {
		if ((tOut < dgFloat64 (0.0f)) && (tOut > dist)) {
			for (dgInt32 i = 0; i < indexCount; i ++) {
				dgInt32 i2 = indexArray[i] * stride;
				dgBigVector v1 (&polygon[i2]);
				dgBigVector p0v1_ (v1 - m_p0_);
				// calculate the volume formed by the line and the edge of the polygon
				dgFloat64 alpha = (diff_ * p0v1_) % p0v0_;
				// if a least one volume is negative it mean the line cross the polygon outside this edge and do not hit the face
				if (alpha < DG_RAY_TOL_ERROR) {
					return 1.2f;
				}
				p0v0_ = p0v1_;
			}

			//the line is to the left of all the polygon edges, 
			//then the intersection is the point we the line intersect the plane of the polygon
			tOut = tOut / dist;
			dgAssert (tOut >= dgFloat32 (0.0f));
			dgAssert (tOut <= dgFloat32 (1.0f));
			return dgFloat32 (tOut);
		}
	}
	return dgFloat32 (1.2f);
}



dgFloat32 dgFastRayTest::PolygonIntersect (const dgVector& normal, dgFloat32 maxT, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const
{
#if 1
	dgAssert (m_p0.m_w == dgFloat32 (0.0f));
	dgAssert (m_p1.m_w == dgFloat32 (0.0f));
	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	dgBigVector v0 (&polygon[indexArray[indexCount - 1] * stride]);
	dgBigVector m_p0_ (m_p0);
	dgBigVector p0v0_ (v0 - m_p0_);
	dgBigVector normal_ (normal);
	dgBigVector diff_ (m_diff);
	dgFloat64 tOut = normal_ % p0v0_;
	dgFloat64 dist = normal_ % diff_;
	if (tOut >= dist * maxT) {
		if ((tOut < dgFloat64 (0.0f)) && (tOut > dist)) {
			for (dgInt32 i = 0; i < indexCount; i ++) {
				dgInt32 i2 = indexArray[i] * stride;
				dgBigVector v1 (&polygon[i2]);
				dgBigVector p0v1_ (v1 - m_p0_);
				// calculate the volume formed by the line and the edge of the polygon
				dgFloat64 alpha = (diff_ * p0v1_) % p0v0_;
				// if a least one volume is negative it mean the line cross the polygon outside this edge and do not hit the face
				if (alpha < DG_RAY_TOL_ERROR) {
					return 1.2f;
				}
				p0v0_ = p0v1_;
			}

			//the line is to the left of all the polygon edges, 
			//then the intersection is the point we the line intersect the plane of the polygon
			tOut = tOut / dist;
			dgAssert (tOut >= dgFloat32 (0.0f));
			dgAssert (tOut <= dgFloat32 (1.0f));
			return dgFloat32 (tOut);
		}
	}
	return dgFloat32 (1.2f);

#else 
	dgAssert (m_p0.m_w == dgFloat32 (0.0f));
	dgAssert (m_p1.m_w == dgFloat32 (0.0f));

	dgFloat32 dist = normal % m_diff;
	if (dist < m_dirError) {

		dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

		dgVector v0 (&polygon[indexArray[indexCount - 1] * stride]);
		dgVector p0v0 (v0 - m_p0);
		dgFloat32 tOut = normal % p0v0;
		if (tOut >= dist * maxT) {
			// this only work for convex polygons and for single side faces 
			// walk the polygon around the edges and calculate the volume 
			dgFloat32 errorTest = m_magRayTest * DG_RAY_TOL_ADAPTIVE_ERROR;
			if ((tOut < dgFloat32 (0.0f)) && (tOut > dist)) {
				for (dgInt32 i = 0; i < indexCount; i ++) {
					dgInt32 i2 = indexArray[i] * stride;
					dgVector v1 (&polygon[i2]);
					dgVector p0v1 (v1 - m_p0);
					// calculate the volume formed by the line and the edge of the polygon
					dgFloat32 alpha = (m_diff * p0v1) % p0v0;
					// if a least one volume is negative it mean the line cross the polygon outside this edge and do not hit the face
					if (dgAbsf (alpha) < errorTest) {
						return PolygonIntersectFallback (normal, maxT, polygon, strideInBytes, indexArray, indexCount);
					} else if (alpha < 0.0f) {
						return dgFloat32 (1.2f);
					}
					p0v0 = p0v1;
				}

				tOut = tOut / dist;
				dgAssert (tOut >= dgFloat32 (0.0f));
				dgAssert (tOut <= dgFloat32 (1.0f));
				return tOut;
			}
		}
	}
	return dgFloat32 (1.2f);
#endif
}



bool dgApi dgRayBoxClip (dgVector& p0, dgVector& p1, const dgVector& boxP0, const dgVector& boxP1) 
{	
	dgAssert (p0.m_w == dgFloat32(0.0f));
	dgAssert (p1.m_w == dgFloat32(0.0f));
	dgAssert (boxP0.m_w == dgFloat32(0.0f));
	dgAssert (boxP1.m_w == dgFloat32(0.0f));
	for (int i = 0; i < 3; i ++) {
		dgFloat32 tmp0 = boxP1[i] - p0[i];
		if (tmp0 > dgFloat32 (0.0f)) {
			dgFloat32 tmp1 = boxP1[i] - p1[i];
			if (tmp1 < dgFloat32 (0.0f)) {
				p1 = p0 + (p1 - p0).Scale4 (tmp0 / (p1[i] - p0[i])); 
				p1[i] = boxP1[i];
			}
		} else {
			dgFloat32 tmp1 = boxP1[i] - p1[i];
			if (tmp1 > dgFloat32 (0.0f)) {
				p0 += (p1 - p0).Scale4 (tmp0 / (p1[i] - p0[i])); 
				p0[i] = boxP1[i];
			} else {
				return false;
			}
		}

		tmp0 = boxP0[i] - p0[i];
		if (tmp0 < dgFloat32 (0.0f)) {
			dgFloat32 tmp1 = boxP0[i] - p1[i];
			if (tmp1 > dgFloat32 (0.0f)) {
				p1 = p0 + (p1 - p0).Scale4 (tmp0 / (p1[i] - p0[i])); 
				p1[i] = boxP0[i];
			}
		} else {
			dgFloat32 tmp1 = boxP0[i] - p1[i];
			if (tmp1 < dgFloat32 (0.0f)) {
				p0 += (p1 - p0).Scale4 (tmp0 / (p1[i] - p0[i])); 
				p0[i] = boxP0[i];
			} else {
				return false;
			}
		}
	}
	return true;
}


dgVector dgApi dgPointToRayDistance (const dgVector& point, const dgVector& ray_p0, const dgVector& ray_p1)
{
	dgFloat32 t;
	dgVector dp (ray_p1 - ray_p0);
	t = dgClamp (((point - ray_p0) % dp) / (dp % dp), dgFloat32(dgFloat32 (0.0f)), dgFloat32 (1.0f));
	return ray_p0 + dp.Scale3 (t);
}

void dgApi dgRayToRayDistance (const dgVector& ray_p0, const dgVector& ray_p1, const dgVector& ray_q0, const dgVector& ray_q1, dgVector& pOut, dgVector& qOut)
{
	dgFloat32 sN;
	dgFloat32 tN;

	dgVector u (ray_p1 - ray_p0);
	dgVector v (ray_q1 - ray_q0);
	dgVector w (ray_p0 - ray_q0);

	dgFloat32 a = u % u;        // always >= 0
	dgFloat32 b = u % v;
	dgFloat32 c = v % v;        // always >= 0
	dgFloat32 d = u % w;
	dgFloat32 e = v % w;
	dgFloat32 D = a*c - b*b;   // always >= 0
	dgFloat32 sD = D;			// sc = sN / sD, default sD = D >= 0
	dgFloat32 tD = D;			// tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < dgFloat32 (1.0e-8f)) { // the lines are almost parallel
		sN = dgFloat32 (0.0f);        // force using point P0 on segment S1
		sD = dgFloat32 (1.0f);        // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	} else {                // get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < dgFloat32 (0.0f)) {       // sc < 0 => the s=0 edge is visible
			sN = dgFloat32 (0.0f);
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}


	if (tN < dgFloat32 (0.0f)) {           // tc < 0 => the t=0 edge is visible
		tN = dgFloat32 (0.0f);
		// recompute sc for this edge
		if (-d < dgFloat32 (0.0f))
			sN = dgFloat32 (0.0f);
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < dgFloat32 (0.0f))
			sN = dgFloat32 (0.0f);
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}

	// finally do the division to get sc and tc
	dgFloat32 sc = (dgAbsf(sN) < dgFloat32(1.0e-8f) ? dgFloat32 (0.0f) : sN / sD);
	dgFloat32 tc = (dgAbsf(tN) < dgFloat32(1.0e-8f) ? dgFloat32 (0.0f) : tN / tD);

	pOut = ray_p0 + u.Scale3 (sc);
	qOut = ray_q0 + v.Scale3 (tc);
}

/*
dgVector dgPointToTriangleDistance____ (const dgVector& point, const dgVector& p0, const dgVector& p1, const dgVector& p2)
{
	const dgVector p10 (p1 - p0);
	const dgVector p20 (p2 - p0);
	const dgVector p_p0 (point - p0);

	dgFloat32 alpha1 = p10 % p_p0;
	dgFloat32 alpha2 = p20 % p_p0;
	if ((alpha1 <= dgFloat32 (0.0f)) && (alpha2 <= dgFloat32 (0.0f))) {
		return p0;
	}

	dgVector p_p1 (point - p1);
	dgFloat32 alpha3 = p10 % p_p1;
	dgFloat32 alpha4 = p20 % p_p1;
	if ((alpha3 >= dgFloat32 (0.0f)) && (alpha4 <= alpha3)) {
		return p1;
	}

	dgFloat32 vc = alpha1 * alpha4 - alpha3 * alpha2;
	if ((vc <= dgFloat32 (0.0f)) && (alpha1 >= dgFloat32 (0.0f)) && (alpha3 <= dgFloat32 (0.0f))) {
		dgFloat32 t = alpha1 / (alpha1 - alpha3);
		dgAssert (t >= dgFloat32 (0.0f));
		dgAssert (t <= dgFloat32 (1.0f));
		return p0 + p10.Scale3 (t);
	}


	dgVector p_p2 (point - p2);
	dgFloat32 alpha5 = p10 % p_p2;
	dgFloat32 alpha6 = p20 % p_p2;
	if ((alpha6 >= dgFloat32 (0.0f)) && (alpha5 <= alpha6)) {
		return p2;
	}


	dgFloat32 vb = alpha5 * alpha2 - alpha1 * alpha6;
	if ((vb <= dgFloat32 (0.0f)) && (alpha2 >= dgFloat32 (0.0f)) && (alpha6 <= dgFloat32 (0.0f))) {
		dgFloat32 t = alpha2 / (alpha2 - alpha6);
		dgAssert (t >= dgFloat32 (0.0f));
		dgAssert (t <= dgFloat32 (1.0f));
		return p0 + p20.Scale3 (t);
	}


	dgFloat32 va = alpha3 * alpha6 - alpha5 * alpha4;
	if ((va <= dgFloat32 (0.0f)) && ((alpha4 - alpha3) >= dgFloat32 (0.0f)) && ((alpha5 - alpha6) >= dgFloat32 (0.0f))) {
		dgFloat32 t = (alpha4 - alpha3) / ((alpha4 - alpha3) + (alpha5 - alpha6));
		dgAssert (t >= dgFloat32 (0.0f));
		dgAssert (t <= dgFloat32 (1.0f));
		return p1 + (p2 - p1).Scale3 (t);
	}

	dgFloat32 den = float(dgFloat32 (1.0f)) / (va + vb + vc);
	dgFloat32 t = vb * den;
	dgFloat32 s = vc * den;
	dgAssert (t >= dgFloat32 (0.0f));
	dgAssert (s >= dgFloat32 (0.0f));
	dgAssert (t <= dgFloat32 (1.0f));
	dgAssert (s <= dgFloat32 (1.0f));
	return p0 + p10.Scale3 (t) + p20.Scale3 (s);
}
*/

dgVector dgPointToTriangleDistance (const dgVector& point, const dgVector& p0, const dgVector& p1, const dgVector& p2, const dgVector& normal)
{
//	return  dgPointToTriangleDistance____ (point, p0, p1, p2);

#ifdef _DEBUG
	dgVector faceNormal ((p1 - p0) * (p2 - p0));
	dgFloat64 faceNormal2 = faceNormal % faceNormal;
	dgFloat64 normal2 = normal % normal;
	dgFloat64 faceNormalNormal = faceNormal % normal;
	dgFloat64 error = (faceNormalNormal * faceNormalNormal - faceNormal2 * normal2) / (faceNormalNormal * faceNormalNormal);
	dgAssert (fabsf (error < dgFloat32 (1.0e-5f)));
	dgAssert (faceNormalNormal > dgFloat32 (0.0f));
#endif

	dgVector array[3];
	array[0] = p0;
	array[1] = p1;
	array[2] = p2;

	dgInt32 i0 = 2;
	dgInt32 closestIndex = -1;
	dgVector p1p0 (array[2] - point);
	for (dgInt32 i1 = 0; i1 < 3; i1 ++) {
		dgVector p2p0 (array[i1] - point);

		dgFloat32 volume = (p1p0 * p2p0) % normal;
		if (volume < dgFloat32 (0.0f)) {
			dgVector segment (array[i1] - array[i0]);
			dgVector poinP0 (point - array[i0]);
			dgFloat32 den = segment % segment;
			dgAssert (den > dgFloat32 (0.0f));
			dgFloat32 num = poinP0 % segment;
			if (num < dgFloat32 (0.0f)) {
				closestIndex = i0;
			} else if (num > den) {
				closestIndex = i1;
			} else {
				return array[i0] + segment.Scale3 (num / den);
			}
		}
		p1p0 = p2p0;
		i0 = i1;
	}

	if (closestIndex >= 0) {
		return array[closestIndex];
	} else {
		return point - normal.Scale3((normal % (point - p0)) / (normal % normal));
	}
}


dgBigVector dgPointToTriangleDistance (const dgBigVector& point, const dgBigVector& p0, const dgBigVector& p1, const dgBigVector& p2, const dgBigVector& normal)
{
#ifdef _DEBUG
	dgBigVector faceNormal ((p1 - p0) * (p2 - p0));
	dgFloat64 faceNormal2 = faceNormal % faceNormal;
	dgFloat64 normal2 = normal % normal;
	dgFloat64 faceNormalNormal = faceNormal % normal;
	dgFloat64 error = (faceNormalNormal * faceNormalNormal - faceNormal2 * normal2) / (faceNormalNormal * faceNormalNormal);
	dgAssert (fabsf (error < dgFloat32 (1.0e-5f)));
	dgAssert (faceNormalNormal > dgFloat32 (0.0f));
#endif

	dgBigVector array[3];
	array[0] = p0;
	array[1] = p1;
	array[2] = p2;

	dgInt32 i0 = 2;
	dgInt32 closestIndex = -1;
	dgBigVector p1p0 (array[2] - point);
	for (dgInt32 i1 = 0; i1 < 3; i1 ++) {
		dgBigVector p2p0 (array[i1] - point);

		dgFloat64 volume = (p1p0 * p2p0) % normal;
		if (volume < dgFloat32 (0.0f)) {
			dgBigVector segment (array[i1] - array[i0]);
			dgBigVector poinP0 (point - array[i0]);
			dgFloat64 den = segment % segment;
			dgAssert (den > dgFloat32 (0.0f));
			dgFloat64 num = poinP0 % segment;
			if (num < dgFloat32 (0.0f)) {
				closestIndex = i0;
			} else if (num > den) {
				closestIndex = i1;
			} else {
				return array[i0] + segment.Scale3 (num / den);
			}
		}
		p1p0 = p2p0;
		i0 = i1;
	}

	if (closestIndex >= 0) {
		return array[closestIndex];
	} else {
		return point - normal.Scale3((normal % (point - p0)) / (normal % normal));
	}
}


/*
bool dgApi dgPointToPolygonDistance (const dgVector& p, const dgFloat32* const polygon, dgInt32 strideInBytes,
									 const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 bailDistance, dgVector& out)
{
	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	dgInt32 i0 = indexArray[0] * stride;
	dgInt32 i1 = indexArray[1] * stride;

	const dgVector v0 (&polygon[i0]);
	dgVector v1 (&polygon[i1]);
	dgVector closestPoint (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgFloat32 minDist = dgFloat32 (1.0e20f);
	for (dgInt32 i = 2; i < indexCount; i ++) {
		dgInt32 i2 = indexArray[i] * stride;
		const dgVector v2 (&polygon[i2]);
		const dgVector q (dgPointToTriangleDistance (p, v0, v1, v2));
		const dgVector error (q - p);
		dgFloat32 dist = error % error;
		if (dist < minDist) {
			minDist = dist;
			closestPoint = q;
		}
		v1 = v2;
	}

	if (minDist > (bailDistance * bailDistance)) {
		return false;
	}

	out = closestPoint;
	return true;
}

dgBigVector LineTriangleIntersection (const dgBigVector& p0, const dgBigVector& p1, const dgBigVector& A, const dgBigVector& B, const dgBigVector& C)
{
	dgHugeVector ph0 (p0);
	dgHugeVector ph1 (p1);
	dgHugeVector Ah (A);
	dgHugeVector Bh (B);
	dgHugeVector Ch (C);

	dgHugeVector p1p0 (ph1 - ph0);
	dgHugeVector Ap0 (Ah - ph0);
	dgHugeVector Bp0 (Bh - ph0);
	dgHugeVector Cp0 (Ch - ph0);

	dgGoogol t0 ((Bp0 * Cp0) % p1p0);
	dgFloat64 val0 = t0.GetAproximateValue();	
	if (val0 < dgFloat64 (0.0f)) {
		return dgBigVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (-1.0f));
	}

	dgGoogol t1 ((Cp0 * Ap0) % p1p0);
	dgFloat64 val1 = t1.GetAproximateValue();	
	if (val1 < dgFloat64 (0.0f)) {
		return dgBigVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (-1.0f));
	}

	dgGoogol t2 ((Ap0 * Bp0) % p1p0);
	dgFloat64 val2 = t2.GetAproximateValue();	
	if (val2 < dgFloat64 (0.0f)) {
		return dgBigVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (-1.0f));
	}

	dgGoogol sum = t0 + t1 + t2;
	dgFloat64 den = sum.GetAproximateValue();

#ifdef _DEBUG
	dgBigVector testpoint (A.Scale3 (val0 / den) + B.Scale3 (val1 / den) + C.Scale3(val2 / den));
	dgFloat64 volume = ((B - A) * (C - A)) % (testpoint - A);
	dgAssert (fabs (volume) < dgFloat64 (1.0e-12f));
#endif
	return dgBigVector (val0 / den, val1 / den, val2 / den, dgFloat32 (0.0f));
}
*/





// using my old method which clip the line again the voronoi regions of the polygon first, then find the 
// shortest of the first time to impact of each segment to each respective voronoi region region. 
// the method is much slower than Paul Nettle method but is is more reliably and accurate.
// in fact I believe could never get Paul Nettle's method to work correctly in all cases, 
// I start to have strong suspicion that Paul Nettle method has an analytical flaw and does not really extend to 3d.
class dgSweepSphereToPolygon
{
	public: 
	dgSweepSphereToPolygon (dgInt32 count, const dgVector* const polygon, const dgVector& normal)
		:m_count(count) 
		,m_polygon(polygon) 
		,m_normal(normal) 
	{
		m_vertexIndex[0] = 0;
		for (dgInt32 i = 0; i < count; i ++) {
			m_prevVertex[i] = i - 1;
			m_nextVertex[i] = i + 1;
			m_edgeVoronoi[i] = i + 1;
			m_vertexVoronoi[i] = count + i + 1;
			m_vertexIndex[i + 1] = i;
			m_vertexIndex[m_count + i + 1] = i;
		}
		m_prevVertex[0] = m_count - 1;
		dgAssert ((m_count - 1) > 0);
		m_nextVertex[m_count - 1] = 0;
	}

	dgFloat32 CalculateTimeOfImpact(const dgVector& p0, const dgVector& p1, dgFloat32 radius, dgVector& normalOut, dgVector& contactOut) 
	{
		dgBigVector q0(p0); 
		dgBigVector q1(p1); 
		dgBigVector diff (p1 - p0);

		dgBigVector r0 (q0);
		dgInt32 startRegion = GetRegionIndex (r0);
		for (dgInt32 i = 0; i < 10; i ++) {
			dgFloat64 clipParam;
			dgInt32 exitRegion = GetExitRegion (startRegion, r0, q1, clipParam);
			dgBigVector r1 (r0 + (q1 - r0).Scale3 (clipParam));
			dgFloat64 param = CalculateInterstionParam(startRegion, r0, r1, radius, normalOut, contactOut);
			if (param >= dgFloat32 (0.0f)) {
				dgBigVector q2 (r0 + (r1 - r0).Scale3 (param));
				return dgFloat32 (((q2 - q0) % diff) / (diff % diff));
			}
			if (startRegion == exitRegion) {
				break;
			}

			r0 = r1;
			startRegion = exitRegion;
		}
		return dgFloat32 (-1.0f);
	} 

	private:

	void CalculateVertexPlanes (dgBigPlane& planeA, dgBigPlane& planeB, dgInt32 vertexIndex) const
	{
		dgBigVector v0 (m_polygon[vertexIndex]);

		dgInt32 j = m_nextVertex[vertexIndex];
		dgBigVector v1 (m_polygon[j]);
		dgBigVector dir0 (v1 - v0);
		dir0 = dir0.Scale3 (sqrt (dgFloat64(1.0f) / (dir0 % dir0)));
		planeA = dgBigPlane (dir0, -(dir0 % v0));

		j = m_prevVertex[vertexIndex];
		dgBigVector v2 (m_polygon[j]);
		dgBigVector dir2 (v2 - v0);
		dir2 = dir2.Scale3 (sqrt (dgFloat64(1.0f) / (dir2 % dir2)));
		planeB = dgBigPlane (dir2, -(dir2 % v0));
	}


	dgInt32 GetRegionIndex (const dgBigVector& point) const
	{
		dgInt32 voronoiRegion = 0;
		dgBigVector v0 (m_polygon[m_count-1]);
		dgVector pointInPlane (point - m_normal.Scale3 ((point - v0) % m_normal));

		dgBigVector p0 (point);
		dgBigVector p1 (pointInPlane);
		dgBigVector diff (p1 - p0);
		dgBigVector p0v0 (v0 - p0);

		for (dgInt32 i = 0; i < m_count; i ++) {
			dgBigVector v1 (m_polygon[i]);
			dgBigVector p0v1 (v1 - p0);

			// calculate the volume formed by the line and the edge of the polygon
			dgFloat64 alpha = (diff * p0v1) % p0v0;
			if (alpha < dgFloat32 (0.0f)) {

				// calculate the voronoi regions for the edge
				dgInt32 k = m_prevVertex[i];
				dgBigPlane plane0;
				dgBigPlane plane1;
				dgBigPlane plane2;
				dgBigPlane plane3;
				CalculateVertexPlanes (plane0, plane1, k);
				CalculateVertexPlanes (plane2, plane3, i);

				if ((plane0.Evalue(p0) >= dgFloat32 (0.0f)) && (plane3.Evalue(p0) >= dgFloat32 (0.0f))) {
					return m_edgeVoronoi[m_prevVertex[i]];
				} else if ((plane0.Evalue(p0) <= dgFloat32 (0.0f)) && (plane1.Evalue(p0) <= dgFloat32 (0.0f))) {
					return m_vertexVoronoi[m_prevVertex[i]];
				} else {
					dgAssert ((plane2.Evalue(p0) <= dgFloat32 (0.0f)) && (plane3.Evalue(p0) <= dgFloat32 (0.0f)));
					return m_vertexVoronoi[i];
				}
			}

			v0 = v1;
			p0v0 = p0v1;
		}
		return voronoiRegion;
	}


	dgInt32 dgCalculateVoronoiIntersectionParam (dgInt32 region, const dgBigVector& p0, const dgBigVector& p1, dgInt32 planesCount, dgBigPlane* const planes, const dgInt32* const planeRegions, dgFloat64& param)
	{
		param = dgFloat32 (1.0f);
		dgInt32 changedRegion = region; 
		dgInt32 planeIndex = -1;
		dgFloat64 minT = dgFloat64 (1.0f);
		dgBigVector p1p0 (p1 - p0);
		for (dgInt32 i = 0; i < planesCount; i ++) {
			dgFloat64 den = planes[i] % p1p0;
			if (fabs (den) > dgFloat64 (1.0e-20f)) {
				dgFloat64 t = -planes[i].Evalue(p0) / den;
				if (t > dgFloat32 (1.0e-10f)) {
					if (t < minT) {
						planeIndex = i;
						changedRegion = planeRegions[i];
						minT = t;
					}
				}
			}
		}
		if (planeIndex >= 0) {
			param = minT;
		}
		return changedRegion;
	}


	dgInt32 GetExitRegion (dgInt32 orginRegion, const dgBigVector& p0, const dgBigVector& p1, dgFloat64& param)
	{
		param = dgFloat32 (1.0f);
		if (orginRegion == 0) {
			// inside the plane
			// the exit region is the line the point intersect if there is any

			dgInt32 adjacentRegion[32];
			dgBigPlane planes[32];

			dgBigVector diff (p1 - p0);
			// deal we special case that the line is parallel to the normal
			dgFloat64 diff2 = diff % diff;
			dgFloat64 parallelTest = (diff % m_normal);
			if ((parallelTest * parallelTest) < (diff2 * dgFloat32 (0.9999f))) {
				dgBigVector v0 (m_polygon[m_count-1]);
				for (dgInt32 i = 0; i < m_count; i ++) {
					dgBigVector v1 (m_polygon[i]);
					dgBigVector edge (v1 - v0);
					dgBigVector wallNormal (edge * m_normal);
					wallNormal = wallNormal.Scale3 (sqrt (dgFloat64(1.0f) / (wallNormal % wallNormal)));
					planes[i] =  dgBigPlane (wallNormal, - (wallNormal % v1));
					adjacentRegion[i] = m_edgeVoronoi[m_prevVertex[i]];
					v0 = v1;
				}
				return dgCalculateVoronoiIntersectionParam (orginRegion, p0, p1, m_count, planes, adjacentRegion, param);
			}

		} else if (orginRegion <= m_count) {
			// edge region
			dgInt32 i0 = m_vertexIndex[orginRegion];
			dgInt32 i1 = m_nextVertex[i0];

			dgInt32 adjacentRegion[3];
			dgBigPlane planes[3];

			dgBigVector q0 (m_polygon[i0]);
			dgBigVector q1 (m_polygon[i1]);
			dgBigVector dir (q0 - q1);
			dir = dir * m_normal;
			dir = dir.Scale3 (sqrt (dgFloat64(1.0f) / (dir % dir)));
			planes[0] =  dgBigPlane (dir, - (dir % q0));
			adjacentRegion[0] = 0;

			dgBigPlane plane0;
			CalculateVertexPlanes (planes[1], plane0, i0);
			CalculateVertexPlanes (plane0, planes[2], i1);

			adjacentRegion[1] = m_vertexVoronoi[i0];
			adjacentRegion[2] = m_vertexVoronoi[i1];

			planes[1] = planes[1].Scale(dgFloat32 (-1.0f));
			planes[2] = planes[2].Scale(dgFloat32 (-1.0f));
			return dgCalculateVoronoiIntersectionParam (orginRegion, p0, p1, 3, planes, adjacentRegion, param);
		} else {
			// point region
			dgInt32 index = m_vertexIndex[orginRegion];
			dgBigPlane planes[2];
			dgInt32 adjacentRegion[2];
			CalculateVertexPlanes (planes[1], planes[0], index);
			adjacentRegion[0] = m_edgeVoronoi[m_prevVertex[index]];
			adjacentRegion[1] = m_edgeVoronoi[index];
			return dgCalculateVoronoiIntersectionParam (orginRegion, p0, p1, 2, planes, adjacentRegion, param);
		}
		return orginRegion;
	}

	bool dgQuadraticRoots (dgFloat64 a, dgFloat64 b, dgFloat64 c, dgFloat64& t0, dgFloat64& t1) const 
	{
		dgFloat64 desc = b * b - dgFloat64 (4.0f) * a * c;
		if (desc > dgFloat64 (0.0f)) {
			desc = sqrt (desc);
			dgFloat64 den = dgFloat64 (0.5f) / a;
			t0 = (-b + desc) * den;
			t1 = (-b - desc) * den;
			return true;
		}
		return false;
	}


	dgFloat64 CalculateInterstionParam (dgInt32 region, const dgBigVector& p0, const dgBigVector& p1, dgFloat32 radius, dgVector& normalOut, dgVector& contactOut) const 
	{
		if (region == 0) {
			// inside the plane
			dgBigVector diff (p1 - p0);
			dgBigVector dist (p0 - dgBigVector(m_polygon[0]));
			dgFloat64 num = radius - m_normal % dist;
			dgFloat64 den = m_normal % diff;
			dgFloat64 t = num / den;

			if (t < dgFloat32 (0.0f)) {
				dgBigVector r (diff.Scale3 (t));
				if ((r % r) < radius * radius) {
					t = dgFloat64 (0.0f);
				}
			}

			if ((t < dgFloat32 (0.0f)) || (t > dgFloat32 (1.0f))) {
				//t = dgFloat32 (0.0f);
				t = dgFloat32 (-1.0f);
			} else {
				dgBigVector p (p0 + diff.Scale3 (t));
				contactOut = dgVector (p - m_normal.Scale3 (m_normal % (p - dgBigVector(m_polygon[0]))));
				normalOut = m_normal;
			}
			return t;
			
		} else if (region <= m_count) {
			// edge region
			dgInt32 i0 = m_vertexIndex [region];
			dgInt32 i1 = m_nextVertex [i0];
			
			dgBigVector q0 (m_polygon[i0]);
			dgBigVector q1 (m_polygon[i1]);

			dgBigVector q1q0 (q1 - q0);
			dgBigVector p0q0 (p0 - q0);
			dgBigVector p1p0 (p1 - p0);
			dgFloat64 den = dgFloat64 (1.0f) / (q1q0 % q1q0);

			dgBigVector B (q1q0.Scale3 ((p0q0 % q1q0) * den));
			dgBigVector C (q1q0.Scale3 ((p1p0 % q1q0) * den));
			
			dgBigVector r0 (p0q0 - B);
			dgBigVector r1 (p1p0 - C);

			dgFloat64 a = r1 % r1;
			dgFloat64 b = dgFloat64 (2.0f) * (r0 % r1);
			dgFloat64 c = r0 % r0 - radius * radius;

			dgFloat64 t0 = dgFloat64 (-1.0f);
			dgFloat64 t1;
			if (dgQuadraticRoots (a, b, c, t0, t1)) {
				t0 = dgMin(t0, t1);
				if (t0 < dgFloat32 (0.0f)) {
					dgBigVector r ((p1 - p0).Scale3 (t0));
					if ((r % r) < radius * radius) {
						t0 = dgFloat64 (0.0f);
					}
				}

				if ((t0 < dgFloat32 (0.0f)) || (t0 > dgFloat32 (1.0f))) {
					t0 = dgFloat32 (-1.0f);
				} else {
					dgBigVector p (p0 + p1p0.Scale3 (t0));
					dgBigVector q (q0 + B + C.Scale3 (t0));
					
					contactOut = q;
					normalOut = dgVector (p - q);
					normalOut = normalOut.Scale3 (dgRsqrt(normalOut % normalOut));
				}
			}
			return t0;


		} else {
			// point region
			dgInt32 index = m_vertexIndex [region];
			dgBigVector q (m_polygon[index]);

			dgBigVector p1p0 (p1 - p0);
			dgBigVector p0q (p0 - q);
			dgFloat64 a = p1p0 % p1p0;
			dgFloat64 b = dgFloat64 (2.0f) * (p1p0 % p0q);
			dgFloat64 c = p0q % p0q - radius * radius;
			dgFloat64 t0 = dgFloat64 (-1.0f);
			dgFloat64 t1;
			if (dgQuadraticRoots (a, b, c, t0, t1)) {
				t0 = dgMin(t0, t1);

				if (t0 < dgFloat32 (0.0f)) {
					dgBigVector r ((p1 - p0).Scale3 (t0));
					if ((r % r) < radius * radius) {
						t0 = dgFloat64 (0.0f);
					}
				}

				if ((t0 < dgFloat32 (0.0f)) || (t0 > dgFloat32 (1.0f))) {
					t0 = dgFloat32 (-1.0f);
				} else {
					contactOut = q;
					normalOut = dgVector (p0 + p1p0.Scale3 (t0) - q);
					normalOut = normalOut.Scale3 (dgRsqrt (normalOut % normalOut));
				}
			}
			return t0;
		}
	}

	dgFloat32 m_radius;
	dgInt32 m_count; 
	const dgVector* m_polygon; 
	dgBigVector m_normal; 

	dgInt32 m_prevVertex[32];
	dgInt32 m_nextVertex[32];
	dgInt32 m_edgeVoronoi[32];
	dgInt32 m_vertexVoronoi[32];
	dgInt32 m_vertexIndex[64];
};


dgFloat32 dgSweepLineToPolygonTimeOfImpact (const dgVector& p0, const dgVector& p1, dgFloat32 radius, dgInt32 count, const dgVector* const polygon, const dgVector& normal, dgVector& normalOut, dgVector& contactOut)
{
	dgVector diff (p1 - p0);
	dgFloat32 projectVeloc = normal % diff;
	if (projectVeloc >= dgFloat32 (0.0f)) {
		return dgFloat32 (-1.0f);
	}

	dgFloat32 planeSide = (p0 - polygon[0]) % normal;
	if ((planeSide) <= dgFloat32 (0.0f)) {
		return dgFloat32 (-1.0f);
	}

	dgSweepSphereToPolygon sweepSphere (count, polygon, normal);
	return sweepSphere.CalculateTimeOfImpact(p0, p1, radius, normalOut, contactOut);
}


dgFloat32 dgRayCastSphere (const dgVector& p0, const dgVector& p1, const dgVector& origin, dgFloat32 radius)
{
	dgVector p0Origin (p0 - origin);

	if ((p0Origin % p0Origin) < (100.0f * radius * radius)) {
		dgVector dp (p1 - p0);
		dgFloat32 a = dp % dp;
		dgFloat32 b = dgFloat32 (2.0f) * (p0Origin % dp);
		dgFloat32 c = p0Origin % p0Origin - radius * radius;
		dgFloat32 desc = b * b - dgFloat32 (4.0f) * a * c;
		if (desc >= 0.0f) {
			desc = dgSqrt (desc);
			dgFloat32 den = dgFloat32 (0.5f) / a;
			dgFloat32 t0 = (-b + desc) * den;
			dgFloat32 t1 = (-b - desc) * den;
			if ((t0 >= dgFloat32 (0.0f)) && (t1 >= dgFloat32 (0.0f))) {
				t0 =  dgMin(t0, t1);
				if (t0 <= dgFloat32 (1.0f)) {
					return t0;
				}
			} else if (t0 >= dgFloat32 (0.0f)) {
				if (t0 <= dgFloat32 (1.0f)) {
					return t0;
				}
			} else {
				if ((t1 >= dgFloat32 (0.0f)) && (t1 <= dgFloat32 (1.0f))) {
					return t1;
				}
			}
		}
	} else {
		dgBigVector p0Origin1 (p0Origin);
		dgBigVector dp (p1 - p0);
		dgFloat64 a = dp % dp;
		dgFloat64 b = dgFloat32 (2.0f) * (p0Origin1 % dp);
		dgFloat64 c = p0Origin1 % p0Origin1 - dgFloat64(radius) * radius;
		dgFloat64 desc = b * b - dgFloat32 (4.0f) * a * c;
		if (desc >= 0.0f) {
			desc = sqrt (desc);
			dgFloat64 den = dgFloat32 (0.5f) / a;
			dgFloat64 t0 = (-b + desc) * den;
			dgFloat64 t1 = (-b - desc) * den;
			if ((t0 >= dgFloat32 (0.0f)) && (t1 >= dgFloat32 (0.0f))) {
				t0 =  dgMin(t0, t1);
				if (t0 <= dgFloat32 (1.0f)) {
					return dgFloat32 (t0);
				}
			} else if (t0 >= dgFloat32 (0.0f)) {
				if (t0 <= dgFloat32 (1.0f)) {
					return dgFloat32 (t0);
				}
			} else {
				if ((t1 >= dgFloat32 (0.0f)) && (t1 <= dgFloat32 (1.0f))) {
					return dgFloat32 (t1);
				}
			}
		}
	}
	return dgFloat32 (1.2f);
}

dgFloat32 dgRayCastBox (const dgVector& p0, const dgVector& p1, const dgVector& boxP0, const dgVector& boxP1, dgVector& normalOut)
{
	dgInt32 index = 0;
	dgFloat32 signDir = dgFloat32 (0.0f);
	dgFloat32 tmin = dgFloat32 (0.0f);
	dgFloat32 tmax = dgFloat32 (1.0f);

	//dgVector size (boxP1 - boxP0);
	for (dgInt32 i = 0; i < 3; i++) {
		dgFloat32 dp = p1[i] - p0[i];
		if (dgAbsf (dp) < dgFloat32 (1.0e-8f)) {
			if (p0[i] <= boxP0[i] || p0[i] >= boxP1[i]) {
				return dgFloat32 (1.2f);
			}
		} else {
			dp = dgFloat32 (1.0f) / dp; 
			dgFloat32 t1 = (boxP0[i] - p0[i]) * dp;
			dgFloat32 t2 = (boxP1[i] - p0[i]) * dp;

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
		normalOut = dgVector (dgFloat32 (0.0f));
		normalOut[index] = signDir;
	} else {
		tmin = dgFloat32 (1.2f);
	}
	return tmin;

}

