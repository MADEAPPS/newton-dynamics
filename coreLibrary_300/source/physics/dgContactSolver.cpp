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
#include "dgCollisionMesh.h"
#include "dgCollisionConvex.h"
#include "dgCollisionInstance.h"
#include "dgCollisionConvexHull.h"
#include "dgCollisionConvexPolygon.h"


dgVector dgContactSolver::m_hullDirs[] =
{
	dgVector(dgFloat32(0.577350f), dgFloat32(-0.577350f), dgFloat32(0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(-0.577350f), dgFloat32(-0.577350f), dgFloat32(-0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.577350f), dgFloat32(-0.577350f), dgFloat32(-0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(-0.577350f), dgFloat32(0.577350f), dgFloat32(0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.577350f), dgFloat32(0.577350f), dgFloat32(-0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(-0.577350f), dgFloat32(0.577350f), dgFloat32(-0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(-0.577350f), dgFloat32(-0.577350f), dgFloat32(0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.577350f), dgFloat32(0.577350f), dgFloat32(0.577350f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.000000f), dgFloat32(-1.000000f), dgFloat32(0.000000f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.000000f), dgFloat32(1.000000f), dgFloat32(0.000000f), dgFloat32(0.0f)),
	dgVector(dgFloat32(1.000000f), dgFloat32(0.000000f), dgFloat32(0.000000f), dgFloat32(0.0f)),
	dgVector(dgFloat32(-1.000000f), dgFloat32(0.000000f), dgFloat32(0.000000f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.000000f), dgFloat32(0.000000f), dgFloat32(1.000000f), dgFloat32(0.0f)),
	dgVector(dgFloat32(0.000000f), dgFloat32(0.000000f), dgFloat32(-1.000000f), dgFloat32(0.0f)),
};

dgInt32 dgContactSolver::m_rayCastSimplex[4][4] =
{
	{ 0, 1, 2, 3 },
	{ 0, 2, 3, 1 },
	{ 2, 1, 3, 0 },
	{ 1, 0, 3, 2 },
};

dgContactSolver::dgContactSolver(dgCollisionInstance* const instance)
	:dgDownHeap<dgMinkFace*, dgFloat32>(m_heapBuffer, sizeof (m_heapBuffer))
	,m_proxy (NULL)
	,m_instance0(instance)
	,m_instance1(instance)
	,m_vertexIndex(0)
{
}

dgContactSolver::dgContactSolver(dgCollisionParamProxy* const proxy)
	:dgDownHeap<dgMinkFace*, dgFloat32>(m_heapBuffer, sizeof (m_heapBuffer))
	,m_normal (proxy->m_contactJoint->m_separtingVector)
	,m_proxy (proxy)
	,m_instance0(proxy->m_instance0)
	,m_instance1(proxy->m_instance1)
	,m_vertexIndex(0)
{
}

// for ray Cast

DG_INLINE void dgContactSolver::SupportVertex(const dgVector& dir0, dgInt32 vertexIndex)
{
	dgAssert(dir0.m_w == dgFloat32(0.0f));
	dgAssert(dgAbsf(dir0 % dir0 - dgFloat32(1.0f)) < dgFloat32(1.0e-3f));
	dgVector dir1 (dir0.Scale4(dgFloat32 (-1.0f)));

	const dgMatrix& matrix0 = m_instance0->m_globalMatrix;
	const dgMatrix& matrix1 = m_instance1->m_globalMatrix;
	dgVector p(matrix0.TransformVector(m_instance0->SupportVertexSpecial(matrix0.UnrotateVector (dir0), &m_polygonFaceIndex0[vertexIndex])));
	dgVector q(matrix1.TransformVector(m_instance1->SupportVertexSpecial(matrix1.UnrotateVector (dir1), &m_polygonFaceIndex1[vertexIndex])));
	p.m_w = dgFloat32(0.0f);
	q.m_w = dgFloat32(0.0f);
	m_hullDiff[vertexIndex] = p - q;
	m_hullSum[vertexIndex] = p + q;
}

DG_INLINE dgVector dgContactSolver::ReduceLine(dgInt32& indexOut)
{
	const dgVector& p0 = m_hullDiff[0];
	const dgVector& p1 = m_hullDiff[1];
	dgVector dp(p1 - p0);
	dgVector v;

	dgAssert(dp.m_w == dgFloat32(0.0f));
	dgFloat32 mag2 = dp.DotProduct4(dp).GetScalar();
	if (mag2 < dgFloat32(1.0e-24f)) {
		v = p0;
		indexOut = 1;
	} else {
		//dgFloat32 alpha0 = - (p0 % dp) / mag2;
		//if (alpha0 > dgFloat32 (1.0f)) {
		dgFloat32 alpha0 = -p0.DotProduct4(dp).GetScalar();
		if (alpha0 > mag2) {
			v = p1;
			indexOut = 1;
			m_hullSum[0] = m_hullSum[1];
			m_hullDiff[0] = m_hullDiff[1];
			m_polygonFaceIndex0[0] = m_polygonFaceIndex0[1];
			m_polygonFaceIndex1[0] = m_polygonFaceIndex1[1];
		} else if (alpha0 < dgFloat32(0.0f)) {
			v = p0;
			indexOut = 1;
		} else {
			v = p0 + dp.Scale4(alpha0 / mag2);
		}
	}
	return v;
}

DG_INLINE dgVector dgContactSolver::ReduceLineLarge(dgInt32& indexOut)
{
	dgBigVector p0(m_hullDiff[0]);
	dgBigVector p1(m_hullDiff[1]);
	dgBigVector dp(p1 - p0);
	dgBigVector v;

	dgFloat64 mag2 = dp % dp;
	if (mag2 < dgFloat32(1.0e-24f)) {
		v = p0;
		indexOut = 1;
	} else {
		//dgFloat64 alpha0 = - (p0 % dp) / mag2;
		//if (alpha0 > dgFloat32 (1.0f)) {
		dgFloat64 alpha0 = -(p0 % dp);
		if (alpha0 > mag2) {
			v = p1;
			indexOut = 1;
			m_hullSum[0] = m_hullSum[1];
			m_hullDiff[0] = m_hullDiff[1];
			m_polygonFaceIndex0[0] = m_polygonFaceIndex0[1];
			m_polygonFaceIndex1[0] = m_polygonFaceIndex1[1];
		} else if (alpha0 < dgFloat32(0.0f)) {
			v = p0;
			indexOut = 1;
		} else {
			v = p0 + dp.Scale4(alpha0 / mag2);
		}
	}
	return v;
}


DG_INLINE void dgContactSolver::ReduceDegeneratedTriangle()
{
	dgVector e10(m_hullDiff[1] - m_hullDiff[0]);
	if ((e10 % e10) < dgFloat32(1.0e-12f)) {
		m_hullSum[1] = m_hullSum[2];
		m_hullDiff[1] = m_hullDiff[2];
		m_polygonFaceIndex0[1] = m_polygonFaceIndex0[2];
		m_polygonFaceIndex1[1] = m_polygonFaceIndex1[2];

#ifdef _DEBUG
	} else {
		dgVector e21(m_hullDiff[2] - m_hullDiff[1]);
		dgVector e20(m_hullDiff[2] - m_hullDiff[0]);
		dgAssert(((e20 % e20) < dgFloat32(1.0e-8f)) || ((e21 % e21) < dgFloat32(1.0e-8f)));
#endif
	}
}


DG_INLINE dgVector dgContactSolver::ReduceTriangle(dgInt32& indexOut)
{
	dgVector e10(m_hullDiff[1] - m_hullDiff[0]);
	dgVector e20(m_hullDiff[2] - m_hullDiff[0]);
	dgVector normal(e10 * e20);
	dgAssert(normal.m_w == dgFloat32(0.0f));
	//if ((normal % normal) > dgFloat32 (1.0e-14f)) {
	if (normal.DotProduct4(normal).GetScalar() > dgFloat32(1.0e-14f)) {
		dgInt32 i0 = 2;
		dgInt32 minIndex = -1;
		for (dgInt32 i1 = 0; i1 < 3; i1++) {
			const dgVector& p1p0 = m_hullDiff[i0];
			const dgVector& p2p0 = m_hullDiff[i1];

			//dgFloat32 volume = (p1p0 * p2p0) % normal;
			dgFloat32 volume = normal.DotProduct4(p1p0 * p2p0).GetScalar();
			if (volume < dgFloat32(0.0f)) {
				dgVector segment(m_hullDiff[i1] - m_hullDiff[i0]);
				dgAssert(segment.m_w == dgFloat32(0.0f));
				dgVector poinP0(m_hullDiff[i0].CompProduct4(dgVector::m_negOne));
				dgFloat32 den = segment.DotProduct4(segment).GetScalar();
				dgAssert(den > dgFloat32(0.0f));
				dgFloat32 num = poinP0.DotProduct4(segment).GetScalar();
				if (num < dgFloat32(0.0f)) {
					minIndex = i0;
				} else if (num > den) {
					minIndex = i1;
				} else {
					indexOut = 2;
					dgVector tmp0(m_hullDiff[i0]);
					dgVector tmp1(m_hullDiff[i1]);
					m_hullDiff[0] = tmp0;
					m_hullDiff[1] = tmp1;

					tmp0 = m_hullSum[i0];
					tmp1 = m_hullSum[i1];
					m_hullSum[0] = tmp0;
					m_hullSum[1] = tmp1;

					dgInt32 j0 = m_polygonFaceIndex0[i0];
					dgInt32 j1 = m_polygonFaceIndex0[i1];
					m_polygonFaceIndex0[0] = j0;
					m_polygonFaceIndex0[1] = j1;

					j0 = m_polygonFaceIndex1[i0];
					j1 = m_polygonFaceIndex1[i1];
					m_polygonFaceIndex1[0] = j0;
					m_polygonFaceIndex1[1] = j1;

					return m_hullDiff[0] + segment.Scale4(num / den);
				}
			}
			i0 = i1;
		}
		if (minIndex != -1) {
			indexOut = 1;
			m_polygonFaceIndex0[0] = m_polygonFaceIndex0[minIndex];
			m_polygonFaceIndex1[0] = m_polygonFaceIndex1[minIndex];
			m_hullSum[0] = m_hullSum[minIndex];
			m_hullDiff[0] = m_hullDiff[minIndex];
			return m_hullDiff[0];
		} else {
			indexOut = 3;
			return normal.Scale4(normal.DotProduct4(m_hullDiff[0]).GetScalar() / normal.DotProduct4(normal).GetScalar());
		}
	} else {
		indexOut = 2;
		ReduceDegeneratedTriangle();
		return ReduceLine(indexOut);
	}
}

DG_INLINE dgVector dgContactSolver::ReduceTriangleLarge (dgInt32& indexOut)
{
	dgBigVector triangleDiffExtended[3];
	triangleDiffExtended[0] = m_hullDiff[0];
	triangleDiffExtended[1] = m_hullDiff[1];
	triangleDiffExtended[2] = m_hullDiff[2];

	dgBigVector e10 (triangleDiffExtended[1] - triangleDiffExtended[0]);
	dgBigVector e20 (triangleDiffExtended[2] - triangleDiffExtended[0]);
	dgBigVector normal (e10 * e20);
	if ((normal % normal) > dgFloat32 (1.0e-14f)) {
		dgInt32 i0 = 2;
		dgInt32 minIndex = -1;
		for (dgInt32 i1 = 0; i1 < 3; i1 ++) {
			dgBigVector p1p0 (triangleDiffExtended[i0]);
			dgBigVector p2p0 (triangleDiffExtended[i1]);

			dgFloat64 volume = (p1p0 * p2p0) % normal;
			if (volume < dgFloat32 (0.0f)) {
				dgBigVector segment (triangleDiffExtended[i1] - triangleDiffExtended[i0]);
				dgBigVector poinP0 (triangleDiffExtended[i0].Scale3 (dgFloat32 (-1.0f)));
				dgFloat64 den = segment % segment;
				dgAssert (den > dgFloat32 (0.0f));
				dgFloat64 num = poinP0 % segment;
				if (num < dgFloat32 (0.0f)) {
					minIndex = i0;
				} else if (num > den){
					minIndex = i1;
				} else {
					indexOut = 2;
					dgBigVector tmp0 (triangleDiffExtended[i0]);
					dgBigVector tmp1 (triangleDiffExtended[i1]);
					triangleDiffExtended[0] = tmp0;
					triangleDiffExtended[1] = tmp1;
					m_hullDiff[0] = tmp0;
					m_hullDiff[1] = tmp1;

					tmp0 = m_hullSum[i0];
					tmp1 = m_hullSum[i1];
					m_hullSum[0] = tmp0;
					m_hullSum[1] = tmp1;

					dgInt32 j0 = m_polygonFaceIndex0[i0];
					dgInt32 j1 = m_polygonFaceIndex0[i1];
					m_polygonFaceIndex0[0] = j0;
					m_polygonFaceIndex0[1] = j1;

					j0 = m_polygonFaceIndex1[i0];
					j1 = m_polygonFaceIndex1[i1];
					m_polygonFaceIndex1[0] = j0;
					m_polygonFaceIndex1[1] = j1;

					return dgVector (triangleDiffExtended[0] + segment.Scale3 (num / den));
				}
			}
			i0 = i1;
		}

		if (minIndex != -1) {
			indexOut = 1;
			m_polygonFaceIndex0[0] = m_polygonFaceIndex0[minIndex];
			m_polygonFaceIndex1[0] = m_polygonFaceIndex1[minIndex];
			m_hullSum[0] = m_hullSum[minIndex];
			m_hullDiff[0] = dgVector (m_hullDiff[minIndex]);
			//triangleDiffExtended[0] = triangleDiffExtended[minIndex];
			return m_hullDiff[0];
		} else {
			indexOut = 3;
			return dgVector (normal.Scale3((normal % triangleDiffExtended[0]) / (normal % normal)));
		}
	} else {
		indexOut = 2;
		ReduceDegeneratedTriangle ();
		return ReduceLineLarge (indexOut);
	}
}

DG_INLINE dgVector dgContactSolver::ReduceTetrahedrum(dgInt32& indexOut)
{
	dgInt32 i0 = m_rayCastSimplex[0][0];
	dgInt32 i1 = m_rayCastSimplex[0][1];
	dgInt32 i2 = m_rayCastSimplex[0][2];
	dgInt32 i3 = m_rayCastSimplex[0][3];
	const dgVector& p0 = m_hullDiff[i0];
	const dgVector& p1 = m_hullDiff[i1];
	const dgVector& p2 = m_hullDiff[i2];
	const dgVector& p3 = m_hullDiff[i3];

	dgVector p10(p1 - p0);
	dgVector p20(p2 - p0);
	dgVector p30(p3 - p0);
	dgVector n(p10 * p20);
	dgAssert(n.m_w == dgFloat32(0.0f));
	//dgFloat32 volume = p30 % n;
	dgFloat32 volume = n.DotProduct4(p30).GetScalar();
	if (volume < dgFloat32(0.0f)) {
		volume = -volume;
		dgSwap(m_hullSum[i2], m_hullSum[i3]);
		dgSwap(m_hullDiff[i2], m_hullDiff[i3]);
		dgSwap(m_polygonFaceIndex0[i2], m_polygonFaceIndex0[i3]);
		dgSwap(m_polygonFaceIndex1[i2], m_polygonFaceIndex1[i3]);
	}
	if (volume < dgFloat32(1.0e-8f)) {
		dgTrace(("very import to finish this\n"));
		//		dgAssert (0);
	}

	dgInt32 faceIndex = -1;
	dgFloat32 minDist = dgFloat32(dgFloat32(0.0f));
	const dgVector origin(dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f));
	for (dgInt32 i = 0; i < 4; i++) {
		dgInt32 i0 = m_rayCastSimplex[i][0];
		dgInt32 i1 = m_rayCastSimplex[i][1];
		dgInt32 i2 = m_rayCastSimplex[i][2];

		const dgVector& p0 = m_hullDiff[i0];
		const dgVector& p1 = m_hullDiff[i1];
		const dgVector& p2 = m_hullDiff[i2];

		dgVector p10(p1 - p0);
		dgVector p20(p2 - p0);
		dgVector normal(p10 * p20);
		dgAssert(normal.m_w == dgFloat32(0.0f));

		//dgFloat32 area = normal % normal;
		//dgAssert (dgAbsf (area) > dgFloat32 (0.0f));
		//normal = normal.Scale3 (dgRsqrt (area));
		dgVector normalDot4(normal.DotProduct4(normal));
		dgAssert(normalDot4.GetScalar() > dgFloat32(0.0f));
		if (normalDot4.GetScalar() > dgFloat32(1.0e-24f)) {
			normal = normal.CompProduct4(normalDot4.InvSqrt());
			//dgFloat32 dist = normal % (origin - p0);
			dgFloat32 dist = normal.DotProduct4(origin - p0).GetScalar();
			if (dist <= minDist) {
				minDist = dist;
				faceIndex = i;
			}
		}
	}

	if (faceIndex != -1) {
		dgVector tmp[3];
		dgInt32 i0 = m_rayCastSimplex[faceIndex][0];
		dgInt32 i1 = m_rayCastSimplex[faceIndex][1];
		dgInt32 i2 = m_rayCastSimplex[faceIndex][2];

		tmp[0] = m_hullSum[i0];
		tmp[1] = m_hullSum[i1];
		tmp[2] = m_hullSum[i2];
		m_hullSum[0] = tmp[0];
		m_hullSum[1] = tmp[1];
		m_hullSum[2] = tmp[2];

		tmp[0] = m_hullDiff[i0];
		tmp[1] = m_hullDiff[i1];
		tmp[2] = m_hullDiff[i2];
		m_hullDiff[0] = tmp[0];
		m_hullDiff[1] = tmp[1];
		m_hullDiff[2] = tmp[2];

		dgInt32 j0 = m_polygonFaceIndex0[i0];
		dgInt32 j1 = m_polygonFaceIndex0[i1];
		dgInt32 j2 = m_polygonFaceIndex0[i2];
		m_polygonFaceIndex0[0] = j0;
		m_polygonFaceIndex0[1] = j1;
		m_polygonFaceIndex0[2] = j2;

		j0 = m_polygonFaceIndex1[i0];
		j1 = m_polygonFaceIndex1[i1];
		j2 = m_polygonFaceIndex1[i2];
		m_polygonFaceIndex1[0] = j0;
		m_polygonFaceIndex1[1] = j1;
		m_polygonFaceIndex1[2] = j2;

		indexOut = 3;
		return ReduceTriangle(indexOut);
	}

	indexOut = 4;
	return origin;
}

DG_INLINE dgVector dgContactSolver::ReduceTetrahedrumLarge (dgInt32& indexOut)
{
	dgBigVector tetraDiffExtended[4];
	tetraDiffExtended[0] = m_hullDiff[0];
	tetraDiffExtended[1] = m_hullDiff[1];
	tetraDiffExtended[2] = m_hullDiff[2];
	tetraDiffExtended[3] = m_hullDiff[3];

	dgInt32 i0 = m_rayCastSimplex[0][0];
	dgInt32 i1 = m_rayCastSimplex[0][1];
	dgInt32 i2 = m_rayCastSimplex[0][2];
	dgInt32 i3 = m_rayCastSimplex[0][3];
	const dgBigVector& p0 = tetraDiffExtended[i0];
	const dgBigVector& p1 = tetraDiffExtended[i1]; 
	const dgBigVector& p2 = tetraDiffExtended[i2];
	const dgBigVector& p3 = tetraDiffExtended[i3];

	dgBigVector p10 (p1 - p0);
	dgBigVector p20 (p2 - p0);
	dgBigVector p30 (p3 - p0);
	dgFloat64 volume = p30 % (p10 * p20);
	if (volume < dgFloat32 (0.0f)) {
		volume = -volume;
		dgSwap (m_hullSum[i2], m_hullSum[i3]);
		dgSwap (m_hullDiff[i2], m_hullDiff[i3]);
		dgSwap (m_polygonFaceIndex0[i2], m_polygonFaceIndex0[i3]);
		dgSwap (m_polygonFaceIndex1[i2], m_polygonFaceIndex1[i3]);
		dgSwap (tetraDiffExtended[i2], tetraDiffExtended[i3]);
	}
	if (volume < dgFloat32 (1.0e-8f)) {
		dgAssert (0);
	}

	dgInt32 faceIndex = -1;
	dgFloat64 minDist = dgFloat32 (0.0f);
	const dgBigVector origin (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < 4; i ++) {
		dgInt32 i0 = m_rayCastSimplex[i][0];
		dgInt32 i1 = m_rayCastSimplex[i][1];
		dgInt32 i2 = m_rayCastSimplex[i][2];

		const dgBigVector& p0 = tetraDiffExtended[i0];
		const dgBigVector& p1 = tetraDiffExtended[i1]; 
		const dgBigVector& p2 = tetraDiffExtended[i2];

		dgBigVector p10 (p1 - p0);
		dgBigVector p20 (p2 - p0);
		dgBigVector normal (p10 * p20);
		dgFloat64 area = normal % normal;
		dgAssert (fabs (area) > dgFloat32 (0.0f));
		normal = normal.Scale3 (dgFloat32 (1.0f) / sqrt (area));
		dgFloat64 dist = normal % (origin - p0);
		if (dist < minDist) {
			minDist = dist;
			faceIndex = i;
		}
	}

	if (faceIndex != -1) {
		dgVector tmp[3];
		dgInt32 i0 = m_rayCastSimplex[faceIndex][0];
		dgInt32 i1 = m_rayCastSimplex[faceIndex][1];
		dgInt32 i2 = m_rayCastSimplex[faceIndex][2];

		tmp[0] = m_hullSum[i0];
		tmp[1] = m_hullSum[i1];
		tmp[2] = m_hullSum[i2];
		m_hullSum[0] = tmp[0];
		m_hullSum[1] = tmp[1];
		m_hullSum[2] = tmp[2];

		tmp[0] = m_hullDiff[i0];
		tmp[1] = m_hullDiff[i1];
		tmp[2] = m_hullDiff[i2];
		m_hullDiff[0] = tmp[0];
		m_hullDiff[1] = tmp[1];
		m_hullDiff[2] = tmp[2];

		dgInt32 j0 = m_polygonFaceIndex0[i0];
		dgInt32 j1 = m_polygonFaceIndex0[i1];
		dgInt32 j2 = m_polygonFaceIndex0[i2];
		m_polygonFaceIndex0[0] = j0;
		m_polygonFaceIndex0[1] = j1;
		m_polygonFaceIndex0[2] = j2;

		j0 = m_polygonFaceIndex1[i0];
		j1 = m_polygonFaceIndex1[i1];
		j2 = m_polygonFaceIndex1[i2];
		m_polygonFaceIndex1[0] = j0;
		m_polygonFaceIndex1[1] = j1;
		m_polygonFaceIndex1[2] = j2;

		indexOut = 3;
		return ReduceTriangleLarge (indexOut);
	}

	indexOut = 4;
	return origin;
}


DG_INLINE dgInt32 dgContactSolver::CalculateClosestSimplex()
{
	dgVector v(dgFloat32(0.0f));
	dgInt32 index = 1;
	if (m_vertexIndex <= 0) {
		SupportVertex(m_proxy->m_contactJoint->m_separtingVector, 0);
		v = m_hullDiff[0];
	} else {
		switch (m_vertexIndex) 
		{
			case 1:
			{
				  v = m_hullDiff[0];
				  break;
			}

			case 2:
			{
				  v = ReduceLine(m_vertexIndex);
				  break;
			}

			case 3:
			{
				  v = ReduceTriangle(m_vertexIndex);
				  break;
			}

			case 4:
			{
				  v = ReduceTetrahedrum(m_vertexIndex);
				  break;
			}
		}
		index = m_vertexIndex;
	}

	dgInt32 iter = 0;
	dgInt32 cycling = 0;
	dgFloat32 minDist = dgFloat32(1.0e20f);
	do {
		dgFloat32 dist = v % v;
		if (dist < dgFloat32(1.0e-9f)) {
			// very deep penetration, resolve with generic Minkowski solver
			return -index;
		}

		if (dist < minDist) {
			minDist = dist;
			cycling = -1;
		}
		cycling++;
		if (cycling > 4) {
			// for now return -1
			return -index;
		}

		dgVector dir(v.Scale4(-dgRsqrt(dist)));
		SupportVertex(dir, index);

		const dgVector& w = m_hullDiff[index];
		dgVector wv(w - v);
		dist = dir % wv;
		if (dist < dgFloat32(1.0e-3f)) {
			m_normal = dir;
			break;
		}

		index++;
		switch (index) 
		{
			case 2:
			{
				v = ReduceLine(index);
				break;
			}

			case 3:
			{
				v = ReduceTriangle(index);
				break;
			}

			case 4:
			{
				v = ReduceTetrahedrum(index);
				break;
			}
		}

		iter++;
		cycling++;
	} while (iter < DG_CONNICS_CONTATS_ITERATIONS);

	return (index < 4) ? index : -4;
}

DG_INLINE dgInt32 dgContactSolver::CalculateClosestSimplexLarge ()
{
	dgVector v(dgFloat32 (0.0f));
	dgInt32 index = 1;
	if (m_vertexIndex <= 0) {
		SupportVertex (m_proxy->m_contactJoint->m_separtingVector, 0);
		v = m_hullDiff[0];
	} else {
		switch (m_vertexIndex) 
		{
			case 1:
			{
				v = m_hullDiff[0];
				break;
			}

			case 2:
			{
				v = ReduceLineLarge (m_vertexIndex);
				break;
			}

			case 3:
			{
				v = ReduceTriangleLarge (m_vertexIndex);
				break;
			}

			case 4:
			{
				v = ReduceTetrahedrumLarge (m_vertexIndex);
				break;
			}
		}
		index = m_vertexIndex;
	}

	dgInt32 iter = 0;
	dgInt32 cycling = 0;
	dgFloat32 minDist = dgFloat32 (1.0e20f);
	do {

		dgFloat32 dist = v % v;
		if (dist < dgFloat32 (1.0e-9f)) {
			// very deep penetration, resolve with generic minkosky solver
			return -index; 
		}

		if (dist < minDist) {
			minDist = dist;
			cycling = -1;
		}
		cycling ++;
		if (cycling > 4) {
			// for now return -1
			return -index;
		}

		dgVector dir (v.Scale3 (-dgRsqrt(dist)));
		SupportVertex (dir, index);

		const dgVector& w = m_hullDiff[index];
		dgVector wv (w - v);
		dist = dir % wv;
		if (dist < dgFloat32 (1.0e-3f)) {
			m_normal = dir;
			break;
		}

		index ++;
		switch (index) 
		{
			case 2:
			{
				v = ReduceLineLarge (index);
				break;
			}

			case 3:
			{
				v = ReduceTriangleLarge (index);
				break;
			}

			case 4:
			{
				v = ReduceTetrahedrumLarge (index);
				break;
			}
		}

		iter ++;
		cycling ++;
	} while (iter < DG_CONNICS_CONTATS_ITERATIONS); 
	return (index < 4) ? index : -4;
}


DG_INLINE dgMinkFace* dgContactSolver::NewFace()
{
	dgMinkFace* face = (dgMinkFace*)m_freeFace;
	if (m_freeFace) {
		m_freeFace = m_freeFace->m_next;
	} else {
		face = &m_facePool[m_faceIndex];
		m_faceIndex++;
		dgAssert(m_faceIndex < DG_CONVEX_MINK_MAX_FACES);
	}

#ifdef _DEBUG
	memset(face, 0, sizeof (dgMinkFace));
#endif
	return face;
}

DG_INLINE dgMinkFace* dgContactSolver::AddFace(dgInt32 v0, dgInt32 v1, dgInt32 v2)
{
	dgMinkFace* const face = NewFace();
	face->m_mark = 0;
	face->m_vertex[0] = dgInt16(v0);
	face->m_vertex[1] = dgInt16(v1);
	face->m_vertex[2] = dgInt16(v2);
	return face;
}

DG_INLINE void dgContactSolver::DeleteFace(dgMinkFace* const face)
{
	dgFaceFreeList* const freeFace = (dgFaceFreeList*)face;
	freeFace->m_next = m_freeFace;
	m_freeFace = freeFace;
}


DG_INLINE void dgContactSolver::PushFace(dgMinkFace* const face)
{
	dgInt32 i0 = face->m_vertex[0];
	dgInt32 i1 = face->m_vertex[1];
	dgInt32 i2 = face->m_vertex[2];

	dgPlane plane(m_hullDiff[i0], m_hullDiff[i1], m_hullDiff[i2]);
	dgFloat32 mag2 = plane % plane;
	face->m_alive = 1;
	if (mag2 > dgFloat32(1.0e-16f)) {
		face->m_plane = plane.Scale(dgRsqrt(mag2));
		dgMinkFace* face1 = face;
		Push(face1, face->m_plane.m_w);
	} else {
		face->m_plane = dgPlane(dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f));
	}
}

DG_INLINE dgInt32 dgContactSolver::CalculateIntersectingPlane(dgInt32 count)
{
	dgAssert(count >= 1);
	if (count == 1) {
		SupportVertex(m_proxy->m_contactJoint->m_separtingVector.Scale4(dgFloat32(-1.0f)), 1);
#ifdef _DEBUG
		dgVector err(m_hullDiff[1] - m_hullDiff[0]);
		dgAssert((err % err) > dgFloat32(0.0f));
#endif
		count = 2;
	}
	
	if (count == 2) {
		dgVector e0(m_hullDiff[1] - m_hullDiff[0]);
		dgAssert((e0 % e0) > dgFloat32(0.0f));
		dgMatrix matrix(e0.Scale4(dgRsqrt(e0 % e0)));
		dgMatrix rotation(dgPitchMatrix(dgFloat32(45.0f * 3.141592f / 180.0f)));
		dgFloat32 maxArea = dgFloat32(0.0f);
		for (dgInt32 i = 0; i < 8; i++) {
			SupportVertex(matrix[1], 3);
			dgVector e1(m_hullDiff[3] - m_hullDiff[0]);
			dgVector area(e0 * e1);
			dgFloat32 area2 = area % area;
			if (area2 > maxArea) {
				m_hullSum[2] = m_hullSum[3];
				m_hullDiff[2] = m_hullDiff[3];
				m_polygonFaceIndex0[2] = m_polygonFaceIndex0[3];
				m_polygonFaceIndex1[2] = m_polygonFaceIndex1[3];
				maxArea = area2;
			}
			matrix = rotation * matrix;
		}
		if (dgAbsf (maxArea) < dgFloat32(1e-15f)) {
			return -1;
		}
		dgAssert(maxArea > dgFloat32(0.0f));
		count++;
	}

	dgFloat32 volume = dgFloat32(0.0f);
	if (count == 3) {
		dgVector e10(m_hullDiff[1] - m_hullDiff[0]);
		dgVector e20(m_hullDiff[2] - m_hullDiff[0]);
		dgVector normal(e10 * e20);
		dgFloat32 mag2 = normal % normal;
		dgAssert(mag2 > dgFloat32(0.0f));
		normal = normal.Scale4(dgRsqrt(mag2));
		SupportVertex(normal, 3);
		volume = (m_hullDiff[3] - m_hullDiff[0]) % normal;
		if (dgAbsf(volume) < dgFloat32(1.0e-10f)) {
			normal = normal.Scale4(dgFloat32(-1.0f));
			SupportVertex(normal, 3);
			volume = -((m_hullDiff[3] - m_hullDiff[0]) % normal);
			if (dgAbsf(volume) < dgFloat32(1.0e-10f)) {
				volume = dgFloat32(0.0f);
			}
		}
		count = 4;
	} else if (count == 4) {
		dgVector e0(m_hullDiff[1] - m_hullDiff[0]);
		dgVector e1(m_hullDiff[2] - m_hullDiff[0]);
		dgVector e2(m_hullDiff[3] - m_hullDiff[0]);
		dgVector n(e1 * e2);
		volume = e0 % n;
	}


	dgAssert(count == 4);
	if (volume > dgFloat32(0.0f)) {
		dgSwap(m_hullSum[1], m_hullSum[0]);
		dgSwap(m_hullDiff[1], m_hullDiff[0]);
		dgSwap(m_polygonFaceIndex0[1], m_polygonFaceIndex0[0]);
		dgSwap(m_polygonFaceIndex1[1], m_polygonFaceIndex1[0]);
	}

	if (dgAbsf(volume) < dgFloat32(1e-15f)) {

		// this volume is unrealizable, let us build  a different tetrahedron using the method of core 200
		dgVector e1;
		dgVector e2;
		dgVector e3;
		dgVector normal(dgFloat32(0.0f));

		const dgInt32 nCount = dgInt32(sizeof(m_hullDirs) / sizeof(m_hullDirs[0]));
		const dgFloat32 DG_CALCULATE_SEPARATING_PLANE_ERROR = dgFloat32(1.0f / 1024.0f);

		dgFloat32 error2 = dgFloat32(0.0f);
		SupportVertex(m_hullDirs[0], 0);

		dgInt32 i = 1;
		for (; i < nCount; i++) {
			SupportVertex(m_hullDirs[i], 1);
			e1 = m_hullDiff[1] - m_hullDiff[0];
			error2 = e1 % e1;
			if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR) {
				break;
			}
		}

		for (i++; i < nCount; i++) {
			SupportVertex(m_hullDirs[i], 2);
			e2 = m_hullDiff[2] - m_hullDiff[0];
			normal = e1 * e2;
			error2 = normal % normal;
			if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR) {
				break;
			}
		}

		error2 = dgFloat32(0.0f);
		for (i++; i < nCount; i++) {
			SupportVertex(m_hullDirs[i], 3);
			e3 = m_hullDiff[3] - m_hullDiff[0];
			error2 = normal % e3;
			if (dgAbsf(error2) > DG_CALCULATE_SEPARATING_PLANE_ERROR) {
				break;
			}
		}

		if (i >= nCount) {
			dgAssert(0);
			return -1;
		}

		if (error2 > dgFloat32(0.0f)) {
			dgSwap(m_hullSum[1], m_hullSum[2]);
			dgSwap(m_hullDiff[1], m_hullDiff[2]);
			dgSwap(m_polygonFaceIndex0[1], m_polygonFaceIndex0[2]);
			dgSwap(m_polygonFaceIndex1[1], m_polygonFaceIndex1[2]);
		}

#ifdef _DEBUG
		{
			dgVector e0(m_hullDiff[1] - m_hullDiff[0]);
			dgVector e1(m_hullDiff[2] - m_hullDiff[0]);
			dgVector e2(m_hullDiff[3] - m_hullDiff[0]);
			dgVector n(e1 * e2);
			dgFloat32 volume = e0 % n;
			dgAssert(volume < dgFloat32(0.0f));
		}
#endif
	}

	// clear the face cache!!
	Flush();
	m_faceIndex = 0;
	m_vertexIndex = 4;
	m_freeFace = NULL;

	dgMinkFace* const f0 = AddFace(0, 1, 2);
	dgMinkFace* const f1 = AddFace(0, 2, 3);
	dgMinkFace* const f2 = AddFace(2, 1, 3);
	dgMinkFace* const f3 = AddFace(1, 0, 3);

	f0->m_twin[0] = f3;
	f0->m_twin[1] = f2;
	f0->m_twin[2] = f1;

	f1->m_twin[0] = f0;
	f1->m_twin[1] = f2;
	f1->m_twin[2] = f3;

	f2->m_twin[0] = f0;
	f2->m_twin[1] = f3;
	f2->m_twin[2] = f1;

	f3->m_twin[0] = f0;
	f3->m_twin[1] = f1;
	f3->m_twin[2] = f2;

	PushFace(f0);
	PushFace(f1);
	PushFace(f2);
	PushFace(f3);

	dgInt32 iterCount = 0;
	dgInt32 cycling = 0;
	dgFloat32 cyclingMem[4];
	cyclingMem[0] = dgFloat32(1.0e10f);
	cyclingMem[1] = dgFloat32(1.0e10f);
	cyclingMem[2] = dgFloat32(1.0e10f);
	cyclingMem[3] = dgFloat32(1.0e10f);

	const dgFloat32 resolutionScale = dgFloat32(0.125f);
	//dgFloat32 minTolerance = dgFloat32 (DG_IMPULSIVE_CONTACT_PENETRATION / dgFloat32 (4.0f));
	const dgFloat32 minTolerance = dgFloat32(DG_RESTING_CONTACT_PENETRATION * dgFloat32(0.5f));

	while (GetCount()) {
		dgMinkFace* const faceNode = (*this)[0];
		Pop();

		if (faceNode->m_alive) {

			//SaveOFF ("xxx.off");

			SupportVertex(faceNode->m_plane & dgVector::m_triplexMask, m_vertexIndex);
			const dgVector& p = m_hullDiff[m_vertexIndex];
			dgFloat32 dist = faceNode->m_plane.Evalue(p);
			dgFloat32 distTolerance = dgMax(dgAbsf(faceNode->m_plane.m_w) * resolutionScale, minTolerance);

			if (dist < distTolerance) {
				//if (xxx > 1000){
				//	SaveOFF ("xxx.off");
				//}
				dgVector sum[3];
				dgVector diff[3];
				dgInt32 index0[3];
				dgInt32 index1[3];
				m_normal = faceNode->m_plane & dgVector::m_triplexMask;
				for (dgInt32 i = 0; i < 3; i++) {
					dgInt32 j = faceNode->m_vertex[i];
					sum[i] = m_hullSum[j];
					diff[i] = m_hullDiff[j];
					index0[i] = m_polygonFaceIndex0[j];
					index1[i] = m_polygonFaceIndex1[j];
				}
				for (dgInt32 i = 0; i < 3; i++) {
					m_hullSum[i] = sum[i];
					m_hullDiff[i] = diff[i];
					m_polygonFaceIndex0[i] = index0[i];
					m_polygonFaceIndex1[i] = index1[i];
				}
				return 3;
			}

			iterCount++;
			bool isCycling = false;
			cyclingMem[cycling] = dist;
			if (iterCount > 10) {
				dgInt32 cyclingIndex = cycling;
				for (dgInt32 i = 0; i < 3; i++) {
					dgInt32 cyclingIndex0 = (cyclingIndex - 1) & 3;
					if (((cyclingMem[cyclingIndex0] - cyclingMem[cyclingIndex]) < dgFloat32(-1.0e-5f))) {
						isCycling = true;
						cyclingMem[0] = dgFloat32(1.0e10f);
						cyclingMem[1] = dgFloat32(1.0e10f);
						cyclingMem[2] = dgFloat32(1.0e10f);
						cyclingMem[3] = dgFloat32(1.0e10f);
						break;
					}
					cyclingIndex = cyclingIndex0;
				}
			}
			cycling = (cycling + 1) & 3;

			if (!isCycling) {
				m_faceStack[0] = faceNode;
				dgInt32 stackIndex = 1;
				dgInt32 deletedCount = 0;

				while (stackIndex) {
					stackIndex--;
					dgMinkFace* const face = m_faceStack[stackIndex];

					if (!face->m_mark && (face->m_plane.Evalue(p) > dgFloat32(0.0f))) {
#ifdef _DEBUG
						for (dgInt32 i = 0; i < deletedCount; i++) {
							dgAssert(m_deletedFaceList[i] != face);
						}
#endif

						m_deletedFaceList[deletedCount] = face;
						deletedCount++;
						dgAssert(deletedCount < sizeof (m_deletedFaceList) / sizeof (m_deletedFaceList[0]));
						face->m_mark = 1;

						for (dgInt32 i = 0; i < 3; i++) {
							dgMinkFace* const twinFace = face->m_twin[i];
							if (!twinFace->m_mark) {
								m_faceStack[stackIndex] = twinFace;
								stackIndex++;
								dgAssert(stackIndex < sizeof (m_faceStack) / sizeof (m_faceStack[0]));
							}
						}
					}
				}

				//dgAssert (SanityCheck());
				dgInt32 newCount = 0;
				for (dgInt32 i = 0; i < deletedCount; i++) {
					dgMinkFace* const face = m_deletedFaceList[i];
					face->m_alive = 0;
					dgAssert(face->m_mark == 1);
					dgInt32 j0 = 2;
					for (dgInt32 j1 = 0; j1 < 3; j1++) {
						dgMinkFace* const twinFace = face->m_twin[j0];
						if (!twinFace->m_mark) {
							dgMinkFace* const newFace = AddFace(m_vertexIndex, face->m_vertex[j0], face->m_vertex[j1]);
							PushFace(newFace);

							newFace->m_twin[1] = twinFace;
							dgInt32 index = (twinFace->m_twin[0] == face) ? 0 : ((twinFace->m_twin[1] == face) ? 1 : 2);
							twinFace->m_twin[index] = newFace;

							m_coneFaceList[newCount] = newFace;
							newCount++;
							dgAssert(newCount < sizeof (m_coneFaceList) / sizeof (m_coneFaceList[0]));
						}
						j0 = j1;
					}
				}

				dgInt32 i0 = newCount - 1;
				for (dgInt32 i1 = 0; i1 < newCount; i1++) {
					dgMinkFace* const faceA = m_coneFaceList[i0];
					dgAssert(faceA->m_mark == 0);

					dgInt32 j0 = newCount - 1;
					for (dgInt32 j1 = 0; j1 < newCount; j1++) {
						if (i0 != j0) {
							dgMinkFace* const faceB = m_coneFaceList[j0];
							dgAssert(faceB->m_mark == 0);
							if (faceA->m_vertex[2] == faceB->m_vertex[1]) {
								faceA->m_twin[2] = faceB;
								faceB->m_twin[0] = faceA;
								break;
							}
						}
						j0 = j1;
					}
					i0 = i1;
				}

				m_vertexIndex++;
				dgAssert(m_vertexIndex < sizeof (m_hullDiff) / sizeof (m_hullDiff[0]));

				dgAssert(SanityCheck());
			}
		} else {
			DeleteFace(faceNode);
		}
	}
	
	return -1;
}

DG_INLINE void dgContactSolver::CalculateContactFromFeacture(dgInt32 featureType)
{
	dgVector d;
	dgVector s;
	switch (featureType) 
	{
		case 1:
		{
			s = m_hullSum[0];
			d = m_hullDiff[0];
			break;
		}
		case 2:
		{
			const dgVector& p0 = m_hullDiff[0];
			const dgVector& p1 = m_hullDiff[1];
			dgVector dp(p1 - p0);
			dgAssert(dp % dp > dgFloat32(0.0f));
			dgFloat32 alpha0 = -(p0 % dp) / (dp % dp);
			dgAssert(alpha0 <= dgFloat32(1.01f));
			dgAssert(alpha0 >= dgFloat32(-0.01f));
			d = p0 + dp.Scale4(alpha0);
			s = m_hullSum[0] + (m_hullSum[1] - m_hullSum[0]).Scale4(alpha0);
			break;
		}

		case 3:
		default:
		{
			dgVector e10(m_hullDiff[1] - m_hullDiff[0]);
			dgVector e20(m_hullDiff[2] - m_hullDiff[0]);
			dgVector normal(e10 * e20);
			dgAssert((normal % normal) > dgFloat32(0.0f));

			dgInt32 i0 = 2;
			dgFloat32 alphas[3];
			for (dgInt32 i1 = 0; i1 < 3; i1++) {
				const dgVector& p1p0 = m_hullDiff[i0];
				const dgVector& p2p0 = m_hullDiff[i1];
				alphas[i0] = (p1p0 * p2p0) % normal;
				i0 = i1;
			}

			dgFloat32 alphaDen = alphas[0] + alphas[1] + alphas[2];
			if (alphaDen > dgFloat32(1.0e-16f)) {
				dgAssert(alphaDen > dgFloat32(0.0f));

				alphaDen = dgFloat32(1.0f / alphaDen);
				alphas[0] *= alphaDen;
				alphas[1] *= alphaDen;
				alphas[2] *= alphaDen;
				s = m_hullSum[0].Scale4(alphas[1]) + m_hullSum[1].Scale4(alphas[2]) + m_hullSum[2].Scale4(alphas[0]);
				d = m_hullDiff[0].Scale4(alphas[1]) + m_hullDiff[1].Scale4(alphas[2]) + m_hullDiff[2].Scale4(alphas[0]);
			} else {
				// this is a degenerated face that is so small that lose accuracy in 32 bit floats
				// get the closest point from the longest edge

				dgVector dir(((e10 % e10) > (e20 % e20)) ? e10 : e20);
				dgInt32 i0 = 0;
				dgInt32 i1 = 0;
				dgFloat32 dist0 = dir % m_hullDiff[0];
				dgFloat32 dist1 = -dist0;
				for (dgInt32 i = 1; i < 3; i++) {
					dgFloat32 test = dir % m_hullDiff[i];
					if (test > dist0) {
						i0 = i;
						dist0 = test;
					}
					test *= dgFloat32(-1.0f);
					if (test > dist1) {
						i1 = i;
						dist1 = test;
					}
				}

				if (i0 != i1) {
					const dgVector& p0 = m_hullDiff[i0];
					const dgVector& p1 = m_hullDiff[i1];
					dgVector dp(p1 - p0);
					dgAssert(dp % dp > dgFloat32(0.0f));
					dgFloat32 alpha0 = -(p0 % dp) / (dp % dp);
					dgAssert(alpha0 <= dgFloat32(1.01f));
					dgAssert(alpha0 >= dgFloat32(-0.01f));
					d = p0 + dp.Scale4(alpha0);
					s = m_hullSum[0] + (m_hullSum[i1] - m_hullSum[i0]).Scale4(alpha0);
				} else {
					s = m_hullSum[i0];
					d = m_hullDiff[i0];
				}
			}
		}
	}

	m_closestPoint0 = (s + d).Scale4(dgFloat32(0.5f));
	m_closestPoint1 = (s - d).Scale4(dgFloat32(0.5f));
	dgAssert(dgAbsf(m_normal % m_normal - dgFloat32(1.0f)) < dgFloat32(1.0e-4f));
	m_proxy->m_contactJoint->m_separtingVector = m_normal;
}


bool dgContactSolver::SanityCheck() const
{
	for (dgInt32 i = 0; i < m_faceIndex; i++) {
		const dgMinkFace* const face = &m_facePool[i];
		if (face->m_alive) {
			for (dgInt32 j = 0; j < 3; j++) {
				dgMinkFace* const twin = face->m_twin[j];
				if (!twin) {
					return false;
				}

				if (!twin->m_alive) {
					return false;
				}

				bool pass = false;
				for (dgInt32 k = 0; k < 3; k++) {
					if (twin->m_twin[k] == face) {
						pass = true;
						break;
					}
				}
				if (!pass) {
					return pass;
				}
			}
		}
	}
	return true;
}


bool dgContactSolver::CalculateClosestPoints()
{
	dgInt32 simplexPointCount = 0;
	dgFloat32 radius0 = m_instance0->GetBoxMaxRadius() * m_instance0->m_maxScale.m_x;
	dgFloat32 radius1 = m_instance1->GetBoxMaxRadius() * m_instance1->m_maxScale.m_x;
	if ((radius1 * dgFloat32(8.0f) > radius0) && (radius0 * dgFloat32(8.0f) > radius1)) {
		simplexPointCount = CalculateClosestSimplex();
		if (simplexPointCount < 0) {
			simplexPointCount = CalculateIntersectingPlane(-simplexPointCount);
		}
	} else {
		simplexPointCount = CalculateClosestSimplexLarge();
		if (simplexPointCount < 0) {
			simplexPointCount = CalculateIntersectingPlane(-simplexPointCount);
		}
	}
	if (simplexPointCount > 0) {
		dgAssert((simplexPointCount > 0) && (simplexPointCount <= 3));

		if (m_instance1->GetCollisionPrimityType() == m_polygonCollision) {
			dgCollisionConvexPolygon* const polygonShape = (dgCollisionConvexPolygon*)m_instance1->GetChildShape();
			polygonShape->SetFeatureHit(simplexPointCount, m_polygonFaceIndex1);
		} else if (m_instance0->GetCollisionPrimityType() == m_polygonCollision) {
			dgAssert (0);
			// this should never happens
			dgCollisionConvexPolygon* const polygonShape = (dgCollisionConvexPolygon*)m_instance0->GetChildShape();
			polygonShape->SetFeatureHit(simplexPointCount, m_polygonFaceIndex0);
		}

		CalculateContactFromFeacture(simplexPointCount);

		const dgMatrix& matrix0 = m_instance0->m_globalMatrix;
		const dgMatrix& matrix1 = m_instance1->m_globalMatrix;
		m_closestPoint0 = matrix0.TransformVector(m_instance0->SupportVertexSpecialProjectPoint(matrix0.UntransformVector(m_closestPoint0), matrix0.UnrotateVector(m_normal)));
		m_closestPoint1 = matrix1.TransformVector(m_instance1->SupportVertexSpecialProjectPoint(matrix1.UntransformVector(m_closestPoint1), matrix1.UnrotateVector(m_normal.Scale4(-1.0f))));

		m_vertexIndex = simplexPointCount;
	}
	return simplexPointCount >= 0;
}

dgInt32 dgContactSolver::ConvexPolygonToLineIntersection(const dgVector& normal, dgInt32 count1, dgVector* const shape1, dgInt32 count2, dgVector* const shape2, dgVector* const contactOut, dgVector* const mem) const
{
	dgInt32 count = 0;
	dgVector* output = mem;

	dgAssert(count1 >= 3);
	dgAssert(count2 <= 2);

	dgVector* ptr = NULL;
	// face line intersection
	if (count2 == 2) {
		ptr = (dgVector*)&shape2[0];
		dgInt32 i0 = count1 - 1;
		for (dgInt32 i1 = 0; i1 < count1; i1++) {
			dgVector n(normal * (shape1[i1] - shape1[i0]));
			dgAssert((n % n) > dgFloat32(0.0f));
			dgPlane plane(n, -(n % shape1[i0]));

			dgFloat32 test0 = plane.Evalue(ptr[0]);
			dgFloat32 test1 = plane.Evalue(ptr[1]);
			if (test0 >= dgFloat32(0.0f)) {
				if (test1 >= dgFloat32(0.0f)) {
					output[count + 0] = ptr[0];
					output[count + 1] = ptr[1];
					count += 2;
				} else {
					dgVector dp(ptr[1] - ptr[0]);
					dgFloat32 den = plane % dp;
					if (dgAbsf(den) < 1.0e-10f) {
						den = 1.0e-10f;
					}
					output[count + 0] = ptr[0];
					output[count + 1] = ptr[0] - dp.Scale3(test0 / den);
					count += 2;
				}
			} else if (test1 >= dgFloat32(0.0f)) {
				dgVector dp(ptr[1] - ptr[0]);
				dgFloat32 den = plane % dp;
				if (dgAbsf(den) < 1.0e-10f) {
					den = 1.0e-10f;
				}
				output[count] = ptr[0] - dp.Scale3(test0 / den);
				count++;
				output[count] = ptr[1];
				count++;
			} else {
				return 0;
			}


			count2 = count;
			ptr = output;
			output = &output[count];
			count = 0;
			i0 = i1;
			//dgAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
		}
	} else if (count2 == 1) {
		const dgVector& p = shape2[0];
		dgInt32 i0 = count1 - 1;
		for (dgInt32 i1 = 0; i1 < count1; i1++) {
			dgVector n(normal * (shape1[i1] - shape1[i0]));
			dgAssert((n % n) > dgFloat32(0.0f));
			dgPlane plane(n, -(n % shape1[i0]));
			dgFloat32 test0 = plane.Evalue(p);
			if (test0 < dgFloat32(-1.e-3f)) {
				return 0;
			}
			i0 = i1;
		}
		ptr = output;
		output[count] = p;
		count++;

	} else {
		count2 = 0;
	}

	for (dgInt32 i0 = 0; i0 < count2; i0++) {
		contactOut[i0] = ptr[i0];
	}
	return count2;
}


DG_INLINE dgContactSolver::dgPerimenterEdge* dgContactSolver::ReduceContacts(dgPerimenterEdge* poly, dgInt32 maxCount) const
{
	dgInt32 buffer[DG_MAX_EDGE_COUNT];
	dgUpHeap<dgPerimenterEdge*, dgFloat32> heap(buffer, sizeof (buffer));

#if 0
	dgPerimenterEdge* ptr = poly;
	do {
		dgVector error(*ptr->m_next->m_vertex - *ptr->m_vertex);
		dgAssert(error.m_w == 0.0f);
		dgFloat32 dist2 = error.DotProduct4(error).GetScalar();
		ptr->m_alived = 1;
		heap.Push(ptr, dist2);
		ptr = ptr->m_next;
	} while (ptr != poly);

	while (heap.GetCount() > maxCount) {
		dgPerimenterEdge* edge = heap[0];
		heap.Pop();
		if (edge->m_alived) {
			edge->m_next->m_alived = 0;
			edge->m_next = edge->m_next->m_next;

			dgVector error(*edge->m_next->m_vertex - *edge->m_vertex);
			dgAssert(error.m_w == 0.0f);
			dgFloat32 dist2 = error.DotProduct4(error).GetScalar();
			heap.Push(edge, dist2);
		}
	}

	while ((heap.GetCount() > 1) && (heap.Value(0) < DG_MINK_VERTEX_ERR2)) {
		dgPerimenterEdge* edge = heap[0];
		heap.Pop();
		if (edge->m_alived) {
			edge->m_next->m_alived = 0;
			edge->m_next = edge->m_next->m_next;

			dgVector error(*edge->m_next->m_vertex - *edge->m_vertex);
			dgAssert(error.m_w == 0.0f);
			dgFloat32 dist2 = error.DotProduct4(error).GetScalar();
			heap.Push(edge, dist2);
		}
	}

	poly = heap[0];
	heap.Pop();
	while (!poly->m_alived) {
		dgAssert(heap.GetCount());
		poly = heap[0];
		heap.Pop();
	}
#else 

	dgInt32 restart = 1;
	while (restart) {
		restart = 0;
		dgPerimenterEdge* ptr0 = poly;
		poly = poly->m_next;
		if (poly->m_next != poly) {
			heap.Flush();
			dgPerimenterEdge* ptr = poly;
			do {
				dgVector error(*ptr->m_next->m_vertex - *ptr->m_vertex);
				dgAssert(error.m_w == 0.0f);
				dgFloat32 dist2 = error.DotProduct4(error).GetScalar();
				if (dist2 < DG_MINK_VERTEX_ERR2) {
					ptr0->m_next = ptr->m_next;
					if (ptr == poly) {
						poly = ptr0;
						restart = 1;
						break;
					}
					ptr = ptr0;
				} else {
					heap.Push(ptr, dist2);
					ptr0 = ptr;
				}

				ptr = ptr->m_next;
			} while (ptr != poly);
		}
	}

	if (heap.GetCount()) {
		if (maxCount > 8) {
			maxCount = 8;
		}
		while (heap.GetCount() > maxCount) {
			dgPerimenterEdge* ptr = heap[0];
			heap.Pop();
			for (dgInt32 i = 0; i < heap.GetCount(); i++) {
				if (heap[i] == ptr->m_next) {
					heap.Remove(i);
					break;
				}
			}

			ptr->m_next = ptr->m_next->m_next;
			dgVector error(*ptr->m_next->m_vertex - *ptr->m_vertex);
			dgFloat32 dist2 = error % error;
			heap.Push(ptr, dist2);
		}
		poly = heap[0];
	}
#endif

	return poly;
}


dgInt32 dgContactSolver::ConvexPolygonsIntersection(const dgVector& normal, dgInt32 count0, dgVector* const shape0, dgInt32 count1, dgVector* const shape1, dgVector* const contactOut, dgInt32 maxContacts) const
{
	dgInt32 count = 0;
	if (count1 <= 2) {
		count = ConvexPolygonToLineIntersection(normal.Scale4 (dgFloat32 (-1.0f)), count0, shape0, count1, shape1, contactOut, &contactOut[count0 + count1 + maxContacts]);
	} else if (count0 <= 2) {
		count = ConvexPolygonToLineIntersection(normal, count1, shape1, count0, shape0, contactOut, &contactOut[count0 + count1 + maxContacts]);
	} else {
		dgAssert(count0 >= 3);
		dgAssert(count1 >= 3);

		dgPerimenterEdge subdivision[128];
		dgAssert((2 * (count0 + count1)) < dgInt32(sizeof (subdivision) / sizeof (subdivision[0])));

		for (dgInt32 i0 = 1; i0 < count1; i0++) {
			subdivision[i0].m_vertex = &shape1[i0];
			subdivision[i0].m_prev = &subdivision[i0 - 1];
			subdivision[i0].m_next = &subdivision[i0 + 1];
		}
		subdivision[0].m_vertex = &shape1[0];
		subdivision[0].m_prev = &subdivision[count1 - 1];
		subdivision[0].m_next = &subdivision[1];

		subdivision[count1 - 1].m_next = &subdivision[0];

		dgPerimenterEdge* edgeClipped[2];
		dgVector* output = &contactOut[count0 + count1 + maxContacts];

		edgeClipped[0] = NULL;
		edgeClipped[1] = NULL;
		dgInt32 j0 = 0;
		dgInt32 edgeIndex = count1;
		dgPerimenterEdge* poly = &subdivision[0];
		for (dgInt32 j1 = count0 - 1; j1 >= 0; j1--) {
			dgVector n(normal * (shape0[j1] - shape0[j0]));
			dgPlane plane(n, -(n % shape0[j0]));
			j0 = j1;
			count = 0;
			dgPerimenterEdge* tmp = poly;
			dgInt32 isInside = 0;
			dgFloat32 test0 = plane.Evalue(*tmp->m_vertex);
			do {
				dgFloat32 test1 = plane.Evalue(*tmp->m_next->m_vertex);

				if (test0 >= dgFloat32(0.0f)) {
					isInside |= 1;
					if (test1 < dgFloat32(0.0f)) {
						const dgVector& p0 = *tmp->m_vertex;
						const dgVector& p1 = *tmp->m_next->m_vertex;
						dgVector dp(p1 - p0);
						dgFloat32 den = plane % dp;
						if (dgAbsf(den) < dgFloat32(1.0e-24f)) {
							den = (den >= dgFloat32(0.0f)) ? dgFloat32(1.0e-24f) : dgFloat32(-1.0e-24f);
						}

						den = test0 / den;
						if (den >= dgFloat32(0.0f)) {
							den = dgFloat32(0.0f);
						}
						else if (den <= -1.0f) {
							den = dgFloat32(-1.0f);
						}
						output[0] = p0 - dp.Scale3(den);
						edgeClipped[0] = tmp;
						count++;
					}
				} else if (test1 >= dgFloat32(0.0f)) {
					const dgVector& p0 = *tmp->m_vertex;
					const dgVector& p1 = *tmp->m_next->m_vertex;
					isInside |= 1;
					dgVector dp(p1 - p0);
					dgFloat32 den = plane % dp;
					if (dgAbsf(den) < dgFloat32(1.0e-24f)) {
						den = (den >= dgFloat32(0.0f)) ? dgFloat32(1.0e-24f) : dgFloat32(-1.0e-24f);
					}
					den = test0 / den;
					if (den >= dgFloat32(0.0f)) {
						den = dgFloat32(0.0f);
					} else if (den <= -1.0f) {
						den = dgFloat32(-1.0f);
					}
					output[1] = p0 - dp.Scale3(den);
					edgeClipped[1] = tmp;
					count++;
				}

				test0 = test1;
				tmp = tmp->m_next;
			} while (tmp != poly && (count < 2));

			if (!isInside) {
				return 0;
			}

			if (count == 2) {
				dgPerimenterEdge* const newEdge = &subdivision[edgeIndex];
				newEdge->m_next = edgeClipped[1];
				newEdge->m_prev = edgeClipped[0];
				edgeClipped[0]->m_next = newEdge;
				edgeClipped[1]->m_prev = newEdge;

				newEdge->m_vertex = &output[0];
				edgeClipped[1]->m_vertex = &output[1];
				poly = newEdge;

				output += 2;
				edgeIndex++;
				//dgAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
				dgAssert(edgeIndex < dgInt32(sizeof (subdivision) / sizeof (subdivision[0])));
			}
		}

		dgAssert(poly);
		poly = ReduceContacts(poly, maxContacts);
		count = 0;
		dgPerimenterEdge* intersection = poly;
		do {
			contactOut[count] = *intersection->m_vertex;
			count++;
			intersection = intersection->m_next;
		} while (intersection != poly);
	}
	return count;
}


dgInt32 dgContactSolver::CalculateContacts (const dgVector& point, const dgVector& normal)
{
	dgInt32 count = 0;

	const dgInt32 baseCount = 16;

	dgVector* const contactsOut = &m_hullDiff[0];
	dgAssert(m_instance1->IsType(dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert(m_instance0->IsType(dgCollision::dgCollisionConvexShape_RTTI));

	dgInt32 count1 = 0;
	dgVector* const shape1 = &contactsOut[baseCount];

	const dgMatrix& matrix1 = m_instance1->m_globalMatrix;
	dgVector ponintOnInstance1(matrix1.UntransformVector(point));
	dgVector normalOnInstance1 (matrix1.UnrotateVector(normal));
	count1 = m_instance1->CalculatePlaneIntersection(normalOnInstance1, ponintOnInstance1, shape1, dgFloat32(1.0f));
	if (!count1) {
		count1 = 1;
		shape1[0] = ponintOnInstance1;
	}
/*
	dgVector supportOnInstance1 (m_instance1->SupportVertex(normalOnInstance1, &dommy));
	dgFloat32 dist = normalOnInstance1 % (supportOnInstance1 - ponintOnInstance1);
	if (dist >= DG_ROBUST_PLANE_CLIP) {
		supportOnInstance1 -= normalOnInstance1.Scale4(DG_ROBUST_PLANE_CLIP);
		count1 = m_instance1->CalculatePlaneIntersection(normalOnInstance1, supportOnInstance1, shape1, dgFloat32(1.0f));
		dgVector err(normalOnInstance1.Scale4(normalOnInstance1 % (supportOnInstance1 - ponintOnInstance1)));
		for (dgInt32 i = 0; i < count1; i++) {
			shape1[i] -= err;
		}
	} else {
		count1 = m_instance1->CalculatePlaneIntersection(normalOnInstance1, ponintOnInstance1, shape1, dgFloat32(1.0f));
		if (!count1) {
			ponintOnInstance1 -= normalOnInstance1.Scale4(DG_ROBUST_PLANE_CLIP);
			count1 = m_instance1->CalculatePlaneIntersection(normalOnInstance1, ponintOnInstance1, shape1, dgFloat32(1.0f));
		}
	}
*/
	if (count1) {
		for (int i = 0; i < count1; i ++) {
			shape1[i] = matrix1.TransformVector(shape1[i]);
		}

		dgInt32 count0 = 0;
		dgVector* const shape0 = &contactsOut[baseCount + count1];

		const dgMatrix& matrix0 = m_instance0->m_globalMatrix;
		dgVector pointOnInstance0(matrix0.UntransformVector(point));
		dgVector normalOnInstance0(matrix0.UnrotateVector(normal.Scale4(dgFloat32(-1.0f))));
/*
		dgVector supportOnInstance0(m_instance0->SupportVertex(normalOnInstance0, &dommy));
		dgFloat32 dist = normalOnInstance0 % (supportOnInstance0 - pointOnInstance0);
		if (dist >= DG_ROBUST_PLANE_CLIP) {
			supportOnInstance0 -= normalOnInstance0.Scale4(DG_ROBUST_PLANE_CLIP);
			count0 = m_instance0->CalculatePlaneIntersection(normalOnInstance0, supportOnInstance0, shape0, dgFloat32(1.0f));
			dgVector err(normalOnInstance0.Scale4(normalOnInstance0 % (supportOnInstance0 - pointOnInstance0)));
			for (dgInt32 i = 0; i < count0; i++) {
				shape0[i] -= err;
			}
		} else {
			count0 = m_instance0->CalculatePlaneIntersection(normalOnInstance0, pointOnInstance0, shape0, dgFloat32(1.0f));
			if (!count0) {
				pointOnInstance0 -= normalOnInstance0.Scale4(DG_ROBUST_PLANE_CLIP);
				count0 = m_instance0->CalculatePlaneIntersection(normalOnInstance0, pointOnInstance0, shape0, dgFloat32(1.0f));
			}
		}
*/
		count0 = m_instance0->CalculatePlaneIntersection(normalOnInstance0, pointOnInstance0, shape0, dgFloat32(1.0f));
		if (!count0) {
			count0 = 1;
			shape0[0] = pointOnInstance0;
		}

		if (count0) {
			for (dgInt32 i = 0; i < count0; i++) {
				shape0[i] = matrix0.TransformVector(shape0[i]);
			}

			if (count1 == 1) {
				count = 1;
				contactsOut[0] = shape1[0];
			} else if (count0 == 1) {
				count = 1;
				contactsOut[0] = shape0[0];
			} else if ((count1 == 2) && (count0 == 2)) {
				dgVector p0(shape1[0]);
				dgVector p1(shape1[1]);
				const dgVector& q0 = shape0[0];
				const dgVector& q1 = shape0[1];
				dgVector p10(p1 - p0);
				dgVector q10(q1 - q0);
				p10 = p10.Scale4(dgRsqrt(p10 % p10 + dgFloat32(1.0e-8f)));
				q10 = q10.Scale4(dgRsqrt(q10 % q10 + dgFloat32(1.0e-8f)));
				dgFloat32 dot = q10 % p10;
				if (dgAbsf(dot) > dgFloat32(0.998f)) {
					dgFloat32 pl0 = p0 % p10;
					dgFloat32 pl1 = p1 % p10;
					dgFloat32 ql0 = q0 % p10;
					dgFloat32 ql1 = q1 % p10;
					if (pl0 > pl1) {
						dgSwap(pl0, pl1);
						dgSwap(p0, p1);
						p10 = p10.Scale4(dgFloat32(-1.0f));
					}
					if (ql0 > ql1) {
						dgSwap(ql0, ql1);
					}
					if (!((ql0 > pl1) && (ql1 < pl0))) {
						dgFloat32 clip0 = (ql0 > pl0) ? ql0 : pl0;
						dgFloat32 clip1 = (ql1 < pl1) ? ql1 : pl1;

						count = 2;
						contactsOut[0] = p0 + p10.Scale4(clip0 - pl0);
						contactsOut[1] = p0 + p10.Scale4(clip1 - pl0);
					}
				}
				else {
					count = 1;
					dgVector c0;
					dgVector c1;
					dgRayToRayDistance(p0, p1, q0, q1, c0, c1);
					contactsOut[0] = (c0 + c1).Scale4(dgFloat32(0.5f));
				}
			} else {
				dgAssert((count1 >= 2) && (count0 >= 2));
				count = ConvexPolygonsIntersection(normal, count0, shape0, count1, shape1, contactsOut, baseCount);
			}
		}
	}

	if (!count && m_proxy->m_continueCollision) {
		count = 1;
		contactsOut[0] = point;
	}

	return count;
}

dgFloat32 dgContactSolver::RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut)
{
	dgVector normal(dgFloat32 (0.0f));
	dgVector point (localP0);
	dgVector point0 (localP0);
	dgVector p0p1 (localP0 - localP1);

	// avoid NaN as a result of a division by zero
	if ((p0p1.TestZero().GetSignMask() & 7) == 7) {
		return dgFloat32(1.2f);
	}

	dgFloat32 param = dgFloat32 (0.0f);

	dgInt32 index = 0;
	memset (m_hullSum, 0, 4 * sizeof (m_hullSum[0]));
	dgVector dir (p0p1.CompProduct4 (p0p1.DotProduct4(p0p1).InvSqrt ()));

	const dgCollisionConvex* const collision = (dgCollisionConvex*)m_instance0->GetChildShape();
	m_hullDiff[0] = collision->SupportVertex (dir, NULL) - point;
	dgVector v (m_hullDiff[0]);
	index = 1;
	do {
		dgInt32 iter = 0;
		dgInt32 cycling = 0;
		dgFloat32 minDist = dgFloat32 (1.0e20f);

		do {
			dgAssert (v.m_w == dgFloat32 (0.0f));
			dgVector dist (v.DotProduct4(v));
			dgFloat32 distance = dist.m_x;

			if (distance < dgFloat32 (1.0e-9f)) {
				index = -1; 
				break;
			}

			if (distance < minDist) {
				minDist = distance;
				cycling = -1;
			}
			cycling ++;
			if (cycling > 4) {
				dgAssert (0);
				index = -1; 
				break;
			}

			//dgVector dir (v.Scale4 (-dgRsqrt(dist)));
			dgVector dir (v.CompProduct4(dist.InvSqrt().CompProduct4(dgVector::m_negOne)));
			dgAssert (dir.m_w == dgFloat32 (0.0f));
			m_hullDiff[index] = collision->SupportVertex (dir, NULL) - point;
			const dgVector& w = m_hullDiff[index];
			dgVector wv (w - v);
			dgAssert (wv.m_w == dgFloat32 (0.0f));
			distance = dir.DotProduct4(wv).m_x;
			if (distance < dgFloat32 (1.0e-3f)) {
				normal = dir;
				break;
			}

			index ++;
			switch (index) 
			{
				case 2:
				{
					v = ReduceLine (index);
					break;
				}

				case 3:
				{
					v = ReduceTriangle (index);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum (index);
					break;
				}
			}

			iter ++;
			cycling ++;
		} while (iter < DG_CONNICS_CONTATS_ITERATIONS); 

		dgAssert (index);
		if (index > 0) {
			dgVector q (v + point);
			dgFloat32 den = normal.DotProduct4(p0p1).m_x;
			dgAssert (den != 0.0f);
			dgFloat32 t1 = (normal % (localP0 - q)) / den;

			if (t1 < param) {
				index = -1;
				t1 = dgFloat32 (0.0f);
			} else if (t1 > maxT) {
				index = -1;
				t1 = dgFloat32 (1.0f);
			}
			param = t1;
	
			point = localP0 - p0p1.Scale4 (param);
			dgVector step (point0 - point);
			point0 = point;
			for(dgInt32 i = 0; i < index; i ++) {
				m_hullDiff[i] += step;
			}

			switch (index) 
			{
				case 1:
				{
					v = m_hullDiff[0];
					break;
				}

				case 2:
				{
					v = ReduceLine (index);
					break;
				}

				case 3:
				{
					v = ReduceTriangle (index);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum (index);
					break;
				}
			}
		}
	} while (index >= 0);

	if ((param > dgFloat32 (0.0f)) && (param < maxT)) {
		contactOut.m_normal = normal;
	} else {
		param = dgFloat32 (1.2f);
	}

	return param;
}


DG_INLINE void dgContactSolver::TranslateSimplex(const dgVector& step)
{
	m_instance1->m_globalMatrix.m_posit -= step;
	for (dgInt32 i = 0; i < m_vertexIndex; i++) {
		m_hullSum[i] -= step;
		m_hullDiff[i] += step;
	}
}



dgInt32 dgContactSolver::CalculateConvexCastContacts()
{
	dgInt32 iter = 0;
	dgInt32 count = 0;
	
	dgFloat32 tacc = dgFloat32(0.0f);
	dgFloat32 timestep = m_proxy->m_timestep;
	m_proxy->m_contactJoint->m_closestDistance = dgFloat32(1.0e10f);

	dgVector savedPosition1 (m_instance1->m_globalMatrix.m_posit);

	dgVector relVeloc (m_proxy->m_body0->GetVelocity() - m_proxy->m_body1->GetVelocity());
	do {
		bool state = CalculateClosestPoints();
		if (!state) {
			break;
		}
		dgAssert(m_normal.m_w == dgFloat32(0.0f));
		dgFloat32 den = m_normal.DotProduct4(relVeloc).GetScalar();
		if (den <= dgFloat32(1.0e-6f)) {
			// bodies are residing from each other, even if they are touching they are not considered to be colliding because the motion will move them apart 
			// get the closet point and the normal at contact point
			m_proxy->m_timestep = dgFloat32 (1.2f);
			m_proxy->m_normal = m_normal.Scale4(-1.0f);
			m_proxy->m_closestPointBody0 = m_closestPoint0;
			m_proxy->m_closestPointBody1 = m_closestPoint1;
			break;
		}

		dgFloat32 num = m_normal.DotProduct4(m_closestPoint1 - m_closestPoint0).GetScalar();
		if ((num <= dgFloat32(1.0e-5f)) && (tacc <= timestep)) {
			// bodies collide at time tacc, but we do not set it yet
			dgVector step(relVeloc.Scale4(tacc));
			m_proxy->m_timestep = tacc;
			m_proxy->m_closestPointBody0 = m_closestPoint0;
			m_proxy->m_closestPointBody1 = m_closestPoint1;
			m_proxy->m_normal = m_normal.Scale4 (dgFloat32 (-1.0f));
			m_proxy->m_contactJoint->m_closestDistance = m_proxy->m_normal.DotProduct4(m_closestPoint0 - m_closestPoint1).GetScalar();
			dgFloat32 penetration = dgMax(num * dgFloat32(-1.0f) - DG_RESTING_CONTACT_PENETRATION, dgFloat32(0.0f));
			m_proxy->m_contactJoint->m_closestDistance = penetration;
			if (m_proxy->m_contacts && !m_proxy->m_intersectionTestOnly) {
				if (m_proxy->m_instance0->GetCollisionMode() & m_proxy->m_instance1->GetCollisionMode()) {

					m_normal = m_normal.Scale4 (dgFloat32 (-1.0f));
					m_proxy->m_contactJoint->m_contactActive = 1;
					dgVector contactPoint((m_closestPoint0 + m_closestPoint1).Scale4(dgFloat32(0.5f)));
					count = CalculateContacts(contactPoint, m_normal);
					if (count) {
			
						count = dgMin(m_proxy->m_maxContacts, count);
						dgContactPoint* const contactOut = m_proxy->m_contacts;

						for (int i = 0; i < count; i++) {
							contactOut[i].m_point = m_hullDiff[i];
							contactOut[i].m_normal = m_normal;
							contactOut[i].m_penetration = penetration;
						}
					}
				}
			}
			break;
		}

		dgAssert (den > dgFloat32 (0.0f));
		dgFloat32 dt = num / den;
		if ((tacc + dt) >= timestep) {
			// object do not collide on this timestep
			m_proxy->m_timestep = tacc + dt;
			m_proxy->m_normal = m_normal.Scale4 (dgFloat32 (-1.0f));
			m_proxy->m_closestPointBody0 = m_closestPoint0;
			m_proxy->m_closestPointBody1 = m_closestPoint1;
			break;
		}

		tacc += dt;
		dgVector step(relVeloc.Scale4(dt));
		TranslateSimplex(step);

		iter++;
	} while (iter < DG_SEPARATION_PLANES_ITERATIONS);

	m_instance1->m_globalMatrix.m_posit = savedPosition1;
	return count;
}


dgInt32 dgContactSolver::CalculateConvexToConvexContacts ()
{
	dgInt32 count = 0;

	if (m_proxy->m_intersectionTestOnly) {
		CalculateClosestPoints();
		dgFloat32 penetration = m_normal.DotProduct4(m_closestPoint1 - m_closestPoint0).GetScalar() - m_proxy->m_skinThickness;
		dgInt32 retVal = (penetration <= dgFloat32(0.0f)) ? -1 : 0;
		m_proxy->m_contactJoint->m_contactActive = retVal;
		return retVal;
	} else {
		if (CalculateClosestPoints()) { 
			dgFloat32 penetration = m_normal.DotProduct4(m_closestPoint1 - m_closestPoint0).GetScalar() - m_proxy->m_skinThickness;

			if (penetration <= dgFloat32(0.0f)) {
				m_proxy->m_contactJoint->m_contactActive = 1;
				if (m_instance0->GetCollisionMode() & m_instance1->GetCollisionMode()) {
					dgVector contactPoint((m_closestPoint0 + m_closestPoint1).Scale4(dgFloat32(0.5f)));
					count = CalculateContacts(contactPoint, m_normal.Scale4(-1.0f));
				}
			}

			m_proxy->m_closestPointBody0 = m_closestPoint0;
			m_proxy->m_closestPointBody1 = m_closestPoint1;
			m_proxy->m_contactJoint->m_closestDistance = penetration;

			m_normal = m_normal.Scale4 (dgFloat32 (-1.0f));
			penetration = -penetration;
			m_proxy->m_normal = m_normal;
			count = dgMin(m_proxy->m_maxContacts, count);
			dgContactPoint* const contactOut = m_proxy->m_contacts;
			for (int i = 0; i < count; i ++) {
				contactOut[i].m_point = m_hullDiff[i];
				contactOut[i].m_normal = m_normal;
				contactOut[i].m_penetration = penetration;
			}
		}
	}

	return count;
}
