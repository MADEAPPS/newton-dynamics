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

#include "dgWorld.h"
#include "dgCollisionBox.h"
#include "dgCollisionMesh.h"
#include "dgMinkowskiConv.h"
#include "dgCollisionConvex.h"
#include "dgCollisionSphere.h"
#include "dgCollisionCapsule.h"
#include "dgWorldDynamicUpdate.h"

#ifdef DG_BUILD_CORE_200_COLLISION

dgVector dgContactSolver::m_dir[14];

dgInt32 dgContactSolver::m_faceIndex[][4] =
{ 
	{0, 1, 2, 3},
	{1, 0, 3, 2},
	{0, 2, 3, 1},
	{2, 1, 3, 0},
};


dgContactSolver::dgContactSolver(dgCollisionParamProxy& proxy)
	:m_matrix (proxy.m_matrix)
{
	void* const hullVertexLarge = m_hullVertex;
	void* const averVertexLarge = m_averVertex;
	m_hullVertexLarge = (dgBigVector*)hullVertexLarge;
	m_averVertexLarge = (dgBigVector*)averVertexLarge;

	dgAssert ((m_matrix.m_front % m_matrix.m_front) > dgFloat32 (0.9995f));
	dgAssert ((m_matrix.m_up % m_matrix.m_up) > dgFloat32 (0.9995f));
	dgAssert ((m_matrix.m_right % m_matrix.m_right) > dgFloat32 (0.9995f));
	dgAssert (((m_matrix.m_front * m_matrix.m_up) % m_matrix.m_right) < dgFloat32 (1.0001f));

	m_lastFaceCode = dgMinkError;

	m_proxy = &proxy;
	m_floatingBody = proxy.m_floatingBody; 
	m_referenceBody = proxy.m_referenceBody; 
	m_penetrationPadding = proxy.m_skinThickness;
	m_floatingcollision = proxy.m_floatingCollision;
	m_referenceCollision = proxy.m_referenceCollision;
}


dgContactSolver::dgContactSolver(dgCollisionParamProxy& proxy, dgCollisionInstance* const polygon)
	:m_matrix (proxy.m_matrix) 
{
	void* const hullVertexLarge = m_hullVertex;
	void* const averVertexLarge = m_averVertex;
	m_hullVertexLarge = (dgBigVector*)hullVertexLarge;
	m_averVertexLarge = (dgBigVector*)averVertexLarge;

	dgAssert ((m_matrix.m_front % m_matrix.m_front) > dgFloat32 (0.9999f));
	dgAssert ((m_matrix.m_up % m_matrix.m_up) > dgFloat32 (0.9999f));
	dgAssert ((m_matrix.m_right % m_matrix.m_right) > dgFloat32 (0.9999f));
	dgAssert (((m_matrix.m_front * m_matrix.m_up) % m_matrix.m_right) < dgFloat32 (1.0001f));


	m_lastFaceCode = dgMinkError;

	m_proxy = &proxy;
	m_floatingBody = proxy.m_floatingBody; 
	//m_floatingcollision = (dgCollisionConvex*) proxy.m_floatingCollision;
	m_floatingcollision = polygon;
	m_referenceBody = proxy.m_referenceBody; 
	m_referenceCollision = proxy.m_referenceCollision;
	m_penetrationPadding = proxy.m_skinThickness;
}

void dgContactSolver::CalcSupportVertexLarge (const dgVector& dir, dgInt32 entry)
{
	dgAssert ((dir % dir) > dgFloat32 (0.999f));
	dgVector p0 (m_referenceCollision->SupportVertex (dir));
	dgVector dir1 (m_matrix.UnrotateVector (dir.Scale (dgFloat32 (-1.0f))));
	dgVector q0 (m_matrix.TransformVector (m_floatingcollision->SupportVertex (dir1)));

	dgBigVector p(p0);
	dgBigVector q(q0);

	m_hullVertexLarge[entry] = p - q;
	m_averVertexLarge[entry] = p + q;
}


void dgContactSolver::CalcSupportVertex (const dgVector& dir, dgInt32 entry)
{
	dgAssert ((dir % dir) > dgFloat32 (0.999f));
	dgVector p (m_referenceCollision->SupportVertex (dir));
	dgVector dir1 (m_matrix.UnrotateVector (dir.Scale (dgFloat32 (-1.0f))));
	dgVector q (m_matrix.TransformVector (m_floatingcollision->SupportVertex (dir1)));

	m_hullVertex[entry] = p - q;
	m_averVertex[entry] = p + q;
}


void dgContactSolver::CalculateVelocities (dgFloat32 timestep) 
{
	dgAssert (0);
	/*

	dgVector refOmega;
	dgVector floatOmega;

	m_referenceBody->GetPredictingVelocity(m_referenceBodyVeloc, refOmega);
	m_floatingBody->GetPredictingVelocity (m_floatingBodyVeloc, floatOmega);

	dgVector vRel (m_floatingBodyVeloc - m_referenceBodyVeloc);
	m_localRelVeloc = m_proxy->m_referenceCollision->m_globalMatrix.UnrotateVector(vRel);
*/
}

dgBigVector dgContactSolver::ReduceLineLarge (const dgBigVector& origin)
{
	const dgBigVector& p0 = m_hullVertex[0];
	const dgBigVector& p1 = m_hullVertex[1];
	dgBigVector dp (p1 - p0);
	dgBigVector v;
	dgFloat64 mag2 = dp % dp;
	if (dp % dp < dgFloat64 (1.0e-24f)) {
		v = p0;
		m_vertexIndex = 1;
	} else {
		dgFloat64 alpha0 = ((origin - p0) % dp) / mag2;
		if (alpha0 > dgFloat64 (1.0f)) {
			v = p1;
			m_vertexIndex = 1;
			m_hullVertexLarge[0] = m_hullVertexLarge[1];
			m_averVertexLarge[0] = m_averVertexLarge[1];

		} else if (alpha0 < dgFloat64 (0.0f)) {
			v = p0;
			m_vertexIndex = 1;
		} else {
			v = p0 + dp.Scale3 (alpha0);
		}
	}
	return v;
}


dgVector dgContactSolver::ReduceLine (const dgVector& origin)
{
	const dgVector& p0 = m_hullVertex[0];
	const dgVector& p1 = m_hullVertex[1];
	dgVector dp (p1 - p0);
	dgVector v;

	dgFloat32 mag2 = dp % dp;
	if (mag2 < dgFloat32 (1.0e-24f)) {
		v = p0;
		m_vertexIndex = 1;
	} else {
		dgFloat32 alpha0 = ((origin - p0) % dp) / mag2;
		if (alpha0 > dgFloat32 (1.0f)) {
			v = p1;
			m_vertexIndex = 1;
			m_hullVertex[0] = m_hullVertex[1];
			m_averVertex[0] = m_averVertex[1];

		} else if (alpha0 < dgFloat32 (0.0f)) {
			v = p0;
			m_vertexIndex = 1;
		} else {
			v = p0 + dp.Scale (alpha0);
		}
	}
	return v;
}

dgBigVector dgContactSolver::ReduceTriangleLarge (const dgBigVector& origin)
{
	dgBigVector normal ((m_hullVertexLarge[1] - m_hullVertexLarge[0]) * (m_hullVertexLarge[2] - m_hullVertexLarge[0]));
	dgAssert ((normal % normal) > dgFloat64 (0.0f));

	dgInt32 i0 = 2;
	dgInt32 minIndex = -1;
	dgBigVector p1p0 (m_hullVertexLarge[2] - origin);
	for (dgInt32 i1 = 0; i1 < 3; i1 ++) {
		dgBigVector p2p0 (m_hullVertexLarge[i1] - origin);

		dgFloat64 volume = (p1p0 * p2p0) % normal;
		if (volume < dgFloat64 (0.0f)) {
			dgBigVector segment (m_hullVertexLarge[i1] - m_hullVertexLarge[i0]);
			dgBigVector poinP0 (origin - m_hullVertexLarge[i0]);
			dgFloat64 den = segment % segment;
			dgAssert (den > dgFloat64 (0.0f));
			dgFloat64 num = poinP0 % segment;
			if (num < dgFloat64 (0.0f)) {
//				m_vertexIndex = 1;
//				m_hullVertexLarge[0] = m_hullVertexLarge[i0];
//				m_averVertexLarge[0] = m_averVertexLarge[i0];
//				return m_hullVertexLarge[0];
				minIndex = i0;

			} else if (num > den) {
//				m_vertexIndex = 1;
//				m_hullVertexLarge[0] = m_hullVertexLarge[i1];
//				m_averVertexLarge[0] = m_averVertexLarge[i1];
//				return m_hullVertexLarge[0];
				minIndex = i1;
			}
			m_vertexIndex = 2;
			dgBigVector tmp0 (m_hullVertexLarge[i0]);
			dgBigVector tmp1 (m_hullVertexLarge[i1]);
			m_hullVertexLarge[0] = tmp0;
			m_hullVertexLarge[1] = tmp1;
			tmp0 = m_averVertexLarge[i0];
			tmp1 = m_averVertexLarge[i1];
			m_averVertexLarge[0] = tmp0;
			m_averVertexLarge[1] = tmp1;
			return m_hullVertexLarge[0] + segment.Scale3 (num / den);
		}
		p1p0 = p2p0;
		i0 = i1;
	}


	if (minIndex != -1) {
		m_vertexIndex = 1;
		m_hullVertexLarge[0] = m_hullVertexLarge[minIndex];
		m_averVertexLarge[0] = m_averVertexLarge[minIndex];
		return m_hullVertexLarge[0];
	} else {
		m_vertexIndex = 3;
		return origin - normal.Scale3((normal % (origin - m_hullVertexLarge[0])) / (normal % normal));
	}

}


dgVector dgContactSolver::ReduceTriangle (const dgVector& origin)
{

	dgVector normal ((m_hullVertex[1] - m_hullVertex[0]) * (m_hullVertex[2] - m_hullVertex[0]));
	dgAssert ((normal % normal) > dgFloat32 (0.0f));

	dgInt32 i0 = 2;
	dgInt32 minIndex = -1;
	dgVector p1p0 (m_hullVertex[2] - origin);
	for (dgInt32 i1 = 0; i1 < 3; i1 ++) {
		dgVector p2p0 (m_hullVertex[i1] - origin);

		dgFloat32 volume = (p1p0 * p2p0) % normal;
		if (volume < dgFloat32 (0.0f)) {
			dgVector segment (m_hullVertex[i1] - m_hullVertex[i0]);
			dgVector poinP0 (origin - m_hullVertex[i0]);
			dgFloat32 den = segment % segment;
			dgAssert (den > dgFloat32 (0.0f));
			dgFloat32 num = poinP0 % segment;

			if (num < dgFloat32 (0.0f)) {
//				m_vertexIndex = 1;
//				m_hullVertex[0] = m_hullVertex[i0];
//				m_averVertex[0] = m_averVertex[i0];
//				return m_hullVertex[0];
				minIndex = i0;
			} else if (num > den) {
//				m_vertexIndex = 1;
//				m_hullVertex[0] = m_hullVertex[i1];
//				m_averVertex[0] = m_averVertex[i1];
//				return m_hullVertex[0];
				minIndex = i1;
			} else {
				m_vertexIndex = 2;
				dgVector tmp0 (m_hullVertex[i0]);
				dgVector tmp1 (m_hullVertex[i1]);
				m_hullVertex[0] = tmp0;
				m_hullVertex[1] = tmp1;
				tmp0 = m_averVertex[i0];
				tmp1 = m_averVertex[i1];
				m_averVertex[0] = tmp0;
				m_averVertex[1] = tmp1;
				return m_hullVertex[0] + segment.Scale (num / den);
			}
		}
		p1p0 = p2p0;
		i0 = i1;
	}

	if (minIndex != -1) {
		m_vertexIndex = 1;
		m_hullVertex[0] = m_hullVertex[minIndex];
		m_averVertex[0] = m_averVertex[minIndex];
		return m_hullVertex[0];
	} else {
		m_vertexIndex = 3;
		return origin - normal.Scale((normal % (origin - m_hullVertex[0])) / (normal % normal));
	}
}


dgBigVector dgContactSolver::ReduceTetrahedrumLarge (const dgBigVector& origin)
{
	dgAssert (0);
	dgInt32 index0 = -1;
	dgInt32 index1 = -1;
	dgInt32 index2 = -1;
	dgBigVector p (origin);
	dgFloat64 minDist = dgFloat32 (1.0e20f);
	for (dgInt32 i = 0; i < 4; i ++) {
		dgInt32 i0 = m_faceIndex[i][0];
		dgInt32 i1 = m_faceIndex[i][1];
		dgInt32 i2 = m_faceIndex[i][2];
		const dgBigVector& p0 = m_hullVertexLarge[i0];
		const dgBigVector& p1 = m_hullVertexLarge[i1]; 
		const dgBigVector& p2 = m_hullVertexLarge[i2];

		dgBigVector p10 (p1 - p0);
		dgBigVector p20 (p2 - p0);
		dgBigVector p_p0 (origin - p0);
		dgBigVector normal (p10 * p20);
		dgFloat64 volume = p_p0 % normal;
		if (volume < dgFloat64 (0.0f)) {
			dgBigVector q (dgPointToTriangleDistance (origin, p0, p1, p2, normal));
			dgBigVector qDist (q - origin);

			dgFloat64 dist = qDist % qDist;
			if (dist < minDist) {
				p = q;
				index0 = i0;
				index1 = i1;
				index2 = i2;
				minDist = dist;
			}
		}
	}

	if (index0 != -1) {
		dgBigVector tmpSum[2];
		dgBigVector tmpDiff[2];
		tmpDiff[0] = m_hullVertexLarge[index1];
		tmpDiff[1] = m_hullVertexLarge[index2];
		tmpSum[0] = m_averVertexLarge[index1];
		tmpSum[1] = m_averVertexLarge[index2];

		m_hullVertexLarge[0] = m_hullVertexLarge[index0];
		m_hullVertexLarge[1] = tmpDiff[0];
		m_hullVertexLarge[2] = tmpDiff[1];
		m_averVertexLarge[0] = m_averVertexLarge[index0];
		m_averVertexLarge[1] = tmpSum[0];
		m_averVertexLarge[2] = tmpSum[1];
		return ReduceTriangleLarge (origin);
	}
	return p;
}


dgVector dgContactSolver::ReduceTetrahedrum (const dgVector& origin)
{
//	dgAssert (0);
	dgInt32 index0 = -1;
	dgInt32 index1 = -1;
	dgInt32 index2 = -1;
	dgVector p (origin);
	dgFloat32 minDist = dgFloat32 (1.0e20f);
	for (dgInt32 i = 0; i < 4; i ++) {
		dgInt32 i0 = m_faceIndex[i][0];
		dgInt32 i1 = m_faceIndex[i][1];
		dgInt32 i2 = m_faceIndex[i][2];
		const dgVector& p0 = m_hullVertex[i0];
		const dgVector& p1 = m_hullVertex[i1]; 
		const dgVector& p2 = m_hullVertex[i2];

		dgVector p10 (p1 - p0);
		dgVector p20 (p2 - p0);
		const dgVector p_p0 (origin - p0);
		dgVector normal (p10 * p20);
		dgFloat32 volume = p_p0 % normal;
		if (volume < dgFloat32 (0.0f)) {
			dgVector q (dgPointToTriangleDistance (origin, p0, p1, p2, normal));
			dgVector qDist (q - origin);

			dgFloat32 dist = qDist % qDist;
			if (dist < minDist) {
				p = q;
				index0 = i0;
				index1 = i1;
				index2 = i2;
				minDist = dist;
			}
		}
	}

	if (index0 != -1) {
		dgVector tmpSum[2];
		dgVector tmpDiff[2];

		tmpDiff[0] = m_hullVertex[index1];
		tmpDiff[1] = m_hullVertex[index2];
		tmpSum[0] = m_averVertex[index1];
		tmpSum[1] = m_averVertex[index2];

		m_hullVertex[0] = m_hullVertex[index0];
		m_hullVertex[1] = tmpDiff[0];
		m_hullVertex[2] = tmpDiff[1];
		m_averVertex[0] = m_averVertex[index0];
		m_averVertex[1] = tmpSum[0];
		m_averVertex[2] = tmpSum[1];
		return ReduceTriangle (origin);
	}
	return p;
}

dgContactSolver::dgMinkReturnCode dgContactSolver::UpdateSeparatingPlaneFallbackSolutionLarge(dgMinkFace*& plane, const dgBigVector& origin)
{
	dgInt32 cycling = -1;
	dgFloat64 minDist = dgFloat64 (1.0e20f);
	dgMinkReturnCode code = dgMinkError;

	dgBigVector v (ReduceTetrahedrumLarge (origin) - origin);
	dgBigVector dir0 (dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f));

	for (dgInt32 i = 0; (i < DG_FALLBACK_SEPARATING_PLANE_ITERATIONS) && (m_vertexIndex < 4); i ++) {
		dgFloat64 dist = v % v;
		if (dist < dgFloat64 (1.0e-9f)) {
			switch (m_vertexIndex) 
			{
				case 1:
				{
					dgInt32 best = 0;
					dgFloat64 maxErr = dgFloat64 (0.0f);
					dgInt32 i = 0;
					for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
						dgFloat64 error2;
						CalcSupportVertexLarge (m_dir[i], 1);
						dgBigVector e (m_hullVertexLarge[1] - m_hullVertexLarge[0]);
						error2 = e % e;
						if (error2 > dgFloat64 (1.0e-4f)) {
							break;
						}
						if (error2 > maxErr) {
							best = i;
							maxErr = error2;
						}
					}

					if (i == dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
						dgAssert (maxErr > dgFloat64 (0.0f));
						CalcSupportVertexLarge (m_dir[best], 1);
					}
					m_vertexIndex = 2;

				}

				case 2:
				{
					dgInt32 best = 0;
					dgFloat64 maxErr = dgFloat64 (0.0f);
					dgBigVector e0 (m_hullVertexLarge[1] - m_hullVertexLarge[0]);
					dgInt32 i = 0;
					for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
						dgFloat64 error2;
						CalcSupportVertexLarge (m_dir[i], 2);
						dgBigVector e1 (m_hullVertexLarge[2] - m_hullVertexLarge[0]);
						dgBigVector n (e0 * e1);
						error2 = n % n;
						if (error2 > dgFloat64 (1.0e-4f)) {
							break;
						}
						if (error2 > maxErr) {
							best = i;
							maxErr = error2;
						}
					}

					if (i == dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
						dgAssert (maxErr > dgFloat64 (0.0f));
						CalcSupportVertexLarge (m_dir[best], 2);
					}
					m_vertexIndex = 3;
				}

				default:
				{
					const dgBigVector& p0 = m_hullVertexLarge[0];
					const dgBigVector& p1 = m_hullVertexLarge[1]; 
					const dgBigVector& p2 = m_hullVertexLarge[2];
					dgBigVector normal ((p1 - p0) * (p2 - p0));
					dgFloat64 mag2 = normal % normal;
					dgAssert (mag2 > dgFloat64 (1.0e-10f));
					normal = normal.Scale3 (dgFloat64 (1.0f)/ sqrt(mag2));
					dgVector dir (dgFloat32 (normal.m_x), dgFloat32 (normal.m_y), dgFloat32 (normal.m_z), dgFloat32 (0.0f)); 
					CalcSupportVertexLarge (dir, 3);
					CalcSupportVertexLarge (dir.Scale (dgFloat32 (-1.0f)), 4);
					if (fabs((m_hullVertexLarge[4] - p0) % normal) > fabs((m_hullVertexLarge[3] - p0) % normal)) {
						m_hullVertexLarge[3] = m_hullVertexLarge[4];
						m_averVertexLarge[3] = m_averVertexLarge[4];
					}
					m_vertexIndex = 4;

					if (!CheckTetrahedronVolumeLarge()) {
						dgSwap (m_hullVertexLarge[2], m_hullVertexLarge[1]);
						dgSwap (m_averVertexLarge[2], m_averVertexLarge[1]);
						dgAssert (CheckTetrahedronVolumeLarge());
					}
				}
			}
			return dgMinkIntersecting;
		}


		if (dist < minDist) {
			minDist = dist;
			cycling = -1;
		}


		dgAssert (dist > dgFloat64 (1.0e-24f));
		dgBigVector dir (v.Scale3 (- dgFloat64 (1.0f) / sqrt (dist)));
		dist = dir0 % dir;
		if (dist < dgFloat64 (0.9995f)) {
			dgVector dir1 (dgFloat32 (dir.m_x), dgFloat32 (dir.m_y), dgFloat32 (dir.m_z), dgFloat32 (0.0f)); 
			CalcSupportVertexLarge (dir1, m_vertexIndex);
			dgBigVector w (m_hullVertexLarge[m_vertexIndex] - origin);
			dgBigVector wv (w - v);
			dist = dir % wv;
		} else {
			dist = dgFloat64 (0.0f);
		}

		cycling ++;
		if (cycling > 4) {
			dist = dgFloat64 (0.0f);
		}

		dir0 = dir;
		if (dist < dgFloat64 (5.0e-4f)) {
			dgMatrix rotMatrix;
			//dgBigPlane separatingPlane (dir.Scale (dgFloat64 (-1.0f)), origin % dir);
			dgVector dir32 (dgFloat32 (dir.m_x), dgFloat32 (dir.m_y), dgFloat32 (dir.m_z), dgFloat32 (0.0f));
			dgPlane separatingPlane (dir32.Scale (dgFloat32 (-1.0f)), dgFloat32 (origin % dir));

			switch (m_vertexIndex) 
			{
				case 1:
				{
					dgFloat64 minDist = dgFloat64 (1.0e10f);
					rotMatrix = dgMatrix (dir32);
					dgAssert (rotMatrix.m_front.m_w == dgFloat64 (0.0f));
					dgAssert (rotMatrix.m_up.m_w == dgFloat64 (0.0f));
					dgAssert (rotMatrix.m_right.m_w == dgFloat64 (0.0f));


					dgInt32 keepSeaching = 1;
					dgVector dir1 (dgFloat32 (1.0), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat64 (0.0f) );
					// 2.0 degree rotation
					dgVector yawRoll (dgFloat32 (0.99939083f), dgFloat32 (0.0f), dgFloat32 (0.034899497f), dgFloat32 (0.0f));
					// 45 degree rotation
					dgVector yawPitch (dgFloat32 (0.0f), dgFloat64 (0.70710678f), dgFloat32 (0.70710678f), dgFloat32 (0.0f));
					for (dgInt32 j = 0; keepSeaching && (j < 180) ; j ++) {
						//						do {
						dgVector tmp (yawRoll.m_x * dir1.m_x - yawRoll.m_z * dir1.m_z, dgFloat64 (0.0f),
							yawRoll.m_z * dir1.m_x + yawRoll.m_x * dir1.m_z, dgFloat64 (0.0f));

						dgFloat32 val1 = tmp % tmp;
						if (dgAbsf (val1 - dgFloat32 (1.0f)) > dgFloat32 (1.0e-4f)) {
							tmp = tmp.Scale (dgRsqrt (val1));
						}

						dir1 = tmp;
						dgVector dir2 (dir1);
						for (dgInt32 i = 0; i < 8; i ++) {
							dgVector tmp (dir2.m_x, dir2.m_y * yawPitch.m_y - dir2.m_z * yawPitch.m_z, 
								dir2.m_y * yawPitch.m_z + dir2.m_z * yawPitch.m_y, dgFloat64 (0.0f));

							//dgAssert (dgAbsf ((tmp % tmp) - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));

							dir2 = tmp;
							tmp = rotMatrix.RotateVector(dir2);
							CalcSupportVertexLarge (tmp, 2);
							dgBigVector err0 (m_hullVertex[2] - m_hullVertex[0]);
							dgFloat64 val = err0 % err0;
							if (val > DG_FALLBACK_SEPARATING_DIST_TOLERANCE ) {
								//									val = separatingPlane.Evalue(m_hullVertex[2]);
								val = separatingPlane.m_x * m_hullVertexLarge[2].m_x +
									separatingPlane.m_y * m_hullVertexLarge[2].m_y +
									separatingPlane.m_z * m_hullVertexLarge[2].m_z +
									separatingPlane.m_w;
								dgAssert (val > dgFloat64 (0.0f));
								if (val < minDist) {
									keepSeaching = 0;
									minDist = val;
									m_hullVertexLarge[1] = m_hullVertexLarge[2];
									m_averVertexLarge[1] = m_averVertexLarge[2];
								}
							}
						}
						//} while (keepSeaching);
					}
					if (keepSeaching) {
						return dgMinkDisjoint;
					}
				}

				case 2:
				{
					rotMatrix.m_front = dir32;
					//rotMatrix.m_up = m_hullVertexLarge[1] - m_hullVertexLarge[0];
					//mag2 = rotMatrix.m_up % rotMatrix.m_up;
					dgBigVector up (m_hullVertexLarge[1] - m_hullVertexLarge[0]);
					dgFloat64 mag2 = up % up;
					dgAssert (mag2 > dgFloat64 (1.0e-24f));
					//rotMatrix.m_up = rotMatrix.m_up.Scale(dgRsqrt (mag2));
					up = up.Scale3(dgFloat64(1.0f) / sqrt (mag2));
					rotMatrix.m_up = dgVector (dgFloat32 (up.m_x), dgFloat32 (up.m_y), dgFloat32 (up.m_z), dgFloat32 (0.0f));
					rotMatrix.m_right = rotMatrix.m_front * rotMatrix.m_up;

					//rotMatrix.m_front.m_w = dgFloat64 (0.0f);
					//rotMatrix.m_up.m_w    = dgFloat64 (0.0f);
					rotMatrix.m_right.m_w = dgFloat64 (0.0f);

					dgFloat64 val = dgFloat64 (0.0f);
					// 2.0 degree rotation
					dgVector rot (dgFloat32 (0.99939083f), dgFloat32 (0.0f), dgFloat32 (0.034899497f), dgFloat32 (0.0f));
					dgVector dir1 (dgFloat64 (1.0), dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f) );
					do {
						dgVector tmp (rot.m_x * dir1.m_x - rot.m_z * dir1.m_z, dgFloat64 (0.0f),
							rot.m_z * dir1.m_x + rot.m_x * dir1.m_z, dgFloat64 (0.0f));

						dir1 = tmp;
						tmp = rotMatrix.RotateVector(dir1);
						tmp  = tmp .Scale (dgRsqrt(tmp  % tmp ));
						CalcSupportVertexLarge (tmp, 2);
						dgBigVector err0 (m_hullVertexLarge[2] - m_hullVertexLarge[0]);
						val = err0 % err0;
						if (val > DG_FALLBACK_SEPARATING_DIST_TOLERANCE ) {
							dgBigVector err0 (m_hullVertexLarge[2] - m_hullVertexLarge[1]);
							val = err0 % err0;
						}
					} while (val < DG_FALLBACK_SEPARATING_DIST_TOLERANCE);
					dgAssert (((m_hullVertexLarge[0] - m_hullVertexLarge[2]) % dir) >= dgFloat64 (-1.0e-1f));
					dir1 = dgVector (dgFloat64 (1.0), dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f) );

					do {
						dgVector tmp (rot.m_x * dir1.m_x + rot.m_z * dir1.m_z, dgFloat64 (0.0f),
							rot.m_x * dir1.m_z - rot.m_z * dir1.m_x, dgFloat64 (0.0f));

						dir1 = tmp;
						tmp = rotMatrix.RotateVector(dir1);
						tmp  = tmp .Scale (dgRsqrt(tmp  % tmp ));
						CalcSupportVertexLarge (tmp, 3);
						dgBigVector err0 (m_hullVertexLarge[3] - m_hullVertexLarge[0]);
						val = err0 % err0;
						if (val > DG_FALLBACK_SEPARATING_DIST_TOLERANCE) {
							dgBigVector err0 (m_hullVertexLarge[3] - m_hullVertexLarge[1]);
							val = err0 % err0;
						}
					} while (val < DG_FALLBACK_SEPARATING_DIST_TOLERANCE);
					dgAssert (((m_hullVertexLarge[0] - m_hullVertexLarge[3]) % dir) >= dgFloat64 (-1.0e-1f));
					//dist2 = separatingPlane.Evalue(m_hullVertexLarge[2]);
					//dist3 = separatingPlane.Evalue(m_hullVertexLarge[3]);
					dgFloat64 dist2 = separatingPlane.m_x * m_hullVertexLarge[2].m_x +
						separatingPlane.m_y * m_hullVertexLarge[2].m_y +
						separatingPlane.m_z * m_hullVertexLarge[2].m_z +
						separatingPlane.m_w;
					dgFloat64 dist3 = separatingPlane.m_x * m_hullVertexLarge[3].m_x +
						separatingPlane.m_y * m_hullVertexLarge[3].m_y +
						separatingPlane.m_z * m_hullVertexLarge[3].m_z +
						separatingPlane.m_w;

					dgAssert (dist2 > dgFloat64 (0.0f));
					dgAssert (dist3 > dgFloat64 (0.0f));
					if (dist3 < dist2) {
						m_hullVertexLarge[2] = m_hullVertexLarge[3];
						m_averVertexLarge[2] = m_averVertexLarge[3];
						dist2 = dist3;
					}
				}

				case 3:
				{
					CalcSupportVertexLarge (separatingPlane, 3);
				}
			}

			m_vertexIndex = 4;
			plane = &m_simplex[0];
			if (!CheckTetrahedronVolumeLarge()) {
				dgSwap (m_hullVertexLarge[2], m_hullVertexLarge[1]);
				dgSwap (m_averVertexLarge[2], m_averVertexLarge[1]);
				dgAssert (CheckTetrahedronVolumeLarge());
			}

			return dgMinkDisjoint;
		}

		m_vertexIndex ++;
		switch (m_vertexIndex) 
		{
			case 1:
			{
				dgAssert (0);
				break;
			}

			case 2:
			{
				v = ReduceLineLarge (origin) - origin;
				break;
			}

			case 3:
			{
				v = ReduceTriangleLarge (origin) - origin;
				break;
			}

			case 4:
			{
				v = ReduceTetrahedrumLarge (origin) - origin;
				break;
			}
		}
	}


	if (m_vertexIndex == 4) {
		if (!CheckTetrahedronVolumeLarge()) {
			dgSwap (m_hullVertexLarge[2], m_hullVertexLarge[1]);
			dgSwap (m_averVertexLarge[2], m_averVertexLarge[1]);
			dgAssert (CheckTetrahedronVolumeLarge());
		}

		dgFloat64 minDist = dgFloat64 (1.0e20f);
		for (dgInt32 i = 0; i < 4; i ++) {
			dgInt32 i0 = m_faceIndex[i][0];
			dgInt32 i1 = m_faceIndex[i][1];
			dgInt32 i2 = m_faceIndex[i][2];

			dgAssert (i0 == m_simplex[i].m_vertex[0]);
			dgAssert (i1 == m_simplex[i].m_vertex[1]);
			dgAssert (i2 == m_simplex[i].m_vertex[2]);

			const dgBigVector& p0 = m_hullVertexLarge[i0];
			const dgBigVector& p1 = m_hullVertexLarge[i1];
			const dgBigVector& p2 = m_hullVertexLarge[i2];
			dgBigVector e0 (p1 - p0);
			dgBigVector e1 (p2 - p0);
			dgBigVector n (e0 * e1);

			dgFloat64 dist = n % n;
			dgAssert (dist > dgFloat64 (1.0e-20f));
			if (dist > DG_DISTANCE_TOLERANCE_ZERO) {
				n = n.Scale3 (dgFloat32 (1.0f) / sqrt (dist));
				dist = fabs (n % (origin - p0));
				if (dist < minDist) {
					minDist = dist;
					plane = &m_simplex[i];
				}
			}
		}
		dgAssert (plane);
		code = dgMinkIntersecting;
	}
#ifdef _DEBUG
	if (m_vertexIndex < 4) {
		dgAssert (0);
		dgTrace (("too many iterations  in: %s\n",  __FUNCDNAME__));
	}
#endif
	return code;
}


dgContactSolver::dgMinkReturnCode dgContactSolver::UpdateSeparatingPlaneFallbackSolution(dgMinkFace*& plane, const dgVector& origin)
{
	dgInt32 cycling = -1;
	dgFloat32 minDist = dgFloat32 (1.0e20f);
	dgMinkReturnCode code = dgMinkError;

	dgVector v (ReduceTetrahedrum (origin) - origin);
	dgVector dir0 (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; (i < DG_FALLBACK_SEPARATING_PLANE_ITERATIONS) && (m_vertexIndex < 4); i ++) {
		dgFloat32 dist = v % v;
		if (dist < dgFloat32 (1.0e-9f)) {
			switch (m_vertexIndex) 
			{
				case 1:
				{
					dgInt32 best = 0;
					dgFloat32 maxErr = dgFloat32 (0.0f);
					dgInt32 i = 0;
					for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
						CalcSupportVertex (m_dir[i], 1);
						dgVector e (m_hullVertex[1] - m_hullVertex[0]);
						dgFloat32 error2 = e % e;
						if (error2 > dgFloat32 (1.0e-4f)) {
							break;
						}
						if (error2 > maxErr) {
							best = i;
							maxErr = error2;
						}
					}

					if (i == dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
						dgAssert (maxErr > dgFloat32 (0.0f));
						CalcSupportVertex (m_dir[best], 1);
					}
					m_vertexIndex = 2;

				}

				case 2:
				{
					dgInt32 best = 0;
					dgFloat32 maxErr = dgFloat32 (0.0f);
					dgVector e0 (m_hullVertex[1] - m_hullVertex[0]);
					dgInt32 i = 0;
					for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
						CalcSupportVertex (m_dir[i], 2);
						dgVector e1 (m_hullVertex[2] - m_hullVertex[0]);
						dgVector n (e0 * e1);
						dgFloat32 error2 = n % n;
						if (error2 > dgFloat32 (1.0e-4f)) {
							break;
						}
						if (error2 > maxErr) {
							best = i;
							maxErr = error2;
						}
					}

					if (i == dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
						dgAssert (maxErr > dgFloat32 (0.0f));
						CalcSupportVertex (m_dir[best], 2);
					}
					m_vertexIndex = 3;
				}

				default:
				{
					const dgVector& p0 = m_hullVertex[0];
					const dgVector& p1 = m_hullVertex[1]; 
					const dgVector& p2 = m_hullVertex[2];
					dgVector normal ((p1 - p0) * (p2 - p0));
					dgFloat32 mag2 = normal % normal;
					dgAssert (mag2 > dgFloat32 (1.0e-10f));
					normal = normal.Scale (dgRsqrt(mag2));
					CalcSupportVertex (normal, 3);
					CalcSupportVertex (normal.Scale (dgFloat32 (-1.0f)), 4);
					if (dgAbsf((m_hullVertex[4] - p0) % normal) > dgAbsf((m_hullVertex[3] - p0) % normal)) {
						m_hullVertex[3] = m_hullVertex[4];
						m_averVertex[3] = m_averVertex[4];
					}
					m_vertexIndex = 4;

					if (!CheckTetrahedronVolume()) {
						dgSwap (m_hullVertex[2], m_hullVertex[1]);
						dgSwap (m_averVertex[2], m_averVertex[1]);
						dgAssert (CheckTetrahedronVolume());
					}
				}
			}
			return dgMinkIntersecting;
		}


		if (dist < minDist) {
			minDist = dist;
			cycling = -1;
		}


		dgAssert (dist > dgFloat32 (1.0e-24f));
		dgVector dir (v.Scale (-dgRsqrt (dist)));
		dist = dir0 % dir;
		if (dist < dgFloat32 (0.9995f)) {
			CalcSupportVertex (dir, m_vertexIndex);
			dgVector w (m_hullVertex[m_vertexIndex] - origin);
			dgVector wv (w - v);
			dist = dir % wv;
		} else {
			dist = dgFloat32 (0.0f);
		}

		cycling ++;
		if (cycling > 4) {
			dist = dgFloat32 (0.0f);
		}

		dir0 = dir;
		if (dist < dgFloat32 (5.0e-4f)) {
			dgMatrix rotMatrix;
			dgPlane separatingPlane (dir.Scale (dgFloat32 (-1.0f)), origin % dir);
			switch (m_vertexIndex) 
			{
				case 1:
				{
					dgFloat32 minDist = dgFloat32 (1.0e10f);
					rotMatrix = dgMatrix (dir);
					dgAssert (rotMatrix.m_front.m_w == dgFloat32 (0.0f));
					dgAssert (rotMatrix.m_up.m_w == dgFloat32 (0.0f));
					dgAssert (rotMatrix.m_right.m_w == dgFloat32 (0.0f));

					dgInt32 keepSeaching = 1;
					dgVector dir1 (dgFloat32 (1.0), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f) );
					// 2.0 degree rotation
					dgVector yawRoll (dgFloat32 (0.99939083f), dgFloat32 (0.0f), dgFloat32 (0.034899497f), dgFloat32 (0.0f));
					// 45 degree rotation
					dgVector yawPitch (dgFloat32 (0.0f), dgFloat32 (0.70710678f), dgFloat32 (0.70710678f), dgFloat32 (0.0f));
					//						do {
					for (dgInt32 j = 0; keepSeaching && (j < 180) ; j ++) {
						dgFloat32 val;
						dgVector tmp (yawRoll.m_x * dir1.m_x - yawRoll.m_z * dir1.m_z, dgFloat32 (0.0f),
							yawRoll.m_z * dir1.m_x + yawRoll.m_x * dir1.m_z, dgFloat32 (0.0f));

						val = tmp % tmp;
						if (dgAbsf (val - dgFloat32 (1.0f)) > dgFloat32 (1.0e-4f)) {
							tmp = tmp.Scale (dgRsqrt (val));
						}

						dir1 = tmp;
						dgVector dir2 (dir1);
						for (dgInt32 i = 0; i < 8; i ++) {
							dgVector tmp (dir2.m_x, dir2.m_y * yawPitch.m_y - dir2.m_z * yawPitch.m_z, 
								dir2.m_y * yawPitch.m_z + dir2.m_z * yawPitch.m_y, dgFloat32 (0.0f));

							dgAssert (dgAbsf ((tmp % tmp) - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));

							dir2 = tmp;
							tmp = rotMatrix.RotateVector(dir2);
							CalcSupportVertex (tmp, 2);
							dgVector err0 (m_hullVertex[2] - m_hullVertex[0]);
							val = err0 % err0;
							if (val > DG_FALLBACK_SEPARATING_DIST_TOLERANCE ) {
								val = separatingPlane.Evalue(m_hullVertex[2]);
								dgAssert (val > dgFloat32 (0.0f));
								if (val < minDist) {
									keepSeaching = 0;
									minDist = val;
									m_hullVertex[1] = m_hullVertex[2];
									m_averVertex[1] = m_averVertex[2];
								}
							}
						}
						//						} while (keepSeaching);
					}

					if (keepSeaching) {
						return dgMinkDisjoint;
					}
				}

				case 2:
				{
					rotMatrix.m_front = dir;
					rotMatrix.m_up = m_hullVertex[1] - m_hullVertex[0];
					dgFloat32 mag2 = rotMatrix.m_up % rotMatrix.m_up;
					dgAssert (mag2 > dgFloat32 (1.0e-24f));
					rotMatrix.m_up = rotMatrix.m_up.Scale(dgRsqrt (mag2));
					rotMatrix.m_right = rotMatrix.m_front * rotMatrix.m_up;

					rotMatrix.m_front.m_w = dgFloat32 (0.0f);
					rotMatrix.m_up.m_w    = dgFloat32 (0.0f);
					rotMatrix.m_right.m_w = dgFloat32 (0.0f);

					dgFloat32 val = dgFloat32 (0.0f);
					// 2.0 degree rotation
					dgVector rot (dgFloat32 (0.99939083f), dgFloat32 (0.0f), dgFloat32 (0.034899497f), dgFloat32 (0.0f));
					dgVector dir1 (dgFloat32 (1.0), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f) );
					do {
						dgVector tmp (rot.m_x * dir1.m_x - rot.m_z * dir1.m_z, dgFloat32 (0.0f),
							rot.m_z * dir1.m_x + rot.m_x * dir1.m_z, dgFloat32 (0.0f));

						dir1 = tmp;
						tmp = rotMatrix.RotateVector(dir1);
						tmp  = tmp .Scale (dgRsqrt(tmp  % tmp ));
						CalcSupportVertex (tmp, 2);
						dgVector err0 (m_hullVertex[2] - m_hullVertex[0]);
						val = err0 % err0;
						if (val > DG_FALLBACK_SEPARATING_DIST_TOLERANCE ) {
							dgVector err0 (m_hullVertex[2] - m_hullVertex[1]);
							val = err0 % err0;
						}
					} while (val < DG_FALLBACK_SEPARATING_DIST_TOLERANCE);
#ifdef _DEBUG
					dgFloat32 test = (m_hullVertex[0] - m_hullVertex[2]) % dir;
					dgAssert (test  >= dgFloat32 (-2.0e-1f));
#endif
					dir1 = dgVector (dgFloat32 (1.0), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f) );

					do {
						dgVector tmp (rot.m_x * dir1.m_x + rot.m_z * dir1.m_z, dgFloat32 (0.0f),
							rot.m_x * dir1.m_z - rot.m_z * dir1.m_x, dgFloat32 (0.0f));

						dir1 = tmp;
						tmp = rotMatrix.RotateVector(dir1);
						tmp  = tmp .Scale (dgRsqrt(tmp  % tmp ));
						CalcSupportVertex (tmp, 3);
						dgVector err0 (m_hullVertex[3] - m_hullVertex[0]);
						val = err0 % err0;
						if (val > DG_FALLBACK_SEPARATING_DIST_TOLERANCE) {
							dgVector err0 (m_hullVertex[3] - m_hullVertex[1]);
							val = err0 % err0;
						}
					} while (val < DG_FALLBACK_SEPARATING_DIST_TOLERANCE);
#ifdef _DEBUG
					dgFloat32 test1 = (m_hullVertex[0] - m_hullVertex[3]) % dir;
					dgAssert (test1  >= dgFloat32 (-2.0e-1f));
#endif


					dgFloat32 dist2 = separatingPlane.Evalue(m_hullVertex[2]);
					dgFloat32 dist3 = separatingPlane.Evalue(m_hullVertex[3]);
					dgAssert (dist2 > dgFloat32 (0.0f));
					dgAssert (dist3 > dgFloat32 (0.0f));
					if (dist3 < dist2) {
						m_hullVertex[2] = m_hullVertex[3];
						m_averVertex[2] = m_averVertex[3];
						dist2 = dist3;
					}
				}

				case 3:
				{
					CalcSupportVertex (separatingPlane, 3);
				}
			}

			m_vertexIndex = 4;
			plane = &m_simplex[0];
			if (!CheckTetrahedronVolume()) {
				dgSwap (m_hullVertex[2], m_hullVertex[1]);
				dgSwap (m_averVertex[2], m_averVertex[1]);
				dgAssert (CheckTetrahedronVolume());
			}

			return dgMinkDisjoint;
		}

		m_vertexIndex ++;
		switch (m_vertexIndex) 
		{
			case 1:
			{
				dgAssert (0);
				break;
			}

			case 2:
			{
				v = ReduceLine (origin) - origin;
				break;
			}

			case 3:
			{
				v = ReduceTriangle (origin) - origin;
				break;
			}

			case 4:
			{
				v = ReduceTetrahedrum (origin) - origin;
				break;
			}
		}
	}

	if (m_vertexIndex == 4) {
		dgFloat32 minDist;
		if (!CheckTetrahedronVolume()) {
			dgSwap (m_hullVertex[2], m_hullVertex[1]);
			dgSwap (m_averVertex[2], m_averVertex[1]);
			dgAssert (CheckTetrahedronVolume());
		}

		minDist = dgFloat32 (1.0e20f);
		for (dgInt32 i = 0; i < 4; i ++) {
			//dgFloat32 dist;
			dgInt32 i0 = m_faceIndex[i][0];
			dgInt32 i1 = m_faceIndex[i][1];
			dgInt32 i2 = m_faceIndex[i][2];

			dgAssert (i0 == m_simplex[i].m_vertex[0]);
			dgAssert (i1 == m_simplex[i].m_vertex[1]);
			dgAssert (i2 == m_simplex[i].m_vertex[2]);

			const dgVector& p0 = m_hullVertex[i0];
			const dgVector& p1 = m_hullVertex[i1];
			const dgVector& p2 = m_hullVertex[i2];
			dgVector e0 (p1 - p0);
			dgVector e1 (p2 - p0);
			dgVector n (e0 * e1);

			dgFloat32 dist = n % n;
			dgAssert (dist > dgFloat32 (1.0e-20f));
			if (dist > DG_DISTANCE_TOLERANCE_ZERO) {
				n = n.Scale (dgRsqrt (dist));
				dist = dgAbsf (n % (origin - p0));
				if (dist < minDist) {
					minDist = dist;
					plane = &m_simplex[i];
				}
			}
		}
		dgAssert (plane);
		code = dgMinkIntersecting;
	}
#ifdef _DEBUG
	if (m_vertexIndex < 4) {
		dgAssert (0);
		dgTrace (("too many iterations  in: %s\n",  __FUNCDNAME__));
	}
#endif

	return code;
}


dgContactSolver::dgMinkReturnCode dgContactSolver::UpdateSeparatingPlaneLarge(dgMinkFace*& plane, const dgBigVector& origin)
{
	dgBigVector diff[4];
	dgBigVector aveg[4];

	dgBigVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	plane = NULL;
	dgMinkFace* face = &m_simplex[0];
	dgMinkReturnCode code = dgMinkIntersecting;

	dgInt32 cyclingCount = -1;
	dgMinkFace* lastDescendFace = NULL;
	dgFloat64 minDist = dgFloat32 (1.0e20f);


	// this loop can calculate the closest point to the origin usually in 4 to 5 passes,
	dgInt32 j = 0;
	for (; face && (j < DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION); j ++) {
		face = NULL;
		// initialize distance to zero (very important)
		dgFloat64 maxDist = dgFloat32 (0.0f);

		for (dgInt32 i = 0; i < 4; i ++) {
			dgInt32 i0 = m_faceIndex[i][0];
			dgInt32 i1 = m_faceIndex[i][1];
			dgInt32 i2 = m_faceIndex[i][2];

			dgAssert (i0 == m_simplex[i].m_vertex[0]);
			dgAssert (i1 == m_simplex[i].m_vertex[1]);
			dgAssert (i2 == m_simplex[i].m_vertex[2]);

			const dgBigVector& p0 = m_hullVertexLarge[i0];
			const dgBigVector& p1 = m_hullVertexLarge[i1];
			const dgBigVector& p2 = m_hullVertexLarge[i2];

			dgBigVector e0 (p1 - p0);
			dgBigVector e1 (p2 - p0);
			dgBigVector n (e0 * e1);

			dgFloat64 dist = n % n;
			if (dist > DG_DISTANCE_TOLERANCE_ZERO) {
				n = n.Scale3 (dgFloat64 (1.0f)/ sqrt(dist));
				dist = n % (origin - p0);

				// find the plane farther away from the origin
				if (dist > maxDist) {
					maxDist = dist;
					normal = n;
					face = &m_simplex[i];
				}
			}
		}


		// if we do not have a face at this point it means that the mink shape of the tow convexity face have a very 
		// skew ratios on floating point accuracy is not enough to guarantee convexity of the shape
		if (face) {
			dgInt32 index = face->m_vertex[0];
			dgVector dir (dgFloat32 (normal.m_x), dgFloat32 (normal.m_y), dgFloat32 (normal.m_z), 0.0f);
			CalcSupportVertexLarge (dir, 4);

			dgFloat64 dist = normal % (m_hullVertexLarge[4] - m_hullVertexLarge[index]);

			// if we are doing too many passes it means that it is a skew shape with big and small floats  
			// significant bits may be lost in dist calculation, increasing the tolerance help to resolve the problem
			if(dist < DG_UPDATE_SEPARATING_PLANE_DISTANCE_TOLERANCE1) {
				plane = face;
				code = dgMinkDisjoint;
				break;
			}

			if (dist < minDist) {
				minDist = dist;
				lastDescendFace = face;
				cyclingCount = -1;
				for (dgInt32 k = 0; k < 4; k ++) {
					diff[k] = m_hullVertexLarge[k];
					aveg[k] = m_averVertexLarge[k];
				}
			}

			cyclingCount ++;
			if (cyclingCount > 4) {
				for (dgInt32 k = 0; k < 4; k ++) {
					m_hullVertexLarge[k] = diff[k];
					m_averVertexLarge[k] = aveg[k];
				}
				code = dgMinkDisjoint;
				plane = lastDescendFace;
				break;
			}


			if (dist < DG_DISTANCE_TOLERANCE) {
				dgInt32 i = 0;
				for (; i < 4; i ++ ) {
					dgBigVector error (m_hullVertexLarge[i] - m_hullVertexLarge[4]);
					if ((error % error) < (DG_DISTANCE_TOLERANCE * DG_DISTANCE_TOLERANCE)) {
						plane = face;
						//code = dgMinkDisjoint;
						code = UpdateSeparatingPlaneFallbackSolutionLarge (plane, origin);
						dgAssert ((code == dgMinkDisjoint) || ((code == dgMinkIntersecting) && (m_vertexIndex == 4)));
						break;
					}
				}
				if (i < 4) {
					break;
				}
			}

			dgInt32 i0 = face->m_vertex[0];
			dgInt32 i1 = face->m_vertex[1];
			dgInt32 i2 = m_faceIndex[face - m_simplex][3];
			dgAssert (i2 != face->m_vertex[0]);
			dgAssert (i2 != face->m_vertex[1]);
			dgAssert (i2 != face->m_vertex[2]);
			dgSwap (m_hullVertexLarge[i0], m_hullVertexLarge[i1]);
			dgSwap (m_averVertexLarge[i0], m_averVertexLarge[i1]);
			m_hullVertexLarge[i2] = m_hullVertexLarge[4];
			m_averVertexLarge[i2] = m_averVertexLarge[4];
			if (!CheckTetrahedronVolumeLarge ()) {
				dgSwap (m_hullVertexLarge[1], m_hullVertexLarge[2]);
				dgSwap (m_averVertexLarge[1], m_averVertexLarge[2]);
				dgAssert (CheckTetrahedronVolumeLarge ());
			}
		}

	} 

	if (j >= DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION) {
		dgAssert (CheckTetrahedronVolumeLarge());
		code = UpdateSeparatingPlaneFallbackSolutionLarge (plane, origin);
	}
	return code;
}


dgContactSolver::dgMinkReturnCode dgContactSolver::UpdateSeparatingPlane(dgMinkFace*& plane, const dgVector& origin)
{
	dgVector diff[4];
	dgVector aveg[4];


	plane = NULL;
	dgInt32 faceIndex = 0;
	dgMinkFace* lastDescendFace = NULL;
	dgMinkReturnCode code = dgMinkIntersecting;

	// this loop can calculate the closest point to the origin usually in 4 to 5 passes,
	dgInt32 j = 0;
	dgInt32 cyclingCount = -1;
	dgFloat32 minDist = dgFloat32 (1.0e20f);
	for (; (faceIndex != -1) && (j < DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION); j ++) {
		faceIndex = -1;
		dgVector normal;
		// initialize distance to zero (very important)
		dgFloat32 maxDist = dgFloat32 (0.0f);
		for (dgInt32 i = 0; i < 4; i ++) {
			dgInt32 i0 = m_faceIndex[i][0];
			dgInt32 i1 = m_faceIndex[i][1];
			dgInt32 i2 = m_faceIndex[i][2];

			dgAssert (i0 == m_simplex[i].m_vertex[0]);
			dgAssert (i1 == m_simplex[i].m_vertex[1]);
			dgAssert (i2 == m_simplex[i].m_vertex[2]);

			const dgVector& p0 = m_hullVertex[i0];
			const dgVector& p1 = m_hullVertex[i1];
			const dgVector& p2 = m_hullVertex[i2];
			dgVector e0 (p1 - p0);
			dgVector e1 (p2 - p0);
			dgVector n (e0 * e1);

			dgFloat32 dist = n % n;
			if (dist > DG_DISTANCE_TOLERANCE_ZERO) {
				n = n.Scale (dgRsqrt (dist));
				dist = n % (origin - p0);

				// find the plane farther away from the origin
				if (dist > maxDist) {
					maxDist = dist;
					normal = n;
					//face = &m_simplex[i];
					faceIndex = i;
				}
			}
		}


		// if we do not have a face at this point it means that the mink shape of the tow convexity face have a very 
		// skew ratios on floating point accuracy is not enough to guarantee convexity of the shape
		if (faceIndex != -1) {
			dgMinkFace* const face = &m_simplex[faceIndex];
			dgInt32 index = face->m_vertex[0];
			CalcSupportVertex (normal, 4);
			dgFloat32 dist = normal % (m_hullVertex[4] - m_hullVertex[index]);

			// if we are doing too many passes it means that it is a skew shape with big and small floats  
			// significant bits may be lost in dist calculation, increasing the tolerance help to resolve the problem
			if(dist < DG_UPDATE_SEPARATING_PLANE_DISTANCE_TOLERANCE1) {
				plane = face;
				code = dgMinkDisjoint;
				break;
			}

			if (dist < minDist) {
				minDist = dist;
				lastDescendFace = face;
				cyclingCount = -1;
				for (dgInt32 k = 0; k < 4; k ++) {
					diff[k] = m_hullVertex[k];
					aveg[k] = m_averVertex[k];
				}
			}

			cyclingCount ++;
			if (cyclingCount > 4) {
				for (dgInt32 k = 0; k < 4; k ++) {
					m_hullVertex[k] = diff[k];
					m_averVertex[k] = aveg[k];
				}
				code = dgMinkDisjoint;
				plane = lastDescendFace;
				break;
			}


			if (dist < DG_DISTANCE_TOLERANCE) {
				dgInt32 i = 0;
				for (; i < 4; i ++ ) {
					dgVector error (m_hullVertex[i] - m_hullVertex[4]);
					if ((error % error) < (DG_DISTANCE_TOLERANCE * DG_DISTANCE_TOLERANCE)) {
						plane = face;
						code = UpdateSeparatingPlaneFallbackSolution (plane, origin);
						dgAssert ((code == dgMinkDisjoint) || ((code == dgMinkIntersecting) && (m_vertexIndex == 4)));
						break;
					}
				}
				if (i < 4) {
					break;
				}
			}

			//faceIndex = -1;
			dgInt32 i0 = face->m_vertex[0];
			dgInt32 i1 = face->m_vertex[1];
			dgInt32 i2 = m_faceIndex[face - m_simplex][3];
			
			dgAssert (i2 != face->m_vertex[0]);
			dgAssert (i2 != face->m_vertex[1]);
			dgAssert (i2 != face->m_vertex[2]);
			dgSwap (m_hullVertex[i0], m_hullVertex[i1]);
			dgSwap (m_averVertex[i0], m_averVertex[i1]);

			m_hullVertex[i2] = m_hullVertex[4];
			m_averVertex[i2] = m_averVertex[4];

			dgAssert (dgAbsf ((normal% normal) - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

			if (!CheckTetrahedronVolume ()) {
				dgSwap (m_hullVertex[1], m_hullVertex[2]);
				dgSwap (m_averVertex[1], m_averVertex[2]);

				dgAssert (CheckTetrahedronVolume ());
			}
		}
	} 

	if (j >= DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION) {
		dgAssert (CheckTetrahedronVolume());
		code = UpdateSeparatingPlaneFallbackSolution (plane, origin);
	}

	return code;
}


dgContactSolver::dgMinkReturnCode dgContactSolver::CalcSeparatingPlaneLarge(dgMinkFace*& plane, const dgBigVector& origin)
{
	dgFloat64 error2;
	dgBigVector e1;
	dgBigVector e2;
	dgBigVector e3;
	dgBigVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	CalcSupportVertexLarge (m_dir[0], 0);
	dgInt32 i = 1;
	for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		dgFloat64 error2;
		CalcSupportVertexLarge (m_dir[i], 1);
		e1 = m_hullVertexLarge[1] - m_hullVertexLarge[0];
		error2 = e1 % e1;
		if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR) {
			break;
		}
	}

	for (i ++; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		dgFloat64 error2;
		CalcSupportVertexLarge (m_dir[i], 2);
		e2 = m_hullVertexLarge[2] - m_hullVertexLarge[0];
		normal = e1 * e2;
		error2 = normal % normal;
		if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR1) {
			break;
		}
	}

	error2 = dgFloat32 (0.0f);
	for (i ++; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertexLarge (m_dir[i], 3);
		e3 = m_hullVertexLarge[3] - m_hullVertexLarge[0];
		error2 = normal % e3;
		if (fabs (error2) > DG_CALCULATE_SEPARATING_PLANE_ERROR1) {
			break;
		}
	}


	if (i >= dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
		dgInt32 best;
		dgFloat64 maxErr;

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertexLarge (m_dir[i], 1);
			e1 = m_hullVertexLarge[1] - m_hullVertexLarge[0];
			error2 = e1 % e1;
			if (error2 > maxErr) {
				best = i;
				maxErr = error2;
			}
		}
		CalcSupportVertexLarge (m_dir[best], 1);
		e1 = m_hullVertexLarge[1] - m_hullVertexLarge[0];

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertexLarge (m_dir[i], 2);
			dgBigVector e2 (m_hullVertexLarge[2] - m_hullVertexLarge[0]);
			normal = e1 * e2;
			error2 = normal % normal;
			if (error2 > maxErr) {
				best = i;
				maxErr = error2;
			}
		}

		CalcSupportVertexLarge (m_dir[best], 2);
		dgBigVector e2 (m_hullVertexLarge[2] - m_hullVertexLarge[0]);
		normal = e1 * e2;

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertexLarge (m_dir[i], 3);

			dgBigVector e3 (m_hullVertexLarge[3] - m_hullVertexLarge[0]);
			error2 = normal % e3;
			if (fabs (error2) > fabs (maxErr)) {
				best = i;
				maxErr = error2;
			}
		}
		error2 = maxErr;
		CalcSupportVertexLarge (m_dir[best], 3);
	}


	m_vertexIndex = 4;
	if (error2 > dgFloat32 (0.0f)) {
		dgSwap (m_hullVertexLarge[1], m_hullVertexLarge[2]);
		dgSwap (m_averVertexLarge[1], m_averVertexLarge[2]);
	}
	dgAssert (CheckTetrahedronVolumeLarge ());

	dgAssert ( (((dgUnsigned64)&m_simplex[0]) & 0x0f)== 0);
	dgAssert ( (((dgUnsigned64)&m_simplex[1]) & 0x0f)== 0);

	// face 0
	m_simplex[0].m_vertex[0] = 0;
	m_simplex[0].m_vertex[1] = 1;
	m_simplex[0].m_vertex[2] = 2;
	m_simplex[0].m_vertex[3] = 0;
	m_simplex[0].m_adjancentFace[0] = 1;	
	m_simplex[0].m_adjancentFace[1] = 3;	
	m_simplex[0].m_adjancentFace[2] = 2;	

	// face 1
	m_simplex[1].m_vertex[0] = 1;
	m_simplex[1].m_vertex[1] = 0;
	m_simplex[1].m_vertex[2] = 3;
	m_simplex[1].m_vertex[3] = 1;
	m_simplex[1].m_adjancentFace[0] = 0;	
	m_simplex[1].m_adjancentFace[1] = 2;	
	m_simplex[1].m_adjancentFace[2] = 3;	

	// face 2
	m_simplex[2].m_vertex[0] = 0;
	m_simplex[2].m_vertex[1] = 2;
	m_simplex[2].m_vertex[2] = 3;
	m_simplex[2].m_vertex[3] = 0;
	m_simplex[2].m_adjancentFace[0] = 0;	
	m_simplex[2].m_adjancentFace[1] = 3;	
	m_simplex[2].m_adjancentFace[2] = 1;	


	// face 3
	m_simplex[3].m_vertex[0] = 2;
	m_simplex[3].m_vertex[1] = 1;
	m_simplex[3].m_vertex[2] = 3;
	m_simplex[3].m_vertex[3] = 2;
	m_simplex[3].m_adjancentFace[0] = 0;	
	m_simplex[3].m_adjancentFace[1] = 1;	
	m_simplex[3].m_adjancentFace[2] = 2;	

	return UpdateSeparatingPlaneLarge(plane, origin);
}



dgContactSolver::dgMinkReturnCode dgContactSolver::CalcSeparatingPlane(dgMinkFace*& plane, const dgVector& origin)
{
	dgVector e1;
	dgVector e2;
	dgVector e3;

	dgFloat32 error2 = dgFloat32 (0.0f);

	dgVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));


	CalcSupportVertex (m_dir[0], 0);

	dgInt32 i = 1;
	for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertex (m_dir[i], 1);
		e1 = m_hullVertex[1] - m_hullVertex[0];
		error2 = e1 % e1;
		if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR) {
			break;
		}
	}

	for (i ++; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertex (m_dir[i], 2);
		e2 = m_hullVertex[2] - m_hullVertex[0];
		normal = e1 * e2;
		error2 = normal % normal;
		if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR1) {
			break;
		}
	}

	error2 = dgFloat32 (0.0f);
	for (i ++; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertex (m_dir[i], 3);
		e3 = m_hullVertex[3] - m_hullVertex[0];
		error2 = normal % e3;
		if (dgAbsf (error2) > DG_CALCULATE_SEPARATING_PLANE_ERROR1) {
			break;
		}
	}

	if (i >= dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
		dgInt32 best = 0;
		dgFloat32 maxErr = dgFloat32 (0.0f);
		for (dgInt32 i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertex (m_dir[i], 1);
			e1 = m_hullVertex[1] - m_hullVertex[0];
			error2 = e1 % e1;
			if (error2 > maxErr) {
				best = i;
				maxErr = error2;
			}
		}
		CalcSupportVertex (m_dir[best], 1);
		e1 = m_hullVertex[1] - m_hullVertex[0];

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (dgInt32 i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertex (m_dir[i], 2);
			e2 = m_hullVertex[2] - m_hullVertex[0];
			normal = e1 * e2;
			error2 = normal % normal;
			if (error2 > maxErr) {
				best = i;
				maxErr = error2;
			}
		}

		CalcSupportVertex (m_dir[best], 2);
		e2 = m_hullVertex[2] - m_hullVertex[0];
		normal = e1 * e2;

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (dgInt32 i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertex (m_dir[i], 3);

			e3 = m_hullVertex[3] - m_hullVertex[0];
			error2 = normal % e3;
			if (dgAbsf (error2) > dgAbsf (maxErr)) {
				best = i;
				maxErr = error2;
			}
		}
		error2 = maxErr;
		CalcSupportVertex (m_dir[best], 3);
	}


	m_vertexIndex = 4;
	if (error2 > dgFloat32 (0.0f)) {
		dgSwap (m_hullVertex[1], m_hullVertex[2]);
		dgSwap (m_averVertex[1], m_averVertex[2]);
	}
	dgAssert (CheckTetrahedronVolume ());

	dgAssert ( (((dgUnsigned64)&m_simplex[0]) & 0x0f)== 0);
	dgAssert ( (((dgUnsigned64)&m_simplex[1]) & 0x0f)== 0);

	// face 0
	m_simplex[0].m_vertex[0] = 0;
	m_simplex[0].m_vertex[1] = 1;
	m_simplex[0].m_vertex[2] = 2;
	m_simplex[0].m_vertex[3] = 0;
	m_simplex[0].m_adjancentFace[0] = 1;	
	m_simplex[0].m_adjancentFace[1] = 3;	
	m_simplex[0].m_adjancentFace[2] = 2;	

	// face 1
	m_simplex[1].m_vertex[0] = 1;
	m_simplex[1].m_vertex[1] = 0;
	m_simplex[1].m_vertex[2] = 3;
	m_simplex[1].m_vertex[3] = 1;
	m_simplex[1].m_adjancentFace[0] = 0;	
	m_simplex[1].m_adjancentFace[1] = 2;	
	m_simplex[1].m_adjancentFace[2] = 3;	

	// face 2
	m_simplex[2].m_vertex[0] = 0;
	m_simplex[2].m_vertex[1] = 2;
	m_simplex[2].m_vertex[2] = 3;
	m_simplex[2].m_vertex[3] = 0;
	m_simplex[2].m_adjancentFace[0] = 0;	
	m_simplex[2].m_adjancentFace[1] = 3;	
	m_simplex[2].m_adjancentFace[2] = 1;	


	// face 3
	m_simplex[3].m_vertex[0] = 2;
	m_simplex[3].m_vertex[1] = 1;
	m_simplex[3].m_vertex[2] = 3;
	m_simplex[3].m_vertex[3] = 2;
	m_simplex[3].m_adjancentFace[0] = 0;	
	m_simplex[3].m_adjancentFace[1] = 1;	
	m_simplex[3].m_adjancentFace[2] = 2;	


   return UpdateSeparatingPlane(plane, origin);
}



dgInt32 dgContactSolver::CalculateClosestPoints ()
{
	dgMinkFace* face;

	dgVector contactA (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector contactB (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgMinkReturnCode code = CalcSeparatingPlane(face);
	if (code == dgMinkDisjoint) {
		dgAssert (face);
		dgVector hullVertex[3];
		dgVector averVertex[3];

		for (dgInt32 i = 0; i < 3; i ++) {
			hullVertex[i] = m_hullVertex[face->m_vertex[i]];
			averVertex[i] = m_averVertex[face->m_vertex[i]];
		}
		for (dgInt32 i = 0; i < 3; i ++) {
			m_hullVertex[i] = hullVertex[i];
			m_averVertex[i] = averVertex[i];
		}

		m_vertexIndex = 3;
		dgVector dir0 (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector origin (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector v (ReduceTriangle (origin));
		for (dgInt32 i = 0; (i < 32) && (m_vertexIndex < 4); i ++) {
			//				dist = v % v + dgFloat32 (1.0e-12f);
			dgFloat32 dist = v % v;
			dgVector dir (dir0);
			if (dist > dgFloat32 (1.0e-12f)) {
				dgAssert (dist > DG_DISTANCE_TOLERANCE_ZERO);

				dir = v.Scale (-dgRsqrt (dist));
				dist = dir0 % dir;
				if (dist < dgFloat32 (0.9995f)) {
					CalcSupportVertex (dir, m_vertexIndex);
					const dgVector& w = m_hullVertex[m_vertexIndex];
					dgVector wv (w - v);
					dist = dir % wv;
				} else {
					dist = dgFloat32 (0.0f);
				}
			} else {
				dist = dgFloat32 (0.0f);
			}

			dir0 = dir;

			//				dgAssert (0); // !!! this can be take outside the loop
			//				if (dist < dgFloat32 (5.0e-4f)) {
			if (dist < (DG_DISTANCE_TOLERANCE * dgFloat32 (0.5f))) 
			{
				switch (m_vertexIndex) 
				{
					case 1:
					{
						contactA = m_hullVertex[0] + m_averVertex[0];
						contactB = m_averVertex[0] - m_hullVertex[0];
						break;
					}

					case 2:
					{
						dgVector lp0 (m_hullVertex[0] + m_averVertex[0]);
						dgVector lp1 (m_hullVertex[1] + m_averVertex[1]);
						dgVector lr0 (m_averVertex[0] - m_hullVertex[0]);
						dgVector lr1 (m_averVertex[1] - m_hullVertex[1]);

						dgVector dp  (m_hullVertex[1] - m_hullVertex[0]);

						dgFloat32 alpha1 = - (m_hullVertex[0] % dp) / (dp % dp + dgFloat32 (1.0e-24f));
						dgFloat32 alpha0 = dgFloat32 (1.0f) - alpha1;
						dgAssert (alpha1 >= dgFloat32 (0.0f));
						dgAssert (alpha1 <= dgFloat32 (1.0f));

						contactA = lp0.Scale (alpha0) + lp1.Scale (alpha1);
						contactB = lr0.Scale (alpha0) + lr1.Scale (alpha1);
						break;
					}

					case 3:
					{
						dgAssert (0);
						//this is numrically unsatble;
/*
						dgVector lp0 (m_hullVertex[0] + m_averVertex[0]);
						dgVector lp1 (m_hullVertex[1] + m_averVertex[1]);
						dgVector lp2 (m_hullVertex[2] + m_averVertex[2]);
						dgVector lr0 (m_averVertex[0] - m_hullVertex[0]);
						dgVector lr1 (m_averVertex[1] - m_hullVertex[1]);
						dgVector lr2 (m_averVertex[2] - m_hullVertex[2]);

						const dgVector& p0 = m_hullVertex[0];
						const dgVector& p1 = m_hullVertex[1];
						const dgVector& p2 = m_hullVertex[2];

						dgVector p10 (p1 - p0);
						dgVector p20 (p2 - p0);

						dgFloat32 a11 = p10 % p10;
						dgFloat32 a22 = p20 % p20;
						dgFloat32 a21 = p10 % p20;
						dgFloat32 b1 = - (p10 % p0);
						dgFloat32 b2 = - (p20 % p0);
						dgFloat32 den = a11 * a22 - a21 * a21;

						dgFloat32 alpha0;
						dgFloat32 alpha1;
						dgFloat32 alpha2;
						if (den > dgFloat32 (1.0e-7f)) {
							dgAssert (den > dgFloat32 (0.0f));
							alpha1 = b1 * a22 - a21 * b2;
							alpha2 = a11 * b2 - b1 * a21;
							alpha0 = den - alpha1 - alpha2;

							den = dgFloat32 (1.0f) / den;

							alpha0 *= den;
							alpha1 *= den;
							alpha2 *= den;
						} else {
							alpha0 = dgFloat32 (0.33f);
							alpha1 = dgFloat32 (0.33f);
							alpha2 = dgFloat32 (0.33f);
						}

						dgAssert (alpha0 >= dgFloat32 (-2.0e-2f));
						dgAssert (alpha1 >= dgFloat32 (-2.0e-2f));
						dgAssert (alpha2 >= dgFloat32 (-2.0e-2f));
						dgAssert (alpha0 <= dgFloat32 ( 1.0f + 2.0e-2f));
						dgAssert (alpha1 <= dgFloat32 ( 1.0f + 2.0e-2f));				
						dgAssert (alpha2 <= dgFloat32 ( 1.0f + 2.0e-2f));

						contactA = lp0.Scale (alpha0) + lp1.Scale (alpha1) + lp2.Scale (alpha2);
						contactB = lr0.Scale (alpha0) + lr1.Scale (alpha1) + lr2.Scale (alpha2);
*/
						break;
					}
				}


				//contactA = m_proxy->m_referenceMatrix.TransformVector (contactA.Scale (dgFloat32 (0.5f)));
				//contactB = m_proxy->m_referenceMatrix.TransformVector (contactB.Scale (dgFloat32 (0.5f)));
				contactA = m_proxy->m_referenceCollision->m_globalMatrix.TransformVector (contactA.Scale (dgFloat32 (0.5f)));
				contactB = m_proxy->m_referenceCollision->m_globalMatrix.TransformVector (contactB.Scale (dgFloat32 (0.5f)));
				dir = m_proxy->m_referenceCollision->m_globalMatrix.RotateVector(dir);

				dgContactPoint* const contact = m_proxy->m_contacts;
				contact[0].m_point.m_x = contactA.m_x;
				contact[0].m_point.m_y = contactA.m_y;
				contact[0].m_point.m_z = contactA.m_z;
				contact[1].m_point.m_x = contactB.m_x;
				contact[1].m_point.m_y = contactB.m_y;
				contact[1].m_point.m_z = contactB.m_z;

				contact[0].m_normal.m_x = dir.m_x;
				contact[0].m_normal.m_y = dir.m_y;
				contact[0].m_normal.m_z = dir.m_z;

				contact[1].m_normal.m_x = -dir.m_x;
				contact[1].m_normal.m_y = -dir.m_y;
				contact[1].m_normal.m_z = -dir.m_z;
				return 1;
			}

			m_vertexIndex ++;
			switch (m_vertexIndex) 
			{
				case 1:
				{
					dgAssert (0);
					break;
				}

				case 2:
				{
					v = ReduceLine (origin);	
					break;

				}

				case 3:
				{
					v = ReduceTriangle (origin);
					break;
				}

				case 4:
				{
					v = ReduceTetrahedrum (origin);
					break;
				}
			}
		}
	}

	//dgAssert (i < CLOSE_DISTANCE_MAX_ITERATION);
	dgAssert (code != dgMinkError);
	return 0;
}


dgContactSolver::dgPerimenterEdge* dgContactSolver::ReduceContacts (dgPerimenterEdge* poly, dgInt32 maxCount)
{
	dgInt32 buffer [256];
	dgUpHeap<dgPerimenterEdge*, dgFloat32> heap (buffer, sizeof (buffer));	

	dgInt32 restart = 1;
	while (restart) {
		restart = 0;
		dgPerimenterEdge* ptr0 = poly; 
		poly = poly->m_next;
		if (poly->m_next != poly) {
			heap.Flush();
			dgPerimenterEdge* ptr = poly; 
			do {
				dgFloat32 dist2;
				dgVector error (*ptr->m_next->m_vertex - *ptr->m_vertex);
				dist2 = error % error;
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
			//dgFloat32 dist2;
			dgPerimenterEdge* ptr = heap[0];
			heap.Pop();
			for (dgInt32 i = 0; i < heap.GetCount(); i ++) {
				if (heap[i] == ptr->m_next) {
					heap.Remove(i);
					break;
				}
			}

			ptr->m_next = ptr->m_next->m_next;
			dgVector error (*ptr->m_next->m_vertex - *ptr->m_vertex);
			dgFloat32 dist2 = error % error;
			heap.Push(ptr, dist2);
		}
		poly = heap[0];
	}

	return poly;
}



dgInt32 dgContactSolver::CalculateConvexShapeIntersectionLine (
	const dgMatrix& matrix, const dgVector& shapeNormal, dgUnsigned32 id, dgFloat32 penetration,
	dgInt32 shape1VertexCount, dgVector* const shape1, 
	dgInt32 shape2VertexCount, dgVector* const shape2, dgContactPoint* const contactOut)
{
	dgInt32 count = 0;
//	dgVector* output = (dgVector*) &m_hullVertex[shape1VertexCount + shape2VertexCount + 1];
	dgVector pool[64];
	dgVector* output = &pool[0];

	dgAssert (shape1VertexCount >= 3);
	dgAssert (shape2VertexCount <= 2);

	dgVector* ptr = NULL;
	// face line intersection
	if (shape2VertexCount == 2) {
		ptr = (dgVector*)&shape2[0];
		dgInt32 i0 = shape1VertexCount - 1;
		for (dgInt32 i1 = 0; i1 < shape1VertexCount; i1 ++) {
			dgVector n (shapeNormal * (shape1[i1] - shape1[i0]));
			dgAssert ((n % n) > dgFloat32 (0.0f));
			dgPlane plane (n, - (n % shape1[i0]));

			dgFloat32 test0 = plane.Evalue (ptr[0]);
			dgFloat32 test1 = plane.Evalue (ptr[1]);
			if (test0 >= dgFloat32 (0.0f)) {
				if (test1 >= dgFloat32 (0.0f)) {
					output[count + 0] = ptr[0];
					output[count + 1] = ptr[1];
					count += 2;
				} else {
					dgVector dp (ptr[1] - ptr[0]);
					dgFloat32 den = plane % dp;
					if (dgAbsf(den) < 1.0e-10f) {
						den = 1.0e-10f;
					}
					output[count + 0] = ptr[0];
					output[count + 1] = ptr[0] - dp.Scale (test0 / den);
					count += 2;
				}
			} else if (test1 >= dgFloat32 (0.0f)) {
				dgVector dp (ptr[1] - ptr[0]);
				dgFloat32 den = plane % dp;
				if (dgAbsf(den) < 1.0e-10f) {
					den = 1.0e-10f;
				}
				output[count] = ptr[0] - dp.Scale (test0 / den);
				count ++;
				output[count] = ptr[1];
				count ++;
			} else {
				return 0;
			}


			shape2VertexCount = count;
			ptr = output;
			output = &output[count];
			count = 0;
			i0 = i1;
			dgAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
		}

	} else if (shape2VertexCount == 1){
		const dgVector& p = shape2[0]; 
		dgInt32 i0 = shape1VertexCount - 1;
		for (dgInt32 i1 = 0; i1 < shape1VertexCount; i1 ++) {
			dgVector n (shapeNormal * (shape1[i1] - shape1[i0]));
			dgAssert ((n % n) > dgFloat32 (0.0f));
			dgPlane plane (n, - (n % shape1[i0]));
			dgFloat32 test0 = plane.Evalue (p);
			if (test0 < dgFloat32 (-1.e-3f)) {
				return 0;
			}
			i0 = i1;
		}
		ptr = output;
		output[count] = p;
		count ++;

	} else {
		shape2VertexCount = 0;
	}

	dgVector normal = matrix.RotateVector(shapeNormal);
	for (dgInt32 i0 = 0; i0 < shape2VertexCount; i0 ++) {
		contactOut[i0].m_point = matrix.TransformVector(ptr[i0]);
		contactOut[i0].m_normal = normal;
		contactOut[i0].m_penetration = penetration;
		contactOut[i0].m_userId = id;
		contactOut[i0].m_isEdgeContact = 0;
	}

	return shape2VertexCount;
}


dgInt32 dgContactSolver::CalculateConvexShapeIntersection (
	const dgMatrix& matrix, const dgVector& shapeNormal, dgUnsigned32 id, dgFloat32 penetration,
	dgInt32 shape1VertexCount, dgVector* const shape1, dgInt32 shape2VertexCount, 
	dgVector* const shape2, dgContactPoint* const contactOut, dgInt32 maxContacts)
{
	dgInt32 count = 0;
	if (shape2VertexCount <= 2) {
		count = CalculateConvexShapeIntersectionLine (matrix, shapeNormal, id, penetration, shape1VertexCount, shape1, shape2VertexCount, shape2, contactOut);
		if (count > maxContacts) {
			count = maxContacts;
		}
	} else if (shape1VertexCount <= 2) {
		count = CalculateConvexShapeIntersectionLine (matrix, shapeNormal, id, penetration, shape2VertexCount, shape2, shape1VertexCount, shape1, contactOut);
		if (count > maxContacts) {
			count = maxContacts;
		}
	} else {
		dgPerimenterEdge *edgeClipped[2];
		dgPerimenterEdge subdivision[128];

		dgAssert (shape1VertexCount >= 3);
		dgAssert (shape2VertexCount >= 3);
		dgVector pool[64];
		dgVector* output = &pool[0];

		dgAssert ((shape1VertexCount + shape2VertexCount) < dgInt32 (sizeof (subdivision) / (2 * sizeof (subdivision[0]))));
		for (dgInt32 i0 = 0; i0 < shape2VertexCount; i0 ++) {
			subdivision[i0].m_vertex = &shape2[i0];
			subdivision[i0].m_prev = &subdivision[i0 - 1];
			subdivision[i0].m_next = &subdivision[i0 + 1];
		}
		subdivision[0].m_prev = &subdivision[shape2VertexCount - 1];
		subdivision[shape2VertexCount - 1].m_next = &subdivision[0];

		dgPerimenterEdge* poly = &subdivision[0];
		edgeClipped[0] = NULL;
		edgeClipped[1] = NULL;
		dgInt32 i0 = shape1VertexCount - 1;
		dgInt32 edgeIndex = shape2VertexCount;
		for (dgInt32 i1 = 0; i1 < shape1VertexCount; i1 ++) {
			dgVector n (shapeNormal * (shape1[i1] - shape1[i0]));
			dgPlane plane (n, - (n % shape1[i0]));
			i0 = i1;

			count = 0;
			dgPerimenterEdge* tmp = poly;
			dgInt32 isInside = 0;
			dgFloat32 test0 = plane.Evalue (*tmp->m_vertex);
			do {
				dgFloat32 test1 = plane.Evalue (*tmp->m_next->m_vertex);
				if (test0 >= dgFloat32 (0.0f)) {
					isInside |= 1;
					if (test1 < dgFloat32 (0.0f)) {
						const dgVector& p0 = *tmp->m_vertex;
						const dgVector& p1 = *tmp->m_next->m_vertex;
						dgVector dp (p1 - p0); 
						dgFloat32 den = plane % dp;
						if (dgAbsf(den) < dgFloat32 (1.0e-24f)) {
							den = (den >= dgFloat32 (0.0f)) ?  dgFloat32 (1.0e-24f) :  dgFloat32 (-1.0e-24f);
						}

						den = test0 / den;
						if (den >= dgFloat32 (0.0f)) {
							den = dgFloat32 (0.0f);
						} else if (den <= -1.0f) {
							den = dgFloat32 (-1.0f);
						}
						output[0] = p0 - dp.Scale (den);
						edgeClipped[0] = tmp;
						count ++;
					}
				} else if (test1 >= dgFloat32 (0.0f)) {
					const dgVector& p0 = *tmp->m_vertex;
					const dgVector& p1 = *tmp->m_next->m_vertex;
					isInside |= 1;
					dgVector dp (p1 - p0); 
					dgFloat32 den = plane % dp;
					if (dgAbsf(den) < dgFloat32 (1.0e-24f)) {
						den = (den >= dgFloat32 (0.0f)) ?  dgFloat32 (1.0e-24f) :  dgFloat32 (-1.0e-24f);
					}
					den = test0 / den;
					if (den >= dgFloat32 (0.0f)) {
						den = dgFloat32 (0.0f);
					} else if (den <= -1.0f) {
						den = dgFloat32 (-1.0f);
					}
					output[1] = p0 - dp.Scale (den);
					edgeClipped[1] = tmp;
					count ++;
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
				edgeIndex ++;
				dgAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
				dgAssert (edgeIndex < dgInt32 (sizeof (subdivision) / sizeof (subdivision[0])));
			}
		}

		dgAssert (poly);
		poly = ReduceContacts (poly, maxContacts);

		count = 0;
		dgPerimenterEdge* intersection = poly;
		dgVector normal = matrix.RotateVector(shapeNormal);
		do {
			contactOut[count].m_point = matrix.TransformVector(*intersection->m_vertex);
			contactOut[count].m_normal = normal;
			contactOut[count].m_penetration = penetration;
			contactOut[count].m_userId = id;
			contactOut[count].m_isEdgeContact = 0;
			count ++;
			intersection = intersection->m_next;
		} while (intersection != poly);

	}

	return count;
}



dgInt32 dgContactSolver::CalculateContactAlternateMethod(dgMinkFace* const face, dgInt32 contacID, dgContactPoint* const contactOut, dgInt32 maxContacts)
{
	dgInt32 count = 0;
	// Get the contact form the last face
	const dgPlane& plane = *face;
	dgFloat32 penetration = plane.m_w - m_penetrationPadding;
	dgFloat32 dist = (plane % m_averVertex[face->m_vertex[0]]) * dgFloat32 (0.5f);
	const dgPlane clipPlane (plane.Scale(dgFloat32 (-1.0f)), dist);

	dgVector point1 (clipPlane.Scale (-clipPlane.m_w));
	dgVector* const shape1 = m_hullVertex;

	dgVector p1 (m_referenceCollision->SupportVertex (clipPlane.Scale (dgFloat32 (-1.0f))));
	p1 += clipPlane.Scale (DG_ROBUST_PLANE_CLIP);
	dgInt32 count1 = m_referenceCollision->CalculatePlaneIntersection (clipPlane, p1, shape1);
	dgVector err (clipPlane.Scale (clipPlane % (point1 - p1)));
	for (dgInt32 i = 0; i < count1; i ++) {
		shape1[i] += err;
	}

	//		dgAssert (penetration <= dgFloat32 (2.0e-1f));
	dist = dgMax (- (penetration + DG_IMPULSIVE_CONTACT_PENETRATION), dgFloat32 (0.0f));
	if (count1) {

		dgVector* const shape2 = &m_hullVertex[count1];
		const dgPlane clipPlane2 (m_matrix.UntransformPlane(clipPlane));

		dgVector point2 (clipPlane2.Scale (-clipPlane2.m_w));
		dgVector p2 (m_floatingcollision->SupportVertex (clipPlane2.Scale(dgFloat32 (-1.0f))));
		p2 += clipPlane2.Scale (DG_ROBUST_PLANE_CLIP);
		dgInt32 count2 = m_floatingcollision->CalculatePlaneIntersection (clipPlane2, p2, shape2);
		dgVector err (clipPlane2.Scale (clipPlane2 % (point2 - p2)));
		for (dgInt32 i = 0; i < count2; i ++) {
			shape2[i] += err;
		}

		if (count2) {
			dgAssert (count1);
			dgAssert (count2);

			//const dgMatrix& matrix1 = m_proxy->m_referenceMatrix;
			const dgMatrix& matrix1 = m_proxy->m_referenceCollision->m_globalMatrix;
			if (count1 == 1) {
				count = 1;
				contactOut[0].m_point = matrix1.TransformVector (shape1[0]);
				contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
				contactOut[0].m_userId = contacID;
				contactOut[0].m_penetration = dist;
			} else if (count2 == 1) {

				count = 1;
				contactOut[0].m_point = matrix1.TransformVector (m_matrix.TransformVector(shape2[0]));
				contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
				contactOut[0].m_userId = contacID;
				contactOut[0].m_penetration = dist;

			} else if ((count1 == 2) && (count2 == 2)) {
				dgVector p0 (shape1[0]); 
				dgVector p1 (shape1[1]); 
				dgVector q0 (m_matrix.TransformVector(shape2[0])); 
				dgVector q1 (m_matrix.TransformVector(shape2[1])); 
				dgVector p10 (p1 - p0);
				dgVector q10 (q1 - q0);
				p10 = p10.Scale (dgRsqrt (p10 % p10 + dgFloat32 (1.0e-8f)));
				q10 = q10.Scale (dgRsqrt (q10 % q10 + dgFloat32 (1.0e-8f)));
				dgFloat32 dot = q10 % p10;
				if (dgAbsf (dot) > dgFloat32 (0.998f)) {
					dgFloat32 pl0 = p0 % p10;
					dgFloat32 pl1 = p1 % p10;
					dgFloat32 ql0 = q0 % p10;
					dgFloat32 ql1 = q1 % p10;
					if (pl0 > pl1) {
						dgSwap (pl0, pl1);
						dgSwap (p0, p1);
						p10 = p10.Scale (dgFloat32 (-1.0f));
					}
					if (ql0 > ql1) {
						dgSwap (ql0, ql1);
					}
					if ( !((ql0 > pl1) && (ql1 < pl0))) {
						dgFloat32 clip0 = (ql0 > pl0) ? ql0 : pl0;
						dgFloat32 clip1 = (ql1 < pl1) ? ql1 : pl1;

						count = 2;
						contactOut[0].m_point = p0 + p10.Scale (clip0 - pl0);
						contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
						contactOut[0].m_userId = contacID;
						contactOut[0].m_penetration = dist;

						contactOut[1].m_point = p0 + p10.Scale (clip1 - pl0);
						contactOut[1].m_normal = matrix1.RotateVector (clipPlane);
						contactOut[1].m_userId = contacID;
						contactOut[1].m_penetration = dist;
					}

				} else {
					dgVector c0;
					dgVector c1;
					count = 1;
					dgRayToRayDistance (p0, p1, q0, q1, c0, c1);
					contactOut[0].m_point = (c0 + c1).Scale (dgFloat32 (0.5f));
					contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
					contactOut[0].m_userId = contacID;
					contactOut[0].m_penetration = dist;
				}
				for (dgInt32 i = 0; i < count; i ++) {
					contactOut[i].m_point = matrix1.TransformVector(contactOut[i].m_point);
				}

			} else {
				//const dgMatrix& matrix1 = m_proxy->m_referenceMatrix;
				m_matrix.TransformTriplex(&shape2[0].m_x, sizeof (dgVector), &shape2[0].m_x, sizeof (dgVector), count2);
				count = CalculateConvexShapeIntersection (matrix1, clipPlane, dgUnsigned32 (contacID), dist, count1, shape1, count2, shape2, contactOut, maxContacts);
			}
		}
	}

	return count;
}



dgInt32 dgContactSolver::CalculateContacts(dgMinkFace* const face, dgInt32 contacID, dgContactPoint* const contactOut, dgInt32 maxContacts)
{
	dgInt32 count = 0;
	// Get the contact form the last face
	const dgPlane& plane = *face;
	dgFloat32 penetration = plane.m_w - m_penetrationPadding;
	dgFloat32 dist = (plane % m_averVertex[face->m_vertex[0]]) * dgFloat32 (0.5f);
	const dgPlane clipPlane (plane.Scale(dgFloat32 (-1.0f)), dist);

	dgVector point1 (clipPlane.Scale (-clipPlane.m_w));
	dgVector* const shape1 = m_hullVertex;
	dgInt32 count1 = m_referenceCollision->CalculatePlaneIntersection (clipPlane, point1, shape1);
	if (!count1) {
		dgVector p1 (m_referenceCollision->SupportVertex (clipPlane.Scale (dgFloat32 (-1.0f))));
		p1 += clipPlane.Scale (DG_ROBUST_PLANE_CLIP);
		count1 = m_referenceCollision->CalculatePlaneIntersection (clipPlane, p1, shape1);
		dgVector err (clipPlane.Scale (clipPlane % (point1 - p1)));
		for (dgInt32 i = 0; i < count1; i ++) {
			shape1[i] += err;
		}
	}

	dgAssert (penetration <= dgFloat32 (2.0e-1f));
	dist = dgMax (- (penetration + DG_IMPULSIVE_CONTACT_PENETRATION), dgFloat32 (0.0f));
	if (count1) {
		dgVector* const shape2 = &m_hullVertex[count1];
		const dgPlane clipPlane2 (m_matrix.UntransformPlane(clipPlane));

		dgVector point2 (clipPlane2.Scale (-clipPlane2.m_w));
		dgInt32 count2 = m_floatingcollision->CalculatePlaneIntersection (clipPlane2, point2, shape2);
		if (!count2) {
			dgVector p2 (m_floatingcollision->SupportVertex (clipPlane2.Scale(dgFloat32 (-1.0f))));
			p2 += clipPlane2.Scale (DG_ROBUST_PLANE_CLIP);
			count2 = m_floatingcollision->CalculatePlaneIntersection (clipPlane2, p2, shape2);
			dgVector err (clipPlane2.Scale (clipPlane2 % (point2 - p2)));
			for (dgInt32 i = 0; i < count2; i ++) {
				shape2[i] += err;
			}
		}

		if (count2) {
			dgAssert (count1);
			dgAssert (count2);

			//const dgMatrix& matrix1 = m_proxy->m_referenceMatrix;
			const dgMatrix& matrix1 = m_proxy->m_referenceCollision->m_globalMatrix;
			if (count1 == 1) {

				count = 1;
				contactOut[0].m_point = matrix1.TransformVector (shape1[0]);
				contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
				contactOut[0].m_userId = contacID;
				contactOut[0].m_penetration = dist;
			} else if (count2 == 1) {
			
				count = 1;
				contactOut[0].m_point =  matrix1.TransformVector (m_matrix.TransformVector(shape2[0]));
				contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
				contactOut[0].m_userId = contacID;
				contactOut[0].m_penetration = dist;

			} else if ((count1 == 2) && (count2 == 2)) {
				//const dgMatrix& matrix1 = m_proxy->m_referenceMatrix;
				dgVector p0 (shape1[0]); 
				dgVector p1 (shape1[1]); 
				dgVector q0 (m_matrix.TransformVector(shape2[0])); 
				dgVector q1 (m_matrix.TransformVector(shape2[1])); 
				dgVector p10 (p1 - p0);
				dgVector q10 (q1 - q0);
				p10 = p10.Scale (dgRsqrt (p10 % p10 + dgFloat32 (1.0e-8f)));
				q10 = q10.Scale (dgRsqrt (q10 % q10 + dgFloat32 (1.0e-8f)));
				dgFloat32 dot = q10 % p10;
				if (dgAbsf (dot) > dgFloat32 (0.998f)) {
					dgFloat32 pl0 = p0 % p10;
					dgFloat32 pl1 = p1 % p10;
					dgFloat32 ql0 = q0 % p10;
					dgFloat32 ql1 = q1 % p10;
					if (pl0 > pl1) {
						dgSwap (pl0, pl1);
						dgSwap (p0, p1);
						p10 = p10.Scale (dgFloat32 (-1.0f));
					}
					if (ql0 > ql1) {
						dgSwap (ql0, ql1);
					}
					if ( !((ql0 > pl1) && (ql1 < pl0))) {
						dgFloat32 clip0 = (ql0 > pl0) ? ql0 : pl0;
						dgFloat32 clip1 = (ql1 < pl1) ? ql1 : pl1;

						count = 2;
						contactOut[0].m_point = p0 + p10.Scale (clip0 - pl0);
						contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
						contactOut[0].m_userId = contacID;
						contactOut[0].m_penetration = dist;

						contactOut[1].m_point = p0 + p10.Scale (clip1 - pl0);
						contactOut[1].m_normal = matrix1.RotateVector (clipPlane);
						contactOut[1].m_userId = contacID;
						contactOut[1].m_penetration = dist;
					}

				} else {
					dgVector c0;
					dgVector c1;
					count = 1;
					dgRayToRayDistance (p0, p1, q0, q1, c0, c1);
					contactOut[0].m_point = (c0 + c1).Scale (dgFloat32 (0.5f));
					contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
					contactOut[0].m_userId = contacID;
					contactOut[0].m_penetration = dist;
				}
				if (count > maxContacts) {
					count = maxContacts;
				}
				for (dgInt32 i = 0; i < count; i ++) {
					contactOut[i].m_point = matrix1.TransformVector(contactOut[i].m_point);
				}

			} else {
				//const dgMatrix& matrix1 = m_proxy->m_referenceMatrix;
				m_matrix.TransformTriplex(&shape2[0].m_x, sizeof (dgVector), &shape2[0].m_x, sizeof (dgVector), count2);
				count = CalculateConvexShapeIntersection (matrix1, clipPlane, dgUnsigned32 (contacID), dist, count1, shape1, count2, shape2, contactOut, maxContacts);
				if (!count) {
					count = CalculateContactAlternateMethod(face, contacID, contactOut, maxContacts);
				}
			}

			dgInt32 edgeContactFlag = (m_floatingcollision->IsEdgeIntersection() | m_referenceCollision->IsEdgeIntersection()) ? 1 : 0;
			for (dgInt32 i = 0; i < count; i ++) {
				contactOut[i].m_isEdgeContact = edgeContactFlag;
			}
		}
	}
	return count;
}


dgContactSolver::dgMinkFace* dgContactSolver::CalculateClipPlaneLarge ()
{
	dgFloat64 cyclingMem[4];
	dgMinkFace* stackPool[128];
	dgMinkFace* deadFaces[128];
	SilhouetteFaceCap sillueteCap[128];
	dgBigVector diff[3];
	dgBigVector aver[3];
	dgInt8 buffer[DG_HEAP_EDGE_COUNT * (sizeof (dgFloat32) + sizeof (dgMinkFace *))];
	dgClosestFace heapSort (buffer, sizeof (buffer));

	m_planeIndex = 4;
	dgMinkFace* closestFace = NULL;
	m_freeFace = NULL;

	dgAssert (m_vertexIndex == 4);
	for (dgInt32 i = 0; i < 4; i ++) {
		dgMinkFace* face = &m_simplex[i];
		face->m_inHeap = 0;
		face->m_isActive = 1;
		if (CalcFacePlaneLarge (face)) {
			face->m_inHeap = 1;
			heapSort.Push(face, face->m_w);
		}
	}

	dgInt32 cycling = 0;
	cyclingMem[0] = dgFloat32 (1.0e10f);
	cyclingMem[1] = dgFloat32 (1.0e10f);
	cyclingMem[2] = dgFloat32 (1.0e10f);
	cyclingMem[3] = dgFloat32 (1.0e10f);

	dgFloat64 minValue = dgFloat32 ( 1.0e10f);
	dgPlane bestPlane (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f),dgFloat32 (0.0f));
	diff[0] = dgBigVector (dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f),dgFloat64 (0.0f));
	diff[1] = diff[0];
	diff[2] = diff[0];
	aver[0] = diff[0];
	aver[1] = diff[0];
	aver[2] = diff[0];

	dgMinkFace* face = NULL;
	while (heapSort.GetCount()) {
		face = heapSort[0];
		face->m_inHeap = 0;
		heapSort.Pop();

		if (face->m_isActive) {

			const dgPlane& plane = *face;

			CalcSupportVertexLarge (plane, m_vertexIndex);
			dgVector p (dgFloat32 (m_hullVertexLarge[m_vertexIndex].m_x), dgFloat32 (m_hullVertexLarge[m_vertexIndex].m_y), dgFloat32 (m_hullVertexLarge[m_vertexIndex].m_z), dgFloat64(0.0f));
			dgFloat64 dist = plane.Evalue (p);
			m_vertexIndex ++;

			if (m_vertexIndex > 16) {
				if (dist < minValue) {
					if (dist >= dgFloat64 (0.0f)) {
						minValue = dist;
						bestPlane = plane;

						diff[0] = m_hullVertexLarge[face->m_vertex[0]];
						aver[0] = m_averVertexLarge[face->m_vertex[0]];

						diff[1] = m_hullVertexLarge[face->m_vertex[1]];
						aver[1] = m_averVertexLarge[face->m_vertex[1]];

						diff[2] = m_hullVertexLarge[face->m_vertex[2]];
						aver[2] = m_averVertexLarge[face->m_vertex[2]];
					}
				}
			}

			cyclingMem[cycling] = dist;
			cycling = (cycling + 1) & 3;
			dgInt32 cyclingIndex = 0;
			for (; cyclingIndex < 4; cyclingIndex ++) {
				if (fabs (dist - cyclingMem[cyclingIndex]) > dgFloat64 (1.0e-6f)) {
					break;
				}
			}
			if (cyclingIndex == 4) {
				dist = dgFloat64 (0.0f);
			}


			if ((m_vertexIndex > DG_MINK_MAX_POINTS) ||
				(m_planeIndex > DG_MINK_MAX_FACES) ||
				(heapSort.GetCount() > (DG_HEAP_EDGE_COUNT - 24))) {
					dgPlane& plane = *face;
					plane = bestPlane;

					dgInt32 index = face->m_vertex[0];
					face->m_vertex[0] = 0;
					m_hullVertexLarge[index] = diff[0];
					m_averVertexLarge[index] = aver[0];

					index = face->m_vertex[1];
					face->m_vertex[1] = 1;
					m_hullVertexLarge[index] = diff[1];
					m_averVertexLarge[index] = aver[1];

					index = face->m_vertex[2];
					face->m_vertex[2] = 2;
					m_hullVertexLarge[index] = diff[2];
					m_averVertexLarge[index] = aver[2];
					dist = dgFloat32 (0.0f);
			}


			if (dist < (dgFloat32 (DG_IMPULSIVE_CONTACT_PENETRATION) / dgFloat32 (16.0f))) {
				dgAssert (m_planeIndex <= DG_MINK_MAX_FACES_SIZE);
				dgAssert (heapSort.GetCount() <= DG_HEAP_EDGE_COUNT);
				closestFace = face;
				break;
			} else if (dist > dgFloat32 (0.0f)) {
				dgAssert (face->m_inHeap == 0);

				dgInt32 stack = 0;
				dgInt32 deadCount = 1;
				dgMinkFace* silhouette = NULL;
				deadFaces[0] = face;
				closestFace = face;
				face->m_isActive = 0;
				for (dgInt32 i = 0; i < 3; i ++) {
					dgMinkFace* const adjacent = &m_simplex[face->m_adjancentFace[i]];
					dgAssert (adjacent->m_isActive);
					dist = adjacent->Evalue (p);  
					if (dist > dgFloat64 (0.0f)) { 
						adjacent->m_isActive = 0;
						stackPool[stack] = adjacent;
						deadFaces[deadCount] = adjacent;
						stack ++;
						deadCount ++;
					} else {
						silhouette = adjacent;
					}
				}

				while (stack) {
					stack --;
					face = stackPool[stack];
					for (dgInt32 i = 0; i < 3; i ++) {
						dgMinkFace* const adjacent = &m_simplex[face->m_adjancentFace[i]];
						if (adjacent->m_isActive){
							dist = adjacent->Evalue (p);  
							if (dist > dgFloat64 (0.0f)) { 
								adjacent->m_isActive = 0;
								stackPool[stack] = adjacent;
								deadFaces[deadCount] = adjacent;
								stack ++;
								deadCount ++;
								dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
								dgAssert (deadCount < dgInt32 (sizeof (deadFaces) / sizeof (deadFaces[0])));

							} else {
								silhouette = adjacent;
							}
						}
					}
				}

				if (!silhouette) {
					closestFace = face;
					break;
				}
				// build silhouette						
				dgAssert (silhouette);
				dgAssert (silhouette->m_isActive);

				dgInt32 i2 = (m_vertexIndex - 1);
				dgMinkFace* const lastSilhouette = silhouette;
				dgAssert ((silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[1]) && (silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[2]) && (silhouette->m_adjancentFace[1] != silhouette->m_adjancentFace[2]));

				dgInt32 adjacentIndex = DG_GETADJACENTINDEX_ACTIVE (silhouette);
				face = NewFace();
				dgInt32 i0 = silhouette->m_vertex[adjacentIndex];
				dgInt32 i1 = silhouette->m_vertex[adjacentIndex + 1];

				face->m_vertex[0] = dgInt16 (i1);
				face->m_vertex[1] = dgInt16 (i0);
				face->m_vertex[2] = dgInt16 (i2);
				face->m_vertex[3] = face->m_vertex[0];
				face->m_adjancentFace[0] = dgInt16 (silhouette - m_simplex);
				face->m_inHeap = 0; 
				face->m_isActive = 1; 

				sillueteCap[0].m_face = face;
				sillueteCap[0].m_faceCopling = &silhouette->m_adjancentFace[adjacentIndex];
				dgInt32 silhouetteCapCount = 1;
				dgAssert (silhouetteCapCount < dgInt32 (sizeof (sillueteCap) / sizeof (sillueteCap[0])));
				do {
					silhouette = &m_simplex[silhouette->m_adjancentFace[adjacentIndex]];
					adjacentIndex = (DG_GETADJACENTINDEX_VERTEX(silhouette, i0)); 
				} while (!silhouette->m_isActive);

				dgMinkFace* prevFace = face;
				dgMinkFace* const firstFace = face;
				dgInt32 lastSilhouetteVertex = i0;
				dgInt32 prevEdgeIndex = dgInt32 (face - m_simplex);
				do {
					dgAssert ((silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[1]) && (silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[2]) && (silhouette->m_adjancentFace[1] != silhouette->m_adjancentFace[2]));

					adjacentIndex = adjacentIndex ? adjacentIndex - 1 : 2;

					face = NewFace();
					i0 = silhouette->m_vertex[adjacentIndex];
					i1 = silhouette->m_vertex[adjacentIndex + 1];

					face->m_vertex[0] = dgInt16 (i1);
					face->m_vertex[1] = dgInt16 (i0);
					face->m_vertex[2] = dgInt16 (i2);
					face->m_vertex[3] = face->m_vertex[0];
					face->m_adjancentFace[0] = dgInt16 (silhouette - m_simplex);
					face->m_adjancentFace[2] = dgInt16 (prevEdgeIndex);
					face->m_inHeap = 0; 
					face->m_isActive = 1; 

					prevEdgeIndex = dgInt32 (face - m_simplex);
					prevFace->m_adjancentFace[1] = dgInt16 (prevEdgeIndex);
					prevFace = face;

					sillueteCap[silhouetteCapCount].m_face = face;
					sillueteCap[silhouetteCapCount].m_faceCopling = &silhouette->m_adjancentFace[adjacentIndex];
					silhouetteCapCount ++;
					dgAssert (silhouetteCapCount < dgInt32 (sizeof (sillueteCap) / sizeof (sillueteCap[0])));

					do {
						silhouette = &m_simplex[silhouette->m_adjancentFace[adjacentIndex]];
						adjacentIndex = (DG_GETADJACENTINDEX_VERTEX(silhouette, i0)); 
					} while (!silhouette->m_isActive);

				} while ((silhouette != lastSilhouette) || (silhouette->m_vertex[adjacentIndex ? adjacentIndex - 1 : 2] != lastSilhouetteVertex));
				firstFace->m_adjancentFace[2] = dgInt16 (prevEdgeIndex);
				prevFace->m_adjancentFace[1] = dgInt16 (firstFace - m_simplex);


				for (dgInt32 i = 0; i < deadCount; i ++) {
					if (!deadFaces[i]->m_inHeap){
						dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) deadFaces[i];
						nextFreeFace->m_next = m_freeFace;
						m_freeFace = nextFreeFace;
					}
				}

				while (heapSort.GetCount() && (!heapSort[0]->m_isActive)) {
					face = heapSort[0];
					heapSort.Pop();
					dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) face;
					nextFreeFace->m_next = m_freeFace;
					m_freeFace = nextFreeFace;
				}

				for (dgInt32 i = 0; i < silhouetteCapCount; i ++) {
					face = sillueteCap[i].m_face;
					*sillueteCap[i].m_faceCopling = dgInt16 (face - m_simplex);

					if (CalcFacePlaneLarge (face)) {
						face->m_inHeap = 1;
						heapSort.Push(face, face->m_w);
					}
				}
			}

		} else {
			dgAssert (0);
			dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) face;
			nextFreeFace->m_next = m_freeFace;
			m_freeFace = nextFreeFace;
		}
	}

	dgAssert (face);
	dgInt32 i = face->m_vertex[0];
	//m_hullVertex[i] = dgVector (dgFloat32 (m_hullVertexLarge[i].m_x), dgFloat32 (m_hullVertexLarge[i].m_y), dgFloat32 (m_hullVertexLarge[i].m_z), dgFloat32 (0.0f));
	m_averVertex[i] = dgVector (dgFloat32 (m_averVertexLarge[i].m_x), dgFloat32 (m_averVertexLarge[i].m_y), dgFloat32 (m_averVertexLarge[i].m_z), dgFloat32 (0.0f));;
	return closestFace;
}



dgContactSolver::dgMinkFace* dgContactSolver::CalculateClipPlane ()
{
	dgFloat32 cyclingMem[4];
	dgMinkFace* stackPool[128];
	dgMinkFace* deadFaces[128];
	SilhouetteFaceCap sillueteCap[128];
	dgVector diff[3];
	dgVector aver[3];
	dgInt8  buffer[DG_HEAP_EDGE_COUNT * (sizeof (dgFloat32) + sizeof (dgMinkFace *))];
	dgClosestFace heapSort (buffer, sizeof (buffer));

	m_planeIndex = 4;
	dgMinkFace* closestFace = NULL;
	m_freeFace = NULL;

	dgAssert (m_vertexIndex == 4);
	for (dgInt32 i = 0; i < 4; i ++) {
		dgMinkFace* face = &m_simplex[i];
		face->m_inHeap = 0;
		face->m_isActive = 1;
		if (CalcFacePlane (face)) {
			face->m_inHeap = 1;
			heapSort.Push(face, face->m_w);
		}
	}

	dgInt32 cycling = 0;
	cyclingMem[0] = dgFloat32 (1.0e10f);
	cyclingMem[1] = dgFloat32 (1.0e10f);
	cyclingMem[2] = dgFloat32 (1.0e10f);
	cyclingMem[3] = dgFloat32 (1.0e10f);

	dgFloat32 minValue = dgFloat32 ( 1.0e10f);
	dgPlane bestPlane (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f),dgFloat32 (0.0f));
	diff[0] = bestPlane;
	diff[1] = bestPlane;
	diff[2] = bestPlane;
	aver[0] = bestPlane;
	aver[1] = bestPlane;
	aver[2] = bestPlane;

	while (heapSort.GetCount()) {
		dgMinkFace* face = heapSort[0];
		face->m_inHeap = 0;
		heapSort.Pop();

		if (face->m_isActive) {
			const dgPlane& plane = *face;

			CalcSupportVertex (plane, m_vertexIndex);
			const dgVector& p = m_hullVertex[m_vertexIndex];
			dgFloat32 dist = plane.Evalue (p);
			m_vertexIndex ++;

			if (m_vertexIndex > 16) {
				if (dist < minValue) {
					if (dist >= dgFloat32 (0.0f)) {
						minValue = dist;
						bestPlane = plane;

						diff[0] = m_hullVertex[face->m_vertex[0]];
						aver[0] = m_averVertex[face->m_vertex[0]];

						diff[1] = m_hullVertex[face->m_vertex[1]];
						aver[1] = m_averVertex[face->m_vertex[1]];

						diff[2] = m_hullVertex[face->m_vertex[2]];
						aver[2] = m_averVertex[face->m_vertex[2]];
					}
				}
			}

			cyclingMem[cycling] = dist;
			cycling = (cycling + 1) & 3;
			dgInt32 cyclingEntry = 0;
			for (; cyclingEntry < 4; cyclingEntry ++) {
				if (dgAbsf (dist - cyclingMem[cyclingEntry]) > dgFloat32 (1.0e-6f)) {
					break;
				}
			}
			if (cyclingEntry == 4) {
				dist = dgFloat32 (0.0f);
			}


			if ((m_vertexIndex > DG_MINK_MAX_POINTS) || (m_planeIndex > DG_MINK_MAX_FACES) || (heapSort.GetCount() > (DG_HEAP_EDGE_COUNT - 24))) {
				dgPlane& plane = *face;
				plane = bestPlane;

				dgInt32 index = face->m_vertex[0];
				face->m_vertex[0] = 0;
				m_hullVertex[index] = diff[0];
				m_averVertex[index] = aver[0];

				index = face->m_vertex[1];
				face->m_vertex[1] = 1;
				m_hullVertex[index] = diff[1];
				m_averVertex[index] = aver[1];

				index = face->m_vertex[2];
				face->m_vertex[2] = 2;
				m_hullVertex[index] = diff[2];
				m_averVertex[index] = aver[2];
				dist = dgFloat32 (0.0f);
			}

			if (dist < (dgFloat32 (DG_IMPULSIVE_CONTACT_PENETRATION) / dgFloat32 (16.0f))) {
				dgAssert (m_planeIndex <= DG_MINK_MAX_FACES_SIZE);
				dgAssert (heapSort.GetCount() <= DG_HEAP_EDGE_COUNT);
				closestFace = face;
				break;
			} else if (dist > dgFloat32 (0.0f)) {
				dgAssert (face->m_inHeap == 0);

				dgInt32 stack = 0;
				dgInt32 deadCount = 1;
				dgMinkFace* silhouette = NULL;
				deadFaces[0] = face;
				closestFace = face;
				face->m_isActive = 0;
				for (dgInt32 i = 0; i < 3; i ++) {
					dgMinkFace* const adjacent = &m_simplex[face->m_adjancentFace[i]];
					dgAssert (adjacent->m_isActive);
					dist = adjacent->Evalue (p);  
					if (dist > dgFloat32 (0.0f)) { 
						adjacent->m_isActive = 0;
						stackPool[stack] = adjacent;
						deadFaces[deadCount] = adjacent;
						stack ++;
						deadCount ++;
					} else {
						silhouette = adjacent;
					}
				}
				while (stack) {
					stack --;
					face = stackPool[stack];
					for (dgInt32 i = 0; i < 3; i ++) {
						dgMinkFace* const adjacent = &m_simplex[face->m_adjancentFace[i]];
						if (adjacent->m_isActive){
							dist = adjacent->Evalue (p);  
							if (dist > dgFloat32 (0.0f)) { 
								adjacent->m_isActive = 0;
								stackPool[stack] = adjacent;
								deadFaces[deadCount] = adjacent;
								stack ++;
								deadCount ++;
								dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
								dgAssert (deadCount < dgInt32 (sizeof (deadFaces) / sizeof (deadFaces[0])));

							} else {
								silhouette = adjacent;
							}
						}
					}
				}

				if (!silhouette) {
					closestFace = face;
					break;
				}
				// build silhouette						
				dgAssert (silhouette);
				dgAssert (silhouette->m_isActive);

				dgInt32 i2 = (m_vertexIndex - 1);
				dgMinkFace* const lastSilhouette = silhouette;
				dgAssert ((silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[1]) &&
					(silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[2]) &&
					(silhouette->m_adjancentFace[1] != silhouette->m_adjancentFace[2]));


				dgInt32 adjacentIndex = DG_GETADJACENTINDEX_ACTIVE (silhouette);
				face = NewFace();
				dgInt32 i0 = silhouette->m_vertex[adjacentIndex];
				dgInt32 i1 = silhouette->m_vertex[adjacentIndex + 1];

				face->m_vertex[0] = dgInt16 (i1);
				face->m_vertex[1] = dgInt16 (i0);
				face->m_vertex[2] = dgInt16 (i2);
				face->m_vertex[3] = face->m_vertex[0];
				face->m_adjancentFace[0] = dgInt16 (silhouette - m_simplex);
				face->m_inHeap = 0; 
				face->m_isActive = 1; 

				sillueteCap[0].m_face = face;
				sillueteCap[0].m_faceCopling = &silhouette->m_adjancentFace[adjacentIndex];
				dgInt32 silhouetteCapCount = 1;
				dgAssert (silhouetteCapCount < dgInt32 (sizeof (sillueteCap) / sizeof (sillueteCap[0])));
				do {
					silhouette = &m_simplex[silhouette->m_adjancentFace[adjacentIndex]];
					adjacentIndex = (DG_GETADJACENTINDEX_VERTEX(silhouette, i0)); 
				} while (!silhouette->m_isActive);

				dgMinkFace* prevFace = face;
				dgMinkFace* const firstFace = face;
				dgInt32 lastSilhouetteVertex = i0;
				dgInt32 prevEdgeIndex = dgInt32 (face - m_simplex);
				do {
					dgAssert ((silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[1]) &&
						(silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[2]) &&
						(silhouette->m_adjancentFace[1] != silhouette->m_adjancentFace[2]));


					adjacentIndex = adjacentIndex ? adjacentIndex - 1 : 2;

					face = NewFace();
					i0 = silhouette->m_vertex[adjacentIndex];
					i1 = silhouette->m_vertex[adjacentIndex + 1];

					face->m_vertex[0] = dgInt16 (i1);
					face->m_vertex[1] = dgInt16 (i0);
					face->m_vertex[2] = dgInt16 (i2);
					face->m_vertex[3] = face->m_vertex[0];
					face->m_adjancentFace[0] = dgInt16 (silhouette - m_simplex);
					face->m_adjancentFace[2] = dgInt16 (prevEdgeIndex);
					face->m_inHeap = 0; 
					face->m_isActive = 1; 

					prevEdgeIndex = dgInt32 (face - m_simplex);
					prevFace->m_adjancentFace[1] = dgInt16 (prevEdgeIndex);
					prevFace = face;

					sillueteCap[silhouetteCapCount].m_face = face;
					sillueteCap[silhouetteCapCount].m_faceCopling = &silhouette->m_adjancentFace[adjacentIndex];
					silhouetteCapCount ++;
					dgAssert (silhouetteCapCount < dgInt32 (sizeof (sillueteCap) / sizeof (sillueteCap[0])));

					do {
						silhouette = &m_simplex[silhouette->m_adjancentFace[adjacentIndex]];
						adjacentIndex = (DG_GETADJACENTINDEX_VERTEX(silhouette, i0)); 
					} while (!silhouette->m_isActive);

				} while ((silhouette != lastSilhouette) || (silhouette->m_vertex[adjacentIndex ? adjacentIndex - 1 : 2] != lastSilhouetteVertex));
				firstFace->m_adjancentFace[2] = dgInt16 (prevEdgeIndex);
				prevFace->m_adjancentFace[1] = dgInt16 (firstFace - m_simplex);


				for (dgInt32 i = 0; i < deadCount; i ++) {
					if (!deadFaces[i]->m_inHeap){
						dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) deadFaces[i];
						nextFreeFace->m_next = m_freeFace;
						m_freeFace = nextFreeFace;
					}
				}

				while (heapSort.GetCount() && (!heapSort[0]->m_isActive)) {
					face = heapSort[0];
					heapSort.Pop();
					dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) face;
					nextFreeFace->m_next = m_freeFace;
					m_freeFace = nextFreeFace;
				}

				for (dgInt32 i = 0; i < silhouetteCapCount; i ++) {
					face = sillueteCap[i].m_face;
					*sillueteCap[i].m_faceCopling = dgInt16 (face - m_simplex);

					if (CalcFacePlane (face)) {
						face->m_inHeap = 1;
						heapSort.Push(face, face->m_w);
					}
				}
			}
		} else {
			dgAssert (0);
			dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) face;
			nextFreeFace->m_next = m_freeFace;
			m_freeFace = nextFreeFace;
		}
	}
	return closestFace;
}


dgInt32 dgContactSolver::HullHullContinueContacts (dgFloat32& timeStep, dgContactPoint* const contactOut, dgInt32 contactID, dgInt32 maxContacts)
{
	dgMinkFace* face;
	dgInt32 count = 0;
	dgMinkReturnCode code = CalcSeparatingPlane(face);

	m_lastFaceCode = code;
	if (code == dgMinkIntersecting) {

		face = CalculateClipPlane ();
		if (face) {
			if (maxContacts) {
				count = CalculateContacts (face, contactID, contactOut, maxContacts);
				dgAssert (count <= maxContacts);
			}
			timeStep = dgFloat32 (0.0f);
		}

	} else if (code == dgMinkDisjoint) {
		dgVector saveHull[3];
		dgVector saveAver[3];

		dgAssert (face);
		dgInt32 i0 = face->m_vertex[0];
		dgInt32 i1 = face->m_vertex[1];
		dgInt32 i2 = face->m_vertex[2];
		dgVector plane ((m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]));
		dgAssert (plane % plane > dgFloat32 (0.0f));
		dgFloat32 projVeloc = plane % m_localRelVeloc;
		if (projVeloc >= dgFloat32 (-1.0e-24f)) {
			code = UpdateSeparatingPlane(face, dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));
			if (code != dgMinkDisjoint) {
				return 0;
			}
			dgAssert (code == dgMinkDisjoint);
			i0 = face->m_vertex[0];
			i1 = face->m_vertex[1];
			i2 = face->m_vertex[2];
			plane = (m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]);

			dgAssert (plane % plane > dgFloat32 (0.0f));
			projVeloc = plane % m_localRelVeloc;
			if (projVeloc >= dgFloat32 (-1.0e-24f)) {
				return 0;
			}
		}

		dgFloat32 timeOfImpact = (plane % m_hullVertex[i0]) / (plane % m_localRelVeloc + dgFloat32 (1.0e-24f));
		if (timeOfImpact > 0.0f) {
			saveHull[0] = m_hullVertex[i0];
			saveHull[1] = m_hullVertex[i1];
			saveHull[2] = m_hullVertex[i2];
			saveAver[0] = m_averVertex[i0];
			saveAver[1] = m_averVertex[i1];
			saveAver[2] = m_averVertex[i2];
			dgVector p1 (m_localRelVeloc.Scale (timeOfImpact));

			dgFloat32 t0 = dgFloat32 (0.0f);
			for (dgInt32 maxPasses = 0; (maxPasses < 32) && (timeOfImpact < timeStep) && (timeOfImpact > t0); maxPasses ++) {
				t0 = timeOfImpact;

				dgMinkFace* tmpFaceface;
				code = UpdateSeparatingPlane(tmpFaceface, p1);
				dgAssert (code != dgMinkError);
				if (code == dgMinkDisjoint) {
					dgFloat32 den;

					dgAssert (tmpFaceface);

					face = tmpFaceface;
					i0 = face->m_vertex[0];
					i1 = face->m_vertex[1];
					i2 = face->m_vertex[2];
					dgVector plane ((m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]));
					dgAssert (plane % plane > dgFloat32 (0.0f));

					den = plane % m_localRelVeloc ;
					if (den >= dgFloat32 (-1.0e-24f)) {
						code = UpdateSeparatingPlane(tmpFaceface, p1);
						dgAssert (code == dgMinkDisjoint);

						i0 = face->m_vertex[0];
						i1 = face->m_vertex[1];
						i2 = face->m_vertex[2];
						dgVector plane ((m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]));
						dgAssert (plane % plane > dgFloat32 (-1.0e-24f));
						den = plane % m_localRelVeloc;
						if (den >= dgFloat32 (0.0f)) {
							return 0;
						}
					}

					saveHull[0] = m_hullVertex[i0];
					saveHull[1] = m_hullVertex[i1];
					saveHull[2] = m_hullVertex[i2];
					saveAver[0] = m_averVertex[i0];
					saveAver[1] = m_averVertex[i1];
					saveAver[2] = m_averVertex[i2];
					if (den < dgFloat32 (-1.0e-24f)) {
						timeOfImpact = (plane % m_hullVertex[i0]) / den;
						if (timeOfImpact < 0.0f) {
							return 0;
						}

						dgAssert (timeOfImpact >= 0.0f);

						//dgAssert (timeOfImpact >= dgFloat32 (-1.0f));
						p1 = m_localRelVeloc.Scale (timeOfImpact);
					}
				}
			}

			if ((timeOfImpact <= timeStep) && (timeOfImpact >= dgFloat32 (0.0f))) {

				if (maxContacts) {
					count = CalculateContactsContinue(contactID, contactOut, maxContacts, saveHull, saveAver, timeOfImpact);
					dgAssert(count <= maxContacts);
				}

				timeStep = timeOfImpact;
				if (count) {
					dgVector step (m_referenceBodyVeloc.Scale (timeOfImpact));
					for (i0 = 0; i0 < count; i0 ++) {
						contactOut[i0].m_point += step;
					}
				}
			}
		}
	}
	return count;
}


dgInt32 dgContactSolver::CalculateContactsContinue(dgInt32 contacID, dgContactPoint* const contactOut, dgInt32 maxContacts, const dgVector* const diffPoins, const dgVector* const averPoins, dgFloat32 timestep)
{
	dgMinkFace* const face = &m_simplex[0];

	dgVector step (m_localRelVeloc.Scale (timestep + DG_ROBUST_PLANE_CLIP * dgRsqrt(m_localRelVeloc % m_localRelVeloc)));
	for (dgInt32 i = 0; i < 3; i ++) {
		m_hullVertex[i] = diffPoins[i] - step;
		m_averVertex[i] = averPoins[i] + step;
	}
	CalcFacePlane (face);
	dgPlane &facePlane  = *face;
	if ((facePlane % m_localRelVeloc) > dgFloat32 (0.0f)) {
		facePlane = facePlane.Scale (dgFloat32 (-1.0f));
	}

	dgVector minkFloatingPosit (m_matrix.m_posit );
	m_matrix.m_posit += step;
	dgInt32 count = CalculateContacts(face, contacID, contactOut, maxContacts);
	dgAssert (count <= maxContacts);

	m_matrix.m_posit = minkFloatingPosit;
	return count;
}


dgInt32 dgContactSolver::HullHullContactsLarge (dgInt32 contactID)
{
	dgInt32 count = 0;
	dgMinkFace* face;
	dgMinkReturnCode code = CalcSeparatingPlaneLarge(face);

	switch (code)
	{
		case dgMinkIntersecting:
		{
			face = CalculateClipPlaneLarge ();
			if (face) {
				count = CalculateContacts (face, contactID, m_proxy->m_contacts, m_proxy->m_maxContacts);
			}
			break;
		}

		case dgMinkDisjoint:
		{
			if (CalcFacePlaneLarge (face)) {

				//dgAssert (face->m_w >= dgFloat32 (0.0f));
				dgAssert (face->m_w >= dgFloat32 (-1.0e-1f));
				dgAssert ((*face) % (*face) > dgFloat32 (0.0f));
				if (face->m_w < m_penetrationPadding) {
					dgVector step (*face);
					step = step.Scale (-(face->m_w + DG_IMPULSIVE_CONTACT_PENETRATION));

					dgInt32 i0 = face->m_vertex[0];
					m_hullVertex[i0] = dgVector (dgFloat32 (m_hullVertexLarge[i0].m_x), dgFloat32 (m_hullVertexLarge[i0].m_y), dgFloat32 (m_hullVertexLarge[i0].m_z), dgFloat32 (0.0f));
					m_averVertex[i0] = dgVector (dgFloat32 (m_averVertexLarge[i0].m_x), dgFloat32 (m_averVertexLarge[i0].m_y), dgFloat32 (m_averVertexLarge[i0].m_z), dgFloat32 (0.0f));
					m_hullVertex[i0] -= step;
					m_averVertex[i0] += step;

					m_matrix.m_posit += step;
					//dgVector stepWorld (m_proxy->m_referenceMatrix.RotateVector(step));
					dgVector stepWorld (m_proxy->m_referenceCollision->m_globalMatrix.RotateVector(step));

					//m_proxy->m_floatingMatrix.m_posit += stepWorld;
					dgCollisionInstance* const saveShape = m_proxy->m_floatingCollision; 
					dgCollisionInstance tmpInstance (*m_proxy->m_floatingCollision);
					m_proxy->m_floatingCollision = &tmpInstance;
					tmpInstance.m_globalMatrix.m_posit += stepWorld;

					count = CalculateContacts(face, contactID, m_proxy->m_contacts, m_proxy->m_maxContacts);
					stepWorld = stepWorld.Scale (dgFloat32 (0.5f));

					m_proxy->m_floatingCollision = saveShape;

					dgContactPoint* const contactOut = m_proxy->m_contacts; 
					for (dgInt32 i0 = 0; i0 < count; i0 ++ ) {
						contactOut[i0].m_point -= stepWorld ;
					}
					return count;
				}
			}
		}
		case dgMinkError:
		default:;
	}
	return count;
}


dgInt32 dgContactSolver::HullHullContacts (dgInt32 contactID)
{
	dgInt32 count = 0;

	dgMinkFace* face;
	dgMinkReturnCode code = CalcSeparatingPlane(face);
	switch (code)
	{
		case dgMinkIntersecting:
		{
			face = CalculateClipPlane ();
			if (face) {
				count = CalculateContacts (face, contactID, m_proxy->m_contacts, m_proxy->m_maxContacts);
				dgAssert (count <= m_proxy->m_maxContacts);
			}
			break;
		}

		case dgMinkDisjoint:
		{
			dgAssert (face);

			if (CalcFacePlane (face)) {
				//dgAssert (face->m_w >= dgFloat32 (0.0f));
				dgAssert (face->m_w >= dgFloat32 (-1.0e-2f));
				dgAssert ((*face) % (*face) > dgFloat32 (0.0f));
				if (face->m_w < m_penetrationPadding) {
					dgVector step (*face);
					step = step.Scale (-(face->m_w + DG_IMPULSIVE_CONTACT_PENETRATION));

					dgInt32 i0 = face->m_vertex[0];
					m_hullVertex[i0] -= step;
					m_averVertex[i0] += step;

					m_matrix.m_posit += step;
					//dgVector stepWorld (m_proxy->m_referenceMatrix.RotateVector(step));
					dgVector stepWorld (m_proxy->m_referenceCollision->m_globalMatrix.RotateVector(step));

					dgCollisionInstance* const saveShape = m_proxy->m_floatingCollision; 
					dgCollisionInstance tmpInstance (*m_proxy->m_floatingCollision);
					m_proxy->m_floatingCollision = &tmpInstance;
					tmpInstance.m_globalMatrix.m_posit += stepWorld;

					count = CalculateContacts(face, contactID, m_proxy->m_contacts, m_proxy->m_maxContacts);
					stepWorld = stepWorld.Scale (dgFloat32 (0.5f));

					m_proxy->m_floatingCollision = saveShape;

					dgContactPoint* const contactOut = m_proxy->m_contacts; 
					for (i0 = 0; i0 < count; i0 ++ ) {
						contactOut[i0].m_point -= stepWorld ;
					}
					return count;
				}
			}
		}
		case dgMinkError:
		default:;
	}
	return count;
}


#endif
