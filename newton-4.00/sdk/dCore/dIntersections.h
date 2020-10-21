/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_INTERSECTION_H__
#define __D_INTERSECTION_H__

#include "dCoreStdafx.h"
//#include "dgObb.h"
#include "dDebug.h"
#include "dVector.h"
#include "dMatrix.h"

class dPlane;
//class dgObject;
//class dPolyhedra;

enum dIntersectStatus
{
	t_StopSearh,
	t_ContinueSearh
};

//typedef dIntersectStatus (*dgAABBIntersectCallback) (void* const context, 
//													  const dFloat32* const polygon, dInt32 strideInBytes,
//													  const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
//
//typedef dFloat32 (*dgRayIntersectCallback) (void* const context, 
//											 const dFloat32* const polygon, dInt32 strideInBytes,
//											 const dInt32* const indexArray, dInt32 indexCount);

D_CORE_API dBigVector dPointToRayDistance (const dBigVector& point, const dBigVector& ray_p0, const dBigVector& ray_p1); 
D_CORE_API dBigVector dPointToTriangleDistance (const dBigVector& point, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2);
D_CORE_API dBigVector dPointToTetrahedrumDistance (const dBigVector& point, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3);

D_CORE_API bool dRayBoxClip (dVector& ray_p0, dVector& ray_p1, const dVector& boxP0, const dVector& boxP1); 
D_CORE_API void dRayToRayDistance (const dVector& ray_p0, const dVector& ray_p1, const dVector& ray_q0, const dVector& ray_q1, dVector& p0Out, dVector& p1Out); 
D_CORE_API dFloat32 dRayCastBox (const dVector& p0, const dVector& p1, const dVector& boxP0, const dVector& boxP1, dVector& normalOut);
D_CORE_API dFloat32 dRayCastSphere (const dVector& p0, const dVector& p1, const dVector& origin, dFloat32 radius);

D_INLINE dInt32 dOverlapTest (const dVector& p0, const dVector& p1, const dVector& q0, const dVector& q1)
{
	dVector r0(p0 - q1);
	dVector r1(p1 - q0);
	dVector val(r0 * r1);
	dInt32 mask = val.GetSignMask() & 0x07;
	return (mask == 0x07);
}

D_INLINE dInt32 dBoxInclusionTest (const dVector& p0, const dVector& p1, const dVector& q0, const dVector& q1)
{
	dVector val(dVector::m_negOne & ((p0 >= q0) & (p1 <= q1)));
	dInt32 mask = val.GetSignMask() & 0x07;
	return (mask == 0x07);
}

D_INLINE dInt32 dCompareBox (const dVector& p0, const dVector& p1, const dVector& q0, const dVector& q1)
{
	dAssert(0);
	return (p0.m_x != q0.m_x) || (p0.m_y != q0.m_y) || (p0.m_z != q0.m_z) || (p1.m_x != q1.m_x) || (p1.m_y != q1.m_y) || (p1.m_z != q1.m_z);
}

D_INLINE void dMovingAABB (dVector& p0, dVector& p1, const dVector& veloc, const dVector& omega, dFloat32 timestep, dFloat32 maxRadius, dFloat32 minRadius)
{
	dVector linearStep (veloc.Scale (timestep));

	// estimate the maximum effect of the angular velocity and enlarge that box by that value (use 45 degrees as max angle not 90)
	dAssert (omega.m_w == dFloat32 (0.0f));
	dFloat32 maxAngle = dMin (dSqrt (omega.DotProduct(omega).GetScalar() * timestep * timestep), dFloat32 (45.0f * dDegreeToRad));

	dFloat32 angularTravel = (maxRadius - minRadius) * maxAngle;
	dVector angularStep (angularTravel, angularTravel, angularTravel, dFloat32 (0.0f));
	
	dVector r0 (p0 - angularStep);
	dVector r1 (p1 + angularStep);
	dVector q0 (r0 + linearStep);
	dVector q1 (r1 + linearStep);
	p0 = r0.GetMin (q0) & dVector::m_triplexMask;
	p1 = r1.GetMax (q1) & dVector::m_triplexMask;
}

D_INLINE dFloat32 dBoxPenetration (const dVector& minBox, const dVector& maxBox)
{
	dAssert(maxBox.m_x >= minBox.m_x);
	dAssert(maxBox.m_y >= minBox.m_y);
	dAssert(maxBox.m_z >= minBox.m_z);

	dVector mask ((minBox * maxBox) < dVector::m_zero);
	dVector dist (maxBox.GetMin (minBox.Abs()) & mask);
	dist = dist.GetMin(dist.ShiftTripleRight());
	dist = dist.GetMin(dist.ShiftTripleRight());
	return dist.GetScalar();
}

D_INLINE dFloat32 dBoxDistanceToOrigin2 (const dVector& minBox, const dVector& maxBox)
{
	dAssert(maxBox.m_x >= minBox.m_x);
	dAssert(maxBox.m_y >= minBox.m_y);
	dAssert(maxBox.m_z >= minBox.m_z);
	dVector mask ((minBox * maxBox) > dVector::m_zero);
	dVector dist (maxBox.Abs().GetMin (minBox.Abs()) & mask);
	return dist.DotProduct(dist).GetScalar();
}

D_MSV_NEWTON_ALIGN_32
class dFastRayTest
{
	public:
	D_INLINE dFastRayTest(const dVector& l0, const dVector& l1)
		:m_p0(l0)
		,m_p1(l1)
		,m_diff((l1 - l0) & dVector::m_triplexMask)
		,m_minT(dFloat32(0.0f))
		,m_maxT(dFloat32(1.0f))
	{
		dAssert(m_p0.m_w == dFloat32(0.0f));
		dAssert(m_p1.m_w == dFloat32(0.0f));
		dAssert(m_diff.m_w == dFloat32(0.0f));

		dAssert (m_diff.DotProduct(m_diff).GetScalar() > dFloat32 (0.0f));
		m_isParallel = (m_diff.Abs() < dVector(1.0e-8f));
		//m_dpInv = (((dVector(dFloat32(1.0e-20)) & m_isParallel) | m_diff.AndNot(m_isParallel)).Reciproc()) & dVector::m_triplexMask;
		m_dpInv = m_diff.Select (dVector(dFloat32(1.0e-20f)), m_isParallel).Reciproc() & dVector::m_triplexMask;
		m_unitDir = m_diff.Normalize();
	}

	dFloat32 PolygonIntersect(const dVector& normal, dFloat32 maxT, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount) const;

	D_INLINE dInt32 BoxTest(const dVector& minBox, const dVector& maxBox) const
	{
#if 1
		dVector test(((m_p0 <= minBox) | (m_p0 >= maxBox)) & m_isParallel);
		if (test.GetSignMask() & 0x07) {
			return 0;
		}

		dVector tt0(m_dpInv * (minBox - m_p0));
		dVector tt1(m_dpInv * (maxBox - m_p0));

		dVector t0(m_minT.GetMax(tt0.GetMin(tt1)));
		dVector t1(m_maxT.GetMin(tt0.GetMax(tt1)));
		t0 = t0.GetMax(t0.ShiftTripleRight());
		t1 = t1.GetMin(t1.ShiftTripleRight());
		t0 = t0.GetMax(t0.ShiftTripleRight());
		t1 = t1.GetMin(t1.ShiftTripleRight());
		return ((t0 < t1).GetSignMask() & 1);

#else

		dFloat32 tmin = 0.0f;
		dFloat32 tmax = 1.0f;

		for (dInt32 i = 0; i < 3; i++) 
		{
			if (m_isParallel[i]) 
			{
				if (m_p0[i] <= minBox[i] || m_p0[i] >= maxBox[i]) 
				{
					return 0;
				}
			} 
			else 
			{
				dFloat32 t1 = (minBox[i] - m_p0[i]) * m_dpInv[i];
				dFloat32 t2 = (maxBox[i] - m_p0[i]) * m_dpInv[i];

				if (t1 > t2) 
				{
					dSwap(t1, t2);
				}
				if (t1 > tmin) 
				{
					tmin = t1;
				}
				if (t2 < tmax) 
				{
					tmax = t2;
				}
				if (tmin > tmax) 
				{
					return 0;
				}
			}
		}
		return 0x1;
#endif
	}

	D_INLINE dFloat32 BoxIntersect(const dVector& minBox, const dVector& maxBox) const
	{
		dVector test(((m_p0 <= minBox) | (m_p0 >= maxBox)) & m_isParallel);
		if (test.GetSignMask() & 0x07) 
		{
			return dFloat32(1.2f);
		}
		dVector tt0(m_dpInv * (minBox - m_p0));
		dVector tt1(m_dpInv * (maxBox - m_p0));
		dVector t0(m_minT.GetMax(tt0.GetMin(tt1)));
		dVector t1(m_maxT.GetMin(tt0.GetMax(tt1)));
		t0 = t0.GetMax(t0.ShiftTripleRight());
		t1 = t1.GetMin(t1.ShiftTripleRight());
		t0 = t0.GetMax(t0.ShiftTripleRight());
		t1 = t1.GetMin(t1.ShiftTripleRight());
		dVector mask(t0 < t1);
		dVector maxDist(dFloat32(1.2f));
		t0 = maxDist.Select(t0, mask);
		dAssert((mask.GetSignMask() & 1) == (t0.m_x < dFloat32(1.0f)));
		return t0.GetScalar();
	}

	const dVector m_p0;
	const dVector m_p1;
	const dVector m_diff;
	dVector m_dpInv;
	dVector m_minT;
	dVector m_maxT;
	dVector m_unitDir;
	dVector m_isParallel;
} D_GCC_NEWTON_ALIGN_32 ;

D_MSV_NEWTON_ALIGN_32 
//class dFastAabbInfo: public dgObb
class dFastAabbInfo : public dMatrix
{
	public:
	D_INLINE dFastAabbInfo()
		//:dgObb()
		//,m_absDir(dGetIdentityMatrix())
		//,m_separationDistance(dFloat32(1.0e10f))
	{
		dAssert(0);
	}
//
//	D_INLINE dFastAabbInfo(const dMatrix& matrix, const dVector& size)
//		:dgObb(matrix, size)
//		,m_separationDistance(dFloat32(1.0e10f))
//	{
//		SetTransposeAbsMatrix (matrix);
//		dVector size1 (matrix[0].Abs().Scale(size.m_x) + matrix[1].Abs().Scale(size.m_y) + matrix[2].Abs().Scale(size.m_z));
//		m_p0 = (matrix[3] - size1) & dVector::m_triplexMask;
//		m_p1 = (matrix[3] + size1) & dVector::m_triplexMask;
//	}
//
//	D_INLINE dFastAabbInfo(const dVector& p0, const dVector& p1)
//		:dgObb(dGetIdentityMatrix(), dVector::m_half * (p1 - p0))
//		,m_absDir(dGetIdentityMatrix())
//		,m_separationDistance(dFloat32(1.0e10f))
//		,m_p0(p0)
//		,m_p1(p1)
//	{
//		m_posit = ((dVector::m_half * (p1 + p0)) & dVector::m_triplexMask) | dVector::m_wOne;
//	}
//
//	D_INLINE void SetTransposeAbsMatrix (const dMatrix& matrix)
//	{
//		m_absDir = matrix.Transpose();
//		m_absDir[0] = m_absDir[0].Abs();
//		m_absDir[1] = m_absDir[1].Abs();
//		m_absDir[2] = m_absDir[2].Abs();
//		//m_absDir[3] = dVector::m_wOne;
//	}
//
//	D_INLINE dFloat32 PolygonBoxRayDistance (const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, const dgFastRayTest& ray) const
//	{
//		dVector minBox;
//		dVector maxBox;
//		MakeBox1 (indexCount, indexArray, stride, vertexArray, minBox, maxBox);
//		dFloat32 dist0 = ray.BoxIntersect(minBox, maxBox);
//		if (dist0 < dFloat32 (1.0f)) {
//			dMatrix faceMatrix (MakeFaceMatrix (faceNormal, indexCount, indexArray, stride, vertexArray));
//
//			MakeBox2 (faceMatrix, indexCount, indexArray, stride, vertexArray, minBox, maxBox);
//			dVector veloc (faceMatrix.RotateVector(ray.m_diff) & dVector::m_triplexMask);
//			dgFastRayTest localRay (dVector (dFloat32 (0.0f)), veloc);
//			dFloat32 dist1 = localRay.BoxIntersect(minBox, maxBox);
//			dist0 = dMax (dist1, dist0);
//		}
//		return dist0;
//	}
//
//
//	D_INLINE dFloat32 PolygonBoxDistance (const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const
//	{
//		dVector minBox;
//		dVector maxBox;
//		MakeBox1 (indexCount, indexArray, stride, vertexArray, minBox, maxBox);
//		dVector mask(minBox * maxBox < dVector(dFloat32(0.0f)));
//		dVector dist(maxBox.GetMin(minBox.Abs()) & mask);
//		dist = dist.GetMin(dist.ShiftTripleRight());
//		dist = dist.GetMin(dist.ShiftTripleRight());
//		dFloat32 dist0 = dist.GetScalar();
//		if (dist0 > dFloat32 (0.0f)) {
//			dMatrix faceMatrix (MakeFaceMatrix (faceNormal, indexCount, indexArray, stride, vertexArray));
//			MakeBox2 (faceMatrix, indexCount, indexArray, stride, vertexArray, minBox, maxBox);
//			dVector mask2(minBox * maxBox < dVector(dFloat32(0.0f)));
//			dVector dist2(maxBox.GetMin(minBox.Abs()) & mask2);
//			dist2 = dist2.GetMin(dist2.ShiftTripleRight());
//			dist2 = dist2.GetMin(dist2.ShiftTripleRight());
//			dFloat32 dist1 = dist2.GetScalar();
//			dist0 = (dist1 > dFloat32 (0.0f)) ? dMax (dist0, dist1) : dFloat32 (0.0f);
//			if (dist0 <= dFloat32(0.0f)) {
//				dVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask2));
//				dist2 = p1p0.DotProduct(p1p0);
//				dist2 = dist2.Sqrt() * dVector::m_negOne;
//				dist0 = dist2.GetScalar();
//			}
//		} else {
//			dVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask));
//			dist = p1p0.DotProduct(p1p0);
//			dist = dist.Sqrt() * dVector::m_negOne;
//			dist0 = dist.GetScalar();
//		}
//		return	dist0;
//	}
//
//	private:
//	D_INLINE void MakeBox1 (dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const
//	{
//		dVector faceBoxP0 (&vertexArray[indexArray[0] * stride]);
//		faceBoxP0 = faceBoxP0 & dVector::m_triplexMask;
//		dVector faceBoxP1 (faceBoxP0);
//		for (dInt32 i = 1; i < indexCount; i ++) {
//			dVector p (&vertexArray[indexArray[i] * stride]);
//			p = p & dVector::m_triplexMask;
//			faceBoxP0 = faceBoxP0.GetMin(p); 
//			faceBoxP1 = faceBoxP1.GetMax(p); 
//		}
//
//		minBox = faceBoxP0 - m_p1;
//		maxBox = faceBoxP1 - m_p0;
//	}
//
//	D_INLINE void MakeBox2 (const dMatrix& faceMatrix, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const
//	{
//		dVector faceBoxP0 (faceMatrix.TransformVector (dVector (&vertexArray[indexArray[0] * stride]) & dVector::m_triplexMask));
//		dVector faceBoxP1 (faceBoxP0);
//		for (dInt32 i = 1; i < indexCount; i ++) {
//			dVector p (faceMatrix.TransformVector (dVector (&vertexArray[indexArray[i] * stride]) & dVector::m_triplexMask));
//			faceBoxP0 = faceBoxP0.GetMin(p); 
//			faceBoxP1 = faceBoxP1.GetMax(p); 
//		}
//		faceBoxP0 = faceBoxP0 & dVector::m_triplexMask;
//		faceBoxP1 = faceBoxP1 & dVector::m_triplexMask;
//
//		dMatrix matrix = *this * faceMatrix;
//		dVector size (matrix[0].Abs().Scale(m_size.m_x) + matrix[1].Abs().Scale(m_size.m_y) + matrix[2].Abs().Scale(m_size.m_z));
//		dVector boxP0 ((matrix.m_posit - size) & dVector::m_triplexMask);
//		dVector boxP1 ((matrix.m_posit + size) & dVector::m_triplexMask);
//
//		minBox = faceBoxP0 - boxP1;
//		maxBox = faceBoxP1 - boxP0;
//	}
//
//
//	D_INLINE dMatrix MakeFaceMatrix (const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const
//	{
//		dMatrix faceMatrix;
//		dVector origin (&vertexArray[indexArray[0] * stride]);
//		dVector pin (&vertexArray[indexArray[0] * stride]);
//		pin = pin & dVector::m_triplexMask;
//		origin = origin & dVector::m_triplexMask;
//
//		dVector pin1 (&vertexArray[indexArray[1] * stride]);
//		pin1 = pin1 & dVector::m_triplexMask;
//
//		faceMatrix[0] = faceNormal;
//		faceMatrix[1] = pin1 - origin;
//		faceMatrix[1] = faceMatrix[1].Normalize();
//		faceMatrix[2] = faceMatrix[0].CrossProduct(faceMatrix[1]);
//		faceMatrix[3] = origin | dVector::m_wOne; 
//		return faceMatrix.Inverse();
//	}
//
//	protected:
	dMatrix m_absDir;
//	mutable dVector m_separationDistance;
	dVector m_p0;
	dVector m_p1;
	dVector m_size;

	friend class dAABBPolygonSoup;
//	friend class dCollisionUserMesh;
//	friend class dCollisionHeightField;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

