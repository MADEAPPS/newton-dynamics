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

#ifndef __dgIntersections__
#define __dgIntersections__

#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgVector.h"


class dgPlane;
class dgObject;
class dgPolyhedra;

enum dgIntersectStatus
{
	t_StopSearh,
	t_ContinueSearh
};

typedef dgIntersectStatus (*dgAABBIntersectCallback) (void* const context, 
													  const dgFloat32* const polygon, dgInt32 strideInBytes,
													  const dgInt32* const indexArray, dgInt32 indexCount);

typedef dgFloat32 (*dgRayIntersectCallback) (void* const context, 
											 const dgFloat32* const polygon, dgInt32 strideInBytes,
											 const dgInt32* const indexArray, dgInt32 indexCount);



DG_MSC_VECTOR_ALIGMENT 
class dgFastRayTest
{
	public:
	dgFastRayTest(const dgVector& l0, const dgVector& l1);

	dgFloat32 PolygonIntersect (const dgVector& normal, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const;
	dgFloat32 PolygonIntersectFallback (const dgVector& normal, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const;

	DG_INLINE void Reset (dgFloat32 t) 
	{
		m_dpInv = m_dpBaseInv.Scale4 (dgFloat32 (1.0f) / (t + dgFloat32 (1.0e-12f)));
	}

	DG_INLINE dgInt32 BoxTest (const dgVector& minBox, const dgVector& maxBox) const
	{
		#if 1
			dgVector test (((m_p0 <= minBox) | (m_p0 >= maxBox)) & m_isParallel);
			if (test.GetSignMask() & 0x07) {
				return 0;
			}
			dgVector tt0 = (minBox - m_p0).CompProduct4(m_dpInv);
			dgVector tt1 ((maxBox - m_p0).CompProduct4(m_dpInv));
			dgVector t0 (m_minT.GetMax(tt0.GetMin(tt1)));
			dgVector t1 (m_maxT.GetMin(tt0.GetMax(tt1)));
			t0 = t0.GetMax(t0.ShiftTripleRight());
			t1 = t1.GetMin(t1.ShiftTripleRight());
			t0 = t0.GetMax(t0.ShiftTripleRight());
			t1 = t1.GetMin(t1.ShiftTripleRight());
			return ((t0 < t1).GetSignMask() & 1);

		#else

			dgFloat32 tmin = 0.0f;          
			dgFloat32 tmax = 1.0f;

			for (dgInt32 i = 0; i < 3; i++) {
				if (m_isParallel[i]) {
					if (m_p0[i] <= minBox[i] || m_p0[i] >= maxBox[i]) {
						return 0;
					}
				} else {
					dgFloat32 t1 = (minBox[i] - m_p0[i]) * m_dpInv[i];
					dgFloat32 t2 = (maxBox[i] - m_p0[i]) * m_dpInv[i];

					if (t1 > t2) {
						dgSwap(t1, t2);
					}
					if (t1 > tmin) {
						tmin = t1;
					}
					if (t2 < tmax) {
						tmax = t2;
					}
					if (tmin > tmax) {
						return 0;
					}
				}
			}
			return 0x1;
		#endif
	}

	dgVector m_p0;
	dgVector m_p1;
	dgVector m_diff;
	dgVector m_dpInv;
	dgVector m_dpBaseInv;
	dgVector m_minT;
	dgVector m_maxT;
	dgVector m_zero;
	dgVector m_isParallel;
	dgFloat32 m_dirError;
	dgFloat32 m_magRayTest;
}DG_GCC_VECTOR_ALIGMENT;

class dgBeamHitStruct
{
	public:
	dgVector m_Origin; 
	dgVector m_NormalOut;
	dgObject* m_HitObjectOut; 
	dgFloat32 m_ParametricIntersctionOut;
};


bool dgApi dgRayBoxClip (dgVector& ray_p0, dgVector& ray_p1, const dgVector& boxP0, const dgVector& boxP1); 


dgVector dgApi dgPointToRayDistance (const dgVector& point, const dgVector& ray_p0, const dgVector& ray_p1); 

void dgApi dgRayToRayDistance (const dgVector& ray_p0, const dgVector& ray_p1, const dgVector& ray_q0, const dgVector& ray_q1, dgVector& p0Out, dgVector& p1Out); 

dgVector dgPointToTriangleDistance (const dgVector& point, const dgVector& p0, const dgVector& p1, const dgVector& p2, const dgVector& normal);
dgBigVector dgPointToTriangleDistance (const dgBigVector& point, const dgBigVector& p0, const dgBigVector& p1, const dgBigVector& p2, const dgBigVector& normal);


dgFloat32 dgApi dgRayCastBox (const dgVector& p0, const dgVector& p1, const dgVector& boxP0, const dgVector& boxP1, dgVector& normalOut);
dgFloat32 dgApi dgRayCastSphere (const dgVector& p0, const dgVector& p1, const dgVector& origin, dgFloat32 radius);

DG_INLINE dgInt32 dgOverlapTest (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
//	dgInt32 test = ((p0.m_x < q1.m_x) && (p1.m_x > q0.m_x) && (p0.m_z < q1.m_z) && (p1.m_z > q0.m_z) && (p0.m_y < q1.m_y) && (p1.m_y > q0.m_y));
//	return  test
	dgVector val ((p0 < q1) & (p1 > q0));
	dgInt32 mask = val.GetSignMask();
	return ((mask & 0x07) == 0x07);
}



DG_INLINE dgInt32 dgBoxInclusionTest (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
//	dgInt32 test = (p0.m_x >= q0.m_x) && (p0.m_y >= q0.m_y) && (p0.m_z >= q0.m_z) && (p1.m_x <= q1.m_x) && (p1.m_y <= q1.m_y) && (p1.m_z <= q1.m_z);
//	return test;

	dgVector val ((p0 >= q0) & (p1 <= q1));
	dgInt32 mask = val.GetSignMask();
	return ((mask & 0x07) == 0x07);
}


DG_INLINE dgInt32 dgCompareBox (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
	dgAssert(0);
	return (p0.m_x != q0.m_x) || (p0.m_y != q0.m_y) || (p0.m_z != q0.m_z) || (p1.m_x != q1.m_x) || (p1.m_y != q1.m_y) || (p1.m_z != q1.m_z);
}


DG_INLINE void dgMovingAABB (dgVector& p0, dgVector& p1, const dgVector& veloc, const dgVector& omega, dgFloat32 timestep, dgFloat32 maxRadius, dgFloat32 minRadius)
{
	dgVector linearStep (veloc.Scale3 (timestep));

	// estimate the maximum effect of the angular velocity and enlarge that box by that value (use 45 degrees as max angle not 90)
	dgFloat32 maxAngle = dgMin (dgSqrt ((omega % omega) * timestep * timestep), dgFloat32 (45.0f * 3.14159f / 180.0f));

	dgFloat32 angularTravel = (maxRadius - minRadius) * maxAngle;
	dgVector angularStep (angularTravel, angularTravel, angularTravel, dgFloat32 (0.0f));
	
	dgVector r0 (p0 - angularStep);
	dgVector r1 (p1 + angularStep);
	dgVector q0 (r0 + linearStep);
	dgVector q1 (r1 + linearStep);
	//p0 = dgVector (dgMin (r0.m_x, q0.m_x), dgMin (r0.m_y, q0.m_y), dgMin (r0.m_z, q0.m_z), dgFloat32 (0.0f));
	p0 = r0.GetMin (q0);
	//p1 = dgVector (dgMax (r1.m_x, q1.m_x), dgMax (r1.m_y, q1.m_y), dgMax (r1.m_z, q1.m_z), dgFloat32 (0.0f));
	p1 = r1.GetMax (q1);
	p0.m_w = dgFloat32 (0.0f);
	p1.m_w = dgFloat32 (0.0f);
}

DG_INLINE dgFloat32 dgBoxDistanceToOrigin2 (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
	dgVector boxP0 (p0 - q1);
	dgVector boxP1 (p1 - q0);

	dgVector dist (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < 3; i ++) {
		if(boxP0[i] > dgFloat32 (0.0f)) {
			dist[i] = boxP0[i];
		} else if(boxP1[i] < dgFloat32 (0.0f)) {
			dist[i] = boxP1[i];
		}
	}
	return dist % dist;
}


dgFloat32 dgApi dgSweepLineToPolygonTimeOfImpact (const dgVector& p0, const dgVector& p1, dgFloat32 radius, dgInt32 count, const dgVector* const polygon, const dgVector& normal, dgVector& normalOut, dgVector& contactOut);

#endif

