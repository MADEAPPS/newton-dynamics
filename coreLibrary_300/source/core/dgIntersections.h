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

	dgInt32 BoxTest (const dgVector& minBox, const dgVector& maxBox) const;
	dgInt32 BoxTestSimd (const dgVector& minBox, const dgVector& maxBox) const;

	dgFloat32 PolygonIntersect (const dgVector& normal, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const;
	dgFloat32 PolygonIntersectSimd (const dgVector& normal, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const;

	dgFloat32 PolygonIntersectFallback (const dgVector& normal, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount) const;

	void Reset (dgFloat32 t) 
	{
		m_dpInv = m_dpBaseInv.Scale3 (dgFloat32 (1.0f) / (t + dgFloat32 (1.0e-12f)));
	}

	dgVector m_p0;
	dgVector m_p1;
	dgVector m_diff;
	dgVector m_dpInv;
	dgVector m_dpBaseInv;
	dgVector m_minT;
	dgVector m_maxT;
	dgVector m_zero;

	dgSimd m_ray_xxxx;
	dgSimd m_ray_yyyy;
	dgSimd m_ray_zzzz;
	dgInt32 m_isParallel[4];

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


DG_INLINE dgInt32 dgOverlapTestSimd (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
	dgSimd val (((dgSimd&)p0 < (dgSimd&)q1) & ((dgSimd&)p1 > (dgSimd&)q0));
	dgInt32 mask = val.GetSignMask();
	return ((mask & 0x07) == 0x07);
}

DG_INLINE dgInt32 dgOverlapTest (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
	#ifdef _M_X64
		return  dgOverlapTestSimd (p0, p1, q0, q1);
	#else 
		return ((p0.m_x < q1.m_x) && (p1.m_x > q0.m_x) && (p0.m_z < q1.m_z) && (p1.m_z > q0.m_z) && (p0.m_y < q1.m_y) && (p1.m_y > q0.m_y)); 
	#endif
}


DG_INLINE dgInt32 dgBoxInclusionTestSimd (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
	dgSimd val (((dgSimd&)p0 >= (dgSimd&)q0) & ((dgSimd&)p1 <= (dgSimd&)q1));
	dgInt32 mask = val.GetSignMask();
	return ((mask & 0x07) == 0x07);
}

DG_INLINE dgInt32 dgBoxInclusionTest (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
	#ifdef _M_X64
		return dgBoxInclusionTestSimd (p0, p1, q0, q1);
	#else 
		return (p0.m_x >= q0.m_x) && (p0.m_y >= q0.m_y) && (p0.m_z >= q0.m_z) && (p1.m_x <= q1.m_x) && (p1.m_y <= q1.m_y) && (p1.m_z <= q1.m_z);
	#endif
}


DG_INLINE dgInt32 dgCompareBox (const dgVector& p0, const dgVector& p1, const dgVector& q0, const dgVector& q1)
{
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
	dgVector q1 (r0 + linearStep);
	p0 = dgVector (dgMin (r0.m_x, q0.m_x), dgMin (r0.m_y, q0.m_y), dgMin (r0.m_z, q0.m_z), dgFloat32 (0.0f));
	p1 = dgVector (dgMax (r1.m_x, q1.m_x), dgMax (r1.m_y, q1.m_y), dgMax (r1.m_z, q1.m_z), dgFloat32 (0.0f));
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

