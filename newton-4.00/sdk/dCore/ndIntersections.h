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

#ifndef __ND_INTERSECTION_H__
#define __ND_INTERSECTION_H__

#include "ndCoreStdafx.h"
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"

class ndPlane;
class ndFastRay;

enum dIntersectStatus
{
	t_StopSearh,
	t_ContinueSearh
};

typedef dIntersectStatus (*dAaabbIntersectCallback) (void* const context, 
													  const dFloat32* const polygon, dInt32 strideInBytes,
													  const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);

typedef dFloat32 (*dRayIntersectCallback) (void* const context, 
										   const dFloat32* const polygon, dInt32 strideInBytes,
										   const dInt32* const indexArray, dInt32 indexCount);

D_CORE_API ndBigVector dPointToRayDistance (const ndBigVector& point, const ndBigVector& ray_p0, const ndBigVector& ray_p1); 
D_CORE_API ndBigVector dPointToTriangleDistance (const ndBigVector& point, const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2);
D_CORE_API ndBigVector dPointToTetrahedrumDistance (const ndBigVector& point, const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3);

D_CORE_API bool dRayBoxClip (ndVector& ray_p0, ndVector& ray_p1, const ndVector& boxP0, const ndVector& boxP1); 
D_CORE_API dFloat32 dRayCastBox (const ndVector& p0, const ndVector& p1, const ndVector& boxP0, const ndVector& boxP1, ndVector& normalOut);
D_CORE_API dFloat32 dRayCastSphere (const ndVector& p0, const ndVector& p1, const ndVector& origin, dFloat32 radius);
//D_CORE_API void dRayToRayDistance(const ndVector& ray_p0, const ndVector& ray_p1, const ndVector& ray_q0, const ndVector& ray_q1, ndVector& p0Out, ndVector& p1Out);

inline dInt32 dOverlapTest (const ndVector& p0, const ndVector& p1, const ndVector& q0, const ndVector& q1)
{
	ndVector r0(p0 - q1);
	ndVector r1(p1 - q0);
	ndVector val(r0 * r1);
	dInt32 mask = val.GetSignMask() & 0x07;
	return (mask == 0x07);
}

inline dInt32 dBoxInclusionTest (const ndVector& p0, const ndVector& p1, const ndVector& q0, const ndVector& q1)
{
	ndVector val(ndVector::m_negOne & ((p0 >= q0) & (p1 <= q1)));
	dInt32 mask = val.GetSignMask() & 0x07;
	return (mask == 0x07);
}

inline dInt32 dCompareBox (const ndVector& p0, const ndVector& p1, const ndVector& q0, const ndVector& q1)
{
	dAssert(0);
	return (p0.m_x != q0.m_x) || (p0.m_y != q0.m_y) || (p0.m_z != q0.m_z) || (p1.m_x != q1.m_x) || (p1.m_y != q1.m_y) || (p1.m_z != q1.m_z);
}

inline void dMovingAABB (ndVector& p0, ndVector& p1, const ndVector& veloc, const ndVector& omega, dFloat32 timestep, dFloat32 maxRadius, dFloat32 minRadius)
{
	ndVector linearStep (veloc.Scale (timestep));

	// estimate the maximum effect of the angular velocity and enlarge that box by that value (use 45 degrees as max angle not 90)
	dAssert (omega.m_w == dFloat32 (0.0f));
	dFloat32 maxAngle = dMin (dSqrt (omega.DotProduct(omega).GetScalar() * timestep * timestep), dFloat32 (45.0f * dDegreeToRad));

	dFloat32 angularTravel = (maxRadius - minRadius) * maxAngle;
	ndVector angularStep (angularTravel, angularTravel, angularTravel, dFloat32 (0.0f));
	
	ndVector r0 (p0 - angularStep);
	ndVector r1 (p1 + angularStep);
	ndVector q0 (r0 + linearStep);
	ndVector q1 (r1 + linearStep);
	p0 = r0.GetMin (q0) & ndVector::m_triplexMask;
	p1 = r1.GetMax (q1) & ndVector::m_triplexMask;
}

//inline dFloat32 dBoxPenetration (const ndVector& minBox, const ndVector& maxBox)
//{
//	dAssert(maxBox.m_x >= minBox.m_x);
//	dAssert(maxBox.m_y >= minBox.m_y);
//	dAssert(maxBox.m_z >= minBox.m_z);
//
//	ndVector mask ((minBox * maxBox) < ndVector::m_zero);
//	ndVector dist (maxBox.GetMin (minBox.Abs()) & mask);
//	dist = dist.GetMin(dist.ShiftTripleRight());
//	dist = dist.GetMin(dist.ShiftTripleRight());
//	return dist.GetScalar();
//}

inline dFloat32 dBoxDistanceToOrigin2 (const ndVector& minBox, const ndVector& maxBox)
{
	dAssert(maxBox.m_x >= minBox.m_x);
	dAssert(maxBox.m_y >= minBox.m_y);
	dAssert(maxBox.m_z >= minBox.m_z);
	const ndVector mask (((minBox * maxBox) > ndVector::m_zero) & ndVector::m_triplexMask);
	const ndVector dist (maxBox.Abs().GetMin (minBox.Abs()) & mask);
	return dist.DotProduct(dist).GetScalar();
}

#endif

