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

#ifndef __D_INTERSECTION_H__
#define __D_INTERSECTION_H__

#include "dCoreStdafx.h"
#include "dDebug.h"
#include "dVector.h"
#include "dMatrix.h"

class dPlane;
class dFastRay;

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

D_CORE_API dInt32 dConvexHull2d(dInt32 count, dVector* const pointArray);
D_CORE_API dBigVector dPointToRayDistance (const dBigVector& point, const dBigVector& ray_p0, const dBigVector& ray_p1); 
D_CORE_API dBigVector dPointToTriangleDistance (const dBigVector& point, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2);
D_CORE_API dBigVector dPointToTetrahedrumDistance (const dBigVector& point, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3);

D_CORE_API bool dRayBoxClip (dVector& ray_p0, dVector& ray_p1, const dVector& boxP0, const dVector& boxP1); 
D_CORE_API dFloat32 dRayCastBox (const dVector& p0, const dVector& p1, const dVector& boxP0, const dVector& boxP1, dVector& normalOut);
D_CORE_API dFloat32 dRayCastSphere (const dVector& p0, const dVector& p1, const dVector& origin, dFloat32 radius);
//D_CORE_API void dRayToRayDistance(const dVector& ray_p0, const dVector& ray_p1, const dVector& ray_q0, const dVector& ray_q1, dVector& p0Out, dVector& p1Out);

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

//D_INLINE dFloat32 dBoxPenetration (const dVector& minBox, const dVector& maxBox)
//{
//	dAssert(maxBox.m_x >= minBox.m_x);
//	dAssert(maxBox.m_y >= minBox.m_y);
//	dAssert(maxBox.m_z >= minBox.m_z);
//
//	dVector mask ((minBox * maxBox) < dVector::m_zero);
//	dVector dist (maxBox.GetMin (minBox.Abs()) & mask);
//	dist = dist.GetMin(dist.ShiftTripleRight());
//	dist = dist.GetMin(dist.ShiftTripleRight());
//	return dist.GetScalar();
//}

D_INLINE dFloat32 dBoxDistanceToOrigin2 (const dVector& minBox, const dVector& maxBox)
{
	dAssert(maxBox.m_x >= minBox.m_x);
	dAssert(maxBox.m_y >= minBox.m_y);
	dAssert(maxBox.m_z >= minBox.m_z);
	const dVector mask (((minBox * maxBox) > dVector::m_zero) & dVector::m_triplexMask);
	const dVector dist (maxBox.Abs().GetMin (minBox.Abs()) & mask);
	return dist.DotProduct(dist).GetScalar();
}

D_MSV_NEWTON_ALIGN_32 
class dFastAabbInfo : public dMatrix
{
	public:
	D_INLINE dFastAabbInfo()
		:dMatrix(dGetIdentityMatrix())
		,m_absDir(dGetIdentityMatrix())
		,m_p0(dVector::m_zero)
		,m_p1(dVector::m_zero)
		,m_size(dVector::m_zero)
		,m_separationDistance(dFloat32(1.0e10f))
	{
	}

	D_INLINE dFastAabbInfo(const dMatrix& matrix, const dVector& size)
		:dMatrix(matrix)
		,m_separationDistance(dFloat32(1.0e10f))
	{
		SetTransposeAbsMatrix (matrix);
		m_size = dVector(matrix[0].Abs().Scale(size.m_x) + matrix[1].Abs().Scale(size.m_y) + matrix[2].Abs().Scale(size.m_z));
		m_p0 = (matrix[3] - m_size) & dVector::m_triplexMask;
		m_p1 = (matrix[3] + m_size) & dVector::m_triplexMask;
	}

	D_INLINE dFastAabbInfo(const dVector& p0, const dVector& p1)
		:dMatrix(dGetIdentityMatrix())
		,m_absDir(dGetIdentityMatrix())
		,m_p0(p0)
		,m_p1(p1)
		,m_size(dVector::m_half * (p1 - p0))
		,m_separationDistance(dFloat32(1.0e10f))
	{
		m_posit = (dVector::m_half * (p1 + p0)) | dVector::m_wOne;
		dAssert(m_size.m_w == dFloat32(0.0f));
		dAssert(m_posit.m_w == dFloat32(1.0f));
	}

	D_INLINE const dVector& GetOrigin() const
	{
		return m_p0;
	}

	D_INLINE const dVector& GetTarget() const
	{
		return m_p1;
	}

	D_INLINE void SetSeparatingDistance(const dFloat32 distance)
	{
		m_separationDistance = distance;
	}

	D_INLINE void SetTransposeAbsMatrix (const dMatrix& matrix)
	{
		m_absDir = matrix.Transpose();
		m_absDir[0] = m_absDir[0].Abs();
		m_absDir[1] = m_absDir[1].Abs();
		m_absDir[2] = m_absDir[2].Abs();
	}

	dFloat32 PolygonBoxDistance(const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const;
	dFloat32 PolygonBoxRayDistance(const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, const dFastRay& ray) const;

	private:
	dMatrix MakeFaceMatrix(const dVector& faceNormal, dInt32, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const;
	void MakeBox1(dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const;
	void MakeBox2(const dMatrix& faceMatrix, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const;

	protected:
	dMatrix m_absDir;
	dVector m_p0;
	dVector m_p1;
	dVector m_size;
	mutable dVector m_separationDistance;

	friend class dAabbPolygonSoup;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

