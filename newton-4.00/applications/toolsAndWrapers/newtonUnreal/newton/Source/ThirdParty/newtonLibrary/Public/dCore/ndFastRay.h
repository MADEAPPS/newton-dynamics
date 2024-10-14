/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_FAST_RAY_H__
#define __ND_FAST_RAY_H__

#include "ndCoreStdafx.h"
#include "ndDebug.h"
#include "ndVector.h"
#include "ndClassAlloc.h"

D_MSV_NEWTON_ALIGN_32
class ndRay: public ndClassAlloc
{
	public:
	ndRay()
		:ndClassAlloc()
	{
	}

	ndRay(const ndVector& l0, const ndVector& l1)
		:ndClassAlloc()
		,m_p0(l0 & ndVector::m_triplexMask)
		,m_p1(l1 & ndVector::m_triplexMask)
	{
	}

	const ndVector m_p0;
	const ndVector m_p1;
} D_GCC_NEWTON_ALIGN_32;


D_MSV_NEWTON_ALIGN_32
class ndFastRay: public ndRay
{
	public:
	ndFastRay(const ndVector& l0, const ndVector& l1);

	ndInt32 BoxTest(const ndVector& minBox, const ndVector& maxBox) const;
	ndFloat32 BoxIntersect(const ndVector& minBox, const ndVector& maxBox) const;

	ndRay PointDistance(const ndVector& point) const;
	D_CORE_API ndRay RayDistance(const ndVector& ray_p0, const ndVector& ray_p1) const;
	D_CORE_API ndFloat32 PolygonIntersect(const ndVector& normal, ndFloat32 maxT, const ndFloat32* const polygon, ndInt32 strideInBytes, const ndInt32* const indexArray, ndInt32 indexCount) const;

	const ndVector m_diff;
	ndVector m_dpInv;
	ndVector m_minT;
	ndVector m_maxT;
	ndVector m_unitDir;
	ndVector m_isParallel;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndFastRay::ndFastRay(const ndVector& l0, const ndVector& l1)
	:ndRay(l0, l1)
	,m_diff(m_p1 - m_p0)
	,m_minT(ndFloat32(0.0f))
	,m_maxT(ndFloat32(1.0f))
{
	ndAssert(m_p0.m_w == ndFloat32(0.0f));
	ndAssert(m_p1.m_w == ndFloat32(0.0f));
	ndAssert(m_diff.m_w == ndFloat32(0.0f));

	ndAssert(m_diff.DotProduct(m_diff).GetScalar() > ndFloat32(0.0f));
	m_isParallel = (m_diff.Abs() < ndVector(1.0e-8f));
	m_dpInv = m_diff.Select(ndVector(ndFloat32(1.0e-20f)), m_isParallel).Reciproc() & ndVector::m_triplexMask;
	m_unitDir = m_diff.Normalize();
}

inline ndRay ndFastRay::PointDistance(const ndVector& point) const
{
	//dBigVector dp(ray_p1 - ray_p0);
	//ndAssert(dp.m_w == ndFloat32(0.0f));
	ndFloat32 t = ndClamp(m_diff.DotProduct(point - m_p0).GetScalar() / m_diff.DotProduct(m_diff).GetScalar(), ndFloat32(0.0f), ndFloat32(1.0f));
	return ndRay (m_p0 + m_diff.Scale(t), point);
}

inline ndInt32 ndFastRay::BoxTest(const ndVector& minBox, const ndVector& maxBox) const
{
#if 1
	ndVector test(((m_p0 <= minBox) | (m_p0 >= maxBox)) & m_isParallel);
	if (test.GetSignMask() & 0x07) 
	{
		return 0;
	}

	ndVector tt0(m_dpInv * (minBox - m_p0));
	ndVector tt1(m_dpInv * (maxBox - m_p0));

	ndVector t0(m_minT.GetMax(tt0.GetMin(tt1)));
	ndVector t1(m_maxT.GetMin(tt0.GetMax(tt1)));
	t0 = t0.GetMax(t0.ShiftTripleRight());
	t1 = t1.GetMin(t1.ShiftTripleRight());
	t0 = t0.GetMax(t0.ShiftTripleRight());
	t1 = t1.GetMin(t1.ShiftTripleRight());
	return ((t0 < t1).GetSignMask() & 1);

#else

	ndFloat32 tmin = 0.0f;
	ndFloat32 tmax = 1.0f;

	for (ndInt32 i = 0; i < 3; ++i)
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
			ndFloat32 t1 = (minBox[i] - m_p0[i]) * m_dpInv[i];
			ndFloat32 t2 = (maxBox[i] - m_p0[i]) * m_dpInv[i];

			if (t1 > t2)
			{
				ndSwap(t1, t2);
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

inline ndFloat32 ndFastRay::BoxIntersect(const ndVector& minBox, const ndVector& maxBox) const
{
	ndVector test(((m_p0 <= minBox) | (m_p0 >= maxBox)) & m_isParallel);
	if (test.GetSignMask() & 0x07)
	{
		return ndFloat32(1.2f);
	}
	ndVector tt0(m_dpInv * (minBox - m_p0));
	ndVector tt1(m_dpInv * (maxBox - m_p0));
	ndVector t0(m_minT.GetMax(tt0.GetMin(tt1)));
	ndVector t1(m_maxT.GetMin(tt0.GetMax(tt1)));
	t0 = t0.GetMax(t0.ShiftTripleRight());
	t1 = t1.GetMin(t1.ShiftTripleRight());
	t0 = t0.GetMax(t0.ShiftTripleRight());
	t1 = t1.GetMin(t1.ShiftTripleRight());
	ndVector mask(t0 < t1);
	ndVector maxDist(ndFloat32(1.2f));
	t0 = maxDist.Select(t0, mask);
	ndAssert((mask.GetSignMask() & 1) == (t0.m_x < ndFloat32(1.0f)));
	return t0.GetScalar();
}

#endif

