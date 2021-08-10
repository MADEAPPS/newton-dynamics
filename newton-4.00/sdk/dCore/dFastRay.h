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

#ifndef __D_FAST_RAY_H__
#define __D_FAST_RAY_H__

#include "dCoreStdafx.h"
#include "dDebug.h"
#include "dVector.h"

D_MSV_NEWTON_ALIGN_32
class dFastRay
{
	public:
	dFastRay(const dVector& l0, const dVector& l1);
	dInt32 BoxTest(const dVector& minBox, const dVector& maxBox) const;
	dFloat32 BoxIntersect(const dVector& minBox, const dVector& maxBox) const;

	dFastRay PointDistance(const dVector& point) const;
	D_CORE_API dFastRay RayDistance(const dVector& ray_p0, const dVector& ray_p1) const;
	D_CORE_API dFloat32 PolygonIntersect(const dVector& normal, dFloat32 maxT, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount) const;

	const dVector m_p0;
	const dVector m_p1;
	const dVector m_diff;
	dVector m_dpInv;
	dVector m_minT;
	dVector m_maxT;
	dVector m_unitDir;
	dVector m_isParallel;
} D_GCC_NEWTON_ALIGN_32 ;

inline dFastRay::dFastRay(const dVector& l0, const dVector& l1)
	:m_p0(l0 & dVector::m_triplexMask)
	,m_p1(l1 & dVector::m_triplexMask)
	,m_diff(m_p1 - m_p0)
	,m_minT(dFloat32(0.0f))
	,m_maxT(dFloat32(1.0f))
{
	dAssert(m_p0.m_w == dFloat32(0.0f));
	dAssert(m_p1.m_w == dFloat32(0.0f));
	dAssert(m_diff.m_w == dFloat32(0.0f));

	dAssert(m_diff.DotProduct(m_diff).GetScalar() > dFloat32(0.0f));
	m_isParallel = (m_diff.Abs() < dVector(1.0e-8f));
	m_dpInv = m_diff.Select(dVector(dFloat32(1.0e-20f)), m_isParallel).Reciproc() & dVector::m_triplexMask;
	m_unitDir = m_diff.Normalize();
}

inline dFastRay dFastRay::PointDistance(const dVector& point) const
{
	//dBigVector dp(ray_p1 - ray_p0);
	//dAssert(dp.m_w == dFloat32(0.0f));
	dFloat32 t = dClamp(m_diff.DotProduct(point - m_p0).GetScalar() / m_diff.DotProduct(m_diff).GetScalar(), dFloat32(0.0f), dFloat32(1.0f));
	return dFastRay (m_p0 + m_diff.Scale(t), point);
}

inline dInt32 dFastRay::BoxTest(const dVector& minBox, const dVector& maxBox) const
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

inline dFloat32 dFastRay::BoxIntersect(const dVector& minBox, const dVector& maxBox) const
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

#endif

