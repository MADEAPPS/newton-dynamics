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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"
#include "ndFastRay.h"

dFloat32 dFastRay::PolygonIntersect (const dVector& faceNormal, dFloat32 maxT, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount) const
{
	dAssert (m_p0.m_w == dFloat32 (0.0f));
	dAssert (m_p1.m_w == dFloat32 (0.0f));

	if (faceNormal.DotProduct(m_unitDir).GetScalar() < dFloat32 (0.0f)) 
	{
		dInt32 stride = dInt32(strideInBytes / sizeof (dFloat32));
		dBigVector v0(dVector(&polygon[indexArray[indexCount - 1] * stride]) & dVector::m_triplexMask);
		dBigVector p0(m_p0);
		dBigVector p0v0(v0 - p0);

		dBigVector diff(m_diff);
		dBigVector normal(faceNormal);
		dFloat64 tOut = normal.DotProduct(p0v0).GetScalar() / normal.DotProduct(diff).GetScalar();
		if ((tOut >= dFloat64(0.0f)) && (tOut <= maxT)) 
		{
			dBigVector p (p0 + diff.Scale (tOut));
			dBigVector unitDir(m_unitDir);
			for (dInt32 i = 0; i < indexCount; i++) 
			{
				dInt32 i2 = indexArray[i] * stride;
				dBigVector v1(dVector(&polygon[i2]) & dVector::m_triplexMask);

				dBigVector edge0(p - v0);
				dBigVector edge1(v1 - v0);
				dFloat64 area = unitDir.DotProduct (edge0.CrossProduct(edge1)).GetScalar();
				if (area < dFloat32 (0.0f)) 
				{
					return dFloat32 (1.2f);
				}
				v0 = v1;
			}

			return dFloat32(tOut);
		}
	}

	return dFloat32 (1.2f);
}

dRay dFastRay::RayDistance(const dVector& ray_q0, const dVector& ray_q1) const
{
	const dVector u(m_diff);
	const dVector v(dVector::m_triplexMask & (ray_q1 - ray_q0));
	const dVector w(dVector::m_triplexMask & (m_p0 - ray_q0));
	dAssert(u.m_w == dFloat32(0.0f));
	dAssert(v.m_w == dFloat32(0.0f));
	dAssert(w.m_w == dFloat32(0.0f));
	
	const dFloat32 a = u.DotProduct(u).GetScalar();
	const dFloat32 b = u.DotProduct(v).GetScalar();
	const dFloat32 c = v.DotProduct(v).GetScalar();
	const dFloat32 d = u.DotProduct(w).GetScalar();
	const dFloat32 e = v.DotProduct(w).GetScalar();
	const dFloat32 D = a*c - b*b;

	dFloat32 sD = D;
	dFloat32 tD = D;
	dFloat32 sN;
	dFloat32 tN;

	// compute the line parameters of the two closest points
	if (D < dFloat32(1.0e-8f))
	{
		sN = dFloat32(0.0f);
		sD = dFloat32(1.0f);
		tN = e;
		tD = c;
	}
	else
	{
		// get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < dFloat32(0.0f))
		{
			// sc < 0 => the s=0 edge is visible
			sN = dFloat32(0.0f);
			tN = e;
			tD = c;
		}
		else if (sN > sD)
		{
			// sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}
	
	if (tN < dFloat32(0.0f))
	{
		// tc < 0 => the t=0 edge is visible
		tN = dFloat32(0.0f);
		// recompute sc for this edge
		if (-d < dFloat32(0.0f))
		{
			sN = dFloat32(0.0f);
		}
		else if (-d > a)
		{
			sN = sD;
		}
		else
		{
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD)
	{
		// tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < dFloat32(0.0f))
		{
			sN = dFloat32(0.0f);
		}
		else if ((-d + b) > a)
		{
			sN = sD;
		}
		else
		{
			sN = (-d + b);
			sD = a;
		}
	}
	
	// finally do the division to get sc and tc
	dFloat32 sc = (dAbs(sN) < dFloat32(1.0e-8f) ? dFloat32(0.0f) : sN / sD);
	dFloat32 tc = (dAbs(tN) < dFloat32(1.0e-8f) ? dFloat32(0.0f) : tN / tD);
	
	dAssert(u.m_w == dFloat32(0.0f));
	dAssert(v.m_w == dFloat32(0.0f));
	return dRay(m_p0 + u.Scale(sc), ray_q0 + v.Scale(tc));
}
