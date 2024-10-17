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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"
#include "ndFastRay.h"

ndFloat32 ndFastRay::PolygonIntersect (const ndVector& faceNormal, ndFloat32 maxT, const ndFloat32* const polygon, ndInt32 strideInBytes, const ndInt32* const indexArray, ndInt32 indexCount) const
{
	ndAssert (m_p0.m_w == ndFloat32 (0.0f));
	ndAssert (m_p1.m_w == ndFloat32 (0.0f));

	if (faceNormal.DotProduct(m_unitDir).GetScalar() < ndFloat32 (0.0f)) 
	{
		ndInt32 stride = ndInt32(strideInBytes / sizeof (ndFloat32));
		ndBigVector v0(ndVector(&polygon[indexArray[indexCount - 1] * stride]) & ndVector::m_triplexMask);
		ndBigVector p0(m_p0);
		ndBigVector p0v0(v0 - p0);

		ndBigVector diff(m_diff);
		ndBigVector normal(faceNormal);
		ndFloat64 tOut = normal.DotProduct(p0v0).GetScalar() / normal.DotProduct(diff).GetScalar();
		if ((tOut >= ndFloat64(0.0f)) && (tOut <= maxT)) 
		{
			ndBigVector p (p0 + diff.Scale (tOut));
			ndBigVector unitDir(m_unitDir);
			for (ndInt32 i = 0; i < indexCount; ++i) 
			{
				ndInt32 i2 = indexArray[i] * stride;
				ndBigVector v1(ndVector(&polygon[i2]) & ndVector::m_triplexMask);

				ndBigVector edge0(p - v0);
				ndBigVector edge1(v1 - v0);
				ndFloat64 area = unitDir.DotProduct (edge0.CrossProduct(edge1)).GetScalar();
				if (area < ndFloat32 (0.0f)) 
				{
					return ndFloat32 (1.2f);
				}
				v0 = v1;
			}

			return ndFloat32(tOut);
		}
	}

	return ndFloat32 (1.2f);
}

ndRay ndFastRay::RayDistance(const ndVector& ray_q0, const ndVector& ray_q1) const
{
	const ndVector u(m_diff);
	const ndVector v(ndVector::m_triplexMask & (ray_q1 - ray_q0));
	const ndVector w(ndVector::m_triplexMask & (m_p0 - ray_q0));
	ndAssert(u.m_w == ndFloat32(0.0f));
	ndAssert(v.m_w == ndFloat32(0.0f));
	ndAssert(w.m_w == ndFloat32(0.0f));
	
	const ndFloat32 a = u.DotProduct(u).GetScalar();
	const ndFloat32 b = u.DotProduct(v).GetScalar();
	const ndFloat32 c = v.DotProduct(v).GetScalar();
	const ndFloat32 d = u.DotProduct(w).GetScalar();
	const ndFloat32 e = v.DotProduct(w).GetScalar();
	const ndFloat32 D = a*c - b*b;

	ndFloat32 sD = D;
	ndFloat32 tD = D;
	ndFloat32 sN;
	ndFloat32 tN;

	// compute the line parameters of the two closest points
	if (D < ndFloat32(1.0e-8f))
	{
		sN = ndFloat32(0.0f);
		sD = ndFloat32(1.0f);
		tN = e;
		tD = c;
	}
	else
	{
		// get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < ndFloat32(0.0f))
		{
			// sc < 0 => the s=0 edge is visible
			sN = ndFloat32(0.0f);
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
	
	if (tN < ndFloat32(0.0f))
	{
		// tc < 0 => the t=0 edge is visible
		tN = ndFloat32(0.0f);
		// recompute sc for this edge
		if (-d < ndFloat32(0.0f))
		{
			sN = ndFloat32(0.0f);
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
		if ((-d + b) < ndFloat32(0.0f))
		{
			sN = ndFloat32(0.0f);
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
	ndFloat32 sc = (ndAbs(sN) < ndFloat32(1.0e-8f) ? ndFloat32(0.0f) : sN / sD);
	ndFloat32 tc = (ndAbs(tN) < ndFloat32(1.0e-8f) ? ndFloat32(0.0f) : tN / tD);
	
	ndAssert(u.m_w == ndFloat32(0.0f));
	ndAssert(v.m_w == ndFloat32(0.0f));
	return ndRay(m_p0 + u.Scale(sc), ray_q0 + v.Scale(tc));
}
