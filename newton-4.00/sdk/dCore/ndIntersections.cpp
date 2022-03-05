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
#include "ndSort.h"
#include "ndMemory.h"
#include "ndVector.h"
#include "ndPlane.h"
#include "ndGoogol.h"
#include "ndFastRay.h"
#include "ndIntersections.h"

bool dRayBoxClip (ndVector& p0, ndVector& p1, const ndVector& boxP0, const ndVector& boxP1) 
{	
	dAssert (p0.m_w == ndFloat32(0.0f));
	dAssert (p1.m_w == ndFloat32(0.0f));
	dAssert (boxP0.m_w == ndFloat32(0.0f));
	dAssert (boxP1.m_w == ndFloat32(0.0f));
	for (ndInt32 i = 0; i < 3; i ++) 
	{
		ndFloat32 tmp0 = boxP1[i] - p0[i];
		if (tmp0 > ndFloat32 (0.0f)) 
		{
			ndFloat32 tmp1 = boxP1[i] - p1[i];
			if (tmp1 < ndFloat32 (0.0f)) 
			{
				p1 = p0 + (p1 - p0).Scale (tmp0 / (p1[i] - p0[i])); 
				p1[i] = boxP1[i];
			}
		} 
		else 
		{
			ndFloat32 tmp1 = boxP1[i] - p1[i];
			if (tmp1 > ndFloat32 (0.0f)) 
			{
				p0 += (p1 - p0).Scale (tmp0 / (p1[i] - p0[i])); 
				p0[i] = boxP1[i];
			} 
			else 
			{
				return false;
			}
		}

		tmp0 = boxP0[i] - p0[i];
		if (tmp0 < ndFloat32 (0.0f)) 
		{
			ndFloat32 tmp1 = boxP0[i] - p1[i];
			if (tmp1 > ndFloat32 (0.0f)) 
			{
				p1 = p0 + (p1 - p0).Scale (tmp0 / (p1[i] - p0[i])); 
				p1[i] = boxP0[i];
			}
		} 
		else 
		{
			ndFloat32 tmp1 = boxP0[i] - p1[i];
			if (tmp1 < ndFloat32 (0.0f)) 
			{
				p0 += (p1 - p0).Scale (tmp0 / (p1[i] - p0[i])); 
				p0[i] = boxP0[i];
			} 
			else 
			{
				return false;
			}
		}
	}
	return true;
}

ndBigVector dPointToRayDistance(const ndBigVector& point, const ndBigVector& ray_p0, const ndBigVector& ray_p1)
{
	ndBigVector dp (ray_p1 - ray_p0);
	dAssert (dp.m_w == ndFloat32 (0.0f));
	ndFloat64 t = dClamp (dp.DotProduct(point - ray_p0).GetScalar() / dp.DotProduct(dp).GetScalar(), ndFloat64(0.0f), ndFloat64 (1.0f));
	return ray_p0 + dp.Scale (t);
}

ndBigVector dPointToTriangleDistance(const ndBigVector& point, const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2)
{
	const ndBigVector e10(p1 - p0);
	const ndBigVector e20(p2 - p0);
	const ndFloat64 a00 = e10.DotProduct(e10).GetScalar();
	const ndFloat64 a11 = e20.DotProduct(e20).GetScalar();
	const ndFloat64 a01 = e10.DotProduct(e20).GetScalar();

	const ndFloat64 det = a00 * a11 - a01 * a01;
	dAssert(det >= ndFloat32(0.0f));
	if (dAbs(det) > ndFloat32(1.0e-24f)) 
	{
		ndBigVector p0Point (point - p0);
		const ndFloat64 b0 = e10.DotProduct(p0Point).GetScalar();
		const ndFloat64 b1 = e20.DotProduct(p0Point).GetScalar();

		const ndFloat64 beta = b1 * a00 - a01 * b0;
		const ndFloat64 alpha = b0 * a11 - a01 * b1;
		if (beta < ndFloat32(0.0f)) 
		{
			return dPointToRayDistance (point, p0, p1);
		} 
		else if (alpha < ndFloat32(0.0f)) 
		{
			return dPointToRayDistance (point, p0, p2);
		} 
		else if ((alpha + beta) > det) 
		{
			return dPointToRayDistance (point, p1, p2);
		}
		return p0 + (e10.Scale(alpha) + e20.Scale(beta)).Scale(ndFloat64(1.0f) / det);
	}
	// this is a degenerated triangle. this should never happens
	dAssert(0);
	return p0;
}

ndBigVector dPointToTetrahedrumDistance (const ndBigVector& point, const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3)
{
	const ndBigVector e10(p1 - p0);
	const ndBigVector e20(p2 - p0);
	const ndBigVector e30(p3 - p0);

	const ndFloat64 d0 = sqrt(e10.DotProduct(e10).GetScalar());
	if (d0 > ndFloat64(0.0f)) 
	{
		const ndFloat64 invd0 = ndFloat64(1.0f) / d0;
		const ndFloat64 l10 = e20.DotProduct(e10).GetScalar() * invd0;
		const ndFloat64 l20 = e30.DotProduct(e10).GetScalar() * invd0;
		const ndFloat64 desc11 = e20.DotProduct(e20).GetScalar() - l10 * l10;
		if (desc11 > ndFloat64(0.0f)) 
		{
			const ndFloat64 d1 = sqrt(desc11);
			const ndFloat64 invd1 = ndFloat64(1.0f) / d1;
			const ndFloat64 l21 = (e30.DotProduct(e20).GetScalar() - l20 * l10) * invd1;
			const ndFloat64 desc22 = e30.DotProduct(e30).GetScalar() - l20 * l20 - l21 * l21;
			if (desc22 > ndFloat64(0.0f)) 
			{
				ndBigVector p0Point (point - p0);
				const ndFloat64 d2 = sqrt(desc22);
				const ndFloat64 invd2 = ndFloat64(1.0f) / d2;
				
				const ndFloat64 b0 = e10.DotProduct(p0Point).GetScalar();
				const ndFloat64 b1 = e20.DotProduct(p0Point).GetScalar();
				const ndFloat64 b2 = e30.DotProduct(p0Point).GetScalar();

				ndFloat64 u1 = b0 * invd0;
				ndFloat64 u2 = (b1 - l10 * u1) * invd1;
				ndFloat64 u3 = (b2 - l20 * u1 - l21 * u2) * invd2 * invd2;
				u2 = (u2 - l21 * u3) * invd1;
				u1 = (u1 - l10 * u2 - l20 * u3) * invd0;
				if (u3 < ndFloat64(0.0f)) 
				{
					// this looks funny but it is correct
					return dPointToTriangleDistance(point, p0, p1, p2);
				} 
				else if (u2 < ndFloat64(0.0f)) 
				{
					return dPointToTriangleDistance(point, p0, p1, p3);
				} 
				else if (u1 < ndFloat64(0.0f)) 
				{
					return dPointToTriangleDistance(point, p0, p2, p3);
				} 
				else if (u1 + u2 + u3 > ndFloat64(1.0f)) 
				{
					return dPointToTriangleDistance(point, p1, p2, p3);
				}
				return p0 + e10.Scale(u1) + e20.Scale(u2) + e30.Scale(u3);
			}
		}
	}
	// this is a degenerated tetra. this should never happens
	dAssert(0);
	return p0;
}

void dRayToRayDistance(const ndBigVector& ray_p0, const ndBigVector& ray_p1, const ndVector& ray_q0, const ndVector& ray_q1, ndBigVector& p0Out, ndBigVector& p1Out)
{
	ndFloat64 sN;
	ndFloat64 tN;
	
	ndBigVector u(ray_p1 - ray_p0);
	ndBigVector v(ray_q1 - ray_q0);
	ndBigVector w(ray_p0 - ray_q0);
	dAssert(u.m_w == ndFloat64(0.0f));
	dAssert(v.m_w == ndFloat64(0.0f));
	dAssert(w.m_w == ndFloat64(0.0f));
	
	ndFloat64 a = u.DotProduct(u).GetScalar();
	ndFloat64 b = u.DotProduct(v).GetScalar();
	ndFloat64 c = v.DotProduct(v).GetScalar();
	ndFloat64 d = u.DotProduct(w).GetScalar();
	ndFloat64 e = v.DotProduct(w).GetScalar();
	ndFloat64 D = a*c - b*b;
	ndFloat64 sD = D;
	ndFloat64 tD = D;
	
	// compute the line parameters of the two closest points
	if (D < ndFloat64(1.0e-8f)) 
	{
		sN = ndFloat64(0.0f);
		sD = ndFloat64(1.0f);
		tN = e;
		tD = c;
	}
	else 
	{
		// get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < ndFloat64(0.0f)) 
		{
			// sc < 0 => the s=0 edge is visible
			sN = ndFloat64(0.0f);
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
	
	if (tN < ndFloat64(0.0f)) 
	{
		// tc < 0 => the t=0 edge is visible
		tN = ndFloat64(0.0f);
		// recompute sc for this edge
		if (-d < ndFloat64(0.0f)) 
		{
			sN = ndFloat64(0.0f);
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
		if ((-d + b) < ndFloat64(0.0f)) 
		{
			sN = ndFloat64(0.0f);
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
	ndFloat64 sc = (dAbs(sN) < ndFloat64(1.0e-8f) ? ndFloat64(0.0f) : sN / sD);
	ndFloat64 tc = (dAbs(tN) < ndFloat64(1.0e-8f) ? ndFloat64(0.0f) : tN / tD);
	
	dAssert(u.m_w == ndFloat64(0.0f));
	dAssert(v.m_w == ndFloat64(0.0f));
	p0Out = ray_p0 + u.Scale(sc);
	p1Out = ray_q0 + v.Scale(tc);
}

ndFloat32 dRayCastSphere (const ndVector& p0, const ndVector& p1, const ndVector& origin, ndFloat32 radius)
{
	ndVector p0Origin (p0 - origin);
	dAssert (p0Origin.m_w == ndFloat32 (0.0f));
	if (p0Origin.DotProduct(p0Origin).GetScalar() < (ndFloat32 (100.0f) * radius * radius)) 
	{
		ndVector dp (p1 - p0);
		dAssert (dp.m_w == ndFloat32 (0.0f));
		ndFloat32 a = dp.DotProduct(dp).GetScalar();
		ndFloat32 b = ndFloat32 (2.0f) * p0Origin.DotProduct(dp).GetScalar();
		ndFloat32 c = p0Origin.DotProduct(p0Origin).GetScalar() - radius * radius;
		ndFloat32 desc = b * b - ndFloat32 (4.0f) * a * c;
		if (desc >= 0.0f) 
		{
			desc = ndSqrt (desc);
			ndFloat32 den = ndFloat32 (0.5f) / a;
			ndFloat32 t0 = (-b + desc) * den;
			ndFloat32 t1 = (-b - desc) * den;
			if ((t0 >= ndFloat32 (0.0f)) && (t1 >= ndFloat32 (0.0f))) 
			{
				t0 =  dMin(t0, t1);
				if (t0 <= ndFloat32 (1.0f)) 
				{
					return t0;
				}
			} 
			else if (t0 >= ndFloat32 (0.0f)) 
			{
				if (t0 <= ndFloat32 (1.0f)) 
				{
					return t0;
				}
			} 
			else 
			{
				if ((t1 >= ndFloat32 (0.0f)) && (t1 <= ndFloat32 (1.0f))) 
				{
					return t1;
				}
			}
		}
	} 
	else 
	{
		ndBigVector p0Origin1 (p0Origin);
ndBigVector dp(p1 - p0);
dAssert(dp.m_w == ndFloat32(0.0f));
ndFloat64 a = dp.DotProduct(dp).GetScalar();
ndFloat64 b = ndFloat32(2.0f) * p0Origin1.DotProduct(dp).GetScalar();
ndFloat64 c = p0Origin1.DotProduct(p0Origin1).GetScalar() - ndFloat64(radius) * radius;
ndFloat64 desc = b * b - ndFloat32(4.0f) * a * c;
if (desc >= 0.0f)
{
	desc = sqrt(desc);
	ndFloat64 den = ndFloat32(0.5f) / a;
	ndFloat64 t0 = (-b + desc) * den;
	ndFloat64 t1 = (-b - desc) * den;
	if ((t0 >= ndFloat32(0.0f)) && (t1 >= ndFloat32(0.0f)))
	{
		t0 = dMin(t0, t1);
		if (t0 <= ndFloat32(1.0f))
		{
			return ndFloat32(t0);
		}
	}
	else if (t0 >= ndFloat32(0.0f))
	{
		if (t0 <= ndFloat32(1.0f))
		{
			return ndFloat32(t0);
		}
	}
	else
	{
		if ((t1 >= ndFloat32(0.0f)) && (t1 <= ndFloat32(1.0f)))
		{
			return ndFloat32(t1);
		}
	}
}
	}
	return ndFloat32(1.2f);
}

ndFloat32 dRayCastBox(const ndVector& p0, const ndVector& p1, const ndVector& boxP0, const ndVector& boxP1, ndVector& normalOut)
{
	ndInt32 index = 0;
	ndFloat32 signDir = ndFloat32(0.0f);
	ndFloat32 tmin = ndFloat32(0.0f);
	ndFloat32 tmax = ndFloat32(1.0f);

	for (ndInt32 i = 0; i < 3; i++)
	{
		ndFloat32 dp = p1[i] - p0[i];
		if (dAbs(dp) < ndFloat32(1.0e-8f))
		{
			if (p0[i] <= boxP0[i] || p0[i] >= boxP1[i])
			{
				return ndFloat32(1.2f);
			}
		}
		else
		{
			dp = ndFloat32(1.0f) / dp;
			ndFloat32 t1 = (boxP0[i] - p0[i]) * dp;
			ndFloat32 t2 = (boxP1[i] - p0[i]) * dp;

			ndFloat32 sign = ndFloat32(-1.0f);
			if (t1 > t2)
			{
				sign = 1;
				dSwap(t1, t2);
			}
			if (t1 > tmin)
			{
				signDir = sign;
				index = i;
				tmin = t1;
			}
			if (t2 < tmax)
			{
				tmax = t2;
			}
			if (tmin > tmax)
			{
				return ndFloat32(1.2f);
			}
		}
	}

	if (tmin > ndFloat32(0.0f))
	{
		dAssert(tmin < 1.0f);
		normalOut = ndVector(ndFloat32(0.0f));
		normalOut[index] = signDir;
	}
	else
	{
		tmin = ndFloat32(1.2f);
	}
	return tmin;
}
