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

bool dRayBoxClip (dVector& p0, dVector& p1, const dVector& boxP0, const dVector& boxP1) 
{	
	dAssert (p0.m_w == dFloat32(0.0f));
	dAssert (p1.m_w == dFloat32(0.0f));
	dAssert (boxP0.m_w == dFloat32(0.0f));
	dAssert (boxP1.m_w == dFloat32(0.0f));
	for (dInt32 i = 0; i < 3; i ++) 
	{
		dFloat32 tmp0 = boxP1[i] - p0[i];
		if (tmp0 > dFloat32 (0.0f)) 
		{
			dFloat32 tmp1 = boxP1[i] - p1[i];
			if (tmp1 < dFloat32 (0.0f)) 
			{
				p1 = p0 + (p1 - p0).Scale (tmp0 / (p1[i] - p0[i])); 
				p1[i] = boxP1[i];
			}
		} 
		else 
		{
			dFloat32 tmp1 = boxP1[i] - p1[i];
			if (tmp1 > dFloat32 (0.0f)) 
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
		if (tmp0 < dFloat32 (0.0f)) 
		{
			dFloat32 tmp1 = boxP0[i] - p1[i];
			if (tmp1 > dFloat32 (0.0f)) 
			{
				p1 = p0 + (p1 - p0).Scale (tmp0 / (p1[i] - p0[i])); 
				p1[i] = boxP0[i];
			}
		} 
		else 
		{
			dFloat32 tmp1 = boxP0[i] - p1[i];
			if (tmp1 < dFloat32 (0.0f)) 
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

dBigVector dPointToRayDistance(const dBigVector& point, const dBigVector& ray_p0, const dBigVector& ray_p1)
{
	dBigVector dp (ray_p1 - ray_p0);
	dAssert (dp.m_w == dFloat32 (0.0f));
	dFloat64 t = dClamp (dp.DotProduct(point - ray_p0).GetScalar() / dp.DotProduct(dp).GetScalar(), dFloat64(0.0f), dFloat64 (1.0f));
	return ray_p0 + dp.Scale (t);
}

dBigVector dPointToTriangleDistance(const dBigVector& point, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2)
{
	const dBigVector e10(p1 - p0);
	const dBigVector e20(p2 - p0);
	const dFloat64 a00 = e10.DotProduct(e10).GetScalar();
	const dFloat64 a11 = e20.DotProduct(e20).GetScalar();
	const dFloat64 a01 = e10.DotProduct(e20).GetScalar();

	const dFloat64 det = a00 * a11 - a01 * a01;
	dAssert(det >= dFloat32(0.0f));
	if (dAbs(det) > dFloat32(1.0e-24f)) 
	{
		dBigVector p0Point (point - p0);
		const dFloat64 b0 = e10.DotProduct(p0Point).GetScalar();
		const dFloat64 b1 = e20.DotProduct(p0Point).GetScalar();

		const dFloat64 beta = b1 * a00 - a01 * b0;
		const dFloat64 alpha = b0 * a11 - a01 * b1;
		if (beta < dFloat32(0.0f)) 
		{
			return dPointToRayDistance (point, p0, p1);
		} 
		else if (alpha < dFloat32(0.0f)) 
		{
			return dPointToRayDistance (point, p0, p2);
		} 
		else if ((alpha + beta) > det) 
		{
			return dPointToRayDistance (point, p1, p2);
		}
		return p0 + (e10.Scale(alpha) + e20.Scale(beta)).Scale(dFloat64(1.0f) / det);
	}
	// this is a degenerated triangle. this should never happens
	dAssert(0);
	return p0;
}

dBigVector dPointToTetrahedrumDistance (const dBigVector& point, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3)
{
	const dBigVector e10(p1 - p0);
	const dBigVector e20(p2 - p0);
	const dBigVector e30(p3 - p0);

	const dFloat64 d0 = sqrt(e10.DotProduct(e10).GetScalar());
	if (d0 > dFloat64(0.0f)) 
	{
		const dFloat64 invd0 = dFloat64(1.0f) / d0;
		const dFloat64 l10 = e20.DotProduct(e10).GetScalar() * invd0;
		const dFloat64 l20 = e30.DotProduct(e10).GetScalar() * invd0;
		const dFloat64 desc11 = e20.DotProduct(e20).GetScalar() - l10 * l10;
		if (desc11 > dFloat64(0.0f)) 
		{
			const dFloat64 d1 = sqrt(desc11);
			const dFloat64 invd1 = dFloat64(1.0f) / d1;
			const dFloat64 l21 = (e30.DotProduct(e20).GetScalar() - l20 * l10) * invd1;
			const dFloat64 desc22 = e30.DotProduct(e30).GetScalar() - l20 * l20 - l21 * l21;
			if (desc22 > dFloat64(0.0f)) 
			{
				dBigVector p0Point (point - p0);
				const dFloat64 d2 = sqrt(desc22);
				const dFloat64 invd2 = dFloat64(1.0f) / d2;
				
				const dFloat64 b0 = e10.DotProduct(p0Point).GetScalar();
				const dFloat64 b1 = e20.DotProduct(p0Point).GetScalar();
				const dFloat64 b2 = e30.DotProduct(p0Point).GetScalar();

				dFloat64 u1 = b0 * invd0;
				dFloat64 u2 = (b1 - l10 * u1) * invd1;
				dFloat64 u3 = (b2 - l20 * u1 - l21 * u2) * invd2 * invd2;
				u2 = (u2 - l21 * u3) * invd1;
				u1 = (u1 - l10 * u2 - l20 * u3) * invd0;
				if (u3 < dFloat64(0.0f)) 
				{
					// this looks funny but it is correct
					return dPointToTriangleDistance(point, p0, p1, p2);
				} 
				else if (u2 < dFloat64(0.0f)) 
				{
					return dPointToTriangleDistance(point, p0, p1, p3);
				} 
				else if (u1 < dFloat64(0.0f)) 
				{
					return dPointToTriangleDistance(point, p0, p2, p3);
				} 
				else if (u1 + u2 + u3 > dFloat64(1.0f)) 
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

dFloat32 dRayCastSphere (const dVector& p0, const dVector& p1, const dVector& origin, dFloat32 radius)
{
	dVector p0Origin (p0 - origin);
	dAssert (p0Origin.m_w == dFloat32 (0.0f));
	if (p0Origin.DotProduct(p0Origin).GetScalar() < (dFloat32 (100.0f) * radius * radius)) 
	{
		dVector dp (p1 - p0);
		dAssert (dp.m_w == dFloat32 (0.0f));
		dFloat32 a = dp.DotProduct(dp).GetScalar();
		dFloat32 b = dFloat32 (2.0f) * p0Origin.DotProduct(dp).GetScalar();
		dFloat32 c = p0Origin.DotProduct(p0Origin).GetScalar() - radius * radius;
		dFloat32 desc = b * b - dFloat32 (4.0f) * a * c;
		if (desc >= 0.0f) 
		{
			desc = dSqrt (desc);
			dFloat32 den = dFloat32 (0.5f) / a;
			dFloat32 t0 = (-b + desc) * den;
			dFloat32 t1 = (-b - desc) * den;
			if ((t0 >= dFloat32 (0.0f)) && (t1 >= dFloat32 (0.0f))) 
			{
				t0 =  dMin(t0, t1);
				if (t0 <= dFloat32 (1.0f)) 
				{
					return t0;
				}
			} 
			else if (t0 >= dFloat32 (0.0f)) 
			{
				if (t0 <= dFloat32 (1.0f)) 
				{
					return t0;
				}
			} 
			else 
			{
				if ((t1 >= dFloat32 (0.0f)) && (t1 <= dFloat32 (1.0f))) 
				{
					return t1;
				}
			}
		}
	} 
	else 
	{
		dBigVector p0Origin1 (p0Origin);
dBigVector dp(p1 - p0);
dAssert(dp.m_w == dFloat32(0.0f));
dFloat64 a = dp.DotProduct(dp).GetScalar();
dFloat64 b = dFloat32(2.0f) * p0Origin1.DotProduct(dp).GetScalar();
dFloat64 c = p0Origin1.DotProduct(p0Origin1).GetScalar() - dFloat64(radius) * radius;
dFloat64 desc = b * b - dFloat32(4.0f) * a * c;
if (desc >= 0.0f)
{
	desc = sqrt(desc);
	dFloat64 den = dFloat32(0.5f) / a;
	dFloat64 t0 = (-b + desc) * den;
	dFloat64 t1 = (-b - desc) * den;
	if ((t0 >= dFloat32(0.0f)) && (t1 >= dFloat32(0.0f)))
	{
		t0 = dMin(t0, t1);
		if (t0 <= dFloat32(1.0f))
		{
			return dFloat32(t0);
		}
	}
	else if (t0 >= dFloat32(0.0f))
	{
		if (t0 <= dFloat32(1.0f))
		{
			return dFloat32(t0);
		}
	}
	else
	{
		if ((t1 >= dFloat32(0.0f)) && (t1 <= dFloat32(1.0f)))
		{
			return dFloat32(t1);
		}
	}
}
	}
	return dFloat32(1.2f);
}

dFloat32 dRayCastBox(const dVector& p0, const dVector& p1, const dVector& boxP0, const dVector& boxP1, dVector& normalOut)
{
	dInt32 index = 0;
	dFloat32 signDir = dFloat32(0.0f);
	dFloat32 tmin = dFloat32(0.0f);
	dFloat32 tmax = dFloat32(1.0f);

	for (dInt32 i = 0; i < 3; i++)
	{
		dFloat32 dp = p1[i] - p0[i];
		if (dAbs(dp) < dFloat32(1.0e-8f))
		{
			if (p0[i] <= boxP0[i] || p0[i] >= boxP1[i])
			{
				return dFloat32(1.2f);
			}
		}
		else
		{
			dp = dFloat32(1.0f) / dp;
			dFloat32 t1 = (boxP0[i] - p0[i]) * dp;
			dFloat32 t2 = (boxP1[i] - p0[i]) * dp;

			dFloat32 sign = dFloat32(-1.0f);
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
				return dFloat32(1.2f);
			}
		}
	}

	if (tmin > dFloat32(0.0f))
	{
		dAssert(tmin < 1.0f);
		normalOut = dVector(dFloat32(0.0f));
		normalOut[index] = signDir;
	}
	else
	{
		tmin = dFloat32(1.2f);
	}
	return tmin;
}
