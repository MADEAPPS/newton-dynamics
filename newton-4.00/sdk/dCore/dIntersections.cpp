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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dMemory.h"
#include "dVector.h"
#include "dPlane.h"
#include "dGoogol.h"
#include "dIntersections.h"

#define D_USE_FLOAT_VERSION
#define D_RAY_TOL_ERROR (dFloat32 (-1.0e-3f))
#define D_RAY_TOL_ADAPTIVE_ERROR (dFloat32 (1.0e-1f))

void dFastAabbInfo::MakeBox1(dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const
{
	dVector faceBoxP0(&vertexArray[indexArray[0] * stride]);
	faceBoxP0 = faceBoxP0 & dVector::m_triplexMask;
	dVector faceBoxP1(faceBoxP0);
	for (dInt32 i = 1; i < indexCount; i++)
	{
		dVector p(&vertexArray[indexArray[i] * stride]);
		p = p & dVector::m_triplexMask;
		faceBoxP0 = faceBoxP0.GetMin(p);
		faceBoxP1 = faceBoxP1.GetMax(p);
	}

	minBox = faceBoxP0 - m_p1;
	maxBox = faceBoxP1 - m_p0;
}

void dFastAabbInfo::MakeBox2(const dMatrix& faceMatrix, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const
{
	dVector faceBoxP0(faceMatrix.TransformVector(dVector(&vertexArray[indexArray[0] * stride]) & dVector::m_triplexMask));
	dVector faceBoxP1(faceBoxP0);
	for (dInt32 i = 1; i < indexCount; i++)
	{
		dVector p(faceMatrix.TransformVector(dVector(&vertexArray[indexArray[i] * stride]) & dVector::m_triplexMask));
		faceBoxP0 = faceBoxP0.GetMin(p);
		faceBoxP1 = faceBoxP1.GetMax(p);
	}
	faceBoxP0 = faceBoxP0 & dVector::m_triplexMask;
	faceBoxP1 = faceBoxP1 & dVector::m_triplexMask;

	dMatrix matrix = *this * faceMatrix;
	dVector size(matrix[0].Abs().Scale(m_size.m_x) + matrix[1].Abs().Scale(m_size.m_y) + matrix[2].Abs().Scale(m_size.m_z));
	dVector boxP0((matrix.m_posit - size) & dVector::m_triplexMask);
	dVector boxP1((matrix.m_posit + size) & dVector::m_triplexMask);

	minBox = faceBoxP0 - boxP1;
	maxBox = faceBoxP1 - boxP0;
}

dMatrix dFastAabbInfo::MakeFaceMatrix(const dVector& faceNormal, dInt32, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const
{
	dMatrix faceMatrix;
	dVector origin(&vertexArray[indexArray[0] * stride]);
	dVector pin(&vertexArray[indexArray[0] * stride]);
	pin = pin & dVector::m_triplexMask;
	origin = origin & dVector::m_triplexMask;

	dVector pin1(&vertexArray[indexArray[1] * stride]);
	pin1 = pin1 & dVector::m_triplexMask;

	faceMatrix[0] = faceNormal;
	faceMatrix[1] = pin1 - origin;
	faceMatrix[1] = faceMatrix[1].Normalize();
	faceMatrix[2] = faceMatrix[0].CrossProduct(faceMatrix[1]);
	faceMatrix[3] = origin | dVector::m_wOne;
	return faceMatrix.Inverse();
}

dFloat32 dFastAabbInfo::PolygonBoxRayDistance(const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, const dFastRayTest& ray) const
{
	dVector minBox;
	dVector maxBox;
	MakeBox1(indexCount, indexArray, stride, vertexArray, minBox, maxBox);
	dFloat32 dist0 = ray.BoxIntersect(minBox, maxBox);
	if (dist0 < dFloat32(1.0f))
	{
		dMatrix faceMatrix(MakeFaceMatrix(faceNormal, indexCount, indexArray, stride, vertexArray));

		MakeBox2(faceMatrix, indexCount, indexArray, stride, vertexArray, minBox, maxBox);
		dVector veloc(faceMatrix.RotateVector(ray.m_diff) & dVector::m_triplexMask);
		dFastRayTest localRay(dVector(dFloat32(0.0f)), veloc);
		dFloat32 dist1 = localRay.BoxIntersect(minBox, maxBox);
		dist0 = dMax(dist1, dist0);
	}
	return dist0;
}

dFloat32 dFastAabbInfo::PolygonBoxDistance(const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const
{
	dVector minBox;
	dVector maxBox;
	MakeBox1(indexCount, indexArray, stride, vertexArray, minBox, maxBox);

	dVector mask(minBox * maxBox < dVector(dFloat32(0.0f)));
	dVector dist(maxBox.GetMin(minBox.Abs()) & mask);
	dist = dist.GetMin(dist.ShiftTripleRight());
	dist = dist.GetMin(dist.ShiftTripleRight());
	dFloat32 dist0 = dist.GetScalar();
	if (dist0 > dFloat32(0.0f))
	{
		dMatrix faceMatrix(MakeFaceMatrix(faceNormal, indexCount, indexArray, stride, vertexArray));
		MakeBox2(faceMatrix, indexCount, indexArray, stride, vertexArray, minBox, maxBox);

		dVector mask2(minBox * maxBox < dVector(dFloat32(0.0f)));
		dVector dist2(maxBox.GetMin(minBox.Abs()) & mask2);
		dist2 = dist2.GetMin(dist2.ShiftTripleRight());
		dist2 = dist2.GetMin(dist2.ShiftTripleRight());
		dFloat32 dist1 = dist2.GetScalar();
		dist0 = (dist1 > dFloat32(0.0f)) ? dMax(dist0, dist1) : dFloat32(0.0f);
		if (dist0 <= dFloat32(0.0f))
		{
			// some how clang crashes in relese and release with debug, 
			// but not Visual studio, Intel or Gcc
			// I do can't determine why because Clang inline teh code without debug 
			//dVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask2));
			const dVector box0(minBox.Abs());
			const dVector box1(maxBox.Abs());
			const dVector p1p0((box0.GetMin(box1)).AndNot(mask2));
			dist2 = p1p0.DotProduct(p1p0);
			dist2 = dist2.Sqrt() * dVector::m_negOne;
			dist0 = dist2.GetScalar();
		}
	}
	else
	{
		//dVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask));
		const dVector box0(minBox.Abs());
		const dVector box1(maxBox.Abs());
		const dVector p1p0(box0.GetMin(box1).AndNot(mask));
		dist = p1p0.DotProduct(p1p0);
		dist = dist.Sqrt() * dVector::m_negOne;
		dist0 = dist.GetScalar();
	}
	return	dist0;
}

dFloat32 dFastRayTest::PolygonIntersect (const dVector& faceNormal, dFloat32 maxT, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount) const
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

//void dRayToRayDistance(const dVector& ray_p0, const dVector& ray_p1, const dVector& ray_q0, const dVector& ray_q1, dVector& pOut, dVector& qOut)
dFastRayTest dFastRayTest::RayDistance(const dVector& ray_q0, const dVector& ray_q1) const
{
	//dVector u(ray_p1 - ray_p0);
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
	//pOut = ray_p0 + u.Scale(sc);
	//qOut = ray_q0 + v.Scale(tc);
	return dFastRayTest(m_p0 + u.Scale(sc), ray_q0 + v.Scale(tc));
}

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

dBigVector dPointToRayDistance (const dBigVector& point, const dBigVector& ray_p0, const dBigVector& ray_p1)
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
		dBigVector dp (p1 - p0);
		dAssert (dp.m_w == dFloat32 (0.0f));
		dFloat64 a = dp.DotProduct(dp).GetScalar();
		dFloat64 b = dFloat32 (2.0f) * p0Origin1.DotProduct(dp).GetScalar();
		dFloat64 c = p0Origin1.DotProduct(p0Origin1).GetScalar() - dFloat64(radius) * radius;
		dFloat64 desc = b * b - dFloat32 (4.0f) * a * c;
		if (desc >= 0.0f) 
		{
			desc = sqrt (desc);
			dFloat64 den = dFloat32 (0.5f) / a;
			dFloat64 t0 = (-b + desc) * den;
			dFloat64 t1 = (-b - desc) * den;
			if ((t0 >= dFloat32 (0.0f)) && (t1 >= dFloat32 (0.0f))) 
			{
				t0 =  dMin(t0, t1);
				if (t0 <= dFloat32 (1.0f)) 
				{
					return dFloat32 (t0);
				}
			} 
			else if (t0 >= dFloat32 (0.0f)) 
			{
				if (t0 <= dFloat32 (1.0f)) 
				{
					return dFloat32 (t0);
				}
			} 
			else 
			{
				if ((t1 >= dFloat32 (0.0f)) && (t1 <= dFloat32 (1.0f))) 
				{
					return dFloat32 (t1);
				}
			}
		}
	}
	return dFloat32 (1.2f);
}

dFloat32 dRayCastBox (const dVector& p0, const dVector& p1, const dVector& boxP0, const dVector& boxP1, dVector& normalOut)
{
	dInt32 index = 0;
	dFloat32 signDir = dFloat32 (0.0f);
	dFloat32 tmin = dFloat32 (0.0f);
	dFloat32 tmax = dFloat32 (1.0f);

	//dVector size (boxP1 - boxP0);
	for (dInt32 i = 0; i < 3; i++) 
	{
		dFloat32 dp = p1[i] - p0[i];
		if (dAbs (dp) < dFloat32 (1.0e-8f)) 
		{
			if (p0[i] <= boxP0[i] || p0[i] >= boxP1[i]) 
			{
				return dFloat32 (1.2f);
			}
		} 
		else 
		{
			dp = dFloat32 (1.0f) / dp; 
			dFloat32 t1 = (boxP0[i] - p0[i]) * dp;
			dFloat32 t2 = (boxP1[i] - p0[i]) * dp;

			dFloat32 sign = dFloat32 (-1.0f);
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
				return dFloat32 (1.2f);
			}
		}
	}

	if (tmin > dFloat32 (0.0f)) 
	{
		dAssert (tmin < 1.0f);
		normalOut = dVector (dFloat32 (0.0f));
		normalOut[index] = signDir;
	} 
	else 
	{
		tmin = dFloat32 (1.2f);
	}
	return tmin;
}
