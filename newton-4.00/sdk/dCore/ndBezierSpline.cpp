/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMemory.h"
#include "ndArray.h"
#include "ndVector.h"
#include "ndBezierSpline.h"

ndBezierSpline::ndBezierSpline ()
	:ndClassAlloc()
	,m_knotVector()
	,m_controlPoints()
	,m_degree(0)
	,m_knotsCount(0)
	,m_controlPointsCount(0)
{
}

ndBezierSpline::ndBezierSpline (const ndBezierSpline& src)
	:ndClassAlloc()
	,m_knotVector()
	,m_controlPoints()
	,m_degree(0)
	,m_knotsCount(0)
	,m_controlPointsCount(0)
{
	if (src.m_knotsCount) 
	{
		CreateFromKnotVectorAndControlPoints (src.m_degree, src.m_knotsCount - 2 * src.m_degree, &src.m_knotVector[src.m_degree], &src.m_controlPoints[0]);
	}
}

ndBezierSpline::~ndBezierSpline ()
{
	Clear();
}

void ndBezierSpline::Clear()
{
	m_degree = 0;
	m_knotsCount = 0;
	m_controlPointsCount = 0;
}

ndBezierSpline& ndBezierSpline::operator = (const ndBezierSpline &copy)
{
	Clear();
	if (copy.m_knotsCount) 
	{
		CreateFromKnotVectorAndControlPoints (copy.m_degree, copy.m_knotsCount - 2 * copy.m_degree, &copy.m_knotVector[copy.m_degree], &copy.m_controlPoints[0]);
	}
	return *this;
}

ndInt32 ndBezierSpline::GetDegree () const
{
	return m_degree;
}

void ndBezierSpline::CreateFromKnotVectorAndControlPoints (ndInt32 degree, ndInt32 knotCount, const ndFloat64* const knotVector, const ndBigVector* const controlPoints)
{
	Clear();
	dAssert (knotCount);
	dAssert (knotVector[0] == ndFloat32 (0.0f));
	dAssert (knotVector[knotCount - 1] == ndFloat32 (1.0f));

	m_degree = degree;
	m_knotsCount = knotCount + 2 * degree;
	m_controlPointsCount = knotCount + m_degree - 1;
	m_knotVector.SetCount(m_knotsCount);
	m_controlPoints.SetCount(m_controlPointsCount);

	for (ndInt32 i = 0; i < m_controlPointsCount; i++) 
	{
		m_controlPoints[i] = controlPoints[i];
	}
	
	for (ndInt32 i = 0; i < m_degree; i ++) 
	{
		m_knotVector[i] = ndFloat32 (0.0f);
		m_knotVector[i + m_knotsCount - m_degree] = ndFloat64(ndFloat32 (1.0f));
	}

	for (ndInt32 i = 0; i < knotCount; i ++) 
	{
		m_knotVector[i + m_degree] = knotVector[i];
		dAssert (m_knotVector[i + m_degree] >= m_knotVector[i + m_degree - 1]);
	}
}

ndInt32 ndBezierSpline::GetKnotCount() const
{
	return m_knotsCount;
}

ndArray<ndFloat64>& ndBezierSpline::GetKnotArray()
{
	return m_knotVector;
}

const ndArray<ndFloat64>& ndBezierSpline::GetKnotArray() const
{
	return m_knotVector;
}

ndFloat64 ndBezierSpline::GetKnot(ndInt32 i) const
{
	return m_knotVector[i];
}

ndInt32 ndBezierSpline::GetControlPointCount() const
{
	return m_controlPointsCount;
}

ndBigVector ndBezierSpline::GetControlPoint(ndInt32 i) const
{
	return m_controlPoints[i];
}

void ndBezierSpline::SetControlPoint(ndInt32 i, const ndBigVector& point)
{
	m_controlPoints[i] = point;
}

ndArray<ndBigVector>& ndBezierSpline::GetControlPointArray()
{
	return m_controlPoints;
}

const ndArray<ndBigVector>& ndBezierSpline::GetControlPointArray() const
{
	return m_controlPoints;
}

ndInt32 ndBezierSpline::GetSpan(ndFloat64 u) const
{
	ndInt32 low = m_degree;
	ndInt32 high = m_knotsCount - m_degree - 1;

	dAssert (u >= ndFloat32 (0.0f));
	dAssert (u <= ndFloat32 (1.0f));
	while ((high - low) >= 4) 
	{
		ndInt32 mid = (low + high) >> 1;
		if (u > m_knotVector[mid]) 
		{
			low = mid;
		} 
		else 
		{
			high = mid;
		}
	}

	dAssert (m_knotVector[low] <= u);
	for (ndInt32 i = low; i < m_degree + m_knotsCount + 1; i ++) 
	{
		if (m_knotVector[i + 1] >= u) 
		{
			return i;
		}
	}
	dAssert (0);
	return 0;
}

void ndBezierSpline::BasicsFunctions (ndFloat64 u, ndInt32 span, ndFloat64* const BasicFunctionsOut) const
{
	BasicFunctionsOut[0] = ndFloat32 (1.0f);

	ndFloat64* const left = dAlloca(ndFloat64, m_knotsCount + 32);
	ndFloat64* const right = dAlloca(ndFloat64, m_knotsCount + 32);

	for (ndInt32 j = 1; j <= m_degree; j ++) 
	{
		left[j] = u - m_knotVector[span + 1 - j]; 
		right[j] = m_knotVector[span + j] - u;

		ndFloat64 saved = ndFloat32 (0.0f);
		for (ndInt32 r = 0; r < j; r ++) 
		{
			ndFloat64 temp = BasicFunctionsOut[r] / (right[r + 1] + left[j - r]);
			BasicFunctionsOut[r] = saved + temp * right[r + 1];
			saved = temp * left[j - r];
		}
		BasicFunctionsOut[j] = saved;
	}
}

void ndBezierSpline::BasicsFunctionsDerivatives (ndFloat64 u, ndInt32 span, ndFloat64* const derivativesOut) const
{
	ndFloat64* const a = dAlloca(ndFloat64, m_knotsCount + 32);
	ndFloat64* const ndu = dAlloca(ndFloat64, m_knotsCount + 32);
	ndFloat64* const left = dAlloca(ndFloat64, m_knotsCount + 32);
	ndFloat64* const right = dAlloca(ndFloat64, m_knotsCount + 32);

	const ndInt32 width = m_degree + 1;
	ndu[0] = ndFloat32 (1.0f);
	for (ndInt32 j = 1; j <= m_degree; j ++) 
	{
		left[j] = u - m_knotVector[span + 1 - j];
		right[j] = m_knotVector[span + j] - u;
		ndFloat64 saved = ndFloat32 (0.0f);
		for (ndInt32 r = 0; r < j; r ++) 
		{
			ndu[j * width + r] = right[r + 1] + left[j - r];

			ndFloat64 temp = ndu[r * width + j - 1] / ndu[j * width + r];
			ndu[r * width + j] = saved + temp * right[r + 1];
			saved = temp * left[j - r];
		}
		ndu[j * width + j] = saved;
	}


	for (ndInt32 j = 0; j <= m_degree; j ++) 
	{
		derivativesOut[width * 0 + j] = ndu [width * j + m_degree];
	}

	for (ndInt32 r = 0; r <= m_degree; r ++) 
	{
		ndInt32 s1 = 0;
		ndInt32 s2 = 1;
		a[0] = ndFloat32 (1.0f);
		for (ndInt32 k = 1; k <= m_degree; k ++) 
		{
			ndFloat64 d = ndFloat32 (0.0f);
			ndInt32 rk = r - k;
			ndInt32 pk = m_degree - k;
			if (r >= k)  
			{
				a[width * s2 + 0] = a[width * s1 + 0] / ndu[width * (pk + 1) + rk];
				d = a[width * s2 + 0] * ndu[width * rk + pk];
			}
			ndInt32 j1 = 0;
			ndInt32 j2 = 0;
			if (rk >= -1) 
			{
				j1 = 1;
			} 
			else 
			{
				j1 = -rk;
			}

			if ((r - 1) <= pk) 
			{
				j2 = k-1;
			} 
			else 
			{
				j2 = m_degree-r;
			}
			for (ndInt32 j = j1; j <= j2; j ++) 
			{
				a[width * s2 + j] = (a[width * s1 + j] - a[width * s1 + j - 1]) / ndu[width * (pk + 1) + rk + j];
				d += a[width * s2 + j] * ndu[width * (rk + j) + pk];
			}
			if (r <= pk) 
			{
				a[width * s2 + k] = -a[width * s1 + k - 1] / ndu[width * (pk + 1) + r];
				d += a[width * s2 + k] * ndu[width * r + pk];
			}
			derivativesOut[width * k + r] = d;
			dSwap(s1, s2);
		}
	}

	ndInt32 s = m_degree;
	for (ndInt32 k = 1; k <= m_degree; k ++) 
	{
		for (ndInt32 j = 0; j <= m_degree; j ++) 
		{
			derivativesOut[width * k + j] *= s;
		}
		s *= (m_degree - k);
	}
}

ndBigVector ndBezierSpline::CurvePoint (ndFloat64 u, ndInt32 span) const
{
	ndBigVector point (ndFloat32 (0.0f));
	ndFloat64* const basicFunctions = dAlloca(ndFloat64, m_knotsCount + 32);
	BasicsFunctions (u, span, basicFunctions);
	for (ndInt32 i = 0; i <= m_degree; i ++) 
	{
		point += m_controlPoints[span - m_degree + i].Scale (basicFunctions[i]);
	}
	point.m_w = ndFloat32(1.0f);
	return point;
}

ndBigVector ndBezierSpline::CurvePoint (ndFloat64 u) const
{
	u = dClamp (u, ndFloat64 (ndFloat32 (0.0f)), ndFloat64 (ndFloat32 (1.0f)));
	ndInt32 span = GetSpan(u);
	return CurvePoint (u, span);
}

ndBigVector ndBezierSpline::CurveDerivative (ndFloat64 u, ndInt32 index) const
{
	u = dClamp (u, ndFloat64 (ndFloat32 (0.0f)), ndFloat64 (ndFloat32 (1.0f)));
	dAssert (index <= m_degree);
	
	ndFloat64* const basicsFuncDerivatives = dAlloca(ndFloat64, m_knotsCount + 32);
	ndInt32 span = GetSpan(u);
	BasicsFunctionsDerivatives (u, span, basicsFuncDerivatives);

	const ndInt32 with = m_degree + 1;
	ndBigVector point (ndFloat32 (0.0f));
	for (ndInt32 i = 0; i <= m_degree; i ++) 
	{
		point += m_controlPoints[span - m_degree + i].Scale (basicsFuncDerivatives[with * index + i]);
	}
	return point;
}

ndInt32 ndBezierSpline::CurveAllDerivatives (ndFloat64 u, ndBigVector* const derivatives) const
{
	u = dMod (u, ndFloat64(ndFloat32 (1.0f)));
	ndFloat64* const basicsFuncDerivatives = dAlloca(ndFloat64, m_knotsCount + 32);
	ndInt32 span = GetSpan(u);
	BasicsFunctionsDerivatives (u, span, basicsFuncDerivatives);

	const ndInt32 with = m_degree + 1;
	for (ndInt32 j = 0; j <= m_degree; j ++) 
	{
		ndBigVector ck (ndBigVector::m_zero);
		for (ndInt32 i = 0; i <= m_degree; i ++) 
		{
			ck += m_controlPoints[span - m_degree + i].Scale (basicsFuncDerivatives[with * j + i]);
		}
		ck.m_w = ndFloat32(0.0f);
		derivatives[j] = ck;
	}

	return m_degree + 1;
}

void ndBezierSpline::GlobalCubicInterpolation (ndInt32 count, const ndBigVector* const points, const ndBigVector& firstTangent, const ndBigVector& lastTangent)
{
	CreateCubicKnotVector (count, points);
	CreateCubicControlPoints (count, points, firstTangent, lastTangent);
}

void ndBezierSpline::CreateCubicKnotVector(ndInt32 count, const ndBigVector* const points)
{
	dAssert (count >= 2);

	ndFloat64* const u = dAlloca(ndFloat64, m_knotsCount + 32);
#if 0
	u[0] = dFloat32 (0.0f);
	dFloat64 d = dFloat32(0.0f);
	for (dInt32 i = 1; i < count; i ++) 
	{
		dBigVector step (points[i] - points[i - 1]);
		dAssert(step.m_w == dFloat32 (0.0f));
		dFloat64 len = dSqrt (step.DotProduct(step).GetScalar());
		u[i] = dSqrt (len); 
		d += u[i];
	}
	
	for (dInt32 i = 1; i < count; i ++) 
	{
		u[i] = u[i-1] + u[i] / d;
	}

#else

	ndFloat64 d = ndFloat32(0.0f);
	for (ndInt32 i = 1; i < count; i++)
	{
		ndBigVector step(points[i] - points[i - 1]);
		dAssert(step.m_w == ndFloat32(0.0f));
		d += ndSqrt(step.DotProduct(step).GetScalar());
		u[i] = d;
	}

	d = ndFloat32(1.0f) / d;
	for (ndInt32 i = 0; i < count; i++)
	{
		u[i] *= d;
	}
#endif

	u[0] = ndFloat64 (ndFloat32 (0.0f));
	u[count - 1] = ndFloat64(ndFloat32 (1.0f));

	m_degree = 3;
	m_knotsCount = count + 2 * m_degree;

	m_knotVector.SetCount(m_knotsCount);
	for (ndInt32 i = 0; i < (m_degree + 1); i ++) 
	{
		m_knotVector[i] = ndFloat32 (0.0f);
		m_knotVector[i + m_knotsCount - m_degree - 1] = ndFloat64(ndFloat32 (1.0f));
	}

	for (ndInt32 i = 1; i < (count - 1); i ++) 
	{
		ndFloat64 acc = ndFloat64 (ndFloat32 (0.0f));
		for (ndInt32 j = 0; j < m_degree; j ++) 
		{
			acc += u[j + i - 1];
		}
		m_knotVector[m_degree + i] = acc / ndFloat64 (3.0f);
	}
}

void ndBezierSpline::CreateCubicControlPoints(ndInt32 count, const ndBigVector* const points, const ndBigVector& firstTangent, const ndBigVector& lastTangent)
{
	ndFloat64 abc[4];
	if ((m_knotsCount - 2 * (m_degree - 1)) != m_controlPointsCount) 
	{
		m_controlPointsCount = m_knotsCount - 2 * (m_degree - 1);
	}

	m_controlPoints.SetCount(m_controlPointsCount);
	m_controlPoints[0] = points[0];
	m_controlPoints[m_controlPointsCount - 1] = points[count - 1];

	m_controlPoints[1] = m_controlPoints[0] + firstTangent.Scale (m_knotVector[m_degree + 1] / 3.0f);
	m_controlPoints[m_controlPointsCount - 2] = m_controlPoints[m_controlPointsCount - 1] - lastTangent.Scale ((ndFloat32 (1.0f) - m_knotVector[m_knotsCount - m_degree - 2]) / 3.0f);
	if (count == 3) 
	{
		BasicsFunctions (m_knotVector[m_degree + 1], m_degree + 1, abc);
		m_controlPoints[2]  = points[1] - m_controlPoints[1].Scale (abc[0]) - m_controlPoints[3].Scale (abc[2]);
		m_controlPoints[2] = m_controlPoints[2].Scale (ndFloat32 (1.0f) / abc[1]);
	} 
	else 
	{
		ndFloat64* const dd = dAlloca(ndFloat64, m_knotsCount + 32);
		BasicsFunctions (m_knotVector[m_degree + 1], m_degree + 1, abc);
		ndFloat64 den = abc[1];
		m_controlPoints[2]  = (points[1] - m_controlPoints[1].Scale (abc[0])).Scale (ndFloat32 (1.0f) / den);
		for (ndInt32 i = 3; i < (count - 1); i ++) 
		{
			dd[i + 1] = abc[2] / den;
			BasicsFunctions (m_knotVector[i + 2], i + 2, abc);
			den = abc[1] - abc[0] * dd[i + 1];
			m_controlPoints[i]  = (points[i - 1] - m_controlPoints[i - 1].Scale (abc[0])).Scale (ndFloat32 (1.0f) / den);
		}

		dd[count] = abc[2] / den;
		BasicsFunctions (m_knotVector[count + 1], count + 1, abc);
		den = abc[1] - abc[0] * dd[count];
		m_controlPoints[count - 1] = (points[count - 2] - m_controlPoints[count].Scale (abc[2]) - m_controlPoints[count - 2].Scale (abc[0])).Scale (ndFloat32 (1.0f) / den);

		for (ndInt32 i = count - 2; i >= 2; i --) 
		{
			m_controlPoints[i] -= m_controlPoints[i + 1].Scale (dd[i + 2]);
		}
	}
}

ndFloat64 ndBezierSpline::CalculateLength (ndFloat64 tol) const
{
	ndBigVector stackPool[32][3];
	ndInt32 stack = 0;

	ndFloat64 length = ndFloat32 (0.0f);
	ndFloat64 tol2 = tol * tol;
	ndFloat64 u0 = m_knotVector[m_degree];
	ndBigVector p0 (CurvePoint (u0));

	for (ndInt32 i = m_degree; i < (m_knotsCount - m_degree - 1); i ++) 
	{
		ndFloat64 u1 = m_knotVector[i + 1];
		ndBigVector p1 (CurvePoint (u1));
		stackPool[stack][0] = p0;
		stackPool[stack][1] = p1;
		stackPool[stack][2] = ndBigVector (u0, u1, ndFloat32 (0.0f), ndFloat32 (0.0f));
		stack ++;
		while (stack) 
		{
			stack --;
			ndBigVector q0 (stackPool[stack][0]);
			ndBigVector q1 (stackPool[stack][1]);
			ndFloat64 t0 = stackPool[stack][2][0];
			ndFloat64 t1 = stackPool[stack][2][1];
			ndFloat64 t01 = (t1 + t0) * 0.5f;

			ndBigVector p01 ((q1 + q0).Scale (0.5f));
			ndBigVector q01 (CurvePoint (t01));
			ndBigVector err (q01 - p01);
			dAssert(err.m_w == ndFloat32 (0.0f));
			ndFloat64 err2 = err.DotProduct(err).GetScalar();
			if (err2 < tol2) 
			{
				ndBigVector step (q1 - q0);
				dAssert(step.m_w == ndFloat32 (0.0f));
				length += ndSqrt (step.DotProduct(step).GetScalar());
			} 
			else 
			{
				stackPool[stack][0] = q01;
				stackPool[stack][1] = q1;
				stackPool[stack][2] = ndBigVector (t01, t1, ndFloat32 (0.0f), ndFloat32 (0.0f));
				stack ++;

				stackPool[stack][0] = q0;
				stackPool[stack][1] = q01;
				stackPool[stack][2] = ndBigVector (t0, t01, ndFloat32 (0.0f), ndFloat32 (0.0f));
				stack ++;
			}
		}
		u0 = u1;
		p0 = p1;
	}

	return length;
}

void ndBezierSpline::InsertKnot (ndFloat64 u)
{
	const ndInt32 k = GetSpan(u);
	ndInt32 multiplicity = 0;
	for (ndInt32 i = 0; i < m_degree; i ++) 
	{
		multiplicity += (dAbs (m_knotVector[k + i + 1] - u) < ndFloat64 (1.0e-5f)) ? 1 : 0;
	}
	if (multiplicity == m_degree) 
	{
		return;
	}

	m_knotVector.SetCount(m_knotsCount + 1);
	for (ndInt32 i = m_knotsCount; i > (k + 1); i --) 
	{
		m_knotVector[i] = m_knotVector[i - 1];
	}
	m_knotVector[k + 1] = u;

	ndBigVector Rw[16];
	for (ndInt32 i = 0; i <= m_degree; i ++) 
	{
		Rw[i] = m_controlPoints[k - m_degree + i];
	}

	const ndInt32 m = k - m_degree + 1;
	dAssert(m >= 0);
	dAssert((k + 1 - 1 - 0) >= 0);
	dAssert((m_degree - 1 - 0) >= 0);

	for (ndInt32 i = 0; i <= (m_degree - 1); i ++) 
	{
		ndFloat64 alpha = (u  - m_knotVector[m + i]) / (m_knotVector[i + k + 1] - m_knotVector[m + i]);
		Rw[i] = Rw[i + 1].Scale (alpha) + Rw[i].Scale (ndFloat64 (ndFloat32 (1.0f)) - alpha);
	}

	m_controlPoints.SetCount(m_controlPointsCount + 1);
	for (ndInt32 i = m_controlPointsCount; i > k; i--) 
	{
		m_controlPoints[i] = m_controlPoints[i - 1];
	}

	m_controlPoints[m] = Rw[0];
	m_controlPoints[k + 1 - 1 - 0] = Rw[m_degree - 1 - 0];
	for (ndInt32 i = m + 1; i < k; i++) 
	{
		dAssert((i - m) >= 0);
		m_controlPoints[i] = Rw[i - m];
	}

	m_knotsCount ++;
	m_controlPointsCount ++;
}

bool ndBezierSpline::RemoveKnot (ndFloat64 u, ndFloat64 tol)
{
	ndInt32 r = GetSpan(u) + 1;
	dAssert (m_knotVector[r - 1] < u);
	if (dAbs (m_knotVector[r] - u) > 1.0e-5f) 
	{
		return false;
	}

	ndInt32 s = 1;
	ndInt32 last = r - s;
	ndInt32 first = r - m_degree;
	ndInt32 ord = m_degree + 1;
	ndBigVector temp[16];

	bool removableFlag = false;
	ndInt32 t = 0;
	for ( ; t < m_degree; t ++) 
	{
		ndInt32 off = first - 1;
		temp[0] = m_controlPoints[off];
		temp[last + 1 - off] = m_controlPoints[last + 1];
		ndInt32 i = first;
		ndInt32 j = last;
		ndInt32 ii = 1;
		ndInt32 jj = last - off;

		while ((j - i) > t) 
		{
			ndFloat64 alpha_i = (u - m_knotVector[i]) / (m_knotVector[i + ord + t] - m_knotVector[i]);
			ndFloat64 alpha_j = (u - m_knotVector[j - t]) / (m_knotVector[j + ord] - m_knotVector[j - t]);
			temp[ii] = (m_controlPoints[i] - temp[ii - 1].Scale (ndFloat64 (ndFloat32 (1.0f)) - alpha_i)).Scale (ndFloat64 (ndFloat32 (1.0f)) / alpha_i);
			temp[jj] = (m_controlPoints[j] - temp[jj + 1].Scale (alpha_j)).Scale (ndFloat64 (ndFloat32 (1.0f)) / (ndFloat64 (ndFloat32 (1.0f)) - alpha_j));
			i ++;
			j --;
			ii ++;
			jj --;
		}
		if ((j - i) < t) 
		{
			ndBigVector diff (temp[ii - 1] - temp[jj + 1]);
			dAssert(diff.m_w == ndFloat32 (0.0f));
			removableFlag = diff.DotProduct(diff).GetScalar() < (tol * tol);
		} 
		else 
		{
			ndFloat64 alpha_i = (u - m_knotVector[i]) / (m_knotVector[i + ord + t] - m_knotVector[i]);
			ndBigVector p (temp[ii + t + 1].Scale (alpha_i) + temp[ii - 1].Scale (ndFloat64 (ndFloat32 (1.0f)) - alpha_i));
			ndBigVector diff (m_controlPoints[i] - p);
			dAssert(diff.m_w == ndFloat32 (0.0f));
			removableFlag = diff.DotProduct(diff).GetScalar() < (tol * tol);
		}
		if (!removableFlag) 
		{
			break;
		}

		i = first;
		j = last;
		while ((j - 1) > t) 
		{
			m_controlPoints[i] = temp[i - off];
			m_controlPoints[j] = temp[j - off];
			i ++;
			j --;
		}
		first --;
		last ++;
	}

	if (t) 
	{
		for (ndInt32 k = r + t; k < m_knotsCount; k ++) 
		{
			m_knotVector[k - t] = m_knotVector[k];
		}

		ndInt32 fOut = (2 * r - s - m_degree) / 2;
		ndInt32 j = fOut;
		ndInt32 i = j;
		for (ndInt32 k = 1; k < t; k ++) 
		{
			if ((k % 2) == 1) 
			{
				i ++;
			} 
			else 
			{
				j = j - 1;
			}
		}

		for (ndInt32 k = i + 1; k < m_controlPointsCount; k ++) 
		{
			m_controlPoints[j] = m_controlPoints[k];
			j ++;
		}

		m_knotsCount -= t;
		m_controlPointsCount -= t;
	}

	return removableFlag;
}

ndFloat64 ndBezierSpline::FindClosestKnot(ndBigVector& closestPoint, const ndBigVector& point, ndInt32 subdivitionSteps) const
{
	ndInt32 startSpan = m_degree;
	ndFloat64 bestU = ndFloat32 (0.0f);
	ndFloat64 distance2 = ndFloat32 (1.0e10f);
	ndBigVector closestControlPoint(m_controlPoints[0]);
	subdivitionSteps = dMax(subdivitionSteps, 1);
	ndFloat64 scale = ndFloat32 (1.0f) / subdivitionSteps;
	for (ndInt32 span = m_degree; span < (m_knotsCount - m_degree - 1); span++) 
	{
		ndFloat64 param = ndFloat32 (0.0f);
		for (ndInt32 i = 0; i < subdivitionSteps; i++) 
		{
			ndFloat64 u = m_knotVector[span] + (m_knotVector[span + 1] - m_knotVector[span]) * param;
			param += scale;
			ndBigVector p(CurvePoint(u, span));
			ndBigVector dp(p - point);
			dAssert(dp.m_w == ndFloat32 (0.0f));
			ndFloat64 dist2 = dp.DotProduct(dp).GetScalar();
			if (dist2 < distance2) 
			{
				bestU = u;
				startSpan = span;
				distance2 = dist2;
				closestControlPoint = p;
			}
		}
	}

	ndBigVector p(CurvePoint(ndFloat32 (0.999f)));
	ndBigVector dp(p - point);
	dAssert(dp.m_w == ndFloat32 (0.0f));
	ndFloat64 dist2 = dp.DotProduct(dp).GetScalar();
	if (dist2 < distance2) 
	{
		bestU = ndFloat64(0.999f);
		startSpan = m_knotsCount - m_degree - 2;
		closestControlPoint = p;
	}

	ndBigVector derivatives[32];
	ndFloat64 u0 = bestU;

	bool stop = false;
	for (ndInt32 i = 0; (i < 20) && !stop; i++) 
	{
		CurveAllDerivatives(u0, derivatives);

		ndBigVector dist(closestControlPoint - point);
		dAssert(dist.m_w == ndFloat32 (0.0f));
		dAssert(derivatives[1].m_w == ndFloat32 (0.0f));
		dAssert(derivatives[2].m_w == ndFloat32 (0.0f));
		ndFloat64 num = derivatives[1].DotProduct(dist).GetScalar();
		ndFloat64 den = derivatives[2].DotProduct(dist).GetScalar() + derivatives[1].DotProduct(derivatives[1]).GetScalar();
		ndFloat64 u1 = dClamp(u0 - num / den, ndFloat64(0.0), ndFloat64(1.0));
		if (u1 < m_knotVector[startSpan]) 
		{
			startSpan--;
			dAssert(startSpan >= 0);
		} 
		else if (u1 >= m_knotVector[startSpan + 1]) 
		{
			startSpan++;
			dAssert(startSpan < (m_knotsCount - m_degree));
		}

		dAssert(startSpan >= m_degree);
		dAssert(startSpan <= (m_knotsCount - m_degree - 1));
		closestControlPoint = CurvePoint(u1, startSpan);

		stop |= (dAbs(u1 - u0) < ndFloat64(1.0e-10)) || (num * num < ((dist.DotProduct(dist).GetScalar()) * (derivatives[1].DotProduct(derivatives[1]).GetScalar()) * ndFloat64(1.0e-10)));
		u0 = u1;
	}

	closestPoint = closestControlPoint;
	return u0;
}
