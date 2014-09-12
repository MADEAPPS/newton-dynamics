/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dAnimationStdAfx.h"
#include "dBezierSpline.h"

#define D_BEZIER_LOCAL_BUFFER_SIZE 256

dBezierSpline::dBezierSpline ()
	:m_knotVector(NULL)
	,m_controlPoints(NULL) 
	,m_degree(0)
	,m_knotsCount(0)
{
}

dBezierSpline::dBezierSpline (int degree, int knotsCount, const dFloat* const knotVector, const dVector* const controlPoints)
	:dContainersAlloc()
	,m_degree(degree)
	,m_knotsCount(knotsCount + 2 * degree)
	,m_controlPointsCount (knotsCount + m_degree - 1)
{
	dAssert (knotsCount);
	dAssert (knotVector[0] == 0.0f);
	dAssert (knotVector[knotsCount - 1] == 1.0f);

	m_knotVector = (dFloat*) Alloc (m_knotsCount * sizeof (dFloat));
	m_controlPoints = (dVector*) Alloc (m_controlPointsCount * sizeof (dVector));

	memcpy (m_controlPoints, controlPoints, m_controlPointsCount * sizeof (dVector));
	for (int i = 0; i < m_degree; i ++) {
		m_knotVector[i] = 0.0f;
		m_knotVector[i + m_knotsCount - m_degree] = 1.0f;
	}

	for (int i = 0; i < knotsCount; i ++) {
		m_knotVector[i + m_degree] = knotVector[i];
		dAssert (m_knotVector[i + m_degree] >= m_knotVector[i + m_degree - 1]);
	}
}

dBezierSpline::~dBezierSpline ()
{
	Clear();
}

int dBezierSpline::GetControlPointCount() const
{
	return m_controlPointsCount;
}

dVector dBezierSpline::GetControlPoint(int i) const
{
	return m_controlPoints[i];
}


void dBezierSpline::Clear()
{
	if (m_knotVector) {
		Free (m_knotVector);
	}

	if (m_controlPoints) {
		Free (m_controlPoints);
	}
	m_knotVector = NULL;
	m_controlPoints = NULL;
}

int dBezierSpline::GetSpan(dFloat u) const
{
	int low = m_degree;
	int high = m_knotsCount - m_degree - 1;

	dAssert (u >= 0.0f);
	dAssert (u <= 1.0f);
	while ((high - low) >= 4) {
		int mid = (low + high) >> 1;
		if (u > m_knotVector[mid]) {
			low = mid;
		} else {
			high = mid;
		}
	}

	dAssert (m_knotVector[low] <= u);
	for (int i = low; i < m_degree + m_knotsCount + 1; i ++) {
		if (m_knotVector[i + 1] >= u) {
			return i;
		}
	}
	dAssert (0);
	return 0;
}

void dBezierSpline::BasicsFunctions (dFloat u, int span, dFloat* const BasicFunctionsOut) const
{
	BasicFunctionsOut[0] = 1.0f;

	dFloat left[D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat right[D_BEZIER_LOCAL_BUFFER_SIZE];
	for (int j = 1; j <= m_degree; j ++) {
		left[j] = u - m_knotVector[span + 1 - j]; 
		right[j] = m_knotVector[span + j] - u;

		dFloat saved = 0.0f;
		for (int r = 0; r < j; r ++) {
			dFloat temp = BasicFunctionsOut[r] / (right[r + 1] + left[j - r]);
			BasicFunctionsOut[r] = saved + temp * right[r + 1];
			saved = temp * left[j - r];
		}
		BasicFunctionsOut[j] = saved;
	}
}

void dBezierSpline::BasicsFunctionsDerivatives (dFloat u, int span, dFloat* const derivativesOut) const
{
	dFloat ndu[D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat left[D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat right[D_BEZIER_LOCAL_BUFFER_SIZE];

	const int width = m_degree + 1;
	ndu[0] = 1.0f;
	for (int j = 1; j <= m_degree; j ++) {
		left[j] = u - m_knotVector[span + 1 - j];
		right[j] = m_knotVector[span + j] - u;
		dFloat saved = 0.0f;
		for (int r = 0; r < j; r ++) {
			ndu[j * width + r] = right[r + 1] + left[j - r];

			dFloat temp = ndu[r * width + j - 1] / ndu[j * width + r];
			ndu[r * width + j] = saved + temp * right[r + 1];
			saved = temp * left[j - r];
		}
		ndu[j * width + j] = saved;
	}

	dFloat a[D_BEZIER_LOCAL_BUFFER_SIZE];
	for (int j = 0; j <= m_degree; j ++) {
		derivativesOut[width * 0 + j] = ndu [width * j + m_degree];
	}

	for (int r = 0; r <= m_degree; r ++) {
		int s1 = 0;
		int s2 = 1;
		a[0] = 1.0f;
		for (int k = 1; k <= m_degree; k ++) {
			dFloat d = 0.0f;
			int rk = r - k;
			int pk = m_degree - k;
			if (r >= k)  {
				a[width * s2 + 0] = a[width * s1 + 0] / ndu[width * (pk + 1) + rk];
				d = a[width * s2 + 0] * ndu[width * rk + pk];
			}
			int j1 = 0;
			int j2 = 0;
			if (rk >= -1) {
				j1 = 1;
			} else {
				j1 = -rk;
			}

			if ((r - 1) <= pk) {
				j2 = k-1;
			} else {
				j2 = m_degree-r;
			}
			for (int j = j1; j <= j2; j ++) {
				a[width * s2 + j] = (a[width * s1 + j] - a[width * s1 + j - 1]) / ndu[width * (pk + 1) + rk + j];
				d += a[width * s2 + j] * ndu[width * (rk + j) + pk];
			}
			if (r <= pk) {
				a[width * s2 + k] = -a[width * s1 + k - 1] / ndu[width * (pk + 1) + r];
				d += a[width * s2 + k] * ndu[width * r + pk];
			}
			derivativesOut[width * k + r] = d;
			dSwap(s1, s2);
		}
	}

	int s = m_degree;
	for (int k = 1; k <= m_degree; k ++) {
		for (int j = 0; j <= m_degree; j ++) {
			derivativesOut[width * k + j] *= s;
		}
		s *= (m_degree - k);
	}
}

dVector dBezierSpline::CurvePoint (dFloat u) const
{
	dVector point (0.0f, 0.0f, 0.0f, 0.0f);
	dFloat basicFunctions[D_BEZIER_LOCAL_BUFFER_SIZE];
	int span = GetSpan(u);
	BasicsFunctions (u, span, basicFunctions);
	
	for (int i = 0; i <= m_degree; i ++) {
		point += m_controlPoints[span - m_degree + i].Scale (basicFunctions[i]);
	}
	return point;
}

dVector dBezierSpline::CurveDerivative (dFloat u, int index) const
{
	dAssert (index <= m_degree);
	
	dFloat basicsFuncDerivatives[D_BEZIER_LOCAL_BUFFER_SIZE];
	dVector ck (0.0f, 0.0f, 0.0f, 0.0f);
	int span = GetSpan(u);
	BasicsFunctionsDerivatives (u, span, basicsFuncDerivatives);

	const int with = m_degree + 1;
	dVector point (0.0f, 0.0f, 0.0f, 0.0f);
	for (int i = 0; i <= m_degree; i ++) {
		point += m_controlPoints[span - m_degree + i].Scale (basicsFuncDerivatives[with * index + i]);
	}
	return point;
}

void dBezierSpline::GlobalCubicInterpolation (int count, const dVector* const points, const dVector& firstTangent, const dVector& lastTangent)
{
	CreateCubicKnotVector (count, points);
	CreateCubicControlPoints (count, points, firstTangent, lastTangent);
}


void dBezierSpline::CreateCubicKnotVector(int count, const dVector* const points)
{
	dFloat d = 0.0f;
	dAssert (count >= 2);

	dFloat u[D_BEZIER_LOCAL_BUFFER_SIZE];
	u[0] = 0.0f;
	for (int i = 1; i < count; i ++) {
		dVector step (points[i] - points[i - 1]);
		dFloat len = dSqrt (step % step);
//		u[i] = len; 
		u[i] = dSqrt (len); 
		d += u[i];
	}

	for (int i = 1; i < count; i ++) {
		u[i] = u[i-1] + u[i] / d;
	}
	u[0] = 0.0f;
	u[count - 1] = 1.0f;

	m_degree = 3;
	if ((count + 2 * m_degree) != m_knotsCount) {
		if (m_knotVector) {
			Free (m_knotVector);
		}
		m_knotsCount = count + 2 * m_degree;
		m_knotVector = (dFloat*) Alloc (m_knotsCount * sizeof (dFloat));
	}

	for (int i = 0; i < (m_degree + 1); i ++) {
		m_knotVector[i] = 0.0f;
		m_knotVector[i + m_knotsCount - m_degree - 1] = 1.0f;
	}

	for (int i = 1; i < (count - 1); i ++) {
		dFloat acc = 0.0f;
		for (int j = 0; j < m_degree; j ++) {
			acc += u[j + i - 1];
		}
		m_knotVector[m_degree + i] = acc / 3.0f;
	}

//m_knotVector[4] = 1.0f/4.0f;
//m_knotVector[5] = 2.0f/4.0f;
//m_knotVector[6] = 3.0f/4.0f;
}


void dBezierSpline::CreateCubicControlPoints(int count, const dVector* const points, const dVector& firstTangent, const dVector& lastTangent)
{
	dFloat abc[4];

	if ((m_knotsCount - 2 * (m_degree - 1)) != m_controlPointsCount) {
		if (m_controlPoints) {
			Free (m_controlPoints);
		}
		m_controlPointsCount = m_knotsCount - 2 * (m_degree - 1);
		m_controlPoints = (dVector*) Alloc (m_controlPointsCount * sizeof (dVector));
	}

	m_controlPoints[0] = points[0];
	m_controlPoints[m_controlPointsCount - 1] = points[count - 1];

	m_controlPoints[1] = m_controlPoints[0] + firstTangent.Scale (m_knotVector[m_degree + 1] / 3.0f);
	m_controlPoints[m_controlPointsCount - 2] = m_controlPoints[m_controlPointsCount - 1] - lastTangent.Scale ((1.0f - m_knotVector[m_knotsCount - m_degree - 2]) / 3.0f);
	if (count == 3) {
		BasicsFunctions (m_knotVector[m_degree + 1], m_degree + 1, abc);
		m_controlPoints[2]  = points[1] - m_controlPoints[1].Scale (abc[0]) - m_controlPoints[3].Scale (abc[2]);
		m_controlPoints[2] = m_controlPoints[2].Scale (1.0f / abc[1]);
	} else {
		//dVector r [D_BEZIER_LOCAL_BUFFER_SIZE];
		//dFloat dd [D_BEZIER_LOCAL_BUFFER_SIZE];
		//for (int i = 3; i < count; i ++) {
			//r[i] = points[i - 1];
		//}
		
		dFloat dd [D_BEZIER_LOCAL_BUFFER_SIZE];
		BasicsFunctions (m_knotVector[m_degree + 1], m_degree + 1, abc);
		dFloat den = abc[1];
		m_controlPoints[2]  = (points[1] - m_controlPoints[1].Scale (abc[0])).Scale (1.0f / den);
		for (int i = 3; i < (count - 1); i ++) {
			dd[i + 1] = abc[2] / den;
			BasicsFunctions (m_knotVector[i + 2], i + 2, abc);
			den = abc[1] - abc[0] * dd[i + 1];
			m_controlPoints[i]  = (points[i - 1] - m_controlPoints[i - 1].Scale (abc[0])).Scale (1.0f / den);
		}

		dd[count] = abc[2] / den;
		BasicsFunctions (m_knotVector[count + 1], count + 1, abc);
		den = abc[1] - abc[0] * dd[count];
		m_controlPoints[count - 1] = (points[count - 2] - m_controlPoints[count].Scale (abc[2]) - m_controlPoints[count - 2].Scale (abc[0])).Scale (1.0f / den);

		for (int i = count - 2; i >= 2; i --) {
			m_controlPoints[i] -= m_controlPoints[i + 1].Scale (dd[i + 2]);
		}
	}
}