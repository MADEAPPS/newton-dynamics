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
#include "dBézierSpline.h"

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

void dBezierSpline::BascisFunctions (dFloat u, int span, dFloat* const BasicFunctionsOut) const
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

dVector dBezierSpline::CurvePoint (dFloat u) const
{
	dVector point (0.0f, 0.0f, 0.0f, 0.0f);
	dFloat basicFunctions[D_BEZIER_LOCAL_BUFFER_SIZE];
	int span = GetSpan(u);
	BascisFunctions (u, span, basicFunctions);
	
	for (int i = 0; i <= m_degree; i ++) {
		point += m_controlPoints[span - m_degree + i].Scale (basicFunctions[i]);
	}
	return point;
}

dVector dBezierSpline::CurveDerivative (dFloat u) const
{
	dVector point (0.0f, 0.0f, 0.0f, 0.0f);

	return point;
}

void dBezierSpline::GlobalCubicInterpolation (int count, const dVector* const points, const dVector& firstTangent, const dVector& lastTangent)
{
	Clear();
	CreateCubicKnotVector (count, points);
	CreateCubicControlPoints (count, points, firstTangent, lastTangent);
}


void dBezierSpline::CreateCubicKnotVector(int count, const dVector* const points)
{
	dFloat d = 0.0f;
	dAssert (count >= 3);

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
	m_knotsCount = count + m_degree + 1;
	m_knotVector = (dFloat*) Alloc (m_knotsCount * sizeof (dFloat));
	for (int i = 0; i < (m_degree + 1); i ++) {
		m_knotVector[i] = 0.0f;
		m_knotVector[i + m_knotsCount - m_degree - 1] = 1.0f;
	}

	for (int i = 1; i < (count - m_degree); i ++) {
		dFloat acc = 0.0f;
		for (int j = 0; j < m_degree; j ++) {
			acc += u[j + i];
		}
		m_knotVector[m_degree + i] = acc / 3.0f;
	}
}


void dBezierSpline::CreateCubicControlPoints(int count, const dVector* const points, const dVector& firstTangent, const dVector& lastTangent)
{
	dVector r [D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat dd [D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat abc[4];

	m_controlPointsCount = m_knotsCount - m_degree + 1;
	m_controlPoints = (dVector*) Alloc (m_controlPointsCount * sizeof (dVector));

	m_controlPoints[0] = points[0];
	m_controlPoints[m_controlPointsCount - 1] = points[count - 1];

	m_controlPoints[1] = m_controlPoints[0] + firstTangent.Scale (m_knotVector[m_degree + 1] / 3.0f);
	m_controlPoints[m_controlPointsCount - 2] = m_controlPoints[m_controlPointsCount - 1] + lastTangent.Scale (m_knotVector[m_knotsCount - m_degree - 2] / 3.0f);

	for (int i = 3; i < count; i ++) {
		r[i] = points[i - 1];
	}
	BascisFunctions (m_knotVector[m_degree + 1], m_degree + 1, abc);

	dFloat den = abc[1];
	m_controlPoints[2]  = (points[1] - m_controlPoints[1].Scale (abc[0])).Scale (1.0f / den);
	for (int i = 3; i < count; i ++) {
		dd[i] = abc[2] / den;
		BascisFunctions (m_knotVector[i + 2], i + 2, abc);
		den = abc[1] - abc[0] * dd[i];
		m_controlPoints[i]  = (r[i] - m_controlPoints[i-1].Scale (abc[0])).Scale (1.0f / den);
	}
	dd[count] = abc[2] / den;
	BascisFunctions (m_knotVector[count + 2], count + 2, abc);
	den = abc[1] - abc[0] * dd[count];
	m_controlPoints[count]  = (points[count - 1] - m_controlPoints[count + 1].Scale (abc[2]) - m_controlPoints[count - 1].Scale (abc[0])).Scale (1.0f / den);

	for (int i = count - 1; i >= 2; i --) {
		m_controlPoints[i] -= m_controlPoints[i + 1].Scale (dd[i + 1]);
	}
}