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


dBezierSpline::dBezierSpline (int degree, int knotsCount, const dFloat* const knotVector, const dVector* const controlPoints)
	:dContainersAlloc()
	,m_degree(degree)
	,m_knotsCount(knotsCount)
{
	dAssert (!knotsCount || (knotVector[0] > 0.0f));
	dAssert (!knotsCount || (knotVector[knotsCount - 1] < 1.0f));

	int knotCount = 2 * (m_degree + 1) + m_knotsCount;
	m_knotVector = (dFloat*) Alloc (knotCount * sizeof (dFloat));

	int controlPointsCount = knotsCount + m_degree + 1;
	m_controlPoints = (dVector*) Alloc (controlPointsCount * sizeof (dVector));

	memcpy (m_controlPoints, controlPoints, controlPointsCount * sizeof (dVector));
	for (int i = 0; i < (m_degree + 1); i ++) {
		m_knotVector[i] = 0.0f;
		m_knotVector[i + knotsCount + m_degree + 1] = 1.0f;
	}

	for (int i = 0; i < m_knotsCount; i ++) {
		m_knotVector[i + m_degree + 1] = knotVector[i];
		dAssert (m_knotVector[i + m_degree + 1] <= m_knotVector[i + m_degree]);
	}
}

dBezierSpline::~dBezierSpline ()
{
	if (m_knotVector) {
		Free (m_knotVector);
	}

	if (m_controlPoints) {
		Free (m_controlPoints);
	}
}


int dBezierSpline::GetSpan(dFloat u) const
{
	int low = m_degree;
	int high = m_degree + m_knotsCount + 1;

	dAssert (u >= 0.0f);
	dAssert (u <= 1.0f);
	while ((high - low) >= 4) {
		int mid = (low + high) >> 1;
		if (u >= m_knotVector[mid]) {
			high = mid;
		} else {
			low = mid;
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

	dFloat left[128];
	dFloat right[128];
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
	dFloat basicFunctions[128];
	int span = GetSpan(u);
	BascisFunctions (u, span, basicFunctions);

	dVector point (0.0f, 0.0f, 0.0f, 0.0f);
	for (int i = 0; i <= m_degree; i ++) {
		point += m_controlPoints[span - m_degree + i].Scale (basicFunctions[i]);
	}
	return point;
}