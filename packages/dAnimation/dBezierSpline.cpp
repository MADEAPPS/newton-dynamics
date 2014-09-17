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
	,m_controlPointsCount(0)
{
}

dBezierSpline::dBezierSpline (const dBezierSpline& src)
	:m_knotVector(NULL)
	,m_controlPoints(NULL) 
	,m_degree(0)
	,m_knotsCount(0)
	,m_controlPointsCount(0)
{
	if (src.m_knotsCount) {
		CreateFromKnotVectorAndControlPoints (src.m_degree, src.m_knotsCount - 2 * src.m_degree, &src.m_knotVector[src.m_degree], src.m_controlPoints);
	}
}

dBezierSpline& dBezierSpline::operator = (const dBezierSpline &copy)
{
	Clear();
	if (copy.m_knotsCount) {
		CreateFromKnotVectorAndControlPoints (copy.m_degree, copy.m_knotsCount - 2 * copy.m_degree, &copy.m_knotVector[copy.m_degree], copy.m_controlPoints);
	}
	return *this;
}

dBezierSpline::~dBezierSpline ()
{
	Clear();
}

int dBezierSpline::GetDegree () const
{
	return m_degree;
}

void dBezierSpline::CreateFromKnotVectorAndControlPoints (int degree, int knotCount, const dFloat64* const knotVector, const dBigVector* const controlPoints)
{
	Clear();
	dAssert (knotCount);
	dAssert (knotVector[0] == 0.0f);
	dAssert (knotVector[knotCount - 1] == 1.0f);

	m_degree = degree;
	m_knotsCount = knotCount + 2 * degree;
	m_controlPointsCount = knotCount + m_degree - 1;

	m_knotVector = (dFloat64*) Alloc (m_knotsCount * sizeof (dFloat64));
	m_controlPoints = (dBigVector*) Alloc (m_controlPointsCount * sizeof (dBigVector));

	memcpy (m_controlPoints, controlPoints, m_controlPointsCount * sizeof (dBigVector));
	for (int i = 0; i < m_degree; i ++) {
		m_knotVector[i] = 0.0f;
		m_knotVector[i + m_knotsCount - m_degree] = 1.0f;
	}

	for (int i = 0; i < knotCount; i ++) {
		m_knotVector[i + m_degree] = knotVector[i];
		dAssert (m_knotVector[i + m_degree] >= m_knotVector[i + m_degree - 1]);
	}
}

int dBezierSpline::GetKnotCount() const
{
	return m_knotsCount;
}

dFloat64* dBezierSpline::GetKnotArray()
{
	return m_knotVector;
}

const dFloat64* dBezierSpline::GetKnotArray() const
{
	return m_knotVector;
}

dFloat64 dBezierSpline::GetKnot(int i) const
{
	return m_knotVector[i];
}


int dBezierSpline::GetControlPointCount() const
{
	return m_controlPointsCount;
}

dBigVector dBezierSpline::GetControlPoint(int i) const
{
	return m_controlPoints[i];
}

void dBezierSpline::SetControlPoint(int i, const dBigVector& point)
{
	m_controlPoints[i] = point;
}

dBigVector* dBezierSpline::GetControlPointArray()
{
	return m_controlPoints;
}

const dBigVector* dBezierSpline::GetControlPointArray() const
{
	return m_controlPoints;
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

int dBezierSpline::GetSpan(dFloat64 u) const
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

void dBezierSpline::BasicsFunctions (dFloat64 u, int span, dFloat64* const BasicFunctionsOut) const
{
	BasicFunctionsOut[0] = 1.0f;

	dFloat64 left[D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat64 right[D_BEZIER_LOCAL_BUFFER_SIZE];
	for (int j = 1; j <= m_degree; j ++) {
		left[j] = u - m_knotVector[span + 1 - j]; 
		right[j] = m_knotVector[span + j] - u;

		dFloat64 saved = 0.0f;
		for (int r = 0; r < j; r ++) {
			dFloat64 temp = BasicFunctionsOut[r] / (right[r + 1] + left[j - r]);
			BasicFunctionsOut[r] = saved + temp * right[r + 1];
			saved = temp * left[j - r];
		}
		BasicFunctionsOut[j] = saved;
	}
}

void dBezierSpline::BasicsFunctionsDerivatives (dFloat64 u, int span, dFloat64* const derivativesOut) const
{
	dFloat64 ndu[D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat64 left[D_BEZIER_LOCAL_BUFFER_SIZE];
	dFloat64 right[D_BEZIER_LOCAL_BUFFER_SIZE];

	const int width = m_degree + 1;
	ndu[0] = 1.0f;
	for (int j = 1; j <= m_degree; j ++) {
		left[j] = u - m_knotVector[span + 1 - j];
		right[j] = m_knotVector[span + j] - u;
		dFloat64 saved = 0.0f;
		for (int r = 0; r < j; r ++) {
			ndu[j * width + r] = right[r + 1] + left[j - r];

			dFloat64 temp = ndu[r * width + j - 1] / ndu[j * width + r];
			ndu[r * width + j] = saved + temp * right[r + 1];
			saved = temp * left[j - r];
		}
		ndu[j * width + j] = saved;
	}

	dFloat64 a[D_BEZIER_LOCAL_BUFFER_SIZE];
	for (int j = 0; j <= m_degree; j ++) {
		derivativesOut[width * 0 + j] = ndu [width * j + m_degree];
	}

	for (int r = 0; r <= m_degree; r ++) {
		int s1 = 0;
		int s2 = 1;
		a[0] = 1.0f;
		for (int k = 1; k <= m_degree; k ++) {
			dFloat64 d = 0.0f;
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

dBigVector dBezierSpline::CurvePoint (dFloat64 u, int span) const
{
	dBigVector point (0.0f, 0.0f, 0.0f, 0.0f);
	dFloat64 basicFunctions[D_BEZIER_LOCAL_BUFFER_SIZE];
	BasicsFunctions (u, span, basicFunctions);
	for (int i = 0; i <= m_degree; i ++) {
		point += m_controlPoints[span - m_degree + i].Scale (basicFunctions[i]);
	}
	return point;
}

dBigVector dBezierSpline::CurvePoint (dFloat64 u) const
{
	int span = GetSpan(u);
	return CurvePoint (u, span);
}

dBigVector dBezierSpline::CurveDerivative (dFloat64 u, int index) const
{
	dAssert (index <= m_degree);
	
	dFloat64 basicsFuncDerivatives[D_BEZIER_LOCAL_BUFFER_SIZE];
	int span = GetSpan(u);
	BasicsFunctionsDerivatives (u, span, basicsFuncDerivatives);

	const int with = m_degree + 1;
	dBigVector point (0.0f, 0.0f, 0.0f, 0.0f);
	for (int i = 0; i <= m_degree; i ++) {
		point += m_controlPoints[span - m_degree + i].Scale (basicsFuncDerivatives[with * index + i]);
	}
	return point;
}

int dBezierSpline::CurveAllDerivatives (dFloat64 u, dBigVector* const derivatives) const
{
	dFloat64 basicsFuncDerivatives[D_BEZIER_LOCAL_BUFFER_SIZE];
	int span = GetSpan(u);
	BasicsFunctionsDerivatives (u, span, basicsFuncDerivatives);

	const int with = m_degree + 1;
	dBigVector point (0.0f, 0.0f, 0.0f, 0.0f);
	for (int j = 0; j <= m_degree; j ++) {
		dBigVector ck (0.0f, 0.0f, 0.0f, 0.0f);
		for (int i = 0; i <= m_degree; i ++) {
			ck += m_controlPoints[span - m_degree + i].Scale (basicsFuncDerivatives[with * j + i]);
		}
		derivatives[j] = ck;
	}

	return m_degree + 1;
}

void dBezierSpline::GlobalCubicInterpolation (int count, const dBigVector* const points, const dBigVector& firstTangent, const dBigVector& lastTangent)
{
	CreateCubicKnotVector (count, points);
	CreateCubicControlPoints (count, points, firstTangent, lastTangent);
}


void dBezierSpline::CreateCubicKnotVector(int count, const dBigVector* const points)
{
	dFloat64 d = 0.0f;
	dAssert (count >= 2);

	dFloat64 u[D_BEZIER_LOCAL_BUFFER_SIZE];
	u[0] = 0.0f;
	for (int i = 1; i < count; i ++) {
		dBigVector step (points[i] - points[i - 1]);
		dFloat64 len = dSqrt (step % step);
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
		m_knotVector = (dFloat64*) Alloc (m_knotsCount * sizeof (dFloat64));
	}

	for (int i = 0; i < (m_degree + 1); i ++) {
		m_knotVector[i] = 0.0f;
		m_knotVector[i + m_knotsCount - m_degree - 1] = 1.0f;
	}

	for (int i = 1; i < (count - 1); i ++) {
		dFloat64 acc = 0.0f;
		for (int j = 0; j < m_degree; j ++) {
			acc += u[j + i - 1];
		}
		m_knotVector[m_degree + i] = acc / 3.0f;
	}

//m_knotVector[4] = 1.0f/4.0f;
//m_knotVector[5] = 2.0f/4.0f;
//m_knotVector[6] = 3.0f/4.0f;
}


void dBezierSpline::CreateCubicControlPoints(int count, const dBigVector* const points, const dBigVector& firstTangent, const dBigVector& lastTangent)
{
	dFloat64 abc[4];

	if ((m_knotsCount - 2 * (m_degree - 1)) != m_controlPointsCount) {
		if (m_controlPoints) {
			Free (m_controlPoints);
		}
		m_controlPointsCount = m_knotsCount - 2 * (m_degree - 1);
		m_controlPoints = (dBigVector*) Alloc (m_controlPointsCount * sizeof (dBigVector));
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
		//dBigVector r [D_BEZIER_LOCAL_BUFFER_SIZE];
		//dFloat64 dd [D_BEZIER_LOCAL_BUFFER_SIZE];
		//for (int i = 3; i < count; i ++) {
			//r[i] = points[i - 1];
		//}
		
		dFloat64 dd [D_BEZIER_LOCAL_BUFFER_SIZE];
		BasicsFunctions (m_knotVector[m_degree + 1], m_degree + 1, abc);
		dFloat64 den = abc[1];
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


dFloat64 dBezierSpline::CalculateLength (dFloat64 tol) const
{
	dBigVector stackPool[32][3];
	int stack = 0;

	dFloat64 length = 0.0f;
	dFloat64 tol2 = tol * tol;
	dFloat64 u0 = m_knotVector[m_degree];
	dBigVector p0 (CurvePoint (u0));

	for (int i = m_degree; i < (m_knotsCount - m_degree - 1); i ++) {
		dFloat64 u1 = m_knotVector[i + 1];
		dBigVector p1 (CurvePoint (u1));
		stackPool[stack][0] = p0;
		stackPool[stack][1] = p1;
		stackPool[stack][2] = dBigVector (u0, u1, 0.0, 0.0);
		stack ++;
		while (stack) {
			stack --;
			dBigVector q0 (stackPool[stack][0]);
			dBigVector q1 (stackPool[stack][1]);
			dFloat64 t0 = stackPool[stack][2][0];
			dFloat64 t1 = stackPool[stack][2][1];
			dFloat64 t01 = (t1 + t0) * 0.5f;

			dBigVector p01 ((q1 + q0).Scale (0.5f));
			dBigVector q01 (CurvePoint (t01));
			dBigVector err (q01 - p01);

			dFloat64 err2 = err % err;
			if (err2 < tol2) {
				dBigVector step (q1 - q0);
				length += dSqrt (step % step);
			} else {
				stackPool[stack][0] = q01;
				stackPool[stack][1] = q1;
				stackPool[stack][2] = dBigVector (t01, t1, 0.0, 0.0);
				stack ++;

				stackPool[stack][0] = q0;
				stackPool[stack][1] = q01;
				stackPool[stack][2] = dBigVector (t0, t01, 0.0, 0.0);
				stack ++;
			}
		}
		u0 = u1;
		p0 = p1;
	}

	return length;
}


dFloat64 dBezierSpline::FindClosestKnot (dBigVector& closestPoint, const dBigVector& point, int subdivitionSteps) const
{
	int startSpan = 0;
	dFloat64 bestU = 0.0f;;
	dFloat64 distance2 = 1.0e10f;
	dBigVector closestControlPoint (m_controlPoints[0]);
	subdivitionSteps = dMax (subdivitionSteps, 1);
	dFloat64 scale = 1.0f / subdivitionSteps;
	for (int span = m_degree; span < (m_knotsCount - m_degree); span ++) {
		dFloat64 param = 0.0f;
		for (int i = 0; i < subdivitionSteps; i ++) {
			dFloat64 u = m_knotVector[span] + (m_knotVector[span + 1] - m_knotVector[span]) * param;
			param += scale;
			dBigVector p (CurvePoint (u, span));
			dBigVector dp (p - point);
			dFloat64 dist2 = dp % dp;
			if (dist2 < distance2) {
				bestU = u;
				startSpan = span;
				distance2 = dist2;
				closestControlPoint = p;
			}
		}
	}

	dBigVector derivatives[32];
	dFloat64 u0 = bestU;

	bool stop = false;
	for (int i = 0; (i < 20) && !stop; i ++) {
		CurveAllDerivatives (u0, derivatives);

		dBigVector dist (closestControlPoint - point);
		dFloat64 num = derivatives[1] % dist;
		dFloat64 den = derivatives[2] % dist + derivatives[1] % derivatives[1];
		
		dFloat64 u1 = dClamp(u0 - num / den, 0.0, 1.0);
		if (u1 < m_knotVector[startSpan]) {
			startSpan --;
			dAssert (startSpan >= 0);
		} else if (u1 >= m_knotVector[startSpan + 1]) {
			startSpan ++;
			dAssert (startSpan < (m_knotsCount - m_degree));
		} 

		closestControlPoint = CurvePoint (u1, startSpan);
		//dFloat64 xxx0 = num * num;
		//dFloat64 xxx1 = dist % dist;
		//dFloat64 xxx2 = derivatives[1] % derivatives[1];
		//dFloat64 xxx3 = xxx1 * xxx2 * 1.0e-10;

		stop |= (dAbs (u1 - u0) < 1.0e-10) || (num * num < ((dist % dist) * (derivatives[1] % derivatives[1]) * 1.0e-10));
		u0 = u1;
	}

	closestPoint = closestControlPoint;
	return u0;
}