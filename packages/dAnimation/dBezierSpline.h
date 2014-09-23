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

#ifndef __D_BEZIER_SPLINE_H__
#define __D_BEZIER_SPLINE_H__

class dBezierSpline : public dContainersAlloc
{
	public:

	// empty spline
	dBezierSpline ();
	dBezierSpline (const dBezierSpline& src);

	// create from knot vector and control points
	virtual ~dBezierSpline ();

	dBezierSpline& operator = (const dBezierSpline &copy) ;

	int GetDegree () const;

	dBigVector CurvePoint (dFloat64 u) const;
	dBigVector CurveDerivative (dFloat64 u, int index = 1) const;
	int CurveAllDerivatives (dFloat64 u, dBigVector* const defivatives) const;

	dFloat64 CalculateLength (dFloat64 tol) const;

	void GlobalCubicInterpolation (int count, const dBigVector* const points, const dBigVector& firstTangent, const dBigVector& lastTangent);
	void CreateFromKnotVectorAndControlPoints (int degree, int knotCount, const dFloat64* const knotVector, const dBigVector* const controlPoints);

	void InsertKnot (dFloat64 u);
	bool RemoveKnot (dFloat64 u, dFloat64 tol);
	

	int GetControlPointCount() const;
	dBigVector* GetControlPointArray();
	const dBigVector* GetControlPointArray() const;
	dBigVector GetControlPoint(int i) const;
	void SetControlPoint(int i, const dBigVector& point);
	
	int GetKnotCount() const;
	dFloat64* GetKnotArray();
	const dFloat64* GetKnotArray() const;
	dFloat64 GetKnot(int i) const;

	dFloat64 FindClosestKnot (dBigVector& closestPoint, const dBigVector& point, int subdivitionSteps = 2) const;

	private:
	void Clear();
	int GetSpan(dFloat64 u) const;

	dBigVector CurvePoint (dFloat64 u, int span) const;
	void CreateCubicKnotVector(int count, const dBigVector* const points);
	void CreateCubicControlPoints(int count, const dBigVector* const points, const dBigVector& firstTangent, const dBigVector& lastTangent);

	void BasicsFunctions (dFloat64 u, int span, dFloat64* const functionOut) const;
	void BasicsFunctionsDerivatives (dFloat64 u, int span, dFloat64* const derivatyivesOut) const;
	

	dFloat64* m_knotVector;
	dBigVector* m_controlPoints; 

	int m_degree;
	int m_knotsCount;
	int m_controlPointsCount;
};




#endif

