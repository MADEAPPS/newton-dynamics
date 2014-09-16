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

	dVector CurvePoint (dFloat u) const;
	dVector CurveDerivative (dFloat u, int index = 1) const;

	dFloat CalculateLength (dFloat tol) const;

	void CreateFromKnotVectorAndControlPoints (int degree, int knotCount, const dFloat* const knotVector, const dVector* const controlPoints);

	void GlobalCubicInterpolation (int count, const dVector* const points, const dVector& firstTangent, const dVector& lastTangent);

	int GetControlPointCount() const;
	dVector* GetControlPointArray();
	const dVector* GetControlPointArray() const;
	dVector GetControlPoint(int i) const;
	void SetControlPoint(int i, const dVector& point);
	
	int GetKnotCount() const;
	dFloat* GetKnotArray();
	const dFloat* GetKnotArray() const;
	dFloat GetKnot(int i) const;

	dFloat FindClosestKnot (const dVector& point, int subdivitionSteps = 2) const;
	
	private:
	void Clear();
	int GetSpan(dFloat u) const;

	dVector CurvePoint (dFloat u, int span) const;
	void CreateCubicKnotVector(int count, const dVector* const points);
	void CreateCubicControlPoints(int count, const dVector* const points, const dVector& firstTangent, const dVector& lastTangent);

	void BasicsFunctions (dFloat u, int span, dFloat* const functionOut) const;
	void BasicsFunctionsDerivatives (dFloat u, int span, dFloat* const derivatyivesOut) const;
	

	dFloat* m_knotVector;
	dVector* m_controlPoints; 

	int m_degree;
	int m_knotsCount;
	int m_controlPointsCount;
//	int m_knotMaxSize;
};




#endif

