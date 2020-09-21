/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCoreStdafx.h"
#include "dArray.h"
#include "dClassAlloc.h"

#ifndef __D_BEZIER_SPLINE_H__
#define __D_BEZIER_SPLINE_H__

class dBezierSpline : public dClassAlloc
{
	public:

	// empty spline
	D_CORE_API dBezierSpline ();
	D_CORE_API dBezierSpline (const dBezierSpline& src);

	// create from knot vector and control points
	D_CORE_API virtual ~dBezierSpline ();

	D_CORE_API dBezierSpline& operator = (const dBezierSpline &copy) ;

	D_CORE_API int GetDegree () const;

	D_CORE_API dBigVector CurvePoint (dFloat64 u) const;
	D_CORE_API dBigVector CurveDerivative (dFloat64 u, int index = 1) const;
	D_CORE_API int CurveAllDerivatives (dFloat64 u, dBigVector* const defivatives) const;

	D_CORE_API dFloat64 CalculateLength (dFloat64 tol) const;

	D_CORE_API void GlobalCubicInterpolation (dInt32 count, const dBigVector* const points, const dBigVector& firstTangent, const dBigVector& lastTangent);
	D_CORE_API void CreateFromKnotVectorAndControlPoints (dInt32 degree, dInt32 knotCount, const dFloat64* const knotVector, const dBigVector* const controlPoints);

	D_CORE_API void InsertKnot (dFloat64 u);
	D_CORE_API bool RemoveKnot (dFloat64 u, dFloat64 tol);

	D_CORE_API dInt32 GetControlPointCount() const;
	D_CORE_API dArray<dBigVector>& GetControlPointArray();
	D_CORE_API const dArray<dBigVector>& GetControlPointArray() const;

	D_CORE_API dBigVector GetControlPoint(dInt32 i) const;
	D_CORE_API void SetControlPoint(dInt32 i, const dBigVector& point);
	
	D_CORE_API dInt32 GetKnotCount() const;
	D_CORE_API dArray<dFloat64>& GetKnotArray();
	D_CORE_API const dArray<dFloat64>& GetKnotArray() const;

	D_CORE_API dFloat64 GetKnot(dInt32 i) const;
	D_CORE_API dFloat64 FindClosestKnot (dBigVector& closestPointOnCurve, const dBigVector& point, dInt32 subdivitionSteps = 2) const;

	private:
	void Clear();
	dInt32 GetSpan(dFloat64 u) const;

	dBigVector CurvePoint (dFloat64 u, dInt32 span) const;
	void CreateCubicKnotVector(dInt32 count, const dBigVector* const points);
	void CreateCubicControlPoints(dInt32 count, const dBigVector* const points, const dBigVector& firstTangent, const dBigVector& lastTangent);

	void BasicsFunctions (dFloat64 u, dInt32 span, dFloat64* const functionOut) const;
	void BasicsFunctionsDerivatives (dFloat64 u, dInt32 span, dFloat64* const derivatyivesOut) const;
	
	dArray<dFloat64> m_knotVector;
	dArray<dBigVector> m_controlPoints;

	dInt32 m_degree;
	dInt32 m_knotsCount;
	dInt32 m_controlPointsCount;
};
#endif

