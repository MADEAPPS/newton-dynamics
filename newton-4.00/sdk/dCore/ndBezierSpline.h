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
#include "ndArray.h"
#include "ndClassAlloc.h"

#ifndef __ND_BEZIER_SPLINE_H__
#define __ND_BEZIER_SPLINE_H__

class ndBezierSpline : public ndClassAlloc
{
	public:

	// empty spline
	D_CORE_API ndBezierSpline ();
	D_CORE_API ndBezierSpline (const ndBezierSpline& src);

	// create from knot vector and control points
	D_CORE_API virtual ~ndBezierSpline ();

	D_CORE_API ndBezierSpline& operator = (const ndBezierSpline &copy) ;

	D_CORE_API dInt32 GetDegree () const;

	D_CORE_API ndBigVector CurvePoint (dFloat64 u) const;
	D_CORE_API ndBigVector CurveDerivative (dFloat64 u, dInt32 index = 1) const;
	D_CORE_API dInt32 CurveAllDerivatives (dFloat64 u, ndBigVector* const defivatives) const;

	D_CORE_API dFloat64 CalculateLength (dFloat64 tol) const;

	D_CORE_API void GlobalCubicInterpolation (dInt32 count, const ndBigVector* const points, const ndBigVector& firstTangent, const ndBigVector& lastTangent);
	D_CORE_API void CreateFromKnotVectorAndControlPoints (dInt32 degree, dInt32 knotCount, const dFloat64* const knotVector, const ndBigVector* const controlPoints);

	D_CORE_API void InsertKnot (dFloat64 u);
	D_CORE_API bool RemoveKnot (dFloat64 u, dFloat64 tol);

	D_CORE_API dInt32 GetControlPointCount() const;
	D_CORE_API ndArray<ndBigVector>& GetControlPointArray();
	D_CORE_API const ndArray<ndBigVector>& GetControlPointArray() const;

	D_CORE_API ndBigVector GetControlPoint(dInt32 i) const;
	D_CORE_API void SetControlPoint(dInt32 i, const ndBigVector& point);
	
	D_CORE_API dInt32 GetKnotCount() const;
	D_CORE_API ndArray<dFloat64>& GetKnotArray();
	D_CORE_API const ndArray<dFloat64>& GetKnotArray() const;

	D_CORE_API dFloat64 GetKnot(dInt32 i) const;
	D_CORE_API dFloat64 FindClosestKnot (ndBigVector& closestPointOnCurve, const ndBigVector& point, dInt32 subdivitionSteps = 2) const;

	private:
	void Clear();
	dInt32 GetSpan(dFloat64 u) const;

	ndBigVector CurvePoint (dFloat64 u, dInt32 span) const;
	void CreateCubicKnotVector(dInt32 count, const ndBigVector* const points);
	void CreateCubicControlPoints(dInt32 count, const ndBigVector* const points, const ndBigVector& firstTangent, const ndBigVector& lastTangent);

	void BasicsFunctions (dFloat64 u, dInt32 span, dFloat64* const functionOut) const;
	void BasicsFunctionsDerivatives (dFloat64 u, dInt32 span, dFloat64* const derivatyivesOut) const;
	
	ndArray<dFloat64> m_knotVector;
	ndArray<ndBigVector> m_controlPoints;

	dInt32 m_degree;
	dInt32 m_knotsCount;
	dInt32 m_controlPointsCount;
};
#endif

