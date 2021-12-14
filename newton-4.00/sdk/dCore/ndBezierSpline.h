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

	D_CORE_API ndInt32 GetDegree () const;

	D_CORE_API ndBigVector CurvePoint (ndFloat64 u) const;
	D_CORE_API ndBigVector CurveDerivative (ndFloat64 u, ndInt32 index = 1) const;
	D_CORE_API ndInt32 CurveAllDerivatives (ndFloat64 u, ndBigVector* const defivatives) const;

	D_CORE_API ndFloat64 CalculateLength (ndFloat64 tol) const;

	D_CORE_API void GlobalCubicInterpolation (ndInt32 count, const ndBigVector* const points, const ndBigVector& firstTangent, const ndBigVector& lastTangent);
	D_CORE_API void CreateFromKnotVectorAndControlPoints (ndInt32 degree, ndInt32 knotCount, const ndFloat64* const knotVector, const ndBigVector* const controlPoints);

	D_CORE_API void InsertKnot (ndFloat64 u);
	D_CORE_API bool RemoveKnot (ndFloat64 u, ndFloat64 tol);

	D_CORE_API ndInt32 GetControlPointCount() const;
	D_CORE_API ndArray<ndBigVector>& GetControlPointArray();
	D_CORE_API const ndArray<ndBigVector>& GetControlPointArray() const;

	D_CORE_API ndBigVector GetControlPoint(ndInt32 i) const;
	D_CORE_API void SetControlPoint(ndInt32 i, const ndBigVector& point);
	
	D_CORE_API ndInt32 GetKnotCount() const;
	D_CORE_API ndArray<ndFloat64>& GetKnotArray();
	D_CORE_API const ndArray<ndFloat64>& GetKnotArray() const;

	D_CORE_API ndFloat64 GetKnot(ndInt32 i) const;
	D_CORE_API ndFloat64 FindClosestKnot (ndBigVector& closestPointOnCurve, const ndBigVector& point, ndInt32 subdivitionSteps = 2) const;

	private:
	void Clear();
	ndInt32 GetSpan(ndFloat64 u) const;

	ndBigVector CurvePoint (ndFloat64 u, ndInt32 span) const;
	void CreateCubicKnotVector(ndInt32 count, const ndBigVector* const points);
	void CreateCubicControlPoints(ndInt32 count, const ndBigVector* const points, const ndBigVector& firstTangent, const ndBigVector& lastTangent);

	void BasicsFunctions (ndFloat64 u, ndInt32 span, ndFloat64* const functionOut) const;
	void BasicsFunctionsDerivatives (ndFloat64 u, ndInt32 span, ndFloat64* const derivatyivesOut) const;
	
	ndArray<ndFloat64> m_knotVector;
	ndArray<ndBigVector> m_controlPoints;

	ndInt32 m_degree;
	ndInt32 m_knotsCount;
	ndInt32 m_controlPointsCount;
};
#endif

