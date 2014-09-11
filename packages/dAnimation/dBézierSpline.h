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
	dBezierSpline (int degree, int knotCount, const dFloat* const knotVector, const dVector* const controlPoints);
	virtual ~dBezierSpline ();

	dVector CurvePoint (dFloat u) const;

	private:
	int GetSpan(dFloat u) const;
	void BascisFunctions (dFloat u, int span, dFloat* const funtionOut) const;
	

	dFloat* m_knotVector;
	dVector* m_controlPoints; 
	int m_degree;

	int m_knotsCount;
	int m_knotMaxSize;
};




#endif

