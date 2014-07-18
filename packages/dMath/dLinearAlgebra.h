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

#include "dStdAfxMath.h"
#include "dMathDefines.h"

#ifndef __D_LINEAR_ALGEBRA_H__
#define __D_LINEAR_ALGEBRA_H__

class dSymmetricBiconjugateGradientSolve
{
	public:
	dSymmetricBiconjugateGradientSolve ();
	~dSymmetricBiconjugateGradientSolve ();	

	dFloat64 Solve (int size, dFloat64 tolerance, dFloat64* const x, const dFloat64* const b) const;

	protected:
	virtual void MatrixTimeVector (dFloat64* const out, const dFloat64* const v) const = 0;
	virtual bool InversePrecoditionerTimeVector (dFloat64* const out, const dFloat64* const v) const = 0;

	private:
	dFloat64 DotProduct (int size, const dFloat64* const b, const dFloat64* const c) const;
	void ScaleAdd (int size, dFloat64* const a, const dFloat64* const b, dFloat64 scale, const dFloat64* const c) const;
	void Sub (int size, dFloat64* const a, const dFloat64* const b, const dFloat64* const c) const;

};



#endif

