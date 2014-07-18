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
#include "dLinearAlgebra.h"


dSymmetricBiconjugateGradientSolve::dSymmetricBiconjugateGradientSolve ()
{
}

dSymmetricBiconjugateGradientSolve::~dSymmetricBiconjugateGradientSolve ()
{
}


void dSymmetricBiconjugateGradientSolve::ScaleAdd (int size, dFloat64* const a, const dFloat64* const b, dFloat64 scale, const dFloat64* const c) const
{
	for (int i = 0; i < size; i ++) {
		a[i] = b[i] + scale * c[i];
	}
}

void dSymmetricBiconjugateGradientSolve::Sub (int size, dFloat64* const a, const dFloat64* const b, const dFloat64* const c) const
{
	for (int i = 0; i < size; i ++) {
		a[i] = b[i] - c[i];
	}
}

dFloat64 dSymmetricBiconjugateGradientSolve::DotProduct (int size, const dFloat64* const b, const dFloat64* const c) const
{
	dFloat64 product = dFloat64 (0.0f);
	for (int i = 0; i < size; i ++) {
		product += b[i] * c[i];
	}
	return product;
}

dFloat64 dSymmetricBiconjugateGradientSolve::Solve (int size, dFloat64 tolerance, dFloat64* const x, const dFloat64* const b) const
{
//	dgStack<dFloat64> bufferR0(size);
//	dgStack<dFloat64> bufferP0(size);
//	dgStack<dFloat64> matrixTimesP0(size);
//	dgStack<dFloat64> bufferConditionerInverseTimesR0(size);
//	dFloat64* const r0 = &bufferR0[0];
//	dFloat64* const p0 = &bufferP0[0];
//	dFloat64* const MinvR0 = &bufferConditionerInverseTimesR0[0];
//	dFloat64* const matrixP0 = &matrixTimesP0[0];

	dFloat64* const r0 = new dFloat64 [size];
	dFloat64* const p0 = new dFloat64 [size];
	dFloat64* const MinvR0 = new dFloat64 [size];
	dFloat64* const matrixP0 = new dFloat64 [size];

	MatrixTimeVector (matrixP0, x);
	Sub(size, r0, b, matrixP0);
	bool continueExecution = InversePrecoditionerTimeVector (p0, r0);

	int iter = 0;
	dFloat64 num = DotProduct (size, r0, p0);
	dFloat64 error2 = num;
	for (int j = 0; (j < size) && (error2 > tolerance) && continueExecution; j ++) {

		MatrixTimeVector (matrixP0, p0);
		dFloat64 den = DotProduct (size, p0, matrixP0);

		dAssert (fabs(den) > dFloat64 (0.0f));
		dFloat64 alpha = num / den;

		ScaleAdd (size, x, x, alpha, p0);
        if ((j % 50) != 49) {
		    ScaleAdd (size, r0, r0, -alpha, matrixP0);
        } else {
            MatrixTimeVector (matrixP0, x);
            Sub(size, r0, b, matrixP0);
        }

		continueExecution = InversePrecoditionerTimeVector (MinvR0, r0);

		dFloat64 num1 = DotProduct (size, r0, MinvR0);
		dFloat64 beta = num1 / num;
		ScaleAdd (size, p0, MinvR0, beta, p0);
		num = DotProduct (size, r0, MinvR0);
		iter ++;
		error2 = num;
		if (j > 10) {
			error2 = dFloat64 (0.0f);
			for (int i = 0; i < size; i ++) {
				error2 = dMax (error2, r0[i] * r0[i]);
			}
		}
	}

	
	

	delete[] matrixP0;
	delete[] MinvR0;
	delete[] p0;
	delete[] r0;

	dAssert (iter < size);
	return num;
}
