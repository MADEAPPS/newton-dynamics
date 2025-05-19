
/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_CONJUGATE_GRADIENT_H__
#define __ND_CONJUGATE_GRADIENT_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndClassAlloc.h"
#include "ndGeneralVector.h"
#include "ndGeneralMatrix.h"

template<class T>
class ndConjugateGradient : public ndClassAlloc
{
	public:
	ndConjugateGradient(bool usePreconditioner = true);
	~ndConjugateGradient();
	T Solve(ndInt32 size, ndInt32 stride, T tolerance, T* const x, const T* const b, const T* const matrix) const;

	bool m_usedPreconditioner;
};

template<class T>
ndConjugateGradient<T>::ndConjugateGradient(bool usePreconditioner)
	:m_usedPreconditioner(usePreconditioner)
{
}

template<class T>
ndConjugateGradient<T>::~ndConjugateGradient()
{
}

template<class T>
T ndConjugateGradient<T>::Solve(ndInt32 size, ndInt32 stride, T tolerance, T* const x, const T* const b, const T* const matrix) const
{
	T* const r = ndAlloca(T, size);
	T* const p = ndAlloca(T, size);
	T* const q = ndAlloca(T, size);
	T* const tmp = ndAlloca(T, size);
	T* const preconditioner = ndAlloca(T, size);
	T* const bPreconditioned = ndAlloca(T, size);
	T* const xPreconditioned = ndAlloca(T, size);


	ndMemCpy(bPreconditioned, b, size);
	ndMemCpy(xPreconditioned, x, size);
	ndMemSet(preconditioner, T(1.0f), size);

	// naive preconditioner but very effective.
	if (m_usedPreconditioner)
	{
		for (ndInt32 i = 0; i < size; ++i)
		{
			const T* const row = &matrix[i * stride];
			T diag = row[i];
			T offDiagDominant = ndFloat32 (0.0f);
			for (ndInt32 j = 0; j < i; ++j)
			{
				offDiagDominant = ndMax(offDiagDominant, ndAbs(row[j]));
			}
			for (ndInt32 j = i + 1; j < size; ++j)
			{
				offDiagDominant = ndMax(offDiagDominant, ndAbs(row[j]));
			}
			if (diag <= offDiagDominant)
			{
				T precond = T(1.0f) / ndSqrt(diag);
				preconditioner[i] = precond;
				bPreconditioned[i] = precond * b[i];
				xPreconditioned[i] = x[i] / precond;
			}
		}
	}

	auto StartResidual = [size, stride, r, p, q, tmp, matrix, xPreconditioned, bPreconditioned, preconditioner]()
	{
		ndMul<T>(size, tmp, xPreconditioned, preconditioner);
		ndMatrixTimeVector<T>(size, stride, matrix, tmp, q);
		ndMul<T>(size, q, preconditioner);
		ndSub<T>(size, r, bPreconditioned, q);
		ndMemCpy(p, r, size);
		return ndDotProduct<T>(size, r, r);
	};

	T num = StartResidual();
	for (ndInt32 j = 0; (j < size) && (num > tolerance); ++j)
	{
		if ((j % 32) == 31)
		{
			ndAssert(0);
			// restart after many iterations because of numeric drift
			num = StartResidual();
		}

		ndMul<T>(size, tmp, p, preconditioner);
		ndMatrixTimeVector<T>(size, stride, matrix, tmp, q);
		ndMul<T>(size, q, preconditioner);
		T den = ndDotProduct<T>(size, p, q);
		T alpha = num / den;

		ndScaleAdd(size, r, q, -alpha);
		ndScaleAdd(size, xPreconditioned, p, alpha);

		den = num;
		num = ndDotProduct<T>(size, r, r);
		if (num > tolerance)
		{
			T beta = num / den;
			ndScale(size, p, beta);
			ndAdd(size, p, r);
		}
	}

	ndMul<T>(size, x, xPreconditioned, preconditioner);
	return num;
}

#endif
