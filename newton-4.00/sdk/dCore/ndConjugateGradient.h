
/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#define D_USE_JACOBI_PRECONDITIONER

template<class T>
class ndDefaultMatrixOperator
{
	public:
	ndDefaultMatrixOperator(ndInt32 size, const T* const matrix, T* const preconditonerBuffer)
		:m_matrix(matrix)
		,m_preconditoner(preconditonerBuffer)
		,m_size(size)
	{
#ifdef D_USE_JACOBI_PRECONDITIONER
		// use Jacobi preconditiner
		const T* row = m_matrix;
		for (ndInt32 i = 0; i < size; i++)
		{
			m_preconditoner[i] = T(1.0f) / row[i];
			row += m_size;
		}
#endif

		// trying Gauss sidel preconditoner later, but does no seem to get any better reasult than Jacobi.
		//memcpy(A, matrix, sizeof(A));
		//
		//dCholeskyFactorization(size, size, &A[0][0]);
		//for (ndInt32 i = 2; i < 6; i++)
		//{
		//	for (ndInt32 j = 0; j < i - 1; j++)
		//	{
		//		A[i][j] = 0.0f;
		//	}
		//}

		//for (ndInt32 i = 0; i < 6; i++)
		//{
		//	for (ndInt32 j = i + 2; j < 6; j++)
		//	{
		//		A[i][j] = 0.0f;
		//		A[j][i] = 0.0f;
		//	}
		//}
		//
		//for (ndInt32 i = 0; i < 5; i++)
		//{
		//	T val = dAbs(A[i][i] * T(0.99f));
		//	if (val < dAbs(A[i][i + 1]))
		//	{
		//		//val *= dSign(A[i][i + 1]);
		//		val = 0;
		//		A[i][i + 1] = val;
		//		A[i + 1][i] = val;
		//		//A[i][i + 1] = 0;
		//		//A[i + 1][i] = 0;
		//	}
		//}
		//
		//for (ndInt32 i = 1; i < 6; i++)
		//{
		//	T val = dAbs(A[i][i] * T(0.99f));
		//	if (val < dAbs(A[i][i - 1]))
		//	{
		//		//val *= dSign(A[i][i - 1]);
		//		val = 0;
		//		A[i][i - 1] = val;
		//		A[i - 1][i] = val;
		//	}
		//}
		//
		//
		//dCholeskyFactorization(size, size, &A[0][0]);
	}

	void PreconditionerSolve(const T* const input, T* const output)
	{
#ifdef D_USE_JACOBI_PRECONDITIONER
		for (ndInt32 i = 0; i < m_size; i++)
		{
			output[i] = input[i] * m_preconditoner[i];
		}
#endif
		//dSolveCholesky(m_size, m_size, &A[0][0], output, input);
	}

	void MatrixTimeVector(const T* const input, T* const output)
	{
		dMatrixTimeVector(m_size, m_matrix, input, output);
	}

	const T* m_matrix;
	T* m_preconditoner;
	ndInt32 m_size;

	T A[6][6];
};

template<class T, class ndMatrixOperator = ndDefaultMatrixOperator<T>>
class ndConjugateGradient : public ndClassAlloc
{
	public:
	ndConjugateGradient();
	ndConjugateGradient(T* const r0, T* const z0, T* const p0, T* const q0);
	~ndConjugateGradient();

	void SetBuffers(T* const r0, T* const z0, T* const p0, T* const q0);
	T Solve(ndInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer);

	private:
	T SolveInternal(ndInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer) const;

	T* m_r0;
	T* m_z0;
	T* m_p0;
	T* m_q0;
};

template<class T, class ndMatrixOperator>
ndConjugateGradient<T, ndMatrixOperator>::ndConjugateGradient()
{
	SetBuffers(nullptr, nullptr, nullptr, nullptr);
}

template<class T, class ndMatrixOperator>
ndConjugateGradient<T, ndMatrixOperator>::ndConjugateGradient(T* const r0, T* const z0, T* const p0, T* const q0)
{
	SetBuffers(r0, z0, p0, q0);
}

template<class T, class ndMatrixOperator>
ndConjugateGradient<T, ndMatrixOperator>::~ndConjugateGradient()
{
}

template<class T, class ndMatrixOperator>
void ndConjugateGradient<T, ndMatrixOperator>::SetBuffers(T* const r0, T* const z0, T* const p0, T* const q0)
{
	m_r0 = r0;
	m_z0 = z0;
	m_p0 = p0;
	m_q0 = q0;
}

template<class T, class ndMatrixOperator>
T ndConjugateGradient<T, ndMatrixOperator>::Solve(ndInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer)
{
	if (m_r0) 
	{
		return SolveInternal(size, tolerance, x, b, matrix, preconditionerBuffer);
	} 
	else 
	{
		T* const r0 = dAlloca(T, size);
		T* const z0 = dAlloca(T, size);
		T* const p0 = dAlloca(T, size);
		T* const q0 = dAlloca(T, size);
		SetBuffers(r0, z0, p0, q0);
		T error = SolveInternal(size, tolerance, x, b, matrix, preconditionerBuffer);
		SetBuffers(nullptr, nullptr, nullptr, nullptr);
		return error;
	}
}

template<class T, class ndMatrixOperator>
T ndConjugateGradient<T, ndMatrixOperator>::SolveInternal(ndInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer) const
{
	ndMatrixOperator matrixOper(size, matrix, preconditionerBuffer);

	matrixOper.MatrixTimeVector(x, m_z0);
	dSub(size, m_r0, b, m_z0);
	matrixOper.PreconditionerSolve(m_r0, m_p0);
	
	ndInt32 iter = 0;
	T num = dDotProduct(size, m_r0, m_p0);
	T error2 = num;
	for (ndInt32 j = 0; (j < size) && (error2 > tolerance); j++) 
	{
		matrixOper.MatrixTimeVector(m_p0, m_z0);
		T den = dDotProduct(size, m_p0, m_z0);
	
		dAssert(fabs(den) > T(0.0f));
		T alpha = num / den;
	
		dMulAdd(size, x, x, m_p0, alpha);
		if ((j % 50) != 49) 
		{
			dMulAdd(size, m_r0, m_r0, m_z0, -alpha);
		} 
		else 
		{
			matrixOper.MatrixTimeVector(x, m_z0);
			dSub(size, m_r0, b, m_z0);
		}
	
		matrixOper.PreconditionerSolve(m_r0, m_q0);
	
		T num1 = dDotProduct(size, m_r0, m_q0);
		T beta = num1 / num;
		dMulAdd(size, m_p0, m_q0, m_p0, beta);
		num = dDotProduct(size, m_r0, m_q0);
		iter++;
		error2 = num;
	}
	dAssert(iter <= size);
	return num;
}

#endif
