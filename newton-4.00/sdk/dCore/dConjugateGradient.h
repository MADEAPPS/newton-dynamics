
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

#ifndef __D_CONJUGATE_GRADIENT_H__
#define __D_CONJUGATE_GRADIENT_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dUtils.h"
#include "dClassAlloc.h"
#include "dGeneralVector.h"
#include "dGeneralMatrix.h"

template<class T>
class dDefaultMatrixOperator
{
	public:
	dDefaultMatrixOperator(dInt32 size, const T* const matrix, T* const preconditonerBuffer)
		:m_matrix(matrix)
		,m_preconditoner(preconditonerBuffer)
		,m_size(size)
	{
	}

	void MatrixTimeVector(const T* const input, T* const output)
	{
		dMatrixTimeVector(m_size, m_matrix, input, output);
	}

	void PreconditionerSolve(const T* const input, T* const output)
	{
		const T* row = m_matrix;
		for (dInt32 i = 0; i < m_size; i++)
		{
			output[i] = input[i]/row[i];
			row += m_size;
		}
	}

	const T* m_matrix;
	T* m_preconditoner;
	dInt32 m_size;
};

template<class T, class dMatrixOperator = dDefaultMatrixOperator<T>>
class dConjugateGradient : public dClassAlloc
{
	public:
	dConjugateGradient();
	dConjugateGradient(T* const r0, T* const z0, T* const p0, T* const q0);
	~dConjugateGradient();

	void SetBuffers(T* const r0, T* const z0, T* const p0, T* const q0);
	T Solve(dInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer);

	private:
	T SolveInternal(dInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer) const;

	T* m_r0;
	T* m_z0;
	T* m_p0;
	T* m_q0;
};

template<class T, class dMatrixOperator>
dConjugateGradient<T, dMatrixOperator>::dConjugateGradient()
{
	SetBuffers(nullptr, nullptr, nullptr, nullptr);
}

template<class T, class dMatrixOperator>
dConjugateGradient<T, dMatrixOperator>::dConjugateGradient(T* const r0, T* const z0, T* const p0, T* const q0)
{
	SetBuffers(r0, z0, p0, q0);
}

template<class T, class dMatrixOperator>
dConjugateGradient<T, dMatrixOperator>::~dConjugateGradient()
{
}

template<class T, class dMatrixOperator>
void dConjugateGradient<T, dMatrixOperator>::SetBuffers(T* const r0, T* const z0, T* const p0, T* const q0)
{
	m_r0 = r0;
	m_z0 = z0;
	m_p0 = p0;
	m_q0 = q0;
}

template<class T, class dMatrixOperator>
T dConjugateGradient<T, dMatrixOperator>::Solve(dInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer)
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

template<class T, class dMatrixOperator>
T dConjugateGradient<T, dMatrixOperator>::SolveInternal(dInt32 size, T tolerance, T* const x, const T* const b, const T* const matrix, T* const preconditionerBuffer) const
{
	dMatrixOperator matrixOper(size, matrix, preconditionerBuffer);

	matrixOper.MatrixTimeVector(x, m_z0);
	dSub(size, m_r0, b, m_z0);
	matrixOper.PreconditionerSolve(m_r0, m_p0);
	
	dInt32 iter = 0;
	T num = dDotProduct(size, m_r0, m_p0);
	T error2 = num;
	for (dInt32 j = 0; (j < size) && (error2 > tolerance); j++) 
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
