/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgStdafx.h"
#include "dgGeneralMatrix.h"
#include "dgStack.h"
#include "dgMemory.h"

/*
class dgOldSolverNetwon1_5
{
	public:
	dgOldSolverNetwon1_5()
	{
	}

	void SetSize(dgInt32 size)
	{
		m_size = size;
	}

	dgInt32 GetSize() const
	{
		return m_size;
	}

	dgFloat32* GetX()
	{
		return m_x;
	}

	dgFloat32* GetB()
	{
		return m_b;
	}

	dgFloat32* GetLow()
	{
		return m_low;
	}

	dgFloat32* GetHigh()
	{
		return m_high;
	}

	dgInt16* GetFrictionIndex()
	{
		return m_frictionIndex;
	}

	dgFloat32* GetInvDiag()
	{
		return m_invDiag;
	}

	dgFloat32* GetMatrixRow(dgInt32 i)
	{
		return &m_matrix[i * m_size];
	}

	dgFloat32 Solve()
	{
		dgInt32 stride = 0;
		m_x[m_size] = dgFloat32(1.0f);
		dgFloat32 accelNorm = dgFloat32(0.0f);
		for (dgInt32 i = 0; i < m_size; i++) {
			dgVector error(m_b[i]);
			dgFloat32 r = error.GetScalar();
			const dgFloat32* const row = &m_matrix[stride];
			for (dgInt32 j = 0; j < m_size; j++) {
				r += row[j] * m_x[j];
			}
			dgVector x((r + row[i] * m_x[i]) * m_invDiag[i]);
			const dgInt32 frictionIndex = m_frictionIndex[i];
			const dgVector low(m_low[i] * m_x[frictionIndex]);
			const dgVector high(m_high[i] * m_x[frictionIndex]);
			error = error.AndNot((x > high) | (x < low));
			accelNorm += error.GetScalar() * error.GetScalar();

			m_b[i] = r;
			stride += m_size;
		}

		int xxx = 1;
		const dgFloat32 tol2 = dgFloat32(1.0e-5f);
		if (accelNorm > tol2) {
			dgFloat32 accelNorm0 = accelNorm;
			for (dgInt32 i = 0; (i < 6) && (accelNorm0 > tol2); i++) {
				xxx++;
				stride = 0;
				accelNorm0 = dgFloat32(0.0f);
				for (dgInt32 j = 0; j < m_size; j++) {
					const dgFloat32* const row = &m_matrix[stride];
					dgFloat32 r = m_b[j];
					for (dgInt32 k = 0; k < m_size; k++) {
						r = r - row[k] * m_x[k];
					}
					const dgInt32 frictionIndex = m_frictionIndex[j];
					const dgVector low(m_low[j] * m_x[frictionIndex]);
					const dgVector high(m_high[j] * m_x[frictionIndex]);
					dgVector x((r + row[j] * m_x[j]) * m_invDiag[j]);

					dgVector a(r);
					a = a.AndNot((x > high) | (x < low));
					x = x.GetMax(low).GetMin(high);
					m_x[j] = x.GetScalar();

					accelNorm0 += a.GetScalar() * a.GetScalar();
					stride += m_size;
				}
			}

			if (accelNorm0 > tol2) {
				stride = 0;
				xxx++;
				dgFloat32 mask[DG_CONSTRAINT_MAX_ROWS];
				for (dgInt32 i = 0; i < m_size; i++) {
					dgFloat32 r = dgFloat32(0.0f);
					const dgFloat32* const row = &m_matrix[stride];
					for (dgInt32 j = 0; j < m_size; j++) {
						r += row[j] * m_x[j];
					}
					m_b[i] -= r;
					m_delta_x[i] = m_b[i];
					mask[i] = dgFloat32(1.0f);
					const dgInt32 frictionIndex = m_frictionIndex[i];
					m_low[i] *= m_x[frictionIndex];
					m_high[i] *= m_x[frictionIndex];
					stride += m_size;
				}

				dgFloat32 beta = dgFloat32(1.0f);
				//while (beta > tol2) {
				for (dgInt32 k = 0; (k < 20) && (beta > tol2); k++) {
					stride = 0;
					dgFloat32 num = dgFloat32(0.0f);
					dgFloat32 den = dgFloat32(0.0f);
					xxx++;
					for (dgInt32 i = 0; i < m_size; i++) {
						const dgFloat32* const row = &m_matrix[stride];
						dgFloat32 r = dgFloat32(0.0f);
						for (dgInt32 j = 0; j < m_size; j++) {
							r += row[j] * m_delta_x[j];
						}
						stride += m_size;
						m_delta_r[i] = r;
						den += m_delta_x[i] * r;
						num += m_b[i] * m_b[i] * mask[i];
					}

					dgInt32 index = -1;
					dgFloat32 alpha = num / den;
					dgAssert(alpha > dgFloat32(0.0f));

					for (dgInt32 i = 0; (i < m_size) && (alpha > dgFloat32(0.0f)); i++) {

						if (m_delta_x[i]) {
							dgFloat32 x = m_x[i] + alpha * m_delta_x[i];
							if (x < m_low[i]) {
								index = i;
								alpha = (m_low[i] - m_x[i]) / m_delta_x[i];
							} else if (x > m_high[i]) {
								index = i;
								alpha = (m_high[i] - m_x[i]) / m_delta_x[i];
							}
							dgAssert(alpha >= dgFloat32(-1.0e-4f));
							if (alpha < dgFloat32(1.0e-6f)) {
								alpha = dgFloat32(0.0f);
							}
						}
					}

					beta = dgFloat32(0.0f);
					for (dgInt32 i = 0; i < m_size; i++) {
						m_x[i] += alpha * m_delta_x[i];
						m_b[i] -= alpha * m_delta_r[i];
						beta += m_b[i] * m_b[i] * mask[i];
					}

					if (index >= 0) {
						beta = dgFloat32(0.0f);
						mask[index] = dgFloat32(0.0f);
						for (dgInt32 i = 0; i < m_size; i++) {
							m_delta_x[i] = m_b[i] * mask[i];
							beta += m_b[i] * m_b[i] * mask[i];
							stride += m_size;
						}
					} else {
						alpha = beta / num;
						for (dgInt32 i = 0; i < m_size; i++) {
							m_delta_x[i] = m_b[i] * mask[i] + alpha * m_delta_x[i];
						}
					}
				}
			}
		}

		dgTrace(("%d\n", xxx));
		return accelNorm;
	}

	private:
	dgFloat32 m_x[DG_CONSTRAINT_MAX_ROWS + 4];
	dgFloat32 m_b[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_low[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_high[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_invDiag[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_delta_x[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_delta_r[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_matrix[DG_CONSTRAINT_MAX_ROWS * DG_CONSTRAINT_MAX_ROWS];
	dgInt16 m_frictionIndex[DG_CONSTRAINT_MAX_ROWS];
	dgInt32 m_size;
};


class TestSolver_xxxxxxx: public SymmetricBiconjugateGradientSolve
{
	public:
	dgFloat64 a[4][4];

	TestSolver_xxxxxxx()
		:SymmetricBiconjugateGradientSolve()
	{
		dgFloat64 b[] = {1, 2, 3, 4};
		dgFloat64 x[] = {0, 0, 0, 0};
		dgFloat64 c[4];

		memset (a, 0, sizeof (a));
		a[0][0] = 2;
		a[1][1] = 3;
		a[2][2] = 4;
		a[0][3] = 1;
		a[1][3] = 1;
		a[2][3] = 1;
		a[3][0] = 1;
		a[3][1] = 1;
		a[3][2] = 1;


		Solve (4, dgFloat64  (1.0e-10f), x, b);

		MatrixTimeVector (c, x);
		MatrixTimeVector (c, x);
	}

	void MatrixTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		out[0] = a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2] + a[0][3] * v[3];
		out[1] = a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2] + a[1][3] * v[3];
		out[2] = a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2] + a[2][3] * v[3];
		out[3] = a[3][0] * v[0] + a[3][1] * v[1] + a[3][2] * v[2] + a[3][3] * v[3];
	}

	void InversePrecoditionerTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		out[0] = v[0]/a[0][0];
		out[1] = v[1]/a[1][1];
		out[2] = v[2]/a[2][2];
		out[3] = v[3];
	}
};
*/


dgSymmetricBiconjugateGradientSolve::dgSymmetricBiconjugateGradientSolve ()
{
}

dgSymmetricBiconjugateGradientSolve::~dgSymmetricBiconjugateGradientSolve ()
{
}


void dgSymmetricBiconjugateGradientSolve::ScaleAdd (dgInt32 size, dgFloat64* const a, const dgFloat64* const b, dgFloat64 scale, const dgFloat64* const c) const
{
	for (dgInt32 i = 0; i < size; i ++) {
		a[i] = b[i] + scale * c[i];
	}
}

void dgSymmetricBiconjugateGradientSolve::Sub (dgInt32 size, dgFloat64* const a, const dgFloat64* const b, const dgFloat64* const c) const
{
	for (dgInt32 i = 0; i < size; i ++) {
		a[i] = b[i] - c[i];
	}
}

dgFloat64 dgSymmetricBiconjugateGradientSolve::DotProduct (dgInt32 size, const dgFloat64* const b, const dgFloat64* const c) const
{
	dgFloat64 product = dgFloat64 (0.0f);
	for (dgInt32 i = 0; i < size; i ++) {
		product += b[i] * c[i];
	}
	return product;
}

dgFloat64 dgSymmetricBiconjugateGradientSolve::Solve (dgInt32 size, dgFloat64 tolerance, dgFloat64* const x, const dgFloat64* const b) const
{
	dgStack<dgFloat64> bufferR0(size);
	dgStack<dgFloat64> bufferP0(size);
	dgStack<dgFloat64> matrixTimesP0(size);
	dgStack<dgFloat64> bufferConditionerInverseTimesR0(size);

	dgFloat64* const r0 = &bufferR0[0];
	dgFloat64* const p0 = &bufferP0[0];
	dgFloat64* const MinvR0 = &bufferConditionerInverseTimesR0[0];
	dgFloat64* const matrixP0 = &matrixTimesP0[0];

	MatrixTimeVector (matrixP0, x);
	Sub(size, r0, b, matrixP0);
	bool continueExecution = InversePrecoditionerTimeVector (p0, r0);

	dgInt32 iter = 0;
	dgFloat64 num = DotProduct (size, r0, p0);
	dgFloat64 error2 = num;
	for (dgInt32 j = 0; (j < size) && (error2 > tolerance) && continueExecution; j ++) {

		MatrixTimeVector (matrixP0, p0);
		dgFloat64 den = DotProduct (size, p0, matrixP0);

		dgAssert (fabs(den) > dgFloat64 (0.0f));
		dgFloat64 alpha = num / den;

		ScaleAdd (size, x, x, alpha, p0);
        if ((j % 50) != 49) {
		    ScaleAdd (size, r0, r0, -alpha, matrixP0);
        } else {
            MatrixTimeVector (matrixP0, x);
            Sub(size, r0, b, matrixP0);
        }

//dgUnsigned64 xxx0 = dgGetTimeInMicrosenconds();
		continueExecution = InversePrecoditionerTimeVector (MinvR0, r0);
//xxx0 = dgGetTimeInMicrosenconds() - xxx0;
//dgTrace (("%d\n", dgUnsigned64 (xxx0)));


		dgFloat64 num1 = DotProduct (size, r0, MinvR0);
		dgFloat64 beta = num1 / num;
		ScaleAdd (size, p0, MinvR0, beta, p0);
		num = DotProduct (size, r0, MinvR0);
		iter ++;
		error2 = num;
		if (j > 10) {
			error2 = dgFloat64 (0.0f);
			for (dgInt32 i = 0; i < size; i ++) {
				error2 = dgMax (error2, r0[i] * r0[i]);
			}
		}
	}

	dgAssert (iter < size);
	return num;
}



DG_INLINE bool dgCholeskyFactorizationAddRow(dgInt32 size, dgInt32 n, dgFloat32* const matrix, dgInt32 rowStride)
{
	dgFloat32* const rowN = &matrix[rowStride * n];
	dgCheckAligment(rowN);

	dgInt32 stride = 0;
	for (dgInt32 j = 0; j <= n; j++) {
		dgFloat32 s = dgFloat32(0.0f);
		dgFloat32* const rowJ = &matrix[stride];
		dgCheckAligment(rowJ);
		for (dgInt32 k = 0; k < j; k++) {
			s += rowN[k] * rowJ[k];
		}

		if (n == j) {
			dgFloat32 diag = rowN[n] - s;
			if (diag < dgFloat32(dgFloat32(1.0e-6f))) {
				return false;
			}

			rowN[n] = dgFloat32(sqrt(diag));
		} else {
			rowN[j] = (rowN[j] - s) / rowJ[j];
		}

		stride += rowStride;
	}

	return true;
}


bool dgCholeskyFactorization(dgInt32 size, dgFloat32* const psdMatrix, dgInt32 rowStride)
{
	bool state = true;
	for (dgInt32 i = 0; (i < size) && state; i++) {
		state = state && dgCholeskyFactorizationAddRow(size, i, psdMatrix, rowStride);
	}
	return state;
}
