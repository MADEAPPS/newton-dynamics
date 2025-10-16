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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndGeneralMatrix.h"
#include "ndSpatialMatrix.h"

ndSpatialMatrix ndSpatialMatrix::CholeskyFactorization(ndInt32 rows) const
{
#ifdef _DEBUG
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndAssert(m_rows[i][i] > ndFloat64(0.0f));
		for (ndInt32 j = i + 1; j < rows; ++j)
		{
			ndFloat64 a = m_rows[i][j];
			ndFloat64 b = m_rows[j][i];
			ndAssert(ndAbs(a - b) < ndFloat64(1.0e-6f));
		}
	}
#endif

	ndSpatialMatrix psdMatrix(*this);
	psdMatrix[0][0] = ndFloat64(sqrt(psdMatrix[0][0]));
	for (ndInt32 j = 1; j < rows; ++j)
	{
		ndFloat64 s = ndFloat64(0.0f);
		for (ndInt32 k = 0; k < j; ++k)
		{
			s += psdMatrix[j][k] * psdMatrix[j][k];
		}
		ndFloat64 d = psdMatrix[j][j];
		psdMatrix[j][j] = ndFloat64(sqrt(d - s));
		for (ndInt32 i = j + 1; i < rows; ++i)
		{
			s = ndFloat64(0.0f);
			for (ndInt32 k = 0; k < j; ++k)
			{
				s += psdMatrix[i][k] * psdMatrix[j][k];
			}
			psdMatrix[i][j] = (psdMatrix[i][j] - s) / psdMatrix[j][j];
			psdMatrix[j][i] = ndFloat64(0.0f);
		}
	}

#ifdef _DEBUG
	ndSpatialMatrix psdMatrix1(*this);
	ndCholeskyFactorization(rows, 8, &psdMatrix1[0][0]);

	for (ndInt32 i = 0; i < rows; ++i)
	{
		for (ndInt32 j = 0; j <= i; ++j)
		{
			ndFloat64 a = psdMatrix.m_rows[i][j];
			ndFloat64 b = psdMatrix.m_rows[i][j];
			ndAssert(ndAbs(a - b) < ndFloat64(1.0e-6f));
		}
	}
#endif

	return psdMatrix;
}

ndSpatialMatrix ndSpatialMatrix::Inverse(ndInt32 rows) const
{
	ndSpatialMatrix tmp;
	ndSpatialMatrix inv;
	for (ndInt32 i = 0; i < rows; ++i) 
	{
		tmp[i] = m_rows[i];
		inv[i] = ndSpatialVector::m_zero;
		inv[i][i] = ndFloat32(1.0f);
	}

	for (ndInt32 i = 0; i < rows; ++i) 
	{
		ndFloat64 pivot = ndAbs(tmp[i][i]);
		if (pivot < ndFloat64(0.01f)) 
		{
			ndInt32 permute = i;
			for (ndInt32 j = i + 1; j < rows; ++j) 
			{
				ndFloat64 pivot1 = ndAbs(tmp[j][i]);
				if (pivot1 > pivot) {
					permute = j;
					pivot = pivot1;
				}
			}
			ndAssert(pivot > ndFloat32(0.0f));
			ndAssert((pivot > ndFloat32(1.0e-6f)) || (ndConditionNumber(rows, 6, (ndFloat64*)&m_rows[0]) < ndFloat32(1.0e5f)));
			if (permute != i) 
			{
				for (ndInt32 j = 0; j < rows; ++j) 
				{
					ndSwap(tmp[i][j], tmp[permute][j]);
					ndSwap(tmp[i][j], tmp[permute][j]);
				}
			}
		}

		for (ndInt32 j = i + 1; j < rows; ++j) 
		{
			ndFloat64 scale = tmp[j][i] / tmp[i][i];
			tmp[j][i] = ndFloat64(0.0f);
			for (ndInt32 k = i + 1; k < rows; ++k) 
			{
				tmp[j][k] -= scale * tmp[i][k];
			}
			for (ndInt32 k = 0; k <= i; ++k) 
			{
				inv[j][k] -= scale * inv[i][k];
			}
		}
	}

	for (ndInt32 i = rows - 1; i >= 0; i--) 
	{
		ndSpatialVector acc(ndFloat64(0.0f));
		for (ndInt32 j = i + 1; j < rows; ++j) 
		{
			ndFloat64 pivot = tmp[i][j];
			for (ndInt32 k = 0; k < rows; ++k) 
			{
				acc[k] += pivot * inv[j][k];
			}
		}
		ndFloat64 den = ndFloat64(1.0f) / tmp[i][i];
		for (ndInt32 k = 0; k < rows; ++k) 
		{
			inv[i][k] = den * (inv[i][k] - acc[k]);
		}
	}

#ifdef _DEBUG
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndFloat64 sum = ndFloat32(0.0f);
		for (ndInt32 k = 0; k < rows; ++k)
		{
			sum += m_rows[i][k] * inv[k][i];
		}
		ndAssert(ndAbs(sum - ndFloat64(1.0f)) < ndFloat64(1.0e-6f));

		for (ndInt32 j = i + 1; j < rows; ++j)
		{
			sum = ndFloat32(0.0f);
			for (ndInt32 k = 0; k < rows; ++k)
			{
				sum += m_rows[i][k] * inv[k][j];
			}
			ndAssert(ndAbs(sum) < ndFloat64(1.0e-6f));
		}
	}
#endif

	return inv;
}


ndSpatialMatrix ndSpatialMatrix::InversePositiveDefinite(ndInt32 rows) const
{
	const ndSpatialMatrix lower(CholeskyFactorization(rows));
	const ndSpatialMatrix lowerInverse(lower.Inverse(rows));

	ndSpatialMatrix inv;
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndFloat64 sum = ndFloat32(0.0f);
		for (ndInt32 k = i; k < rows; ++k)
		{
			sum += lowerInverse[k][i] * lowerInverse[k][i];
		}
		inv[i][i] = sum;

		for (ndInt32 j = i + 1; j < rows; ++j)
		{
			sum = ndFloat32(0.0f);
			for (ndInt32 k = j; k < rows; ++k)
			{
				sum += lowerInverse[k][i] * lowerInverse[k][j];
			}
			inv[i][j] = sum;
			inv[j][i] = sum;
		}
	}

#ifdef _DEBUG
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndFloat64 sum = ndFloat32(0.0f);
		for (ndInt32 k = 0; k < rows; ++k)
		{
			sum += m_rows[i][k] * inv[k][i];
		}
		ndAssert(ndAbs(sum - ndFloat64(1.0f)) < ndFloat64(1.0e-6f));

		for (ndInt32 j = i + 1; j < rows; ++j)
		{
			sum = ndFloat32(0.0f);
			for (ndInt32 k = 0; k < rows; ++k)
			{
				sum += m_rows[i][k] * inv[k][j];
			}
			ndAssert(ndAbs(sum) < ndFloat64(1.0e-6f));
		}
	}
#endif

	return inv;
}