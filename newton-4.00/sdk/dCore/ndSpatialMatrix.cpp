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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndGeneralMatrix.h"
#include "ndSpatialMatrix.h"

ndSpatialMatrix ndSpatialMatrix::Inverse(ndInt32 rows) const
{
	ndSpatialMatrix tmp;
	ndSpatialMatrix inv;
	for (ndInt32 i = 0; i < rows; i++) 
	{
		tmp[i] = (*this)[i];
		inv[i] = ndSpatialVector::m_zero;
		inv[i][i] = ndFloat32(1.0f);
	}

	for (ndInt32 i = 0; i < rows; i++) 
	{
		ndFloat64 pivot = dAbs(tmp[i][i]);
		if (pivot < ndFloat64(0.01f)) 
		{
			ndInt32 permute = i;
			for (ndInt32 j = i + 1; j < rows; j++) 
			{
				ndFloat64 pivot1 = dAbs(tmp[j][i]);
				if (pivot1 > pivot) {
					permute = j;
					pivot = pivot1;
				}
			}
			dAssert(pivot > ndFloat32(0.0f));
			dAssert((pivot > ndFloat32(1.0e-6f)) || (dConditionNumber(rows, 6, (ndFloat64*)&m_rows[0]) < ndFloat32(1.0e5f)));
			//if (!((pivot > ndFloat32(1.0e-6f)) || (dConditionNumber(rows, 6, (dFloat64*)&m_rows[0]) < ndFloat32(1.0e5f))))
			//{
			//	for (ndInt32 m = 0; m < rows; m++) {
			//		for (ndInt32 n = 0; n < rows; n++) {
			//			dTrace(("%f ", m_rows[m][n]));
			//		}
			//		dTrace(("\n"));
			//	}
			//	dAssert(0);
			//}

			if (permute != i) 
			{
				for (ndInt32 j = 0; j < rows; j++) 
				{
					dSwap(tmp[i][j], tmp[permute][j]);
					dSwap(tmp[i][j], tmp[permute][j]);
				}
			}
		}

		for (ndInt32 j = i + 1; j < rows; j++) 
		{
			ndFloat64 scale = tmp[j][i] / tmp[i][i];
			tmp[j][i] = ndFloat64(0.0f);
			for (ndInt32 k = i + 1; k < rows; k++) 
			{
				tmp[j][k] -= scale * tmp[i][k];
			}
			for (ndInt32 k = 0; k <= i; k++) 
			{
				inv[j][k] -= scale * inv[i][k];
			}
		}
	}

	for (ndInt32 i = rows - 1; i >= 0; i--) 
	{
		ndSpatialVector acc(ndFloat64(0.0f));
		for (ndInt32 j = i + 1; j < rows; j++) 
		{
			ndFloat64 pivot = tmp[i][j];
			for (ndInt32 k = 0; k < rows; k++) 
			{
				acc[k] += pivot * inv[j][k];
			}
		}
		ndFloat64 den = ndFloat64(1.0f) / tmp[i][i];
		for (ndInt32 k = 0; k < rows; k++) 
		{
			inv[i][k] = den * (inv[i][k] - acc[k]);
		}
	}


#ifdef _DEBUG
	for (ndInt32 i = 0; i < rows; i++) 
	{
		for (ndInt32 j = 0; j < rows; j++) 
		{
			tmp[i][j] = m_rows[j][i];
		}
	}

	for (ndInt32 i = 0; i < rows; i++) 
	{
		ndSpatialVector v(inv.VectorTimeMatrix(tmp[i], rows));
		dAssert(dAbs(v[i] - ndFloat64(1.0f)) < ndFloat64(1.0e-6f));
		for (ndInt32 j = 0; j < rows; j++) 
		{
			if (j != i) 
			{
				dAssert(dAbs(v[j]) < ndFloat64(1.0e-6f));
			}
		}
	}
#endif

	return inv;
}
