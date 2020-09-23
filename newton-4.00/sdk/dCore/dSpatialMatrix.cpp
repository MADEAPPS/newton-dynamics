/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dGeneralMatrix.h"
#include "dSpatialMatrix.h"

dSpatialMatrix dSpatialMatrix::Inverse(dInt32 rows) const
{
	dSpatialMatrix tmp(*this);
	dSpatialMatrix inv(dFloat64(0.0f));
	for (dInt32 i = 0; i < rows; i++) 
	{
		inv[i][i] = dFloat32(1.0f);
	}

	for (dInt32 i = 0; i < rows; i++) 
	{
		dFloat64 pivot = dAbs(tmp[i][i]);
		if (pivot < dFloat64(0.01f)) 
		{
			int permute = i;
			for (dInt32 j = i + 1; j < rows; j++) 
			{
				dFloat64 pivot1 = dAbs(tmp[j][i]);
				if (pivot1 > pivot) {
					permute = j;
					pivot = pivot1;
				}
			}
			dAssert(pivot > dFloat32(0.0f));
			dAssert((pivot > dFloat32(1.0e-6f)) || (dConditionNumber(rows, 6, (dFloat64*)&m_rows[0]) < dFloat32(1.0e5f)));
			//if (!((pivot > dFloat32(1.0e-6f)) || (dConditionNumber(rows, 6, (dFloat64*)&m_rows[0]) < dFloat32(1.0e5f))))
			//{
			//	for (dInt32 m = 0; m < rows; m++) {
			//		for (dInt32 n = 0; n < rows; n++) {
			//			dTrace(("%f ", m_rows[m][n]));
			//		}
			//		dTrace(("\n"));
			//	}
			//	dAssert(0);
			//}

			if (permute != i) 
			{
				for (dInt32 j = 0; j < rows; j++) 
				{
					dSwap(tmp[i][j], tmp[permute][j]);
					dSwap(tmp[i][j], tmp[permute][j]);
				}
			}
		}

		for (dInt32 j = i + 1; j < rows; j++) 
		{
			dFloat64 scale = tmp[j][i] / tmp[i][i];
			tmp[j][i] = dFloat64(0.0f);
			for (int k = i + 1; k < rows; k++) 
			{
				tmp[j][k] -= scale * tmp[i][k];
			}
			for (int k = 0; k <= i; k++) 
			{
				inv[j][k] -= scale * inv[i][k];
			}
		}
	}

	for (dInt32 i = rows - 1; i >= 0; i--) 
	{
		dSpatialVector acc(dFloat64(0.0f));
		for (dInt32 j = i + 1; j < rows; j++) 
		{
			dFloat64 pivot = tmp[i][j];
			for (int k = 0; k < rows; k++) 
			{
				acc[k] += pivot * inv[j][k];
			}
		}
		dFloat64 den = dFloat64(1.0f) / tmp[i][i];
		for (dInt32 k = 0; k < rows; k++) 
		{
			inv[i][k] = den * (inv[i][k] - acc[k]);
		}
	}


#ifdef _DEBUG
	for (dInt32 i = 0; i < rows; i++) 
	{
		for (dInt32 j = 0; j < rows; j++) 
		{
			tmp[i][j] = m_rows[j][i];
		}
	}

	for (dInt32 i = 0; i < rows; i++) 
	{
		dSpatialVector v(inv.VectorTimeMatrix(tmp[i], rows));
		dAssert(dAbs(v[i] - dFloat64(1.0f)) < dFloat64(1.0e-6f));
		for (dInt32 j = 0; j < rows; j++) 
		{
			if (j != i) 
			{
				dAssert(dAbs(v[j]) < dFloat64(1.0e-6f));
			}
		}
	}
#endif

	return inv;
}
