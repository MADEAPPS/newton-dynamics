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

#include "dgStdafx.h"
#include "dgGeneralMatrix.h"
#include "dgStack.h"
#include "dgMemory.h"

/*
DG_INLINE bool dgCholeskyFactorizationAddRow(dgInt32 size, dgInt32 n, dgFloat32* const matrix, dgInt32 rowStride)
{
	dgFloat32* const rowN = &matrix[rowStride * n];

	dgInt32 stride = 0;
	for (dgInt32 j = 0; j <= n; j++) {
		dgFloat32 s = dgFloat32(0.0f);
		dgFloat32* const rowJ = &matrix[stride];
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
*/