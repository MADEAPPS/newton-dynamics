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
#include "ndGoogol.h"
#include "ndSmallDeterminant.h"

#define Absolute(a)  ((a) >= 0.0 ? (a) : -(a))

ndFloat64 Determinant2x2 (const ndFloat64 matrix[2][2], ndFloat64* const error)
{
	ndFloat64 a00xa11 = matrix[0][0] * matrix[1][1];
	ndFloat64 a01xa10 = matrix[0][1] * matrix[1][0];
	*error = Absolute(a00xa11) + Absolute(a01xa10);
	return a00xa11 - a01xa10;
}

ndGoogol Determinant2x2 (const ndGoogol matrix[2][2])
{
	ndGoogol a00xa11 (matrix[0][0] * matrix[1][1]);
	ndGoogol a01xa10 (matrix[0][1] * matrix[1][0]);
	return a00xa11 - a01xa10;
}

ndFloat64 Determinant3x3 (const ndFloat64 matrix[3][3], ndFloat64* const error)
{
	ndFloat64 sign = ndFloat64 (-1.0f);
	ndFloat64 det = ndFloat64 (0.0f);
	ndFloat64 accError = ndFloat64 (0.0f); 
	for (ndInt32 i = 0; i < 3; ++i)  
	{
		ndFloat64 cofactor[2][2];
		for (ndInt32 j = 0; j < 2; ++j) 
		{
			ndInt32 k0 = 0;
			for (ndInt32 k = 0; k < 3; ++k) 
			{
				if (k != i) 
				{
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		ndFloat64 parcialError;
		ndFloat64 minorDet = Determinant2x2 (cofactor, &parcialError);
		accError += parcialError * Absolute (matrix[2][i]);
		det += sign * minorDet * matrix[2][i];
		sign *= ndFloat64 (-1.0f);
	}

	*error = accError;
	return det;
}

ndGoogol Determinant3x3 (const ndGoogol matrix[3][3])
{
	ndGoogol negOne (ndFloat64 (-1.0f));
	ndGoogol sign (ndFloat64 (-1.0f));
	ndGoogol det = ndFloat64 (0.0f);
	for (ndInt32 i = 0; i < 3; ++i)  
	{
		ndGoogol cofactor[2][2];
		for (ndInt32 j = 0; j < 2; ++j) 
		{
			ndInt32 k0 = 0;
			for (ndInt32 k = 0; k < 3; ++k) 
			{
				if (k != i) 
				{
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		ndGoogol minorDet (Determinant2x2 (cofactor));
		det = det + sign * minorDet * matrix[2][i];
		sign = sign * negOne;
	}
	return det;
}

ndFloat64 Determinant4x4 (const ndFloat64 matrix[4][4], ndFloat64* const error)
{
	ndFloat64 sign = ndFloat64 (1.0f);
	ndFloat64 det = ndFloat64 (0.0f);
	ndFloat64 accError = ndFloat64 (0.0f); 
	for (ndInt32 i = 0; i < 4; ++i)  
	{
		ndFloat64 cofactor[3][3];
		for (ndInt32 j = 0; j < 3; ++j) 
		{
			ndInt32 k0 = 0;
			for (ndInt32 k = 0; k < 4; ++k) 
			{
				if (k != i) {
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		ndFloat64 parcialError;
		ndFloat64 minorDet = Determinant3x3 (cofactor, &parcialError);
		accError +=  parcialError * Absolute (matrix[3][i]);
		det += sign * minorDet * matrix[3][i];
		sign *= ndFloat64 (-1.0f);
	}

	*error = accError;
	return det;
}

ndGoogol Determinant4x4 (const ndGoogol matrix[4][4])
{
	ndGoogol sign = ndFloat64 (1.0f);
	ndGoogol det = ndFloat64 (0.0f);
	ndGoogol negOne (ndFloat64 (-1.0f));
	//dGoogol accError = ndFloat64 (0.0f);
	for (ndInt32 i = 0; i < 4; ++i)  
	{
		ndGoogol  cofactor[3][3];
		for (ndInt32 j = 0; j < 3; ++j) 
		{
			ndInt32 k0 = 0;
			for (ndInt32 k = 0; k < 4; ++k) 
			{
				if (k != i) 
				{
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		ndGoogol minorDet = Determinant3x3 (cofactor);
		det = det + sign * minorDet * matrix[3][i];
		sign = sign * negOne;
	}
	return det;
}


