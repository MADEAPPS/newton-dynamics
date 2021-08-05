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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dSpatialVector.h"

#ifndef D_NEWTON_USE_DOUBLE

	dVector dVector::m_xMask(dInt32(-1), dInt32(0), dInt32(0), dInt32(0));
	dVector dVector::m_yMask(dInt32(0), dInt32(-1), dInt32(0), dInt32(0));
	dVector dVector::m_zMask(dInt32(0), dInt32(0), dInt32(-1), dInt32(0));
	dVector dVector::m_wMask(dInt32(0), dInt32(0), dInt32(0), dInt32(-1));
	dVector dVector::m_xyzwMask(dInt32(-1), dInt32(-1), dInt32(-1), dInt32(-1));
	dVector dVector::m_triplexMask(dInt32(-1), dInt32(-1), dInt32(-1), dInt32(0));
	dVector dVector::m_signMask(dVector(dInt32(-1), dInt32(-1), dInt32(-1), dInt32(-1)).ShiftRightLogical(1));

	dVector dVector::m_zero(dFloat32(0.0f));
	dVector dVector::m_one(dFloat32(1.0f));
	dVector dVector::m_two(dFloat32(2.0f));
	dVector dVector::m_half(dFloat32(0.5f));
	dVector dVector::m_three(dFloat32(3.0f));
	dVector dVector::m_negOne(dFloat32(-1.0f));
	dVector dVector::m_epsilon(dFloat32(1.0e-20f));
	dVector dVector::m_wOne(dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f));
#endif

dBigVector dBigVector::m_zero (dFloat64 (0.0f));
dBigVector dBigVector::m_one (dFloat64 (1.0f));
dBigVector dBigVector::m_two (dFloat64 (2.0f));
dBigVector dBigVector::m_half (dFloat32 (0.5f));
dBigVector dBigVector::m_three (dFloat32 (3.0f));
dBigVector dBigVector::m_negOne (dFloat32 (-1.0f));
dBigVector dBigVector::m_epsilon(dFloat32(1.0e-20f));
dBigVector dBigVector::m_wOne (dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (1.0f));
dBigVector dBigVector::m_triplexMask (dInt32 (-1), dInt32 (-1),	dInt32 (-1), dInt32 (0));
dBigVector dBigVector::m_signMask (dBigVector(dInt32 (-1), dInt32 (-1), dInt32 (-1), dInt32 (-1)).ShiftRightLogical(1));

dBigVector dBigVector::m_xMask (dInt32 (-1), dInt32 ( 0),	dInt32 ( 0), dInt32 ( 0));
dBigVector dBigVector::m_yMask (dInt32 ( 0), dInt32 (-1),	dInt32 ( 0), dInt32 ( 0));
dBigVector dBigVector::m_zMask (dInt32 ( 0), dInt32 ( 0),	dInt32 (-1), dInt32 ( 0));
dBigVector dBigVector::m_wMask (dInt32 ( 0), dInt32 ( 0),	dInt32 ( 0), dInt32 (-1));
dBigVector dBigVector::m_xyzwMask (dInt32(-1), dInt32(-1), dInt32(-1), dInt32(-1));

dSpatialVector dSpatialVector::m_zero (dFloat32 (0.0f));

dMatrix dMatrix::m_zeroMatrix(
	dVector(dFloat32(0.0f)),
	dVector(dFloat32(0.0f)),
	dVector(dFloat32(0.0f)),
	dVector(dFloat32(0.0f)));

dMatrix dMatrix::m_identityMatrix(
	dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)),
	dVector(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f)),
	dVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f)),
	dVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f)));
