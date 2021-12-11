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
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndSpatialVector.h"

#ifndef D_NEWTON_USE_DOUBLE

	ndVector ndVector::m_xMask(dInt32(-1), dInt32(0), dInt32(0), dInt32(0));
	ndVector ndVector::m_yMask(dInt32(0), dInt32(-1), dInt32(0), dInt32(0));
	ndVector ndVector::m_zMask(dInt32(0), dInt32(0), dInt32(-1), dInt32(0));
	ndVector ndVector::m_wMask(dInt32(0), dInt32(0), dInt32(0), dInt32(-1));
	ndVector ndVector::m_xyzwMask(dInt32(-1), dInt32(-1), dInt32(-1), dInt32(-1));
	ndVector ndVector::m_triplexMask(dInt32(-1), dInt32(-1), dInt32(-1), dInt32(0));
	ndVector ndVector::m_signMask(ndVector(dInt32(-1), dInt32(-1), dInt32(-1), dInt32(-1)).ShiftRightLogical(1));

	ndVector ndVector::m_zero(dFloat32(0.0f));
	ndVector ndVector::m_one(dFloat32(1.0f));
	ndVector ndVector::m_two(dFloat32(2.0f));
	ndVector ndVector::m_half(dFloat32(0.5f));
	ndVector ndVector::m_three(dFloat32(3.0f));
	ndVector ndVector::m_negOne(dFloat32(-1.0f));
	ndVector ndVector::m_epsilon(dFloat32(1.0e-20f));
	ndVector ndVector::m_wOne(dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f));
#endif

ndBigVector ndBigVector::m_zero (dFloat64 (0.0f));
ndBigVector ndBigVector::m_one (dFloat64 (1.0f));
ndBigVector ndBigVector::m_two (dFloat64 (2.0f));
ndBigVector ndBigVector::m_half (dFloat32 (0.5f));
ndBigVector ndBigVector::m_three (dFloat32 (3.0f));
ndBigVector ndBigVector::m_negOne (dFloat32 (-1.0f));
ndBigVector ndBigVector::m_epsilon(dFloat32(1.0e-20f));
ndBigVector ndBigVector::m_wOne (dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (1.0f));
ndBigVector ndBigVector::m_triplexMask (dInt32 (-1), dInt32 (-1),	dInt32 (-1), dInt32 (0));
ndBigVector ndBigVector::m_signMask (ndBigVector(dInt32 (-1), dInt32 (-1), dInt32 (-1), dInt32 (-1)).ShiftRightLogical(1));

ndBigVector ndBigVector::m_xMask (dInt32 (-1), dInt32 ( 0),	dInt32 ( 0), dInt32 ( 0));
ndBigVector ndBigVector::m_yMask (dInt32 ( 0), dInt32 (-1),	dInt32 ( 0), dInt32 ( 0));
ndBigVector ndBigVector::m_zMask (dInt32 ( 0), dInt32 ( 0),	dInt32 (-1), dInt32 ( 0));
ndBigVector ndBigVector::m_wMask (dInt32 ( 0), dInt32 ( 0),	dInt32 ( 0), dInt32 (-1));
ndBigVector ndBigVector::m_xyzwMask (dInt32(-1), dInt32(-1), dInt32(-1), dInt32(-1));

ndSpatialVector ndSpatialVector::m_zero (dFloat32 (0.0f));

//dMatrix dMatrix::m_zeroMatrix(
//	ndVector(dFloat32(0.0f)),
//	ndVector(dFloat32(0.0f)),
//	ndVector(dFloat32(0.0f)),
//	ndVector(dFloat32(0.0f)));
//
//dMatrix dMatrix::m_identityMatrix(
//	ndVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)),
//	ndVector(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f)),
//	ndVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f)),
//	ndVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f)));
