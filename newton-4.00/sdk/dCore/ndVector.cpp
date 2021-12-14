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

	ndVector ndVector::m_xMask(ndInt32(-1), ndInt32(0), ndInt32(0), ndInt32(0));
	ndVector ndVector::m_yMask(ndInt32(0), ndInt32(-1), ndInt32(0), ndInt32(0));
	ndVector ndVector::m_zMask(ndInt32(0), ndInt32(0), ndInt32(-1), ndInt32(0));
	ndVector ndVector::m_wMask(ndInt32(0), ndInt32(0), ndInt32(0), ndInt32(-1));
	ndVector ndVector::m_xyzwMask(ndInt32(-1), ndInt32(-1), ndInt32(-1), ndInt32(-1));
	ndVector ndVector::m_triplexMask(ndInt32(-1), ndInt32(-1), ndInt32(-1), ndInt32(0));
	ndVector ndVector::m_signMask(ndVector(ndInt32(-1), ndInt32(-1), ndInt32(-1), ndInt32(-1)).ShiftRightLogical(1));

	ndVector ndVector::m_zero(ndFloat32(0.0f));
	ndVector ndVector::m_one(ndFloat32(1.0f));
	ndVector ndVector::m_two(ndFloat32(2.0f));
	ndVector ndVector::m_half(ndFloat32(0.5f));
	ndVector ndVector::m_three(ndFloat32(3.0f));
	ndVector ndVector::m_negOne(ndFloat32(-1.0f));
	ndVector ndVector::m_epsilon(ndFloat32(1.0e-20f));
	ndVector ndVector::m_wOne(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f));
#endif

ndBigVector ndBigVector::m_zero (ndFloat64 (0.0f));
ndBigVector ndBigVector::m_one (ndFloat64 (1.0f));
ndBigVector ndBigVector::m_two (ndFloat64 (2.0f));
ndBigVector ndBigVector::m_half (ndFloat32 (0.5f));
ndBigVector ndBigVector::m_three (ndFloat32 (3.0f));
ndBigVector ndBigVector::m_negOne (ndFloat32 (-1.0f));
ndBigVector ndBigVector::m_epsilon(ndFloat32(1.0e-20f));
ndBigVector ndBigVector::m_wOne (ndFloat32 (0.0f), ndFloat32 (0.0f), ndFloat32 (0.0f), ndFloat32 (1.0f));
ndBigVector ndBigVector::m_triplexMask (ndInt32 (-1), ndInt32 (-1),	ndInt32 (-1), ndInt32 (0));
ndBigVector ndBigVector::m_signMask (ndBigVector(ndInt32 (-1), ndInt32 (-1), ndInt32 (-1), ndInt32 (-1)).ShiftRightLogical(1));

ndBigVector ndBigVector::m_xMask (ndInt32 (-1), ndInt32 ( 0),	ndInt32 ( 0), ndInt32 ( 0));
ndBigVector ndBigVector::m_yMask (ndInt32 ( 0), ndInt32 (-1),	ndInt32 ( 0), ndInt32 ( 0));
ndBigVector ndBigVector::m_zMask (ndInt32 ( 0), ndInt32 ( 0),	ndInt32 (-1), ndInt32 ( 0));
ndBigVector ndBigVector::m_wMask (ndInt32 ( 0), ndInt32 ( 0),	ndInt32 ( 0), ndInt32 (-1));
ndBigVector ndBigVector::m_xyzwMask (ndInt32(-1), ndInt32(-1), ndInt32(-1), ndInt32(-1));

ndSpatialVector ndSpatialVector::m_zero (ndFloat32 (0.0f));

//dMatrix dMatrix::m_zeroMatrix(
//	ndVector(ndFloat32(0.0f)),
//	ndVector(ndFloat32(0.0f)),
//	ndVector(ndFloat32(0.0f)),
//	ndVector(ndFloat32(0.0f)));
//
//dMatrix dMatrix::m_identityMatrix(
//	ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
//	ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
//	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f)),
//	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f)));
