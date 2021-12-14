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

#ifndef __ND_SMALL_DETERMINANT_H__
#define __ND_SMALL_DETERMINANT_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"

class ndGoogol;
ndFloat64 Determinant2x2 (const ndFloat64 matrix[2][2], ndFloat64* const error);
ndFloat64 Determinant3x3 (const ndFloat64 matrix[3][3], ndFloat64* const error);
ndFloat64 Determinant4x4 (const ndFloat64 matrix[4][4], ndFloat64* const error);

ndGoogol Determinant2x2 (const ndGoogol matrix[2][2]);
ndGoogol Determinant3x3 (const ndGoogol matrix[3][3]);
ndGoogol Determinant4x4 (const ndGoogol matrix[4][4]);

#endif
