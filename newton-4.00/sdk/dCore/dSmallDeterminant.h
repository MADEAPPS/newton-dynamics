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

#ifndef __D_SMALL_DETERMINANT_H__
#define __D_SMALL_DETERMINANT_H__

#include "dCoreStdafx.h"
#include "dTypes.h"

class dgGoogol;
dFloat64 Determinant2x2 (const dFloat64 matrix[2][2], dFloat64* const error);
dFloat64 Determinant3x3 (const dFloat64 matrix[3][3], dFloat64* const error);
dFloat64 Determinant4x4 (const dFloat64 matrix[4][4], dFloat64* const error);

dgGoogol Determinant2x2 (const dgGoogol matrix[2][2]);
dgGoogol Determinant3x3 (const dgGoogol matrix[3][3]);
dgGoogol Determinant4x4 (const dgGoogol matrix[4][4]);

#endif
