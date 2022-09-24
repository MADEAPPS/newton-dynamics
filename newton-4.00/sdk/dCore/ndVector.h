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

#ifndef __ND_VECTOR_H__
#define __ND_VECTOR_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndClassAlloc.h"

#define ndCheckVector(x) (ndCheckFloat(x[0]) && ndCheckFloat(x[1]) && ndCheckFloat(x[2]) && ndCheckFloat(x[3]))

#ifdef D_SCALAR_VECTOR_CLASS
	#include "ndVectorScalar.h"
#elif (defined (__x86_64) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64))
	#include "ndVectorSimd.h"
#elif (defined(__arm__) && defined(__aarch64__))
	// assume arm instruction set until otherwise
	#include "ndVectorArmNeon.h"
#else // _x86
	#include "ndVectorScalar.h"
#endif

#endif