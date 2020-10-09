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

#ifndef __D_DYNAMIC_BODY_C_H__ 
#define __D_DYNAMIC_BODY_C_H__ 

#include "ndTypes.h"
#include "ndShapeInstanceC.h"

#ifdef __cplusplus 
extern "C" 
{
#endif

	typedef void* ndBodyDynamicC;

	typedef void(*ndForceAndTorque) (ndBodyDynamicC body, dFloat32 timestep);
	typedef void(*ndSetTransform) (ndBodyDynamicC body, const dFloat32* const matrix);

	NEWTON_API ndBodyDynamicC ndCreateBodyDynamic();
	NEWTON_API void ndBodyDynamicDestroy(ndBodyDynamicC bodyc);

	NEWTON_API void* ndBodyDynamicGetUserData(ndBodyDynamicC bodyc);
	NEWTON_API void ndBodyDynamicSetMatrix(ndBodyDynamicC bodyc, dFloat32* const matrix);
	NEWTON_API void ndBodyDynamicSetCollisionShape(ndBodyDynamicC bodyc, ndShapeInstanceC shapeInstancec);
	NEWTON_API void ndBodyDynamicSetMassMatrix(ndBodyDynamicC bodyc, dFloat32 mass, ndShapeInstanceC shapeInstancec);
	NEWTON_API void ndBodyDynamicSetCallbacks(ndBodyDynamicC bodyc, void* const usedData, ndForceAndTorque forceAndTorque, ndSetTransform transform);

#ifdef __cplusplus 
}
#endif

#endif 

