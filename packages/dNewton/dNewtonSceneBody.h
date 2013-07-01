/* Copyright (c) <2003-2013> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_NEWTON_SCENE_BODY_H_
#define _D_NEWTON_SCENE_BODY_H_


#include "dStdAfxNewton.h"
#include "dNewtonBody.h"

class dNewtonSceneBody: public dNewtonBody  
{
	public:
	CNEWTON_API dNewtonSceneBody (dNewton* const world);
	CNEWTON_API virtual ~dNewtonSceneBody();

	CNEWTON_API virtual void OnForceAndTorque (dFloat timestep, int threadIndex) {};

	CNEWTON_API virtual void BeginAddRemoveCollision();
	CNEWTON_API virtual void* AddCollision(const dNewtonCollision* const collision);
	CNEWTON_API virtual void RemoveCollision (void* const handle);
	CNEWTON_API virtual void EndAddRemoveCollision();
};

#endif
