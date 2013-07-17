/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dStdAfxNewton.h"
#include "dNewton.h"
#include "dNewtonBody.h"
#include "dNewtonCollision.h"
#include "dNewtonPlayerManager.h"


dNewtonPlayerManager::dNewtonPlayerManager (dNewton* const world)
	:CustomPlayerControllerManager (world->GetNewton())
{
}

dNewtonPlayerManager::~dNewtonPlayerManager ()
{
}


void dNewtonPlayerManager::ApplyPlayerMove (CustomPlayerController* const controller, dFloat timestep)
{
	dNewtonPlayer* const body = (dNewtonPlayer*) NewtonBodyGetUserData(controller->GetBody());
	body->OnPlayerMove (timestep);
}



/*
void dNewtonPlayerManager::DestroyPlayer (dPlayerController* const player)
{
}


dPlayerController::dPlayerController (CustomPlayerControllerManager::CustomController* const controller)
	:dNewtonAlloc()
{
}

dPlayerController::~dPlayerController()
{

}
*/

/*
dNewtonPlayerManager::dNewtonPlayer::dNewtonPlayer (dNewtonPlayerManager* const myManager, dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep)
	:dNewtonKinematicBody()
{
	dAssert (0);
}

dNewtonPlayerManager::dNewtonPlayer::~dNewtonPlayer ()
{
	dAssert (0);
}
*/



dNewtonPlayerManager::dNewtonPlayer::dNewtonPlayer (dNewtonPlayerManager* const manager, void* const userData, dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dFloat* const upDir, const dFloat* const frontDir)
	:dNewtonKinematicBody()
{
	dMatrix playerAxis; 
	playerAxis[0] = dVector (upDir); // the y axis is the character up vector
	playerAxis[1] = dVector (frontDir); // the x axis is the character front direction
	playerAxis[2] = playerAxis[0] * playerAxis[1];
	playerAxis[3] = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	m_controller = manager->CreatePlayer(mass, outerRadius, innerRadius, height, stairStep, playerAxis);

	SetUserData (userData);

	NewtonBody* const body = m_controller->GetBody();
	SetBody (body);
	NewtonCollision* const collision = NewtonBodyGetCollision(body);
	new dNewtonCollisionCompound (collision);

	for (void* node = NewtonCompoundCollisionGetFirstNode (collision); node; node = NewtonCompoundCollisionGetNextNode (collision, node)) {
		NewtonCollision* const childShape = NewtonCompoundCollisionGetCollisionFromNode (collision, node);

		switch (NewtonCollisionGetType (childShape)) 
		{
			case SERIALIZE_ID_CAPSULE:
			{
				new dNewtonCollisionCapsule (collision);
				break;
			}

			case SERIALIZE_ID_CYLINDER:
			{
				new dNewtonCollisionCylinder (collision);
				break;
			}
			default: 
				dAssert (0);
		}
	}


}

dNewtonPlayerManager::dNewtonPlayer::~dNewtonPlayer ()
{
	dAssert (0);
}
