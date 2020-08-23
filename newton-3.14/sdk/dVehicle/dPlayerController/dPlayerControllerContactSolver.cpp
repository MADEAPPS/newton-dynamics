/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////

#include "dStdafxVehicle.h"
#include "dVehicleManager.h"
#include "dPlayerController.h"
#include "dPlayerControllerContactSolver.h"

unsigned dPlayerControllerContactSolver::PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	dPlayerController* const controller = (dPlayerController*)userData;
	if (controller->GetBody() == body) {
		return false;
	}
	return 1;
}

void dPlayerControllerContactSolver::CalculateContacts()
{
	dMatrix matrix;
	NewtonBody* const body = m_controller->GetBody();
	NewtonWorld* const world = m_controller->GetManager()->GetWorld();
	NewtonCollision* const shape = NewtonBodyGetCollision(body);

	NewtonBodyGetMatrix(body, &matrix[0][0]);
	m_contactCount = NewtonWorldCollide(world, &matrix[0][0], shape, m_controller, PrefilterCallback, m_contactBuffer, D_PLAYER_MAX_CONTACTS, 0);
}
