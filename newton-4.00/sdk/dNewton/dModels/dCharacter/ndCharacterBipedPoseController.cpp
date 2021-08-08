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
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndCharacter.h"
#include "ndCharacterEffectorNode.h"
#include "ndCharacterBipedPoseController.h"

ndCharacterBipedPoseController::ndCharacterBipedPoseController()
	:ndCharacterPoseController(nullptr)
	,m_leftFootEffector(nullptr)
	,m_rightFootEffector(nullptr)
	,m_walkCycle(this)
{
}

ndCharacterBipedPoseController::~ndCharacterBipedPoseController()
{
}

void ndCharacterBipedPoseController::Init(ndCharacter* const owner, const ndBipedControllerConfig& config)
{
	m_owner = owner;
	m_leftFootEffector = config.m_leftFootEffector;
	m_rightFootEffector = config.m_rightFootEffector;

	m_walkCycle.m_angle = dFloat32(0.0f);
	m_walkCycle.m_stride = dFloat32(0.25f);
	m_walkCycle.m_high = dFloat32(1.0f);
}

//bool ndCharacterBipedPoseController::Evaluate(ndWorld* const world, dFloat32 timestep)
bool ndCharacterBipedPoseController::Evaluate(ndWorld* const , dFloat32 timestep)
{
	//ndCharacter::ndCentreOfMassState comState(m_owner->CalculateCentreOfMassState());
	//m_owner->UpdateGlobalPose(world, timestep);
	//m_owner->CalculateLocalPose(world, timestep);
	m_walkCycle.Update(timestep);
	return true;
}


