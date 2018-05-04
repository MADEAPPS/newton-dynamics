/* 
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


#include "stdafx.h"
#include "dNewtonWorld.h"
#include "dNewtonVehicle.h"
#include "dNewtonVehicleManager.h"


dNewtonVehicle::dNewtonVehicle(dNewtonWorld* const world, dNewtonCollision* const collision, dMatrix matrix, dFloat mass)
	:dNewtonDynamicBody(world, collision, matrix, mass)
{
	dNewtonVehicleManager* const vehicleManager = world->GetVehicleManager();

	dMatrix vehicleFrame(dGetIdentityMatrix());
	NewtonApplyForceAndTorque forceCallback = NewtonBodyGetForceAndTorqueCallback(m_body);

	dFloat gravidyMag = dSqrt(world->GetGravity().DotProduct3(world->GetGravity()));
//	m_controller = vehicleManager->CreateVehicle(m_body, vehicleFrame, forceCallback, this, gravidyMag);
	dAssert(0);
	m_controller = NULL;
}

dNewtonVehicle::~dNewtonVehicle()
{
	dNewtonVehicleManager* const vehicleManager = (dNewtonVehicleManager*) m_controller->GetManager();
	vehicleManager->DestroyController(m_controller);
}



dNewtonWheel::dNewtonWheel(dNewtonVehicle* const owner, dTireData tireData)
	:dAlloc()
	,m_owner(tireData.m_owner)
{

}

dNewtonWheel::~dNewtonWheel()
{
}

void* dNewtonWheel::GetUserData()
{
	return m_owner;
}
