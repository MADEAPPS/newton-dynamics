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
#include "ndCharacterIdlePose.h"
#include "ndCharacterEffectorNode.h"
#include "ndCharacterPoseGenerator.h"
#include "ndCharacterBipedPoseController.h"

ndCharacterIdlePose::ndCharacterIdlePose(ndCharacterBipedPoseController* const owner)
	:m_owner(owner)
{
	m_angle = dFloat32(0.0f);
	m_high = dFloat32(1.0f);
	m_stride = dFloat32(0.25f);
}

void ndCharacterIdlePose::MoveFoot(ndCharacterEffectorNode* const footEffector, dFloat32 angle)
{
	const dFloat32 hipHigh = dFloat32(0.03f);

	dFloat32 y = hipHigh;
	angle = dMod(angle, dFloat32(2.0f) * dPi);
	dFloat32 x = m_stride * dSin(angle);

	dVector posit (x, y, dFloat32(0.0f), dFloat32(1.0f));
	footEffector->SetTargetMatrix(posit);
}

//void ndCharacterIdlePose::Update(dFloat32 timestep)
void ndCharacterIdlePose::Update(dFloat32)
{
	const ndBipedControllerConfig& config = m_owner->GetConfig();
	if (config.m_rightFootEffector)
	{
		MoveFoot(config.m_rightFootEffector, m_angle + dFloat32(0.0f) * dPi);
	}

	if (config.m_leftFootEffector)
	{
		MoveFoot(config.m_leftFootEffector, m_angle + dFloat32(1.0f) * dPi);
	}
	//m_angle = dMod(m_angle + timestep * 2.0f, dFloat32(2.0f) * dPi);
	m_angle = 20.0f * dDegreeToRad;
}

