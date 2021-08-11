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

#ifndef __D_CHARACTER_INVERTED_PENDULUM_POSE_CONTROLLER_H__
#define __D_CHARACTER_INVERTED_PENDULUM_POSE_CONTROLLER_H__

#include "ndNewtonStdafx.h"
#include "ndCharacterIdlePose.h"
#include "ndCharacterPoseController.h"
#include "ndCharacterWalkCycleGenerator.h"

class ndCharacterEffectorNode;

class ndBipedControllerConfig
{
	public:
	ndBipedControllerConfig()
		:m_leftFootEffector(nullptr)
		,m_rightFootEffector(nullptr)
	{
	}

	ndCharacterEffectorNode* m_leftFootEffector;
	ndCharacterEffectorNode* m_rightFootEffector;
};

class ndCharacterBipedPoseController: public ndCharacterPoseController
{
	public:
	D_CLASS_RELECTION(ndCharacterBipedPoseController);

	D_NEWTON_API ndCharacterBipedPoseController();
	D_NEWTON_API virtual ~ndCharacterBipedPoseController ();

	D_NEWTON_API void Init(ndCharacter* const owner, const ndBipedControllerConfig& config);

	const ndBipedControllerConfig& GetConfig() const;
	dRay CalculateSupportPoint(const dVector& comInGlobalSpace) const;

	protected:
	virtual void Debug(ndConstraintDebugCallback& context) const;
	virtual bool Evaluate(ndWorld* const world, dFloat32 timestep);

	

	ndBipedControllerConfig m_config;
	ndCharacterIdlePose m_idleCycle;
	ndCharacterWalkCycleGenerator m_walkCycle;
};

inline const ndBipedControllerConfig& ndCharacterBipedPoseController::GetConfig() const
{
	return m_config;
}

#endif