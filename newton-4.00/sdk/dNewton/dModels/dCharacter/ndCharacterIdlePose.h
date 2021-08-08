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

#ifndef __D_CHARACTER_IDLE_POSE_H__
#define __D_CHARACTER_IDLE_POSE_H__

#include "ndNewtonStdafx.h"
#include "ndCharacterPoseGenerator.h"

class ndCharacterEffectorNode;
class ndCharacterBipedPoseController;

class ndCharacterIdlePose : public ndCharacterPoseGenerator
{
	public:
	ndCharacterIdlePose(ndCharacterBipedPoseController* const owner);

	protected:
	void Update(dFloat32 timestep);

	void MoveFoot(ndCharacterEffectorNode* const footEffector, dFloat32 angle);

	dFloat32 m_high;
	dFloat32 m_angle;
	dFloat32 m_stride;
	ndCharacterBipedPoseController* m_owner;

	friend class ndCharacterBipedPoseController;
};


#endif