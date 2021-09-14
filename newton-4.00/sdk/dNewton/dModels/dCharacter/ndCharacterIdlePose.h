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
#include "ndCharacterPose.h"
#include "ndCharacterPoseGenerator.h"

class ndCharacterEffectorNode;
class ndCharacterCentreOfMassState;
class ndCharacterBipedPoseController;

class ndCharacterIdlePose : public ndCharacterPoseGenerator
{
	public:
	enum ndIdleState
	{
		m_airborne,
		m_oneFeet,
		m_twoFeet,
	};

	ndCharacterIdlePose(ndCharacterBipedPoseController* const owner);

	protected:
	virtual void Init();

	void Update(dFloat32 timestep);
	void SetEffectorMatrix(const dVector& localCom, const ndCharaterKeyFramePose& pose);

	void TwoFeetState(dFloat32 timestep);
	void AirBorneState(dFloat32 timestep);

	bool IsComSupported(const dVector& com) const;
	void GetHeelPoints(dFixSizeArray<dVector, 32>& points) const;
	

	dVector m_zeroMomentPoint;
	//dFixSizeArray<ndCharaterKeyFramePose, 2> m_referencePose;
	ndCharacterBipedPoseController* m_owner;
	dFloat32 m_invertedPendulumRadius;

	ndIdleState m_state;
	friend class ndCharacterBipedPoseController;
};


#endif