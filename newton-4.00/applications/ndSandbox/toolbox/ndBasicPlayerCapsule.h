/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _D_BASIC_PLAYER_CAPSULE_H_
#define _D_BASIC_PLAYER_CAPSULE_H_

#include "ndSandboxStdafx.h"
#include "ndAnimationPose.h"

class ndDemoMesh;
class ndDemoEntity;
class ndPhysicsWorld;
class ndDemoEntityManager;
class ndAnimationBlendTreeNode;

class ndBasicPlayerCapsule: public ndBodyPlayerCapsule
{
	public:
	D_CLASS_REFLECTION(ndBasicPlayerCapsule);
	class PlayerInputs
	{
		public:
		PlayerInputs()
		{
			m_heading = 0.0f;
			m_forwardSpeed = 0.0f;
			m_strafeSpeed = 0.0f;
			m_jump = false;
		}
		ndFloat32 m_heading;
		ndFloat32 m_forwardSpeed;
		ndFloat32 m_strafeSpeed;
		bool m_jump;
	};

	ndBasicPlayerCapsule(
		ndDemoEntityManager* const scene, const ndDemoEntity* const modelEntity,
		const ndMatrix& localAxis, const ndMatrix& location, 
		ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight, bool isPlayer = false);

	~ndBasicPlayerCapsule();

	ndBasicPlayerCapsule(const ndLoadSaveBase::dLoadDescriptor& desc);
	virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	void ApplyInputs(ndFloat32 timestep);
	ndFloat32 ContactFrictionCallback(const ndVector& position, const ndVector& normal, ndInt32 contactId, const ndBodyKinematic* const otherbody) const;

	void SetCamera();
	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, ndFloat32 timestep);

	ndDemoEntityManager* m_scene;
	PlayerInputs m_playerInput;
	bool m_isPlayer;

	ndAnimationPose m_output;
	ndAnimationBlendTreeNode* m_animBlendTree;
};

#endif