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
		dFloat32 m_heading;
		dFloat32 m_forwardSpeed;
		dFloat32 m_strafeSpeed;
		bool m_jump;
	};

	ndBasicPlayerCapsule(
		ndDemoEntityManager* const scene, const ndDemoEntity* const modelEntity,
		const dMatrix& localAxis, const dMatrix& location, 
		dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight, bool isPlayer = false);

	~ndBasicPlayerCapsule();

	ndBasicPlayerCapsule(const dClassLoaderBase::dDesc& desc);
	virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeId) const;

	void ApplyInputs(dFloat32 timestep);
	dFloat32 ContactFrictionCallback(const dVector& position, const dVector& normal, dInt32 contactId, const ndBodyKinematic* const otherbody) const;

	void SetCamera();
	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, dFloat32 timestep);

	ndDemoEntityManager* m_scene;
	PlayerInputs m_playerInput;
	bool m_isPlayer;

	ndAnimationPose m_output;
	ndAnimationBlendTreeNode* m_animBlendTree;
};

#endif