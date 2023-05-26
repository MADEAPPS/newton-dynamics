/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
class ndAnimationSequencePlayer;

class ndBasicPlayerCapsule: public ndBodyPlayerCapsule
{
	public:
	D_CLASS_REFLECTION(ndBasicPlayerCapsule, ndBodyPlayerCapsule)
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

	class ndFileBasicPlayerCapsule : public ndFileFormatBodyKinematicPlayerCapsule
	{
		public:
		ndFileBasicPlayerCapsule()
			:ndFileFormatBodyKinematicPlayerCapsule(ndBasicPlayerCapsule::StaticClassName())
		{
		}

		virtual ndBody* LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap);
		virtual void SaveBody(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body);
	};

	ndBasicPlayerCapsule();
	ndBasicPlayerCapsule(
		ndDemoEntityManager* const scene, const ndDemoEntity* const modelEntity,
		const ndMatrix& localAxis, const ndMatrix& location, 
		ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight, bool isPlayer = false);
	~ndBasicPlayerCapsule();

	void ApplyInputs(ndFloat32 timestep);
	ndFloat32 ContactFrictionCallback(const ndVector& position, const ndVector& normal, ndInt32 contactId, const ndBodyKinematic* const otherbody) const;

	void SetCamera(ndDemoEntityManager* const scene);
	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, ndFloat32 timestep);

	//ndDemoEntityManager* m_scene;
	PlayerInputs m_playerInput;
	bool m_isPlayer;

	ndAnimationPose m_output;
	ndAnimationSequencePlayer* m_walkCycle;
	ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
};

#endif