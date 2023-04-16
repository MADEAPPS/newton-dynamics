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
#ifndef __PHYSICS_WORLD_H__
#define __PHYSICS_WORLD_H__

#include "ndSandboxStdafx.h"

#define MAX_PHYSICS_FPS			60.0f

class ndSoundManager;
class ndDemoEntityManager;

class ndPhysicsWorld: public ndWorld
{
	public:
	D_CLASS_REFLECTION(ndPhysicsWorld, ndWorld)

	class ndPhysicsWorldFileLoadSave : public ndFileFormatWorld
	{
		public:
		ndPhysicsWorldFileLoadSave()
			:ndFileFormatWorld(ndPhysicsWorld::StaticClassName())
		{
		}

		void SaveWorld(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndWorld* const world)
		{
			nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndPhysicsWorld", ndPhysicsWorld::StaticClassName());
			ndFileFormatWorld::SaveWorld(scene, classNode, world);
		}
	};

	class ndDefferentDeleteEntities : public ndArray<ndDemoEntity*>
	{
		public:
		ndDefferentDeleteEntities(ndDemoEntityManager* const manager);

		void Update();
		void RemoveEntity(ndDemoEntity* const entity);

		ndDemoEntityManager* m_manager;
		std::thread::id m_renderThreadId;
	};

	ndPhysicsWorld(ndDemoEntityManager* const manager);
	virtual ~ndPhysicsWorld();
	virtual void CleanUp();

	void AdvanceTime(ndFloat32 timestep);
	ndDemoEntityManager* GetManager() const;
	ndSoundManager* GetSoundManager() const;
	void RemoveEntity(ndDemoEntity* const entity);

	private:
	void OnPostUpdate(ndFloat32 timestep);

	ndDemoEntityManager* m_manager;
	ndSoundManager* m_soundManager;
	ndFloat32 m_timeAccumulator;
	ndDefferentDeleteEntities m_deadEntities;
};

#endif