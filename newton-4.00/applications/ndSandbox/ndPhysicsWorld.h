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

class ndDemoEntity;
class ndSoundManager;
class ndDemoEntityManager;

class ndDemoContactCallback : public ndContactCallback
{
	public:
	enum ndMaterialUserIDs
	{
		m_default = 0,
		m_frictionTest = 1,
		m_aiCar = 2,
		m_aiTerrain = 3,
		m_modelPart = 4,
		m_vehicleTirePart = 5,
		m_dedris = 100,
	};

	enum ndMaterialFlags
	{
		//m_playSound = 1 << 0,
		//m_debrisBody = 1 << 1,
	};

	enum ndMaterialShapeUserIDs
	{
		m_density,
		m_friction,
		m_modelPointer,
		m_materialFlags,
		m_soundSpeedThreshold,
	};

	ndDemoContactCallback();
	~ndDemoContactCallback();
};

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

	void NormalUpdates();
	void AccelerateUpdates();

	private:
	void PreUpdate(ndFloat32 timestep);
	void PostUpdate(ndFloat32 timestep);

	ndDemoEntityManager* m_manager;
	ndSoundManager* m_soundManager;
	ndFloat32 m_timeAccumulator;
	ndDefferentDeleteEntities m_deadEntities;
	bool m_acceleratedUpdate;
};

#endif