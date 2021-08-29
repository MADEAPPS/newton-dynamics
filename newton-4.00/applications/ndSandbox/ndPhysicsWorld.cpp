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

#include "ndSandboxStdafx.h"
#include "ndPhysicsWorld.h"
#include "ndDemoCamera.h"
#include "ndSoundManager.h"
#include "ndContactCallback.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraManager.h"
#include "ndDemoMeshInterface.h"
#include "ndBasicPlayerCapsule.h"
#include "ndArchimedesBuoyancyVolume.h"

#define MAX_PHYSICS_STEPS			1
#define MAX_PHYSICS_FPS				60.0f
//#define MAX_PHYSICS_RECOVER_STEPS	2

class ndPhysicsWorldSettings : public ndWordSettings
{
	public:
	D_CLASS_REFLECTION(ndPhysicsWorldSettings);

	ndPhysicsWorldSettings(ndPhysicsWorld* const world)
		:ndWordSettings()
		,m_cameraMatrix(dGetIdentityMatrix())
		,m_world(world)
	{
	}

	ndPhysicsWorldSettings(const dLoadSaveBase::dLoadDescriptor& desc)
		:ndWordSettings(dLoadSaveBase::dLoadDescriptor(desc))
		,m_world(nullptr)
	{
	}

	virtual void Load(const dLoadSaveBase::dLoadDescriptor& desc)
	{
		dLoadSaveBase::dLoadDescriptor childDesc(desc);
		ndWordSettings::Load(childDesc);
		
		// load application specific settings here
		m_cameraMatrix = xmlGetMatrix(desc.m_rootNode, "cameraMatrix");
	}

	virtual void Save(const dLoadSaveBase::dSaveDescriptor& desc) const
	{
		nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
		desc.m_rootNode->LinkEndChild(childNode);
		ndWordSettings::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

		ndDemoEntityManager* const manager = m_world->GetManager();
		ndDemoCamera* const camera = manager->GetCamera();

		dMatrix cameraMatrix (camera->GetCurrentMatrix());
		xmlSaveParam(childNode, "description", "string", "this scene was saved from Newton 4.0 sandbox demos");
		xmlSaveParam(childNode, "cameraMatrix", cameraMatrix);
	}
	
	dMatrix m_cameraMatrix;
	ndPhysicsWorld* m_world;
};

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndPhysicsWorldSettings);

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
	,m_soundManager(new ndSoundManager(manager))
	,m_timeAccumulator(0.0f)
	,m_deletedBodies()
	,m_hasPendingObjectToDelete(false)
	,m_deletedLock()
{
	ClearCache();
	SetContactNotify(new ndContactCallback);
}

ndPhysicsWorld::~ndPhysicsWorld()
{
	if (m_soundManager)
	{
		delete m_soundManager;
	}
}

ndDemoEntityManager* ndPhysicsWorld::GetManager() const
{
	return m_manager;
}

void ndPhysicsWorld::QueueBodyForDelete(ndBody* const body)
{
	dScopeSpinLock lock(m_deletedLock);
	m_hasPendingObjectToDelete.store(true);
	m_deletedBodies.PushBack(body);
}

void ndPhysicsWorld::DeletePendingObjects()
{
	if (m_hasPendingObjectToDelete.load())
	{
		Sync();
		m_hasPendingObjectToDelete.store(false);
		for (dInt32 i = 0; i < m_deletedBodies.GetCount(); i++)
		{
			DeleteBody(m_deletedBodies[i]);
		}
		m_deletedBodies.SetCount(0);
	}
}

void ndPhysicsWorld::AdvanceTime(dFloat32 timestep)
{
	const dFloat32 descreteStep = (1.0f / MAX_PHYSICS_FPS);

	dInt32 maxSteps = MAX_PHYSICS_STEPS;
	m_timeAccumulator += timestep;

	// if the time step is more than max timestep par frame, throw away the extra steps.
	if (m_timeAccumulator > descreteStep * maxSteps)
	{
		dFloat32 steps = dFloor(m_timeAccumulator / descreteStep) - maxSteps;
		dAssert(steps >= 0.0f);
		m_timeAccumulator -= descreteStep * steps;
	}
	
	while (m_timeAccumulator > descreteStep)
	{
		Update(descreteStep);
		m_timeAccumulator -= descreteStep;

		DeletePendingObjects();
	}
	if (m_manager->m_synchronousPhysicsUpdate)
	{
		Sync();
	}
}

ndSoundManager* ndPhysicsWorld::GetSoundManager() const
{
	return m_soundManager;
}

void ndPhysicsWorld::OnPostUpdate(dFloat32 timestep)
{
	m_manager->m_cameraManager->FixUpdate(m_manager, timestep);
	if (m_manager->m_updateCamera)
	{
		m_manager->m_updateCamera(m_manager, m_manager->m_updateCameraContext, timestep);
	}

	if (m_soundManager)
	{
		m_soundManager->Update(this, timestep);
	}
}

bool ndPhysicsWorld::LoadScene(const char* const path)
{
	ndLoadSave loadScene;
	loadScene.LoadScene(path);

	// iterate over the loaded scene and add all objects to the world.
	if (loadScene.m_setting && (strcmp("ndPhysicsWorldSettings", loadScene.m_setting->SubClassName()) == 0))
	{
		ndPhysicsWorldSettings* const settings = (ndPhysicsWorldSettings*)loadScene.m_setting;
		ndDemoEntityManager* const manager = GetManager();
		manager->SetCameraMatrix(settings->m_cameraMatrix, settings->m_cameraMatrix.m_posit);
	}

	ndBodyLoaderCache::Iterator bodyIter(loadScene.m_bodyMap);
	for (bodyIter.Begin(); bodyIter; bodyIter++)
	{
		const ndBody* const body = (ndBody*)bodyIter.GetNode()->GetInfo();
		AddBody((ndBody*)body);
	}
	
	ndJointLoaderCache::Iterator jointIter(loadScene.m_jointMap);
	for (jointIter.Begin(); jointIter; jointIter++)
	{
		const ndJointBilateralConstraint* const joint = (ndJointBilateralConstraint*)jointIter.GetNode()->GetInfo();
		AddJoint((ndJointBilateralConstraint*)joint);
	}
	
	ndModelLoaderCache::Iterator modelIter(loadScene.m_modelMap);
	for (modelIter.Begin(); modelIter; modelIter++)
	{
		const ndModel* const model = modelIter.GetNode()->GetInfo();
		AddModel((ndModel*)model);
	}

	return true;
}

void ndPhysicsWorld::SaveScene(const char* const path)
{
	ndLoadSave loadScene;
	ndPhysicsWorldSettings setting(this);
	
	loadScene.SaveScene(path, this, &setting);
}

void ndPhysicsWorld::SaveSceneModel(const char* const path)
{
	ndLoadSave loadScene;
	loadScene.SaveModel(path, m_manager->m_selectedModel);
}