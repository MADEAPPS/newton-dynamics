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
#ifndef __PHYSICS_WORLD_H__
#define __PHYSICS_WORLD_H__

#include "ndSandboxStdafx.h"

#define MAX_PHYSICS_FPS			60.0f
class ndDemoEntityManager;

class ndPhysicsWorld: public ndWorld
{
	public:
	ndPhysicsWorld(ndDemoEntityManager* const manager);
	virtual ~ndPhysicsWorld();

	void AdvanceTime(dFloat32 timestep);
	ndDemoEntityManager* GetManager() const;

	void QueueBodyForDelete(ndBody* const body);

	private:
	void OnPostUpdate(dFloat32 timestep);
	ndBody* LoadUserDefinedBody(const nd::TiXmlNode* const parentNode, const char* const bodyClassName, dTree<const ndShape*, dUnsigned32>& shapesCache, const char* const assetPath) const;

	void DeletePendingObjects();

	ndDemoEntityManager* m_manager;
	dFloat32 m_timeAccumulator;
	dArray<ndBody*> m_deletedBodies;
	dAtomic<bool> m_hasPendingObjectToDelete;
	dSpinLock m_deletedLock;
};

#endif