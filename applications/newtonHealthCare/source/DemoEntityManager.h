/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __DEMO_ENTITY_MANAGER_H__
#define __DEMO_ENTITY_MANAGER_H__

#include "dRuntimeProfiler.h"
#include "DemoEntityListener.h"
#include "DemoListenerManager.h"
#include "dHighResolutionTimer.h"
#include "DemoVisualDebugerListener.h"

class DemoMesh;
class DemoEntity;
class DemoCamera;
class NewtonDemos;
class DemoEntityManager;


class PushButtonKey
{
	public:
	PushButtonKey ();
	bool IsMouseKeyDown (const NewtonDemos* const mainWin, int key);
	bool IsKeyDown (const NewtonDemos* const mainWin, int key);

	bool m_memory;
};



typedef void (*RenderHoodCallback) (DemoEntityManager* manager, void* const context);

class DemoEntityManager: public FXGLCanvas,  public dList <DemoEntity*>
{
	public:
	class EntityDictionary: public dTree<DemoEntity*, dScene::dTreeNode*>
	{
	};

	DemoEntityManager(FXComposite* const parent, NewtonDemos* const mainFrame);
	~DemoEntityManager(void);

	void create();

	void ResetTimer();
	void UpdateScene (float timestep);
	void UpdatePhysics(float timestep);

	float GetPhysicsTime();

	int GetWidth() const;
	int GetHeight() const;

	dAI* GetAI() const;
	NewtonWorld* GetNewton() const;
	NewtonDemos* GetRootWindow() const;

	DemoCamera* GetCamera() const;
	void SetNewCamera(DemoCamera* const camera);

	void Set2DDisplayRenderFunction (RenderHoodCallback callback, void* const context);

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	void LoadScene (const char* const name);
	void SerializedPhysicScene (const char* const name);
	void DeserializedPhysicScene (const char* const name);
	

	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);

	void Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... ) const;


	void Cleanup ();
	void RemoveEntity (DemoEntity* const ent);
	void RemoveEntity (dList<DemoEntity*>::dListNode* const entNode);

	void CreateSkyBox();
	

	static void SerializeFile (void* const serializeHandle, const void* const buffer, int size);
	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, NewtonSerializeCallback serializecallback, void* const serializeHandle);
	static void BodyDeserialization (NewtonBody* const body, NewtonDeserializeCallback serializecallback, void* const serializeHandle);
	


	private:
	void InterpolateMatrices ();
	void LoadVisualScene(dScene* const scene, EntityDictionary& dictionary);

//	private:
//	int m_cannonBallRate;
//	DemoMesh* m_meshBallMesh;

	NewtonDemos* m_mainWindow;
	NewtonWorld* m_world;
	
	dAI* m_aiWorld;
	DemoCamera* m_camera;
	DemoEntity* m_sky;
	unsigned64 m_microsecunds;
	dFloat m_physicsTime;
	bool m_physicsUpdate;
	bool m_reEntrantUpdate;
	void* m_renderHoodContext;
	RenderHoodCallback m_renderHood;
	GLuint m_font;
	
	DemoListenerManager m_listenerManager;
	DemoVisualDebugerListener* m_visualDebugger;

	int m_showProfiler[10]; 
	dRuntimeProfiler m_profiler;

	dFloat m_mainThreadGraphicsTime;
	dFloat m_mainThreadPhysicsTime;
	dFloat m_physThreadTime;



	friend class NewtonDemos;
	friend class dRuntimeProfiler;
	friend class DemoListenerManager;
};

// for simplicity we are not going to run the demo in a separate thread at this time
// this confuses many user int thinking it is more complex than it really is  
inline void DemoEntityManager::Lock(unsigned& atomicLock)
{
}

inline void DemoEntityManager::Unlock(unsigned& atomicLock)
{
}


inline NewtonWorld* DemoEntityManager::GetNewton() const
{
	return m_world;
}

inline dAI* DemoEntityManager::GetAI() const
{
	return m_aiWorld;
}

inline NewtonDemos* DemoEntityManager::GetRootWindow () const
{
	return m_mainWindow;
};

inline DemoCamera* DemoEntityManager::GetCamera() const
{
	return m_camera;
}


inline int DemoEntityManager::GetWidth() const 
{ 
	return getWidth(); 
}

inline int DemoEntityManager::GetHeight() const 
{ 
	return getHeight(); 
}

#endif