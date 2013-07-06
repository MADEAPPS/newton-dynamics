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
#include "DemoSoundListener.h"
#include "DemoEntityListener.h"
#include "DemoListenerBase.h"
#include "dHighResolutionTimer.h"
#include "DemoVisualDebugerListener.h"

class DemoMesh;
class DemoEntity;
class DemoCamera;
class NewtonDemos;
class DemoEntityManager;
class DemoCameraListener;


typedef void (*RenderHoodCallback) (DemoEntityManager* const manager, void* const context);

class DemoEntityManager: public wxGLCanvas, public dList <DemoEntity*>
{
	public:

	class ButtonKey
	{
		public:
		ButtonKey (bool initialState);

		bool UpdateTriggerButton (const NewtonDemos* const mainWin, int keyCode);
		void UpdatePushButton (const NewtonDemos* const mainWin, int keyCode);
		bool GetPushButtonState() const { return m_state;}


		bool UpdateTriggerJoystick (const NewtonDemos* const mainWin, int buttonMask);
//		bool IsMouseKeyDown (const NewtonDemos* const mainWin, int key);
//		bool IsKeyDown (const NewtonDemos* const mainWin, int key);

		private:
		bool m_state;
		bool m_memory0;
		bool m_memory1;
	};

	class EntityDictionary: public dTree<DemoEntity*, dScene::dTreeNode*>
	{
	};

	DemoEntityManager(NewtonDemos* const mainFrame);
	~DemoEntityManager(void);

	void InitGraphicsSystem();

	void ResetTimer();
	void RenderFrame ();
	void UpdatePhysics(float timestep);

	float GetPhysicsTime();

	int GetWidth() const;
	int GetHeight() const;

//	dAI* GetAI() const;
	NewtonWorld* GetNewton() const;
	NewtonDemos* GetRootWindow() const;

	DemoCamera* GetCamera() const;
	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);
//	void SetNewCamera(DemoCamera* const camera);

	void Set2DDisplayRenderFunction (RenderHoodCallback callback, void* const context);

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	void LoadScene (const char* const name);
	void SerializedPhysicScene (const char* const name);
	void DeserializedPhysicScene (const char* const name);

	

	void Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... ) const;


	void Cleanup ();
	void RemoveEntity (DemoEntity* const ent);
	void RemoveEntity (dList<DemoEntity*>::dListNode* const entNode);

	void CreateSkyBox();
	dSoundManager* GetSoundManager() const; 


	static void SerializeFile (void* const serializeHandle, const void* const buffer, int size);
	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, NewtonSerializeCallback serializecallback, void* const serializeHandle);
	static void BodyDeserialization (NewtonBody* const body, NewtonDeserializeCallback serializecallback, void* const serializeHandle);

	private:
	void OnSize(wxSizeEvent &event);
	void OnIdle(wxIdleEvent &event);
	void OnPaint(wxPaintEvent& event);
	void OnEraseBackground(wxEraseEvent& event);

	void OnKeyUp(wxKeyEvent &event);
	void OnKeyDown(wxKeyEvent &event);
	void OnMouse(wxMouseEvent &event);

	dFloat CalculateInteplationParam () const;
	void LoadVisualScene(dScene* const scene, EntityDictionary& dictionary);

	void CreateOpenGlFont();


	NewtonDemos* m_mainWindow;
	NewtonWorld* m_world;
	
//	dAI* m_aiWorld;
	DemoEntity* m_sky;
	unsigned64 m_microsecunds;
	dFloat m_physicsTime;
	dFloat m_currentListenerTimestep;
	bool m_physicsUpdate;
	bool m_reEntrantUpdate;
	void* m_renderHoodContext;
	RenderHoodCallback m_renderHood;
	GLuint m_font;
	GLuint m_fontImage;
	DemoSoundListener* m_soundManager;
	DemoCameraListener* m_cameraManager;
	DemoVisualDebugerListener* m_visualDebugger;

	int m_showProfiler[10]; 
	dRuntimeProfiler m_profiler;

	dFloat m_mainThreadGraphicsTime;
	dFloat m_mainThreadPhysicsTime;
	dFloat m_physThreadTime;



	static int m_attributes[];

	DECLARE_EVENT_TABLE()

	friend class NewtonDemos;
	friend class dRuntimeProfiler;
	friend class DemoEntityListener;
	friend class DemoListenerManager;

};

// for simplicity we are not going to run the demo in a separate thread at this time
// this confuses many user int thinking it is more complex than it really is  
inline void DemoEntityManager::Lock(unsigned& atomicLock)
{
	while (NewtonAtomicSwap((int*)&atomicLock, 1)) {
		NewtonYield();
	}
}

inline void DemoEntityManager::Unlock(unsigned& atomicLock)
{
	NewtonAtomicSwap((int*)&atomicLock, 0);
}


inline NewtonWorld* DemoEntityManager::GetNewton() const
{
	return m_world;
}

//inline dAI* DemoEntityManager::GetAI() const
//{
//	return m_aiWorld;
//}

inline NewtonDemos* DemoEntityManager::GetRootWindow () const
{
	return m_mainWindow;
};


inline dSoundManager* DemoEntityManager::GetSoundManager() const
{
	return m_soundManager;
}

inline int DemoEntityManager::GetWidth() const 
{ 
	int width;
	int height;
	GetSize (&width, &height);
	return width; 
}

inline int DemoEntityManager::GetHeight() const 
{ 
	int width;
	int height;
	GetSize (&width, &height);
	return height; 
}

#endif