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

#include "DemoMesh.h"
#include "DemoEntityListener.h"
#include "DemoListenerBase.h"
#include "dHighResolutionTimer.h"
//#include "DemoVisualDebugerListener.h"

//class DemoMesh;
class DemoEntity;
class DemoCamera;
class NewtonDemos;
class DemoEntityManager;
class DemoCameraListener;


typedef void (*RenderHoodCallback) (DemoEntityManager* const manager, void* const context, int lineNumber);

class DemoEntityManager: public wxGLCanvas, public dList <DemoEntity*>
{
	public:
    class TransparentMesh
    {
        public: 
        TransparentMesh()
            :m_matrix(dGetIdentityMatrix())
            ,m_mesh(NULL)
        {
        }

        TransparentMesh(const dMatrix& matrix, const DemoMesh* const mesh)
            :m_matrix(matrix)
            ,m_mesh(mesh)
        {
        }

        dMatrix m_matrix;
        const DemoMesh* m_mesh;
    };

    class TransparentHeap: public dUpHeap <TransparentMesh, dFloat>
    {
        public:
        TransparentHeap()
            :dUpHeap <TransparentMesh, dFloat>(256)
        {
        }
    };

	class ButtonKey
	{
		public:
		ButtonKey (bool initialState);

		bool UpdateTriggerButton (const NewtonDemos* const mainWin, int keyCode);
		bool UpdatePushButton (const NewtonDemos* const mainWin, int keyCode);
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
	void UpdatePhysics(dFloat timestep);

	dFloat GetPhysicsTime();

	int GetWidth() const;
	int GetHeight() const;

	NewtonWorld* GetNewton() const;
	NewtonDemos* GetRootWindow() const;

	DemoCamera* GetCamera() const;
	void SetCameraMouseLock (bool state);
	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);

    void PushTransparentMesh (const DemoMeshInterface* const mesh); 

	void Set2DDisplayRenderFunction (RenderHoodCallback callback, void* const context);

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	void LoadScene (const char* const name);
	void SerializedPhysicScene (const char* const name);
	void DeserializedPhysicScene (const char* const name);
	

	int Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... ) const;


	void Cleanup ();
	void RemoveEntity (DemoEntity* const ent);
	void RemoveEntity (dList<DemoEntity*>::dListNode* const entNode);

	void CreateSkyBox();

	static void SerializeFile (void* const serializeHandle, const void* const buffer, int size);
	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, void* const userData, NewtonSerializeCallback serializecallback, void* const serializeHandle);
	static void BodyDeserialization (NewtonBody* const body, void* const userData, NewtonDeserializeCallback serializecallback, void* const serializeHandle);

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

	wxGLContext* m_context;	

	NewtonDemos* m_mainWindow;
	NewtonWorld* m_world;
	
	DemoEntity* m_sky;
	unsigned64 m_microsecunds;
	dFloat m_currentListenerTimestep;
	bool m_physicsUpdate;
	bool m_reEntrantUpdate;
	void* m_renderHoodContext;
	RenderHoodCallback m_renderHood;
	GLuint m_font;
	GLuint m_fontImage;
	DemoCameraListener* m_cameraManager;

    TransparentHeap m_tranparentHeap;
//	DemoVisualDebugerListener* m_visualDebugger;

	dFloat m_mainThreadGraphicsTime;
	dFloat m_mainThreadPhysicsTime;

	static int m_attributes[];

	DECLARE_EVENT_TABLE()

	friend class NewtonDemos;
	//friend class dRuntimeProfiler;
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

inline NewtonDemos* DemoEntityManager::GetRootWindow () const
{
	return m_mainWindow;
};


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
