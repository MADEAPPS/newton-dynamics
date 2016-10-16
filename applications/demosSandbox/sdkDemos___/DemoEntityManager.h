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
#ifndef __DEMO_MAIN_FRAME_H__
#define __DEMO_MAIN_FRAME_H__


struct GLFWwindow;
struct ImDrawData;

class DemoMesh;
class DemoEntity;
class DemoCamera;
class DemoMeshInterface;
class DemoCameraListener;

class DemoEntityManager: public dList <DemoEntity*>
{
	public:
	typedef void (*LaunchSDKDemoCallback) (DemoEntityManager* const scene);
	typedef void (*RenderHoodCallback) (DemoEntityManager* const manager, void* const context, int lineNumber);


	class ButtonKey
	{
		public:
		ButtonKey (bool initialState);

		bool UpdateTriggerButton (const DemoEntityManager* const mainWin, int keyCode);
		bool UpdatePushButton (const DemoEntityManager* const mainWin, int keyCode);
		bool GetPushButtonState() const { return m_state;}

		bool UpdateTriggerJoystick (const DemoEntityManager* const mainWin, int buttonMask);
		//bool IsMouseKeyDown (const DemoEntityManager* const mainWin, int key);
		//bool IsKeyDown (const DemoEntityManager* const mainWin, int key);

		private:
		bool m_state;
		bool m_memory0;
		bool m_memory1;
	};

	class EntityDictionary: public dTree<DemoEntity*, dScene::dTreeNode*>
	{
	};

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

	class SDKDemos
	{
		public:
		const char *m_name;
		const char *m_description;
		LaunchSDKDemoCallback m_launchDemoCallback;
	};


	DemoEntityManager ();
	~DemoEntityManager ();

	void Run();

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);


	NewtonWorld* GetNewton() const;
	void CreateSkyBox();

	void ResetTimer();
	void LoadScene (const char* const name);
	void RemoveEntity (DemoEntity* const ent);
	void RemoveEntity (dListNode* const entNode);

	DemoCamera* GetCamera() const;
	bool GetMousePosition (int& posX, int& posY) const;
	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);

	void PushTransparentMesh (const DemoMeshInterface* const mesh); 

	bool GetKeyState(int key) const;
	bool GetJoytickPosition (dFloat& posX, dFloat& posY, int& buttonsMask) const;

	static void SerializeFile (void* const serializeHandle, const void* const buffer, int size);
	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, void* const userData, NewtonSerializeCallback serializecallback, void* const serializeHandle);
	static void BodyDeserialization (NewtonBody* const body, void* const userData, NewtonDeserializeCallback serializecallback, void* const serializeHandle);

	private:
	void BeginFrame();
	void RenderUI();
	void LoadFont();
	void Cleanup();

	void RenderScene();
	void UpdatePhysics(dFloat timestep);
	dFloat CalculateInteplationParam () const;

	void CalculateFPS(dFloat timestep);

	
	void ShowMainMenuBar();
	void LoadVisualScene(dScene* const scene, EntityDictionary& dictionary);

	static void RenderDrawListsCallback(ImDrawData* const draw_data);
	static void KeyCallback(GLFWwindow* const window, int key, int, int action, int mods);

	static void CursorposCallback  (GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback (GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, int button, int action, int mods);
	static void ErrorCallback(int error, const char* const description);
	GLFWwindow* m_mainFrame;
	int	m_defaultFont;
	bool m_mousePressed[3];

	DemoEntity* m_sky;
	NewtonWorld* m_world;
	DemoCameraListener* m_cameraManager;
	void* m_renderHoodContext;
	RenderHoodCallback m_renderHood;

	unsigned64 m_microsecunds;
	TransparentHeap m_tranparentHeap;

	int m_currentScene;
	int m_framesCount;
	int m_physicsFramesCount;
	dFloat m_fps;
	dFloat m_timestepAcc;
	dFloat m_currentListenerTimestep;
	dFloat m_mainThreadPhysicsTime;
	dFloat m_mainThreadPhysicsTimeAcc;
	bool m_showStats;
	bool m_synchronousPhysicsUpdateMode;
	bool m_hideVisualMeshes;


	static SDKDemos m_demosSelection[];
	friend class DemoEntityListener;
	friend class DemoListenerManager;
};


inline NewtonWorld* DemoEntityManager::GetNewton() const
{
	return m_world;
}


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


#endif