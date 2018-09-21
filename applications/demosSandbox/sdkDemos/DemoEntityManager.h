/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
class DemoCameraManager;

class DemoEntityManager: public dList <DemoEntity*>
{
	public:
	typedef void (*LaunchSDKDemoCallback) (DemoEntityManager* const scene);
	typedef void (*RenderGuiHelpCallback) (DemoEntityManager* const manager, void* const context);
	typedef void(*UpdateCameraCallback) (DemoEntityManager* const manager, void* const context, dFloat timestep);

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

	class ButtonKey
	{
		public:
		ButtonKey (bool initialState);
		int UpdateTrigger (bool triggerValue);
		int UpdatePushButton (bool triggerValue);
		int GetPushButtonState() const { return m_state ? 1 : 0;}

		private:
		bool m_state;
		bool m_memory0;
		bool m_memory1;
	};

	class EntityDictionary: public dTree<DemoEntity*, dScene::dTreeNode*>
	{
	};

	DemoEntityManager ();
	~DemoEntityManager ();

	void Run();

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	int GetWidth() const;
	int GetHeight() const;

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
	void SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context);
	void Set2DDisplayRenderFunction (RenderGuiHelpCallback helpCallback, RenderGuiHelpCallback UIcallback, void* const context);

	bool IsShiftKeyDown () const;
	bool IsControlKeyDown () const;
	bool GetKeyState(int key) const;
	int GetJoystickAxis (dFloat* const axisValues, int maxAxis = 8) const;
	int GetJoystickButtons (char* const axisbuttons, int maxButton = 32) const;

	static void SerializeFile (void* const serializeHandle, const void* const buffer, int size);
	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, void* const userData, NewtonSerializeCallback serializecallback, void* const serializeHandle);
	static void BodyDeserialization (NewtonBody* const body, void* const userData, NewtonDeserializeCallback serializecallback, void* const serializeHandle);

	bool GetMouseKeyState (int button ) const;
	int Print (const dVector& color, const char *fmt, ... ) const;
	int GetDebugDisplay() const;
	void SetDebugDisplay(int mode) const;

	private:
	void BeginFrame();
	void RenderStats();
	void LoadFont();
	void Cleanup();

	//void RenderUI();
	void RenderScene();
	
	void UpdatePhysics(dFloat timestep);
	dFloat CalculateInteplationParam () const;

	void CalculateFPS(dFloat timestep);
	
	void ShowMainMenuBar();
	void LoadVisualScene(dScene* const scene, EntityDictionary& dictionary);

	static void RenderDrawListsCallback(ImDrawData* const draw_data);

	static void CharCallback(GLFWwindow* window, unsigned int ch);
	static void KeyCallback(GLFWwindow* const window, int key, int, int action, int mods);
	static void CursorposCallback  (GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback (GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, int button, int action, int mods);
	static void ErrorCallback(int error, const char* const description);
	static void PostUpdateCallback(const NewtonWorld* const world, dFloat timestep);

	void ApplyMenuOptions();

	GLFWwindow* m_mainFrame;
	int	m_defaultFont;
	bool m_mousePressed[3];

	DemoEntity* m_sky;
	NewtonWorld* m_world;
	DemoCameraManager* m_cameraManager;
	void* m_renderUIContext;
	void* m_updateCameraContext;
	RenderGuiHelpCallback m_renderDemoGUI;
	RenderGuiHelpCallback m_renderHelpMenus;
	UpdateCameraCallback m_updateCamera;

	unsigned64 m_microsecunds;
	TransparentHeap m_tranparentHeap;

	int m_currentScene;
	int m_framesCount;
	int m_physicsFramesCount;
	int m_currentPlugin;
	dFloat m_fps;
	dFloat m_timestepAcc;
	dFloat m_currentListenerTimestep;
	dFloat m_mainThreadPhysicsTime;
	dFloat m_mainThreadPhysicsTimeAcc;

	int m_solverPasses;
	int m_broadPhaseType;
	int m_workerThreads;
	int m_debugDisplayMode;
	int m_collisionDisplayMode;
	
	bool m_showUI;
	bool m_showAABB;
	bool m_showStats;
	bool m_hasJoytick;
	bool m_autoSleepMode;
	bool m_hideVisualMeshes;
	bool m_showNormalForces;
	bool m_showCenterOfMass;
	bool m_showBodyFrame;
	bool m_updateMenuOptions;
	bool m_showContactPoints;
	bool m_showJointDebugInfo;
	bool m_showCollidingFaces;
	bool m_suspendPhysicsUpdate;
	bool m_asynchronousPhysicsUpdate;
	bool m_solveLargeIslandInParallel;

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

inline int DemoEntityManager::GetWidth() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
}

inline int DemoEntityManager::GetHeight() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
}

inline int DemoEntityManager::GetDebugDisplay() const
{
	dAssert (0);
	return 0;
}

inline void DemoEntityManager::SetDebugDisplay(int mode) const
{
	dAssert (0);
}



#endif