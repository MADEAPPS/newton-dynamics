/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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

#include "ndSandboxStdafx.h"
#include "ndShaderPrograms.h"

struct GLFWwindow;
struct ImDrawData;

class ndDemoMesh;
class ndDemoEntity;
class ndPhysicsWorld;
//class ndDemoCamera;
//class DemoMeshInterface;
class ndDemoCameraManager;

class ndDemoEntityManager: public dList <ndDemoEntity*>
{
#if 0
	public:
	typedef void(*UpdateCameraCallback) (ndDemoEntityManager* const manager, void* const context, dFloat32 timestep);

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

	class TransparentHeap: public dUpHeap <TransparentMesh, dFloat32>
	{
		public:
		TransparentHeap()
			:dUpHeap <TransparentMesh, dFloat32>(2048)
		{
		}
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

	

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	int GetWidth() const;
	int GetHeight() const;

	NewtonWorld* GetNewton() const;

	void LoadScene (const char* const name);

	void ImportPLYfile (const char* const name);

	DemoCamera* GetCamera() const;
	bool GetMousePosition (int& posX, int& posY) const;

	void PushTransparentMesh (const DemoMeshInterface* const mesh); 
	void SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context);
	void Set2DDisplayRenderFunction (RenderGuiHelpCallback helpCallback, RenderGuiHelpCallback UIcallback, void* const context);

	bool IsShiftKeyDown () const;
	bool IsControlKeyDown () const;
	bool GetKeyState(int key) const;
	int GetJoystickAxis (dFloat32* const axisValues, int maxAxis = 8) const;
	int GetJoystickButtons (char* const axisbuttons, int maxButton = 32) const;

	void SerializedPhysicScene(const char* const name);
	void DeserializedPhysicScene(const char* const name);

	static void SerializeFile (void* const serializeHandle, const void* const buffer, int size);
	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, void* const userData, NewtonSerializeCallback serializecallback, void* const serializeHandle);
	static void BodyDeserialization (NewtonBody* const body, void* const userData, NewtonDeserializeCallback serializecallback, void* const serializeHandle);

	static void OnCreateContact(const NewtonWorld* const world, NewtonJoint* const contact);
	static void OnDestroyContact(const NewtonWorld* const world, NewtonJoint* const contact);

	bool GetCaptured () const;
	bool GetMouseKeyState (int button ) const;
	int Print (const dVector& color, const char *fmt, ... ) const;
	int GetDebugDisplay() const;
	void SetDebugDisplay(int mode) const;

	private:
	//void RenderUI();
	dFloat32 CalculateInteplationParam () const;
	
	void LoadVisualScene(dScene* const scene, EntityDictionary& dictionary);

	void ToggleProfiler();
	static void PostUpdateCallback(const NewtonWorld* const world, dFloat32 timestep);
	
	void* m_renderUIContext;
	void* m_updateCameraContext;
	
	RenderGuiHelpCallback m_renderHelpMenus;
	UpdateCameraCallback m_updateCamera;

	
	TransparentHeap m_tranparentHeap;
	
	int m_physicsFramesCount;
	int m_currentPlugin;
	
	dFloat32 m_currentListenerTimestep;
	
	dFloat32 m_mainThreadPhysicsTimeAcc;

	int m_broadPhaseType;
	int m_debugDisplayMode;
	int m_collisionDisplayMode;
	
	bool m_showNormalForces;
	bool m_showCenterOfMass;
	bool m_showBodyFrame;
	bool m_showContactPoints;
	bool m_showJointDebugInfo;
	bool m_showListenersDebugInfo;
	bool m_showCollidingFaces;
	bool m_solveLargeIslandInParallel;
	bool m_showRaycastHit;

	unsigned m_profilerMode;
	unsigned m_contactLock;
	dList<NewtonJoint*> m_contactList;
	friend class DemoEntityListener;
	friend class DemoListenerManager;
#endif
	public:
	ndDemoEntityManager();
	~ndDemoEntityManager();

	void Run();

	void CreateSkyBox();
	const ndShaderPrograms& GetShaderCache() const;

	void AddEntity(ndDemoEntity* const ent);
	void RemoveEntity(ndDemoEntity* const ent);

	void SetCameraMatrix(const dQuaternion& rotation, const dVector& position);

	private:
	typedef void(*RenderGuiHelpCallback) (ndDemoEntityManager* const manager, void* const context);
	typedef void(*LaunchSDKDemoCallback) (ndDemoEntityManager* const scene);

	class SDKDemos
	{
		public:
		const char* m_description;
		LaunchSDKDemoCallback m_launchDemoCallback;
	};

	static void RenderDrawListsCallback(ImDrawData* const draw_data);
	static void ErrorCallback(int error, const char* const description);
	static void CharCallback(GLFWwindow* window, unsigned int ch);
	static void KeyCallback(GLFWwindow* const window, int key, int, int action, int mods);
	static void CursorposCallback(GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback(GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, int button, int action, int mods);

	void Cleanup();
	void LoadFont();
	void ResetTimer();
	void BeginFrame();
	void RenderScene();
	void RenderStats();

	void LoadDemo(int menu);
	void ShowMainMenuBar();
	void ApplyMenuOptions();

	void CalculateFPS();
	void UpdatePhysics();

	GLFWwindow* m_mainFrame;
	ndPhysicsWorld* m_world;
	ndDemoCameraManager* m_cameraManager;
	ndDemoEntity* m_sky;
	ndShaderPrograms m_shadeCache;

	dFloat32 m_fps;
	dUnsigned64 m_microsecunds;

	dSpinLock m_addDeleteLock;

	int	m_defaultFont;
	int m_framesCount;
	int m_currentScene;
	int m_solverPasses;
	int m_workerThreads;
	int m_solverSubSteps;
	int m_lastCurrentScene;

	bool m_showUI;
	bool m_showAABB;
	bool m_showStats;
	bool m_hasJoytick;
	bool m_autoSleepMode;
	bool m_hideVisualMeshes;
	bool m_updateMenuOptions;
	bool m_suspendPhysicsUpdate;
	bool m_asynchronousPhysicsUpdate;
	bool m_mousePressed[3];
	RenderGuiHelpCallback m_renderDemoGUI;

	static SDKDemos m_demosSelection[];

	friend class ndPhysicsWorld;
};

#if 0
inline NewtonWorld* ndDemoEntityManager::GetNewton() const
{
	return m_world;
}

// for simplicity we are not going to run the demo in a separate thread at this time
// this confuses many user int thinking it is more complex than it really is  
inline void ndDemoEntityManager::Lock(unsigned& atomicLock)
{
	while (NewtonAtomicSwap((int*)&atomicLock, 1)) {
		NewtonYield();
	}
}

inline void ndDemoEntityManager::Unlock(unsigned& atomicLock)
{
	NewtonAtomicSwap((int*)&atomicLock, 0);
}

inline int ndDemoEntityManager::GetWidth() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
}

inline int ndDemoEntityManager::GetHeight() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
}

inline int ndDemoEntityManager::GetDebugDisplay() const
{
	dAssert (0);
	return 0;
}

inline void ndDemoEntityManager::SetDebugDisplay(int mode) const
{
	dAssert (0);
}
#endif

inline const ndShaderPrograms& ndDemoEntityManager::GetShaderCache() const
{
	return m_shadeCache;
}


#endif