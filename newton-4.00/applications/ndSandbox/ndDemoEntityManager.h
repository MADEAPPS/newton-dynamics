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
#ifndef __DEMO_MAIN_FRAME_H__
#define __DEMO_MAIN_FRAME_H__

#include "ndShaderPrograms.h"

struct GLFWwindow;
struct ImDrawData;

class ndDemoMesh;
class ndDemoEntity;
class ndDemoCamera;
class ndPhysicsWorld;
class ndDemoMeshInterface;
class ndDemoCameraManager;
class ndWireFrameDebugMesh;
class ndFlatShadedDebugMesh;

class ndDemoEntityManager: public dList <ndDemoEntity*>
{
	public:
	typedef void (*LaunchSDKDemoCallback) (ndDemoEntityManager* const scene);
	typedef void (*RenderGuiHelpCallback) (ndDemoEntityManager* const manager, void* const context);
	typedef void(*UpdateCameraCallback) (ndDemoEntityManager* const manager, void* const context, dFloat32 timestep);

	class ndKeyTrigger
	{
		public: 
		ndKeyTrigger()
			:m_memory(false)
		{
		}

		bool Update(bool value)
		{
			bool ret = !m_memory & value;
			m_memory = value;
			return ret;
		}

		bool m_memory;
	};

	class ndLightSource
	{
		public:
		dVector m_position;
		dVector m_ambient;
		dVector m_diffuse;
		dVector m_specular;
		dFloat32 m_shininess;
	};


	class TransparentMesh
	{
		public: 
		TransparentMesh()
			:m_matrix(dGetIdentityMatrix())
			,m_mesh(nullptr)
		{
		}

		TransparentMesh(const dMatrix& matrix, ndDemoMesh* const mesh)
			:m_matrix(matrix)
			,m_mesh(mesh)
		{
		}

		dMatrix m_matrix;
		ndDemoMesh* m_mesh;
	};

	class TransparentHeap: public dUpHeap <TransparentMesh, dFloat32>
	{
		public:
		TransparentHeap()
			:dUpHeap <TransparentMesh, dFloat32>(2048)
		{
		}
	};

	class SDKDemos
	{
		public:
		const char *m_name;
		LaunchSDKDemoCallback m_launchDemoCallback;
	};

	class ButtonKey
	{
		public:
		ButtonKey (bool initialState);
		dInt32 UpdateTrigger (bool triggerValue);
		dInt32 UpdatePushButton (bool triggerValue);
		dInt32 GetPushButtonState() const { return m_state ? 1 : 0;}

		private:
		bool m_state;
		bool m_memory0;
		bool m_memory1;
	};

	class ndDebuMesh
	{
		public:
		ndDebuMesh()
			:m_flatShaded(nullptr)
			,m_wireFrameOpenEdge(nullptr)
			,m_wireFrameShareEdge(nullptr)
		{
		}

		ndFlatShadedDebugMesh* m_flatShaded;
		ndWireFrameDebugMesh* m_wireFrameOpenEdge;
		ndWireFrameDebugMesh* m_wireFrameShareEdge;
	};

	class ndDebugMeshCache: public dTree<ndDebuMesh, const ndShape*>
	{
	};
	

	ndDemoEntityManager ();
	~ndDemoEntityManager ();

	void Run();

	void AddEntity(ndDemoEntity* const ent);
	void RemoveEntity(ndDemoEntity* const ent);

	dInt32 GetWidth() const;
	dInt32 GetHeight() const;

	ndPhysicsWorld* GetWorld() const;

	void CreateSkyBox();
	void ResetTimer();
	void ImportPLYfile (const char* const name);

	ndDemoCamera* GetCamera() const;
	ndDemoCameraManager* GetCameraManager() const;
	bool GetMouseSpeed(dFloat32& posX, dFloat32& posY) const;
	bool GetMousePosition (dFloat32& posX, dFloat32& posY) const;
	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);
	
	void SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context);
	void PushTransparentMesh(const ndDemoMeshInterface* const mesh, const dMatrix& modelMatrix);
	void Set2DDisplayRenderFunction (RenderGuiHelpCallback helpCallback, RenderGuiHelpCallback UIcallback, void* const context);

	bool IsShiftKeyDown () const;
	bool IsControlKeyDown () const;
	bool GetKeyState(dInt32 key) const;
	dInt32 GetJoystickAxis (dFixSizeBuffer<dFloat32, 8>& axisValues);
	dInt32 GetJoystickButtons (dFixSizeBuffer<char, 32>& axisbuttons);

	bool GetCaptured () const;
	bool GetMouseKeyState (dInt32 button ) const;
	dInt32 Print (const dVector& color, const char *fmt, ... ) const;
	dInt32 GetDebugDisplay() const;
	void SetDebugDisplay(dInt32 mode) const;

	const ndShaderPrograms& GetShaderCache() const;  
	
	private:
	void BeginFrame();
	void RenderStats();
	void LoadFont();
	void Cleanup();

	//void RenderUI();
	void RenderScene();
	
	void UpdatePhysics(dFloat32 timestep);
	dFloat32 CalculateInteplationParam () const;

	void CalculateFPS(dFloat32 timestep);
	
	void ShowMainMenuBar();
	//void LoadVisualScene(dScene* const scene, EntityDictionary& dictionary);

	void ToggleProfiler();

	static void RenderDrawListsCallback(ImDrawData* const draw_data);

	static void CharCallback(GLFWwindow* window, dUnsigned32 ch);
	static void KeyCallback(GLFWwindow* const window, dInt32 key, dInt32, dInt32 action, dInt32 mods);
	static void CursorposCallback  (GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback (GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, dInt32 button, dInt32 action, dInt32 mods);
	static void ErrorCallback(dInt32 error, const char* const description);
	//static void PostUpdateCallback(const NewtonWorld* const world, dFloat32 timestep);

	void ApplyMenuOptions();
	void LoadDemo(dInt32 menu);

	void DrawDebugShapes();
	
	GLFWwindow* m_mainFrame;
	dInt32	m_defaultFont;
	bool m_mousePressed[3];

	ndDemoEntity* m_sky;
	ndPhysicsWorld* m_world;
	ndDemoCameraManager* m_cameraManager;
	ndShaderPrograms m_shaderCache;
	void* m_renderUIContext;
	void* m_updateCameraContext;
	RenderGuiHelpCallback m_renderDemoGUI;
	RenderGuiHelpCallback m_renderHelpMenus;
	UpdateCameraCallback m_updateCamera;

	dUnsigned64 m_microsecunds;
	TransparentHeap m_tranparentHeap;

	dInt32 m_currentScene;
	dInt32 m_lastCurrentScene;
	dInt32 m_framesCount;
	dInt32 m_physicsFramesCount;
	dInt32 m_currentPlugin;
	dInt32 m_solverPasses;
	dInt32 m_solverSubSteps;
	dInt32 m_sceneType;
	dInt32 m_workerThreads;
	dInt32 m_debugDisplayMode;
	dInt32 m_collisionDisplayMode;

	dFloat32 m_fps;
	dFloat32 m_timestepAcc;
	dFloat32 m_currentListenerTimestep;
	dSpinLock m_addDeleteLock;
	
	bool m_showUI;
	bool m_showAABB;
	bool m_showStats;
	bool m_hasJoytick;
	bool m_autoSleepMode;
	bool m_showScene;
	bool m_showConcaveEdge;
	bool m_hideVisualMeshes;
	bool m_showNormalForces;
	bool m_showCenterOfMass;
	bool m_showBodyFrame;
	bool m_updateMenuOptions;
	bool m_showContactPoints;
	bool m_showJointDebugInfo;
	bool m_showModelsDebugInfo;
	bool m_showCollidingFaces;
	bool m_suspendPhysicsUpdate;
	bool m_synchronousPhysicsUpdate;
	bool m_showRaycastHit;
	bool m_profilerMode;

	ndWorld::ndSolverModes m_solverMode;

	ndLightSource m_directionalLight;
	ndDebugMeshCache m_debugShapeCache;

	static SDKDemos m_demosSelection[];
	friend class ndPhysicsWorld;
};

inline ndPhysicsWorld* ndDemoEntityManager::GetWorld() const
{
	return m_world;
}

inline dInt32 ndDemoEntityManager::GetWidth() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (dInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
}

inline dInt32 ndDemoEntityManager::GetHeight() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (dInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
}

inline dInt32 ndDemoEntityManager::GetDebugDisplay() const
{
	dAssert (0);
	return 0;
}

inline void ndDemoEntityManager::SetDebugDisplay(dInt32) const
{
	dAssert (0);
}

inline const ndShaderPrograms& ndDemoEntityManager::GetShaderCache() const
{
	return m_shaderCache;
}

inline ndDemoCameraManager* ndDemoEntityManager::GetCameraManager() const
{
	return m_cameraManager;
}

#endif