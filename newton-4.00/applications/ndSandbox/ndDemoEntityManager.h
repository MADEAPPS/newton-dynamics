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
class fbxDemoEntity;
class ndPhysicsWorld;
class ndAnimationSequence;
class ndDemoMeshInterface;
class ndDemoCameraManager;
class ndWireFrameDebugMesh;
class ndFlatShadedDebugMesh;

class ndDemoEntityManager: public ndList <ndDemoEntity*>
{
	public:
	typedef void (*LaunchSDKDemoCallback) (ndDemoEntityManager* const scene);
	typedef void (*RenderGuiHelpCallback) (ndDemoEntityManager* const manager, void* const context);
	typedef void(*UpdateCameraCallback) (ndDemoEntityManager* const manager, void* const context, ndFloat32 timestep);

	enum ndMenuSelection
	{
		m_new,
		m_load,
		m_save,
		m_saveModel,
		m_none,
	};

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
		ndVector m_position;
		ndVector m_ambient;
		ndVector m_diffuse;
		ndVector m_specular;
		ndFloat32 m_shininess;
	};


	class TransparentMesh
	{
		public: 
		TransparentMesh()
			:m_matrix(dGetIdentityMatrix())
			,m_mesh(nullptr)
		{
		}

		TransparentMesh(const ndMatrix& matrix, ndDemoMesh* const mesh)
			:m_matrix(matrix)
			,m_mesh(mesh)
		{
		}

		ndMatrix m_matrix;
		ndDemoMesh* m_mesh;
	};

	class TransparentHeap: public ndUpHeap <TransparentMesh, ndFloat32>
	{
		public:
		TransparentHeap()
			:ndUpHeap <TransparentMesh, ndFloat32>(2048)
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
		ndInt32 UpdateTrigger (bool triggerValue);
		ndInt32 UpdatePushButton (bool triggerValue);
		ndInt32 GetPushButtonState() const { return m_state ? 1 : 0;}

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

	class ndDebugMeshCache: public ndTree<ndDebuMesh, const ndShape*>
	{
	};

	ndDemoEntityManager ();
	~ndDemoEntityManager ();

	void Run();

	void AddEntity(ndDemoEntity* const ent);
	void RemoveEntity(ndDemoEntity* const ent);

	ndInt32 GetWidth() const;
	ndInt32 GetHeight() const;

	ndPhysicsWorld* GetWorld() const;
	
	void CreateSkyBox();
	void ResetTimer();
	void ImportPLYfile (const char* const name);

	ndDemoCamera* GetCamera() const;
	ndDemoCameraManager* GetCameraManager() const;
	bool GetMouseSpeed(ndFloat32& posX, ndFloat32& posY) const;
	bool GetMousePosition (ndFloat32& posX, ndFloat32& posY) const;
	void SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position);
	
	void SetSelectedModel(ndModel* const model);
	void SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context);
	void PushTransparentMesh(const ndDemoMeshInterface* const mesh, const ndMatrix& modelMatrix);
	void Set2DDisplayRenderFunction (RenderGuiHelpCallback helpCallback, RenderGuiHelpCallback UIcallback, void* const context);

	bool IsShiftKeyDown () const;
	bool IsControlKeyDown () const;
	bool GetKeyState(ndInt32 key) const;
	ndInt32 GetJoystickAxis (ndFixSizeArray<ndFloat32, 8>& axisValues);
	ndInt32 GetJoystickButtons (ndFixSizeArray<char, 32>& axisbuttons);

	bool GetCaptured () const;
	bool GetMouseKeyState (ndInt32 button ) const;
	ndInt32 Print (const ndVector& color, const char *fmt, ... ) const;
	ndInt32 GetDebugDisplay() const;
	void SetDebugDisplay(ndInt32 mode) const;

	const ndShaderPrograms& GetShaderCache() const;  
	fbxDemoEntity* LoadFbxMesh(const char* const meshName);
	ndAnimationSequence* GetAnimationSequence(const char* const meshName);
	
	private:
	void BeginFrame();
	void RenderStats();
	void LoadFont();
	void Cleanup();

	//void RenderUI();
	void RenderScene();
	ndInt32 ParticleCount() const;
	
	void UpdatePhysics(ndFloat32 timestep);
	ndFloat32 CalculateInteplationParam () const;

	void CalculateFPS(ndFloat32 timestep);
	
	void ShowMainMenuBar();
	void ToggleProfiler();

	static void RenderDrawListsCallback(ImDrawData* const draw_data);

	static void CharCallback(GLFWwindow* window, ndUnsigned32 ch);
	static void KeyCallback(GLFWwindow* const window, ndInt32 key, ndInt32, ndInt32 action, ndInt32 mods);
	static void CursorposCallback  (GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback (GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, ndInt32 button, ndInt32 action, ndInt32 mods);
	static void ErrorCallback(ndInt32 error, const char* const description);
	static void OpenMessageCallback(
		GLenum source, GLenum type, GLuint id, GLenum severity,
		GLsizei length, const GLchar* message, const void* userParam);
	

	void ApplyMenuOptions();
	void LoadDemo(ndInt32 menu);

	void DrawDebugShapes();
	
	GLFWwindow* m_mainFrame;
	ndInt32	m_defaultFont;
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

	ndUnsigned64 m_microsecunds;
	TransparentHeap m_tranparentHeap;
	ndTree<ndAnimationSequence*, ndString> m_animationCache;

	ndInt32 m_currentScene;
	ndInt32 m_lastCurrentScene;
	ndInt32 m_framesCount;
	ndInt32 m_physicsFramesCount;
	ndInt32 m_currentPlugin;
	ndInt32 m_solverPasses;
	ndInt32 m_solverSubSteps;
	ndInt32 m_workerThreads;
	ndInt32 m_debugDisplayMode;
	ndInt32 m_collisionDisplayMode;
	ndModel* m_selectedModel;

	ndFloat32 m_fps;
	ndFloat32 m_timestepAcc;
	ndFloat32 m_currentListenerTimestep;
	ndSpinLock m_addDeleteLock;
	
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

	FILE* m_replayLogFile;
	static SDKDemos m_demosSelection[];
	friend class ndPhysicsWorld;
};

inline ndPhysicsWorld* ndDemoEntityManager::GetWorld() const
{
	return m_world;
}

inline ndInt32 ndDemoEntityManager::GetWidth() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
}

inline ndInt32 ndDemoEntityManager::GetHeight() const 
{ 
	ImGuiIO& io = ImGui::GetIO();
	return (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
}

inline ndInt32 ndDemoEntityManager::GetDebugDisplay() const
{
	dAssert (0);
	return 0;
}

inline void ndDemoEntityManager::SetDebugDisplay(ndInt32) const
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

inline void ndDemoEntityManager::SetSelectedModel(ndModel* const model)
{
	m_selectedModel = model;
}

#endif