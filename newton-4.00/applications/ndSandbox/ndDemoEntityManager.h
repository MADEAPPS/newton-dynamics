/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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

class ndDemoMesh;
class ndUIEntity;
class ndRenderPass;
class ndMeshLoader;
class ndPhysicsWorld;
class ndAnimationSequence;
class ndDemoMeshInterface;

//class ndDemoEntityManager : public ndList <ndSharedPtr<ndDemoEntity>>
class ndDemoEntityManager : public ndClassAlloc
{
	public:
	class ndRenderCallback : public ndRender::ndUserCallback
	{
		public:
		ndRenderCallback(ndDemoEntityManager* const owner)
			:ndRender::ndUserCallback()
			,m_owner(owner)
		{
		}

		virtual void KeyCallback(ndInt32 key, ndInt32 action) override
		{
			m_owner->KeyCallback(key, action);
		}

		virtual void CharCallback(ndUnsigned32 ch)
		{
			m_owner->CharCallback(ch);
		}

		virtual void CursorposCallback(ndReal x, ndReal y)
		{
			m_owner->CursorposCallback(x, y);
		}

		virtual void MouseScrollCallback(ndReal x, ndReal y)
		{
			m_owner->MouseScrollCallback(x, y);
		}

		virtual void MouseButtonCallback(ndInt32 button, ndInt32 action)
		{
			m_owner->MouseButtonCallback(button, action);
		}

		ndDemoEntityManager* m_owner;
	};


	typedef void (*LaunchSDKDemoCallback) (ndDemoEntityManager* const scene);
	typedef void(*UpdateCameraCallback) (ndDemoEntityManager* const manager, void* const context, ndFloat32 timestep);

	class OnPostUpdate: public ndClassAlloc
	{
		public:
		OnPostUpdate()
			:ndClassAlloc()
		{
		}

		virtual ~OnPostUpdate()
		{
		}
		
		virtual void OnDebug(ndDemoEntityManager* const, bool) {}
		virtual void Update(ndDemoEntityManager* const scene, ndFloat32 timestep) = 0;
	};

	enum ndMenuSelection
	{
		m_new,
		//m_load,
		//m_save,
		//m_saveModel,
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
			bool ret = bool (!m_memory & value);
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
			:m_matrix(ndGetIdentityMatrix())
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

	ndDemoEntityManager ();
	~ndDemoEntityManager ();

	void Run();

	ndInt32 GetWidth() const;
	ndInt32 GetHeight() const;
	
	ndPhysicsWorld* GetWorld() const;
	ndSharedPtr<ndRender>& GetRenderer();

	void AddEntity(const ndSharedPtr<ndRenderSceneNode>& entity);
	void RemoveEntity(const ndSharedPtr<ndRenderSceneNode>& entity);
	
	void ResetTimer();
	void ImportPLYfile (const char* const name);

	bool GetMouseSpeed(ndFloat32& posX, ndFloat32& posY) const;
	bool GetMousePosition (ndFloat32& posX, ndFloat32& posY) const;
	void SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position);
	
	void* GetUpdateCameraContext() const;
	void SetSelectedModel(ndModel* const model);
	void SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context);
	void PushTransparentMesh(const ndDemoMeshInterface* const mesh, const ndMatrix& modelMatrix);
	void Set2DDisplayRenderFunction(ndSharedPtr<ndUIEntity>& demoGui);

	bool AnyKeyDown() const;
	bool IsShiftKeyDown () const;
	bool JoystickDetected() const;
	bool IsControlKeyDown () const;
	bool GetKeyState(ndInt32 key) const;
	void GetJoystickAxis (ndFixSizeArray<ndFloat32, 8>& axisValues);
	void GetJoystickButtons (ndFixSizeArray<char, 32>& axisbuttons);

	void Terminate();

	void CharCallback(ndUnsigned32 ch);
	void CursorposCallback(ndReal x, ndReal y);
	void MouseScrollCallback(ndReal x, ndReal y);
	void KeyCallback(ndInt32 key, ndInt32 action);
	void MouseButtonCallback(ndInt32 button, ndInt32 action);

	bool GetCaptured () const;
	bool GetMouseKeyState (ndInt32 button ) const;
	ndInt32 Print (const ndVector& color, const char *fmt, ... ) const;
	ndInt32 GetDebugDisplay() const;
	void SetDebugDisplay(ndInt32 mode) const;

	void SetAcceleratedUpdate(); 
	ndVector GetDirectionsLight() const;
	ndSharedPtr<ndAnimationSequence> GetAnimationSequence(ndMeshLoader& loader, const char* const meshName);
	void RegisterPostUpdate(OnPostUpdate* const postUpdate);

	void RenderStats();
	private:
	void Cleanup();

	void RenderScene();
	ndInt32 ParticleCount() const;
	void SetParticleUpdateMode() const;
	
	void UpdatePhysics(ndFloat32 timestep);
	ndFloat32 CalculateInteplationParam () const;

	void CalculateFPS(ndFloat32 timestep);
	
	void ShowMainMenuBar();
	void ToggleProfiler();

	void ApplyMenuOptions();
	void LoadDemo(ndInt32 menu);
	void OnSubStepPostUpdate(ndFloat32 timestep);

	void TestImGui();

	
	ndSharedPtr<ndRender> m_renderer;
	ndSharedPtr<ndRenderPass> m_menuRenderPass;
	ndSharedPtr<ndRenderPass> m_colorRenderPass;
	ndSharedPtr<ndRenderPass> m_shadowRenderPass;
	ndSharedPtr<ndRenderPass> m_environmentRenderPass;
	ndSharedPtr<ndRenderTexture> m_environmentTexture;

	ndPhysicsWorld* m_world;
	void* m_updateCameraContext;
	
	//ndSharedPtr<ndUIEntity> m_renderDemoGUI;
	UpdateCameraCallback m_updateCamera;

	ndUnsigned64 m_microsecunds;
	TransparentHeap m_transparentHeap;
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString> m_animationCache;

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
	OnPostUpdate* m_onPostUpdate;

	ndFloat32 m_fps;
	ndFloat32 m_timestepAcc;
	ndFloat32 m_currentListenerTimestep;
	ndSpinLock m_addDeleteLock;
	
	bool m_showUI;
	bool m_showAABB;
	bool m_showStats;
	bool m_autoSleepMode;
	bool m_showScene;
	bool m_showConcaveEdge;
	bool m_hideVisualMeshes;
	bool m_showNormalForces;
	bool m_showCenterOfMass;
	bool m_showBodyFrame;
	bool m_showMeshSkeleton;
	bool m_updateMenuOptions;
	bool m_showContactPoints;
	bool m_showJointDebugInfo;
	bool m_showModelsDebugInfo;
	bool m_showCollidingFaces;
	bool m_hidePostUpdate;
	bool m_suspendPhysicsUpdate;
	bool m_synchronousPhysicsUpdate;
	bool m_synchronousParticlesUpdate;
	bool m_showRaycastHit;
	bool m_profilerMode;
	
	ndWorld::ndSolverModes m_solverMode;

	ndVector m_diretionalLightDir;
	static SDKDemos m_demosSelection[];

	friend class ndPhysicsWorld;
};

inline ndPhysicsWorld* ndDemoEntityManager::GetWorld() const
{
	return m_world;
}

inline ndSharedPtr<ndRender>& ndDemoEntityManager::GetRenderer()
{
	return m_renderer;
}

inline ndInt32 ndDemoEntityManager::GetDebugDisplay() const
{
	ndAssert (0);
	return 0;
}

inline void ndDemoEntityManager::SetDebugDisplay(ndInt32) const
{
	ndAssert (0);
}

inline void ndDemoEntityManager::SetSelectedModel(ndModel* const model)
{
	m_selectedModel = model;
}

inline ndVector ndDemoEntityManager::GetDirectionsLight() const
{
	return m_diretionalLightDir;
}

#endif