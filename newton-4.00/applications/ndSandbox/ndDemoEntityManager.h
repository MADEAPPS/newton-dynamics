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

class ndUIEntity;
class ndPhysicsWorld;

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

	typedef void (*ndDemoCallbackLauncher) (ndDemoEntityManager* const scene);

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

	class ndDemos
	{
		public:
		const char *m_name;
		ndDemoCallbackLauncher m_demoLauncher;
	};

	class ndDemoHelper: public ndClassAlloc
	{
		public:
		ndDemoHelper()
			:ndClassAlloc()
			,m_currentTime(ndGetTimeInMicroseconds())
		{
			ResetTime();
		}

		void ResetTime()
		{
			m_currentTime = ndGetTimeInMicroseconds();
		}

		bool ExpirationTime() const
		{
			// stops diplay the legend afte 5 secunds
			ndUnsigned64 timestep = ndGetTimeInMicroseconds() - m_currentTime;
			return timestep > 5 * 1024 * 1024;
		}

		virtual ~ndDemoHelper() {}
		virtual void PresentHelp(ndDemoEntityManager* const scene) = 0;

		ndUnsigned64 m_currentTime;
	};

	class OnPostUpdate : public ndClassAlloc
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

	void ImportPLYfile (const char* const name);

	bool GetMouseSpeed(ndFloat32& posX, ndFloat32& posY) const;
	bool GetMousePosition (ndFloat32& posX, ndFloat32& posY) const;
	void SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position);
	
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

	void RenderStats();
	void SetAcceleratedUpdate();
	void SetDemoHelp(ndSharedPtr<ndDemoHelper>& helper);

	void SetNextActiveCamera();

	void RegisterPostUpdate(const ndSharedPtr<OnPostUpdate>& postUpdate);

	private:
	void Cleanup();

	void RenderScene();
	ndInt32 ParticleCount() const;
	void SetParticleUpdateMode() const;

	void CalculateFPS(ndFloat32 timestep);
	void UpdatePhysics(ndFloat32 timestep);
	
	void ShowMainMenuBar();
	void ToggleProfiler();
	void ApplyOptions();

	void ApplyMenuOptions();
	void LoadDemo(ndInt32 menu);
	void OnSubStepPostUpdate(ndFloat32 timestep);

	void TestImGui();
	
	ndPhysicsWorld* m_world;
	ndSharedPtr<ndRender> m_renderer;
	ndSharedPtr<ndRenderPass> m_menuRenderPass;
	ndSharedPtr<ndRenderPass> m_colorRenderPass;
	ndSharedPtr<ndRenderPass> m_shadowRenderPass;
	ndSharedPtr<ndRenderPass> m_environmentRenderPass;
	ndSharedPtr<ndRenderPass> m_transparentRenderPass;
	ndSharedPtr<ndRenderPass> m_debugDisplayRenderPass;
	ndSharedPtr<ndRenderTexture> m_environmentTexture;

	ndSharedPtr<ndDemoHelper> m_demoHelper;
	ndSharedPtr<ndRenderSceneNode> m_defaultCamera;

	ndSharedPtr<OnPostUpdate> m_onPostUpdate;

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

	ndFloat32 m_fps;
	ndFloat32 m_timestepAcc;
	ndFloat32 m_currentListenerTimestep;
	ndSpinLock m_addDeleteLock;
	
	bool m_showUI;
	bool m_showAABB;
	bool m_showStats;
	bool m_helperLegend;
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
	ndKeyTrigger m_nextActiveCamera;
	
	ndWorld::ndSolverModes m_solverMode;
	static ndDemos m_demosSelection[];
	static ndDemos m_machineLearning[];

	friend class ndPhysicsWorld;
};


#endif