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

#ifndef __NEWTON_DEMOS_H__

class DemoMenu;
class DemoEntityManager;



class NewtonDemos: public FXMainWindow
{
	FXDECLARE(NewtonDemos)

	public:
	enum {
		ID_CANVAS = FXMainWindow::ID_LAST,
		ID_CHORE,
		ID_NEW,
		ID_LOAD,
		ID_SAVE,
		ID_SERIALIZE,
		ID_DESERIALIZE,
		ID_ABOUT,
		ID_AUTOSLEEP,
		ID_HIDE_VISUAL_MESHES,
		ID_SHOW_COLLISION_MESH,
		ID_SHOW_CONTACT_POINTS,
		ID_SHOW_AABB,
		ID_AUTOSLEEP_MODE,
		ID_SHOW_PHYSICS_PROFILER,
		ID_SHOW_PROFILER = ID_SHOW_PHYSICS_PROFILER + 16,
		ID_SELECT_ALL_PROFILERS,
		ID_UNSELECT_ALL_PROFILERS,
		ID_SHOW_CONCURRENCE_PROFILER,
		ID_USE_X87_INSTRUCTIONS,
		ID_USE_SIMD_INSTRUCTIONS,
		ID_USE_AVX_INSTRUCTIONS,

		ID_DYNAMICS_BROADPHASE,
		ID_STATIC_BROADPHASE,
		ID_HYBRID_BROADPHASE,

		ID_SELECT_MICROTHREADS,
		ID_CONCURRENT_PHYSICS_UPDATE = ID_SELECT_MICROTHREADS + 16,

		ID_USE_PARALLEL_SOLVER,
		ID_SET_CAMERA_SPEED,
		ID_RUN_DEMO,
		ID_RUN_DEMO_RANGE = ID_RUN_DEMO + 100,
	};

	NewtonDemos();
	NewtonDemos(FXApp& application);
	~NewtonDemos();

	// GUI interface functions
	void create();
	long onPaint(FXObject* sender, FXSelector id, void* eventPtr);
	long onChore(FXObject* sender, FXSelector id, void* eventPtr);
	long onNew(FXObject* sender, FXSelector id, void* eventPtr); 
	long onLoad(FXObject* sender, FXSelector id, void* eventPtr); 
	long onSave(FXObject* sender, FXSelector id, void* eventPtr); 
	long onSerializeWorld(FXObject* sender, FXSelector id, void* eventPtr); 
	long onDeserializeWorld(FXObject* sender, FXSelector id, void* eventPtr); 

	long onRunDemo(FXObject* sender, FXSelector id, void* eventPtr); 
	
	long onKeyPress(FXObject* sender, FXSelector id, void* eventPtr); 
	long onKeyRelease(FXObject* sender, FXSelector id, void* eventPtr); 
	long onMouseMove(FXObject* sender, FXSelector id, void* eventPtr); 
	long onHideVisualMeshes(FXObject* sender, FXSelector id, void* eventPtr);
	long onShowCollisionLines(FXObject* sender, FXSelector id, void* eventPtr);
	long onShowAABB(FXObject* sender, FXSelector id, void* eventPtr);
	long onAutoSleepMode(FXObject* sender, FXSelector id, void* eventPtr);
	long onShowContactPoints(FXObject* sender, FXSelector id, void* eventPtr);
	long onSimdInstructions(FXObject* sender, FXSelector id, void* eventPtr);
	long onBroadPhaseType(FXObject* sender, FXSelector id, void* eventPtr);
	long onShowThreadProfiler(FXObject* sender, FXSelector id, void* eventPtr);
	long onShowConcurrentProfiler(FXObject* sender, FXSelector id, void* eventPtr);
	long onSelectNumberOfMicroThreads(FXObject* sender, FXSelector id, void* eventPtr);
	long onUseParallelSolver(FXObject* sender, FXSelector id, void* eventPtr);
	long onRunPhysicsConcurrent(FXObject* sender, FXSelector id, void* eventPtr);

	long onShowProfiler(FXObject* sender, FXSelector id, void* eventPtr);
	long onSelectAllPerformanceChart(FXObject* sender, FXSelector id, void* eventPtr);
	long onUnselectAllPerformanceChart(FXObject* sender, FXSelector id, void* eventPtr);
	

	// engine interface functions
	void CalculateFPS(float timestep);
	void LoadDemo (int index);

	bool GetKeyState (char key) const;
	bool GetMouseKeyState (int button ) const;
	void GetMousePosition (int& posX, int& posY) const;

	static void* PhysicsAlloc (int sizeInBytes);
	static void PhysicsFree (void *ptr, int sizeInBytes);


	void BEGIN_MENU_OPTION();
	void END_MENU_OPTION();

	private:
	void ReadKeyboardAsync ();
	void RestoreSettings ();

	static int m_totalMemoryUsed;

	bool m_autoSleepState;
	bool m_useParallelSolver;
	bool m_hideVisualMeshes;
	bool m_debugDisplayState;
	bool m_showAABB;
	bool m_showContactPoints;
	bool m_physicProfilerState;
	bool m_threadProfilerState;
	bool m_concurrentProfilerState;
	bool m_showContactPointState;
	bool m_showStatistics;
	bool m_doVisualUpdates;

	int m_mousePosX;
	int m_mousePosY;


	DemoMenu* m_menubar;
	FXStatusBar* m_statusbar;
	DemoEntityManager* m_scene;


	int m_framesCount;
	float m_timestepAcc;
	float m_fps;
	
	char m_key[128];
	char m_keyMap[128];

	FXuint m_broadPhaseMode;
	FXDataTarget m_broadPhaseSelection;

	FXuint m_cpuInstructionsMode;
	FXDataTarget m_cpuInstructionSelection;

	FXuint m_microthreadCount;
	FXDataTarget m_microthreadCountSelection;

	FXuint m_physicsUpdateMode;
//	FXuint m_physicsUpdateModeVal;
//	FXDataTarget m_physicsUpdateModeSelection;

	DemoMenu* m_mainMenu;
	
	friend class DemoMenu;
	friend class dRuntimeProfiler;
	friend class DemoEntityManager;
};



#endif