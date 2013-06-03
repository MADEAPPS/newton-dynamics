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
#define __NEWTON_DEMOS_H__

class DemoMenu;
class DemoEntityManager;




class NewtonDemos: public wxFrame
{
	enum MY_WXDHEDTS_IDS
	{
		ID_CANVAS = wxID_HIGHEST,

		ID_AUTOSLEEP_MODE,
		ID_SHOW_STATISTICS,
		ID_USE_PARALLEL_SOLVER,

		ID_HIDE_VISUAL_MESHES,
		ID_SHOW_COLLISION_MESH,
		ID_SHOW_COLLISION_MESH_RANGE = ID_SHOW_COLLISION_MESH + 4,
		ID_SHOW_CONTACT_POINTS,
		ID_SHOW_NORMAL_FORCES,
		ID_SHOW_AABB,
		ID_SHOW_JOINTS,
		ID_SHOW_CENTER_OF_MASS,
		

		ID_SERIALIZE,
		ID_DESERIALIZE,

		ID_PLATFORMS,
		ID_PLATFORMS_MAX = ID_PLATFORMS + 8,

		ID_SELECT_ALL_PROFILERS,
		ID_UNSELECT_ALL_PROFILERS,

		ID_SHOW_PHYSICS_PROFILER,
		ID_SHOW_PHYSICS_PROFILER_COUNT = ID_SHOW_PHYSICS_PROFILER + 16,

		ID_CONCURRENT_PHYSICS_UPDATE,

		ID_SELECT_MICROTHREADS,
		ID_SELECT_MICROTHREADS_COUNT = ID_SELECT_MICROTHREADS + 16,

		ID_SHOW_PROFILER,
		ID_SHOW_CONCURRENCE_PROFILER,

		ID_RUN_DEMO,
		ID_RUN_DEMO_RANGE = ID_RUN_DEMO + 100,
	};


	public:
	NewtonDemos(const wxString& title, const wxPoint& pos, const wxSize& size);
	~NewtonDemos();

	typedef void (*LaunchSDKDemoCallback) (DemoEntityManager* scene);
	class SDKDemos
	{
		public:
		const char *m_name;
		const char *m_description;
		LaunchSDKDemoCallback m_launchDemoCallback;
	};
	
	wxMenuBar* CreateMainMenu();
	void LoadDemo (int index);

	void CalculateFPS(float timestep);

	void BEGIN_MENU_OPTION();
	void END_MENU_OPTION();
	void RestoreSettings ();

	void KeyPress(const wxKeyEvent &event);
	void KeyRelease(const wxKeyEvent &event);
	void MouseAction(const wxMouseEvent &event);

	bool GetKeyState (int key) const;
	bool GetMouseKeyState (int button ) const;
	bool GetMousePosition (int& posX, int& posY) const;
	bool GetJoytickPosition (dFloat& posX, dFloat& posY, int& buttonsMask) const;



	void OnQuit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);
	void OnRunDemo(wxCommandEvent& event);
	void OnAutoSleepMode(wxCommandEvent& event);	
	void OnHideVisualMeshes(wxCommandEvent& event);
	void OnShowCollisionLines(wxCommandEvent& event);
	void OnShowContactPoints(wxCommandEvent& event);
	void OnShowNormalForces(wxCommandEvent& event);
	void OnShowAABB(wxCommandEvent& event);
	void OnShowShowJoints(wxCommandEvent& event);
	void OnShowCenterOfMass(wxCommandEvent& event);
	void OnUseParallelSolver(wxCommandEvent& event);
	void OnShowConcurrentProfiler(wxCommandEvent& event);
	void OnShowThreadProfiler(wxCommandEvent& event);
	void OnShowProfiler(wxCommandEvent& event);
	void OnSelectAllPerformanceChart(wxCommandEvent& event);
	void OnUnselectAllPerformanceChart(wxCommandEvent& event);
	void OnRunPhysicsConcurrent(wxCommandEvent& event);
	void OnSelectNumberOfMicroThreads(wxCommandEvent& event);
	void OnSelectHardwareDevice(wxCommandEvent& event);
	void OnShowStatistics(wxCommandEvent& event);

	void OnNew(wxCommandEvent& event);
	void OnSerializeWorld(wxCommandEvent& event);
	void OnDeserializeWorld(wxCommandEvent& event);

//	long onLoad(FXObject* sender, FXSelector id, void* eventPtr); 
//	long onSave(FXObject* sender, FXSelector id, void* eventPtr); 



	wxMenuBar* m_mainMenu;
	wxStatusBar* m_statusbar;
	DemoEntityManager* m_scene;

	unsigned m_physicsUpdateMode;

	bool m_suspendVisualUpdates;
	bool m_autoSleepState;
	bool m_useParallelSolver;
	bool m_hideVisualMeshes;
	bool m_showContactPoints;
	bool m_showNormalForces;
	bool m_showAABB;
	bool m_showJoints;
	bool m_showCenterOfMass;
	bool m_showStatistics;
	bool m_concurrentProfilerState;
	bool m_threadProfilerState;
	bool m_hasJoysticController;

	int m_debugDisplayMode;
	int m_mousePosX;
	int m_mousePosY;
	int m_joytickX;
	int m_joytickY;
	int m_joytickButtonMask;
	int m_framesCount;
	int m_microthreadIndex;
	int m_hardwareDevice;
	
	float m_timestepAcc;
	float m_fps;


	int m_key[512];
	int m_keyMap[512];
	wxMenuItem* m_profilerTracksMenu[16];


	static int m_threadsTracks[];
	static SDKDemos m_demosSelection[];

	DECLARE_EVENT_TABLE()

	friend class DemoMenu;
	friend class NewtonDemosApp;
};



#endif