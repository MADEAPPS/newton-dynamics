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

// NewtonDemos.cpp : Defines the entry point for the application.
//


#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoCamera.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "DemoEntityManager.h"



//#define DEFAULT_SCENE	0			// using NetwonMesh Tool
//#define DEFAULT_SCENE	1			// Coefficients of friction
//#define DEFAULT_SCENE	2			// Coefficients of restitution
//#define DEFAULT_SCENE	3			// Precessing tops
//#define DEFAULT_SCENE	4			// closest distance
//#define DEFAULT_SCENE	5			// primitive collision
//#define DEFAULT_SCENE	6 			// Kinematic bodies
//#define DEFAULT_SCENE	7			// primitive convex cast 
//#define DEFAULT_SCENE	8			// Box stacks
//#define DEFAULT_SCENE	9			// simple level mesh collision
//#define DEFAULT_SCENE	10			// optimized level mesh collision
//#define DEFAULT_SCENE	11			// height field Collision
//#define DEFAULT_SCENE	12			// infinite user plane collision
//#define DEFAULT_SCENE	13			// user height field Collision
//#define DEFAULT_SCENE	14			// compound Collision
//#define DEFAULT_SCENE	15			// pjani compound bug
//#define DEFAULT_SCENE	16			// uniform Scaled Collision
//#define DEFAULT_SCENE	17			// non Uniform Scaled Collision
//#define DEFAULT_SCENE	18			// scaled mesh collision
//#define DEFAULT_SCENE	19			// simple convex decomposition
//#define DEFAULT_SCENE	20			// scene Collision
//#define DEFAULT_SCENE	21          // simple boolean operators 
//#define DEFAULT_SCENE	22			// simple convex Shatter
//#define DEFAULT_SCENE	23			// multi ray casting using the threading Job scheduler
//#define DEFAULT_SCENE	24			// continue collision
//#define DEFAULT_SCENE	25			// puck slide continue collision
//#define DEFAULT_SCENE	26			// basic car
#define DEFAULT_SCENE	27			// player controller
//#define DEFAULT_SCENE	28			// advanced player controller
//#define DEFAULT_SCENE	29			// cloth patch			
//#define DEFAULT_SCENE	30			// soft bodies			
//#define DEFAULT_SCENE	31			// high performance super car




void Friction (DemoEntityManager* const scene);
void Restitution (DemoEntityManager* const scene);
void PrecessingTops (DemoEntityManager* const scene);
void ClosestDistance (DemoEntityManager* const scene);
void ConvexCast (DemoEntityManager* const scene);
void PrimitiveCollision (DemoEntityManager* const scene);
void KinematicBodies (DemoEntityManager* const scene);
void ClothPath(DemoEntityManager* const scene);
void SoftBodies (DemoEntityManager* const scene);
void BasicBoxStacks (DemoEntityManager* const scene);
void SimpleMeshLevelCollision (DemoEntityManager* const scene);
void OptimizedMeshLevelCollision (DemoEntityManager* const scene);
void UniformScaledCollision (DemoEntityManager* const scene);
void NonUniformScaledCollision (DemoEntityManager* const scene);
void ScaledMeshCollision (DemoEntityManager* const scene);
void ContinueCollision (DemoEntityManager* const scene);
void PuckSlide (DemoEntityManager* const scene);
void SceneCollision (DemoEntityManager* const scene);
void CompoundCollision(DemoEntityManager* const scene);
void PostCompoundCreateBuildTest(DemoEntityManager* const scene);
void SimpleConvexApproximation(DemoEntityManager* const scene);
void SimpleBooleanOperations(DemoEntityManager* const scene);
void SimpleConvexShatter (DemoEntityManager* const scene);
void UsingNewtonMeshTool (DemoEntityManager* const scene);
void MultiRayCast (DemoEntityManager* const scene);
void BasicCar (DemoEntityManager* const scene);
void BasicPlayerController (DemoEntityManager* const scene);
void AdvancedPlayerController (DemoEntityManager* const scene);
void SuperCar (DemoEntityManager* const scene);
void HeightFieldCollision (DemoEntityManager* const scene);
void UserPlaneCollision (DemoEntityManager* const scene);
void UserHeightFieldCollision (DemoEntityManager* const scene);




NewtonDemos::SDKDemos NewtonDemos::m_demosSelection[] = 
{
	{"Using the newton mesh tool", "demonstrate how to use the newton mesh toll for mesh manipulation", UsingNewtonMeshTool},
	{"Coefficients of friction", "demonstrate the effect of various coefficient of friction", Friction},
	{"Coefficients of restitution", "demonstrate the effect of various coefficient of restitution", Restitution},
	{"Precessing tops", "show natural precession", PrecessingTops},
	{"Closest distance", "demonstrate closest distance to a convex shape", ClosestDistance},
	{"Primitive Collision", "demonstrate separate collision of primitives", PrimitiveCollision},
	{"Kinematic bodies", "demonstrate separate collision of primitives", KinematicBodies},
	{"Primitive convex cast", "demonstrate separate primitive convex cast", ConvexCast},
	{"Simple box Stacks", "show simple stack of Boxes", BasicBoxStacks},
	{"Unoptimized mesh collision", "show simple level mesh", SimpleMeshLevelCollision},
	{"Optimized mesh collision", "show optimized level mesh", OptimizedMeshLevelCollision},
	{"Height field collision mesh", "show high file collision mesh", HeightFieldCollision},
	{"User infinite Plane collision mesh", "show high file collision mesh", UserPlaneCollision},
	{"User Height field collision mesh", "show high file collision mesh", UserHeightFieldCollision},
	{"Compound collision shape", "demonstrate compound collision", CompoundCollision},
	{"PostCompoundCreateBuildTest", "PostCompoundCreateBuildTest", PostCompoundCreateBuildTest},
	{"Uniform scaled collision shape", "demonstrate scaling shape", UniformScaledCollision},
	{"Non uniform scaled collision shape", "demonstrate scaling shape", NonUniformScaledCollision},
	{"Scaled mesh collision", "demonstrate scaling mesh scaling collision", ScaledMeshCollision},
	{"Simple convex decomposition", "demonstrate convex decomposition and compound collision", SimpleConvexApproximation},
	{"Multi geometry collision", "show static mesh with the ability of moving internal parts", SceneCollision},
	{"Simple boolean operations", "demonstrate simple boolean operations ", SimpleBooleanOperations},
	{"Simple convex Shatter", "demonstrate fracture destruction using Voronoi partition", SimpleConvexShatter},
	{"Parallel ray cast", "using the threading Job scheduler", MultiRayCast},
	{"Continue collision", "show continue collision", ContinueCollision},
	{"Puck slide", "show continue collision", PuckSlide},
	{"Basic Car", "implement a basic car", BasicCar},
	{"Basic player controller", "demonstrate simple player controller", BasicPlayerController},
	{"Advanced player controller", "demonstrate player interacting with other objects", AdvancedPlayerController},
//	{"High performance super car", "implement a high performance ray cast car", SuperCar},
	{"Simple cloth Path", "show simple cloth path", ClothPath},
	{"Simple soft Body", "show simple soft body", SoftBodies},


//	{"basic convex hull stacking", "demonstrate convex hull stacking", BasicConvexStacks},
//	{"basic unstable stacking", "demonstrate stability stacking unstable objects", UnstableStacks},
//	{"Jenga stacking", "demonstrate Jenga game", Jenga},
//	{"Large Jenga stacking", "demonstrate Jenga game", JengaTall},
//	{"small pyramid stacking", "demonstrate small pyramid stacking", CreatePyramid},
//	{"wall stacking", "demonstrate wall stacking", CreateWalls},
//	{"small tower stacking", "demonstrate tower stacking", CreateTower},
//	{"large tower stacking", "demonstrate tower stacking", CreateTowerTall},
//	{"user defined polygon static collision", "demonstrate user defined polygon static collision", UserHeighMapColliion},
//	{"attractive magnets force field", "demonstrate attractive force field", Magnets},
//	{"repulsive magnets force field", "demonstrate repulsive magnet force field", Repulsive},
//	{"Archimedes buoyancy force field", "demonstrate user define Archimedes as force field", ArchimedesBuoyancy},
//	{"legacy joints", "demonstrate the build in joints", LegacyJoints},
//	{"custom joints", "demonstrate custom joints", BasicCustomJoints},
//	{"Simple robots", "demonstrate custom joints robot", BasicRobots},
//	{"motorized robots", "demonstrate motorized custom joints robot", TracktionJoints},
//	{"discrete rag doll", "demonstrate simple rag doll", DescreteRagDoll},
//	{"skinned rag doll", "demonstrate simple rag doll", SkinRagDoll},
};


int NewtonDemos::m_threadsTracks[] = {1, 2, 3, 4, 8, 12, 16};



class NewtonDemosApp: public wxApp
{
	virtual bool OnInit()
	{
		// check for memory leaks
		#if defined(_DEBUG) && defined(_MSC_VER)
			// Track all memory leaks at the operating system level.
			// make sure no Newton tool or utility leaves leaks behind.
			_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));
		#endif

		// Set the memory allocation function before creation the newton world
		// this is the only function that can be called before the creation of the newton world.
		// it should be called once, and the the call is optional 
		NewtonSetMemorySystem (PhysicsAlloc, PhysicsFree);

		int version = NewtonWorldGetVersion();
		char tittle[256]; 
		sprintf (tittle, "Newton %d.%02d SDK demos", version / 100, version % 100);
		NewtonDemos* const frame = new NewtonDemos(tittle, wxDefaultPosition, wxSize(1024, 768));

		frame->Show(true);
		SetTopWindow(frame);

		// initialize open gl graphics
		if (frame->m_scene) {
			frame->m_scene->InitGraphicsSystem();
		}

		// load the default Scene		
		//frame->LoadDemo (DEFAULT_SCENE);
		wxMenuEvent loadDemo (wxEVT_COMMAND_MENU_SELECTED, NewtonDemos::ID_RUN_DEMO + DEFAULT_SCENE);
		frame->GetEventHandler()->ProcessEvent(loadDemo);

//		wxMenuEvent autoSleep (wxEVT_COMMAND_MENU_SELECTED, NewtonDemos::ID_AUTOSLEEP_MODE);
//		frame->GetEventHandler()->ProcessEvent(autoSleep);
		return true;
	}

	// memory allocation for Newton
	static void* PhysicsAlloc (int sizeInBytes)
	{
		m_totalMemoryUsed += sizeInBytes;
		return new char[sizeInBytes];
	}

	// memory free use by the engine
	static void PhysicsFree (void* ptr, int sizeInBytes)
	{
		m_totalMemoryUsed -= sizeInBytes;
		delete[] (char*)ptr;
	}

	static int m_totalMemoryUsed;
};

int NewtonDemosApp::m_totalMemoryUsed = 0;


IMPLEMENT_APP(NewtonDemosApp)

BEGIN_EVENT_TABLE(NewtonDemos, wxFrame)
	// mandatory menu events for mac cocoa osx  support

	EVT_MENU(wxID_ABOUT, NewtonDemos::OnAbout)
	EVT_MENU(wxID_EXIT, NewtonDemos::OnQuit)
	EVT_MENU(wxID_HELP, NewtonDemos::OnAbout)
	EVT_MENU(wxID_PREFERENCES, NewtonDemos::OnAbout)

	// game menus events
	EVT_MENU_RANGE(ID_RUN_DEMO, ID_RUN_DEMO_RANGE, NewtonDemos::OnRunDemo)

	EVT_MENU(ID_AUTOSLEEP_MODE,	NewtonDemos::OnAutoSleepMode)
	EVT_MENU(ID_HIDE_VISUAL_MESHES,	NewtonDemos::OnHideVisualMeshes)
	EVT_MENU(ID_SHOW_COLLISION_MESH, NewtonDemos::OnShowCollisionLines)
	EVT_MENU(ID_SHOW_CONTACT_POINTS, NewtonDemos::OnShowContactPoints)
	EVT_MENU(ID_SHOW_NORMAL_FORCES,	NewtonDemos::OnShowNormalForces)
	EVT_MENU(ID_SHOW_AABB, NewtonDemos::OnShowAABB)
	EVT_MENU(ID_USE_PARALLEL_SOLVER, NewtonDemos::OnUseParallelSolver)

	EVT_MENU_RANGE(ID_DYNAMICS_BROADPHASE, ID_HYBRID_BROADPHASE, NewtonDemos::OnBroadPhaseType)
	EVT_MENU_RANGE(ID_PLATFORMS, ID_PLATFORMS_MAX, NewtonDemos::OnSimdInstructions)

	EVT_MENU(ID_SHOW_CONCURRENCE_PROFILER, NewtonDemos::OnShowConcurrentProfiler)
	EVT_MENU(ID_SHOW_PROFILER,	NewtonDemos::OnShowThreadProfiler)

	EVT_MENU(ID_SELECT_ALL_PROFILERS, NewtonDemos::OnSelectAllPerformanceChart)
	EVT_MENU(ID_UNSELECT_ALL_PROFILERS,	NewtonDemos::OnUnselectAllPerformanceChart)
	EVT_MENU_RANGE (ID_SHOW_PHYSICS_PROFILER, ID_SHOW_PHYSICS_PROFILER_COUNT, NewtonDemos::OnShowProfiler)

	EVT_MENU(ID_CONCURRENT_PHYSICS_UPDATE, NewtonDemos::OnRunPhysicsConcurrent)
	EVT_MENU_RANGE(ID_SELECT_MICROTHREADS, ID_SELECT_MICROTHREADS_COUNT, NewtonDemos::OnSelectNumberOfMicroThreads)

//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_NEW,								NewtonDemos::onNew),
//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_LOAD,								NewtonDemos::onLoad),
//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SAVE,								NewtonDemos::onSave),
//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SERIALIZE,							NewtonDemos::onSerializeWorld),
//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_DESERIALIZE,						NewtonDemos::onDeserializeWorld),


END_EVENT_TABLE()


NewtonDemos::NewtonDemos(const wxString& title, const wxPoint& pos, const wxSize& size)
	:wxFrame(NULL, -1, title, pos, size)
	,m_mainMenu(NULL)
	,m_statusbar(NULL)
	,m_scene(NULL)
	,m_physicsUpdateMode(0)
	,m_suspendVisualUpdates(true)
	,m_autoSleepState(true)
	,m_useParallelSolver(false)
	,m_hideVisualMeshes(false)
	,m_showContactPoints(false)
	,m_showNormalForces(false)
//	,m_showNormalForces(true)
	,m_showAABB(false)
	,m_debugDisplayState(false)
//	,m_debugDisplayState(true)
	,m_concurrentProfilerState(false)
	,m_threadProfilerState(false)
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_joytickX(0)
	,m_joytickY(0)
	,m_joytickButtonMask(0)
	,m_hasJoysticController(false)
	,m_framesCount(0)
	,m_timestepAcc(0)
	,m_fps(0.0f)
	,m_microthreadIndex(0)
	,m_cpuInstructionsMode(0)
	,m_broadPhaseMode (0)
{
	memset (m_profilerTracksMenu, 0, sizeof (m_profilerTracksMenu));

	// clear the key map
	memset (m_key, 0, sizeof (m_key));
	for (int i = 0; i < int (sizeof (m_keyMap)/sizeof (m_keyMap[0])); i ++) {
		m_keyMap[i] = i;
	}
	for (int i = 'a'; i <= 'z'; i ++) {
		m_keyMap[i] = i - 'a' + 'A';
	}

#ifdef WIN32
	m_keyMap[0] = VK_LBUTTON;
	m_keyMap[1] = VK_RBUTTON;
	m_keyMap[2] = VK_MBUTTON; 
#endif

	m_scene = new DemoEntityManager(this);
	m_statusbar = CreateStatusBar();
	int widths[] = {150, 160, 150, 90, 80, 100, 100};
	m_statusbar->SetFieldsCount (sizeof (widths)/sizeof (widths[0]), widths);
	CalculateFPS(0.0f);
	m_mainMenu = CreateMainMenu();
}


NewtonDemos::~NewtonDemos()
{
}


wxMenuBar* NewtonDemos::CreateMainMenu()
{
	wxMenuBar* const mainMenu =  new wxMenuBar();

	// adding the file menu
	{
		wxMenu* const fileMenu = new wxMenu;

		fileMenu->Append(wxID_ABOUT, _("About"));
		
		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_PREFERENCES, _("Preferences"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_NEW, _("&New"), _("Create a blank new scene"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_OPEN, _("&Open"), _("Open visual scene in dScene newton format"));
		fileMenu->Append(wxID_SAVE, _("&Save"), _("Save visual scene in dScene newton format"));

	//	fileMenu->AppendSeparator();
	//	fileMenu->Append(m_idImportPhysics, _T("&Open physics scene"), _T("Open physics scene in collada format"));
	//	fileMenu->Append(m_idExportPhysics, _T("&Save physics scene"), _T("Save physics in collada format"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_EXIT, _("E&xit\tAlt-X"), _("Quit SDK sample") );

		// add main menus to menu bar
		mainMenu->Append(fileMenu, _("&File"));
	}

	// engine all demo examples
	{
		wxMenu* const sdkDemos = new wxMenu;
		int demosCount = int (sizeof (m_demosSelection) / sizeof m_demosSelection[0]);
		for (int i = 0; i < demosCount; i ++) {
			sdkDemos->AppendRadioItem (NewtonDemos::ID_RUN_DEMO + i,  m_demosSelection[i].m_name, m_demosSelection[i].m_description);
		}

		mainMenu->Append(sdkDemos, _("&Demos"));
	}

	// option menu
	{
		wxMenu* const optionsMenu = new wxMenu;;

		optionsMenu->AppendCheckItem(ID_AUTOSLEEP_MODE, _("Auto sleep mode"), _("toogle auto sleep bodies"));
		optionsMenu->AppendCheckItem(ID_HIDE_VISUAL_MESHES, _("Hide visual meshes"));
		optionsMenu->AppendCheckItem(ID_SHOW_COLLISION_MESH, _("Show collision Mesh"));
		optionsMenu->AppendCheckItem(ID_SHOW_CONTACT_POINTS, _("Show contact points"));
		optionsMenu->AppendCheckItem(ID_SHOW_NORMAL_FORCES, _("Show normal forces"));
		optionsMenu->AppendCheckItem(ID_SHOW_AABB, _("Show aabb"));
		optionsMenu->AppendCheckItem(ID_USE_PARALLEL_SOLVER, _("Parallel solver on"));

		optionsMenu->Check (ID_AUTOSLEEP_MODE, m_autoSleepState);

		optionsMenu->AppendSeparator();
		int platformsCount = NewtonEnumrateDevices (m_scene->GetNewton());
		for (int i = 0; i < platformsCount; i ++) {
			char platform[256];
			NewtonGetDeviceString (m_scene->GetNewton(), i, platform, sizeof (platform));
			optionsMenu->AppendRadioItem(ID_PLATFORMS + i, _(platform));
		}
		//optionsMenu->Check(ID_PLATFORMS, true);

		optionsMenu->AppendSeparator();
		optionsMenu->AppendRadioItem(ID_DYNAMICS_BROADPHASE, _("dynamics broad phase"));
		optionsMenu->AppendRadioItem(ID_STATIC_BROADPHASE, _("static broad phase"));
		optionsMenu->AppendRadioItem(ID_HYBRID_BROADPHASE, _("hybrid broad phase"));

		optionsMenu->AppendSeparator();
		optionsMenu->Append(ID_SHOW_CONCURRENCE_PROFILER, _("Show concurrent profiler"));
		optionsMenu->Append(ID_SHOW_PROFILER, _("Show micro thread profiler"));

		optionsMenu->AppendSeparator();
		optionsMenu->Append(ID_SELECT_ALL_PROFILERS, _("select all profiler"));
		optionsMenu->Append(ID_UNSELECT_ALL_PROFILERS, _("unselect all profiler"));

		wxMenu* const profilerSubMenu = new wxMenu;
		m_profilerTracksMenu[0] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 0, _("show global physics update performance chart"));
		m_profilerTracksMenu[1] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 1, _("global collision update performance chart"));
		m_profilerTracksMenu[2] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 2, _("broad phase collision performance chart"));
		m_profilerTracksMenu[3] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 3, _("narrow phase collision performance chart"));
		m_profilerTracksMenu[4] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 4, _("global dynamics update performance chart"));
		m_profilerTracksMenu[5] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 5, _("dynamics setup performance chart"));
		m_profilerTracksMenu[6] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 6, _("dynamics solver performance chart"));
		m_profilerTracksMenu[7] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 7, _("force and torque callback performance chart"));
		m_profilerTracksMenu[8] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 8, _("pre-simulation listener"));
		m_profilerTracksMenu[9] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 9, _("post-simulation listener"));
//		if (mainFrame->m_physicProfilerState) {
//			m_profilerTracksMenu[0]->setCheck(true);
//		}
		optionsMenu->AppendSubMenu (profilerSubMenu, _("select sub profiler"));


		optionsMenu->AppendSeparator();
		optionsMenu->AppendCheckItem(ID_CONCURRENT_PHYSICS_UPDATE, _("Concurrent physics update"));

		wxMenu* const microThreadedsSubMenu = new wxMenu;
		for (int i = 0 ; i < int (sizeof (m_threadsTracks)/ sizeof (m_threadsTracks[0])); i ++) {
			wxString msg;
			msg.Printf(wxT ("%d micro threads"), m_threadsTracks[i]);
			microThreadedsSubMenu->AppendRadioItem(ID_SELECT_MICROTHREADS + i, msg);
		}
		optionsMenu->AppendSubMenu (microThreadedsSubMenu, _("select microThread count"));


		mainMenu->Append(optionsMenu, _("&Options"));
	}

	// add help menu
	{
		wxMenu* const helpMenu = new wxMenu;;

		helpMenu->Append(wxID_HELP, _("About"));
//		helpMenu->Append(NewtonDemos::ID_ON_ABOUT, _T("About"));
		mainMenu->Append(helpMenu, _("&Help"));
	}

	SetMenuBar(mainMenu);
	return mainMenu;
}


void NewtonDemos::BEGIN_MENU_OPTION()																				
{
	m_suspendVisualUpdates = true;																			
	if (m_scene && m_scene->GetNewton()) {																			
		NewtonWaitForUpdateToFinish (m_scene->GetNewton());												
	}
}


void NewtonDemos::END_MENU_OPTION()
{
	m_suspendVisualUpdates = false;																			
	if (m_scene && m_scene->GetNewton()) {		
		NewtonWaitForUpdateToFinish (m_scene->GetNewton());
		SetAutoSleepMode (m_scene->GetNewton(), !m_autoSleepState);
		NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);	
	}
}


void NewtonDemos::RestoreSettings ()
{
	NewtonSetCurrentDevice (m_scene->GetNewton(), m_cpuInstructionsMode); 
	NewtonSetThreadsCount(m_scene->GetNewton(), m_threadsTracks[m_microthreadIndex]); 
	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), m_broadPhaseMode);
}


void NewtonDemos::LoadDemo (int index)
{
	BEGIN_MENU_OPTION();

	m_scene->Cleanup();

	m_scene->SetCurrent();
	m_demosSelection[index].m_launchDemoCallback (m_scene);
	m_scene->SwapBuffers(); 

	RestoreSettings ();
	m_scene->ResetTimer();

	END_MENU_OPTION();
}



void NewtonDemos::KeyPress(const wxKeyEvent &event)
{
	int keyCode = event.GetKeyCode();
	if (keyCode == WXK_ESCAPE)  {
		// send a display refresh event in case the runtime update is stopped bu the user.
		wxMenuEvent exitEvent (wxEVT_COMMAND_MENU_SELECTED, wxID_EXIT);
		GetEventHandler()->ProcessEvent(exitEvent);
	}


	if (!event.GetModifiers()) {
		int code = keyCode & 0xff; 
		m_key[m_keyMap[code]] = true;
	}
}


void NewtonDemos::KeyRelease(const wxKeyEvent &event)
{
	if (!event.GetModifiers()) {
		int keyCode = event.GetKeyCode();
		int code = keyCode & 0xff;
		m_key[m_keyMap[code]] = false;
	}
}


void NewtonDemos::MouseAction(const wxMouseEvent &event)
{
	m_mousePosX = event.GetX();
	m_mousePosY = event.GetY();

	if (event.LeftIsDown()) {
		m_key[m_keyMap[0]] = true;
	} else {
		m_key[m_keyMap[0]] = false;
	}

	if (event.RightIsDown()) {
		m_key[m_keyMap[1]] = true;
	} else {
		m_key[m_keyMap[1]] = false;
	}
}

bool NewtonDemos::GetKeyState( int key) const
{
	return m_key[m_keyMap[key & 0xff]] ? true : false;
}

bool NewtonDemos::GetMousePosition (int& posX, int& posY) const
{
	posX = m_mousePosX;
	posY = m_mousePosY;
	return true;
}

bool NewtonDemos::GetJoytickPosition (dFloat& posX, dFloat& posY, int& buttonsMask) const
{
	buttonsMask = m_joytickButtonMask;
	posX = dFloat (m_joytickX - 32767) / 32768.0f;
	posY = -dFloat (m_joytickY - 32767) / 32768.0f;
	return m_hasJoysticController;
}


bool NewtonDemos::GetMouseKeyState (int button) const
{
	if ((button >= 0) && (button <= 2)) {
		return m_key[m_keyMap[button]] ? true : false;
	}

	return false;
}

void NewtonDemos::CalculateFPS(float timestep)
{
	m_framesCount ++;
	m_timestepAcc += timestep;

	// this probably happing on loading of and a pause, just rest counters
	if ((m_timestepAcc <= 0.0f) || (m_timestepAcc > 2.0f)){
		m_timestepAcc = 0;
		m_framesCount = 0;
	}

	//update fps every quarter of a second
	if (m_timestepAcc >= 0.25f) {
		m_fps = float (m_framesCount) / m_timestepAcc;
		m_timestepAcc -= 0.25f;
		m_framesCount = 0.0f;

		char statusText [512] ;
		NewtonWorld* const world = m_scene->GetNewton();
		char platform[256];
		NewtonGetDeviceString(world, NewtonGetCurrentDevice(world), platform, sizeof (platform));
		int memoryUsed = NewtonGetMemoryUsed() / (1024) ;
		
		sprintf (statusText, "render fps: %7.2f", m_fps);
		m_statusbar->SetStatusText (statusText, 0);

		sprintf (statusText, "physics time: %4.2f ms", m_scene->GetPhysicsTime() * 1000.0f);
		m_statusbar->SetStatusText (statusText, 1);

		sprintf (statusText, "memory: %d kbytes", memoryUsed);
		m_statusbar->SetStatusText (statusText, 2);

		sprintf (statusText, "bodies: %d", NewtonWorldGetBodyCount(world));
		m_statusbar->SetStatusText (statusText, 3);

		sprintf (statusText, "threads: %d", NewtonGetThreadsCount(world));
		m_statusbar->SetStatusText (statusText, 4);

		sprintf (statusText, "auto sleep: %s", m_autoSleepState ? "on" : "off");
		m_statusbar->SetStatusText (statusText, 5);

		char floatMode[128];
		NewtonGetDeviceString (m_scene->GetNewton(), m_cpuInstructionsMode, floatMode, sizeof (floatMode));
		sprintf (statusText, "instructions: %s", floatMode);
		m_statusbar->SetStatusText (statusText, 6);

	}
}




void NewtonDemos::OnAbout(wxCommandEvent& event)
{
	wxString msg;
	int version = NewtonWorldGetVersion();
	msg.Printf(wxT ("Hello to Newton Dynamics SDK %d.%02d"), version / 100, version % 100);
	wxMessageBox(msg, wxT ("Newton Dynanics"), wxOK | wxICON_INFORMATION, this);
}


void NewtonDemos::OnQuit(wxCommandEvent& event)
{
	Close ();
}

void NewtonDemos::OnRunDemo(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();

	wxMenu* const menu = m_mainMenu->GetMenu(m_mainMenu->FindMenu(_ ("Demos")));
	if (!menu->IsChecked(event.GetId())) {
		menu->Check (event.GetId(), true);
	}

	LoadDemo (event.GetId() - ID_RUN_DEMO);

	RestoreSettings ();
	END_MENU_OPTION();
}


void NewtonDemos::OnAutoSleepMode(wxCommandEvent& event)	
{
	BEGIN_MENU_OPTION();
	m_autoSleepState = event.IsChecked();
	END_MENU_OPTION();
}


void NewtonDemos::OnHideVisualMeshes(wxCommandEvent& event)	
{
	BEGIN_MENU_OPTION();
	m_hideVisualMeshes = event.IsChecked();
	END_MENU_OPTION();
}


void NewtonDemos::OnShowCollisionLines(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();

	m_debugDisplayState = event.IsChecked(); 
	SetDebugDisplayMode (m_debugDisplayState);

	END_MENU_OPTION();
}


void NewtonDemos::OnShowContactPoints(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_showContactPoints = event.IsChecked(); 
	END_MENU_OPTION();
}

void NewtonDemos::OnShowNormalForces(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_showNormalForces = event.IsChecked(); 
	END_MENU_OPTION();
}

void NewtonDemos::OnShowAABB(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_showAABB = event.IsChecked(); 
	END_MENU_OPTION();
}


void NewtonDemos::OnUseParallelSolver(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_useParallelSolver = event.IsChecked(); 
	NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);
	END_MENU_OPTION();
}


void NewtonDemos::OnBroadPhaseType(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_broadPhaseMode = event.GetId() - ID_DYNAMICS_BROADPHASE;
	dAssert (m_broadPhaseMode >= 0);
	dAssert (m_broadPhaseMode <= 3);
	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), m_broadPhaseMode);

	END_MENU_OPTION();
}

void NewtonDemos::OnShowConcurrentProfiler(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_concurrentProfilerState = event.IsChecked(); 
	END_MENU_OPTION();
}


void NewtonDemos::OnShowThreadProfiler(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_threadProfilerState = event.IsChecked(); 
	END_MENU_OPTION();
}


void NewtonDemos::OnShowProfiler(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	int track = event.GetId() - ID_SHOW_PHYSICS_PROFILER;
	int state = event.IsChecked(); 
	m_scene->m_showProfiler[track] = state;
	END_MENU_OPTION();
}


void NewtonDemos::OnSelectAllPerformanceChart(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	int count = sizeof (m_profilerTracksMenu) / sizeof (m_profilerTracksMenu[0]);
	for (int i = 0; i < count; i ++) {
		if (m_profilerTracksMenu[i]) {
			m_scene->m_showProfiler[i] = 1;
			m_profilerTracksMenu[i]->Check(true);
		}
	}
	END_MENU_OPTION();
}

void NewtonDemos::OnUnselectAllPerformanceChart(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	int count = sizeof (m_profilerTracksMenu) / sizeof (m_profilerTracksMenu[0]);
	for (int i = 0; i < count; i ++) {
		if (m_profilerTracksMenu[i]) {
			m_scene->m_showProfiler[i] = 0;
			m_profilerTracksMenu[i]->Check(false);
		}
	}
	END_MENU_OPTION();
}


void NewtonDemos::OnRunPhysicsConcurrent(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_physicsUpdateMode = event.IsChecked(); 
	END_MENU_OPTION();
}

void NewtonDemos::OnSelectNumberOfMicroThreads(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_microthreadIndex = event.GetId() - ID_SELECT_MICROTHREADS;
	NewtonSetThreadsCount(m_scene->GetNewton(), m_threadsTracks[m_microthreadIndex]);
	END_MENU_OPTION();
}

void NewtonDemos::OnSimdInstructions(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_cpuInstructionsMode = event.GetId() - ID_PLATFORMS;
	NewtonSetCurrentDevice (m_scene->GetNewton(), m_cpuInstructionsMode);
	END_MENU_OPTION();
}


#if 0

long NewtonDemos::onNew(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	m_scene->Cleanup();
	RestoreSettings ();
	m_scene->ResetTimer();
	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onLoad(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	const FXchar patterns[]="Newton Dynamics Files (*.ngd)";
	FXFileDialog open(this,"Load Newton Dynamics scene");
	open.setPatternList(patterns);
	open.setDirectory ("../../../media");
	if(open.execute()){

		m_scene->Cleanup();

		// load the scene from a ngd file format
		m_scene->makeCurrent();
		m_scene->LoadScene (open.getFilename().text());
		m_scene->makeNonCurrent();

		// add a sky box to the scene, make the first object
		m_scene->Addtop (new SkyBox());

		// place camera into position
		dMatrix camMatrix (GetIdentityMatrix());
		//		camMatrix.m_posit = dVector (-40.0f, 10.0f, 0.0f, 0.0f);
		camMatrix = dYawMatrix(-0.0f * 3.1416f / 180.0f);
		camMatrix.m_posit = dVector (-5.0f, 1.0f, -0.0f, 0.0f);
		m_scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);

		RestoreSettings ();
	}


	m_scene->ResetTimer();
	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onSave(FXObject* sender, FXSelector id, void* eventPtr)
{
	return 1;
}


long NewtonDemos::onSerializeWorld(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	const FXchar patterns[]="Newton Dynamics Files (*.bin)";
	FXFileDialog open(this, "Export a Newton Dynamics Serialized Physics Scene");
	open.setPatternList(patterns);
	open.setDirectory ("../../../media");
	if(open.execute()){
		m_scene->SerializedPhysicScene (open.getFilename().text());
	}

	m_scene->ResetTimer();
	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onDeserializeWorld(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	const FXchar patterns[]="Newton Dynamics Files (*.bin)";
	FXFileDialog open(this, "Import a Newton Dynamics Serialized Physics Scene");
	open.setPatternList(patterns);
	open.setDirectory ("../../../media");
	if(open.execute()){
		m_scene->makeCurrent();
		m_scene->DeserializedPhysicScene (open.getFilename().text());
		m_scene->makeNonCurrent();
		RestoreSettings ();
	}

	m_scene->ResetTimer();
	END_MENU_OPTION();
	return 1;
}

#endif