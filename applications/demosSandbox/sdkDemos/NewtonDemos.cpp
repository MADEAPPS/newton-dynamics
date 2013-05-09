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
#define DEFAULT_SCENE	1			// Coefficients of friction
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
//#define DEFAULT_SCENE	27			// player controller
//#define DEFAULT_SCENE	28			// advanced player controller
//#define DEFAULT_SCENE	29			// high performance super car
//#define DEFAULT_SCENE	30			// soft bodies			

#if 0


NewtonDemos::NewtonDemos(FXApp& application)
	:FXMainWindow(&application, "Newton Dynamics 3.00 unit test demos", NULL, NULL, DECOR_ALL, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT)
	,m_showContactPoints(false)
	,m_physicProfilerState(true)
//	,m_showContactPointState(false)
	,m_showStatistics(true)
	,m_broadPhaseMode (ID_DYNAMICS_BROADPHASE)
	,m_broadPhaseSelection(m_broadPhaseMode)
	,m_cpuInstructionsMode(ID_PLATFORMS)
//	,m_cpuInstructionsMode(ID_USE_AVX_INSTRUCTIONS)
	,m_cpuInstructionSelection (m_cpuInstructionsMode)
	,m_microthreadCount(ID_SELECT_MICROTHREADS)
	,m_microthreadCountSelection(m_microthreadCount)
{
	m_broadPhaseSelection.setTarget(this);
	m_broadPhaseSelection.setSelector(ID_DYNAMICS_BROADPHASE);

	m_cpuInstructionSelection.setTarget(this);
	m_cpuInstructionSelection.setSelector(ID_PLATFORMS);

	m_microthreadCountSelection.setTarget(this);
	m_microthreadCountSelection.setSelector(ID_SELECT_MICROTHREADS);


	// create status bar for showing results 
	m_statusbar = new FXStatusBar(this, LAYOUT_SIDE_BOTTOM|LAYOUT_FILL_X|STATUSBAR_WITH_DRAGCORNER);
	// init the fps status bar
	CalculateFPS(0.0f);

	// create the main menu
	FXDockSite* const dockMenuFrame = new FXDockSite(this,DOCKSITE_NO_WRAP|LAYOUT_SIDE_TOP|LAYOUT_FILL_X);


	// Set the initial states
	onShowCollisionLines(this, 0, (void*) m_debugDisplayState);

	// create the open gl canvas and scene manager
	FXHorizontalFrame* const glFrame = new FXHorizontalFrame(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|LAYOUT_FILL_Y, 0,0,0,0, 0,0,0,0, 4,4);

	//create a main frame to hold the Render canvas
	m_scene = new DemoEntityManager (glFrame, this);

	// create main menu
	m_mainMenu = new DemoMenu (dockMenuFrame, this);

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


	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), m_broadPhaseMode - ID_DYNAMICS_BROADPHASE);
	m_broadPhaseMode = m_broadPhaseMode - FXDataTarget::ID_OPTION;

	//m_mainMenu->m_cpuModes[2]->disable();
	NewtonSetCurrentDevice (m_scene->GetNewton(), m_cpuInstructionsMode - ID_PLATFORMS); 
	m_cpuInstructionsMode = m_cpuInstructionsMode - FXDataTarget::ID_OPTION;


	NewtonSetThreadsCount(m_scene->GetNewton(), m_mainMenu->m_threadsTracks[m_microthreadIndex-ID_SELECT_MICROTHREADS]); 
	m_microthreadIndex = m_microthreadIndex - FXDataTarget::ID_OPTION;

	if (m_useParallelSolver) {
		NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), 1);
	}

	if (m_physicProfilerState) {
		m_scene->m_showProfiler[NEWTON_PROFILER_WORLD_UPDATE] = 1;
	}
}



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

long NewtonDemos::onSimdInstructions(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	int selection = ((m_cpuInstructionsMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_PLATFORMS;
	NewtonSetCurrentDevice (m_scene->GetNewton(), selection);
	END_MENU_OPTION();
	return 1;
}
#endif



void Friction (DemoEntityManager* const scene);
void Restitution (DemoEntityManager* const scene);
void PrecessingTops (DemoEntityManager* const scene);
void ClosestDistance (DemoEntityManager* const scene);
void ConvexCast (DemoEntityManager* const scene);
void PrimitiveCollision (DemoEntityManager* const scene);
void KinematicBodies (DemoEntityManager* const scene);
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
	{"Simple soft Body", "show simple stack of Boxes", SoftBodies},


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

		NewtonDemos* const frame = new NewtonDemos( "Newton 3.0 SDK demos", wxDefaultPosition, wxSize(1024, 768));
		frame->Show(true);
		SetTopWindow(frame);

		// initialize open gl graphics
		frame->m_scene->InitGraphicsSystem();


		// load the default Scene		
		//frame->LoadDemo (DEFAULT_SCENE);
		wxMenuEvent loadDemo (wxEVT_COMMAND_MENU_SELECTED, NewtonDemos::ID_RUN_DEMO + DEFAULT_SCENE);
		frame->GetEventHandler()->ProcessEvent(loadDemo);


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
	EVT_MENU(wxID_EXIT, NewtonDemos::OnQuit)
	EVT_MENU(wxID_ABOUT, NewtonDemos::OnAbout)
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
//	FXMAPFUNCS(SEL_COMMAND,		NewtonDemos::ID_PLATFORMS,				NewtonDemos::ID_PLATFORMS + 16,					NewtonDemos::onSimdInstructions),

END_EVENT_TABLE()


NewtonDemos::NewtonDemos(const wxString& title, const wxPoint& pos, const wxSize& size)
	:wxFrame(NULL, -1, title, pos, size)
	,m_physicsUpdateMode(0)
	,m_suspendVisualUpdates(true)
	,m_autoSleepState(false)
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
	,m_microthreadIndex(1)
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
	int widths[] = { 150, 160, 150};
	m_statusbar->SetFieldsCount (sizeof (widths)/sizeof (widths[0]), widths);
	CalculateFPS(0.0f);

	//SetStatusText(_T("Engine settings:"));

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

		fileMenu->Append(wxID_ABOUT, _T("About"));
		
		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_PREFERENCES, _T("Preferences"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_NEW, _T("&New"), _T("Create a blank new scene"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_OPEN, _T("&Open"), _T("Open visual scene in dScene newton format"));
		fileMenu->Append(wxID_SAVE, _T("&Save"), _T("Save visual scene in dScene newton format"));

	//	fileMenu->AppendSeparator();
	//	fileMenu->Append(m_idImportPhysics, _T("&Open physics scene"), _T("Open physics scene in collada format"));
	//	fileMenu->Append(m_idExportPhysics, _T("&Save physics scene"), _T("Save physics in collada format"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_EXIT, _T("E&xit\tAlt-X"), _T("Quit SDK sample") );

		// add main menus to menu bar
		mainMenu->Append(fileMenu, _T("&File"));
	}

	// engine all demo examples
	{
		wxMenu* const sdkDemos = new wxMenu;
		int demosCount = int (sizeof (m_demosSelection) / sizeof m_demosSelection[0]);
		for (int i = 0; i < demosCount; i ++) {
			sdkDemos->AppendRadioItem (NewtonDemos::ID_RUN_DEMO + i,  m_demosSelection[i].m_name, m_demosSelection[i].m_description);
		}

		mainMenu->Append(sdkDemos, _T("&Demos"));
	}

	// option menu
	{
		wxMenu* const optionsMenu = new wxMenu;;

		optionsMenu->AppendCheckItem(ID_AUTOSLEEP_MODE, _T("Autosleep off"), _T("toogle auto sleep bodies"));
		optionsMenu->AppendCheckItem(ID_HIDE_VISUAL_MESHES, _T("Hide visual meshes"));
		optionsMenu->AppendCheckItem(ID_SHOW_COLLISION_MESH, _T("Show collision Mesh"));
		optionsMenu->AppendCheckItem(ID_SHOW_CONTACT_POINTS, _T("Show contact points"));
		optionsMenu->AppendCheckItem(ID_SHOW_NORMAL_FORCES, _T("Show normal forces"));
		optionsMenu->AppendCheckItem(ID_SHOW_AABB, _T("Show aabb"));
		optionsMenu->AppendCheckItem(ID_USE_PARALLEL_SOLVER, _T("Parallel solver on"));

//		optionsMenu->AppendSeparator();
//		//m_cpuModes[0] = new FXMenuRadio(m_optionsMenu, "Use x87 instructions", &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_USE_X87_INSTRUCTIONS);
//		//m_cpuModes[1] = new FXMenuRadio(m_optionsMenu, "Use sse instructions", &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_USE_SIMD_INSTRUCTIONS);
//		//m_cpuModes[2] = new FXMenuRadio(m_optionsMenu, "Use avx instructions", &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_USE_AVX_INSTRUCTIONS);
//		if (mainFrame->m_scene) {
//			int platformsCount = NewtonEnumrateDevices (mainFrame->m_scene->GetNewton());
//			for (int i = 0; i < platformsCount; i ++) {
//				char platform[256];
//				NewtonGetDeviceString (mainFrame->m_scene->GetNewton(), i, platform, sizeof (platform));
//				m_cpuModes[i] = new FXMenuRadio(m_optionsMenu, platform, &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_PLATFORMS + i);
//			}
//		}

		optionsMenu->AppendSeparator();
		optionsMenu->AppendRadioItem(ID_DYNAMICS_BROADPHASE, _T("dynamics broad phase"));
		optionsMenu->AppendRadioItem(ID_STATIC_BROADPHASE, _T("static broad phase"));
		optionsMenu->AppendRadioItem(ID_HYBRID_BROADPHASE, _T("hybrid broad phase"));


		optionsMenu->AppendSeparator();
		optionsMenu->Append(ID_SHOW_CONCURRENCE_PROFILER, _T("Show concurrent profiler"));
//		if (mainFrame->m_concurrentProfilerState) {
//			concurrentProfiler->setCheck(true);
//		}
//		FXMenuCheck* const threadProfiler = new FXMenuCheck(m_optionsMenu, "Show micro thread profiler", mainFrame, NewtonDemos::ID_SHOW_PROFILER);
		optionsMenu->Append(ID_SHOW_PROFILER, _T("Show micro thread profiler"));
//		if (mainFrame->m_threadProfilerState) {
//			threadProfiler->setCheck(true);
//		}

		optionsMenu->AppendSeparator();
		optionsMenu->Append(ID_SELECT_ALL_PROFILERS, _T("select all profiler"));
		optionsMenu->Append(ID_UNSELECT_ALL_PROFILERS, _T("unselect all profiler"));

		wxMenu* const profilerSubMenu = new wxMenu;
		m_profilerTracksMenu[0] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 0, _T("show global physics update performance chart"));
		m_profilerTracksMenu[1] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 1, _T("global collision update performance chart"));
		m_profilerTracksMenu[2] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 2, _T("broad phase collision performance chart"));
		m_profilerTracksMenu[3] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 3, _T("narrow phase collision performance chart"));
		m_profilerTracksMenu[4] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 4, _T("global dynamics update performance chart"));
		m_profilerTracksMenu[5] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 5, _T("dynamics setup performance chart"));
		m_profilerTracksMenu[6] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 6, _T("dynamics solver performance chart"));
		m_profilerTracksMenu[7] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 7, _T("force and torque callback performance chart"));
		m_profilerTracksMenu[8] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 8, _T("pre-simulation listener"));
		m_profilerTracksMenu[9] = profilerSubMenu->AppendCheckItem(ID_SHOW_PHYSICS_PROFILER + 9, _T("post-simulation listener"));
//		if (mainFrame->m_physicProfilerState) {
//			m_profilerTracksMenu[0]->setCheck(true);
//		}
		optionsMenu->AppendSubMenu (profilerSubMenu, _T("select sub profiler"));


		optionsMenu->AppendSeparator();
		optionsMenu->AppendCheckItem(ID_CONCURRENT_PHYSICS_UPDATE, _T("Concurrent physics update"));

		wxMenu* const microThreadedsSubMenu = new wxMenu;
//		FXMenuRadio* threadMenus[128];
		for (int i = 0 ; i < int (sizeof (m_threadsTracks)/ sizeof (m_threadsTracks[0])); i ++) {
			wxString msg;
			msg.Printf(wxT ("%d micro threads"), m_threadsTracks[i]);
			microThreadedsSubMenu->AppendRadioItem(ID_SELECT_MICROTHREADS + i, msg);
		}
//		threadMenus[0]->setCheck(true);
		optionsMenu->AppendSubMenu (microThreadedsSubMenu, _T("select microThread count"));


		mainMenu->Append(optionsMenu, _T("&Options"));
	}

	// add help menu
	{
		wxMenu* const helpMenu = new wxMenu;;

		helpMenu->Append(wxID_HELP, _T("About"));
//		helpMenu->Append(NewtonDemos::ID_ON_ABOUT, _T("About"));
		mainMenu->Append(helpMenu, _T("&Help"));
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
		SetAutoSleepMode (m_scene->GetNewton(), m_autoSleepState);
		NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);	
	}
}


void NewtonDemos::RestoreSettings ()
{
//	int cpuMode = ((m_cpuInstructionsMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_PLATFORMS;
//	NewtonSetCurrentDevice (m_scene->GetNewton(), cpuMode); 

//	int threadIndex = ((m_microthreadCount + FXDataTarget::ID_OPTION) & 0xffff) - ID_SELECT_MICROTHREADS;
//	NewtonSetThreadsCount(m_scene->GetNewton(), m_mainMenu->m_threadsTracks[threadIndex]); 

//	int broadPhase = ((m_broadPhaseMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_DYNAMICS_BROADPHASE;
//	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), broadPhase);
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

//		sprintf (statusText, "render fps: (%7.2f)  physics time: (%4.2fms)  bodyCount: (%d)   physicsThreads: (%d)  platform: (%s)  autosleepMode: (%s)    PhysMemory (%d kbytes)", 
//			m_fps, m_scene->GetPhysicsTime() * 1000.0f, NewtonWorldGetBodyCount(world), 
//			NewtonGetThreadsCount(world), platform, m_autoSleepState ? "on" : "off", memoryUsed);
		
		sprintf (statusText, "render fps: %7.2f", m_fps);
		m_statusbar->SetStatusText (statusText, 0);

		sprintf (statusText, "physics time: %4.2f ms", m_scene->GetPhysicsTime() * 1000.0f);
		m_statusbar->SetStatusText (statusText, 1);

		sprintf (statusText, "memory %d kbytes", memoryUsed);
		m_statusbar->SetStatusText (statusText, 2);
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

	wxMenu* const menu = m_mainMenu->GetMenu(m_mainMenu->FindMenu(_T ("Demos")));
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
	int selection = event.GetId() - ID_DYNAMICS_BROADPHASE;
	dAssert (selection >= 0);
	dAssert (selection <= 3);
	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), selection);

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

