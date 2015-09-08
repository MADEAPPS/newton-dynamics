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


#define DEMO_WIDTH	1280
#define DEMO_HEIGHT  960


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
//#define DEFAULT_SCENE	15			// simple Archimedes buoyancy
//#define DEFAULT_SCENE	16			// uniform Scaled Collision
//#define DEFAULT_SCENE	17			// non Uniform Scaled Collision
//#define DEFAULT_SCENE	18			// scaled mesh collision
//#define DEFAULT_SCENE	19			// simple convex decomposition
//#define DEFAULT_SCENE	20			// scene Collision
//#define DEFAULT_SCENE	21          // simple boolean operators 
//#define DEFAULT_SCENE	22			// simple convex fracturing 
//#define DEFAULT_SCENE	23			// structured convex fracturing 
//#define DEFAULT_SCENE	24			// multi ray casting using the threading Job scheduler
//#define DEFAULT_SCENE	25			// continuous collision
//#define DEFAULT_SCENE	26			// paper wall continuous collision
//#define DEFAULT_SCENE	27			// puck slide continuous collision
//#define DEFAULT_SCENE	28          // standard joints
//#define DEFAULT_SCENE	29			// articulated joints
//#define DEFAULT_SCENE	30			// basic rag doll
//#define DEFAULT_SCENE	31			// basic Car
//#define DEFAULT_SCENE	32			// heavy vehicles
#define DEFAULT_SCENE	33			// super Car
//#define DEFAULT_SCENE	34			// basic player controller
//#define DEFAULT_SCENE	35			// advanced player controller
//#define DEFAULT_SCENE	36			// cloth patch			
//#define DEFAULT_SCENE	37			// soft bodies			


void Friction (DemoEntityManager* const scene);
void Restitution (DemoEntityManager* const scene);
void PrecessingTops (DemoEntityManager* const scene);
void ClosestDistance (DemoEntityManager* const scene);
void ConvexCast (DemoEntityManager* const scene);
void PrimitiveCollision (DemoEntityManager* const scene);
void KinematicPlacement (DemoEntityManager* const scene);
void ClothPatch(DemoEntityManager* const scene);
void SoftBodies (DemoEntityManager* const scene);
void BasicBoxStacks (DemoEntityManager* const scene);
void SimpleMeshLevelCollision (DemoEntityManager* const scene);
void OptimizedMeshLevelCollision (DemoEntityManager* const scene);
void UniformScaledCollision (DemoEntityManager* const scene);
void NonUniformScaledCollision (DemoEntityManager* const scene);
void ScaledMeshCollision (DemoEntityManager* const scene);
void ContinuousCollision (DemoEntityManager* const scene);
void ContinuousCollision1 (DemoEntityManager* const scene);
void PuckSlide (DemoEntityManager* const scene);
void SceneCollision (DemoEntityManager* const scene);
void CompoundCollision(DemoEntityManager* const scene);
void AlchimedesBuoyancy(DemoEntityManager* const scene);
void SimpleConvexApproximation(DemoEntityManager* const scene);
void SimpleBooleanOperations(DemoEntityManager* const scene);
void SimpleConvexFracturing (DemoEntityManager* const scene);
void StructuredConvexFracturing (DemoEntityManager* const scene);
void UsingNewtonMeshTool (DemoEntityManager* const scene);
void MultiRayCast (DemoEntityManager* const scene);
void BasicCar (DemoEntityManager* const scene);
void SuperCar (DemoEntityManager* const scene);
void MilitaryTransport (DemoEntityManager* const scene);
void BasicPlayerController (DemoEntityManager* const scene);
void AdvancedPlayerController (DemoEntityManager* const scene);
void HeightFieldCollision (DemoEntityManager* const scene);
void UserPlaneCollision (DemoEntityManager* const scene);
void UserHeightFieldCollision (DemoEntityManager* const scene);
void DescreteRagDoll (DemoEntityManager* const scene);
void ArticulatedJoints (DemoEntityManager* const scene);
void StandardJoints (DemoEntityManager* const scene);


NewtonDemos::SDKDemos NewtonDemos::m_demosSelection[] = 
{
	{wxT("Using the newton mesh tool"), wxT("demonstrate how to use the newton mesh toll for mesh manipulation"), UsingNewtonMeshTool},
	{wxT("Coefficients of friction"), wxT("demonstrate the effect of various coefficient of friction"), Friction},
	{wxT("Coefficients of restitution"), wxT("demonstrate the effect of various coefficient of restitution"), Restitution},
	{wxT("Precessing tops"), wxT("show natural precession"), PrecessingTops},
	{wxT("Closest distance"), wxT("demonstrate closest distance to a convex shape"), ClosestDistance},
	{wxT("Primitive Collision"), wxT("demonstrate separate collision of primitives"), PrimitiveCollision},
	{wxT("Kinematic bodies"), wxT("demonstrate separate collision of primitives"), KinematicPlacement},
	{wxT("Primitive convex cast"), wxT("demonstrate separate primitive convex cast"), ConvexCast},
	{wxT("Simple box Stacks"), wxT("show simple stack of Boxes"), BasicBoxStacks},
	{wxT("Unoptimized mesh collision"), wxT("show simple level mesh"), SimpleMeshLevelCollision},
	{wxT("Optimized mesh collision"), wxT("show optimized level mesh"), OptimizedMeshLevelCollision},
	{wxT("Height field collision mesh"), wxT("show high file collision mesh"), HeightFieldCollision},
	{wxT("User infinite Plane collision mesh"), wxT("show high file collision mesh"), UserPlaneCollision},
	{wxT("User Height field collision mesh"), wxT("show high file collision mesh"), UserHeightFieldCollision},
	{wxT("Compound collision shape"), wxT("demonstrate compound collision"), CompoundCollision},
	{wxT("Archimedes Buoyancy"), wxT("show Archimedes Buoyancy using the trigger volume manager"), AlchimedesBuoyancy},
	{wxT("Uniform scaled collision shape"), wxT("demonstrate scaling shape"), UniformScaledCollision},
	{wxT("Non uniform scaled collision shape"), wxT("demonstrate scaling shape"), NonUniformScaledCollision},
	{wxT("Scaled mesh collision"), wxT("demonstrate scaling mesh scaling collision"), ScaledMeshCollision},
	{wxT("Simple convex decomposition"), wxT("demonstrate convex decomposition and compound collision"), SimpleConvexApproximation},
	{wxT("Multi geometry collision"), wxT("show static mesh with the ability of moving internal parts"), SceneCollision},
	{wxT("Simple boolean operations"), wxT("demonstrate simple boolean operations "), SimpleBooleanOperations},
	{wxT("Simple convex fracture"), wxT("demonstrate simple fracture destruction using Voronoi partition"), SimpleConvexFracturing},
	{wxT("Structured convex fracture"), wxT("demonstrate structured fracture destruction using Voronoi partition"), StructuredConvexFracturing},
	{wxT("Parallel ray cast"), wxT("using the threading Job scheduler"), MultiRayCast},
	{wxT("Continuous collision"), wxT("show continuous collision"), ContinuousCollision},
    {wxT("Paper wall continuous collision"), wxT("show fast continuous collision"), ContinuousCollision1},
	{wxT("Puck slide"), wxT("show continuous collision"), PuckSlide},
    {wxT("Standard Joints"), wxT("show some of the common joints"), StandardJoints},
	{wxT("Articulated robotic actuators joints"), wxT("demonstrate complex array of bodies interconnect by joints"), ArticulatedJoints},
	{wxT("Basic rag doll"), wxT("demonstrate simple rag doll"), DescreteRagDoll},
	{wxT("Basic Car"), wxT("show how to set up a vehicle controller"), BasicCar},
	{wxT("Heavy vehicles"), wxT("implement military type heavy Vehicles"), MilitaryTransport},
	{wxT("Super car"), wxT("implement a hight performance sport car"), SuperCar},
	{wxT("Basic player controller"), wxT("demonstrate simple player controller"), BasicPlayerController},
	{wxT("Advanced player controller"), wxT("demonstrate player interacting with other objects"), AdvancedPlayerController},
	{wxT("Simple cloth Patch"), wxT("show simple cloth patch"), ClothPatch},
	{wxT("Simple soft Body"), wxT("show simple soft body"), SoftBodies},

//	{wxT("skinned rag doll"), wxT("demonstrate simple rag doll"), SkinRagDoll},
};


int NewtonDemos::m_threadsTracks[] = {1, 2, 3, 4, 8, 12, 16};
int NewtonDemos::m_solverModes[] = {0, 1, 2, 4, 8};


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
		wxString title;
		title.Printf (wxT ("Newton %d.%02d SDK demos"), version / 100, version % 100);
		NewtonDemos* const frame = new NewtonDemos(title, wxDefaultPosition, wxSize(DEMO_WIDTH, DEMO_HEIGHT));

		frame->Show(true);
		SetTopWindow(frame);

		// initialize opengl graphics
		if (frame->m_scene) {
			frame->m_scene->InitGraphicsSystem();
		}

		// load the default Scene
		wxMenuEvent loadDemo (wxEVT_COMMAND_MENU_SELECTED, NewtonDemos::ID_RUN_DEMO + DEFAULT_SCENE);
		frame->GetEventHandler()->ProcessEvent(loadDemo);

		// select solve mode
		wxMenuEvent solverMode (wxEVT_COMMAND_MENU_SELECTED, NewtonDemos::ID_SOLVER_MODE + 3);
		frame->GetEventHandler()->ProcessEvent(solverMode);

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
	EVT_MENU(wxID_NEW, NewtonDemos::OnNew)

	// game menus events
	EVT_MENU_RANGE(ID_RUN_DEMO, ID_RUN_DEMO_RANGE, NewtonDemos::OnRunDemo)

	EVT_MENU_RANGE(ID_BROADPHSE_TYPE0, ID_BROADPHSE_COUNT, NewtonDemos::OnSelectBroadPhase)

	EVT_MENU(ID_AUTOSLEEP_MODE,	NewtonDemos::OnAutoSleepMode)
	EVT_MENU(ID_SHOW_STATISTICS, NewtonDemos::OnShowStatistics)
	EVT_MENU(ID_USE_PARALLEL_SOLVER, NewtonDemos::OnUseParallelSolver)
	EVT_MENU_RANGE(ID_SOLVER_MODE, ID_SOLVER_MODE_COUNT, NewtonDemos::OnSelectSolverMode)

	EVT_MENU_RANGE(ID_SOLVER_QUALITY, ID_SOLVER_QUALITY_COUNT, NewtonDemos::OnSelectSolverQuality)

	EVT_MENU(ID_HIDE_VISUAL_MESHES,	NewtonDemos::OnHideVisualMeshes)

	EVT_MENU_RANGE(ID_SHOW_COLLISION_MESH, ID_SHOW_COLLISION_MESH_RANGE, NewtonDemos::OnShowCollisionLines)

	EVT_MENU(ID_SHOW_CONTACT_POINTS, NewtonDemos::OnShowContactPoints)
	EVT_MENU(ID_SHOW_NORMAL_FORCES,	NewtonDemos::OnShowNormalForces)
	EVT_MENU(ID_SHOW_AABB, NewtonDemos::OnShowAABB)
	EVT_MENU(ID_SHOW_CENTER_OF_MASS, NewtonDemos::OnShowCenterOfMass)
	EVT_MENU(ID_SHOW_JOINTS, NewtonDemos::OnShowShowJoints)
	
	EVT_MENU_RANGE(ID_PLATFORMS, ID_PLATFORMS_MAX, NewtonDemos::OnSelectHardwareDevice)

	EVT_MENU(ID_CONCURRENT_PHYSICS_UPDATE, NewtonDemos::OnRunPhysicsConcurrent)
	EVT_MENU_RANGE(ID_SELECT_MICROTHREADS, ID_SELECT_MICROTHREADS_COUNT, NewtonDemos::OnSelectNumberOfMicroThreads)
	
	EVT_MENU(ID_SERIALIZE, NewtonDemos::OnSerializeWorld)
	EVT_MENU(ID_DESERIALIZE, NewtonDemos::OnDeserializeWorld)

	EVT_JOYSTICK_EVENTS (NewtonDemos::OnJoystickEvent)


//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_LOAD,								NewtonDemos::onLoad),
//	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SAVE,								NewtonDemos::onSave),


END_EVENT_TABLE()


NewtonDemos::NewtonDemos(const wxString& title, const wxPoint& pos, const wxSize& size)
	:wxFrame(NULL, -1, title, pos, size)
	,m_mainMenu(NULL)
	,m_joystick(NULL)
	,m_statusbar(NULL)
	,m_scene(NULL)
	,m_broadPhaseType(0)
	,m_physicsUpdateMode(0)
	,m_suspendVisualUpdates(true)
	,m_autoSleepState(true)
	,m_useParallelSolver(false)
	,m_hideVisualMeshes(false)
	,m_showContactPoints(false)
	,m_showNormalForces(false)
	,m_showAABB(false)
	,m_showJoints(false)
	,m_showCenterOfMass(false)
	,m_showStatistics(false)
	,m_concurrentProfilerState(false)
	,m_threadProfilerState(false)
	,m_hasJoysticController(false)
	,m_shiftKey(false)
	,m_controlKey(false)
	,m_solverModeIndex(3)
	,m_solverModeQuality(0)
	,m_debugDisplayMode(0)
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_joytickX(0)
	,m_joytickY(0)
	,m_joytickButtonMask(0)
	,m_framesCount(0)
	,m_microthreadIndex(0)
	,m_hardwareDevice(0)
	,m_timestepAcc(0)
	,m_fps(0.0f)
{
//m_broadPhaseType = 1;
//m_autoSleepState = false;
//m_microthreadIndex = 1;
//m_useParallelSolver = true;
//m_threadProfilerState = true;
//m_showNormalForces = true;
//m_showCenterOfMass = true;
//m_hideVisualMeshes = true;
//m_physicsUpdateMode = 1;
//m_showContactPoints = true;
//m_hardwareDevice = 2;
//m_showStatistics = true;
m_debugDisplayMode = 2;


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
	//int widths[] = {150, 160, 150, 90, 80, 100, 100};
	int widths[] = {-1, -1, -1, -1, -1, -1, -1};
	m_statusbar->SetFieldsCount (sizeof (widths)/sizeof (widths[0]), widths);
	CalculateFPS(0.0f);
	m_mainMenu = CreateMainMenu();


/*
	wxJoystick stick(wxJOYSTICK1);
	if (!stick.IsOk())
	{
		wxMessageBox(wxT("No joystick detected!"));
		return false;
	}

#if wxUSE_SOUND
	m_fire.Create(wxT("buttonpress.wav"));
#endif // wxUSE_SOUND

	m_minX = stick.GetXMin();
	m_minY = stick.GetYMin();
	m_maxX = stick.GetXMax();
	m_maxY = stick.GetYMax();
*/

//	fucking wxwidget require a fucking library just to read a fucking joystick fuck you WxWidget
//	m_joystick = new wxJoystick(wxJOYSTICK1); 
//	m_joystick->SetCapture(this, 10); 
}


NewtonDemos::~NewtonDemos()
{
//	m_joystick->ReleaseCapture(); 
//	delete m_joystick;
}


wxMenuBar* NewtonDemos::CreateMainMenu()
{
	wxMenuBar* const mainMenu =  new wxMenuBar();

	// adding the file menu
	{
		wxMenu* const fileMenu = new wxMenu;

		fileMenu->Append(wxID_ABOUT, wxT("About"));
		
		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_PREFERENCES, wxT("Preferences"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_NEW, wxT("&New"), wxT("Create a blank new scene"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_OPEN, wxT("&Open"), wxT("Open visual scene in dScene newton format"));
		fileMenu->Append(wxID_SAVE, wxT("&Save"), wxT("Save visual scene in dScene newton format"));

		fileMenu->AppendSeparator();
		fileMenu->Append(ID_SERIALIZE, wxT("&Serialize"), wxT("Serialize scene to binary file"));
		fileMenu->Append(ID_DESERIALIZE, wxT("&Deserialize"), wxT("Load previuoslly serialized scame"));

	//	fileMenu->AppendSeparator();
	//	fileMenu->Append(m_idImportPhysics, wxT("&Open physics scene"), wxT("Open physics scene in collada format"));
	//	fileMenu->Append(m_idExportPhysics, wxT("&Save physics scene"), wxT("Save physics in collada format"));

		fileMenu->AppendSeparator();
		fileMenu->Append(wxID_EXIT, wxT("E&xit\tAlt-X"), wxT("Quit SDK sample") );

		// add main menus to menu bar
		mainMenu->Append(fileMenu, wxT("&File"));
	}

	// engine all demo examples
	{
		wxMenu* const sdkDemos = new wxMenu;
		int demosCount = int (sizeof (m_demosSelection) / sizeof m_demosSelection[0]);
		for (int i = 0; i < demosCount; i ++) {
			sdkDemos->AppendRadioItem (NewtonDemos::ID_RUN_DEMO + i,  m_demosSelection[i].m_name, m_demosSelection[i].m_description);
		}

		mainMenu->Append(sdkDemos, wxT("&Demos"));
	}

	// option menu
	{
		wxMenu* const optionsMenu = new wxMenu;;

		optionsMenu->AppendCheckItem(ID_AUTOSLEEP_MODE, wxT("Auto sleep mode"), wxT("toogle auto sleep bodies"));
		optionsMenu->Check (ID_AUTOSLEEP_MODE, m_autoSleepState);

		optionsMenu->AppendCheckItem(ID_SHOW_STATISTICS, wxT("Show Stats on screen"), wxT("toogle on screen frame rate and other stats"));
		optionsMenu->AppendCheckItem(ID_USE_PARALLEL_SOLVER, wxT("Parallel solver on"));

		optionsMenu->AppendSeparator();
		optionsMenu->AppendRadioItem(ID_BROADPHSE_TYPE0, wxT("Default broaphase"), wxT("for scenes with more dynamics bodies than static"));
		optionsMenu->AppendRadioItem(ID_BROADPHSE_TYPE1, wxT("Persintent broaphase"), wxT("for scenes with lot more static bodies than dynamics"));

		optionsMenu->AppendSeparator();
		optionsMenu->AppendRadioItem(ID_SOLVER_MODE + 0, wxT("Exact solver on"));
		optionsMenu->AppendRadioItem(ID_SOLVER_MODE + 1, wxT("Iterative solver one passes"));
		optionsMenu->AppendRadioItem(ID_SOLVER_MODE + 2, wxT("Iterative solver two passes"));
		optionsMenu->AppendRadioItem(ID_SOLVER_MODE + 3, wxT("Iterative solver four passes"));
		optionsMenu->AppendRadioItem(ID_SOLVER_MODE + 4, wxT("Iterative solver eight passes"));

		dAssert (m_solverModeIndex >= 0);
		dAssert (m_solverModeIndex < int (sizeof (m_solverModes)/sizeof (m_solverModes[0])));
		optionsMenu->Check (ID_SOLVER_MODE + m_solverModeIndex, true);

		optionsMenu->AppendSeparator();
		optionsMenu->AppendRadioItem(ID_SOLVER_QUALITY + 0, wxT("Iterative Solver Quality Low"));
		optionsMenu->AppendRadioItem(ID_SOLVER_QUALITY + 1, wxT("Iterative Solver Quality High"));
		optionsMenu->Check (ID_SOLVER_QUALITY + m_solverModeQuality, true);

		optionsMenu->AppendSeparator();
		optionsMenu->AppendRadioItem(ID_SHOW_COLLISION_MESH, wxT("Hide collision Mesh"));
		optionsMenu->AppendRadioItem(ID_SHOW_COLLISION_MESH + 1, wxT("Show solid collision Mesh"));
		optionsMenu->AppendRadioItem(ID_SHOW_COLLISION_MESH + 2, wxT("Show wire frame collision Mesh"));

		optionsMenu->AppendSeparator();
		optionsMenu->AppendCheckItem(ID_HIDE_VISUAL_MESHES, wxT("Hide visual meshes"));
		optionsMenu->AppendCheckItem(ID_SHOW_CONTACT_POINTS, wxT("Show contact points"));
		optionsMenu->AppendCheckItem(ID_SHOW_NORMAL_FORCES, wxT("Show normal forces"));
		optionsMenu->AppendCheckItem(ID_SHOW_AABB, wxT("Show aabb"));
		optionsMenu->AppendCheckItem(ID_SHOW_CENTER_OF_MASS, wxT("Show center of mass"));
		optionsMenu->AppendCheckItem(ID_SHOW_JOINTS, wxT("show Joint debug info"));
	

		optionsMenu->AppendSeparator();
		int platformsCount = NewtonEnumerateDevices (m_scene->GetNewton());
		for (int i = 0; i < platformsCount; i ++) {
			wxString label;
			char platform[256];
			

			NewtonGetDeviceString (m_scene->GetNewton(), i, platform, sizeof (platform));
			#ifdef _POSIX_VER
				wxChar wPlatform[256];
				mbstowcs (wPlatform, platform, sizeof (platform));
				wxString tmp (wPlatform);
				label.Printf (wxT(" hardware mode %s"), tmp.c_str());
			#else 
				label.Printf (wxT(" hardware mode %s"), wxString(platform));
			#endif
			optionsMenu->AppendRadioItem(ID_PLATFORMS + i, label);
		}
		//optionsMenu->Check(ID_PLATFORMS, true);

		optionsMenu->AppendSeparator();
		optionsMenu->AppendCheckItem(ID_CONCURRENT_PHYSICS_UPDATE, wxT("Concurrent physics update"));

		wxMenu* const microThreadedsSubMenu = new wxMenu;
		for (int i = 0 ; i < int (sizeof (m_threadsTracks)/ sizeof (m_threadsTracks[0])); i ++) {
			wxString msg;
			msg.Printf(wxT ("%d micro threads"), m_threadsTracks[i]);
			microThreadedsSubMenu->AppendRadioItem(ID_SELECT_MICROTHREADS + i, msg);
		}
		optionsMenu->AppendSubMenu (microThreadedsSubMenu, wxT("select microThread count"));


		mainMenu->Append(optionsMenu, wxT("&Options"));
	}

	// add help menu
	{
		wxMenu* const helpMenu = new wxMenu;;

		helpMenu->Append(wxID_HELP, wxT("About"));
//		helpMenu->Append(NewtonDemos::ID_ON_ABOUT, wxT("About"));
		mainMenu->Append(helpMenu, wxT("&Help"));
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
		NewtonSetSolverModel (m_scene->GetNewton(), m_solverModes[m_solverModeIndex]);
		NewtonSetSolverConvergenceQuality (m_scene->GetNewton(), m_solverModeQuality ? 1 : 0);
		NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);	
		NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), m_broadPhaseType);
	}
}


void NewtonDemos::RestoreSettings ()
{
	NewtonSetCurrentDevice (m_scene->GetNewton(), m_hardwareDevice); 
	NewtonSetThreadsCount(m_scene->GetNewton(), m_threadsTracks[m_microthreadIndex]); 
}


void NewtonDemos::LoadDemo (int index)
{
	BEGIN_MENU_OPTION();

	m_scene->Cleanup();
    
	m_demosSelection[index].m_launchDemoCallback (m_scene);
	m_scene->SwapBuffers(); 

	RestoreSettings ();
	m_scene->ResetTimer();

	// clean up all caches the engine have saved
	NewtonInvalidateCache(m_scene->GetNewton());

	END_MENU_OPTION();
}


bool NewtonDemos::IsShiftKeyDown () const
{
	return m_shiftKey;
}

bool NewtonDemos::IsControlKeyDown () const
{
	return m_controlKey;
}



void NewtonDemos::KeyDown(const wxKeyEvent &event)
{
	int keyCode = event.GetKeyCode();
	if (keyCode == WXK_ESCAPE)  {
		// send a display refresh event in case the runtime update is stopped bu the user.
		wxMenuEvent exitEvent (wxEVT_COMMAND_MENU_SELECTED, wxID_EXIT);
		GetEventHandler()->ProcessEvent(exitEvent);
	}

	m_shiftKey = event.ShiftDown();
	m_controlKey = event.ControlDown();

//	if (!event.GetModifiers()) {
		int code = keyCode & 0xff; 
		m_key[m_keyMap[code]] = true;
//	}
}


void NewtonDemos::KeyUp(const wxKeyEvent &event)
{
	m_shiftKey = event.ShiftDown();
	m_controlKey = event.ControlDown();

//	if (!event.GetModifiers()) {
		int keyCode = event.GetKeyCode();
		int code = keyCode & 0xff;
		m_key[m_keyMap[code]] = false;
//	}
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

bool NewtonDemos::GetKeyState(int key) const
{
	return wxGetKeyState(wxKeyCode(key & 0xff));
//	return m_key[m_keyMap[key & 0xff]] ? true : false;
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

		wxString statusText;
		NewtonWorld* const world = m_scene->GetNewton();
		char platform[256];
		NewtonGetDeviceString(world, NewtonGetCurrentDevice(world), platform, sizeof (platform));
		int memoryUsed = NewtonGetMemoryUsed() / (1024) ;
		
		statusText.Printf (wxT ("render fps: %7.2f"), m_fps);
		m_statusbar->SetStatusText (statusText, 0);

		statusText.Printf (wxT ("physics time: %4.2f ms"), m_scene->GetPhysicsTime() * 1000.0f);
		m_statusbar->SetStatusText (statusText, 1);

		statusText.Printf (wxT ("memory: %d kbytes"), memoryUsed);
		m_statusbar->SetStatusText (statusText, 2);

		statusText.Printf (wxT ("bodies: %d"), NewtonWorldGetBodyCount(world));
		m_statusbar->SetStatusText (statusText, 3);

		statusText.Printf (wxT ("threads: %d"), NewtonGetThreadsCount(world));
		m_statusbar->SetStatusText (statusText, 4);

		statusText.Printf (wxT ("auto sleep: %s"), m_autoSleepState ? wxT("on") : wxT("off"));
		m_statusbar->SetStatusText (statusText, 5);

		char floatMode[128];
		NewtonGetDeviceString (world, m_hardwareDevice, floatMode, sizeof (floatMode));
		statusText.Printf (wxT ("instructions: %s"),  wxString::FromAscii(floatMode).wc_str());
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
	m_debugDisplayMode = event.GetId() - ID_SHOW_COLLISION_MESH;
	SetDebugDisplayMode (m_debugDisplayMode);
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

void NewtonDemos::OnShowCenterOfMass(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_showCenterOfMass = event.IsChecked(); 
	END_MENU_OPTION();
}

void NewtonDemos::OnShowShowJoints(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_showJoints = event.IsChecked(); 
	END_MENU_OPTION();
}


void NewtonDemos::OnUseParallelSolver(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_useParallelSolver = event.IsChecked(); 
	NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);
	END_MENU_OPTION();
}

void NewtonDemos::OnSelectSolverMode(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_solverModeIndex = dClamp (event.GetId() - ID_SOLVER_MODE, 0, int (sizeof (m_solverModes)/sizeof (m_solverModes[0])));
	END_MENU_OPTION();
}

void NewtonDemos::OnSelectSolverQuality(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_solverModeQuality = dClamp(event.GetId() - ID_SOLVER_QUALITY, 0, 1);
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

void NewtonDemos::OnSelectHardwareDevice(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_hardwareDevice = event.GetId() - ID_PLATFORMS;
	NewtonSetCurrentDevice (m_scene->GetNewton(), m_hardwareDevice);
	END_MENU_OPTION();
}

void NewtonDemos::OnShowStatistics(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_showStatistics = event.IsChecked(); 
	END_MENU_OPTION();
}

void NewtonDemos::OnNew (wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_scene->Cleanup();
	RestoreSettings ();
	m_scene->ResetTimer();
	END_MENU_OPTION();
}


void NewtonDemos::OnSerializeWorld (wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();

	wxFileDialog open (this, wxT("Export a Newton Dynamics Serialized Physics Scene"), wxT("../../../media"), wxT(""), wxT("*.bin"));
	if (open.ShowModal() == wxID_OK) {
		wxString currentDocPath (open.GetPath());
		m_scene->SerializedPhysicScene (currentDocPath.mb_str());
	}
	m_scene->ResetTimer();

	END_MENU_OPTION();
}

void NewtonDemos::OnDeserializeWorld(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();

	wxFileDialog save (this, wxT("Import a Newton Dynamics Serialized Physics Scene"), wxT("../../../media"), wxT(""), wxT("*.bin"));
	if (save.ShowModal() == wxID_OK) {
		wxString currentDocPath (save.GetPath());
		//m_scene->DeserializedPhysicScene (currentDocPath.c_str());
		m_scene->DeserializedPhysicScene (currentDocPath.mb_str());
		RestoreSettings ();
	}

	m_scene->ResetTimer();
	END_MENU_OPTION();
}

void NewtonDemos::OnJoystickEvent(wxJoystickEvent& event)
{
}

void NewtonDemos::OnSelectBroadPhase(wxCommandEvent& event)
{
	BEGIN_MENU_OPTION();
	m_broadPhaseType = dClamp (event.GetId() - ID_BROADPHSE_TYPE0, 0, 1);
	END_MENU_OPTION();
}

#if 0
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

#endif
