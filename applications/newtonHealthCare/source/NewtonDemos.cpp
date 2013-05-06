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
#include "DemoMenu.h"
#include "DemoCamera.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "DemoEntityManager.h"
#include "DemoDialogHelpers.h"


//#define USE_REPLAY_FILE
//#define CREATE_REPLAY_FILE


#if defined (CREATE_REPLAY_FILE) || defined (USE_REPLAY_FILE)
FILE* replayFile;	
#endif

#define DEFAULT_SCENE	0			// using NetwonMesh Tool
						

// Message Map
FXDEFMAP(NewtonDemos) NewtonDemosMessageMap[]=
{
	FXMAPFUNC(SEL_PAINT,		NewtonDemos::ID_CANVAS,								NewtonDemos::onPaint),
	FXMAPFUNC(SEL_CHORE,		NewtonDemos::ID_CHORE,								NewtonDemos::onChore),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_NEW,								NewtonDemos::onNew),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_LOAD,								NewtonDemos::onLoad),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SAVE,								NewtonDemos::onSave),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SERIALIZE,							NewtonDemos::onSerializeWorld),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_DESERIALIZE,						NewtonDemos::onDeserializeWorld),

	FXMAPFUNC(SEL_KEYPRESS,		0,													NewtonDemos::onKeyPress),
	FXMAPFUNC(SEL_KEYRELEASE,	0,													NewtonDemos::onKeyRelease),
	FXMAPFUNC(SEL_MOTION,		NewtonDemos::ID_CANVAS,								NewtonDemos::onMouseMove),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_HIDE_VISUAL_MESHES,					NewtonDemos::onHideVisualMeshes),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SHOW_COLLISION_MESH,				NewtonDemos::onShowCollisionLines),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SHOW_CONTACT_POINTS,				NewtonDemos::onShowContactPoints),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SHOW_AABB,							NewtonDemos::onShowAABB),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_AUTOSLEEP_MODE,						NewtonDemos::onAutoSleepMode),

	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_USE_PARALLEL_SOLVER,				NewtonDemos::onUseParallelSolver),
	
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SHOW_CONCURRENCE_PROFILER,			NewtonDemos::onShowConcurrentProfiler),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SHOW_PROFILER,						NewtonDemos::onShowThreadProfiler),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_SELECT_ALL_PROFILERS,				NewtonDemos::onSelectAllPerformanceChart),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_UNSELECT_ALL_PROFILERS,				NewtonDemos::onUnselectAllPerformanceChart),
	FXMAPFUNC(SEL_COMMAND,		NewtonDemos::ID_CONCURRENT_PHYSICS_UPDATE,			NewtonDemos::onRunPhysicsConcurrent),
	
	FXMAPFUNCS(SEL_COMMAND,		NewtonDemos::ID_SHOW_PHYSICS_PROFILER,	NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 16 ,	NewtonDemos::onShowProfiler),
	FXMAPFUNCS(SEL_COMMAND,		NewtonDemos::ID_USE_X87_INSTRUCTIONS,	NewtonDemos::ID_USE_AVX_INSTRUCTIONS,			NewtonDemos::onSimdInstructions),
	FXMAPFUNCS(SEL_COMMAND,		NewtonDemos::ID_DYNAMICS_BROADPHASE,	NewtonDemos::ID_HYBRID_BROADPHASE,				NewtonDemos::onBroadPhaseType),
	FXMAPFUNCS(SEL_COMMAND,		NewtonDemos::ID_SELECT_MICROTHREADS,	NewtonDemos::ID_SELECT_MICROTHREADS + 16,		NewtonDemos::onSelectNumberOfMicroThreads),
	FXMAPFUNCS(SEL_COMMAND,		NewtonDemos::ID_RUN_DEMO,				NewtonDemos::ID_RUN_DEMO_RANGE,					NewtonDemos::onRunDemo),
};


// Implementation
FXIMPLEMENT(NewtonDemos,FXMainWindow,NewtonDemosMessageMap,ARRAYNUMBER(NewtonDemosMessageMap))




int NewtonDemos::m_totalMemoryUsed = 0;

void *operator new (size_t size) 
{ 
	void* const ptr = malloc (size);

//	unsigned xxx = unsigned (ptr);
//	xxx &= 0xffff;
//	_ASSERTE (xxx != 0x6ea0);
//	_ASSERTE (!((xxx == 0x6ea0) && (size >= 16384)));
//	dTrace (("%d %x\n", xxx, ptr))
	return ptr; 
}                                          

void operator delete (void *ptr) 
{ 
	free (ptr); 
}


int main(int argc, char *argv[])
{
	// Enable run-time memory check for debug builds.
#ifdef _MSC_VER
	_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif


	FXApp application("Newton Dynamics demos", "Newton Dynamics demos");

	// Set the memory allocation function before creation the newton world
	// this is the only function that can be called before the creation of the newton world.
	// it should be called once, and the the call is optional 
	NewtonSetMemorySystem (NewtonDemos::PhysicsAlloc, NewtonDemos::PhysicsFree);

	application.init(argc,argv);

	// Make main window
	NewtonDemos* const mainWindow = new NewtonDemos (application);

	// Create the application's windows
	application.create();

	// load the default Scene		
	mainWindow->LoadDemo (DEFAULT_SCENE + NewtonDemos::ID_RUN_DEMO);

	// start the real time loop, by sending a message that send himself again
	application.addChore(mainWindow, NewtonDemos::ID_CHORE);
		
	
	// Run the application
	return application.run();
}


NewtonDemos::NewtonDemos()
{

}


NewtonDemos::NewtonDemos(FXApp& application)
	:FXMainWindow(&application, "Newton Dynamics 3.00 unit test demos", NULL, NULL, DECOR_ALL, 0, 0, 1024, 768)
	,m_autoSleepState(false)
	,m_useParallelSolver(false)
//	,m_useParallelSolver(true)
	,m_hideVisualMeshes(false)
	,m_debugDisplayState(false)
//	,m_debugDisplayState(true)
	,m_showAABB(false)
	,m_showContactPoints(false)
	,m_physicProfilerState(true)
	,m_threadProfilerState(true)
	,m_concurrentProfilerState(true)
	,m_showContactPointState(false)
	,m_showStatistics(true)
	,m_doVisualUpdates(true)
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_framesCount(0)
	,m_timestepAcc(0)
	,m_fps(0.0f)
	,m_broadPhaseMode (ID_DYNAMICS_BROADPHASE)
	,m_broadPhaseSelection(m_broadPhaseMode)
	,m_cpuInstructionsMode(ID_USE_X87_INSTRUCTIONS)
//	,m_cpuInstructionsMode(ID_USE_AVX_INSTRUCTIONS)
	,m_cpuInstructionSelection (m_cpuInstructionsMode)
	,m_microthreadCount(ID_SELECT_MICROTHREADS)
	,m_microthreadCountSelection(m_microthreadCount)
	,m_physicsUpdateMode(0)
{
	m_broadPhaseSelection.setTarget(this);
	m_broadPhaseSelection.setSelector(ID_DYNAMICS_BROADPHASE);

	m_cpuInstructionSelection.setTarget(this);
	m_cpuInstructionSelection.setSelector(ID_USE_X87_INSTRUCTIONS);

	m_microthreadCountSelection.setTarget(this);
	m_microthreadCountSelection.setSelector(ID_SELECT_MICROTHREADS);


	// create status bar for showing results 
	m_statusbar = new FXStatusBar(this, LAYOUT_SIDE_BOTTOM|LAYOUT_FILL_X|STATUSBAR_WITH_DRAGCORNER);

	// create the main menu
	FXDockSite* const dockMenuFrame = new FXDockSite(this,DOCKSITE_NO_WRAP|LAYOUT_SIDE_TOP|LAYOUT_FILL_X);
	m_mainMenu = new DemoMenu (dockMenuFrame, this);

	// create a main frame to hold the Render canvas
	FXHorizontalFrame* const glFrame = new FXHorizontalFrame(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|LAYOUT_FILL_Y, 0,0,0,0, 0,0,0,0, 4,4);

	// create the open gl canvas and scene manager
	m_scene = new DemoEntityManager (glFrame, this);

	// init the fps status bar
	CalculateFPS(0.0f);

	// Set the initial states
	onShowCollisionLines(this, 0, (void*) m_debugDisplayState);


	// clear the key map
	memset (m_key, 0, sizeof (m_key));
	for (int i = 0; i < sizeof (m_keyMap); i ++) {
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


#if (defined (CREATE_REPLAY_FILE) || defined (USE_REPLAY_FILE))
	printf ("warning: using replay system\n"); 
	printf ("comment out defines CREATE_REPLAY_FILE and USE_REPLAY_FILE\n");
	printf ("in file \"NewtonDemos.cpp\" and recompile\n");
	#ifdef CREATE_REPLAY_FILE
		replayFile = fopen ("replayFile.bin", "wb");
	#else 
		replayFile = fopen ("replayFile.bin", "rb");
	#endif
#endif


	// figure out the supported modes
	for (int i = 0; i < 3; i ++) {
		char platform[64];
		NewtonWorld* const world = m_scene->GetNewton();

		NewtonSetPlatformArchitecture (m_scene->GetNewton(), i); 
		int arch = NewtonGetPlatformArchitecture(world, platform);
		if (arch != i) {
			// disable this mode from menu
			if (int (m_cpuInstructionsMode) == int (i + ID_USE_X87_INSTRUCTIONS)) {
				m_cpuInstructionsMode = ID_USE_X87_INSTRUCTIONS;
			}
  			m_mainMenu->m_cpuModes[i]->disable();
		}
	}

	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), m_broadPhaseMode - ID_DYNAMICS_BROADPHASE);
	m_broadPhaseMode = m_broadPhaseMode - FXDataTarget::ID_OPTION;

	//m_mainMenu->m_cpuModes[2]->disable();

	NewtonSetPlatformArchitecture (m_scene->GetNewton(), m_cpuInstructionsMode-ID_USE_X87_INSTRUCTIONS); 
	m_cpuInstructionsMode = m_cpuInstructionsMode - FXDataTarget::ID_OPTION;

	NewtonSetThreadsCount(m_scene->GetNewton(), m_mainMenu->m_threadsTracks[m_microthreadCount-ID_SELECT_MICROTHREADS]); 
	m_microthreadCount = m_microthreadCount - FXDataTarget::ID_OPTION;

	if (m_useParallelSolver) {
		NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), 1);
	}

	if (m_physicProfilerState) {
		m_scene->m_showProfiler[NEWTON_PROFILER_WORLD_UPDATE] = 1;
	}
}


NewtonDemos::~NewtonDemos()
{
#if (defined (CREATE_REPLAY_FILE) || defined (USE_REPLAY_FILE))
	 fclose (replayFile);
#endif

}


void NewtonDemos::BEGIN_MENU_OPTION()																				
{
	m_doVisualUpdates = false;																			
	if (m_scene->GetNewton()) {																			
		NewtonWaitForUpdateToFinish (m_scene->GetNewton());												
	}
}


void NewtonDemos::END_MENU_OPTION()
{
	m_doVisualUpdates = true;																			
	if (m_scene->GetNewton()) {		
		NewtonWaitForUpdateToFinish (m_scene->GetNewton());
		SetAutoSleepMode (m_scene->GetNewton(), m_autoSleepState);
		NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);	
	}
}


void NewtonDemos::create()
{
	FXMainWindow::create();
	show(PLACEMENT_SCREEN);
}



// memory allocation for Newton
void* NewtonDemos::PhysicsAlloc (int sizeInBytes)
{
	m_totalMemoryUsed += sizeInBytes;
	return malloc (sizeInBytes);
}

// memory free use by teh engine
void NewtonDemos::PhysicsFree (void* ptr, int sizeInBytes)
{
	m_totalMemoryUsed -= sizeInBytes;
	free (ptr);
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
		char platform[64];
		NewtonGetPlatformArchitecture(world, platform);
		int memoryUsed = NewtonGetMemoryUsed() / (1024) ;

		sprintf (statusText, "render fps: (%7.2f)  physics time: (%4.2fms)  bodyCount: (%d)   physicsThreads: (%d)  platform: (%s)  autosleepMode: (%s)    PhysMemory (%d kbytes)", 
							  m_fps, m_scene->GetPhysicsTime() * 1000.0f, NewtonWorldGetBodyCount(world), 
							  NewtonGetThreadsCount(world), platform, m_autoSleepState ? "on" : "off", memoryUsed);

		m_statusbar->getStatusLine()->setNormalText(statusText);
	}
}

void NewtonDemos::RestoreSettings ()
{
	int cpuMode = ((m_cpuInstructionsMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_USE_X87_INSTRUCTIONS;
	NewtonSetPlatformArchitecture (m_scene->GetNewton(), cpuMode); 

	int threadIndex = ((m_microthreadCount + FXDataTarget::ID_OPTION) & 0xffff) - ID_SELECT_MICROTHREADS;
	NewtonSetThreadsCount(m_scene->GetNewton(), m_mainMenu->m_threadsTracks[threadIndex]); 

	int broadPhase = ((m_broadPhaseMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_DYNAMICS_BROADPHASE;
	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), broadPhase);


//NewtonSetThreadsCount(m_scene->GetNewton(), 4); 
}


/*
void NewtonDemos::OnSave()
{
	BEGIN_MENU_OPTION();
	END_MENU_OPTION();
}
*/


void NewtonDemos::GetMousePosition (int& posX, int& posY) const
{
#if (defined (CREATE_REPLAY_FILE) || defined (USE_REPLAY_FILE))
	int location[2];
	#ifdef CREATE_REPLAY_FILE
		location[0] = m_mousePosX;
		location[1] = m_mousePosY;
		fwrite (&location, sizeof (location), 1, replayFile);
		fflush (replayFile);
	#else 
		fread (&location, sizeof (location), 1, replayFile);
	#endif
	posX = location[0];
	posY = location[1];

#else 
	posX = m_mousePosX;
	posY = m_mousePosY;
#endif
}


bool NewtonDemos::GetMouseKeyState (int button) const
{
#if (defined (CREATE_REPLAY_FILE) || defined (USE_REPLAY_FILE))
	char state = false;
	#ifdef CREATE_REPLAY_FILE
		if ((button >= 0) && (button <= 2)) {
			state = m_key[m_keyMap[button]];
		}
		fwrite (&state, sizeof (state), 1, replayFile);
		fflush (replayFile);
	#else 
		fread (&state, sizeof (state), 1, replayFile);
	#endif
	return state ? true : false;

#else 
	if ((button >= 0) && (button <= 2)) {
		return m_key[m_keyMap[button]] ? true : false;
	}
	return false;
#endif
}


bool NewtonDemos::GetKeyState (char asciiCode) const
{
#if (defined (CREATE_REPLAY_FILE) || defined (USE_REPLAY_FILE))
	char state;
	#ifdef CREATE_REPLAY_FILE
		state = m_key[m_keyMap[asciiCode & 0x7f]];
		fwrite (&state, sizeof (state), 1, replayFile);
		fflush (replayFile);
	#else 
		fread (&state, sizeof (state), 1, replayFile);
	#endif
	return state ? true : false;
#else
	return m_key[m_keyMap[asciiCode & 0x7f]] ? true : false;
#endif

}

void NewtonDemos::ReadKeyboardAsync ()
{
#ifdef WIN32
	for (int i = 1; i < 128; i ++) {
		short state = GetAsyncKeyState(i);
		m_key[i] = state>>15;
	}

	if (m_key[0x1b]) {
		getApp()->addChore(this, ID_CLOSE);
	}
#endif
}


long NewtonDemos::onKeyPress(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXEvent* const event=(FXEvent*)eventPtr;
	
	if ((event->code & 0xff) == 0x1b) {
		getApp()->addChore(this, ID_CLOSE);
	}
#ifndef WIN32
	int code = event->code & 0x7f;  //NOTE ?
	m_key[code] = true;
#endif
	return 1;
}


long NewtonDemos::onKeyRelease(FXObject* sender, FXSelector id, void* eventPtr)
{
#ifndef WIN32
	FXEvent* const event=(FXEvent*)eventPtr;
	int code = event->code & 0x7f;
	m_key[code] = false;
#endif
	return 1;
}


long NewtonDemos::onMouseMove(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXEvent* const event=(FXEvent*)eventPtr;
	m_mousePosX = event->win_x;
	m_mousePosY = event->win_y;
	return 1;
}

void NewtonDemos::LoadDemo (int index)
{
	BEGIN_MENU_OPTION();

	index = FXSELID(index);
	m_scene->Cleanup();
	m_menubar->LoadDemo(m_scene, index - ID_RUN_DEMO);

	RestoreSettings ();
	m_scene->ResetTimer();


	END_MENU_OPTION();
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

		// load the scene from and alchemedia file format
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



long NewtonDemos::onRunDemo(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	LoadDemo (id);

	RestoreSettings ();
	END_MENU_OPTION();

	return 1;
}

long NewtonDemos::onHideVisualMeshes(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	FXuval val = FXuval (eventPtr);

	m_hideVisualMeshes = val ? true : false;

	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onShowConcurrentProfiler(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	FXuval val = FXuval (eventPtr);

	m_concurrentProfilerState = val ? true : false;

	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onShowThreadProfiler(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	FXuval val = FXuval (eventPtr);

	m_threadProfilerState = val ? true : false;

	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onShowProfiler(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	FXuval val = FXuval (eventPtr);

	int track = (id & 0xffff)- ID_SHOW_PHYSICS_PROFILER;
	int state = val ? 1 : 0;
	m_scene->m_showProfiler[track] = state;

	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onSelectAllPerformanceChart(FXObject* sender, FXSelector id, void* eventPtr)
{
	int count = sizeof (m_mainMenu->m_profilerTracksMenu) / sizeof (m_mainMenu->m_profilerTracksMenu[0]);
	for (int i = 0; i < count; i ++) {
		m_scene->m_showProfiler[i] = 1;
		m_mainMenu->m_profilerTracksMenu[i]->setCheck(true);
	}
	return 1;
}

long NewtonDemos::onUnselectAllPerformanceChart(FXObject* sender, FXSelector id, void* eventPtr)
{
	int count = sizeof (m_mainMenu->m_profilerTracksMenu) / sizeof (m_mainMenu->m_profilerTracksMenu[0]);
	for (int i = 0; i < count; i ++) {
		m_scene->m_showProfiler[i] = 0;
		m_mainMenu->m_profilerTracksMenu[i]->setCheck(false);
	}
	return 1;
}

long NewtonDemos::onShowAABB(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	FXuval val = FXuval (eventPtr);
	m_showAABB = val ? true : false;
	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onAutoSleepMode(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	FXuval val = FXuval (eventPtr);
	m_autoSleepState = val ? true : false;
	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onShowContactPoints(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	FXuval val = FXuval (eventPtr);
	m_showContactPoints = val ? true : false;
	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onShowCollisionLines(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	FXuval val = FXuval (eventPtr);

	m_debugDisplayState = val ? true : false;
	SetDebugDisplayMode (m_debugDisplayState);

	END_MENU_OPTION();
	return 1;
}



long NewtonDemos::onRunPhysicsConcurrent(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	FXuval val = FXuval (eventPtr);
	m_physicsUpdateMode = val ? true : false;

	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onSelectNumberOfMicroThreads(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	int selection = ((m_microthreadCount + FXDataTarget::ID_OPTION) & 0xffff) - ID_SELECT_MICROTHREADS;

	NewtonSetThreadsCount(m_scene->GetNewton(), m_mainMenu->m_threadsTracks[selection]);

	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onUseParallelSolver(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();

	FXuval val = FXuval (eventPtr);
	m_useParallelSolver =  val ? true : false;

	NewtonSetMultiThreadSolverOnSingleIsland (m_scene->GetNewton(), m_useParallelSolver ? 1 : 0);

	END_MENU_OPTION();
	return 1;
}


long NewtonDemos::onBroadPhaseType(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	int selection = ((m_broadPhaseMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_DYNAMICS_BROADPHASE;

	_ASSERTE (selection >= 0);
	_ASSERTE (selection <= 3);
	NewtonSelectBroadphaseAlgorithm (m_scene->GetNewton(), selection);

	END_MENU_OPTION();
	return 1;
}

long NewtonDemos::onSimdInstructions(FXObject* sender, FXSelector id, void* eventPtr)
{
	BEGIN_MENU_OPTION();
	int selection = ((m_cpuInstructionsMode + FXDataTarget::ID_OPTION) & 0xffff) - ID_USE_X87_INSTRUCTIONS;

	switch (selection) 
	{
		case 0:
			NewtonSetPlatformArchitecture (m_scene->GetNewton(), 0);  //x87 mode
			break;

		case 1:
			NewtonSetPlatformArchitecture (m_scene->GetNewton(), 1);  //Intel SSE 
			break;

		case 2:
			NewtonSetPlatformArchitecture (m_scene->GetNewton(), 2);  //Intel AVX mode
			break;
	}

	END_MENU_OPTION();
	return 1;
}



long NewtonDemos::onChore(FXObject* sender, FXSelector id, void* eventPtr)
{
	onPaint(sender, id, eventPtr);
	getApp()->addChore(this, ID_CHORE);
	return 1;
}

long NewtonDemos::onPaint(FXObject* sender, FXSelector id, void* eventPtr)
{
	if (m_doVisualUpdates) {
		// render scene
		dFloat timestep = dGetElapsedSeconds();	
		m_scene->UpdateScene(timestep);

		CalculateFPS(timestep);
	}
	return 1;
}

