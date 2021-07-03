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

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndDemoCamera.h"
#include "ndFileBrowser.h"
#include "ndPhysicsWorld.h"
#include "ndSoundManager.h"
#include "ndPhysicsUtils.h"
#include "ndDebugDisplay.h"
#include "ndDemoDebugMesh.h"
#include "ndTargaToOpenGl.h"
#include "ndShaderPrograms.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraManager.h"
#include "ndDemoCameraManager.h"
#include "ndHighResolutionTimer.h"


//#define ENABLE_REPLAY

#ifdef ENABLE_REPLAY
	#define REPLAY_RECORD
#endif

#define PROJECTILE_INITIAL_SPEED	20.0f


//#define DEFAULT_SCENE	0		// setting basic rigidbody
//#define DEFAULT_SCENE	1		// setting gpu basic rigidbody
//#define DEFAULT_SCENE	2		// setting friction ramp
//#define DEFAULT_SCENE	3		// setting basic Stacks
//#define DEFAULT_SCENE	4		// setting basic Trigger
//#define DEFAULT_SCENE	5		// setting basic player
//#define DEFAULT_SCENE	6		// setting particle fluid
//#define DEFAULT_SCENE	7		// static mesh collision 
//#define DEFAULT_SCENE	8		// setting basic joints
#define DEFAULT_SCENE	9		// setting basic vehicle
//#define DEFAULT_SCENE	10		// setting heavy vehicle
//#define DEFAULT_SCENE	11		// conservation of angular momentum 
//#define DEFAULT_SCENE	12		// basic voronoi fracture
//#define DEFAULT_SCENE	13		// simple voronoi fracture
//#define DEFAULT_SCENE	14		// linked voronoi fracture
//#define DEFAULT_SCENE	15		// skin peel voronoi fracture
						 
// demos forward declaration 
void ndBasicStacks(ndDemoEntityManager* const scene);
void ndBasicJoints(ndDemoEntityManager* const scene);
void ndBasicVehicle(ndDemoEntityManager* const scene);
void ndHeavyVehicle(ndDemoEntityManager* const scene);
void ndBasicTrigger(ndDemoEntityManager* const scene);
void ndBasicGpuRigidBody(ndDemoEntityManager* const scene);
void ndBasicRigidBody(ndDemoEntityManager* const scene);
void ndBasicFrictionRamp(ndDemoEntityManager* const scene);
void ndPlayerCapsuleDemo(ndDemoEntityManager* const scene);
void ndBasicFracture_4(ndDemoEntityManager* const scene);
void ndBasicParticleFluid(ndDemoEntityManager* const scene);
void ndBasicAngularMomentum(ndDemoEntityManager* const scene);
void ndBasicFracture_0(ndDemoEntityManager* const scene);
void ndBasicFracture_1(ndDemoEntityManager* const scene);
void ndBasicFracture_2(ndDemoEntityManager* const scene);
void ndBasicFracture_4(ndDemoEntityManager* const scene);
void ndBasicGpuTest0(ndDemoEntityManager* const scene);
void ndStaticMeshCollisionDemo(ndDemoEntityManager* const scene);

ndDemoEntityManager::SDKDemos ndDemoEntityManager::m_demosSelection[] = 
{
	{ "basic rigidbody", ndBasicRigidBody },
	{ "basic gpu rigidbody", ndBasicGpuRigidBody },
	{ "basic friction ramp", ndBasicFrictionRamp },
	{ "basic stack", ndBasicStacks },
	{ "basic trigger", ndBasicTrigger },
	{ "basic player", ndPlayerCapsuleDemo },
	{ "basic particle fluid", ndBasicParticleFluid },
	{ "static mesh", ndStaticMeshCollisionDemo },
	{ "basic joints", ndBasicJoints },
	{ "basic vehicle", ndBasicVehicle },
	{ "heavy vehicle", ndHeavyVehicle },
	{ "angular momentum", ndBasicAngularMomentum },
	{ "basic convex fracture", ndBasicFracture_0 },
	{ "simple convex fracture", ndBasicFracture_1 },
	{ "linked convex fracture", ndBasicFracture_2 },
	{ "simple skin peeling fracture", ndBasicFracture_4 },
};

ndDemoEntityManager::ButtonKey::ButtonKey (bool state)
	:m_state(state)
	,m_memory0(false)
	,m_memory1(false)
{
}

dInt32 ndDemoEntityManager::ButtonKey::UpdateTrigger (bool triggerValue)
{
	m_memory0 = m_memory1;
	m_memory1 = triggerValue;
	return (!m_memory0 & m_memory1) ? 1 : 0;
}

dInt32 ndDemoEntityManager::ButtonKey::UpdatePushButton (bool triggerValue)
{
	if (UpdateTrigger (triggerValue)) 
	{
		m_state = ! m_state;
	}
	return m_state ? 1 : 0;
}


// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
ndDemoEntityManager::ndDemoEntityManager ()
	:m_mainFrame(nullptr)
	,m_defaultFont(0)
	,m_sky(nullptr)
	,m_world(nullptr)
	,m_soundManager(nullptr)
	,m_cameraManager(nullptr)
	,m_renderUIContext(nullptr)
	,m_updateCameraContext(nullptr)
	,m_renderDemoGUI(nullptr)
	,m_renderHelpMenus(nullptr)
	,m_updateCamera(nullptr)
	,m_microsecunds(0)
	,m_tranparentHeap()
	,m_currentScene(DEFAULT_SCENE)
	,m_lastCurrentScene(DEFAULT_SCENE)
	,m_framesCount(0)
	,m_physicsFramesCount(0)
	,m_currentPlugin(0)
	,m_solverPasses(4)
	,m_solverSubSteps(2)
	,m_sceneType(0)
	,m_workerThreads(1)
	,m_debugDisplayMode(0)
	,m_collisionDisplayMode(0)
	,m_fps(0.0f)
	,m_timestepAcc(0.0f)
	,m_currentListenerTimestep(0.0f)
	,m_addDeleteLock()
	,m_showUI(true)
	,m_showAABB(false)
	,m_showStats(true)
	,m_hasJoytick(false)
	,m_autoSleepMode(true)
	,m_showScene(false)
	,m_showConcaveEdge(false)
	,m_hideVisualMeshes(false)
	,m_showNormalForces(false)
	,m_showCenterOfMass(false)
	,m_showBodyFrame(false)
	,m_updateMenuOptions(true)
	,m_showContactPoints(false)
	,m_showJointDebugInfo(false)
	,m_showModelsDebugInfo(false)
	,m_showCollidingFaces(false)
	,m_suspendPhysicsUpdate(false)
	,m_synchronousPhysicsUpdate(false)
	,m_showRaycastHit(false)
	,m_profilerMode(false)
	,m_solverMode(ndWorld::ndSimdSoaSolver)
	//,m_directionalLight(0.0f, 1.0f, 0.0f, 0.0f)
	,m_debugShapeCache()
	,m_replayLogFile(nullptr)
{
	// Setup window
	glfwSetErrorCallback(ErrorCallback);

	glfwInit();

	glfwWindowHint(GLFW_SAMPLES, 4);

	m_hasJoytick = glfwJoystickPresent(0) ?  true : false;

	char version[256];
	sprintf(version, "Newton Dynamics %d.%.2i sandbox demos", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION);
	m_mainFrame = glfwCreateWindow(1280, 720, version, nullptr, nullptr);
	glfwMakeContextCurrent(m_mainFrame);

	dInt32 monitorsCount;
	GLFWmonitor** monitors = glfwGetMonitors(&monitorsCount);
	if (monitorsCount > 1) 
	{
		dInt32 window_x;
		dInt32 window_y;
		dInt32 monitor_x;
		dInt32 monitor_y;

		glfwGetMonitorPos(monitors[1], &monitor_x, &monitor_y);
		glfwGetWindowPos(m_mainFrame, &window_x, &window_y);
		glfwSetWindowPos(m_mainFrame, monitor_x + window_x, monitor_y + 64);
	}

	// attach myself to the main frame
	glfwSetWindowUserPointer(m_mainFrame, this);

	// Setup ImGui binding
	ImGuiIO& io = ImGui::GetIO();
	io.UserData = this;

	// Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
	io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;                     
	io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
	io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
	io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
	io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
	io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
	io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
	io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
	io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
	io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
	io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
	io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
	io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
	io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
	io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
	io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
	io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
	io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
	io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

	// Alternatively you can set this to nullptr and call ImGui::GetDrawData() after ImGui::Render() to get the same ImDrawData pointer.
	io.RenderDrawListsFn = RenderDrawListsCallback;
	//	io.SetClipboardTextFn = ImGui_ImplGlfw_SetClipboardText;
	//	io.GetClipboardTextFn = ImGui_ImplGlfw_GetClipboardText;

#ifdef _MSC_VER 
	io.ImeWindowHandle = glfwGetWin32Window(m_mainFrame);
#else 
	dTrace (("no sure what to set this to for non windews systems\n"))
//	dAssert (0);
#endif

	glfwSwapInterval(0);
	glfwSetKeyCallback(m_mainFrame, KeyCallback);
	glfwSetCharCallback(m_mainFrame, CharCallback);
	glfwSetScrollCallback(m_mainFrame, MouseScrollCallback);
	glfwSetCursorPosCallback(m_mainFrame, CursorposCallback);
	glfwSetMouseButtonCallback(m_mainFrame, MouseButtonCallback);

	LoadFont();

	m_mousePressed[0] = false;
	m_mousePressed[1] = false;
	m_mousePressed[2] = false;

	// initialized the physics world for the new scene
	//m_showUI = false;
	//m_showAABB = true;
	//m_hideVisualMeshes = true;
	//m_showScene = true;
	//m_showConcaveEdge = true;
	//m_autoSleepMode = false;
	//m_solverMode = ndWorld::ndOpenclSolver;
	//m_solverMode = ndWorld::ndSimdSoaSolver;
	m_solverMode = ndWorld::ndStandardSolver;
	//m_sceneType = 1;
	//m_solverPasses = 4;
	//m_workerThreads = 3;
	//m_solverSubSteps = 2;
	//m_showRaycastHit = true;
	//m_showCenterOfMass = false;
	//m_showNormalForces = true;
	//m_showContactPoints = true;
	//m_showJointDebugInfo = true;
	//m_showModelsDebugInfo = true;
	//m_collisionDisplayMode = 2;	
	//m_collisionDisplayMode = 3;		// solid wire frame
	//m_synchronousPhysicsUpdate = false;

	Cleanup();
	ResetTimer();

	m_shaderCache.CreateAllEffects();

#ifdef ENABLE_REPLAY
	#ifdef REPLAY_RECORD
		m_replayLogFile = fopen("replayLog.bin", "wb");
	#else 
		m_replayLogFile = fopen("replayLog.bin", "rb");
	#endif
#endif

/*
	dFloat32 A[2][2];
	dFloat32 x[2];
	dFloat32 b[2];
	dFloat32 l[2];
	dFloat32 h[2];

	A[0][0] = 2.0f;
	A[0][1] = 1.0f;
	A[1][0] = 1.0f;
	A[1][1] = 2.0f;
	b[0] = 1.0f;
	b[1] = 1.0f;
	x[0] = 1;
	x[1] = 2;
	
	l[0] = 0.0f;
	l[1] = 0.0f;
	h[0] = 0.25f;
	h[1] = 1.0f;
	
	dMatrixTimeVector(2, &A[0][0], x, b);
	dSolveDantzigLCP(2, &A[0][0], x, b, l, h);

	dInt32 xxx = 0;
	const dInt32 xxxxxx = 450;
	dDownHeap<dInt32, unsigned> xxxxx (xxxxxx + 2);
	for (dInt32 i = 0; i < xxxxxx; i ++)
	{
		xxxxx.Push (xxx, i);
	}

	for (dInt32 i = 0; i < 10000; i ++)
	{
		dInt32 index = dRand() % xxxxxx;
		dInt32 key = xxxxx.Value(index);
		xxxxx.Remove (index);
		xxxxx.Push (xxx, key);
	}
*/
}

ndDemoEntityManager::~ndDemoEntityManager ()
{
	if (m_replayLogFile)
	{
		fclose(m_replayLogFile);
	}

	Cleanup ();

	// destroy the empty world
	if (m_world) 
	{
		delete m_world;
	}

	if (m_cameraManager) 
	{
		delete m_cameraManager;
	}

	// Cleanup
	GLuint font_texture (m_defaultFont);
	glDeleteTextures(1, &font_texture);
	ImGui::GetIO().Fonts->TexID = 0;

	ImGui::Shutdown();
	glfwTerminate();
}

ndDemoCamera* ndDemoEntityManager::GetCamera() const
{
	return m_cameraManager->GetCamera();
}

bool ndDemoEntityManager::GetKeyState(dInt32 key) const
{
	const ImGuiIO& io = ImGui::GetIO();
	return io.KeysDown[key];
}

bool ndDemoEntityManager::IsShiftKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
	return state;
}

bool ndDemoEntityManager::IsControlKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	return state;
}

bool ndDemoEntityManager::GetCaptured() const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.WantCaptureMouse;
}

bool ndDemoEntityManager::GetMouseKeyState (dInt32 button) const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.MouseDown[button];
}

void ndDemoEntityManager::Set2DDisplayRenderFunction (RenderGuiHelpCallback helpCallback, RenderGuiHelpCallback UIcallback, void* const context)
{
	m_renderUIContext = context;
	m_renderDemoGUI = UIcallback;
	m_renderHelpMenus = helpCallback;
}

void ndDemoEntityManager::SetUpdateCameraFunction(UpdateCameraCallback callback, void* const context)
{
	m_updateCamera = callback;
	m_updateCameraContext = context;
}

dInt32 ndDemoEntityManager::GetJoystickAxis (dFixSizeArray<dFloat32, 8>& axisValues)
{
	dInt32 axisCount = 0;
	for (dInt32 i = 0; i < axisValues.GetCapacity(); i++)
	{
		axisValues[i] = dFloat32(0.0f);
	}
	// for xbox controllers.
	axisValues[4] = dFloat32(-1.0f);
	axisValues[5] = dFloat32(-1.0f);

	if (!m_hasJoytick)
	{
		m_hasJoytick = glfwJoystickPresent(0) ? true : false;
	}

	if (m_hasJoytick) 
	{
		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
		axisCount = dMin (axisCount, axisValues.GetCapacity());
		for (dInt32 i = 0; i < axisCount; i ++) 
		{
			axisValues[i] = axis[i];
			//if (axis[i] && axis[i] > -1.0f) dTrace(("%d %f\n", i, axis[i]));
		}
	}

#ifdef ENABLE_REPLAY
	#ifdef REPLAY_RECORD
		fwrite(&axisCount, sizeof(axisCount), 1, m_replayLogFile);
		fwrite(&axisValues[0], sizeof(dFloat32) * axisValues.GetCapacity(), 1, m_replayLogFile);
		fflush(m_replayLogFile);
	#else 
		fread(&axisCount, sizeof(axisCount), 1, m_replayLogFile);
		fread(&axisValues[0], sizeof(dFloat32) * axisValues.GetCapacity(), 1, m_replayLogFile);
	#endif
#endif
	return axisCount;
}

dInt32 ndDemoEntityManager::GetJoystickButtons(dFixSizeArray<char, 32>& axisbuttons)
{
	dInt32 buttonsCount = 0;
	memset(&axisbuttons[0], 0, axisbuttons.GetCapacity());

	if (!m_hasJoytick)
	{
		m_hasJoytick = glfwJoystickPresent(0) ? true : false;
	}

	if (m_hasJoytick) 
	{
		const unsigned char* const buttons = glfwGetJoystickButtons(0, &buttonsCount);
		buttonsCount = dMin (buttonsCount, axisbuttons.GetCapacity());
		for (dInt32 i = 0; i < buttonsCount; i ++) 
		{
			axisbuttons[i] = buttons[i];
			//if (buttons[i]) dTrace(("%d %d\n", i, buttons[i]));
		}
	}

#ifdef ENABLE_REPLAY
	#ifdef REPLAY_RECORD
		fwrite(&buttonsCount, sizeof(buttonsCount), 1, m_replayLogFile);
		fwrite(&axisbuttons[0], axisbuttons.GetCapacity(), 1, m_replayLogFile);
		fflush(m_replayLogFile);
	#else 
		fread(&buttonsCount, sizeof(buttonsCount), 1, m_replayLogFile);
		fread(&axisbuttons[0], axisbuttons.GetCapacity(), 1, m_replayLogFile);
	#endif
#endif

	return buttonsCount;
}

void ndDemoEntityManager::ResetTimer()
{
	dResetTimer();
	m_microsecunds = dGetTimeInMicrosenconds ();
}

void ndDemoEntityManager::AddEntity(ndDemoEntity* const ent)
{
	dScopeSpinLock lock(m_addDeleteLock);
	dAssert(!ent->m_rootNode);
	ent->m_rootNode = Append(ent);
}

void ndDemoEntityManager::RemoveEntity (ndDemoEntity* const ent)
{
	dScopeSpinLock lock(m_addDeleteLock);
	dAssert(ent->m_rootNode);
	Remove(ent->m_rootNode);
}

void ndDemoEntityManager::Cleanup ()
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) 
	{
		m_world->Sync();
	}

	while (m_debugShapeCache.GetRoot())
	{
		ndDebugMeshCache::dNode* const root = m_debugShapeCache.GetRoot();
		ndDebuMesh& debugMesh = root->GetInfo();
		debugMesh.m_flatShaded->Release();
		debugMesh.m_wireFrameShareEdge->Release();
		if (debugMesh.m_wireFrameOpenEdge)
		{
			debugMesh.m_wireFrameOpenEdge->Release();
		}
		m_debugShapeCache.Remove(root);
	}

	if (m_cameraManager) 
	{
		delete m_cameraManager;
	}

	m_sky = nullptr;
	m_updateCamera = nullptr;

	// destroy the Newton world
	if (m_world) 
	{
		const ndBodyList& bodyList = m_world->GetBodyList();
		for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
		{
			ndBodyKinematic* const body = bodyNode->GetInfo();
			ndDemoEntityNotify* const callback = (ndDemoEntityNotify*)body->GetNotifyCallback();
			if (callback)
			{
				callback->m_entity = nullptr;
			}
		}

		// get serialization call back before destroying the world
		delete m_world;
	}

	// destroy all remaining visual objects
	while (GetFirst())
	{
		ndDemoEntity* const ent = GetFirst()->GetInfo();
		RemoveEntity(ent);
		delete ent;
	}

	// create the newton world
	m_world = new ndPhysicsWorld(this);

	// add a sound manager
	m_soundManager = new ndSoundManager();
	m_world->AddModel(m_soundManager);
	
	// add the camera manager
	m_cameraManager = new ndDemoCameraManager(this);
	
	ApplyMenuOptions();

	// we start without 2d render
	m_renderDemoGUI = nullptr;
	m_renderHelpMenus = nullptr;
	m_renderUIContext = nullptr;
}

void ndDemoEntityManager::LoadFont()
{
	// Build texture atlas
	ImGuiIO& io = ImGui::GetIO();

    // Load Fonts
    // (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
    //io.Fonts->AddFontDefault();

	float pixedSize = 18;
	char pathName[2048];
	const char* const name = "Cousine-Regular.ttf";
	//char* const name = "calibri.ttf";
	//char* const name = "courbd.ttf";

	dGetWorkingFileName (name, pathName);
    io.Fonts->AddFontFromFileTTF(pathName, pixedSize);
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());

	// Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) 
	// because it is more likely to be compatible with user's existing shaders. 
	// If your ImTextureId represent a higher-level concept than just a GL texture id, 
	// consider calling GetTexDataAsAlpha8() instead to save on GPU memory.
	unsigned char* pixels;
	dInt32 width, height;
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   

	// Upload texture to graphics system
	GLint last_texture;
	GLuint font_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGenTextures(1, &font_texture);
	glBindTexture(GL_TEXTURE_2D, font_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	m_defaultFont = dInt32 (font_texture);
	io.Fonts->TexID = (void *)(intptr_t)m_defaultFont;

	// Restore state
	glBindTexture(GL_TEXTURE_2D, last_texture);
}

void ndDemoEntityManager::ApplyMenuOptions()
{
	m_world->Sync();
	// clean up all caches the engine have saved
	////NewtonInvalidateCache(m_world);

	m_world->SetSubSteps(m_solverSubSteps);
	m_world->SetSolverIterations(m_solverPasses);
	m_world->SetThreadCount(m_workerThreads);

	bool state = m_autoSleepMode ? true : false;
	const ndBodyList& bodyList = m_world->GetBodyList();
	for (ndBodyList::dNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo();
		body->SetAutoSleep(state);
	}

	m_world->SelectSolver(m_solverMode);
	m_solverMode = m_world->GetSelectedSolver();
}

void ndDemoEntityManager::ShowMainMenuBar()
{
	dInt32 mainMenu = 0;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File")) 
		{
			m_suspendPhysicsUpdate = true;

			if (ImGui::MenuItem("Preferences", "")) 
			{
				dAssert (0);
			}
			ImGui::Separator();

			if (ImGui::MenuItem("New", "")) 
			{
				mainMenu = 1;
			}
			ImGui::Separator();

			if (ImGui::MenuItem("Open", "")) 
			{
				mainMenu = 2;
			}
			if (ImGui::MenuItem("Save", "")) 
			{
				mainMenu = 3;
			}

			ImGui::Separator();
			if (ImGui::MenuItem("import ply file", "")) 
			{
				mainMenu = 4;
			}

			ImGui::Separator();
			if (ImGui::MenuItem("Exit", "")) 
			{
				glfwSetWindowShouldClose (m_mainFrame, 1);
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Demos")) 
		{
			m_suspendPhysicsUpdate = true;
			dInt32 demosCount = dInt32 (sizeof (m_demosSelection) / sizeof m_demosSelection[0]);
			for (dInt32 i = 0; i < demosCount; i ++) 
			{
				if (ImGui::MenuItem(m_demosSelection[i].m_name, "")) 
				{
					m_currentScene = i;
				}
			}

			ImGui::EndMenu();
		}

		bool optionsOn = ImGui::BeginMenu("Options");
		if (optionsOn) 
		{
			m_updateMenuOptions = true;
			m_suspendPhysicsUpdate = true;

			ImGui::Checkbox("auto sleep mode", &m_autoSleepMode);
			ImGui::Checkbox("show UI", &m_showUI);
			ImGui::Checkbox("show stats", &m_showStats);
			ImGui::Checkbox("synchronous physics update", &m_synchronousPhysicsUpdate);
			ImGui::Separator();

			ImGui::Text("solvers");
			dInt32 solverMode(m_solverMode);
			ImGui::RadioButton("sse soa", &solverMode, ndWorld::ndSimdSoaSolver);
			ImGui::RadioButton("avx2", &solverMode, ndWorld::ndSimdAvx2Solver);
			ImGui::RadioButton("opencl", &solverMode, ndWorld::ndOpenclSolver);
			ImGui::RadioButton("default", &solverMode, ndWorld::ndStandardSolver);
			m_solverMode = ndWorld::ndSolverModes(solverMode);
			ImGui::Separator();

			//dInt32 index = 0;
			//ImGui::RadioButton("default solver", &m_currentPlugin, index);
			//char ids[32][32];
			//for (void* plugin = NewtonGetFirstPlugin(m_world); plugin; plugin = NewtonGetNextPlugin(m_world, plugin)) {
			//	index++;
			//	const char* const id = NewtonGetPluginString(m_world, plugin);
			//	sprintf (&ids[index][0], "%s", id);
			//	ImGui::RadioButton(&ids[index][0], &m_currentPlugin, index);
			//}
			//ImGui::Separator();
			ImGui::Text("solver sub steps");
			ImGui::SliderInt("##solv", &m_solverSubSteps, 2, 8);
			ImGui::Text("iterative solver passes");
			ImGui::SliderInt("##intera", &m_solverPasses, 4, 64);
			ImGui::Text("worker threads");
			ImGui::SliderInt("##worker", &m_workerThreads, 1, D_MAX_THREADS_COUNT);
			ImGui::Separator();

			ImGui::RadioButton("default broad phase", &m_sceneType, 0);
			ImGui::RadioButton("persistence broad phase", &m_sceneType, 1);
			ImGui::Separator();

			ImGui::RadioButton("hide collision Mesh", &m_collisionDisplayMode, 0);
			ImGui::RadioButton("show solid collision", &m_collisionDisplayMode, 1);
			ImGui::RadioButton("show wire frame collision", &m_collisionDisplayMode, 2);
			ImGui::RadioButton("show hidden wire frame collision", &m_collisionDisplayMode, 3);
			ImGui::Separator();

			ImGui::Checkbox("show aabb", &m_showAABB);
			ImGui::Checkbox("show broad phase", &m_showScene);
			ImGui::Checkbox("show concave edge", &m_showConcaveEdge);
			ImGui::Checkbox("hide visual meshes", &m_hideVisualMeshes);
			ImGui::Checkbox("show contact points", &m_showContactPoints);
			ImGui::Checkbox("show ray cast hit point", &m_showRaycastHit);
			ImGui::Checkbox("show normal forces", &m_showNormalForces);
			ImGui::Checkbox("show center of mass", &m_showCenterOfMass);
			ImGui::Checkbox("show body frame", &m_showBodyFrame);
			ImGui::Checkbox("show joints debug info", &m_showJointDebugInfo);
			ImGui::Checkbox("show models debug info", &m_showModelsDebugInfo);
			ImGui::Checkbox("show colliding faces", &m_showCollidingFaces);

			ImGui::EndMenu();

			//SetDebugDisplayMode(m_showCollidingFaces ? 1 : 0);
		}

		if (ImGui::BeginMenu("Help")) 
		{
			m_suspendPhysicsUpdate = true;
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();

		if (!optionsOn && m_updateMenuOptions) 
		{
			m_updateMenuOptions = false;
			ApplyMenuOptions();
		}
	}

	switch (mainMenu)
	{
		case 1:
		{
			// menu new 
			Cleanup();
			ApplyMenuOptions();
			ResetTimer();
			m_currentScene = -1;
			break;
		}

		case 2:
		{
			// open Scene
			m_currentScene = -1;
			char fileName[1024];
			Cleanup();
			if (dGetOpenFileNameNgd(fileName, 1024)) 
			{
				ApplyMenuOptions();
				CreateSkyBox();
				m_world->Load(fileName);
				ResetTimer();
			}
			break;
		}

		case 3:
		{
			//m_currentScene = -1;
			char fileName[1024];
			if (dGetSaveFileNameNgd(fileName, 1024)) 
			{
				_strlwr(fileName);
				char* name = strrchr(fileName, '/');
				if (!name)
				{
					name = strrchr(fileName, '\\');
				}
				if (!name)
				{
					name = fileName;
				}

				char* ext = strrchr(name, '.');
				if (!ext)
				{
					strcat(fileName, ".ngd");
				} 
				else if (strcmp(ext, ".ngd"))
				{
					strcpy(ext, ".ngd");
				}
				m_world->Save(fileName);
			}
			break;
		}

		case 4:
		{
			// open Scene
			m_currentScene = -1;
			char fileName[1024];
			Cleanup();
			if (dGetOpenFileNamePLY(fileName, 1024)) 
			{
				ApplyMenuOptions();
				ImportPLYfile(fileName);
				ResetTimer();
			}
			break;
		}

		default:
		{
			// load a demo 
			if (m_currentScene != -1) 
			{
				LoadDemo(m_currentScene);
				m_lastCurrentScene = m_currentScene;
				m_currentScene = -1;
			}
		}
	}
}

void ndDemoEntityManager::LoadDemo(dInt32 menu)
{
	char newTitle[256];
	Cleanup();

	// make the sky box 
	CreateSkyBox();
	m_demosSelection[menu].m_launchDemoCallback(this);

	sprintf(newTitle, "Newton Dynamics %d.%.2i demo: %s", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION, m_demosSelection[menu].m_name);
	glfwSetWindowTitle(m_mainFrame, newTitle);
	ApplyMenuOptions();
	ResetTimer();
}

void ndDemoEntityManager::ErrorCallback(dInt32 error, const char* description)
{
	dTrace (("Error %d: %s\n", error, description));
	fprintf(stderr, "Error %d: %s\n", error, description);
	dAssert (0);
}

void ndDemoEntityManager::MouseButtonCallback(GLFWwindow*, dInt32 button, dInt32 action, dInt32)
{
	if (button >= 0 && button < 3) 
	{
		ImGuiIO& io = ImGui::GetIO();
		if (action == GLFW_PRESS) 
		{
			io.MouseDown[button] = true;    
		} 
		else if (action == GLFW_RELEASE) 
		{
			io.MouseDown[button] = false;    
		}
	}
}

void ndDemoEntityManager::MouseScrollCallback(GLFWwindow* const, double, double y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += float (y);
}

void ndDemoEntityManager::CursorposCallback  (GLFWwindow* , double x, double y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);
}

bool ndDemoEntityManager::GetMouseSpeed(dFloat32& speedX, dFloat32& speedY) const
{
	ImVec2 speed(ImGui::GetMouseDragDelta(0, 0.0f));
	speedX = speed.x;
	speedY = speed.y;
	return true;
}

bool ndDemoEntityManager::GetMousePosition (dFloat32& posX, dFloat32& posY) const
{
	ImVec2 posit(ImGui::GetMousePos());
	posX = posit.x;
	posY = posit.y;
	return true;
}

void ndDemoEntityManager::CharCallback(GLFWwindow*, dUnsigned32 ch)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter((unsigned short)ch);
}


void ndDemoEntityManager::KeyCallback(GLFWwindow* const window, dInt32 key, dInt32, dInt32 action, dInt32 mods)
{
	ImGuiIO& io = ImGui::GetIO();
	if (action == GLFW_PRESS)
		io.KeysDown[key] = true;
	if (action == GLFW_RELEASE)
		io.KeysDown[key] = false;

	(void)mods; // Modifiers are not reliable across systems
	io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	io.KeyShift = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
	io.KeyAlt = io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
	io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
	
	static dInt32 prevKey;
	ndDemoEntityManager* const manager = (ndDemoEntityManager*)glfwGetWindowUserPointer(window);
	if ((key == GLFW_KEY_F10) && (key != prevKey)) 
	{
		manager->ToggleProfiler();
	}

	if (key == GLFW_KEY_ESCAPE) 
	{
		glfwSetWindowShouldClose (window, 1);
	}

	if (key == GLFW_KEY_F1) 
	{
		manager->LoadDemo(manager->m_lastCurrentScene);
	}

	prevKey = io.KeysDown[key] ? key : 0;
}

void ndDemoEntityManager::ToggleProfiler()
{
	#ifdef D_PROFILER
		dAssert(m_world);
		dTrace(("profiler Enable\n"));
		m_world->Sync();
		m_profilerMode = !m_profilerMode;
		dProfilerEnableProling(m_profilerMode);
	#endif
}

void ndDemoEntityManager::BeginFrame()
{
	glfwPollEvents();
	ImGuiIO& io = ImGui::GetIO();

	// Setup display size (every frame to accommodate for window resizing)
	dInt32 w, h;
	dInt32 display_w, display_h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	glfwGetFramebufferSize(m_mainFrame, &display_w, &display_h);
	io.DisplaySize = ImVec2((float)w, (float)h);
	io.DisplayFramebufferScale = ImVec2(w > 0 ? ((float)display_w / w) : 0, h > 0 ? ((float)display_h / h) : 0);

	// Start the frame
	ImGui::NewFrame();
}

void ndDemoEntityManager::RenderStats()
{
	if (m_showStats) 
	{
		char text[1024];
		
		if (ImGui::Begin("statistics", &m_showStats)) 
		{
			sprintf (text, "fps:            %6.3f", m_fps);
			ImGui::Text(text, "");

			sprintf(text, "physics time:  %6.3f ms", m_world->GetAverageUpdateTime() * 1.0e3f);
			ImGui::Text(text, "");

			sprintf(text, "update mode:    %s", m_synchronousPhysicsUpdate ? "synchronous" : "asynchronous");
			ImGui::Text(text, "");

			if (m_currentPlugin) 
			{
				dAssert(0);
				//dInt32 index = 1;
				//for (void* plugin = NewtonGetFirstPlugin(m_world); plugin; plugin = NewtonGetNextPlugin(m_world, plugin)) {
				//	if (index == m_currentPlugin) {
				//		sprintf(text, "plugin:        %s", NewtonGetPluginString(m_world, plugin));
				//		ImGui::Text(text, "");
				//	}
				//	index++;
				//}
			}

			sprintf(text, "bodies:         %d", m_world->GetBodyList().GetCount());
			ImGui::Text(text, "");

			sprintf(text, "joints:         %d", m_world->GetJointList().GetCount());
			ImGui::Text(text, "");

			sprintf(text, "contact joints: %d", m_world->GetContactList().GetCount());
			ImGui::Text(text, "");

			sprintf(text, "memory used:   %6.3f mbytes", dFloat32(dFloat64(dMemory::GetMemoryUsed()) / (1024 * 1024)));
			ImGui::Text(text, "");


			sprintf(text, "threads:        %d", m_world->GetThreadCount());
			ImGui::Text(text, "");

			sprintf(text, "iterations:     %d", m_world->GetSolverIterations());
			ImGui::Text(text, "");

			sprintf(text, "Substeps:       %d", m_world->GetSubSteps());
			ImGui::Text(text, "");

			sprintf(text, "solver:         %s", m_world->GetSolverString());
			ImGui::Text(text, "");

			m_suspendPhysicsUpdate = m_suspendPhysicsUpdate || (ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0));  
			ImGui::End();
		}
	}

	if (m_showUI && m_renderHelpMenus) 
	{
		if (ImGui::Begin("User Interface", &m_showUI))
		{
			m_renderHelpMenus (this, m_renderUIContext);
			//m_suspendPhysicsUpdate = m_suspendPhysicsUpdate || (ImGui::IsMouseHoveringWindow() && ImGui::IsMouseDown(0));  
			ImGui::End();
		}
	}

	ShowMainMenuBar();
}

void ndDemoEntityManager::CalculateFPS(dFloat32 timestep)
{
	m_framesCount ++;
	m_timestepAcc += timestep;

	// this probably happing on loading of and a pause, just rest counters
	if ((m_timestepAcc <= 0.0f) || (m_timestepAcc > 4.0f))
	{
		m_timestepAcc = 0;
		m_framesCount = 0;
	}

	//update fps every quarter of a second
	const dFloat32 movingAverage = 0.5f;
	if (m_timestepAcc >= movingAverage)
	{
		m_fps = dFloat32 (m_framesCount) / m_timestepAcc;
		m_timestepAcc -= movingAverage;
		m_framesCount = 0;
	}
}

void ndDemoEntityManager::CreateSkyBox()
{
	if (!m_sky)
	{
		m_sky = new ndSkyBox(m_shaderCache.m_skyBox);
		
		dScopeSpinLock lock(m_addDeleteLock);
		dAssert(!m_sky->m_rootNode);
		m_sky->m_rootNode = Addtop(m_sky);
	}
}

void ndDemoEntityManager::PushTransparentMesh (const ndDemoMeshInterface* const mesh, const dMatrix& modelMatrix)
{
	dVector dist (m_cameraManager->GetCamera()->GetViewMatrix().TransformVector(modelMatrix.m_posit));
	TransparentMesh entry (modelMatrix, (ndDemoMesh*) mesh);
	m_tranparentHeap.Push (entry, dist.m_z);
}


//void ndDemoEntityManager::ImportPLYfile (const char* const fileName)
void ndDemoEntityManager::ImportPLYfile(const char* const)
{
	dAssert(0);
	//m_collisionDisplayMode = 2;
	//CreatePLYMesh (this, fileName, true);
}

dInt32 ndDemoEntityManager::Print (const dVector&, const char *fmt, ... ) const
{
	va_list argptr;
	char string[1024];

	va_start (argptr, fmt);
	vsprintf (string, fmt, argptr);
	va_end( argptr );
	ImGui::Text(string, "");
	return 0;
}

void ndDemoEntityManager::SetCameraMatrix (const dQuaternion& rotation, const dVector& position)
{
	m_cameraManager->SetCameraMatrix(rotation, position);
}

void ndDemoEntityManager::UpdatePhysics(dFloat32 timestep)
{
	// update the physics
	if (m_world && !m_suspendPhysicsUpdate) 
	{
		m_world->AdvanceTime(timestep);
	}
}

dFloat32 ndDemoEntityManager::CalculateInteplationParam () const
{
	dUnsigned64 timeStep = dGetTimeInMicrosenconds () - m_microsecunds;		
	dFloat32 param = (dFloat32 (timeStep) * MAX_PHYSICS_FPS) / 1.0e6f;
	dAssert (param >= 0.0f);
	if (param > 1.0f) {
		param = 1.0f;
	}
	return param;
}


// This is the main rendering function that you have to implement and provide to ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by (0.5f,0.5f) or (0.375f,0.375f)
void ndDemoEntityManager::RenderDrawListsCallback(ImDrawData* const draw_data)
{
	// Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
	ImGuiIO& io = ImGui::GetIO();

	dInt32 fb_width = (dInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	dInt32 fb_height = (dInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (fb_width == 0 || fb_height == 0)
		return;

	ndDemoEntityManager* const window = (ndDemoEntityManager*)io.UserData;

	ImVec4 clearColor = ImColor(114, 144, 154);
	glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);

	window->RenderScene();

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_SCISSOR_TEST);
	glEnable(GL_TEXTURE_2D);
	//glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

	// Setup viewport, orthographic projection matrix
	glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	if (window->m_renderDemoGUI) 
	{
		window->m_renderDemoGUI(window, window->m_renderUIContext);
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	// Render command lists
	draw_data->ScaleClipRects(io.DisplayFramebufferScale);
	#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
	for (dInt32 n = 0; n < draw_data->CmdListsCount; n++)
	{
		const ImDrawList* cmd_list = draw_data->CmdLists[n];
		const ImDrawVert* vtx_buffer = cmd_list->VtxBuffer.Data;
		const ImDrawIdx* idx_buffer = cmd_list->IdxBuffer.Data;
		glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, pos)));
		glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, uv)));
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, col)));

		for (dInt32 cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
		{
			const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
			if (pcmd->UserCallback)
			{
				pcmd->UserCallback(cmd_list, pcmd);
			}
			else
			{
				glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
				glScissor((dInt32)pcmd->ClipRect.x, (dInt32)(fb_height - pcmd->ClipRect.w), (dInt32)(pcmd->ClipRect.z - pcmd->ClipRect.x), (dInt32)(pcmd->ClipRect.w - pcmd->ClipRect.y));
				glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer);
			}
			idx_buffer += pcmd->ElemCount;
		}
	}
	#undef OFFSETOF

	// Restore modified state
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

void ndDemoEntityManager::DrawDebugShapes()
{
	const dVector awakeColor(1.0f, 1.0f, 1.0f, 1.0f);
	const dVector sleepColor(0.42f, 0.73f, 0.98f, 1.0f);

	const ndBodyList& bodyList = m_world->GetBodyList();

	if (m_collisionDisplayMode == 3)
	{
		// do a z buffer pre pass for hidden line 
		glColorMask(0, 0, 0, 0);
		for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
		{
			ndBodyKinematic* const body = bodyNode->GetInfo();
			if (!body->GetAsBodyTriggerVolume())
			{
				const ndShapeInstance& shapeInstance = body->GetCollisionShape();
				ndDebugMeshCache::dNode* const shapeNode = m_debugShapeCache.Find(shapeInstance.GetShape());
				if (shapeNode)
				{
					dMatrix matrix(shapeInstance.GetScaledTransform(body->GetMatrix()));
					shapeNode->GetInfo().m_flatShaded->Render(this, matrix);
				}
			}
		}
		glColorMask(1, 1, 1, 1);
	}

	for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();
		const ndShapeInstance& shapeInstance = body->GetCollisionShape();
		const ndShape* const key = shapeInstance.GetShape();
		if (!((ndShape*)key)->GetAsShapeNull())
		{
			ndDebugMeshCache::dNode* shapeNode = m_debugShapeCache.Find(key);
			if (!shapeNode)
			{
				ndShapeInstance shape(body->GetCollisionShape());
				shape.SetScale(dVector(1.0f));
				shape.SetLocalMatrix(dGetIdentityMatrix());

				ndDebuMesh debugMesh;
				debugMesh.m_flatShaded = new ndFlatShadedDebugMesh(m_shaderCache, &shape);
				debugMesh.m_wireFrameShareEdge = new ndWireFrameDebugMesh(m_shaderCache, &shape);
				if (shape.GetShape()->GetAsShapeStaticBVH())
				{
					debugMesh.m_wireFrameOpenEdge = new ndWireFrameDebugMesh(m_shaderCache, &shape, ndShapeDebugCallback::ndEdgeType::m_open);
					debugMesh.m_wireFrameOpenEdge->SetColor(dVector(1.0f, 0.0f, 1.0f, 1.0f));
				}
				shapeNode = m_debugShapeCache.Insert(debugMesh, key);
			}

			dMatrix matrix(shapeInstance.GetScaledTransform(body->GetMatrix()));
			dInt32 sleepState = body->GetSleepState();
			dVector color((sleepState == 1) ? sleepColor : awakeColor);

			if (m_collisionDisplayMode >= 2)
			{
				ndWireFrameDebugMesh* const sharedEdgeMesh = shapeNode->GetInfo().m_wireFrameShareEdge;
				sharedEdgeMesh->SetColor(color);
				sharedEdgeMesh->Render(this, matrix);

				if (shapeNode->GetInfo().m_wireFrameOpenEdge)
				{
					ndWireFrameDebugMesh* const openEdgeMesh = shapeNode->GetInfo().m_wireFrameOpenEdge;
					dVector color1(m_showConcaveEdge ? dVector(1.0f, 0.0f, 1.0f, 1.0f) : color);
					openEdgeMesh->SetColor(color1);
					openEdgeMesh->Render(this, matrix);
				}
			}
			else
			{
				ndFlatShadedDebugMesh* const mesh = shapeNode->GetInfo().m_flatShaded;
				mesh->SetColor(color);
				mesh->Render(this, matrix);
			}
		}
	}

	RenderParticles(this);
}

void ndDemoEntityManager::RenderScene()
{
	dFloat32 timestep = dGetElapsedSeconds();	
	CalculateFPS(timestep);
	UpdatePhysics(timestep);

	D_TRACKTIME();
	// Get the interpolated location of each body in the scene
	m_cameraManager->InterpolateMatrices (this, CalculateInteplationParam());

	ImGuiIO& io = ImGui::GetIO();
	dInt32 display_w = (dInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	dInt32 display_h = (dInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	glViewport(0, 0, display_w, display_h);
	glScissor(0, 0, display_w, display_h);
	glEnable(GL_SCISSOR_TEST);	

	// Rendering
	// Our shading model--Goraud (smooth). 
	glShadeModel (GL_SMOOTH);

	// Culling. 
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);
	glEnable (GL_CULL_FACE);

	//	glEnable(GL_DITHER);
	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);

	// set default lightning
	//dFloat32 cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	//glMaterialParam(GL_FRONT, GL_SPECULAR, cubeColor);
	//glMaterialParam(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	//glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	//
	//// set just one directional light
	//GLfloat lightDiffuse0[] = { 0.8f, 0.8f, 0.8f, 0.0f };
	//GLfloat lightAmbient0[] = { 0.2f, 0.2f, 0.2f, 0.0f };
	//GLfloat lightSpecular0[] = { 1.0f, 1.0f, 1.0f, 0.0f };
	//GLfloat lightPosition0[] = { 0.0f, 200.0f, 150.0f, 0.0f };
	//
	//glMaterialf(GL_FRONT, GL_SHININESS, 60.0f);
	//glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
	//glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular0);
	//glEnable(GL_LIGHT0);

	// one light from the Camera eye point
	dVector camPosition (m_cameraManager->GetCamera()->m_matrix.m_posit);
	GLfloat lightDiffuse1[] = { 0.5f, 0.5f, 0.5f, 0.0f };
	GLfloat lightAmbient1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	GLfloat lightSpecular1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	GLfloat lightPosition1[] = {0.0f, 0.0f, 0.0f, 1.0f};

	glMaterialf(GL_FRONT, GL_SHININESS, 60.0f);
	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightSpecular1);
	glEnable(GL_LIGHT1);

	// Setup camera matrix
	m_cameraManager->GetCamera()->SetViewMatrix(display_w, display_h);

	// render all entities
	const dMatrix globalMatrix (dGetIdentityMatrix());
	if (m_hideVisualMeshes) 
	{
		if (m_sky) 
		{
			m_sky->Render(timestep, this, globalMatrix);
		}
	} 
	else 
	{
		for (dNode* node = dList<ndDemoEntity*>::GetFirst(); node; node = node->GetNext()) 
		{
			ndDemoEntity* const entity = node->GetInfo();
			entity->Render(timestep, this, globalMatrix);
		}

		while (m_tranparentHeap.GetCount()) 
		{
			const TransparentMesh& transparentMesh = m_tranparentHeap[0];
			transparentMesh.m_mesh->RenderTransparency(this, transparentMesh.m_matrix);
			m_tranparentHeap.Pop();
		}
	}

	if (m_showContactPoints)
	{
		m_world->Sync();
		RenderContactPoints(this);
	}
	
	if (m_showAABB) 
	{
		m_world->Sync();
		RenderBodiesAABB(this);
	}

	if (m_showScene)
	{
		m_world->Sync();
		RenderWorldScene(this);
	}

	//if (m_showRaycastHit) {
	//	RenderRayCastHit(m_world);
	//}

	if (m_showJointDebugInfo) 
	{
		RenderJointsDebugInfo(this);
	}

	if (m_showModelsDebugInfo)
	{
		RenderModelsDebugInfo(this);
	}

	if (m_showBodyFrame)
	{
		RenderBodyFrame(this);
	}

	if (m_showCenterOfMass) 
	{
		RenderCenterOfMass(this);
	}

	//if (m_showNormalForces) 
	//{
	//	RenderNormalForces (m_world);
	//}

	if (m_collisionDisplayMode)
	{
		m_world->Sync();
		DrawDebugShapes();
	}
}

void ndDemoEntityManager::Run()
{
    // Main loop
    while (!glfwWindowShouldClose(m_mainFrame))
    {
		m_suspendPhysicsUpdate = false;
		BeginFrame();

		D_TRACKTIME();

		RenderStats();
		ImGui::Render();
		glfwSwapBuffers(m_mainFrame);
    }
}
